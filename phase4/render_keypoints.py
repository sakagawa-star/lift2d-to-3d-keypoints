"""ピンホール3DGSレンダリングへの人体キーポイント重ね描き（オクルージョン考慮、PNG出力）

指定した1台のカメラ視点で 3DGS点群（PLY）を gsplat の古典ピンホール経路で
レンダリングし、その背景画像に人体3Dキーポイント（C3D, Halpe26）の先頭フレーム1枚を
投影して点＋ボーンで重ね描きする。その際、3DGSの深度マップと各キーポイントのカメラ深度を
比較して前後関係（オクルージョン）を反映し、カメラ手前の3DGS（障害物）に隠れる
キーポイント・ボーンを隠蔽する。キャリブレーション（R, t, K）の妥当性を目視検証する用途。

入力（Blenderを経由せず元データを直読み）:
  - PLY : LCC Studio製3DGS（ネイティブ座標＝キャリブレーション座標、Z-up・メートル）
  - TOML: Pose2Simキャリブ（OpenCV規約 world-to-camera、複数カメラ）
  - C3D : 人体キーポイント（Halpe26、mm。先頭フレームのみ使用）

実行は phase4 venv で行う（torch/gsplat が必要）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \\
      data/Blender/point_cloud.ply data/Blender/Config_scene.toml \\
      data/Blender/keypoints.c3d \\
      --camera cam41520554 --near-plane 0.5 --output /tmp/keypoints.png
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import tomli


# === Halpe26 定数（feat-013 design.md より復元） ===

# Halpe26 キーポイント名（C3D label と一致）
HALPE26_NAMES = [
    "Hip", "RHip", "RKnee", "RAnkle", "RBigToe", "RSmallToe", "RHeel",
    "LHip", "LKnee", "LAnkle", "LBigToe", "LSmallToe", "LHeel",
    "Neck", "Head", "Nose", "LEye", "REye", "LEar", "REar",
    "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist",
]

# ボーン端点名 → HALPE26_NAMES内インデックスの辞書
NAME_TO_IDX = {name: i for i, name in enumerate(HALPE26_NAMES)}

# Halpe26 スケルトン（ボーン）定義: (始点名, 終点名, 部位)
# 部位: "R"=右半身, "L"=左半身, "C"=体幹/顔
HALPE26_SKELETON = [
    ("Head", "Nose", "C"), ("Nose", "Neck", "C"), ("Neck", "Hip", "C"),
    ("Nose", "REye", "R"), ("REye", "REar", "R"),
    ("Nose", "LEye", "L"), ("LEye", "LEar", "L"),
    ("Neck", "RShoulder", "R"), ("RShoulder", "RElbow", "R"), ("RElbow", "RWrist", "R"),
    ("Neck", "LShoulder", "L"), ("LShoulder", "LElbow", "L"), ("LElbow", "LWrist", "L"),
    ("Hip", "RHip", "R"), ("RHip", "RKnee", "R"), ("RKnee", "RAnkle", "R"),
    ("Hip", "LHip", "L"), ("LHip", "LKnee", "L"), ("LKnee", "LAnkle", "L"),
    ("RAnkle", "RHeel", "R"), ("RAnkle", "RBigToe", "R"), ("RBigToe", "RSmallToe", "R"),
    ("LAnkle", "LHeel", "L"), ("LAnkle", "LBigToe", "L"), ("LBigToe", "LSmallToe", "L"),
]

# 色 (BGR, OpenCV)。左右は被験者本人の解剖学的左右（マーカー名 R*/L* に従う。カメラ視点ではない）
COLOR_RIGHT = (0, 0, 255)     # 赤（被験者の右半身: RShoulder 等）
COLOR_LEFT = (255, 0, 0)      # 青（被験者の左半身: LShoulder 等）
COLOR_CENTER = (0, 255, 0)    # 緑（体幹・顔の中心線）
POINT_COLOR = (0, 255, 255)   # 黄（キーポイント円）
PART_COLOR = {"R": COLOR_RIGHT, "L": COLOR_LEFT, "C": COLOR_CENTER}
POINT_RADIUS = 4
LINE_THICKNESS = 2

# ボーン部分隠蔽の線分サンプル数
BONE_SAMPLES = 24

# オクルージョン判定
OCCLUSION_MARGIN = 0.05   # [m] 3DGS深度ノイズ＋マーカーが体表より内側に埋まる分を吸収。これ以上奥なら隠す
ALPHA_THRESH = 0.5        # 累積不透明度がこれ未満の画素は「3DGSなし」扱い（隠さない）。内部定数（CLI非公開）


def load_cameras_toml(toml_path: str) -> dict[str, dict]:
    """Pose2Sim キャリブTOMLを読み込み、カメラ名→パラメータ辞書を返す。

    `matrix` キーを持つテーブルのみカメラとして扱う（metadata 等を除外）。
    `D`（distortions）は読むが本機能のレンダリングでは未使用（ピンホール）。feat-016 で使う。
    """
    with open(toml_path, "rb") as f:
        data = tomli.load(f)

    cameras: dict[str, dict] = {}
    for key, value in data.items():
        if not isinstance(value, dict) or "matrix" not in value:
            continue
        size = value["size"]
        cameras[key] = {
            "name": key,
            "K": np.array(value["matrix"], dtype=np.float64),
            "D": np.array(value["distortions"], dtype=np.float64),
            "rvec": np.array(value["rotation"], dtype=np.float64),
            "tvec": np.array(value["translation"], dtype=np.float64),
            "width": int(round(size[0])),
            "height": int(round(size[1])),
        }
    return cameras


def select_camera(cameras: dict[str, dict], name: str) -> dict:
    """指定名のカメラを返す。存在しなければ利用可能名一覧つきで ValueError。"""
    if name not in cameras:
        available = ", ".join(sorted(cameras.keys()))
        raise ValueError(
            f"カメラ '{name}' がTOMLに見つかりません。利用可能なカメラ: {available}"
        )
    return cameras[name]


def camera_to_viewmat(cam: dict) -> np.ndarray:
    """カメラの world-to-camera 行列（OpenCV規約）を gsplat 用 4x4 viewmat にする。

    TOML の rotation(Rodrigues), translation はそのまま OpenCV world-to-camera なので、
    Blender変換を挟まず直接 viewmat = [[R, t], [0,0,0,1]] とする。
    """
    R, _ = cv2.Rodrigues(np.asarray(cam["rvec"], dtype=np.float64))
    viewmat = np.eye(4, dtype=np.float32)
    viewmat[:3, :3] = R
    viewmat[:3, 3] = np.asarray(cam["tvec"], dtype=np.float64).reshape(3)
    return viewmat


def render_image(
    gaussians: dict, cam: dict, near_plane: float, background=(0.0, 0.0, 0.0),
    return_depth: bool = False,
):
    """gsplat でピンホールレンダリングする。

    `render.py` の `render_frame` と同一の古典経路（歪み・UT なし）。歪みを gsplat の
    UT経路に乗せると黒い靄・品質劣化が出るため使わない。

    device/dtype は CUDA・float32 を明示する。`cam["K"]` は float64 numpy、
    `camera_to_viewmat` は CPU float32 を返すため、明示しないと gsplat の CUDAカーネルで
    デバイス不一致／dtype エラーになる。

    Args:
        gaussians: render.load_ply() の戻り値（CUDAテンソル）
        cam: load_cameras_toml() のカメラ辞書
        near_plane: near クリップ距離[m]（floater除去用）
        background: 背景色 RGB [0-1]
        return_depth: True なら深度マップ・αマップも返す（オクルージョン判定用）。
            既定 False のときは feat-015 と完全に同一挙動（戻り値は BGR のみ）

    Returns:
        return_depth=False: (H,W,3) uint8 BGR
        return_depth=True : (bgr, depth_map, alpha_map)
            depth_map: (H,W) float32 期待深度（カメラ光軸方向）
            alpha_map: (H,W) float32 累積不透明度
    """
    import torch  # phase4 venv のみ。関数内 import
    from gsplat import rasterization

    device = gaussians["means"].device
    viewmat = torch.from_numpy(camera_to_viewmat(cam)).to(device)  # float32 → CUDA
    K = torch.tensor(cam["K"], dtype=torch.float32, device=device)
    bg = torch.tensor(background, dtype=torch.float32, device=device)

    total_sh = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
    sh_degree = int(total_sh ** 0.5) - 1

    # 深度が必要なら RGB+ED（末尾チャンネルが期待深度 Σ(α·z)/Σα）。不要なら RGB のみ
    render_mode = "RGB+ED" if return_depth else "RGB"

    with torch.no_grad():
        rendered, alphas, _meta = rasterization(
            means=gaussians["means"],
            quats=gaussians["quats"],
            scales=gaussians["scales"],
            opacities=gaussians["opacities"],
            colors=torch.cat([gaussians["sh0"], gaussians["sh_rest"]], dim=1),
            viewmats=viewmat.unsqueeze(0),
            Ks=K.unsqueeze(0),
            width=cam["width"],
            height=cam["height"],
            sh_degree=sh_degree,
            near_plane=near_plane,
            far_plane=1e10,
            # 古典ピンホール経路を保証するため、gsplat のデフォルトと同値でも明示的に渡す
            # （バージョン差異でデフォルトが変わっても挙動を固定。要求仕様 3.2）
            camera_model="pinhole",
            with_ut=False,
            packed=True,
            render_mode=render_mode,
        )

    # 先頭3chがRGB（RGB / RGB+ED いずれでも先頭3ch）。背景合成は従来同様
    rgb = rendered.squeeze(0)[..., :3]
    result = (rgb + (1 - alphas.squeeze(0)) * bg).clamp(0, 1)
    bgr = (result.cpu().numpy() * 255).astype(np.uint8)[:, :, ::-1].copy()  # RGB → BGR

    if not return_depth:
        return bgr

    depth_map = rendered.squeeze(0)[..., 3].cpu().numpy().astype(np.float32)  # (H,W)
    # αは (1,H,W,1)/(1,H,W) などshape揺れがあるため squeeze で (H,W) に正規化
    alpha_map = np.squeeze(alphas.squeeze(0).cpu().numpy()).astype(np.float32)
    return bgr, depth_map, alpha_map


# === C3D読み込み・キーポイント投影・オクルージョン判定・描画 ===


def load_c3d_first_frame(c3d_path: str) -> tuple[list[str], np.ndarray, np.ndarray]:
    """C3Dを開き、先頭フレームのみ読んで (labels, data, residual) を返す。

    Args:
        c3d_path: C3Dファイルパス

    Returns:
        labels:   マーカー名リスト（strip済み）
        data:     (n_markers, 3) 先頭フレーム生座標（mm）
        residual: (n_markers,) py-c3d の points[:,3]。residual < 0 は無効サンプル

    Raises:
        ValueError: フレームが0件のとき
    """
    import c3d  # phase4 venv のみ。関数内 import

    with open(c3d_path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [label.strip() for label in reader.point_labels]
        for _frame_no, points, _analog in reader.read_frames():
            # 最初の1フレームだけ取り出して即終了（ジェネレータを全消費しない）
            data = np.asarray(points[:, :3], dtype=np.float64)
            residual = np.asarray(points[:, 3], dtype=np.float64)
            return labels, data, residual

    raise ValueError(f"C3Dにフレームがありません: {c3d_path}")


def c3d_to_calib(points_mm: np.ndarray) -> np.ndarray:
    """C3D生座標(mm) (px,py,pz) → キャリブ座標(m) (pz,px,py)*0.001。

    根拠: Pose2Sim の zup_to_yup（新=(Y,Z,X)）の逆変換。
    元X=新Z(pz), 元Y=新X(px), 元Z=新Y(py)。実データで検証済み。
    入力 (...,3) → 出力 (...,3)。
    """
    p = np.asarray(points_mm, dtype=np.float64)
    px, py, pz = p[..., 0], p[..., 1], p[..., 2]
    return np.stack([pz, px, py], axis=-1) * 0.001


def extract_halpe26(
    labels: list[str], data: np.ndarray, residual: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """ラベルから Halpe26 の26点を抽出し、(26,3) 座標と (26,) 有効フラグを返す。

    有効 = residual >= 0 かつ 座標にNaNを含まない。
    Halpe26名がC3Dに欠ける場合は不足名を表示して ValueError。
    """
    name_to_c3d_idx = {name: i for i, name in enumerate(labels)}
    missing = [name for name in HALPE26_NAMES if name not in name_to_c3d_idx]
    if missing:
        raise ValueError(
            f"C3DにHalpe26マーカーが不足しています: {', '.join(missing)}"
        )

    kpts = np.empty((26, 3), dtype=np.float64)
    valid = np.empty(26, dtype=bool)
    for i, name in enumerate(HALPE26_NAMES):
        idx = name_to_c3d_idx[name]
        coord = data[idx]
        kpts[i] = coord
        valid[i] = bool(residual[idx] >= 0) and not bool(np.isnan(coord).any())
    return kpts, valid


def project_keypoints(kpts_calib: np.ndarray, cam: dict) -> np.ndarray:
    """キャリブ座標 (N,3) を歪みなし（ピンホール）投影して (N,2) を返す。

    背景の3DGSがピンホールのため distCoeffs=None で整合させる。
    NaN座標は nan_to_num でサニタイズしてから渡す（無効点は描画側でスキップ）。
    """
    pts = np.nan_to_num(np.asarray(kpts_calib, dtype=np.float64))
    pts2d, _ = cv2.projectPoints(
        pts.reshape(-1, 1, 3),
        np.asarray(cam["rvec"], dtype=np.float64),
        np.asarray(cam["tvec"], dtype=np.float64),
        np.asarray(cam["K"], dtype=np.float64),
        distCoeffs=None,
    )
    return pts2d.reshape(-1, 2)


def compute_keypoint_depth(kpts_calib: np.ndarray, cam: dict) -> np.ndarray:
    """各キーポイントのカメラ深度 (R·X+t) のZ成分を返す。入力 (...,3) → 出力 (...,)。"""
    R, _ = cv2.Rodrigues(np.asarray(cam["rvec"], dtype=np.float64))
    t = np.asarray(cam["tvec"], dtype=np.float64).reshape(3)
    X = np.asarray(kpts_calib, dtype=np.float64)
    # (R @ X^T)^T + t の Z成分
    cam_coords = X @ R.T + t
    return cam_coords[..., 2]


def compute_visibility(
    pts2d: np.ndarray,
    depth_cam: np.ndarray,
    valid: np.ndarray,
    depth_map: np.ndarray,
    alpha_map: np.ndarray,
    margin: float,
    near_plane: float,
) -> np.ndarray:
    """各点の可視性 (n,) bool を返す（オクルージョン判定）。

    判定順序（変えないこと）:
      1. valid False → False
      2. depth_cam が非finite or <= near_plane（背面・至近）→ False
      3. 投影位置が画像外 → True（隠す3DGSが画面内に無い）
      4. alpha_map < ALPHA_THRESH（3DGSなし/薄い）→ True
      5. depth_cam > depth_map + margin（3DGSより奥）→ False（隠れる）
      6. それ以外 → True

    α判定(4)を深度比較(5)より先に置く（低α画素の深度は不安定なため）。
    背面ガード(2)は投影座標を信頼する前に置く。
    """
    pts2d = np.asarray(pts2d, dtype=np.float64)
    depth_cam = np.asarray(depth_cam, dtype=np.float64)
    valid = np.asarray(valid, dtype=bool)
    H, W = depth_map.shape[:2]

    n = pts2d.shape[0]
    visible = np.empty(n, dtype=bool)
    for i in range(n):
        if not valid[i]:
            visible[i] = False
            continue
        d = depth_cam[i]
        if not np.isfinite(d) or d <= near_plane:
            visible[i] = False
            continue
        col = int(round(pts2d[i, 0]))
        row = int(round(pts2d[i, 1]))
        if not (0 <= col < W and 0 <= row < H):
            visible[i] = True  # 枠外：オクルージョン判定不能。描画はOpenCVクリッピングに委ねる
            continue
        if alpha_map[row, col] < ALPHA_THRESH:
            visible[i] = True
            continue
        if d > depth_map[row, col] + margin:
            visible[i] = False
            continue
        visible[i] = True
    return visible


def draw_overlay(
    image: np.ndarray,
    kpts_calib: np.ndarray,
    pts2d: np.ndarray,
    valid: np.ndarray,
    kp_visible: np.ndarray,
    cam: dict,
    depth_map,
    alpha_map,
    margin: float,
    near_plane: float,
    occlusion: bool = True,
) -> np.ndarray:
    """背景画像のコピーに点とボーンをオクルージョン考慮で描画して返す。

    点: occlusion=True 時は valid[i] and kp_visible[i] のみ。occlusion=False 時は valid[i] のみ。
    ボーン: 両端 valid の辺を BONE_SAMPLES 個に補間し、可視な隣接サンプルのみ線で結ぶ
            （occlusion=False 時は両端valid辺を全描画）。
    occlusion=False 時は depth_map/alpha_map を参照しないので None 可。
    """
    img = image.copy()

    # ボーン（点より先に描き、点を上に乗せる）
    for start, end, part in HALPE26_SKELETON:
        ia, ib = NAME_TO_IDX[start], NAME_TO_IDX[end]
        if not (valid[ia] and valid[ib]):
            continue
        color = PART_COLOR[part]
        if not occlusion:
            pa = tuple(np.round(pts2d[ia]).astype(int))
            pb = tuple(np.round(pts2d[ib]).astype(int))
            cv2.line(img, pa, pb, color, LINE_THICKNESS)
            continue

        # 部分隠蔽: 3D座標を線形補間 → 投影・深度 → 可視判定 → 可視な隣接ペアを結ぶ
        ts = np.linspace(0.0, 1.0, BONE_SAMPLES)
        seg3d = (1 - ts)[:, None] * kpts_calib[ia] + ts[:, None] * kpts_calib[ib]
        seg2d = project_keypoints(seg3d, cam)
        seg_depth = compute_keypoint_depth(seg3d, cam)
        seg_visible = compute_visibility(
            seg2d, seg_depth, np.ones(BONE_SAMPLES, dtype=bool),
            depth_map, alpha_map, margin, near_plane,
        )
        for k in range(BONE_SAMPLES - 1):
            if seg_visible[k] and seg_visible[k + 1]:
                pa = tuple(np.round(seg2d[k]).astype(int))
                pb = tuple(np.round(seg2d[k + 1]).astype(int))
                cv2.line(img, pa, pb, color, LINE_THICKNESS)

    # 点
    for i in range(len(HALPE26_NAMES)):
        if not valid[i]:
            continue
        if occlusion and not kp_visible[i]:
            continue
        center = tuple(np.round(pts2d[i]).astype(int))
        cv2.circle(img, center, POINT_RADIUS, POINT_COLOR, -1)

    return img


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ピンホール3DGSレンダリングへの人体キーポイント重ね描き（オクルージョン考慮、PNG出力）"
    )
    parser.add_argument("ply_path", help="3DGS PLYファイルパス")
    parser.add_argument("toml_path", help="キャリブレーションTOMLパス")
    parser.add_argument("c3d_path", help="人体キーポイントC3Dパス（Halpe26、先頭フレームを使用）")
    parser.add_argument("--camera", required=True, help="TOML内の対象カメラ名")
    parser.add_argument("--near-plane", type=float, default=0.1,
                        help="near クリップ距離[m]（floater除去用、default: 0.1）")
    parser.add_argument("--output", default=None,
                        help="出力PNGパス（default: ./data/keypoints_<カメラ名>.png）")
    parser.add_argument("--background", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="背景色 RGB [0-1] (default: 0 0 0)")
    parser.add_argument("--no-occlusion", action="store_true",
                        help="オクルージョン無効（全キーポイントを手前扱いで描画＝旧2D重ね描き。比較用）")
    parser.add_argument("--occlusion-margin", type=float, default=OCCLUSION_MARGIN,
                        help=f"深度マージン[m]（3DGSよりこの値以上奥なら隠す、default: {OCCLUSION_MARGIN}）")
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    # カメラ選択（torch/gsplat の重いロード前に行い、誤ったカメラ名を早期検出する）
    cameras = load_cameras_toml(args.toml_path)
    try:
        cam = select_camera(cameras, args.camera)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    print(f"対象カメラ: {cam['name']} ({cam['width']}x{cam['height']})")

    # C3Dロード・Halpe26抽出・キャリブ座標変換（重いPLY/torchロード前に検証）
    print(f"C3D読み込み中（先頭フレーム）: {args.c3d_path}")
    try:
        labels, c3d_data, residual = load_c3d_first_frame(args.c3d_path)
        kpts_mm, valid = extract_halpe26(labels, c3d_data, residual)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    kpts_calib = c3d_to_calib(kpts_mm)
    print(f"Halpe26: 有効 {int(valid.sum())}/26 点")

    # PLY読み込み（render.py は torch を top-level import するため関数内 import）
    from render import load_ply, print_ply_summary

    print(f"PLYファイル読み込み中: {args.ply_path}")
    gaussians = load_ply(args.ply_path)
    print_ply_summary(args.ply_path, gaussians)

    output = args.output if args.output else f"./data/keypoints_{cam['name']}.png"
    occlusion = not args.no_occlusion

    # レンダリング（オクルージョン時のみ深度・αを取得）
    print(f"\nレンダリング中（near_plane={args.near_plane}, "
          f"occlusion={'ON' if occlusion else 'OFF'}）...")
    start_time = time.time()

    pts2d = project_keypoints(kpts_calib, cam)
    if occlusion:
        bgr, depth_map, alpha_map = render_image(
            gaussians, cam, args.near_plane, tuple(args.background), return_depth=True
        )
        depth_cam = compute_keypoint_depth(kpts_calib, cam)
        kp_visible = compute_visibility(
            pts2d, depth_cam, valid, depth_map, alpha_map,
            args.occlusion_margin, args.near_plane,
        )
        print(f"可視キーポイント: {int((valid & kp_visible).sum())}/26 点")
    else:
        bgr = render_image(
            gaussians, cam, args.near_plane, tuple(args.background)
        )
        depth_map = alpha_map = None
        kp_visible = np.ones(26, dtype=bool)

    overlay = draw_overlay(
        bgr, kpts_calib, pts2d, valid, kp_visible, cam,
        depth_map, alpha_map, args.occlusion_margin, args.near_plane,
        occlusion=occlusion,
    )

    # 出力ディレクトリ作成（カレント直下出力時の makedirs("") を防ぐため空dirnameをガード）
    d = os.path.dirname(output)
    if d:
        os.makedirs(d, exist_ok=True)

    # PNG保存。cv2.imwrite は False 返却（権限・エンコード失敗等）と cv2.error 送出
    # （不正な拡張子・拡張子なし等）の2系統で失敗しうるため両方を扱う
    try:
        ok = cv2.imwrite(output, overlay)
    except cv2.error as e:
        print(f"エラー: PNGの保存に失敗しました: {output}: {e}", file=sys.stderr)
        return 1
    if not ok:
        print(f"エラー: PNGの保存に失敗しました: {output}", file=sys.stderr)
        return 1

    elapsed = time.time() - start_time
    print(f"\n完了: {output} ({elapsed:.1f}秒)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

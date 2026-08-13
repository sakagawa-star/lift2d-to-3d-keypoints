"""ピンホール3DGSレンダリングへの人体キーポイント重ね描き（オクルージョン考慮、連番PNG/MP4出力）

指定した1台のカメラ視点で 3DGS点群（PLY）を gsplat の古典ピンホール経路で
レンダリングし、その背景画像に人体3Dキーポイント（C3D）を全フレーム
（または --start-frame/--end-frame で指定した範囲）投影して点＋ボーンで重ね描きする。
その際、3DGSの深度マップと各キーポイントのカメラ深度を比較して前後関係（オクルージョン）を
反映し、カメラ手前の3DGS（障害物）に隠れるキーポイント・ボーンを隠蔽する。
カメラは固定のため背景レンダリング（RGB・深度・α）は1回だけ計算し全フレームで共有する。
キャリブレーション（R, t, K）の妥当性を時系列（動画）で目視検証する用途。

入力（Blenderを経由せず元データを直読み）:
  - PLY : LCC Studio製3DGS（ネイティブ座標＝キャリブレーション座標、Z-up・メートル）
  - TOML: Pose2Simキャリブ（OpenCV規約 world-to-camera、複数カメラ）
  - C3D または NPZ: 人体キーポイント（Halpe26 + Spine/Thorax の既知28マーカー。
          拡張子 .npz で NPZ、それ以外は C3D と判別。C3D はmm、NPZ は x3d_world（world座標[m]）。
          入力に無い既知マーカーは描画スキップ＝欠損許容。全フレーム使用）

実行は phase4 venv で行う（torch/gsplat が必要）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \\
      data/Blender/point_cloud.ply data/Blender/Config_scene.toml \\
      data/Blender/keypoints.c3d \\
      --camera cam41520554 --near-plane 0.5 --output-dir /tmp/keypoints --mp4

  NPZ を渡す場合は拡張子を .npz にするだけでよい（例: data/Blender/keypoints.npz）

1フレームだけ確認したい場合は --start-frame N --end-frame N で範囲を1枚に絞る。

設定YAML（--config）でオプションをまとめて指定することもできる（feat-033）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py --config run_keypoints.yaml
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import tomli

from render_fps_video import load_yaml_flat, _yaml_bool, parse_config_yaml  # noqa: F401


# === キーポイント定数（feat-013 design.md より復元、feat-021 で欠損許容に拡張） ===

# Halpe26 キーポイント名（C3D label と一致）
HALPE26_NAMES = [
    "Hip", "RHip", "RKnee", "RAnkle", "RBigToe", "RSmallToe", "RHeel",
    "LHip", "LKnee", "LAnkle", "LBigToe", "LSmallToe", "LHeel",
    "Neck", "Head", "Nose", "LEye", "REye", "LEar", "REar",
    "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist",
]

# 既知マーカー（描画対象）: Halpe26 の26点 + 体幹中間点2点。計28点。
# C3Dに存在しない既知マーカーは valid=False として点・ボーンをスキップする（欠損許容）
KEYPOINT_NAMES = HALPE26_NAMES + ["Spine", "Thorax"]

# ボーン端点名 → KEYPOINT_NAMES内インデックスの辞書
NAME_TO_IDX = {name: i for i, name in enumerate(KEYPOINT_NAMES)}

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


def distortions_to_gsplat(D: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """TOML歪み係数（OpenCV並び、長さ4/5/8）を gsplat 用に詰め替える。

    Returns:
        (radial, tangential): radial は6要素 [k1,k2,k3,k4,k5,k6]、
        tangential は2要素 [p1,p2]（いずれも float64 numpy）
    Raises:
        ValueError: 長さが4/5/8以外
    """
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    n = D.shape[0]
    tangential = D[2:4]
    if n == 4:
        radial = np.array([D[0], D[1], 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    elif n == 5:
        radial = np.array([D[0], D[1], D[4], 0.0, 0.0, 0.0], dtype=np.float64)
    elif n == 8:
        radial = np.array([D[0], D[1], D[4], D[5], D[6], D[7]], dtype=np.float64)
    else:
        raise ValueError(f"歪み係数の長さは4/5/8のいずれかである必要があります（実際: {n}）")
    return radial, tangential


def render_image(
    gaussians: dict, cam: dict, near_plane: float, background=(0.0, 0.0, 0.0),
    return_depth: bool = False, distort: bool = False,
):
    """gsplat でレンダリングする。

    `distort=False`（既定）: `render.py` の `render_frame` と同一の古典経路（歪み・UT なし）。
    `distort=True`: TOMLの歪み係数を gsplat の3DGUT経路（`with_ut=True, with_eval3d=True,
    packed=False` + radial/tangential係数）に乗せてOpenCV歪みモデル込みでレンダリングする
    （feat-024。スパイクで、靄の真因は near_plane=0.01 の floater であり、
    `with_eval3d=True` + 適切な near_plane なら UT経路は実用可であることを確認済み）。

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
        distort: True なら TOMLの歪み係数（`cam["D"]`）で3DGUT経路レンダリングする（feat-024）

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
        if distort:
            radial, tangential = distortions_to_gsplat(cam["D"])
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
                camera_model="pinhole",
                with_ut=True,
                with_eval3d=True,
                packed=False,
                radial_coeffs=torch.tensor([radial], dtype=torch.float32, device=device),
                tangential_coeffs=torch.tensor([tangential], dtype=torch.float32, device=device),
                render_mode=render_mode,
            )
        else:
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


def load_c3d_all_frames(c3d_path: str) -> tuple[list[str], list[dict], float]:
    """C3Dを開き、全フレームを読んで (labels, frames_data, point_rate) を返す。

    Args:
        c3d_path: C3Dファイルパス

    Returns:
        labels:      マーカー名リスト（strip済み）
        frames_data: フレームごとの dict のリスト。各要素:
            {"frame_no": int, "data": (n_markers,3) float64 mm,
             "residual": (n_markers,) float64}（residual < 0 は無効サンプル）
        point_rate:  C3D rate [Hz]（float）。取得不能（None・非数値）なら 0.0。
            丸めず小数のまま保持する（MP4 fps の実時間一致のため）

    Raises:
        ValueError: フレームが0件のとき
    """
    import c3d  # phase4 venv のみ。関数内 import

    with open(c3d_path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [label.strip() for label in reader.point_labels]
        # point_rate は None・欠損・非数値があり得るため安全に正規化（例外を投げない）
        raw_rate = getattr(reader, "point_rate", None)
        try:
            point_rate = float(raw_rate)
        except (TypeError, ValueError):
            point_rate = 0.0
        frames_data: list[dict] = []
        for frame_no, points, _analog in reader.read_frames():
            frames_data.append({
                "frame_no": int(frame_no),
                "data": np.asarray(points[:, :3], dtype=np.float64),
                "residual": np.asarray(points[:, 3], dtype=np.float64),
            })

    if not frames_data:
        raise ValueError(f"C3Dにフレームがありません: {c3d_path}")
    return labels, frames_data, point_rate


def load_npz_all_frames(npz_path: str) -> tuple[list[str], list[dict], float]:
    """NPZ（リフトアップ済み3Dキーポイント）を読み込み、全フレームを (labels, frames_data, point_rate) にする。

    `npz_to_c3d.load_npz` / `npz_to_c3d.world_to_c3d_raw` を再利用し、world座標[m]を
    C3D raw座標(mm) に変換して frames_data を構築する（既存パイプラインとの接続、FR-003）。
    residual は座標の有限性から合成する（NPZに residual 相当の情報が無いため、FR-004）。
    `pnp_ok` は参照しない（全フレーム描画、ユーザー決定 2026-08-13）。

    Args:
        npz_path: NPZファイルパス

    Returns:
        labels:      関節名リスト（NPZ の joint_names）
        frames_data: フレームごとの dict のリスト。各要素:
            {"frame_no": int(frame_ids[i]), "data": (n_markers,3) float64 mm,
             "residual": (n_markers,) float64}（residual < 0 は無効サンプル）
        point_rate:  0.0 固定（NPZ にレート情報が無いため）

    Raises:
        ValueError: フレームが0件、または NPZ の検証に失敗したとき
        FileNotFoundError: ファイルが存在しないとき
    """
    from npz_to_c3d import load_npz, world_to_c3d_raw  # 関数内 import（既存流儀に合わせる）

    try:
        x3d_world, frame_ids, joint_names = load_npz(npz_path)
    except (ValueError, FileNotFoundError):
        raise  # そのまま main で捕捉（メッセージ流用）
    except Exception as e:  # zip破損・EOFError・pickle系等を正規化
        raise ValueError(f"NPZファイルを読み込めません: {npz_path}: {e}") from e

    if x3d_world.shape[0] == 0:
        raise ValueError(f"NPZにフレームがありません: {npz_path}")

    raw = world_to_c3d_raw(x3d_world)              # (F, J, 3) float64 mm
    finite = np.isfinite(raw).all(axis=-1)          # (F, J) bool
    residuals = np.where(finite, 0.0, -1.0)         # (F, J) float64

    frames_data = [
        {"frame_no": int(fid), "data": raw[i], "residual": residuals[i]}
        for i, fid in enumerate(frame_ids)
    ]
    return joint_names, frames_data, 0.0


def c3d_to_calib(points_mm: np.ndarray) -> np.ndarray:
    """C3D生座標(mm) (px,py,pz) → キャリブ座標(m) (pz,px,py)*0.001。

    根拠: Pose2Sim の zup_to_yup（新=(Y,Z,X)）の逆変換。
    元X=新Z(pz), 元Y=新X(px), 元Z=新Y(py)。実データで検証済み。
    入力 (...,3) → 出力 (...,3)。
    """
    p = np.asarray(points_mm, dtype=np.float64)
    px, py, pz = p[..., 0], p[..., 1], p[..., 2]
    return np.stack([pz, px, py], axis=-1) * 0.001


def build_skeleton(present: set[str]) -> list[tuple[str, str, str]]:
    """C3Dに存在する既知マーカー集合から描画するスケルトン（ボーン列）を構築する。

    体幹は Spine/Thorax の有無で切り替える（両方あれば Neck–Thorax–Spine–Hip の3本、
    どちらも無ければ従来の Neck–Hip 直結）。体幹ボーンはベーススケルトンの
    Neck–Hip と同じ位置に挿入する（描画順を維持し、26点C3Dで従来と同一描画を保証）。
    両端点のいずれかが present に無いボーンは除外する。

    Args:
        present: C3Dラベルに存在する既知マーカー名の集合（フレームごとの valid ではない）

    Returns:
        (始点名, 終点名, 部位) のリスト。部位: "R"=右半身, "L"=左半身, "C"=体幹/顔
    """
    # 体幹ボーン（Spine/Thorax の有無で4通り）
    has_thorax = "Thorax" in present
    has_spine = "Spine" in present
    if has_thorax and has_spine:
        torso = [("Neck", "Thorax", "C"), ("Thorax", "Spine", "C"), ("Spine", "Hip", "C")]
    elif has_thorax:
        torso = [("Neck", "Thorax", "C"), ("Thorax", "Hip", "C")]
    elif has_spine:
        torso = [("Neck", "Spine", "C"), ("Spine", "Hip", "C")]
    else:
        torso = [("Neck", "Hip", "C")]  # 体幹フォールバックボーン（従来定義）

    skeleton = [
        ("Head", "Nose", "C"), ("Nose", "Neck", "C"), *torso,
        ("Nose", "REye", "R"), ("REye", "REar", "R"),
        ("Nose", "LEye", "L"), ("LEye", "LEar", "L"),
        ("Neck", "RShoulder", "R"), ("RShoulder", "RElbow", "R"), ("RElbow", "RWrist", "R"),
        ("Neck", "LShoulder", "L"), ("LShoulder", "LElbow", "L"), ("LElbow", "LWrist", "L"),
        ("Hip", "RHip", "R"), ("RHip", "RKnee", "R"), ("RKnee", "RAnkle", "R"),
        ("Hip", "LHip", "L"), ("LHip", "LKnee", "L"), ("LKnee", "LAnkle", "L"),
        ("RAnkle", "RHeel", "R"), ("RAnkle", "RBigToe", "R"), ("RBigToe", "RSmallToe", "R"),
        ("LAnkle", "LHeel", "L"), ("LAnkle", "LBigToe", "L"), ("LBigToe", "LSmallToe", "L"),
    ]
    return [b for b in skeleton if b[0] in present and b[1] in present]


def extract_keypoints(
    labels: list[str], data: np.ndarray, residual: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """ラベルから既知マーカー28点を抽出し、(28,3) 座標と (28,) 有効フラグを返す。

    有効 = マーカーがC3Dに存在し、residual >= 0 かつ 座標にNaNを含まない。
    C3Dに存在しない既知マーカーは valid=False・座標 0.0 とする（欠損許容。例外は投げない）。
    """
    name_to_c3d_idx = {name: i for i, name in enumerate(labels)}
    n = len(KEYPOINT_NAMES)
    kpts = np.zeros((n, 3), dtype=np.float64)
    valid = np.zeros(n, dtype=bool)
    for i, name in enumerate(KEYPOINT_NAMES):
        if name not in name_to_c3d_idx:
            continue  # 欠損マーカー: kpts=0, valid=False のまま
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
    skeleton: list[tuple[str, str, str]],
    depth_map,
    alpha_map,
    margin: float,
    near_plane: float,
    occlusion: bool = True,
) -> np.ndarray:
    """背景画像のコピーに点とボーンをオクルージョン考慮で描画して返す。

    点: occlusion=True 時は valid[i] and kp_visible[i] のみ。occlusion=False 時は valid[i] のみ。
    ボーン: skeleton（build_skeleton の戻り値）のうち両端 valid の辺を BONE_SAMPLES 個に
            補間し、可視な隣接サンプルのみ線で結ぶ（occlusion=False 時は両端valid辺を全描画）。
    occlusion=False 時は depth_map/alpha_map を参照しないので None 可。
    """
    img = image.copy()

    # ボーン（点より先に描き、点を上に乗せる）
    for start, end, part in skeleton:
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
    for i in range(len(KEYPOINT_NAMES)):
        if not valid[i]:
            continue
        if occlusion and not kp_visible[i]:
            continue
        center = tuple(np.round(pts2d[i]).astype(int))
        cv2.circle(img, center, POINT_RADIUS, POINT_COLOR, -1)

    return img


def start_ffmpeg(output_dir: str, width: int, height: int, fps: float):
    """rawvideo(rgb24) を受け取りMP4を書き出す ffmpeg プロセスを起動して返す。

    render.py の方式を移植。NVENC が使えれば h264_nvenc、なければ libx264 にフォールバックする。
    fps は float のまま `-r` に渡す（29.97 等の非整数を丸めず実時間を保つ）。

    Args:
        output_dir: 出力ディレクトリ（MP4は <output_dir>/output.mp4）
        width, height: フレーム解像度
        fps: フレームレート [Hz]（小数可）

    Returns:
        (proc, mp4_path): subprocess.Popen と MP4出力パス

    Raises:
        FileNotFoundError: ffmpeg が見つからないとき
    """
    import shutil
    import subprocess

    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin is None:
        raise FileNotFoundError("ffmpegが見つかりません")

    # NVENC の利用可能性を確認
    result = subprocess.run(
        [ffmpeg_bin, "-hide_banner", "-encoders"],
        capture_output=True, text=True
    )
    use_nvenc = "h264_nvenc" in result.stdout
    encoder = "h264_nvenc" if use_nvenc else "libx264"
    print(f"MP4エンコーダ: {encoder}")

    mp4_path = os.path.join(output_dir, "output.mp4")
    ffmpeg_cmd = [
        ffmpeg_bin,
        "-y",
        "-f", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}",
        "-r", f"{fps}",
        "-i", "pipe:0",
        "-c:v", encoder,
        "-pix_fmt", "yuv420p",
        "-loglevel", "error",
        mp4_path,
    ]
    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc, mp4_path


# === feat-033: 設定YAML（--config） ===

def _yaml_floats3(value: str) -> list[float]:
    """設定YAMLの background 値（スペース区切り float 3個）を変換する。"""
    parts = value.split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"スペース区切りの数値3個である必要があります: {value}")
    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"数値に変換できません: {value}") from e


CONFIG_CONVERTERS = {
    "ply_path": str, "toml_path": str, "keypoints_path": str,
    "camera": str, "output_dir": str,
    "near_plane": float, "occlusion_margin": float, "mp4_fps": float,
    "start_frame": int, "end_frame": int,
    "background": _yaml_floats3,
    "no_occlusion": _yaml_bool, "mp4": _yaml_bool, "no_png": _yaml_bool,
    "no_keypoints": _yaml_bool, "distort": _yaml_bool,
}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ピンホール3DGSレンダリングへの人体キーポイント重ね描き（オクルージョン考慮、連番PNG/MP4出力）",
        # 前方一致の短縮を禁止。廃止した --output が --output-dir に化けるのを防ぎ、明示的にエラーにする
        allow_abbrev=False,
    )
    parser.add_argument("ply_path", nargs="?", default=None,
                        help="3DGS PLYファイルパス（--config でも指定可）")
    parser.add_argument("toml_path", nargs="?", default=None,
                        help="キャリブレーションTOMLパス（--config でも指定可）")
    parser.add_argument("keypoints_path", nargs="?", default=None,
                        help="人体キーポイントファイルパス（C3D または NPZ。拡張子 .npz で NPZ と"
                             "判別。Halpe26 + Spine/Thorax の既知28マーカー、欠損許容、"
                             "全フレームを使用。--no-keypoints 時は省略）（--config でも指定可）")
    parser.add_argument("--camera", default=None,
                        help="TOML内の対象カメラ名（--config でも指定可）")
    parser.add_argument("--near-plane", type=float, default=0.1,
                        help="near クリップ距離[m]（floater除去用、default: 0.1）")
    parser.add_argument("--output-dir", default=None,
                        help="連番PNG出力ディレクトリ（default: ./data/keypoints_<カメラ名>/）")
    parser.add_argument("--background", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="背景色 RGB [0-1] (default: 0 0 0)")
    parser.add_argument("--no-occlusion", action="store_true",
                        help="オクルージョン無効（全キーポイントを手前扱いで描画＝旧2D重ね描き。比較用）")
    parser.add_argument("--occlusion-margin", type=float, default=None,
                        help=f"深度マージン[m]（3DGSよりこの値以上奥なら隠す、default: {OCCLUSION_MARGIN}）")
    parser.add_argument("--start-frame", type=int, default=None,
                        help="描画開始フレーム番号（C3D frame_no / NPZ frame_ids。"
                             "この番号を含む、省略時は最小フレーム）")
    parser.add_argument("--end-frame", type=int, default=None,
                        help="描画終了フレーム番号（同、省略時は最大フレーム）")
    parser.add_argument("--mp4", action="store_true",
                        help="連番PNGに加えてMP4動画も出力する")
    parser.add_argument("--mp4-fps", type=float, default=None,
                        help="MP4フレームレート（小数可、default: C3D rate。"
                             "NPZ はレート情報が無いため 30）")
    parser.add_argument("--no-png", action="store_true",
                        help="連番PNG保存をスキップしMP4のみ出力する（--mp4 と併用必須）")
    parser.add_argument("--no-keypoints", action="store_true",
                        help="キーポイント描画をスキップし3DGS背景のみの静止画1枚を出力"
                             "（keypoints_path 省略必須）")
    parser.add_argument("--distort", action="store_true",
                        help="TOMLの歪み係数で歪みモデルレンダリング（--no-keypoints 専用）")
    parser.add_argument("--config", default=None,
                        help="設定YAML（フラット key: value）。優先順位は "
                             "CLI > 設定YAML > デフォルト。フラグ系（--no-occlusion/--mp4/"
                             "--no-png/--no-keypoints/--distort）は CLI から true 方向のみ"
                             "上書き可（設定YAMLの true を false に戻すには設定YAMLを編集する）")
    return parser


def parse_args_with_config(
    argv=None, parser: argparse.ArgumentParser | None = None
) -> argparse.Namespace:
    """--config と CLI 引数を統合してパースする（優先順位: CLI > 設定YAML > デフォルト）。

    Args:
        argv: コマンドライン引数（None ならsys.argv）。
        parser: 使用するパーサー（None なら内部で _build_parser() を呼ぶ）。
            main() は自作の parser を渡し、以降の parser.error 呼び出しで
            同一のparserオブジェクトを使い続ける。

    Returns:
        統合・検証済みの argparse.Namespace。
    """
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default=None)
    pre_args, _ = pre_parser.parse_known_args(argv)

    if parser is None:
        parser = _build_parser()

    if pre_args.config is not None:
        defaults = parse_config_yaml(pre_args.config, parser, converters=CONFIG_CONVERTERS)
        parser.set_defaults(**defaults)

    args = parser.parse_args(argv)

    required_dests = [
        ("ply_path", "ply_path"), ("toml_path", "toml_path"), ("camera", "--camera"),
    ]
    missing = [label for dest, label in required_dests if getattr(args, dest) is None]
    if missing:
        parser.error(
            f"必須項目が未指定です（CLIまたは--configで指定）: {', '.join(missing)}"
        )
    return args


def _run_still_mode(args: argparse.Namespace, cam: dict) -> int:
    """静止画モード（--no-keypoints）を実行する。3DGS背景のみのPNGを1枚出力する。"""
    if args.distort:
        try:
            distortions_to_gsplat(cam["D"])
        except ValueError as e:
            print(e, file=sys.stderr)
            return 1

    from render import load_ply, print_ply_summary

    print(f"PLYファイル読み込み中: {args.ply_path}")
    gaussians = load_ply(args.ply_path)
    print_ply_summary(args.ply_path, gaussians)

    output_dir = args.output_dir if args.output_dir else f"./data/keypoints_{cam['name']}"

    print(f"\n背景レンダリング中（near_plane={args.near_plane}, "
          f"distort={'ON' if args.distort else 'OFF'}）...")
    start_time = time.time()

    bgr = render_image(
        gaussians, cam, args.near_plane, tuple(args.background), distort=args.distort
    )

    os.makedirs(output_dir, exist_ok=True)
    png_path = os.path.join(output_dir, f"still_{cam['name']}.png")
    try:
        ok = cv2.imwrite(png_path, bgr)
    except cv2.error as e:
        print(f"エラー: PNGの保存に失敗しました: {png_path}: {e}", file=sys.stderr)
        return 1
    if not ok:
        print(f"エラー: PNGの保存に失敗しました: {png_path}", file=sys.stderr)
        return 1

    elapsed = time.time() - start_time
    print(f"\n完了: {png_path} ({elapsed:.1f}秒)")
    return 0


def main(argv=None) -> int:
    parser = _build_parser()
    args = parse_args_with_config(argv, parser)

    # --no-keypoints / keypoints_path / --distort の組み合わせ検証（重い処理前、--no-png単独チェックより前）
    if args.no_keypoints:
        if args.keypoints_path is not None:
            parser.error("--no-keypoints 指定時は keypoints_path を渡せません"
                         "（静止画モードはキーポイント入力不要）")
        forbidden = [("--mp4", args.mp4), ("--mp4-fps", args.mp4_fps is not None),
                     ("--no-png", args.no_png),
                     ("--start-frame", args.start_frame is not None),
                     ("--end-frame", args.end_frame is not None),
                     ("--no-occlusion", args.no_occlusion),
                     ("--occlusion-margin", args.occlusion_margin is not None)]
        for name, given in forbidden:
            if given:
                parser.error(f"--no-keypoints 指定時は {name} は使用できません")
    else:
        if args.keypoints_path is None:
            parser.error("keypoints_path を省略する場合は --no-keypoints を指定してください")
        if args.distort:
            parser.error("--distort は --no-keypoints と併用してください（動画モードは未対応）")

    if args.occlusion_margin is None:
        args.occlusion_margin = OCCLUSION_MARGIN

    # --no-png 単独指定は出力が何もなくなるため、重い処理に入る前に拒否する
    if args.no_png and not args.mp4:
        parser.error("--no-png は --mp4 と併用してください（両方省略時は連番PNGのみ出力）")

    # カメラ選択（torch/gsplat の重いロード前に行い、誤ったカメラ名を早期検出する）
    cameras = load_cameras_toml(args.toml_path)
    try:
        cam = select_camera(cameras, args.camera)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    print(f"対象カメラ: {cam['name']} ({cam['width']}x{cam['height']})")

    if args.no_keypoints:
        return _run_still_mode(args, cam)

    # キーポイントロード（全フレーム）・既知マーカー存在チェック（重いPLY/torchロード前に検証）
    is_npz = args.keypoints_path.lower().endswith(".npz")
    fmt = "NPZ" if is_npz else "C3D"
    print(f"キーポイント読み込み中（{fmt}, 全フレーム）: {args.keypoints_path}")
    try:
        if is_npz:
            labels, frames_data, point_rate = load_npz_all_frames(args.keypoints_path)
        else:
            labels, frames_data, point_rate = load_c3d_all_frames(args.keypoints_path)
    except (ValueError, FileNotFoundError) as e:
        print(e, file=sys.stderr)
        return 1
    # マーカー構成は入力全体で固定。既知マーカー0個なら描画対象がないため早期終了する
    present = {name for name in KEYPOINT_NAMES if name in labels}
    if not present:
        print(f"{fmt}に既知マーカーが1つもありません（labels: {', '.join(labels)}）",
              file=sys.stderr)
        return 1
    missing = [name for name in KEYPOINT_NAMES if name not in present]
    print(f"キーポイント: {len(present)}/{len(KEYPOINT_NAMES)} マーカーを描画対象とします")
    if missing:
        print(f"欠損マーカー（描画スキップ）: {', '.join(missing)}")
    skeleton = build_skeleton(present)
    total_frames = len(frames_data)
    print(f"{fmt}: {total_frames} フレーム, rate={point_rate} Hz")

    # フレーム範囲フィルタ（入力フレーム番号ベース、両端含む。render.py と同方式）
    if args.start_frame is not None or args.end_frame is not None:
        start = args.start_frame if args.start_frame is not None else -float("inf")
        end = args.end_frame if args.end_frame is not None else float("inf")
        frames_data = [fr for fr in frames_data if start <= fr["frame_no"] <= end]
        if not frames_data:
            print(f"エラー: 指定範囲 (start={args.start_frame}, end={args.end_frame}) に"
                  f"該当するフレームがありません", file=sys.stderr)
            return 1
        print(f"フレーム範囲: {total_frames} フレーム中 {len(frames_data)} フレームを対象")

    # PLY読み込み（render.py は torch を top-level import するため関数内 import）
    from render import load_ply, print_ply_summary

    print(f"PLYファイル読み込み中: {args.ply_path}")
    gaussians = load_ply(args.ply_path)
    print_ply_summary(args.ply_path, gaussians)

    output_dir = args.output_dir if args.output_dir else f"./data/keypoints_{cam['name']}"
    occlusion = not args.no_occlusion

    # 背景レンダリング（カメラ固定のため全フレームで共有。1回のみ実行）
    print(f"\n背景レンダリング中（near_plane={args.near_plane}, "
          f"occlusion={'ON' if occlusion else 'OFF'}）...")
    start_time = time.time()

    if occlusion:
        bgr_bg, depth_map, alpha_map = render_image(
            gaussians, cam, args.near_plane, tuple(args.background), return_depth=True
        )
    else:
        bgr_bg = render_image(
            gaussians, cam, args.near_plane, tuple(args.background)
        )
        depth_map = alpha_map = None

    # 出力ディレクトリ作成
    os.makedirs(output_dir, exist_ok=True)

    # MP4出力時は ffmpeg をフレームループ前に起動（不在ならPNGも出さず終了）
    ffmpeg_proc = None
    mp4_path = None
    if args.mp4:
        if args.mp4_fps is not None:
            fps = float(args.mp4_fps)
        elif point_rate and point_rate > 0:
            fps = point_rate
        else:
            fps = 30.0
            print("警告: 入力にフレームレート情報がないため MP4 fps=30 を使用", file=sys.stderr)
        try:
            ffmpeg_proc, mp4_path = start_ffmpeg(
                output_dir, cam["width"], cam["height"], fps
            )
        except FileNotFoundError as e:
            print(f"エラー: {e}", file=sys.stderr)
            return 1
        print(f"MP4出力: {mp4_path} (fps={fps})")

    # フレームループ。ffmpeg起動済みならクリーンアップを保証するため try/finally で囲む
    rc = 0
    try:
        n = len(frames_data)
        for i, fr in enumerate(frames_data):
            kpts_mm, valid = extract_keypoints(labels, fr["data"], fr["residual"])
            kpts_calib = c3d_to_calib(kpts_mm)
            pts2d = project_keypoints(kpts_calib, cam)
            if occlusion:
                depth_cam = compute_keypoint_depth(kpts_calib, cam)
                kp_visible = compute_visibility(
                    pts2d, depth_cam, valid, depth_map, alpha_map,
                    args.occlusion_margin, args.near_plane,
                )
            else:
                kp_visible = np.ones(len(KEYPOINT_NAMES), dtype=bool)

            overlay = draw_overlay(
                bgr_bg, kpts_calib, pts2d, valid, kp_visible, cam, skeleton,
                depth_map, alpha_map, args.occlusion_margin, args.near_plane,
                occlusion=occlusion,
            )

            # 連番PNG保存（--no-png 時はスキップ。cv2.imwrite は False 返却と
            # cv2.error 送出の2系統で失敗しうる）
            if not args.no_png:
                png_path = os.path.join(output_dir, f"frame_{fr['frame_no']:06d}.png")
                try:
                    ok = cv2.imwrite(png_path, overlay)
                except cv2.error as e:
                    print(f"エラー: PNGの保存に失敗しました: {png_path}: {e}",
                          file=sys.stderr)
                    rc = 1
                    break
                if not ok:
                    print(f"エラー: PNGの保存に失敗しました: {png_path}", file=sys.stderr)
                    rc = 1
                    break

            # MP4: overlay は BGR。ffmpeg には rgb24 を渡すため RGB化して書き込む
            if ffmpeg_proc is not None:
                rgb = overlay[:, :, ::-1].copy()
                try:
                    ffmpeg_proc.stdin.write(rgb.tobytes())
                except (BrokenPipeError, OSError):
                    stderr = ffmpeg_proc.stderr.read().decode(errors="replace")
                    print(f"エラー: ffmpegが途中終了しました:\n{stderr}", file=sys.stderr)
                    rc = 1
                    break

            # 進捗表示（--no-png 時は png_path が未定義のためフレーム番号で表示）
            if args.no_png:
                print(f"  [{i + 1}/{n}] frame {fr['frame_no']} -> mp4")
            else:
                suffix = " -> mp4" if ffmpeg_proc is not None else ""
                print(f"  [{i + 1}/{n}] {png_path}{suffix}")
    finally:
        # ffmpegプロセスのクリーンアップ（PNG失敗・write失敗・想定外例外の全経路で実行）
        if ffmpeg_proc is not None:
            if ffmpeg_proc.stdin and not ffmpeg_proc.stdin.closed:
                try:
                    ffmpeg_proc.stdin.close()
                except OSError:
                    pass
            ffmpeg_proc.wait()
            if rc == 0 and ffmpeg_proc.returncode != 0:
                stderr = ffmpeg_proc.stderr.read().decode(errors="replace")
                print(f"エラー: ffmpegエラー:\n{stderr}", file=sys.stderr)
                rc = 1

    if rc != 0:
        return rc

    elapsed = time.time() - start_time
    print(f"\n完了: {output_dir} ({len(frames_data)} フレーム, {elapsed:.1f}秒)")
    if mp4_path is not None:
        print(f"MP4保存: {mp4_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

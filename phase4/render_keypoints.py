"""ピンホールカメラによる3DGS点群レンダリング（PNG出力）

指定した1台のカメラ視点で 3DGS点群（PLY）を gsplat の古典ピンホール経路で
レンダリングし、PNG1枚を出力する。出力PNGをグランドトゥルース（Blender+KIRIで
同カメラ視点をレンダリングした `0001_color.png`）と並べて、3DGSレンダリングの
正しさ（カメラ位置・向き・構図・floater除去）を目視検証する。

人体キーポイント重ね描き・オクルージョンは feat-016 で扱う（本スクリプトは扱わない）。

入力（Blenderを経由せず元データを直読み）:
  - PLY : LCC Studio製3DGS（ネイティブ座標＝キャリブレーション座標、Z-up・メートル）
  - TOML: Pose2Simキャリブ（OpenCV規約 world-to-camera、複数カメラ）

実行は phase4 venv で行う（torch/gsplat が必要）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \\
      data/Blender/point_cloud.ply data/Blender/Config_scene.toml \\
      --camera cam41520554 --near-plane 0.5 --output /tmp/render.png
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import tomli


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
    gaussians: dict, cam: dict, near_plane: float, background=(0.0, 0.0, 0.0)
) -> np.ndarray:
    """gsplat でピンホールレンダリングし、(H,W,3) uint8 BGR を返す。

    `render.py` の `render_frame` と同一の古典経路（歪み・UT・深度なし）。歪みを gsplat の
    UT経路に乗せると黒い靄・品質劣化が出るため使わない。

    device/dtype は CUDA・float32 を明示する。`cam["K"]` は float64 numpy、
    `camera_to_viewmat` は CPU float32 を返すため、明示しないと gsplat の CUDAカーネルで
    デバイス不一致／dtype エラーになる。

    Args:
        gaussians: render.load_ply() の戻り値（CUDAテンソル）
        cam: load_cameras_toml() のカメラ辞書
        near_plane: near クリップ距離[m]（floater除去用）
        background: 背景色 RGB [0-1]
    """
    import torch  # phase4 venv のみ。関数内 import
    from gsplat import rasterization

    device = gaussians["means"].device
    viewmat = torch.from_numpy(camera_to_viewmat(cam)).to(device)  # float32 → CUDA
    K = torch.tensor(cam["K"], dtype=torch.float32, device=device)
    bg = torch.tensor(background, dtype=torch.float32, device=device)

    total_sh = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
    sh_degree = int(total_sh ** 0.5) - 1

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
        )

    result = (rendered.squeeze(0) + (1 - alphas.squeeze(0)) * bg).clamp(0, 1)
    bgr = (result.cpu().numpy() * 255).astype(np.uint8)[:, :, ::-1].copy()  # RGB → BGR
    return bgr


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ピンホールカメラによる3DGS点群レンダリング（PNG出力）"
    )
    parser.add_argument("ply_path", help="3DGS PLYファイルパス")
    parser.add_argument("toml_path", help="キャリブレーションTOMLパス")
    parser.add_argument("--camera", required=True, help="TOML内の対象カメラ名")
    parser.add_argument("--near-plane", type=float, default=0.1,
                        help="near クリップ距離[m]（floater除去用、default: 0.1）")
    parser.add_argument("--output", default=None,
                        help="出力PNGパス（default: ./data/render_<カメラ名>.png）")
    parser.add_argument("--background", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        help="背景色 RGB [0-1] (default: 0 0 0)")
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

    # PLY読み込み（render.py は torch を top-level import するため関数内 import）
    from render import load_ply, print_ply_summary

    print(f"PLYファイル読み込み中: {args.ply_path}")
    gaussians = load_ply(args.ply_path)
    print_ply_summary(args.ply_path, gaussians)

    output = args.output if args.output else f"./data/render_{cam['name']}.png"

    # レンダリング
    print(f"\nレンダリング中（near_plane={args.near_plane}）...")
    start_time = time.time()
    bgr = render_image(gaussians, cam, args.near_plane, tuple(args.background))

    # 出力ディレクトリ作成（カレント直下出力時の makedirs("") を防ぐため空dirnameをガード）
    d = os.path.dirname(output)
    if d:
        os.makedirs(d, exist_ok=True)

    # PNG保存。cv2.imwrite は False 返却（権限・エンコード失敗等）と cv2.error 送出
    # （不正な拡張子・拡張子なし等）の2系統で失敗しうるため両方を扱う（要求仕様 3.4）
    try:
        ok = cv2.imwrite(output, bgr)
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

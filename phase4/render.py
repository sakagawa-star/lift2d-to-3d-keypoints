"""gsplatバッチレンダリングスクリプト

PLYファイルとカメラポーズJSONを受け取り、フレームごとに画像を出力する。
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from plyfile import PlyData

from gsplat import rasterization


def load_ply(path: str) -> dict[str, torch.Tensor]:
    """PLYファイルを読み込み、活性化関数適用済みテンソルをCUDAで返す。

    Returns:
        {
            "means": Tensor (N, 3),       # ガウシアン中心座標
            "scales": Tensor (N, 3),      # exp(scale_raw)
            "quats": Tensor (N, 4),       # normalized quaternion
            "opacities": Tensor (N,),     # sigmoid(opacity_raw)
            "sh0": Tensor (N, 1, 3),      # DC成分 (SH degree 0)
            "sh_rest": Tensor (N, K, 3),  # 高次SH成分（存在する場合）
        }
    """
    plydata = PlyData.read(path)
    vertex = plydata["vertex"]
    n = len(vertex)

    # means: そのまま
    means = torch.tensor(
        np.stack([vertex["x"], vertex["y"], vertex["z"]], axis=-1),
        dtype=torch.float32,
    )

    # scales: exp変換
    scales_raw = torch.tensor(
        np.stack([vertex["scale_0"], vertex["scale_1"], vertex["scale_2"]], axis=-1),
        dtype=torch.float32,
    )
    scales = torch.exp(scales_raw)

    # quats: 正規化
    quats_raw = torch.tensor(
        np.stack([vertex["rot_0"], vertex["rot_1"], vertex["rot_2"], vertex["rot_3"]], axis=-1),
        dtype=torch.float32,
    )
    quats = F.normalize(quats_raw, dim=-1)

    # opacities: sigmoid変換
    opacity_raw = torch.tensor(
        np.array(vertex["opacity"]),
        dtype=torch.float32,
    )
    opacities = torch.sigmoid(opacity_raw)

    # SH DC成分: 生のまま保持
    sh0 = torch.tensor(
        np.stack([vertex["f_dc_0"], vertex["f_dc_1"], vertex["f_dc_2"]], axis=-1),
        dtype=torch.float32,
    ).reshape(n, 1, 3)

    # SH高次成分: f_rest_* 属性を探索
    prop_names = [p.name for p in vertex.properties]
    f_rest_names = sorted([name for name in prop_names if name.startswith("f_rest_")])

    if len(f_rest_names) > 0:
        f_rest_data = np.stack([vertex[name] for name in f_rest_names], axis=-1)
        k = len(f_rest_names) // 3  # 3チャンネル分
        sh_rest = torch.tensor(f_rest_data, dtype=torch.float32).reshape(n, k, 3)
    else:
        sh_rest = torch.zeros(n, 0, 3, dtype=torch.float32)

    # CUDA転送
    return {
        "means": means.cuda(),
        "scales": scales.cuda(),
        "quats": quats.cuda(),
        "opacities": opacities.cuda(),
        "sh0": sh0.cuda(),
        "sh_rest": sh_rest.cuda(),
    }


def print_ply_summary(path: str, gaussians: dict[str, torch.Tensor]) -> None:
    """PLYロード結果のサマリーを表示する。"""
    n = gaussians["means"].shape[0]
    total_sh = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
    sh_degree = int(total_sh ** 0.5) - 1

    print(f"=== PLY Load Summary ===")
    print(f"File: {path}")
    print(f"Num Gaussians: {n}")

    for name, tensor in gaussians.items():
        shape_str = f"shape={tuple(tensor.shape)}"
        if tensor.numel() == 0:
            print(f"  {name:12s} {shape_str:30s} (empty)")
            continue
        min_val = tensor.min().item()
        max_val = tensor.max().item()
        extra = ""
        if name == "scales":
            extra = " (after exp)"
        elif name == "quats":
            extra = " (normalized)"
        elif name == "opacities":
            extra = " (after sigmoid)"
        elif name == "sh_rest" and tensor.shape[1] > 0:
            extra = f" (SH degree {sh_degree})"

        print(f"  {name:12s} {shape_str:30s} min={min_val:+.4f}  max={max_val:+.4f}{extra}")


def load_camera_json(path: str) -> list[dict]:
    """カメラポーズJSONを読み込み、必須フィールドを検証する。

    JSONはフレームの配列（トップレベルがリスト）。
    必須フィールド（各フレーム）:
        frame (int), width (int), height (int),
        fx (float), fy (float), cx (float), cy (float),
        c2w (4x4 list)

    フィールドが欠損している場合はValueErrorを送出する。
    """
    with open(path) as f:
        frames = json.load(f)

    required_fields = ["frame", "width", "height", "fx", "fy", "cx", "cy", "c2w"]
    for i, frame in enumerate(frames):
        for field in required_fields:
            if field not in frame:
                raise ValueError(f"フレーム {i}: 必須フィールド '{field}' が欠損しています")
        # c2wが4x4であることを検証
        c2w = frame["c2w"]
        if len(c2w) != 4 or any(len(row) != 4 for row in c2w):
            raise ValueError(f"フレーム {i}: 'c2w' は 4x4 行列である必要があります")

    return frames


def rotate_z90(c2w: torch.Tensor) -> torch.Tensor:
    """c2wをワールドZ軸回りに+90度回転させる（Blenderアドオンのバグ補正用）。

    ワールド座標系での回転のため左乗算: c2w_corrected = R_z90 @ c2w
    blender_to_opencv_c2w の右乗算（カメラローカル軸の変換）とは異なる。
    """
    r = torch.tensor([
        [0, -1, 0, 0],
        [1,  0, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=c2w.dtype)
    return r @ c2w


def blender_to_opencv_c2w(c2w: torch.Tensor) -> torch.Tensor:
    """Blender座標系のcamera-to-worldをOpenCV座標系に変換する。

    Blender: +X右, +Y上, -Z前方
    OpenCV:  +X右, -Y上, +Z前方

    変換: Y軸とZ軸を反転（カメラのローカル軸に対して）
    c2w_opencv = c2w_blender @ M_convert
    M_convert = diag(1, -1, -1, 1)
    """
    convert = torch.tensor([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [0,  0,  0, 1],
    ], dtype=c2w.dtype)
    return c2w @ convert


def render_frame(gaussians: dict[str, torch.Tensor], camera: dict) -> torch.Tensor:
    """1フレームをレンダリングする。

    Args:
        gaussians: load_ply()の戻り値
        camera: {
            "width": int,
            "height": int,
            "fx": float, "fy": float, "cx": float, "cy": float,
            "viewmat": Tensor (4x4),  # world-to-camera (OpenCV座標系)
            "background": Tensor (1, 3),  # 背景色
        }

    Returns:
        Tensor (H, W, 3) float32 [0, 1]
    """
    K = torch.tensor([
        [camera["fx"], 0, camera["cx"]],
        [0, camera["fy"], camera["cy"]],
        [0, 0, 1],
    ], device="cuda", dtype=torch.float32)

    # SH次数の算出
    total_sh_coeffs = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
    sh_degree = int(total_sh_coeffs ** 0.5) - 1

    rendered, alphas, meta = rasterization(
        means=gaussians["means"],
        quats=gaussians["quats"],
        scales=gaussians["scales"],
        opacities=gaussians["opacities"],
        colors=torch.cat([gaussians["sh0"], gaussians["sh_rest"]], dim=1),
        viewmats=camera["viewmat"].unsqueeze(0),
        Ks=K.unsqueeze(0),
        width=camera["width"],
        height=camera["height"],
        sh_degree=sh_degree,
        near_plane=0.01,
        far_plane=1e10,
    )

    # 背景色を適用（alphaブレンディング）
    result = rendered.squeeze(0)  # (H, W, 3)
    bg = camera.get("background")
    if bg is not None:
        alpha = alphas.squeeze(0)  # (H, W, 1)
        result = result + (1 - alpha) * bg.squeeze(0)  # bg: (1,3) → (3,)

    return result.clamp(0, 1)


def main():
    parser = argparse.ArgumentParser(description="gsplat batch renderer")
    parser.add_argument("ply_path", help="3DGS PLYファイルパス")
    parser.add_argument("camera_json", help="カメラポーズJSONパス")
    parser.add_argument("--output-dir", default="./data/images", help="出力ディレクトリ")
    parser.add_argument("--background", type=float, nargs=3, default=[0, 0, 0],
                        help="背景色 RGB [0-1] (default: 0 0 0)")
    parser.add_argument("--rotate-z90", action="store_true",
                        help="カメラポーズをZ軸回りに+90度回転（Blenderアドオンのバグ補正用）")
    parser.add_argument("--start-frame", type=int, default=None,
                        help="レンダリング開始フレーム番号")
    parser.add_argument("--end-frame", type=int, default=None,
                        help="レンダリング終了フレーム番号（この番号を含む）")
    parser.add_argument("--dry-run", action="store_true",
                        help="レンダリングのみ実行し、PNG保存をスキップする")
    args = parser.parse_args()

    # PLY読み込み
    print(f"PLYファイル読み込み中: {args.ply_path}")
    gaussians = load_ply(args.ply_path)
    print_ply_summary(args.ply_path, gaussians)

    # カメラポーズ読み込み
    frames = load_camera_json(args.camera_json)
    total_frames = len(frames)

    # フレーム範囲でフィルタリング
    if args.start_frame is not None or args.end_frame is not None:
        start = args.start_frame if args.start_frame is not None else -float("inf")
        end = args.end_frame if args.end_frame is not None else float("inf")
        frames = [f for f in frames if start <= f["frame"] <= end]
        if len(frames) == 0:
            print(f"エラー: 指定範囲 (start={args.start_frame}, end={args.end_frame}) に該当するフレームがありません")
            sys.exit(1)
        print(f"\nカメラポーズ: {total_frames} フレーム中 {len(frames)} フレームを対象")
    else:
        print(f"\nカメラポーズ: {total_frames} フレーム")

    # 背景色テンソル
    background = torch.tensor([args.background], dtype=torch.float32, device="cuda")

    # 出力ディレクトリ作成
    if not args.dry_run:
        os.makedirs(args.output_dir, exist_ok=True)

    # バッチレンダリング
    start_time = time.time()

    with torch.no_grad():
        if args.dry_run:
            torch.cuda.synchronize()

        for i, frame in enumerate(frames):
            # Blender→OpenCV座標変換
            c2w_blender = torch.tensor(frame["c2w"], dtype=torch.float32)
            if args.rotate_z90:
                c2w_blender = rotate_z90(c2w_blender)
            c2w_opencv = blender_to_opencv_c2w(c2w_blender)
            viewmat = torch.inverse(c2w_opencv).cuda()

            camera = {
                "width": frame["width"],
                "height": frame["height"],
                "fx": frame["fx"], "fy": frame["fy"],
                "cx": frame["cx"], "cy": frame["cy"],
                "viewmat": viewmat,
                "background": background,
            }

            # レンダリング
            image = render_frame(gaussians, camera)

            # PNG保存（dry-run時はGPU→CPU転送もスキップ）
            if not args.dry_run:
                out_path = os.path.join(args.output_dir, f"frame_{frame['frame']:06d}.png")
                img_np = (image.cpu().numpy() * 255).astype(np.uint8)
                Image.fromarray(img_np).save(out_path)
                print(f"  [{i + 1}/{len(frames)}] {out_path}")
            else:
                print(f"  [{i + 1}/{len(frames)}] frame {frame['frame']} (dry-run)")

        if args.dry_run:
            torch.cuda.synchronize()

    elapsed = time.time() - start_time
    print(f"\n完了: {elapsed:.1f}秒 ({elapsed / len(frames):.3f}秒/フレーム)")


if __name__ == "__main__":
    main()

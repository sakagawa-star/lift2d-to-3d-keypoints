"""案1 Stage 2（疎通確認）: 真ポーズでのクエリ生成と再現性・見え・αの確認

内容:
  1. 真ポーズ（cam05520126 の TOML 値）で歪み付きレンダリング（色+深度+α）→ クエリとして保存
  2. 同じ真ポーズでもう一度レンダリングし、画素差を測る（再現性）
  3. αマップの統計（高α画素の割合）を測る

歪み付き深度の取得方法（方式A）:
  gsplat 1.5.3 の 3DGUT 経路（with_ut=True, with_eval3d=True）は render_mode="RGB+ED" を
  受け付けない（カーネルが channels==3 のアサーションで停止。2026-07-31 の疎通確認で判明）。
  そのため 3DGUT レンダを2回呼ぶ:
    1回目: 色（既存 render_image(distort=True, return_depth=False) をそのまま使用）
    2回目: 各ガウシアンのカメラ座標奥行き z_i を「色」として流し、αブレンド出力を α で
           正規化して期待深度 D = Σ(w·z)/Σw を得る（pipeline.md Stage 2 の D の定義式と同一。
           α > 1e-6 の有効画素に限り gsplat の ED と同一セマンティクス。完全背景は gsplat の
           ED では 0 深度相当になるのに対し本実装では NaN とする——Stage 4 の有効性ゲート前提
           ではこの方が安全）
  αは2回目の呼び出しの alphas 出力を用いる（1回目と同一ポーズ・同一ガウシアンなので一致するはず）。

実行方法（プロジェクトルートで）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
    docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage2_smoke_test.py

出力: このスクリプトと同じディレクトリに query_cam05520126.png / depth・α統計（stage2_result.txt）
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase4"))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import (  # noqa: E402
    load_cameras_toml, select_camera, render_image,
    camera_to_viewmat, distortions_to_gsplat,
)


def render_depth_alpha_distorted(gaussians: dict, cam: dict, near_plane: float):
    """3DGUT 経路（歪みモデル込み）で期待深度マップとαマップを得る（方式A）。

    3DGUT 経路は render_mode="RGB+ED" 非対応のため、各ガウシアンのカメラ座標奥行き
    z_i を「色」（3ch複製）として RGB モードで流す。ブレンド出力は Σ(w·z)（背景合成なし）、
    alphas は Σw なので、除算で α 正規化済み期待深度 D = Σ(w·z)/Σw を得る。

    render_image(distort=True) と同一のレンダリング設定（with_ut/with_eval3d/packed/
    near_plane/far_plane/係数の詰め替え）を用いる。sh_degree は渡さない（SH 評価なしの
    直接色として z を流すため）。

    Returns:
        (depth_map, alpha_map): いずれも (H, W) float32 numpy。
        alpha ≈ 0 の画素（ガウシアンが無い背景）の depth は NaN。
    """
    import torch
    from gsplat import rasterization

    device = gaussians["means"].device
    viewmat_np = camera_to_viewmat(cam)  # (4,4) float32, OpenCV world-to-camera
    viewmat = torch.from_numpy(viewmat_np).to(device)
    K = torch.tensor(cam["K"], dtype=torch.float32, device=device)

    # 各ガウシアン中心のカメラ座標奥行き z_i = (R μ + t) の z 成分
    R = viewmat[:3, :3]
    t = viewmat[:3, 3]
    z_cam = (gaussians["means"] @ R.T + t)[:, 2]  # (N,)
    depth_colors = z_cam.unsqueeze(-1).expand(-1, 3).contiguous()  # (N,3) 3ch複製

    radial, tangential = distortions_to_gsplat(cam["D"])
    with torch.no_grad():
        rendered, alphas, _meta = rasterization(
            means=gaussians["means"],
            quats=gaussians["quats"],
            scales=gaussians["scales"],
            opacities=gaussians["opacities"],
            colors=depth_colors,
            viewmats=viewmat.unsqueeze(0),
            Ks=K.unsqueeze(0),
            width=cam["width"],
            height=cam["height"],
            near_plane=near_plane,
            far_plane=1e10,
            camera_model="pinhole",
            with_ut=True,
            with_eval3d=True,
            packed=False,
            radial_coeffs=torch.tensor(np.asarray([radial]), dtype=torch.float32, device=device),
            tangential_coeffs=torch.tensor(np.asarray([tangential]), dtype=torch.float32, device=device),
            render_mode="RGB",
        )

    accum = rendered.squeeze(0)[..., 0]  # Σ(w·z)（3chとも同値のため先頭chを使用）
    alpha = alphas.squeeze(0).squeeze(-1)  # Σw
    depth = torch.where(alpha > 1e-6, accum / alpha.clamp(min=1e-6),
                        torch.full_like(accum, float("nan")))
    return depth.cpu().numpy().astype(np.float32), alpha.cpu().numpy().astype(np.float32)

PLY_PATH = "/home/sakagawa/data/PLY/HandaiHospital-20260418-01/point_cloud/iteration_100/point_cloud.ply"
TOML_PATH = "/home/sakagawa/git/pose2sim/Pose2Sim/20260418_osaka-hosp1/calibration/Config_scene.toml"
TRUE_CAM = "cam05520126"
NEAR_PLANE = 0.5  # render_keypoints.py の使用例に合わせる（floater除去。既定0.1ではない点を記録）
OUT_DIR = Path(__file__).resolve().parent


def main():
    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    print(f"カメラ: {cam['name']} size={cam['width']}x{cam['height']} "
          f"fx={cam['K'][0,0]:.1f} 歪み係数={len(cam['D'])}個")

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    n_gauss = gaussians["means"].shape[0]
    print(f"ガウシアン数: {n_gauss}")

    # 色: レンダ1回目（クエリ）+ 2回目（再現性確認）。方式Aにより色と深度は別呼び出し
    bgr1 = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
    bgr2 = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
    # 深度+α: 1回目 + 2回目（再現性確認）
    depth1, alpha1 = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)
    depth2, alpha2 = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)

    # 予測1: 再現性（画素一致。差 ≤ 1/255 程度）
    img_diff_max = int(np.abs(bgr1.astype(np.int16) - bgr2.astype(np.int16)).max())
    both = np.isfinite(depth1) & np.isfinite(depth2)
    depth_diff_max = float(np.abs(depth1[both] - depth2[both]).max()) if both.any() else float("nan")
    nan_mismatch = int((np.isfinite(depth1) != np.isfinite(depth2)).sum())
    alpha_diff_max = float(np.abs(alpha1 - alpha2).max())

    # 予測3: αの分布
    alpha_stats = {
        "alpha>0.5 の画素割合": f"{float((alpha1 > 0.5).mean())*100:.2f}%",
        "alpha>0.9 の画素割合": f"{float((alpha1 > 0.9).mean())*100:.2f}%",
        "alpha中央値": f"{float(np.median(alpha1)):.4f}",
    }
    # 深度の概況（以降の段の予測材料）。有効画素ゼロ＝疎通失敗（座標系破綻・near_plane過大等）
    # でも結果ファイルを残して明示的に失敗報告する
    valid = (alpha1 > 0.5) & np.isfinite(depth1)
    invalid_in_hi_alpha = (alpha1 > 0.5) & ~np.isfinite(depth1)
    if valid.any():
        smoke_ok = True
        depth_stats = {
            "深度中央値[m](α>0.5)": f"{float(np.median(depth1[valid])):.3f}",
            "深度p5/p95[m]": f"{float(np.percentile(depth1[valid], 5)):.3f} / "
                              f"{float(np.percentile(depth1[valid], 95)):.3f}",
            "α>0.5なのに深度が非有限の画素": f"{int(invalid_in_hi_alpha.sum())}個",
            "深度NaN(背景等)の画素割合": f"{float((~np.isfinite(depth1)).mean())*100:.2f}%",
        }
    else:
        smoke_ok = False
        depth_stats = {"深度統計": "N/A（α>0.5 かつ有限深度の画素がゼロ。"
                                  "座標系の破綻・near_plane 過大・視野外等を疑うこと）"}

    cv2.imwrite(str(OUT_DIR / "query_cam05520126.png"), bgr1)
    np.save(OUT_DIR / "query_depth.npy", depth1)
    np.save(OUT_DIR / "query_alpha.npy", alpha1)

    lines = [
        "Stage 2（疎通確認）実測結果",
        f"疎通判定: {'OK' if smoke_ok else '失敗（有効深度画素ゼロ）'}",
        f"シーン: {PLY_PATH}",
        f"カメラ: {TRUE_CAM} / near_plane={NEAR_PLANE} / distort=True / ガウシアン数={n_gauss}",
        "",
        f"[予測1: 再現性] 2回レンダの画素差 最大={img_diff_max} (uint8), "
        f"深度差 最大={depth_diff_max:.3e} m, NaN位置不一致={nan_mismatch}画素, α差 最大={alpha_diff_max:.3e}",
        f"[予測3: α] " + ", ".join(f"{k}: {v}" for k, v in alpha_stats.items()),
        f"[深度の概況] " + ", ".join(f"{k}: {v}" for k, v in depth_stats.items()),
        "",
        "予測2（見えの成立）は query_cam05520126.png を目視確認のこと",
    ]
    (OUT_DIR / "stage2_result.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()

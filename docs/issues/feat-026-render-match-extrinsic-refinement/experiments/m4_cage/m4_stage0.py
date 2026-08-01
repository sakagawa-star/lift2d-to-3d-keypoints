"""feat-026 M4 Stage 0: 疎通確認(criteria.md §8-1)

檻環境データで以下を実行・記録する(合否判定はしない。判定は本実験 m4_run.py):
  1. 6点PnP(K既知)で初期ポーズを算出(criteria.md §5-1 の経路)
  2. 初期ポーズでレンダし、実写との重畳PNGを出力(座標系整合の目視確認用)
  3. レンダの有効画素率(α≥0.5)・有効深度の分布(z下限0.5mとの干渉有無)・
     レンダ/実写それぞれの SIFT キーポイント数を記録
  4. 初期ポーズでの手動6点の再投影誤差を記録

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_stage0.py

出力: このスクリプトと同じディレクトリに m4_stage0_result.txt(コミット対象) /
      m4_stage0_overlay.png・m4_stage0_render.png(ローカル、目視確認用)
"""
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
A1_DIR = Path(__file__).resolve().parents[1] / "a1_synthetic"
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(A1_DIR))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import render_depth_alpha_distorted, NEAR_PLANE  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
DATA_DIR = OUT_DIR / "data"
PLY_PATH = DATA_DIR / "point_cloud.ply"
INTRINSICS_PATH = DATA_DIR / "intrinsics_all.toml"
IMAGE_PATH = DATA_DIR / "images" / "cam05520125.png"
POINTS_3D_PATH = DATA_DIR / "kijunten_locations.csv"
POINTS_2D_PATH = DATA_DIR / "points_2d.csv"

CAMERA = "cam05520125"  # criteria.md §2: パイロットカメラ
GATE_ALPHA = 0.5        # criteria.md §4(pipeline_loop.py と同値。有効画素率の定義に使用)
GATE_Z_RANGE = (0.5, 10.0)  # criteria.md §4(完全固定。干渉の有無を記録する)


def load_matched_points(cam_name: str):
    """ObjectName で 3D-2D をマッチ(m3_run.py と同一方式)"""
    pts3d = {}
    with open(POINTS_3D_PATH) as f:
        for row in csv.DictReader(f):
            pts3d[row["ObjectName"]] = [float(row["X"]), float(row["Y"]), float(row["Z"])]
    p3, p2, names = [], [], []
    with open(POINTS_2D_PATH) as f:
        for row in csv.DictReader(f):
            if row["camera_name"] == cam_name and row["ObjectName"] in pts3d:
                p3.append(pts3d[row["ObjectName"]])
                p2.append([float(row["X"]), float(row["Y"])])
                names.append(row["ObjectName"])
    return (np.array(p3, dtype=np.float64), np.array(p2, dtype=np.float64), names)


def pnp_6pt(p3, p2, K, D, names, lines):
    """6点PnP(criteria.md §5-1: estimate_camera_params.py の K既知・6点以上経路と同一)"""
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        p3, p2, K, D, iterationsCount=1000,
        reprojectionError=8.0, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None:
        lines.append("6点PnP: solvePnPRansac 失敗")
        return None
    inl = sorted(inl.flatten().tolist())
    outl = [names[i] for i in range(len(names)) if i not in inl]
    if len(inl) < 4:
        lines.append(f"6点PnP: inlier {len(inl)} < 4")
        return None
    ok, rvec, tvec = cv2.solvePnP(
        p3[inl], p2[inl], K, D, rvec=rvec, tvec=tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        lines.append("6点PnP: ITERATIVE 精密化失敗")
        return None
    rvec = np.asarray(rvec, dtype=np.float64).flatten()
    tvec = np.asarray(tvec, dtype=np.float64).flatten()
    lines.append(f"6点PnP: 成功 inlier={len(inl)}/{len(names)}"
                 + (f" outlier={outl}" if outl else ""))
    lines.append(f"  rvec={rvec.tolist()}")
    lines.append(f"  tvec={tvec.tolist()}")
    C = -cv2.Rodrigues(rvec.reshape(3, 1))[0].T @ tvec
    lines.append(f"  カメラ中心(世界座標)={C.tolist()}")
    return rvec, tvec


def main():
    lines = [
        "M4 Stage 0: 疎通確認(criteria.md §8-1。合否判定はしない)",
        f"シーン: {PLY_PATH}",
        f"クエリ: {IMAGE_PATH}",
        f"near_plane={NEAR_PLANE} / distort=True",
        "",
    ]

    # 入力読み込み
    cams = load_cameras_toml(str(INTRINSICS_PATH))
    cam = select_camera(cams, CAMERA)
    img_q = cv2.imread(str(IMAGE_PATH))
    if img_q is None:
        raise SystemExit(f"クエリ画像を読めません: {IMAGE_PATH}")
    if img_q.shape[:2] != (cam["height"], cam["width"]):
        raise SystemExit(f"画像サイズ {img_q.shape[:2]} が TOML"
                         f" ({cam['height']},{cam['width']}) と不一致")
    p3, p2, names = load_matched_points(CAMERA)
    lines.append(f"手動プロット点: {len(names)}点 ({', '.join(names)})")
    if len(names) != 6:
        lines.append(f"手動点数が criteria §5-1(6点PnP)と不一致: {len(names)}点")
        (OUT_DIR / "m4_stage0_result.txt").write_text("\n".join(lines) + "\n")
        raise SystemExit(f"手動点が6点ちょうどではありません: {len(names)}点"
                         "(結果は m4_stage0_result.txt に記録)")

    # 1. 6点PnP
    res = pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res is None:
        (OUT_DIR / "m4_stage0_result.txt").write_text("\n".join(lines) + "\n")
        raise SystemExit("6点PnP 失敗(結果は m4_stage0_result.txt に記録)")
    rvec, tvec = res

    # 4. 初期ポーズでの手動6点の再投影誤差(全点。inlier 名簿の別掲はしない: 6点のみのため)
    proj, _ = cv2.projectPoints(p3, rvec.reshape(3, 1), tvec.reshape(3, 1),
                                cam["K"], cam["D"])
    err = np.linalg.norm(proj.reshape(-1, 2) - p2, axis=1)
    lines.append("")
    lines.append("手動6点の再投影誤差(初期ポーズ):")
    for n, e in zip(names, err):
        lines.append(f"  {n}: {e:.3f}px")
    lines.append(f"  中央値={np.median(err):.3f}px 最大={err.max():.3f}px")

    # 2. 初期ポーズでレンダ
    gaussians = load_ply(str(PLY_PATH))
    cam_r = dict(cam)
    cam_r["rvec"] = rvec
    cam_r["tvec"] = tvec
    bgr = render_image(gaussians, cam_r, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_r, NEAR_PLANE)

    # 3-a. 有効画素率と深度分布
    valid_a = alpha > GATE_ALPHA
    finite_d = np.isfinite(depth) & valid_a
    lines.append("")
    lines.append(f"レンダ有効画素率(α>{GATE_ALPHA}): {valid_a.mean():.3f}")
    if finite_d.any():
        dv = depth[finite_d]
        below = float((dv <= GATE_Z_RANGE[0]).mean())
        above = float((dv >= GATE_Z_RANGE[1]).mean())
        lines.append(f"有効深度分布: min={dv.min():.3f}m p5={np.percentile(dv, 5):.3f}m "
                     f"中央値={np.median(dv):.3f}m p95={np.percentile(dv, 95):.3f}m "
                     f"max={dv.max():.3f}m")
        lines.append(f"z範囲({GATE_Z_RANGE[0]},{GATE_Z_RANGE[1]})mとの干渉: "
                     f"下限以下 {below*100:.1f}% / 上限以上 {above*100:.1f}%"
                     "(criteria.md §4: 記録のみ。合否救済には使わない)")
    else:
        lines.append("有効深度画素なし(レンダ破綻の可能性)")

    # 3-b. SIFT キーポイント数(b-4 の物差し)
    sift = cv2.SIFT_create()
    img_q_gray = cv2.cvtColor(img_q, cv2.COLOR_BGR2GRAY)
    img_r_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    kp_q = sift.detect(img_q_gray, None)
    kp_r = sift.detect(img_r_gray, None)
    ratio = len(kp_r) / len(kp_q) if len(kp_q) > 0 else float("nan")
    lines.append(f"SIFT キーポイント数: 実写={len(kp_q)} レンダ={len(kp_r)} "
                 f"比(レンダ/実写)={ratio:.3f}")

    # 重畳PNG(目視確認用): 実写とレンダの50%ブレンド + 手動点(緑=手動2D, 赤=再投影)
    overlay = cv2.addWeighted(img_q, 0.5, bgr, 0.5, 0)
    for (x, y), (px, py) in zip(p2, proj.reshape(-1, 2)):
        cv2.circle(overlay, (int(round(x)), int(round(y))), 6, (0, 255, 0), -1)
        cv2.circle(overlay, (int(round(px)), int(round(py))), 4, (0, 0, 255), -1)
    cv2.imwrite(str(OUT_DIR / "m4_stage0_render.png"), bgr)
    cv2.imwrite(str(OUT_DIR / "m4_stage0_overlay.png"), overlay)
    lines.append("")
    lines.append("重畳PNG: m4_stage0_overlay.png(緑=手動2D点, 赤=初期ポーズ再投影)。"
                 "座標系整合はユーザー目視で確認する")

    out = "\n".join(lines) + "\n"
    print(out)
    (OUT_DIR / "m4_stage0_result.txt").write_text(out)


if __name__ == "__main__":
    main()

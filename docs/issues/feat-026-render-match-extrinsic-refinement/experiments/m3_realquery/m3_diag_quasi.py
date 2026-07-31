"""feat-026 診断D1: cam05520129 準誤マッチの発生源特定

experiment_log.md「診断D1」の判定基準に基づき、M3 P0 収束解のポーズで反復1回分
(k≥1 条件: ゲート10px・RANSAC 2px)を再現し、ゲート通過マッチを
「インライア」「準誤マッチ(RANSAC外れ)」に分類して以下を測定する:
  1. 空間集中度 f_sp: 8×8 セル分割で準誤マッチ数上位16セル(面積25%)に入る割合
  2. 周期性署名 f_per: 変位ベクトル d=u_q−u_r の2Dヒストグラム(1pxビン)で
     最頻ビン±1px に入る割合と、ピーク変位量 |d_peak|
  3. 可視化: 実写画像にインライア(緑)・準誤マッチ(赤)を重畳した PNG
     (集中領域がブラインドと一致するかはユーザーが目視確認する)

対象: cam05520129 / 対照: cam41520557(M3 で高インライア率・収束したカメラ)

Stage 2〜6 の処理は m3_run.py の run_iteration_real と同一の定数・順序を用いる
(分類データを取り出すため関数化せず本ファイルに展開。差分は oracle 同様の
分類・集計出力のみで、パイプライン判断のロジックは変えないこと)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_diag_quasi.py

出力: m3_diag_result.txt(コミット対象) / m3_diag_<カメラ名>.png(ローカル)
"""
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
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, NEAR_PLANE,
)
from stage3_perturb_match import sample_depth_bilinear  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE,
    SIFT_RATIO,
)
from m3_run import CAMERAS, SEED_CV_BASE  # noqa: E402  # 本実験と同じ seed 式を使う

OUT_DIR = Path(__file__).resolve().parent
IMAGES_DIR = ROOT / "phase0" / "data" / "osaka2" / "images"
HISTORY_NPZ = OUT_DIR / "m3_history.npz"

TARGET_CAM = "cam05520129"
CONTROL_CAM = "cam41520557"
K_DIAG = 1          # k≥1 条件(ゲート10px・RANSAC 2px)で再現


def p0_seed(cam_name: str) -> int:
    """m3_run.py の P0 系列(si=0)と同一の cv2 RANSAC seed"""
    return SEED_CV_BASE + CAMERAS.index(cam_name) * 10 + 0
GRID = 8            # 8×8 セル
TOP_CELLS = 16      # 上位16セル = 面積25%
DISP_BIN_RANGE = 10  # 変位ヒストグラムの範囲 [-10,10]px(ゲート10pxに対応)


def classify_matches(gaussians, cam, img_q_gray, rvec, tvec):
    """m3_run.run_iteration_real の Stage 2〜6 を再現し、ゲート通過マッチを
    インライア/準誤マッチに分類して座標・変位を返す。"""
    p = sched(K_DIAG)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec
    cam_cur["tvec"] = tvec
    K, D = cam["K"], cam["D"]

    # Stage 2: レンダ(方式A)
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)
    img_r = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # Stage 3: SIFT マッチング
    sift = cv2.SIFT_create()
    kp_q, des_q = sift.detectAndCompute(img_q_gray, None)
    kp_r, des_r = sift.detectAndCompute(img_r, None)
    good = []
    if des_q is not None and des_r is not None and len(des_q) >= 2 and len(des_r) > 0:
        knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_r, des_q, k=2)
        good = [pr[0] for pr in knn
                if len(pr) == 2 and pr[0].distance < SIFT_RATIO * pr[1].distance]
    if len(good) == 0:
        raise RuntimeError("マッチ0件")
    u_r = np.array([kp_r[m.queryIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    u_q = np.array([kp_q[m.trainIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)

    # Stage 4: ゲート
    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r)
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    keep = ((var_map[iy, ix] < tau_d ** 2)
            & (np.linalg.norm(u_q - u_r, axis=1) < p["tau_px"])
            & (alpha[iy, ix] > GATE_ALPHA) & depth_ok
            & (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1]))
    if int(keep.sum()) < 4:
        raise RuntimeError(f"ゲート生存 {int(keep.sum())} 点で PnP 不能")

    # Stage 5: 深度3D化
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack(
        [u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec).reshape(1, 3)) @ R_cur

    # Stage 6: PnP-RANSAC(分類が目的なので精密化は行わない)
    u_q_keep = u_q[keep]
    ok, _, _, inl = cv2.solvePnPRansac(
        X_w, u_q_keep, K, D, iterationsCount=1000,
        reprojectionError=p["ransac_px"], flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None:
        raise RuntimeError("solvePnPRansac 失敗")
    inl_mask = np.zeros(len(u_q_keep), dtype=bool)
    inl_mask[inl.flatten()] = True

    disp = u_q_keep - u_r[keep]
    return {
        "u_q_inl": u_q_keep[inl_mask], "u_q_quasi": u_q_keep[~inl_mask],
        "disp_inl": disp[inl_mask], "disp_quasi": disp[~inl_mask],
        "n_gate": int(len(u_q_keep)), "n_inl": int(inl_mask.sum()),
    }


def spatial_concentration(u: np.ndarray, w: int, h: int) -> tuple[float, np.ndarray]:
    """8×8 セル分割で上位 TOP_CELLS セルに入る点の割合 f_sp を返す"""
    cx = (u[:, 0] / w * GRID).astype(int).clip(0, GRID - 1)
    cy = (u[:, 1] / h * GRID).astype(int).clip(0, GRID - 1)
    counts = np.zeros((GRID, GRID), dtype=int)
    np.add.at(counts, (cy, cx), 1)
    flat = np.sort(counts.ravel())[::-1]
    f_sp = float(flat[:TOP_CELLS].sum() / max(1, len(u)))
    return f_sp, counts


def periodicity(disp: np.ndarray) -> tuple[float, float]:
    """変位2Dヒストグラム(1pxビン)の最頻ビン±1pxに入る割合 f_per とピーク変位量 |d_peak|"""
    r = DISP_BIN_RANGE
    hist, xe, ye = np.histogram2d(disp[:, 0], disp[:, 1],
                                  bins=2 * r, range=[[-r, r], [-r, r]])
    pi, pj = np.unravel_index(np.argmax(hist), hist.shape)
    # 最頻ビン±1px = ヒストグラム上の 3×3 近傍
    i0, i1 = max(0, pi - 1), min(hist.shape[0], pi + 2)
    j0, j1 = max(0, pj - 1), min(hist.shape[1], pj + 2)
    f_per = float(hist[i0:i1, j0:j1].sum() / max(1, len(disp)))
    d_peak = np.array([(xe[pi] + xe[pi + 1]) / 2, (ye[pj] + ye[pj + 1]) / 2])
    return f_per, float(np.linalg.norm(d_peak))


def draw_overlay(img_gray: np.ndarray, res: dict, path: Path) -> None:
    """実写画像にインライア(緑)・準誤マッチ(赤)を重畳して保存"""
    vis = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    for x, y in res["u_q_inl"]:
        cv2.circle(vis, (int(round(x)), int(round(y))), 4, (0, 200, 0), 1)
    for x, y in res["u_q_quasi"]:
        cv2.circle(vis, (int(round(x)), int(round(y))), 4, (0, 0, 255), 2)
    cv2.imwrite(str(path), vis)


def main():
    hist_data = np.load(HISTORY_NPZ, allow_pickle=True)
    results = hist_data["results"][0]

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    cams_cfg = load_cameras_toml(TOML_PATH)

    lines = [
        "feat-026 診断D1: 準誤マッチの発生源特定(experiment_log.md 診断D1)",
        f"条件: k≥1(ゲート{sched(K_DIAG)['tau_px']:.0f}px・RANSAC {sched(K_DIAG)['ransac_px']:.0f}px) "
        f"/ ポーズ=M3 P0 収束解 / cv2 seed=m3_run.py P0 と同一式",
        f"判定基準: ブラインド起因支持 = f_sp≥50% かつ f_per≥25% かつ |d_peak|≥2px + 目視確認",
        "",
    ]
    for name in (TARGET_CAM, CONTROL_CAM):
        cam = select_camera(cams_cfg, name)
        p0 = results[name]["series"]["P0"]
        img_q_gray = cv2.imread(str(IMAGES_DIR / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        if img_q_gray is None:
            raise FileNotFoundError(f"クエリ画像がない: {name}")
        seed = p0_seed(name)
        cv2.setRNGSeed(seed)
        res = classify_matches(gaussians, cam, img_q_gray, p0["rvec"], p0["tvec"])
        n_quasi = res["n_gate"] - res["n_inl"]
        f_sp, _ = spatial_concentration(res["u_q_quasi"], cam["width"], cam["height"])
        f_per, d_peak = periodicity(res["disp_quasi"])
        med_inl_disp = float(np.median(np.linalg.norm(res["disp_inl"], axis=1)))
        med_quasi_disp = float(np.median(np.linalg.norm(res["disp_quasi"], axis=1)))
        png = OUT_DIR / f"m3_diag_{name}.png"
        draw_overlay(img_q_gray, res, png)
        role = "対象" if name == TARGET_CAM else "対照"
        lines += [
            f"[{name}]({role} / cv2 seed={seed}) ゲート通過 {res['n_gate']} = "
            f"インライア {res['n_inl']} + 準誤マッチ {n_quasi}(比率 {n_quasi/res['n_gate']*100:.1f}%)",
            f"  f_sp(上位16/64セル集中度) = {f_sp*100:.1f}%(基準50%)",
            f"  f_per(最頻変位±1px集中度) = {f_per*100:.1f}%(基準25%) |d_peak| = {d_peak:.1f}px(基準2px)",
            f"  変位中央値: インライア {med_inl_disp:.2f}px / 準誤マッチ {med_quasi_disp:.2f}px",
            f"  重畳画像: {png.name}(緑=インライア, 赤=準誤マッチ。目視確認用)",
            "",
        ]
        print("\n".join(lines[-6:]))

    (OUT_DIR / "m3_diag_result.txt").write_text("\n".join(lines) + "\n")
    print(f"保存: {OUT_DIR / 'm3_diag_result.txt'}")


if __name__ == "__main__":
    main()

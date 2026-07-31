"""feat-026 診断D2: cam41520554 の2アトラクタの機構調査

experiment_log.md「診断D2」の固定条件に基づき、以下を測定する:
  1. θ_axis: 2アトラクタ中心の変位ベクトル ΔC と光軸方向のなす角(既存データの計算のみ)
  2. アトラクタ別診断値: 各アトラクタ中心にポーズを固定し、k=1 条件(ゲート10px・
     RANSAC 2px)の反復を seed {7000..7009} の10通りで実行(ポーズ更新なし)。
     N・インライア数・RMSE・退化診断を記録し、RMSE は中央値同士の差で比較
  3. Jaccard: 同一 seed の A/B インライア集合を「クエリ座標 u_q の距離≤2px の
     相互最近傍」で対応付け、|A∩B|/|A∪B|。10 seed の中央値

アトラクタ定義(experiment_log.md 固定):
  A = 554 の P0/P1 の pooled クラスタ中心(m3_1_history.npz の40サンプルから算出)
  B = 554 の P2 のクラスタ中心(M3-1 保存値)

反復処理(Stage 2〜6)は m3_run.run_iteration_real と同一の定数・順序。
インライアのクエリ座標と RMSE を返す必要があるため本ファイルに展開する
(差分は返却値の追加のみ。パイプライン判断のロジックは変えないこと)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d2_attractor.py

出力: d2_result.txt(コミット対象)
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
A1_DIR = Path(__file__).resolve().parents[1] / "a1_synthetic"
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(A1_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, NEAR_PLANE,
)
from stage3_perturb_match import sample_depth_bilinear  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE, SIFT_RATIO, N_MIN,
)
from m3_1_sampling import cluster_analysis, center_of, rot_of  # noqa: E402
from m3_run import IMAGES_DIR  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
HISTORY_NPZ = OUT_DIR / "m3_1_history.npz"

CAM_NAME = "cam41520554"
SEEDS = list(range(7000, 7010))     # 固定 seed リスト(experiment_log.md)
K_DIAG = 1                          # k=1 条件
JACCARD_R_PX = 2.0                  # u_q 相互最近傍の対応半径
# 判定閾値(experiment_log.md 診断D2)
TH_AXIS_SUPPORT = 30.0              # θ_axis ≤ 30° で深度弱拘束を支持
TH_AXIS_REJECT = 60.0               # ≥ 60° で不支持
TH_RMSE_DIFF = 0.3                  # RMSE 中央値差 < 0.3px で「等価解」
TH_JACCARD = 0.5                    # Jaccard 中央値 < 0.5 で「別部分集合」


def iterate_classify(gaussians, cam, img_q_gray, rvec, tvec):
    """k=1 条件の反復1回分(m3_run.run_iteration_real と同一処理)。
    ポーズは更新せず、診断値とインライアのクエリ座標を返す。"""
    p = sched(K_DIAG)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec
    cam_cur["tvec"] = tvec
    K, D = cam["K"], cam["D"]

    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)
    img_r = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

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
    N = int(keep.sum())
    if N < N_MIN:
        raise RuntimeError(f"ゲート生存 N={N} < {N_MIN}")

    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack(
        [u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec).reshape(1, 3)) @ R_cur

    u_q_keep = u_q[keep]
    ok, rv, tv, inl = cv2.solvePnPRansac(
        X_w, u_q_keep, K, D, iterationsCount=1000,
        reprojectionError=p["ransac_px"], flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < 4:
        raise RuntimeError("solvePnPRansac 失敗")
    inl = inl.flatten()
    ok, rv, tv = cv2.solvePnP(
        X_w[inl], u_q_keep[inl], K, D, rvec=rv, tvec=tv,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP 精密化失敗")

    Xi = X_w[inl]
    eig = np.sort(np.linalg.eigvalsh(np.cov(Xi.T)))[::-1]
    proj, _ = cv2.projectPoints(Xi, np.asarray(rv).reshape(3, 1),
                                np.asarray(tv).reshape(3, 1), K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_q_keep[inl], axis=1)
    return {
        "N": N, "inliers": int(len(inl)),
        "rmse": float(np.sqrt(np.mean(resid ** 2))),
        "vol_ratio": float(eig[2] / eig[0]),
        "depth_range": float(d_bilin[keep][inl].max() - d_bilin[keep][inl].min()),
        "u_q_inl": u_q_keep[inl],
    }


def mutual_nn_jaccard(uA: np.ndarray, uB: np.ndarray) -> float:
    """u_q 座標の相互最近傍(距離≤JACCARD_R_PX)で対応付けた Jaccard = |A∩B|/|A∪B|"""
    if len(uA) == 0 or len(uB) == 0:
        return 0.0
    dist = np.linalg.norm(uA[:, None, :] - uB[None, :, :], axis=2)
    nnA = dist.argmin(axis=1)   # A→B 最近傍
    nnB = dist.argmin(axis=0)   # B→A 最近傍
    matched = sum(1 for i, j in enumerate(nnA)
                  if nnB[j] == i and dist[i, j] <= JACCARD_R_PX)
    return matched / (len(uA) + len(uB) - matched)


def main():
    hist = np.load(HISTORY_NPZ, allow_pickle=True)
    res554 = hist["results"][0][CAM_NAME]

    # アトラクタ A: P0/P1 の pooled クラスタ中心 / B: P2 のクラスタ中心(保存値)
    pooled_ab = list(res554["P0"]["samples"]) + list(res554["P1"]["samples"])
    _, f_c_A, _, (rvec_A, tvec_A), _, _ = cluster_analysis(pooled_ab)
    rvec_B, tvec_B = res554["P2"]["rvec"], res554["P2"]["tvec"]

    C_A, C_B = center_of(rvec_A, tvec_A), center_of(rvec_B, tvec_B)
    dC = C_B - C_A
    axis_w = rot_of(rvec_A)[2, :]  # 光軸方向(world)
    cos_axis = abs(float(np.dot(dC, axis_w)) / np.linalg.norm(dC))
    theta_axis = float(np.degrees(np.arccos(np.clip(cos_axis, 0.0, 1.0))))

    lines = [
        "feat-026 診断D2: cam41520554 の2アトラクタの機構調査(experiment_log.md 診断D2)",
        f"A = P0/P1 pooled クラスタ中心(f_c={f_c_A:.2f}) / B = P2 クラスタ中心",
        f"|ΔC| = {np.linalg.norm(dC)*100:.3f}cm",
        "",
        f"測定1: θ_axis = {theta_axis:.1f}deg "
        f"(≤{TH_AXIS_SUPPORT}° で深度弱拘束を支持 / ≥{TH_AXIS_REJECT}° で不支持)",
        "",
    ]
    print("\n".join(lines))

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    cams_cfg = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams_cfg, CAM_NAME)
    img_q = cv2.imread(str(IMAGES_DIR / f"{CAM_NAME}.png"), cv2.IMREAD_GRAYSCALE)
    if img_q is None:
        raise FileNotFoundError("クエリ画像がない")

    rows = {"A": [], "B": []}
    jaccards = []
    lines.append("測定2/3: seed別の診断値(k=1条件・ポーズ固定)")
    lines.append("seed | A: inl/N RMSE vol zrange | B: inl/N RMSE vol zrange | Jaccard")
    for seed in SEEDS:
        per = {}
        for tag, (rv, tv) in (("A", (rvec_A, tvec_A)), ("B", (rvec_B, tvec_B))):
            cv2.setRNGSeed(seed)
            per[tag] = iterate_classify(gaussians, cam, img_q,
                                        np.asarray(rv, dtype=np.float64).flatten(),
                                        np.asarray(tv, dtype=np.float64).flatten())
            rows[tag].append(per[tag])
        jac = mutual_nn_jaccard(per["A"]["u_q_inl"], per["B"]["u_q_inl"])
        jaccards.append(jac)
        line = (f"{seed} | A: {per['A']['inliers']}/{per['A']['N']} "
                f"{per['A']['rmse']:.3f}px {per['A']['vol_ratio']:.3f} {per['A']['depth_range']:.2f}m"
                f" | B: {per['B']['inliers']}/{per['B']['N']} "
                f"{per['B']['rmse']:.3f}px {per['B']['vol_ratio']:.3f} {per['B']['depth_range']:.2f}m"
                f" | {jac:.3f}")
        print(line)
        lines.append(line)

    med = lambda xs: float(np.median(xs))  # noqa: E731
    rmse_A = med([r["rmse"] for r in rows["A"]])
    rmse_B = med([r["rmse"] for r in rows["B"]])
    jac_med = med(jaccards)
    rmse_diff = abs(rmse_A - rmse_B)
    lines += [
        "",
        f"RMSE 中央値: A={rmse_A:.3f}px B={rmse_B:.3f}px 差={rmse_diff:.3f}px "
        f"(<{TH_RMSE_DIFF}px で「残差で区別できない等価解」を支持)",
        f"Jaccard 中央値: {jac_med:.3f} (<{TH_JACCARD} で「別のマッチ部分集合が支える」を支持)",
        "",
        "=== 判定(experiment_log.md 診断D2 の事前基準) ===",
        f"深度弱拘束(θ_axis≤{TH_AXIS_SUPPORT}°): {'支持' if theta_axis <= TH_AXIS_SUPPORT else ('不支持' if theta_axis >= TH_AXIS_REJECT else '中間(判定保留)')}",
        f"等価解(RMSE差<{TH_RMSE_DIFF}px): {'支持' if rmse_diff < TH_RMSE_DIFF else '不支持(どちらかが良い)'}",
        f"別部分集合(Jaccard<{TH_JACCARD}): {'支持' if jac_med < TH_JACCARD else '不支持(同じマッチが支える)'}",
    ]
    print("\n".join(lines[-8:]))
    (OUT_DIR / "d2_result.txt").write_text("\n".join(lines) + "\n")
    print(f"保存: {OUT_DIR / 'd2_result.txt'}")


if __name__ == "__main__":
    main()

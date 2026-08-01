"""feat-026 M4 本実験: 檻環境パイロット(criteria.md locked 2026-08-01)

criteria.md §5〜§7 に基づき、パイロットカメラ cam05520125 で
「6点PnP初期値 + サンプリング型リファイン + チェーン統合合意」を実行し判定する:
  1. 6点PnP(K既知)で初期ポーズ算出(§5-1。Stage 0 と同一経路)
  2. 初期値3系統(A=無摂動, B/C=5cm/2°摂動 seed 11/12)の各チェーンで
     k=0 → k≥1 条件で N_S=20 回サンプリング(§5-2。M3-1 と同一手順)
  3. 3チェーン60サンプル pooled の合意クラスタ分析(§5-3。M3-2 と同一ロジック=
     m3_1_sampling.cluster_analysis を import)
  4. 受理判定(§6.2): 単峰高合意 or 数値固定条件を満たす仲裁成立の二峰
  5. 総合判定の前提条件(§6.4): 初期解→最終解の差 ≤ 50cm/10°
  6. (b-1)〜(b-4) の実測を全反復で記録(§6.3。合否には使わない)

反復処理は m3_run.run_iteration_real と同一の処理・定数・順序を保つこと。
(b-1)/(b-2)/(b-3) の段別内訳を記録するため、run_iteration_real_m4 として
本ファイルに展開する(差分は診断カウントの記録のみで、パイプライン判断の
ロジック・数値・順序は一切変えないこと。m3_diag_quasi.py と同じ流儀)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py

出力: m4_result.txt(コミット対象) / m4_history.npz・m4_final_overlay.png(ローカル)
"""
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
A1_DIR = Path(__file__).resolve().parents[1] / "a1_synthetic"
M3_DIR = Path(__file__).resolve().parents[1] / "m3_realquery"
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(A1_DIR))
sys.path.insert(0, str(M3_DIR))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import render_depth_alpha_distorted, NEAR_PLANE  # noqa: E402
from stage3_perturb_match import sample_depth_bilinear, perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402  # M2/M3 と同一の確定値(criteria.md §4)
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE,
    SIFT_RATIO, N_MIN,
)
from m3_run import pose_diff  # noqa: E402
from m3_1_sampling import (  # noqa: E402  # M3-1/M3-2 と同一ロジック(criteria.md §5-3)
    cluster_analysis, N_S, R_POS, R_ANG, F_C_MIN, F2_BIMODAL, SEED_SAMP_BASE,
)

OUT_DIR = Path(__file__).resolve().parent
DATA_DIR = OUT_DIR / "data"
PLY_PATH = DATA_DIR / "point_cloud.ply"
INTRINSICS_PATH = DATA_DIR / "intrinsics_all.toml"
IMAGE_PATH = DATA_DIR / "images" / "cam05520125.png"
POINTS_3D_PATH = DATA_DIR / "kijunten_locations.csv"
POINTS_2D_PATH = DATA_DIR / "points_2d.csv"

CAMERA = "cam05520125"      # criteria.md §2
# criteria.md §5-2: チェーン定義(B/C は M3 の P1/P2 と同方式・同 seed)
CHAINS = [("A", None, None, None), ("B", 0.05, 2.0, 11), ("C", 0.05, 2.0, 12)]
# criteria.md §6.2-2: 仲裁成立の二峰の数値固定条件
F12_MIN = 0.9           # 第1+第2 合計占有率の下限(=54/60)
TH_ARBITRATE_PX = 1.0   # 仲裁: 再投影誤差中央値の差の閾値(D3 と同値)
# criteria.md §6.4: 総合判定の前提条件(M2 実測収束域)
PRECOND_POS_M = 0.50
PRECOND_ANG_DEG = 10.0


def load_matched_points(cam_name: str):
    """ObjectName で 3D-2D をマッチ(m4_stage0.py と同一)"""
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
    """6点PnP(criteria.md §5-1。m4_stage0.py と同一)"""
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
    return rvec, tvec


def run_iteration_real_m4(gaussians, cam, img_q_gray, rvec_cur, tvec_cur, k):
    """反復1回分(Stage 2〜6)。m3_run.run_iteration_real と同一処理+診断記録のみ追加。

    追加した記録(diag): kp_q/kp_r(SIFT キーポイント数)・alpha_cov(有効画素率)・
    ゲート段別内訳(n_disp/n_var/n_alpha/n_depth_ok/n_z)・除外内訳(excl_*)・
    準誤マッチ数 quasi(=ゲート通過 N − RANSAC インライア)。
    パイプライン判断のロジック・数値・順序は run_iteration_real と同一に保つこと。
    """
    p = sched(k)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur
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
    n_raw = 0
    if des_q is not None and des_r is not None and len(des_q) >= 2 and len(des_r) > 0:
        knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_r, des_q, k=2)
        n_raw = len(knn)
        good = [pr[0] for pr in knn
                if len(pr) == 2 and pr[0].distance < SIFT_RATIO * pr[1].distance]
    u_r = np.array([kp_r[m.queryIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    u_q = np.array([kp_q[m.trainIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    diag = {"k": k, "tau_px": p["tau_px"], "ransac_px": p["ransac_px"],
            # 記録のみ(b-1): 逐次通過数 n_raw(生候補)→matches(ratio後)。
            # 以降 n_disp(変位)→N(深度系)→inliers(RANSAC)を順次記録する
            "n_raw": n_raw,
            "matches": len(good),
            # 記録のみ(b-1/b-4): SIFT キーポイント数と有効画素率
            "kp_q": len(kp_q) if kp_q is not None else 0,
            "kp_r": len(kp_r) if kp_r is not None else 0,
            "alpha_cov": float((alpha > GATE_ALPHA).mean())}
    if len(good) == 0:
        diag["fail"] = "マッチ0件"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 4: ゲート
    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r)
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    cond_var = var_map[iy, ix] < tau_d ** 2
    cond_disp = np.linalg.norm(u_q - u_r, axis=1) < p["tau_px"]
    cond_alpha = alpha[iy, ix] > GATE_ALPHA
    cond_depth = depth_ok
    cond_z = (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1])
    keep = cond_var & cond_disp & cond_alpha & cond_depth & cond_z
    N = int(keep.sum())
    diag["N"] = N
    # 記録のみ(b-1/b-2): 逐次通過数(変位→深度系)と、変位通過後の深度系除外内訳
    # (条件は上の keep と同一の配列を集計。keep の計算・判定は変えない)
    # 分類順は depth_ok → α → z → 分散: cond_var は pixel_valid(α・z を含む)に
    # 依存するため、α/z 不良を先に分類しないと分散除外に吸い込まれて内訳が歪む
    diag.update({
        "n_disp": int(cond_disp.sum()),  # 変位ゲート通過(逐次: matches → n_disp)
        # 深度系ゲート(depth_ok/α/z/分散)をすべて通過 = N(逐次: n_disp → N)
        "excl_depth_nan": int((cond_disp & ~cond_depth).sum()),
        "excl_alpha": int((cond_disp & cond_depth & ~cond_alpha).sum()),
        "excl_z": int((cond_disp & cond_depth & cond_alpha & ~cond_z).sum()),
        "excl_var": int((cond_disp & cond_depth & cond_alpha & cond_z & ~cond_var).sum()),
    })
    if N < N_MIN:
        diag["fail"] = f"ゲート生存 N={N} < {N_MIN}"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 5: 深度3D化
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack(
        [u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec_cur).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_cur).reshape(1, 3)) @ R_cur

    # Stage 6: PnP-RANSAC + 精密化
    u_q_keep = u_q[keep]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        X_w, u_q_keep, K, D, iterationsCount=1000,
        reprojectionError=p["ransac_px"], flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < 4:
        diag["fail"] = "solvePnPRansac 失敗"
        return False, (rvec_cur, tvec_cur), diag
    inl = inl.flatten()
    ok, rvec, tvec = cv2.solvePnP(
        X_w[inl], u_q_keep[inl], K, D, rvec=rvec, tvec=tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        diag["fail"] = "solvePnP 精密化失敗"
        return False, (rvec_cur, tvec_cur), diag
    rvec = np.asarray(rvec).flatten()
    tvec = np.asarray(tvec).flatten()

    # 診断値(m3_run と同一)+ 準誤マッチ数(b-3。記録のみ)
    Xi = X_w[inl]
    eig = np.sort(np.linalg.eigvalsh(np.cov(Xi.T)))[::-1]
    proj, _ = cv2.projectPoints(Xi, rvec.reshape(3, 1), tvec.reshape(3, 1), K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_q_keep[inl], axis=1)
    hull = cv2.convexHull(u_q_keep[inl].astype(np.float32))
    diag.update({
        "inliers": int(len(inl)),
        "quasi": int(N - len(inl)),
        "rmse": float(np.sqrt(np.mean(resid ** 2))),
        "vol_ratio": float(eig[2] / eig[0]),
        "depth_range": float(d_bilin[keep][inl].max() - d_bilin[keep][inl].min()),
        "area_ratio": float(cv2.contourArea(hull) / (cam["width"] * cam["height"])),
    })
    return True, (rvec, tvec), diag


def run_sampling_chain(gaussians, cam, img_q_gray, rvec0, tvec0, seed_series, lines):
    """1チェーン: k=0 → k≥1 条件で N_S 回サンプリング(m3_1_sampling.run_sampling_series
    と同一手順。反復関数を run_iteration_real_m4 に差し替え、診断 diag を全反復分保存)"""
    rvec_cur = np.asarray(rvec0, dtype=np.float64).flatten()
    tvec_cur = np.asarray(tvec0, dtype=np.float64).flatten()
    diags = []
    cv2.setRNGSeed(seed_series)  # j=0(k=0 反復)
    ok, (rvec_cur, tvec_cur), diag = run_iteration_real_m4(
        gaussians, cam, img_q_gray, rvec_cur, tvec_cur, 0)
    diags.append(diag)
    if not ok:
        lines.append(f"  j=0(k=0): 失敗({diag.get('fail')}) → 明示的失敗")
        return {"accepted": False, "fail": f"k=0: {diag.get('fail')}", "diags": diags}
    lines.append(f"  j=0(k=0): マッチ{diag['matches']} N={diag['N']} "
                 f"inl={diag['inliers']} RMSE={diag['rmse']:.3f}px")
    samples = []
    for j in range(1, N_S + 1):
        cv2.setRNGSeed(seed_series + j)
        ok, (rvec_new, tvec_new), diag = run_iteration_real_m4(
            gaussians, cam, img_q_gray, rvec_cur, tvec_cur, 1)
        diags.append(diag)
        if not ok:
            lines.append(f"  j={j}: 失敗({diag.get('fail')}) → 明示的失敗")
            return {"accepted": False, "fail": f"j={j}: {diag.get('fail')}", "diags": diags}
        rvec_cur, tvec_cur = rvec_new, tvec_new
        samples.append((rvec_cur.copy(), tvec_cur.copy()))
    rmses = [d["rmse"] for d in diags[1:]]
    quasis = [d["quasi"] / d["N"] for d in diags[1:]]
    lines.append(f"  j=1..{N_S}: RMSE {min(rmses):.3f}〜{max(rmses):.3f}px "
                 f"準誤マッチ率 {min(quasis)*100:.1f}〜{max(quasis)*100:.1f}%")
    return {"accepted": True, "samples": samples, "diags": diags}


def append_b_metrics(lines, chain_results):
    """§6.3 (b-1)〜(b-4) 集計(k=0 と k≥1 を分けて記録。記録のみ、合否に使わない)。
    チェーンが途中失敗していても取得済み diags の範囲で出力する"""
    lines.append("")
    k0 = [r["diags"][0] for r in chain_results if r.get("diags")]
    k1 = [d for r in chain_results if r.get("diags") for d in r["diags"][1:]]
    for tag, dd, n_expect in [("k=0", k0, 3), ("k≥1", k1, 3 * N_S)]:
        lines.append(f"§6.3 実測 {tag}({len(dd)}/{n_expect} 反復"
                     + ("、未実行分あり" if len(dd) < n_expect else "") + "):")
        if not dd:
            lines.append("  (取得データなし)")
            continue
        for key, label in [
                ("n_raw", "(b-1) 生マッチ候補"), ("matches", "(b-1) ratio後"),
                ("n_disp", "(b-1) 変位ゲート後"), ("N", "(b-1) 深度系ゲート後 N"),
                ("inliers", "(b-1) RANSAC インライア")]:
            vals = [d[key] for d in dd if key in d]
            if vals:
                lines.append(f"  {label}: {min(vals)}〜{max(vals)}"
                             f"(中央値 {int(np.median(vals))})")
        for key, label in [("excl_depth_nan", "(b-2) 深度NaN/広がり除外"),
                           ("excl_alpha", "(b-2) α除外"), ("excl_z", "(b-2) z範囲除外"),
                           ("excl_var", "(b-2) 深度分散除外")]:
            vals = [d[key] / d["n_disp"] for d in dd if key in d and d.get("n_disp", 0) > 0]
            if vals:
                lines.append(f"  {label}(変位通過分母): "
                             f"{min(vals)*100:.1f}〜{max(vals)*100:.1f}%")
        quasis = [d["quasi"] / d["N"] for d in dd if "quasi" in d]
        if quasis:
            lines.append(f"  (b-3) 準誤マッチ率: {min(quasis)*100:.1f}〜{max(quasis)*100:.1f}%"
                         f"(中央値 {np.median(quasis)*100:.1f}%。M3 参照値 18〜40%)")
        rmses = [d["rmse"] for d in dd if "rmse" in d]
        if rmses:
            lines.append(f"  RMSE: {min(rmses):.3f}〜{max(rmses):.3f}px"
                         f"(警告閾値 2.0px。M3 参照値 0.90〜1.24px)")
        for key, th in [("vol_ratio", 0.09), ("depth_range", 1.4), ("area_ratio", 0.22)]:
            vals = [d[key] for d in dd if key in d]
            if vals:
                n_warn = sum(v < th for v in vals)
                lines.append(f"  {key}: {min(vals):.3f}〜{max(vals):.3f}"
                             f"(警告閾値 {th}: {n_warn}/{len(vals)} 反復で点灯。記録のみ)")
        covs = [d["alpha_cov"] for d in dd if "alpha_cov" in d]
        kprs = [d["kp_r"] / d["kp_q"] for d in dd if d.get("kp_q", 0) > 0]
        if covs:
            lines.append(f"  (b-4) レンダ有効画素率: {min(covs):.3f}〜{max(covs):.3f}")
        if kprs:
            lines.append(f"  (b-4) SIFT数比(レンダ/実写): {min(kprs):.3f}〜{max(kprs):.3f}")


def median_reproj_px(p3, p2, rvec, tvec, K, D) -> float:
    """手動6点の再投影誤差の中央値(criteria.md §5-4 の仲裁指標)"""
    proj, _ = cv2.projectPoints(p3, np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                                np.asarray(tvec, dtype=np.float64).reshape(3, 1), K, D)
    return float(np.median(np.linalg.norm(proj.reshape(-1, 2) - p2, axis=1)))


def main():
    lines = [
        "M4 本実験: 檻環境パイロット(criteria.md locked 2026-08-01)",
        f"シーン: {PLY_PATH}",
        f"クエリ: {IMAGE_PATH}",
        f"near_plane={NEAR_PLANE} / distort=True / N_S={N_S} "
        f"r_pos={R_POS*100:.0f}cm r_ang={R_ANG}deg f_c≥{F_C_MIN}",
        "",
    ]

    cams = load_cameras_toml(str(INTRINSICS_PATH))
    cam = select_camera(cams, CAMERA)
    img_q = cv2.imread(str(IMAGE_PATH))
    if img_q is None:
        raise SystemExit(f"クエリ画像を読めません: {IMAGE_PATH}")
    if img_q.shape[:2] != (cam["height"], cam["width"]):
        raise SystemExit("画像サイズが TOML と不一致")
    img_q_gray = cv2.cvtColor(img_q, cv2.COLOR_BGR2GRAY)
    p3, p2, names = load_matched_points(CAMERA)
    if len(names) != 6:
        lines.append(f"手動点数が criteria §5-1(6点PnP)と不一致: {len(names)}点")
        (OUT_DIR / "m4_result.txt").write_text("\n".join(lines) + "\n")
        raise SystemExit(f"手動点が6点ちょうどではありません: {len(names)}点")

    # §5-1: 6点PnP 初期値
    res = pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res is None:
        (OUT_DIR / "m4_result.txt").write_text("\n".join(lines) + "\n")
        raise SystemExit("6点PnP 失敗(§6.1 不成立。m4_result.txt に記録)")
    rvec0, tvec0 = res

    print("PLY読み込み中...")
    gaussians = load_ply(str(PLY_PATH))

    # §5-2: 3チェーンのサンプリング
    chain_results = []
    all_samples = []
    for ci, (cname, dpos, dang, seed) in enumerate(CHAINS):
        if dpos is None:
            rv, tv = rvec0.copy(), tvec0.copy()
            lines.append(f"チェーン {cname}: 初期値=6点PnP解(無摂動)")
        else:
            rng = np.random.default_rng(seed)
            rv, tv = perturb_pose(rvec0, tvec0, dpos, dang, rng)
            dp, da = pose_diff(rv, tv, rvec0, tvec0)
            lines.append(f"チェーン {cname}: 初期値=6点PnP解+摂動 {dpos*100:.0f}cm/{dang}° "
                         f"(seed={seed}, 実摂動 {dp*100:.2f}cm/{da:.2f}°)")
        seed_series = SEED_SAMP_BASE + ci * 25
        print(f"チェーン {cname} 実行中(seed_series={seed_series})...")
        r = run_sampling_chain(gaussians, cam, img_q_gray, rv, tv, seed_series, lines)
        r["name"] = cname
        chain_results.append(r)
        if r["accepted"]:
            all_samples.extend(r["samples"])

    # §6.1 成立性: 3チェーンすべてで60サンプル収集
    est_ok = all(r["accepted"] for r in chain_results)
    lines.append("")
    lines.append(f"§6.1 成立性: {'成立' if est_ok else '不成立'}"
                 f"(チェーン成立 {sum(r['accepted'] for r in chain_results)}/3, "
                 f"サンプル {len(all_samples)}/{3*N_S})")
    if not est_ok:
        for r in chain_results:
            if not r["accepted"]:
                lines.append(f"  チェーン {r['name']} 失敗段: {r['fail']}")
        append_b_metrics(lines, chain_results)  # No-Go 時の失敗モード切り分け材料(§7)
        lines.append("")
        lines.append("§7 判定: No-Go(成立性不成立。失敗段は上記)")
        out = "\n".join(lines) + "\n"
        print(out)
        (OUT_DIR / "m4_result.txt").write_text(out)
        np.savez(OUT_DIR / "m4_history.npz",
                 results=np.array([chain_results], dtype=object),
                 rvec0=rvec0, tvec0=tvec0)
        return

    # §5-3: pooled 合意クラスタ分析(M3-2 と同一ロジック)
    cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(all_samples)
    n_pool = len(all_samples)
    lines.append("")
    lines.append(f"§5-3 pooled 合意: f_c={f_c:.3f}({len(cluster)}/{n_pool}) "
                 f"第2クラスタ={f2:.3f} RMS半径={rms_pos*1000:.2f}mm/{rms_ang:.4f}deg")

    # §6.2 受理判定
    accepted = False
    accept_mode = ""
    rvec_sol, tvec_sol = rvec_f, tvec_f
    if f_c >= F_C_MIN and f2 < F2_BIMODAL:
        accepted = True
        accept_mode = f"単峰高合意(f_c={f_c:.3f}≥{F_C_MIN}, 第2={f2:.3f}<{F2_BIMODAL})"
    elif f2 >= F2_BIMODAL:
        # 二峰: 第2クラスタの解を求めて仲裁(criteria.md §5-4, §6.2-2)
        rest = [all_samples[i] for i in range(n_pool)
                if i not in set(cluster.tolist())]
        c2, f_c2_local, _, (rvec_g, tvec_g), _, _ = cluster_analysis(rest)
        f12 = f_c + len(c2) / n_pool
        med_f = median_reproj_px(p3, p2, rvec_f, tvec_f, cam["K"], cam["D"])
        med_g = median_reproj_px(p3, p2, rvec_g, tvec_g, cam["K"], cam["D"])
        diff = abs(med_f - med_g)
        d_pk, a_pk = pose_diff(rvec_f, tvec_f, rvec_g, tvec_g)
        lines.append(f"二峰分類: 第1={f_c:.3f} 第2={len(c2)/n_pool:.3f} 合計={f12:.3f} "
                     f"峰間距離={d_pk*100:.2f}cm/{a_pk:.3f}°")
        lines.append(f"仲裁(手動6点再投影中央値): 第1峰={med_f:.3f}px 第2峰={med_g:.3f}px "
                     f"差={diff:.3f}px(閾値{TH_ARBITRATE_PX}px)")
        if f12 >= F12_MIN and diff >= TH_ARBITRATE_PX:
            accepted = True
            if med_g < med_f:
                rvec_sol, tvec_sol = rvec_g, tvec_g
                accept_mode = f"仲裁成立の二峰(第2峰を採用, 差={diff:.3f}px)"
            else:
                accept_mode = f"仲裁成立の二峰(第1峰を採用, 差={diff:.3f}px)"
        elif f12 < F12_MIN:
            accept_mode = f"不受理(二峰だが合計占有率 {f12:.3f} < {F12_MIN})"
        else:
            accept_mode = f"不受理(仲裁不成立: 差 {diff:.3f}px < {TH_ARBITRATE_PX})"
    else:
        accept_mode = f"不受理(f_c={f_c:.3f} < {F_C_MIN} の非二峰)"
    lines.append(f"§6.2 受理: {'受理' if accepted else '不受理'} — {accept_mode}")

    # §6.4 総合判定の前提条件
    d_init, a_init = pose_diff(rvec_sol, tvec_sol, rvec0, tvec0)
    precond_ok = (d_init <= PRECOND_POS_M) and (a_init <= PRECOND_ANG_DEG)
    lines.append(f"§6.4 初期解→最終解の差: {d_init*100:.2f}cm/{a_init:.3f}° "
                 f"(前提条件 ≤{PRECOND_POS_M*100:.0f}cm/{PRECOND_ANG_DEG}° → "
                 f"{'満足' if precond_ok else '違反'})")
    lines.append("  参考: M1 実測の6点PnP初期誤差は良好データ中央値 3〜5cm/1°未満")

    # 最終解の記録と手動点整合(記録のみ)
    med_sol = median_reproj_px(p3, p2, rvec_sol, tvec_sol, cam["K"], cam["D"])
    med_init = median_reproj_px(p3, p2, rvec0, tvec0, cam["K"], cam["D"])
    C_sol = -cv2.Rodrigues(np.asarray(rvec_sol).reshape(3, 1))[0].T \
        @ np.asarray(tvec_sol).reshape(3)
    lines.append("")
    lines.append(f"最終解: rvec={np.asarray(rvec_sol).flatten().tolist()}")
    lines.append(f"        tvec={np.asarray(tvec_sol).flatten().tolist()}")
    lines.append(f"        カメラ中心={C_sol.tolist()}")
    lines.append(f"手動6点再投影中央値: 初期解={med_init:.3f}px 最終解={med_sol:.3f}px"
                 "(記録のみ。合否に使わない)")

    # §6.3 (b-1)〜(b-4) 集計(関数化。No-Go パスと共通)
    append_b_metrics(lines, chain_results)

    # §7 Go/No-Go
    go = est_ok and accepted and precond_ok
    lines.append("")
    lines.append(f"§7 判定: {'Go' if go else 'No-Go'}"
                 f"(成立性={'○' if est_ok else '×'} 受理={'○' if accepted else '×'} "
                 f"前提条件={'○' if precond_ok else '×'})")

    # b-4: 最終解での重畳PNG(目視記録)
    cam_r = dict(cam)
    cam_r["rvec"] = np.asarray(rvec_sol).flatten()
    cam_r["tvec"] = np.asarray(tvec_sol).flatten()
    bgr = render_image(gaussians, cam_r, near_plane=NEAR_PLANE, distort=True)
    overlay = cv2.addWeighted(img_q, 0.5, bgr, 0.5, 0)
    cv2.imwrite(str(OUT_DIR / "m4_final_overlay.png"), overlay)
    lines.append("最終解の重畳PNG: m4_final_overlay.png")

    out = "\n".join(lines) + "\n"
    print(out)
    (OUT_DIR / "m4_result.txt").write_text(out)
    np.savez(OUT_DIR / "m4_history.npz",
             results=np.array([chain_results], dtype=object),
             rvec0=rvec0, tvec0=tvec0,
             rvec_sol=np.asarray(rvec_sol).flatten(),
             tvec_sol=np.asarray(tvec_sol).flatten())


if __name__ == "__main__":
    main()

"""feat-026 M3: ドメインギャップ検証(実写クエリ)本実験

criteria.md(locked 2026-07-31)に基づき、実写クエリで反復パイプライン(Stage 2〜7 相当)を
実行し、成立性(§5.1)・同一解(§5.2)・一致性(§5.3)・手動点再投影整合(§5.4)・
診断値(§5.5)を記録して Go/No-Go(§6)を判定する。

パイプライン定数(SCHEDULE・ゲート・収束判定)と部品関数は a1_synthetic の実装から
import して M2 との同一性を保証する。反復1回分のみ、oracle 採点(真値既知の合成実験
専用)が実写では不可能なため、pipeline_loop.run_iteration から oracle 部分を除いた
run_iteration_real を本ファイルに持つ(それ以外の処理は同一)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_run.py

出力: このスクリプトと同じディレクトリに m3_result.txt(コミット対象) / m3_history.npz(ローカル)
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
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, NEAR_PLANE,
)
from stage3_perturb_match import sample_depth_bilinear, perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402  # M2 と同一の確定値(criteria.md §4)
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE,
    SIFT_RATIO, N_MIN, EPS_ROT_DEG, EPS_POS_M,
)

OUT_DIR = Path(__file__).resolve().parent
DATA_DIR = ROOT / "phase0" / "data"
IMAGES_DIR = DATA_DIR / "osaka2" / "images"
INTRINSICS_ALL_PATH = DATA_DIR / "osaka2" / "intrinsics_all.toml"
POINTS_3D_PATH = DATA_DIR / "kijunten_locations_osaka2.csv"
POINTS_2D_PATH = DATA_DIR / "points_2d_osaka2.csv"

K_MAX = 5           # criteria.md §4
SEED_CV_BASE = 100  # solvePnPRansac の再現性用(カメラ×系列ごとに一意の seed)

CAMERAS = ["cam05520126", "cam05520129", "cam41520554", "cam41520556", "cam41520557"]

# 摂動系列(criteria.md §5.2。摂動は M2 の perturb_pose と同方式・seed 固定)
SERIES = [
    ("P0", None, None, None),
    ("P1", 0.05, 2.0, 11),
    ("P2", 0.05, 2.0, 12),
    ("P3", 0.20, 5.0, 13),
    ("P4", 0.50, 10.0, 14),
]

# 一致性閾値(criteria.md §5.3。M3-0a 実測 p95、2026-07-31 数値確定)
P95_THRESHOLDS = {
    "cam05520126": (0.0255, 0.35),
    "cam05520129": (0.0229, 0.46),
    "cam41520554": (0.0184, 0.39),
    "cam41520556": (0.0205, 0.44),
    "cam41520557": (0.0240, 0.41),
}

# 同一解判定(criteria.md §5.2): 最大ペア間 カメラ中心距離・角度差
SAME_SOL_POS_M = 0.01
SAME_SOL_ANG_DEG = 0.1

# 診断値の暫定警告閾値(criteria.md §5.5。合否には使わない)
WARN_RMSE_PX = 2.0
WARN_VOL_RATIO = 0.09
WARN_DEPTH_RANGE_M = 1.4
WARN_AREA_RATIO = 0.22

# §5.4 の inlier 名簿: M3-0a の基準ポーズ outlier(m3_0a_results_summary.txt 記載)を除いた集合
M3_0A_OUTLIERS = {
    "cam05520126": {"基準_011", "基準_014"},
    "cam05520129": {"基準_061"},
    "cam41520554": {"基準_002", "基準_037", "基準_052"},
    "cam41520556": set(),
    "cam41520557": set(),
}


def pose_diff(rvec_a, tvec_a, rvec_b, tvec_b) -> tuple[float, float]:
    """2ポーズの差: (カメラ中心距離[m], 回転角[deg])。m3_0a_resample_p95.py と同一定義"""
    Ra, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64).reshape(3, 1))
    Rb, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64).reshape(3, 1))
    Ca = -Ra.T @ np.asarray(tvec_a, dtype=np.float64).reshape(3)
    Cb = -Rb.T @ np.asarray(tvec_b, dtype=np.float64).reshape(3)
    dpos = float(np.linalg.norm(Ca - Cb))
    cos_t = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    dang = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    return dpos, dang


def load_matched_points(cam_name: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """ObjectName で 3D-2D をマッチ(m3_0a_resample_p95.py と同一)"""
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


def reproj_median(p3, p2, names, rvec, tvec, K, D, cam_name: str) -> tuple[float, float]:
    """手動点再投影誤差の中央値(全点, M3-0a inlier名簿上)。criteria.md §5.4"""
    proj, _ = cv2.projectPoints(p3, np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                                np.asarray(tvec, dtype=np.float64).reshape(3, 1), K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - p2, axis=1)
    inl_mask = np.array([n not in M3_0A_OUTLIERS[cam_name] for n in names])
    return float(np.median(err)), float(np.median(err[inl_mask]))


def run_iteration_real(gaussians, cam, img_q_gray, rvec_cur, tvec_cur, k):
    """反復1回分(Stage 2〜6)。pipeline_loop.run_iteration から oracle 採点のみ除いた実写版。

    それ以外の処理・定数・順序は pipeline_loop.run_iteration と同一に保つこと。
    戻り値: (成否, 新ポーズ, 診断dict)
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
    if des_q is not None and des_r is not None and len(des_q) >= 2 and len(des_r) > 0:
        knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_r, des_q, k=2)
        good = [pr[0] for pr in knn
                if len(pr) == 2 and pr[0].distance < SIFT_RATIO * pr[1].distance]
    u_r = np.array([kp_r[m.queryIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    u_q = np.array([kp_q[m.trainIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    diag = {"k": k, "tau_px": p["tau_px"], "ransac_px": p["ransac_px"],
            "matches": len(good)}
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
    keep = ((var_map[iy, ix] < tau_d ** 2)
            & (np.linalg.norm(u_q - u_r, axis=1) < p["tau_px"])
            & (alpha[iy, ix] > GATE_ALPHA) & depth_ok
            & (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1]))
    N = int(keep.sum())
    diag["N"] = N
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

    # 診断値(criteria.md §5.5。暫定警告閾値との比較は記録のみ)
    Xi = X_w[inl]
    eig = np.sort(np.linalg.eigvalsh(np.cov(Xi.T)))[::-1]
    proj, _ = cv2.projectPoints(Xi, rvec.reshape(3, 1), tvec.reshape(3, 1), K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_q_keep[inl], axis=1)
    hull = cv2.convexHull(u_q_keep[inl].astype(np.float32))
    diag.update({
        "inliers": int(len(inl)),
        "rmse": float(np.sqrt(np.mean(resid ** 2))),
        "vol_ratio": float(eig[2] / eig[0]),
        "depth_range": float(d_bilin[keep][inl].max() - d_bilin[keep][inl].min()),
        "area_ratio": float(cv2.contourArea(hull) / (cam["width"] * cam["height"])),
    })
    diag["warns"] = [w for w, bad in [
        (f"RMSE>{WARN_RMSE_PX}px", diag["rmse"] > WARN_RMSE_PX),
        (f"vol_ratio<{WARN_VOL_RATIO}", diag["vol_ratio"] < WARN_VOL_RATIO),
        (f"depth_range<{WARN_DEPTH_RANGE_M}m", diag["depth_range"] < WARN_DEPTH_RANGE_M),
        (f"area_ratio<{WARN_AREA_RATIO}", diag["area_ratio"] < WARN_AREA_RATIO),
    ] if bad]
    return True, (rvec, tvec), diag


def run_series(gaussians, cam, img_q_gray, rvec0, tvec0, lines: list) -> dict:
    """1系列(最大 K_MAX 反復)。戻り値: 系列結果 dict(criteria.md §5.1 の成立性判定を含む)"""
    rvec_cur = np.asarray(rvec0, dtype=np.float64).flatten()
    tvec_cur = np.asarray(tvec0, dtype=np.float64).flatten()
    history = []
    converged = False
    for k in range(K_MAX):
        ok, (rvec_new, tvec_new), diag = run_iteration_real(
            gaussians, cam, img_q_gray, rvec_cur, tvec_cur, k)
        dpos_step, dang_step = pose_diff(rvec_new, tvec_new, rvec_cur, tvec_cur)
        diag.update({"step_pos_m": dpos_step, "step_ang_deg": dang_step})
        history.append(diag)
        line = (f"  k={k}: " + ("OK" if ok else f"失敗({diag.get('fail')})")
                + f" | マッチ{diag.get('matches', 0)} N={diag.get('N', 0)}"
                + f" inl={diag.get('inliers', 0)}"
                + (f" RMSE={diag['rmse']:.3f}px" if "rmse" in diag else "")
                + (f" vol={diag['vol_ratio']:.3f} zrange={diag['depth_range']:.2f}m"
                   f" area={diag['area_ratio']:.3f}" if "vol_ratio" in diag else "")
                + f" | 更新量 {dpos_step*1000:.2f}mm/{dang_step:.4f}deg"
                + (f" | 警告: {','.join(diag['warns'])}" if diag.get("warns") else ""))
        print(line)
        lines.append(line)
        if not ok:
            break
        rvec_cur, tvec_cur = rvec_new, tvec_new
        if dang_step < EPS_ROT_DEG and dpos_step < EPS_POS_M:
            converged = True
            lines.append(f"  → 収束(更新量 < {EPS_ROT_DEG}deg / {EPS_POS_M*1000:.0f}mm)")
            break
    # 成立性(§5.1): 全反復でゲート成立(fail なし)かつ K_MAX 以内に収束
    established = converged and all("fail" not in h for h in history)
    return {"established": established, "converged": converged,
            "rvec": rvec_cur, "tvec": tvec_cur, "history": history}


def precheck_intrinsics(lines: list) -> None:
    """§2 事前確認: Config_scene と intrinsics_all の K・歪み差の記録(判定には使わない)"""
    import re
    try:
        import tomllib
    except ModuleNotFoundError:
        import tomli as tomllib
    text = INTRINSICS_ALL_PATH.read_text()
    text = re.sub(r"^\[metadata\]\n(?:^(?!\[).*\n?)*", "", text, flags=re.M)
    intr = tomllib.loads(text)
    cams_cfg = load_cameras_toml(TOML_PATH)
    lines.append("§2 事前確認: Config_scene vs intrinsics_all の K・歪み差(記録のみ)")
    for name in CAMERAS:
        cc = select_camera(cams_cfg, name)
        v = intr[name]
        Ki = np.array(v["matrix"], dtype=np.float64)
        Di = np.array(v["distortions"], dtype=np.float64)
        Kc, Dc = cc["K"], np.asarray(cc["D"], dtype=np.float64).flatten()
        lines.append(
            f"  {name}: Δfx={Kc[0,0]-Ki[0,0]:+.1f} Δfy={Kc[1,1]-Ki[1,1]:+.1f} "
            f"Δcx={Kc[0,2]-Ki[0,2]:+.1f} Δcy={Kc[1,2]-Ki[1,2]:+.1f} "
            f"|ΔD|max={np.abs(Dc[:len(Di)]-Di).max():.4f}")


def main():
    lines = [
        "feat-026 M3: ドメインギャップ検証(実写クエリ)本実験",
        f"criteria: criteria.md(locked 2026-07-31) / K_MAX={K_MAX} / N_MIN={N_MIN}",
        f"入力: PLY={PLY_PATH}",
        f"      TOML={TOML_PATH}",
        f"      クエリ={IMAGES_DIR}/cam*.png",
        f"バージョン: OpenCV {cv2.__version__} / NumPy {np.__version__}",
        "",
    ]
    precheck_intrinsics(lines)
    lines.append("")

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    cams_cfg = load_cameras_toml(TOML_PATH)

    p3_all, p2_all, names_all = {}, {}, {}
    for name in CAMERAS:
        p3_all[name], p2_all[name], names_all[name] = load_matched_points(name)

    results = {}
    for ci, name in enumerate(CAMERAS):
        cam = select_camera(cams_cfg, name)
        rvec_base = np.asarray(cam["rvec"], dtype=np.float64).flatten()
        tvec_base = np.asarray(cam["tvec"], dtype=np.float64).flatten()
        img_q_gray = cv2.imread(str(IMAGES_DIR / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        if img_q_gray is None:
            raise FileNotFoundError(f"クエリ画像がない: {IMAGES_DIR / f'{name}.png'}")
        if img_q_gray.shape != (cam["height"], cam["width"]):
            raise RuntimeError(f"{name}: 画像サイズ {img_q_gray.shape} が "
                               f"TOML ({cam['height']},{cam['width']}) と不一致")

        lines.append(f"=== {name} ===")
        print(f"\n=== {name} ===")
        cam_res = {"series": {}}
        for si, (sname, ppos, pang, seed) in enumerate(SERIES):
            cv2.setRNGSeed(SEED_CV_BASE + ci * 10 + si)  # RANSAC 再現性(系列ごとに一意)
            if ppos is None:
                rvec0, tvec0 = rvec_base, tvec_base
                lines.append(f"[{sname}] 摂動なし(基準ポーズ)")
            else:
                rng = np.random.default_rng(seed)
                rvec0, tvec0 = perturb_pose(rvec_base, tvec_base, ppos, pang, rng)
                d0, a0 = pose_diff(rvec0, tvec0, rvec_base, tvec_base)
                lines.append(f"[{sname}] 摂動 {ppos*100:.0f}cm/{pang:.0f}deg seed={seed} "
                             f"(実摂動 {d0*100:.2f}cm/{a0:.2f}deg)")
            print(lines[-1])
            sres = run_series(gaussians, cam, img_q_gray, rvec0, tvec0, lines)
            d_base, a_base = pose_diff(sres["rvec"], sres["tvec"], rvec_base, tvec_base)
            sres["diff_base_pos_m"], sres["diff_base_ang_deg"] = d_base, a_base
            lines.append(f"  最終解と基準ポーズの差: {d_base*100:.3f}cm / {a_base:.4f}deg")
            cam_res["series"][sname] = sres

        # §5.1 成立性(P0)
        p0 = cam_res["series"]["P0"]
        cam_res["established"] = p0["established"]
        # §5.2 同一解(P0/P1/P2 全成立 + 最大ペア間 ≤1cm/0.1°)
        trio = [cam_res["series"][s] for s in ("P0", "P1", "P2")]
        if all(s["established"] for s in trio):
            max_dp, max_da = 0.0, 0.0
            for i in range(3):
                for j in range(i + 1, 3):
                    dp, da = pose_diff(trio[i]["rvec"], trio[i]["tvec"],
                                       trio[j]["rvec"], trio[j]["tvec"])
                    max_dp, max_da = max(max_dp, dp), max(max_da, da)
            cam_res["same_solution"] = (max_dp <= SAME_SOL_POS_M and max_da <= SAME_SOL_ANG_DEG)
            lines.append(f"  同一解判定: 最大ペア間 {max_dp*100:.3f}cm/{max_da:.4f}deg → "
                         f"{'同一解' if cam_res['same_solution'] else '不一致'}")
        else:
            cam_res["same_solution"] = False
            lines.append("  同一解判定: P0/P1/P2 に不成立系列があるため不合格")
        # §5.3 一致性(P0 vs 基準ポーズ ≤ p95)
        th_p, th_a = P95_THRESHOLDS[name]
        cam_res["agree"] = (p0["established"]
                            and p0["diff_base_pos_m"] <= th_p
                            and p0["diff_base_ang_deg"] <= th_a)
        lines.append(f"  一致性判定: P0差 {p0['diff_base_pos_m']*100:.3f}cm/"
                     f"{p0['diff_base_ang_deg']:.4f}deg vs 閾値 {th_p*100:.2f}cm/{th_a:.2f}deg → "
                     f"{'一致' if cam_res['agree'] else '不一致'}")
        # §5.4 手動点再投影整合(記録のみ)
        K, D = cam["K"], cam["D"]
        med_all_b, med_inl_b = reproj_median(p3_all[name], p2_all[name], names_all[name],
                                             rvec_base, tvec_base, K, D, name)
        med_all_p, med_inl_p = reproj_median(p3_all[name], p2_all[name], names_all[name],
                                             p0["rvec"], p0["tvec"], K, D, name)
        cam_res["reproj"] = {"base_all": med_all_b, "base_inl": med_inl_b,
                             "p0_all": med_all_p, "p0_inl": med_inl_p}
        lines.append(f"  手動点再投影中央値[px]: 基準 全点{med_all_b:.2f}/名簿{med_inl_b:.2f} → "
                     f"P0解 全点{med_all_p:.2f}/名簿{med_inl_p:.2f} "
                     f"({'悪化なし' if med_inl_p <= med_inl_b else '悪化(記録のみ)'})")
        # §5.6 SIFT 不成立の記録
        cam_res["sift_ok"] = p0["established"]
        cam_res["pass"] = (cam_res["established"] and cam_res["same_solution"]
                           and cam_res["agree"])
        lines.append(f"  カメラ判定: 成立={cam_res['established']} "
                     f"同一解={cam_res['same_solution']} 一致={cam_res['agree']} → "
                     f"{'PASS' if cam_res['pass'] else 'FAIL'}")
        lines.append("")
        results[name] = cam_res

    n_pass = sum(1 for r in results.values() if r["pass"])
    go = n_pass >= 3
    lines.append(f"=== M3 判定(criteria.md §6) === PASS {n_pass}/5 → {'Go' if go else 'No-Go'}")
    sift_fails = [n for n, r in results.items() if not r["sift_ok"]]
    if sift_fails:
        lines.append(f"SIFT 不成立カメラ(M3b 検討対象): {sift_fails}")
    print("\n" + lines[-2] if sift_fails else "\n" + lines[-1])

    (OUT_DIR / "m3_result.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT_DIR / "m3_history.npz",
             results=np.array([results], dtype=object))
    print(f"保存: {OUT_DIR / 'm3_result.txt'}, {OUT_DIR / 'm3_history.npz'}")


if __name__ == "__main__":
    main()

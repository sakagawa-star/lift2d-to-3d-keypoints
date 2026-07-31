"""案1 反復ループエンジン: Stage 2〜7 の全反復を実行する共通実装

用途:
  - Stage 7 検証: Stage 4〜6 の推定ポーズから反復2回目（引き締めゲート）を回す
      uv run --project phase4 python pipeline_loop.py --start-from-stage4-6
  - 収束域マップ（後続実験）: 指定摂動から複数反復を回し収束/発散を記録する
      uv run --project phase4 python pipeline_loop.py --perturb-pos 0.10 --perturb-ang 5 --seed 1

各反復 k: レンダ（色+深度+α、方式A）→ SIFT マッチング → ゲート（kに応じた
段階的引き締め）→ 深度3D化 → PnP-RANSAC + 精密化 → 収束判定。
真ポーズ既知（cam05520126 の TOML 値）を利用して、正解変位場による純度採点と
ポーズ誤差測定を毎反復行う（合成実験専用の評価機構）。

出力: --out-prefix で指定した名前で <prefix>_result.txt / <prefix>_history.npz
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, TRUE_CAM, NEAR_PLANE,
)
from stage3_perturb_match import (  # noqa: E402
    sample_depth_bilinear, oracle_displacement, perturb_pose,
)
from stage4_6_pnp_iteration import depth_variance_map, pose_error  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent

# ゲート・PnPの反復スケジュール（experiment_log.md Stage 7 予測に記載した確定値）
SCHEDULE = {
    0: {"tau_px": 250.0, "ransac_px": 8.0},   # k=0: 初期誤差を通す緩いゲート
    1: {"tau_px": 10.0, "ransac_px": 2.0},    # k>=1: 段階的引き締め
}
GATE_VAR_WINDOW = 5
GATE_VAR_REL = 0.02
GATE_ALPHA = 0.5
GATE_Z_RANGE = (0.5, 10.0)
SIFT_RATIO = 0.75
ORACLE_CORRECT_PX = 2.0
N_MIN = 30
EPS_ROT_DEG = 0.05          # 収束判定 ε_R
EPS_POS_M = 0.005           # 収束判定 ε_t（シーンスケール数mの0.1%強）


def sched(k: int) -> dict:
    return SCHEDULE[min(k, max(SCHEDULE.keys()))]


def run_iteration(gaussians, cam, img_q_gray, rvec_cur, tvec_cur, k,
                  rvec_true, tvec_true):
    """反復1回分（Stage 2〜6）。戻り値: (成否, 新ポーズ, 診断dict)"""
    p = sched(k)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur
    K, D = cam["K"], cam["D"]

    # Stage 2: レンダ（方式A）
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

    # oracle 採点（合成実験専用の評価。パイプラインの判断には使わない）
    u_pred, _ = oracle_displacement(u_r, depth, cam, rvec_cur, tvec_cur,
                                    rvec_true, tvec_true)
    err_oracle = np.linalg.norm(u_q - u_pred, axis=1)

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

    # 診断値
    Xi = X_w[inl]
    eig = np.sort(np.linalg.eigvalsh(np.cov(Xi.T)))[::-1]
    proj, _ = cv2.projectPoints(Xi, rvec.reshape(3, 1), tvec.reshape(3, 1), K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_q_keep[inl], axis=1)
    err_inl = err_oracle[keep][inl]
    known = np.isfinite(err_inl)
    hull = cv2.convexHull(u_q_keep[inl].astype(np.float32))
    diag.update({
        "inliers": int(len(inl)),
        "purity": float((err_inl[known] < ORACLE_CORRECT_PX).mean()) if known.any() else float("nan"),
        "rmse": float(np.sqrt(np.mean(resid ** 2))),
        "vol_ratio": float(eig[2] / eig[0]),
        "depth_range": float(d_bilin[keep][inl].max() - d_bilin[keep][inl].min()),
        "area_ratio": float(cv2.contourArea(hull) / (cam["width"] * cam["height"])),
        "oracle_disp_median": float(np.nanmedian(
            np.linalg.norm(u_pred - u_r, axis=1))),
    })
    return True, (rvec, tvec), diag


def run_pipeline(gaussians, cam, img_q_gray, rvec0, tvec0,
                 rvec_true, tvec_true, k_start=0, k_max=5, verbose=True):
    """反復ループ本体。戻り値: (converged, (rvec, tvec), history, lines)"""
    rvec_cur, tvec_cur = np.asarray(rvec0).flatten(), np.asarray(tvec0).flatten()
    history = []
    lines = []
    converged = False
    for k in range(k_start, k_max):
        ok, (rvec_new, tvec_new), diag = run_iteration(
            gaussians, cam, img_q_gray, rvec_cur, tvec_cur, k, rvec_true, tvec_true)
        dpos, dang = pose_error(rvec_new, tvec_new, rvec_true, tvec_true)
        dpos_step, dang_step = pose_error(rvec_new, tvec_new, rvec_cur, tvec_cur)
        diag.update({"pose_err_pos_m": dpos, "pose_err_ang_deg": dang,
                     "step_pos_m": dpos_step, "step_ang_deg": dang_step})
        history.append(diag)
        line = (f"k={k}: " + ("OK" if ok else f"失敗({diag.get('fail')})")
                + f" | マッチ{diag.get('matches', 0)} N={diag.get('N', 0)}"
                + f" inl={diag.get('inliers', 0)}"
                + (f" 純度={diag['purity']*100:.1f}%" if "purity" in diag else "")
                + (f" RMSE={diag['rmse']:.3f}px" if "rmse" in diag else "")
                + f" | ポーズ誤差 {dpos*1000:.2f}mm/{dang:.4f}deg"
                + f" | 更新量 {dpos_step*1000:.2f}mm/{dang_step:.4f}deg")
        if verbose:
            print(line)
        lines.append(line)
        if not ok:
            lines.append("→ 反復失敗により中断")
            break
        rvec_cur, tvec_cur = rvec_new, tvec_new
        if dang_step < EPS_ROT_DEG and dpos_step < EPS_POS_M:
            converged = True
            lines.append(f"→ 収束（更新量 < {EPS_ROT_DEG}deg / {EPS_POS_M*1000:.0f}mm）")
            break
    return converged, (rvec_cur, tvec_cur), history, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start-from-stage4-6", action="store_true",
                    help="Stage 4〜6 の推定ポーズ（stage4_6_pose.npz）から k=1 として開始")
    ap.add_argument("--perturb-pos", type=float, default=None, help="位置摂動 [m]")
    ap.add_argument("--perturb-ang", type=float, default=None, help="角度摂動 [deg]")
    ap.add_argument("--seed", type=int, default=0, help="摂動方向の乱数シード")
    ap.add_argument("--k-max", type=int, default=5, help="最大反復数")
    ap.add_argument("--out-prefix", type=str, default="stage7",
                    help="出力ファイル名の接頭辞")
    args = ap.parse_args()

    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    rvec_true = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    tvec_true = np.asarray(cam["tvec"], dtype=np.float64).flatten()

    if args.start_from_stage4_6:
        pose = np.load(OUT_DIR / "stage4_6_pose.npz")
        rvec_cur, tvec_cur = pose["rvec_est"], pose["tvec_est"]
        k_start = 1
    elif args.perturb_pos is not None and args.perturb_ang is not None:
        rng = np.random.default_rng(args.seed)
        rvec_cur, tvec_cur = perturb_pose(
            rvec_true, tvec_true, args.perturb_pos, args.perturb_ang, rng)
        k_start = 0
    else:
        ap.error("--start-from-stage4-6 か --perturb-pos/--perturb-ang を指定すること")

    img_q_gray = cv2.imread(str(OUT_DIR / "query_cam05520126.png"), cv2.IMREAD_GRAYSCALE)
    if img_q_gray is None:
        raise FileNotFoundError("クエリ画像がありません（Stage 2 未実行?）")

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)

    dpos0, dang0 = pose_error(rvec_cur, tvec_cur, rvec_true, tvec_true)
    header = (f"pipeline_loop 実行（開始ポーズ誤差: 位置 {dpos0*100:.3f}cm, "
              f"角度 {dang0:.4f}deg / k_start={k_start} / k_max={args.k_max}）")
    print(header)

    converged, (rvec_cur, tvec_cur), history, loop_lines = run_pipeline(
        gaussians, cam, img_q_gray, rvec_cur, tvec_cur,
        rvec_true, tvec_true, k_start=k_start, k_max=args.k_max)
    lines = [header] + loop_lines

    dposF, dangF = pose_error(rvec_cur, tvec_cur, rvec_true, tvec_true)
    lines.append(f"最終: 収束={converged} 位置誤差 {dposF*1000:.3f}mm, "
                 f"角度誤差 {dangF:.4f}deg")
    print(lines[-1])

    (OUT_DIR / f"{args.out_prefix}_result.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT_DIR / f"{args.out_prefix}_history.npz",
             history=np.array(history, dtype=object),
             rvec_final=rvec_cur, tvec_final=tvec_cur,
             converged=converged, final_pos_err_m=dposF, final_ang_err_deg=dangF)


if __name__ == "__main__":
    main()

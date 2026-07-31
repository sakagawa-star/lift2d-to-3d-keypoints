"""案1 Stage 4〜6: ゲート → 深度3D化 → PnP-RANSAC → ポーズ誤差測定（反復1回分）

内容:
  1. Stage 3 の成果物（stage3_matches.npz）からマッチ集合と摂動ポーズを読み込む
  2. 摂動ポーズで深度+αを再レンダ（決定的なので Stage 3 と同一の値が得られる）
  3. Stage 4 ゲート: (ii) 深度平坦性、(iii) 変位、(iv) 深度有効性
     ※(i) マッチャー信頼度は SIFT に信頼度がないため比率テスト代替済みとして省略
  4. Stage 5: 生存対応をbilinear深度で3D化（W系）
  5. Stage 6: solvePnPRansac(8px, ITERATIVE) + solvePnP精密化（既存実装と同型）
     ※ロバスト非線形最小二乗は本実験では省略（phase4環境にscipyなし。本実装時に追加）
  6. 退化診断（λ3/λ1・深度レンジ・凸包面積）と、推定ポーズ vs 真ポーズの誤差を測定

実行方法（プロジェクトルートで）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
    docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py

出力: stage4_6_result.txt / stage4_6_pose.npz
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, TRUE_CAM, NEAR_PLANE,
)
from stage3_perturb_match import sample_depth_bilinear  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
MATCHES_NPZ = OUT_DIR / "stage3_matches.npz"

# ゲートパラメータ（experiment_log.md の実行前予測に記載した確定値）
GATE_VAR_WINDOW = 5          # (ii) 深度分散の窓幅 [px]
GATE_VAR_REL = 0.02          # (ii) τ_d = シーン中央値深度の2%
GATE_DISP_PX = 250.0         # (iii) τ_px^(0)
GATE_ALPHA = 0.5             # (iv) α下限
GATE_Z_RANGE = (0.5, 10.0)   # (iv) 深度妥当範囲 [m]
RANSAC_REPROJ_PX = 8.0       # Stage 6 RANSAC 閾値（既存実装と同値）
ORACLE_CORRECT_PX = 2.0      # インライア純度判定（oracle 正解閾値、Stage 3 と同値）
N_MIN = 30                   # 失敗判定の下限


def depth_variance_map(depth: np.ndarray, valid_mask: np.ndarray,
                       window: int) -> np.ndarray:
    """window×window 局所深度分散。分散は valid_mask（ゲート(iv)相当の有効画素）のみで
    計算する（pipeline.md Stage 4: 「分散の評価は窓内の有効画素のみで行う」）。
    有効画素が窓の半数未満の位置は inf（判定不能=ゲート不通過）。"""
    d0 = np.where(valid_mask, depth, 0.0)
    k = (window, window)
    cnt = cv2.boxFilter(valid_mask.astype(np.float64), -1, k, normalize=False)
    s1 = cv2.boxFilter(d0, -1, k, normalize=False)
    s2 = cv2.boxFilter(d0 * d0, -1, k, normalize=False)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = s1 / cnt
        var = s2 / cnt - mean * mean
    var[cnt < (window * window) / 2] = np.inf
    return np.maximum(var, 0.0)


def pose_error(rvec_est, tvec_est, rvec_true, tvec_true):
    """(カメラ中心距離[m], 回転角[deg])"""
    Re, _ = cv2.Rodrigues(np.asarray(rvec_est, dtype=np.float64).reshape(3, 1))
    Rt, _ = cv2.Rodrigues(np.asarray(rvec_true, dtype=np.float64).reshape(3, 1))
    Ce = -Re.T @ np.asarray(tvec_est, dtype=np.float64).reshape(3)
    Ct = -Rt.T @ np.asarray(tvec_true, dtype=np.float64).reshape(3)
    dpos = float(np.linalg.norm(Ce - Ct))
    cos_t = (np.trace(Re.T @ Rt) - 1.0) / 2.0
    dang = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    return dpos, dang


def main():
    data = np.load(MATCHES_NPZ)
    u_r, u_q, err_oracle = data["u_r"], data["u_q"], data["err"]
    rvec_pert, tvec_pert = data["rvec_pert"], data["tvec_pert"]
    rvec_true, tvec_true = data["rvec_true"], data["tvec_true"]

    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    K, D = cam["K"], cam["D"]

    cam_pert = dict(cam)
    cam_pert["rvec"] = rvec_pert
    cam_pert["tvec"] = tvec_pert

    print("PLY読み込み・摂動ポーズの深度再レンダ中...")
    gaussians = load_ply(PLY_PATH)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_pert, NEAR_PLANE)

    # --- Stage 4: ゲート ---
    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)

    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r)

    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    gate_flat = var_map[iy, ix] < tau_d ** 2                      # (ii)
    disp = np.linalg.norm(u_q - u_r, axis=1)
    gate_disp = disp < GATE_DISP_PX                                # (iii)
    gate_valid = ((alpha[iy, ix] > GATE_ALPHA) & depth_ok          # (iv)
                  & (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1]))

    keep = gate_flat & gate_disp & gate_valid
    N = int(keep.sum())
    gate_stats = (
        f"ゲート生存: (ii)平坦性 {int(gate_flat.sum())}, (iii)変位<{GATE_DISP_PX:.0f}px "
        f"{int(gate_disp.sum())}, (iv)有効性 {int(gate_valid.sum())} → AND: N={N} / {len(u_r)}"
    )
    print(gate_stats)
    if N < N_MIN:
        (OUT_DIR / "stage4_6_result.txt").write_text(
            f"失敗: ゲート生存 N={N} < {N_MIN}\n{gate_stats}\n")
        print(f"失敗: N={N} < {N_MIN}")
        return

    # --- Stage 5: 深度3D化（W系）。pipeline.md Stage 5 の式 ---
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    ones = np.ones((u_pin.shape[0], 1))
    rays = (np.linalg.inv(K) @ np.hstack([u_pin, ones]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_pert, _ = cv2.Rodrigues(np.asarray(rvec_pert).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_pert).reshape(1, 3)) @ R_pert

    # --- Stage 6: PnP-RANSAC + 精密化（既存 estimate_extrinsic と同型） ---
    u_q_keep = u_q[keep]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        X_w, u_q_keep, K, D, iterationsCount=1000,
        reprojectionError=RANSAC_REPROJ_PX, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < 4:
        (OUT_DIR / "stage4_6_result.txt").write_text(
            f"失敗: solvePnPRansac 失敗（inlier={0 if inl is None else len(inl)}）\n{gate_stats}\n")
        print("失敗: solvePnPRansac")
        return
    inl = inl.flatten()
    ok, rvec, tvec = cv2.solvePnP(
        X_w[inl], u_q_keep[inl], K, D, rvec=rvec, tvec=tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        (OUT_DIR / "stage4_6_result.txt").write_text(
            f"失敗: solvePnP(ITERATIVE) 精密化が失敗\n{gate_stats}\n")
        print("失敗: solvePnP 精密化")
        return

    # --- 退化診断（6b、インライア集合 I 上）---
    # 本実験では診断値の記録のみ（閾値 τ_vol/τ_z/τ_area はこの実験群の実測から
    # 較正する予定のため、fail 判定には使わない。experiment_log.md に明記）---
    Xi = X_w[inl]
    cov = np.cov(Xi.T)
    eig = np.sort(np.linalg.eigvalsh(cov))[::-1]  # λ1≥λ2≥λ3
    vol_ratio = float(eig[2] / eig[0])
    d_inl = d_bilin[keep][inl]
    depth_range = float(d_inl.max() - d_inl.min())
    hull = cv2.convexHull(u_q_keep[inl].astype(np.float32))
    area_ratio = float(cv2.contourArea(hull) / (cam["width"] * cam["height"]))

    # --- 評価: 再投影RMSE、インライア純度、ポーズ誤差（真値比較） ---
    proj, _ = cv2.projectPoints(Xi, rvec, tvec, K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_q_keep[inl], axis=1)
    rmse = float(np.sqrt(np.mean(resid ** 2)))

    err_keep = err_oracle[keep]
    purity_known = np.isfinite(err_keep[inl])
    purity = float((err_keep[inl][purity_known] < ORACLE_CORRECT_PX).mean())

    dpos, dang = pose_error(rvec, tvec, rvec_true, tvec_true)
    dpos0, dang0 = pose_error(rvec_pert, tvec_pert, rvec_true, tvec_true)

    lines = [
        "Stage 4〜6（ゲート→3D化→PnP、反復1回分）実測結果",
        f"入力: Stage 3 マッチ {len(u_r)}件 / ゲート設定: 変位<{GATE_DISP_PX:.0f}px, "
        f"分散窓{GATE_VAR_WINDOW}px τ_d={tau_d*100:.1f}cm, α>{GATE_ALPHA}, "
        f"z∈{GATE_Z_RANGE}, RANSAC {RANSAC_REPROJ_PX}px",
        "",
        f"[Stage 4] {gate_stats}",
        f"[Stage 6] RANSAC インライア: {len(inl)} / {N}",
        f"  インライア純度（oracle <{ORACLE_CORRECT_PX}px）: {purity*100:.1f}% "
        f"（oracle判定可能 {int(purity_known.sum())}件中）",
        f"  再投影RMSE（インライア）: {rmse:.3f}px",
        f"[退化診断] λ3/λ1={vol_ratio:.4f}, 深度レンジ={depth_range:.2f}m, "
        f"凸包面積比={area_ratio:.3f}",
        "",
        f"[ポーズ誤差] 初期（摂動）: 位置 {dpos0*100:.2f}cm, 角度 {dang0:.3f}deg",
        f"            推定（反復1回後）: 位置 {dpos*100:.3f}cm ({dpos*1000:.2f}mm), "
        f"角度 {dang:.4f}deg",
        f"            縮小率: 位置 {dpos0/max(dpos,1e-12):.0f}x, 角度 {dang0/max(dang,1e-12):.0f}x",
    ]
    (OUT_DIR / "stage4_6_result.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    np.savez(OUT_DIR / "stage4_6_pose.npz",
             rvec_est=np.asarray(rvec).flatten(), tvec_est=np.asarray(tvec).flatten(),
             inlier_idx_in_keep=inl, keep_mask=keep,
             eig=eig, depth_range=depth_range, area_ratio=area_ratio,
             rmse=rmse, purity=purity, dpos=dpos, dang=dang)


if __name__ == "__main__":
    main()

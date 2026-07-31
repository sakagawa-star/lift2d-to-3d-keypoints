"""案1 粗整列レスキュー実験（Stage 1.5 候補の検証）

収束域マップの失敗10ケース（位置{5,10,20,30,50}cm × 角度{20,30}deg、seed=1）に対し、
反復前に粗整列（回転のみの Kabsch 整列）を挿入して救えるかを測る:

  1. 摂動ポーズでレンダ（色のみ）→ クエリとの生SIFTマッチ（比率テストのみ、変位ゲートなし）
  2. マッチ画素をピンホール光線に直し、回転のみの Kabsch 整列（トリミング反復付き）で
     首振りずれ M を推定: r_q ≈ M r_r → 補正 R ← M R（カメラ中心は不変）
  3. 補正後ポーズから pipeline_loop.run_pipeline を実行し、成功/失敗を記録

実行方法（プロジェクトルートで）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
    docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py

出力: coarse_align_result.txt / coarse_align_history.npz
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from pipeline_loop import (  # noqa: E402
    run_pipeline, PLY_PATH, TOML_PATH, TRUE_CAM, NEAR_PLANE, SIFT_RATIO,
)
from stage3_perturb_match import perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import pose_error  # noqa: E402

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase4"))
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent

POS_LIST_M = [0.05, 0.10, 0.20, 0.30, 0.50]
ANG_LIST_DEG = [20, 30]
SEED = 1                     # 収束域マップと同一（同じ摂動を再現）
K_MAX = 5
SUCCESS_POS_M = 0.005
SUCCESS_ANG_DEG = 0.05
KABSCH_TRIM_ITERS = 3        # トリミング反復回数
KABSCH_TRIM_DEG = 2.0        # 各反復で残差角がこの値超のペアを棄却 [deg]
MIN_RAW_MATCHES = 30         # 生マッチがこれ未満なら粗整列不成立


def rays_from_pixels(uv: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """歪み画素座標 (M,2) → 正規化ピンホール光線（単位ベクトル）(M,3)"""
    pts = uv.reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack(
        [u_pin, np.ones((len(u_pin), 1))]).T).T
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def kabsch_rotation(r_from: np.ndarray, r_to: np.ndarray) -> np.ndarray:
    """r_to ≈ M @ r_from となる回転 M を SVD で解く（Kabsch）"""
    H = r_from.T @ r_to
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    S = np.diag([1.0, 1.0, d])
    return Vt.T @ S @ U.T


def estimate_yaw_correction(u_r, u_q, K, D):
    """生マッチから首振りずれの回転 M（r_q ≈ M r_r）をトリミング付き Kabsch で推定。

    Returns:
        (M, n_used, resid_deg): 回転行列、最終的に使ったペア数、残差角の中央値[deg]
    """
    r_r = rays_from_pixels(u_r, K, D)
    r_q = rays_from_pixels(u_q, K, D)
    use = np.ones(len(r_r), dtype=bool)
    M = np.eye(3)
    for _ in range(KABSCH_TRIM_ITERS):
        if use.sum() < 3:
            break
        M = kabsch_rotation(r_r[use], r_q[use])
        resid = np.degrees(np.arccos(np.clip(
            np.sum((r_r @ M.T) * r_q, axis=1), -1.0, 1.0)))
        use = resid < KABSCH_TRIM_DEG
    if use.sum() >= 3:
        # 最終インライア集合で M を再推定してから返す
        M = kabsch_rotation(r_r[use], r_q[use])
        resid_final = np.degrees(np.arccos(np.clip(
            np.sum((r_r[use] @ M.T) * r_q[use], axis=1), -1.0, 1.0)))
        return M, int(use.sum()), float(np.median(resid_final))
    return M, int(use.sum()), float("nan")


def main():
    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    K, D = cam["K"], cam["D"]
    rvec_true = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    tvec_true = np.asarray(cam["tvec"], dtype=np.float64).flatten()

    img_q_bgr = cv2.imread(str(OUT_DIR / "query_cam05520126.png"))
    if img_q_bgr is None:
        raise FileNotFoundError("クエリ画像がありません（Stage 2 未実行?）")
    img_q_gray = cv2.cvtColor(img_q_bgr, cv2.COLOR_BGR2GRAY)

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)

    sift = cv2.SIFT_create()
    kp_q, des_q = sift.detectAndCompute(img_q_gray, None)

    results = []
    lines = [f"粗整列レスキュー実験（失敗10ケース、seed={SEED}, "
             f"トリミングKabsch {KABSCH_TRIM_ITERS}回/{KABSCH_TRIM_DEG}deg）", ""]
    for pos in POS_LIST_M:
        for ang in ANG_LIST_DEG:
            rng = np.random.default_rng(SEED)
            rvec0, tvec0 = perturb_pose(rvec_true, tvec_true, pos, ang, rng)

            # 1. 摂動ポーズでレンダ（色のみ）→ 生SIFTマッチ
            cam_p = dict(cam)
            cam_p["rvec"], cam_p["tvec"] = rvec0, tvec0
            bgr = render_image(gaussians, cam_p, near_plane=NEAR_PLANE, distort=True)
            kp_r, des_r = sift.detectAndCompute(
                cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), None)
            good = []
            if des_r is not None and des_q is not None and len(des_q) >= 2 and len(des_r) > 0:
                knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(des_r, des_q, k=2)
                good = [p[0] for p in knn
                        if len(p) == 2 and p[0].distance < SIFT_RATIO * p[1].distance]
            n_raw = len(good)
            if n_raw < MIN_RAW_MATCHES:
                line = (f"pos={pos*100:3.0f}cm ang={ang:2.0f}deg: "
                        f"粗整列不成立（生マッチ {n_raw} < {MIN_RAW_MATCHES}）")
                print(line)
                lines.append(line)
                results.append({"pos": pos, "ang": ang, "raw_matches": n_raw,
                                "status": "粗整列不成立"})
                continue
            u_r = np.array([kp_r[m.queryIdx].pt for m in good]).reshape(-1, 2)
            u_q = np.array([kp_q[m.trainIdx].pt for m in good]).reshape(-1, 2)

            # 2. 回転のみ Kabsch 整列 → 補正
            M, n_used, resid_med = estimate_yaw_correction(u_r, u_q, K, D)
            if n_used < MIN_RAW_MATCHES:
                line = (f"pos={pos*100:3.0f}cm ang={ang:2.0f}deg: "
                        f"粗整列不成立（トリミング後 {n_used} < {MIN_RAW_MATCHES}。"
                        f"生マッチ {n_raw}）")
                print(line)
                lines.append(line)
                results.append({"pos": pos, "ang": ang, "raw_matches": n_raw,
                                "kabsch_used": n_used, "status": "粗整列不成立"})
                continue
            R0, _ = cv2.Rodrigues(rvec0.reshape(3, 1))
            C0 = -R0.T @ tvec0.reshape(3)
            R_corr = M @ R0                     # r_q ≈ M r_r → 姿勢を M で補正
            t_corr = -R_corr @ C0               # カメラ中心は不変
            rvec_c, _ = cv2.Rodrigues(R_corr)
            rvec_c = rvec_c.flatten()
            dpos_c, dang_c = pose_error(rvec_c, t_corr, rvec_true, tvec_true)

            # 3. 補正後ポーズから反復
            converged, (rvec_f, tvec_f), history, _ = run_pipeline(
                gaussians, cam, img_q_gray, rvec_c, t_corr,
                rvec_true, tvec_true, k_start=0, k_max=K_MAX, verbose=False)
            dpos_f, dang_f = pose_error(rvec_f, tvec_f, rvec_true, tvec_true)
            accurate = (dpos_f < SUCCESS_POS_M) and (dang_f < SUCCESS_ANG_DEG)
            status = ("成功" if (converged and accurate)
                      else ("SILENT_FAILURE" if converged else "反復失敗"))
            line = (f"pos={pos*100:3.0f}cm ang={ang:2.0f}deg: 生マッチ{n_raw:4d} "
                    f"Kabsch使用{n_used:4d} 残差中央値{resid_med:.2f}deg | "
                    f"補正後誤差 {dpos_c*100:.1f}cm/{dang_c:.2f}deg | "
                    f"{status} 最終 {dpos_f*1000:.2f}mm/{dang_f:.4f}deg")
            print(line)
            lines.append(line)
            results.append({
                "pos": pos, "ang": ang, "raw_matches": n_raw,
                "kabsch_used": n_used, "kabsch_resid_med_deg": resid_med,
                "corrected_pos_err_m": dpos_c, "corrected_ang_err_deg": dang_c,
                "status": status, "final_pos_m": dpos_f, "final_ang_deg": dang_f,
                "history": history,
            })

    n_ok = sum(1 for r in results if r.get("status") == "成功")
    lines += ["", f"レスキュー成功 {n_ok}/{len(results)}"]
    print(lines[-1])
    (OUT_DIR / "coarse_align_result.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT_DIR / "coarse_align_history.npz",
             results=np.array(results, dtype=object))


if __name__ == "__main__":
    main()

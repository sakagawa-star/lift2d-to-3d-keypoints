"""案1 Stage 3: 摂動レンダとマッチング（正解変位場によるマッチ採点）

内容:
  1. 真ポーズ（cam05520126）に既知摂動（位置 5cm・角度 2°、シード固定）を加えた
     初期ポーズを作る
  2. 初期ポーズで方式Aレンダ（色+深度+α。stage2_smoke_test の関数を流用）
  3. 正解変位場を厳密計算: 初期レンダの各有効画素 v を深度+初期ポーズで3D化し、
     真ポーズ（歪み込み projectPoints）で投影した位置 u_pred(v) を得る。
     正しい対応なら クエリ側の対応点は u_pred(v) に一致するはず
  4. クエリ（Stage 2 成果物）と初期レンダの間で SIFT マッチング（比率テスト）
  5. 各マッチ (u_r, u_q) を ||u_q - u_pred(u_r)|| で採点し、正しいマッチ率・
     位置決め誤差分布を測る

実行方法（プロジェクトルートで）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
    docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage3_perturb_match.py

出力: stage3_result.txt / render_perturbed.png / matches_vis.png / stage3_matches.npz
"""
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

OUT_DIR = Path(__file__).resolve().parent
QUERY_PNG = OUT_DIR / "query_cam05520126.png"

SEED = 20260731
PERTURB_POS_M = 0.05    # 位置摂動 5cm
PERTURB_ANG_DEG = 2.0   # 角度摂動 2度
SIFT_RATIO = 0.75       # Lowe比率テスト
CORRECT_TH_PX = 2.0     # 正しいマッチの判定閾値
LOOSE_TH_PX = 5.0       # 緩い判定（参考）


def perturb_pose(rvec: np.ndarray, tvec: np.ndarray,
                 dpos_m: float, dang_deg: float, rng: np.random.Generator):
    """真ポーズに既知摂動を加える。

    位置: カメラ中心 C = -R^T t を乱数方向に dpos_m 移動。
    角度: カメラ座標系で乱数軸まわりに dang_deg 回転（R' = R_δ @ R）。
    """
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    C = -R.T @ tvec.reshape(3)

    dir_pos = rng.normal(size=3)
    dir_pos /= np.linalg.norm(dir_pos)
    C_new = C + dpos_m * dir_pos

    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    R_delta, _ = cv2.Rodrigues(axis * np.deg2rad(dang_deg))
    R_new = R_delta @ R

    t_new = -R_new @ C_new
    rvec_new, _ = cv2.Rodrigues(R_new)
    return rvec_new.flatten(), t_new.flatten()


def sample_depth_bilinear(depth: np.ndarray, uv: np.ndarray,
                          rel_spread_th: float = 0.05):
    """サブピクセル座標 uv (M,2) の深度を bilinear 補間で取得する。

    以下は補間不能として無効（NaN）扱い:
      - 2x2 近傍が画像外
      - 2x2 近傍に NaN を含む
      - 2x2 近傍の深度の広がり (max-min) が中心深度の rel_spread_th を超える
        （深度不連続=シルエット上。floor 参照だと隣接画素の深度を拾い oracle が
        ずれるため、判定不能として除外する）

    Returns:
        (d, valid): d は (M,) float64（無効は NaN）、valid は (M,) bool
    """
    H, W = depth.shape
    x = uv[:, 0]
    y = uv[:, 1]
    x0 = np.floor(x).astype(int)
    y0 = np.floor(y).astype(int)
    inb = (x0 >= 0) & (y0 >= 0) & (x0 + 1 <= W - 1) & (y0 + 1 <= H - 1)

    d = np.full(len(uv), np.nan, dtype=np.float64)
    if not inb.any():
        return d, inb.copy()

    x0c = np.clip(x0, 0, W - 2)
    y0c = np.clip(y0, 0, H - 2)
    q11 = depth[y0c, x0c]
    q21 = depth[y0c, x0c + 1]
    q12 = depth[y0c + 1, x0c]
    q22 = depth[y0c + 1, x0c + 1]
    neigh = np.stack([q11, q21, q12, q22], axis=0)

    all_finite = np.isfinite(neigh).all(axis=0)
    spread = neigh.max(axis=0) - neigh.min(axis=0)
    center = np.nanmedian(neigh, axis=0)
    flat = all_finite & (spread <= rel_spread_th * np.abs(center))
    valid = inb & all_finite & flat

    fx = x - x0c
    fy = y - y0c
    d_interp = (q11 * (1 - fx) * (1 - fy) + q21 * fx * (1 - fy)
                + q12 * (1 - fx) * fy + q22 * fx * fy)
    d[valid] = d_interp[valid]
    return d, valid


def oracle_displacement(u_r: np.ndarray, depth: np.ndarray,
                        cam: dict, rvec_pert, tvec_pert, rvec_true, tvec_true):
    """初期レンダ画素 u_r (M,2) の正解対応位置 u_pred (M,2) を計算する。

    u_r は歪み画像上の座標。undistortPoints でピンホール座標に直し、
    深度 d（bilinear 補間。深度不連続・NaN近傍は無効として除外）で3D化 →
    摂動ポーズの逆変換でW系へ → 真ポーズ（歪み込み projectPoints）で投影。
    無効な点は結果を NaN にする。
    """
    K = cam["K"]
    D = cam["D"]
    M = len(u_r)
    u_pred = np.full((M, 2), np.nan, dtype=np.float64)
    if M == 0:
        return u_pred, np.zeros(0, dtype=bool)

    d, valid = sample_depth_bilinear(depth, u_r)
    if not valid.any():
        return u_pred, valid

    # 歪み座標 → ピンホール画素座標（P=K で画素系に戻す）
    pts = u_r[valid].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)

    # 3D化（摂動ポーズのカメラ系）: X_cam = d * K^{-1} ũ、z成分 = d
    ones = np.ones((u_pin.shape[0], 1))
    rays = (np.linalg.inv(K) @ np.hstack([u_pin, ones]).T).T  # (n,3), z=1
    X_cam = rays * d[valid].reshape(-1, 1)

    # W系へ: X_w = R_pert^T (X_cam - t_pert)
    R_pert, _ = cv2.Rodrigues(np.asarray(rvec_pert).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_pert).reshape(1, 3)) @ R_pert

    # 真ポーズで歪み込み投影
    proj, _ = cv2.projectPoints(X_w, np.asarray(rvec_true).reshape(3, 1),
                                np.asarray(tvec_true).reshape(3, 1), K, D)
    u_pred[valid] = proj.reshape(-1, 2)
    return u_pred, valid


def main():
    rng = np.random.default_rng(SEED)
    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    rvec_true = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    tvec_true = np.asarray(cam["tvec"], dtype=np.float64).flatten()

    # 摂動ポーズ生成（摂動量の実測値も記録）
    rvec_pert, tvec_pert = perturb_pose(rvec_true, tvec_true,
                                        PERTURB_POS_M, PERTURB_ANG_DEG, rng)
    cam_pert = dict(cam)
    cam_pert["rvec"] = rvec_pert
    cam_pert["tvec"] = tvec_pert

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)

    # 初期ポーズで方式Aレンダ
    bgr_pert = render_image(gaussians, cam_pert, near_plane=NEAR_PLANE, distort=True)
    depth_pert, alpha_pert = render_depth_alpha_distorted(gaussians, cam_pert, NEAR_PLANE)
    cv2.imwrite(str(OUT_DIR / "render_perturbed.png"), bgr_pert)

    # 正解変位場の統計（マッチングの前に予測1を照合するため、格子サンプルで算出）
    H, W = depth_pert.shape
    gy, gx = np.mgrid[0:H:20, 0:W:20]
    grid = np.stack([gx.ravel(), gy.ravel()], axis=-1).astype(np.float64)
    grid_pred, grid_valid = oracle_displacement(
        grid, depth_pert, cam, rvec_pert, tvec_pert, rvec_true, tvec_true)
    disp = np.linalg.norm(grid_pred[grid_valid] - grid[grid_valid], axis=1)
    oracle_stats = (
        f"正解変位場（格子サンプル {int(grid_valid.sum())}点）: "
        f"中央値={np.median(disp):.1f}px p5={np.percentile(disp, 5):.1f} "
        f"p95={np.percentile(disp, 95):.1f} 最大={disp.max():.1f}px"
    )
    print(oracle_stats)

    # SIFT マッチング（クエリ ↔ 摂動レンダ）。
    # マッチングの成否自体が測定対象なので、失敗はクラッシュでなく結果として記録する
    img_q = cv2.imread(str(QUERY_PNG), cv2.IMREAD_GRAYSCALE)
    if img_q is None:
        raise FileNotFoundError(f"クエリ画像が読めません（Stage 2 未実行?）: {QUERY_PNG}")
    img_r = cv2.cvtColor(bgr_pert, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create()
    kp_q, des_q = sift.detectAndCompute(img_q, None)
    kp_r, des_r = sift.detectAndCompute(img_r, None)

    good = []
    if des_q is not None and des_r is not None and len(des_q) >= 2 and len(des_r) > 0:
        matcher = cv2.BFMatcher(cv2.NORM_L2)
        knn = matcher.knnMatch(des_r, des_q, k=2)  # レンダ側 → クエリ側
        good = [pair[0] for pair in knn
                if len(pair) == 2 and pair[0].distance < SIFT_RATIO * pair[1].distance]

    u_r = np.array([kp_r[m.queryIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)
    u_q = np.array([kp_q[m.trainIdx].pt for m in good], dtype=np.float64).reshape(-1, 2)

    # 採点
    u_pred, valid = oracle_displacement(
        u_r, depth_pert, cam, rvec_pert, tvec_pert, rvec_true, tvec_true)
    err = np.linalg.norm(u_q - u_pred, axis=1)  # oracle 無効の点は NaN
    n_valid = int(valid.sum())
    finite = np.isfinite(err)
    correct = finite & (err < CORRECT_TH_PX)
    loose = finite & (err < LOOSE_TH_PX)
    n_correct = int(correct.sum())
    n_loose = int(loose.sum())
    loc_err = err[correct]

    lines = [
        "Stage 3（摂動レンダとマッチング）実測結果",
        f"摂動: 位置 {PERTURB_POS_M*100:.0f}cm, 角度 {PERTURB_ANG_DEG:.0f}deg (seed={SEED})",
        f"SIFT: ratio={SIFT_RATIO}, 正解判定閾値={CORRECT_TH_PX}px（緩い判定={LOOSE_TH_PX}px）",
        "",
        f"[予測1: 正解変位場] {oracle_stats}",
        "",
        f"[予測2: マッチング] SIFT特徴点: クエリ={len(kp_q)}, レンダ={len(kp_r)}",
        f"  比率テスト後マッチ数: {len(good)}",
        f"  うち深度有効: {n_valid}（無効深度でスキップ: {len(good)-n_valid}）",
        f"  正しいマッチ（<{CORRECT_TH_PX}px）: {n_correct} "
        f"({n_correct/max(n_valid,1)*100:.1f}% of 深度有効)",
        f"  緩い判定（<{LOOSE_TH_PX}px）: {n_loose} ({n_loose/max(n_valid,1)*100:.1f}%)",
        f"  正しいマッチの位置決め誤差: 中央値={np.median(loc_err):.2f}px "
        f"p95={np.percentile(loc_err, 95):.2f}px" if len(loc_err) else "  正しいマッチなし",
    ]
    (OUT_DIR / "stage3_result.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    # マッチ可視化（上位100件）と生データ保存（Stage 4以降の入力）
    vis = cv2.drawMatches(
        bgr_pert, kp_r, cv2.imread(str(QUERY_PNG)), kp_q,
        sorted(good, key=lambda m: m.distance)[:100], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(OUT_DIR / "matches_vis.png"), vis)
    np.savez(OUT_DIR / "stage3_matches.npz",
             u_r=u_r, u_q=u_q, u_pred=u_pred, err=err,
             rvec_pert=rvec_pert, tvec_pert=tvec_pert,
             rvec_true=rvec_true, tvec_true=tvec_true)


if __name__ == "__main__":
    main()

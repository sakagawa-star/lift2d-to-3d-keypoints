"""feat-026 診断D3: cam41520554 の A/B 仲裁(手動点再投影による判定)

experiment_log.md「診断D3」の固定条件に基づく:
  - A/B の中心ポーズ(D2 と同一定義: A=P0/P1 pooled クラスタ中心、B=P2 クラスタ中心)に
    対し、手動2D-3D対応点の再投影誤差(criteria.md §5.4 と同一定義: 全点中央値と
    M3-0a inlier 名簿中央値)を計算して比較する
  - 判定: 名簿中央値の差 ≥ 1.0px なら小さい側を採用。< 1.0px なら「仲裁不能」で保留
  - 補助(偵察。判定には使わない): seed 7000 の1回分について A 専属/B 専属/共通の
    インライアを色分けした実写重畳 PNG を出力(共通=緑、A専属=赤、B専属=青。
    マーカーは塗りつぶし小円で重なりによる見落としを緩和)

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py

出力: d3_result.txt(コミット対象) / d3_factions_cam41520554.png(ローカル)
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
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402
from stage2_smoke_test import PLY_PATH, TOML_PATH  # noqa: E402
from m3_run import load_matched_points, reproj_median, pose_diff, IMAGES_DIR  # noqa: E402
from m3_1_sampling import cluster_analysis  # noqa: E402
from d2_attractor import iterate_classify, JACCARD_R_PX  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
HISTORY_NPZ = OUT_DIR / "m3_1_history.npz"

CAM_NAME = "cam41520554"
TH_ARBITRATE_PX = 1.0   # 名簿中央値の差がこれ以上なら仲裁成立
VIZ_SEED = 7000         # 偵察可視化に使う seed(D2 の seed リスト先頭)


def mutual_pairs(uA: np.ndarray, uB: np.ndarray) -> list[tuple[int, int]]:
    """u_q 座標の相互最近傍(距離≤JACCARD_R_PX)の対応ペア(d2 の対応付けと同一規則)"""
    if len(uA) == 0 or len(uB) == 0:
        return []
    dist = np.linalg.norm(uA[:, None, :] - uB[None, :, :], axis=2)
    nnA = dist.argmin(axis=1)
    nnB = dist.argmin(axis=0)
    return [(i, int(nnA[i])) for i in range(len(uA))
            if nnB[nnA[i]] == i and dist[i, nnA[i]] <= JACCARD_R_PX]


def main():
    hist = np.load(HISTORY_NPZ, allow_pickle=True)
    res554 = hist["results"][0][CAM_NAME]

    # A/B の中心ポーズ(D2 と同一定義)
    pooled_ab = list(res554["P0"]["samples"]) + list(res554["P1"]["samples"])
    _, _, _, (rvec_A, tvec_A), _, _ = cluster_analysis(pooled_ab)
    rvec_B, tvec_B = (np.asarray(res554["P2"]["rvec"], dtype=np.float64).flatten(),
                      np.asarray(res554["P2"]["tvec"], dtype=np.float64).flatten())
    rvec_A = np.asarray(rvec_A, dtype=np.float64).flatten()
    tvec_A = np.asarray(tvec_A, dtype=np.float64).flatten()

    cams_cfg = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams_cfg, CAM_NAME)
    K, D = cam["K"], cam["D"]
    rvec_base = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    tvec_base = np.asarray(cam["tvec"], dtype=np.float64).flatten()

    p3, p2, names = load_matched_points(CAM_NAME)
    med_all_A, med_inl_A = reproj_median(p3, p2, names, rvec_A, tvec_A, K, D, CAM_NAME)
    med_all_B, med_inl_B = reproj_median(p3, p2, names, rvec_B, tvec_B, K, D, CAM_NAME)
    med_all_base, med_inl_base = reproj_median(p3, p2, names, rvec_base, tvec_base,
                                               K, D, CAM_NAME)
    diff = abs(med_inl_A - med_inl_B)
    d_ab, a_ab = pose_diff(rvec_A, tvec_A, rvec_B, tvec_B)

    lines = [
        "feat-026 診断D3: cam41520554 の A/B 仲裁(experiment_log.md 診断D3)",
        f"A = P0/P1 pooled クラスタ中心 / B = P2 クラスタ中心(A-B 差 {d_ab*100:.3f}cm/{a_ab:.4f}deg)",
        f"手動対応点: {len(p3)}点(名簿 = M3-0a inlier {len(p3) - 3}点)",
        "",
        "手動点再投影誤差の中央値 [px](全点 / 名簿):",
        f"  A:    {med_all_A:.3f} / {med_inl_A:.3f}",
        f"  B:    {med_all_B:.3f} / {med_inl_B:.3f}",
        f"  基準: {med_all_base:.3f} / {med_inl_base:.3f}(参考。基準ポーズ=全点PnP解)",
        "",
        f"名簿中央値の差 = {diff:.3f}px(判定閾値 {TH_ARBITRATE_PX}px)",
    ]
    if diff >= TH_ARBITRATE_PX:
        winner = "A" if med_inl_A < med_inl_B else "B"
        lines.append(f"判定: **{winner} を採用**(手動測量座標系と整合する側)")
    else:
        winner = None
        lines.append("判定: **仲裁不能(保留)**(差が閾値未満。無理に判定しない)")
    print("\n".join(lines))

    # 仲裁本体の結果を先に保存する(補助可視化の失敗に巻き込まれないため。codex-40 指摘)
    result_path = OUT_DIR / "d3_result.txt"
    result_path.write_text("\n".join(lines) + "\n")
    print(f"保存: {result_path}")

    # 補助: 派閥の可視化(判定には使わない。失敗しても仲裁結果には影響なし)
    try:
        print("\n偵察可視化(seed 7000)...")
        gaussians = load_ply(PLY_PATH)
        img_q = cv2.imread(str(IMAGES_DIR / f"{CAM_NAME}.png"), cv2.IMREAD_GRAYSCALE)
        if img_q is None:
            raise FileNotFoundError("クエリ画像がない")
        per = {}
        for tag, (rv, tv) in (("A", (rvec_A, tvec_A)), ("B", (rvec_B, tvec_B))):
            cv2.setRNGSeed(VIZ_SEED)
            per[tag] = iterate_classify(gaussians, cam, img_q, rv, tv)
        uA, uB = per["A"]["u_q_inl"], per["B"]["u_q_inl"]
        pairs = mutual_pairs(uA, uB)
        common_A = {i for i, _ in pairs}
        common_B = {j for _, j in pairs}
        vis = cv2.cvtColor(img_q, cv2.COLOR_GRAY2BGR)
        for i, (x, y) in enumerate(uA):
            color = (0, 200, 0) if i in common_A else (0, 0, 255)   # 共通=緑 / A専属=赤
            cv2.circle(vis, (int(round(x)), int(round(y))), 3, color, -1)
        for j, (x, y) in enumerate(uB):
            if j not in common_B:                                    # B専属=青
                cv2.circle(vis, (int(round(x)), int(round(y))), 3, (255, 0, 0), -1)
        png = OUT_DIR / f"d3_factions_{CAM_NAME}.png"
        cv2.imwrite(str(png), vis)
        lines += [
            "",
            f"偵察可視化(seed {VIZ_SEED}): A インライア {len(uA)} / B インライア {len(uB)} / "
            f"共通 {len(pairs)}",
            f"  {png.name}(緑=共通, 赤=A専属, 青=B専属。塗りつぶし小円)",
        ]
        print("\n".join(lines[-3:]))
    except Exception as e:  # noqa: BLE001
        lines += ["", f"偵察可視化は未生成({e})。判定には影響なし"]
        print(lines[-1])

    result_path.write_text("\n".join(lines) + "\n")
    print(f"保存: {result_path}")


if __name__ == "__main__":
    main()

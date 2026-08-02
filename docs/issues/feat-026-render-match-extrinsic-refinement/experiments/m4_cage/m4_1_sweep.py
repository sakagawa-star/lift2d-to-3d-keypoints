"""feat-026 M4-1 フェーズ0+1: 深度ゲート定数の較正スイープ(criteria_m4_1.md locked 2026-08-02)

フェーズ0(再現検証): M4 と同一手順で3チェーン(A 無摂動 / B・C 5cm2°摂動 seed 11,12)の
  k=0 中間生成物(ratio後マッチ座標・変位ゲート判定・深度/αマップ)を生成・保存し、
  現行定数 (rel_spread_th=0.05, GATE_VAR_REL=0.02) の再適用で criteria_m4_1.md
  フェーズ0 の期待値表(m4_history.npz 由来)と完全一致することを確認する。
  不一致 = 実装バグとして exit code 1 で停止する(フェーズ1 に進まない)。

フェーズ1(較正スイープ): 保存済み中間生成物に 4×3=12 組の定数で深度系ゲートのみ
  再適用し、各組・各チェーンの N と除外内訳(真NaN/広がり超過を分離。criteria §3.3)を
  全記録する。選定規則(criteria §4 フェーズ1。事前固定):
    1. 適格 = 3チェーンすべて N ≥ 30
    2. 複数適格なら変更度(グリッド番号の和)最小
    3. 同点なら rel_spread_th が小さい側
  適格な組がなければ No-Go を出力する。

ゲート適用は m4_run.run_iteration_real_m4 の Stage 4 と同一のロジック・順序を
パラメータ化のみして再現する(判断の変更は一切しない)。除外内訳の逐次分類は
depth_ok(真NaN→広がり) → α → z → 分散の順(codex-48 の指摘事項)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_1_sweep.py

出力: m4_1_sweep_result.txt(コミット対象) / m4_1_intermediates.npz(ローカル、gitignore)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from m4_run import (  # noqa: E402  # M4 と同一の入力・手順(criteria_m4_1.md §2)
    load_matched_points, pnp_6pt, CHAINS, CAMERA,
    PLY_PATH, INTRINSICS_PATH, IMAGE_PATH, OUT_DIR,
)
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import render_depth_alpha_distorted, NEAR_PLANE  # noqa: E402
from stage3_perturb_match import sample_depth_bilinear, perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE,
    SIFT_RATIO, N_MIN,
)
from m3_run import pose_diff  # noqa: E402
from m3_1_sampling import SEED_SAMP_BASE  # noqa: E402

# criteria_m4_1.md §3.1: 較正グリッド(事前固定・事後追加禁止)
GRID_SPREAD = [0.05, 0.10, 0.15, 0.20]
GRID_VAR = [0.02, 0.04, 0.08]
# 現行定数(M2較正値)のグリッド内位置(フェーズ0 の再適用と変更度計算の基準)
CUR_SPREAD = 0.05
CUR_VAR = 0.02
# 真NaN分離用: 広がり条件を実質無効化する閾値(inf は 0*inf=nan を避けて有限大値)
SPREAD_TH_OFF = 1e9

# criteria_m4_1.md フェーズ0: 期待値表(m4_history.npz の k=0 診断レコード。比較の正)
EXPECTED = {
    "A": {"n_raw": 3898, "matches": 97, "n_disp": 78,
          "excl_depth_nan": 31, "excl_alpha": 0, "excl_z": 0, "excl_var": 21, "N": 26},
    "B": {"n_raw": 4461, "matches": 109, "n_disp": 66,
          "excl_depth_nan": 36, "excl_alpha": 0, "excl_z": 0, "excl_var": 16, "N": 14},
    "C": {"n_raw": 4326, "matches": 108, "n_disp": 73,
          "excl_depth_nan": 39, "excl_alpha": 0, "excl_z": 0, "excl_var": 15, "N": 19},
}


def gen_intermediates(gaussians, cam, img_q_gray, rvec_cur, tvec_cur):
    """k=0 の中間生成物を生成する(m4_run.run_iteration_real_m4 の Stage 2〜3 +
    変位ゲート判定と同一。深度系ゲートは適用しない)"""
    p = sched(0)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur

    # Stage 2: レンダ(方式A)
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)
    img_r = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # Stage 3: SIFT マッチング(m4_run と同一)
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

    # 変位ゲート(k=0: tau_px=250。定数スイープの対象外なのでここで確定)
    cond_disp = np.linalg.norm(u_q - u_r, axis=1) < p["tau_px"]

    return {"depth": depth, "alpha": alpha, "u_r": u_r, "u_q": u_q,
            "cond_disp": cond_disp, "n_raw": n_raw, "matches": len(good)}


def apply_depth_gates(inter, rel_spread_th, var_rel):
    """深度系ゲートの再適用(m4_run.run_iteration_real_m4 Stage 4 と同一ロジック。
    rel_spread_th / var_rel のみパラメータ化。判定式・順序は変えない)

    Returns: dict(N と除外内訳。excl_true_nan/excl_spread は criteria §3.3 の分離記録)
    """
    depth, alpha = inter["depth"], inter["alpha"]
    u_r, u_q, cond_disp = inter["u_r"], inter["u_q"], inter["cond_disp"]

    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r, rel_spread_th=rel_spread_th)
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = var_rel * float(np.median(depth[pixel_valid]))
    cond_var = var_map[iy, ix] < tau_d ** 2
    cond_alpha = alpha[iy, ix] > GATE_ALPHA
    cond_z = (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1])
    keep = cond_var & cond_disp & cond_alpha & depth_ok & cond_z

    # 真NaN/広がりの分離(criteria §3.3。診断のみ。keep の計算には使わない):
    # 広がり条件を無効化した bilinear 判定 = 「2×2近傍が画像内かつ全て有限」
    _, ok_nan_only = sample_depth_bilinear(depth, u_r, rel_spread_th=SPREAD_TH_OFF)

    return {
        "N": int(keep.sum()),
        "n_disp": int(cond_disp.sum()),
        "excl_depth_nan": int((cond_disp & ~depth_ok).sum()),
        "excl_true_nan": int((cond_disp & ~ok_nan_only).sum()),
        "excl_spread": int((cond_disp & ok_nan_only & ~depth_ok).sum()),
        "excl_alpha": int((cond_disp & depth_ok & ~cond_alpha).sum()),
        "excl_z": int((cond_disp & depth_ok & cond_alpha & ~cond_z).sum()),
        "excl_var": int((cond_disp & depth_ok & cond_alpha & cond_z & ~cond_var).sum()),
    }


def main():
    lines = [
        "M4-1 フェーズ0+1: 深度ゲート定数の較正スイープ(criteria_m4_1.md locked 2026-08-02)",
        f"シーン: {PLY_PATH}",
        f"クエリ: {IMAGE_PATH}",
        f"グリッド: rel_spread_th={GRID_SPREAD} × GATE_VAR_REL={GRID_VAR}",
        f"固定: α>{GATE_ALPHA}, z∈{GATE_Z_RANGE}m, ratio={SIFT_RATIO}, "
        f"τ_px(k=0)={sched(0)['tau_px']}px, N_min={N_MIN}",
        "",
    ]

    cams = load_cameras_toml(str(INTRINSICS_PATH))
    cam = select_camera(cams, CAMERA)
    img_q = cv2.imread(str(IMAGE_PATH))
    if img_q is None:
        raise SystemExit(f"クエリ画像を読めません: {IMAGE_PATH}")
    img_q_gray = cv2.cvtColor(img_q, cv2.COLOR_BGR2GRAY)
    p3, p2, names = load_matched_points(CAMERA)

    # 6点PnP 初期値(M4 §5-1 と同一経路)
    res = pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res is None:
        (OUT_DIR / "m4_1_sweep_result.txt").write_text("\n".join(lines) + "\n")
        raise SystemExit("6点PnP 失敗(m4_1_sweep_result.txt に記録)")
    rvec0, tvec0 = res

    print("PLY読み込み中...")
    gaussians = load_ply(str(PLY_PATH))

    # ---- フェーズ0: 中間生成物の生成と再現検証 ----
    lines.append("== フェーズ0: 再現検証 ==")
    inters = {}
    phase0_ok = True
    for ci, (cname, dpos, dang, seed) in enumerate(CHAINS):
        if dpos is None:
            rv, tv = rvec0.copy(), tvec0.copy()
        else:
            rng = np.random.default_rng(seed)
            rv, tv = perturb_pose(rvec0, tvec0, dpos, dang, rng)
            dp, da = pose_diff(rv, tv, rvec0, tvec0)
            lines.append(f"チェーン {cname}: 摂動 {dpos*100:.0f}cm/{dang}° "
                         f"(seed={seed}, 実摂動 {dp*100:.2f}cm/{da:.2f}°)")
        cv2.setRNGSeed(SEED_SAMP_BASE + ci * 25)  # m4_run と同一(k=0 相当のみ)
        print(f"チェーン {cname}: k=0 中間生成物を生成中...")
        inter = gen_intermediates(gaussians, cam, img_q_gray, rv, tv)
        inters[cname] = inter

        got = apply_depth_gates(inter, CUR_SPREAD, CUR_VAR)
        got["n_raw"] = inter["n_raw"]
        got["matches"] = inter["matches"]
        exp = EXPECTED[cname]
        mism = [f"{k}: 期待{exp[k]} 実測{got[k]}" for k in exp if got[k] != exp[k]]
        split_ok = got["excl_true_nan"] + got["excl_spread"] == got["excl_depth_nan"]
        lines.append(
            f"チェーン {cname}: n_raw={got['n_raw']} ratio後={got['matches']} "
            f"n_disp={got['n_disp']} excl_nan/広がり={got['excl_depth_nan']}"
            f"(真NaN={got['excl_true_nan']}+広がり={got['excl_spread']}) "
            f"α={got['excl_alpha']} z={got['excl_z']} 分散={got['excl_var']} N={got['N']}"
            f" → {'一致' if not mism else '不一致: ' + '; '.join(mism)}"
            + ("" if split_ok else " [分離和が合算と不一致]"))
        if mism or not split_ok:
            phase0_ok = False

    np.savez(OUT_DIR / "m4_1_intermediates.npz",
             rvec0=rvec0, tvec0=tvec0,
             **{f"{c}_{k}": v for c, it in inters.items() for k, v in it.items()
                if k in ("depth", "alpha", "u_r", "u_q", "cond_disp")})
    lines.append("中間生成物: m4_1_intermediates.npz(ローカル)")

    if not phase0_ok:
        lines.append("")
        lines.append("フェーズ0 不成立: 期待値表と不一致(実装バグ)。フェーズ1 に進まない")
        out = "\n".join(lines) + "\n"
        print(out)
        (OUT_DIR / "m4_1_sweep_result.txt").write_text(out)
        raise SystemExit(1)
    lines.append("フェーズ0 成立: 3チェーンとも期待値表と完全一致")

    # ---- フェーズ1: 較正スイープ ----
    lines.append("")
    lines.append("== フェーズ1: 較正スイープ(12組) ==")
    results = {}
    # criteria §4 フェーズ1: 各組・各チェーンの N と除外内訳(分離込み)を全記録
    table_keys = ["n_disp", "N", "excl_depth_nan", "excl_true_nan", "excl_spread",
                  "excl_alpha", "excl_z", "excl_var"]
    csv_rows = ["rel_spread_th,gate_var_rel,chain," + ",".join(table_keys)]
    for i, s in enumerate(GRID_SPREAD):
        for j, v in enumerate(GRID_VAR):
            per_chain = {c: apply_depth_gates(inters[c], s, v) for c in inters}
            results[(i, j)] = per_chain
            ns = " ".join(f"{c}:N={r['N']}" for c, r in per_chain.items())
            ok = all(r["N"] >= N_MIN for r in per_chain.values())
            lines.append(f"spread={s:.2f} var={v:.2f} (変更度{i+j}): {ns}"
                         f" → {'適格' if ok else '不適格'}")
            for c, r in per_chain.items():
                csv_rows.append(f"{s:.2f},{v:.2f},{c},"
                                + ",".join(str(r[k]) for k in table_keys))
    (OUT_DIR / "m4_1_sweep_table.csv").write_text("\n".join(csv_rows) + "\n")
    lines.append("全組×全チェーンの除外内訳(分離込み): m4_1_sweep_table.csv(コミット対象)")

    # 救済不能量の記録(真NaN は spread に依存しないため組間で不変)
    lines.append("")
    for c in inters:
        r = results[(0, 0)][c]
        lines.append(f"チェーン {c}: 変位後 {r['n_disp']} 件中、真NaN除外 "
                     f"{r['excl_true_nan']} 件は定数緩和では救済不能(criteria §3.3)")

    # 選定規則(criteria §4 フェーズ1: N≥30全チェーン → 変更度最小 → spread小)
    eligible = [(i + j, i, j) for (i, j), pc in results.items()
                if all(r["N"] >= N_MIN for r in pc.values())]
    lines.append("")
    if not eligible:
        lines.append("適格な組なし → フェーズ1 で No-Go(criteria §5.5。"
                     "フェーズ2 に進まない)")
        out = "\n".join(lines) + "\n"
        print(out)
        (OUT_DIR / "m4_1_sweep_result.txt").write_text(out)
        return
    score, i_sel, j_sel = min(eligible)
    s_sel, v_sel = GRID_SPREAD[i_sel], GRID_VAR[j_sel]
    lines.append(f"選定: rel_spread_th={s_sel:.2f}, GATE_VAR_REL={v_sel:.2f}"
                 f"(変更度{score}。適格{len(eligible)}組)")
    for c, r in results[(i_sel, j_sel)].items():
        lines.append(f"  チェーン {c}: N={r['N']}(除外: 真NaN={r['excl_true_nan']} "
                     f"広がり={r['excl_spread']} α={r['excl_alpha']} z={r['excl_z']} "
                     f"分散={r['excl_var']})")
    lines.append("")
    lines.append("次: フェーズ2(m4_1_run.py)をこの選定定数で実行する"
                 "(実行前に experiment_log.md へ予測を記録)")

    out = "\n".join(lines) + "\n"
    print(out)
    (OUT_DIR / "m4_1_sweep_result.txt").write_text(out)


if __name__ == "__main__":
    main()

"""feat-026 M4-2: 檻シーンへの学習ベースマッチャー適用(criteria_m4_2.md locked 2026-08-02)

M4 の手順・判定(6点PnP + サンプリング型リファイン + チェーン統合合意、§6.1/§6.2/§6.4/§7)を
そのまま継承し、Stage 3 のマッチャーのみ LoFTR / MASt3R に差し替える。
フェーズ遷移は criteria_m4_2.md §5 の遷移表に従う:
  フェーズ0(疎通) → フェーズ1(3チェーン×9設定のオフライン比較) →
  フェーズ2(LoFTR E2E) → [LoFTR 未達時のみ] フェーズ3(MASt3R E2E)

再利用:
- m4_run.py: 6点PnP・SIFT 反復(フェーズ0(a) 再現用)・(b-1)〜(b-4) 集計・判定定数
- m3b_matcher/m3b_run.py: check_meta / MatcherOOM / gate_breakdown / save_overlay /
  マッチャー仕様定数(重み sha256 等)
- m3b_matcher/matcher_cli.py: matcher_lab 環境のマッチャー CLI(変更なし)

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_2_run.py \
        [--phase all|0|1|2|3] [--rerun-defect "..."]

出力: m4_2_result.txt(追記。コミット対象) / m4_2_stage_table.csv /
      m4_2_overlay_*.png / m4_2_state.npz・m4_2_history.npz(ローカル)
"""
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
M3B_DIR = Path(__file__).resolve().parents[1] / "m3b_matcher"
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(M3B_DIR))

import m4_run  # noqa: E402  # 内部で phase4 / a1_synthetic / m3_realquery を積む
import m3b_run  # noqa: E402  # check_meta / MatcherOOM / gate_breakdown 等
import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import render_depth_alpha_distorted, NEAR_PLANE  # noqa: E402
from stage3_perturb_match import sample_depth_bilinear, perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE, N_MIN,
)
from m3_run import pose_diff  # noqa: E402
from m3_1_sampling import (  # noqa: E402
    cluster_analysis, N_S, R_POS, R_ANG, F_C_MIN, F2_BIMODAL, SEED_SAMP_BASE,
)

OUT_DIR = Path(__file__).resolve().parent
TMP_DIR = OUT_DIR / "tmp"
RESULT_PATH = OUT_DIR / "m4_2_result.txt"
STATE_PATH = OUT_DIR / "m4_2_state.npz"
TABLE_PATH = OUT_DIR / "m4_2_stage_table.csv"

# criteria_m4_2.md §3: マッチャー仕様(M3b 継承)。LoFTR 解像度は 1280x720 固定
LOFTR_RESOLUTION = "1280x720"
LOFTR_THRESHOLDS = m3b_run.LOFTR_THRESHOLDS      # [0.0, 0.2, 0.5, 0.8]
LOFTR_MATCHERS = m3b_run.LOFTR_MATCHERS
ALL_MATCHERS = m3b_run.ALL_MATCHERS

# criteria_m4_2.md §3.1: SIFT 比較ベースライン(M4 実測 k=0 の N。再計測せず固定値)
SIFT_BASELINE_N = {"A": 26, "B": 14, "C": 19}


def call_matcher(query_png: Path, render_png: Path, matcher: str,
                 resolution: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """matcher_cli.py(m3b_matcher。変更なし)を M4-2 の tmp で実行する"""
    out_npz = TMP_DIR / f"matches_{matcher}.npz"
    cmd = ["uv", "run", "--project", str(ROOT / "matcher_lab"), "python",
           str(M3B_DIR / "matcher_cli.py"), str(query_png), str(render_png),
           "--matcher", matcher, "--out", str(out_npz), "--resolution", resolution]
    res = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
    if res.returncode == 42:
        raise m3b_run.MatcherOOM(matcher)
    if res.returncode != 0:
        raise RuntimeError(f"matcher_cli 失敗({matcher}): {res.stderr[-2000:]}")
    d = np.load(out_npz, allow_pickle=False)
    meta = json.loads(str(d["meta"]))
    return (d["u_q"].astype(np.float64), d["u_r"].astype(np.float64),
            d["conf"].astype(np.float64), meta)


def run_iteration_matcher_m4(gaussians, cam, query_png: Path, matcher: str,
                             conf_th: float, resolution: str, rvec_cur, tvec_cur, k):
    """反復1回分。m4_run.run_iteration_real_m4 の Stage 3 をマッチャー CLI に差し替えた版。

    Stage 2/4/5/6 の処理・定数・順序・診断記録は run_iteration_real_m4 と同一に保つこと
    (SIFT 固有の kp_q/kp_r 記録のみ省略。append_b_metrics は kp_q=0 の場合スキップする)。
    """
    p = sched(k)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur
    K, D = cam["K"], cam["D"]

    # Stage 2: レンダ(方式A。run_iteration_real_m4 と同一)
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)

    # Stage 3: 学習ベースマッチャー(CLI 経由)
    render_png = TMP_DIR / "render_cur.png"
    cv2.imwrite(str(render_png), bgr)
    u_q, u_r, conf, meta = call_matcher(query_png, render_png, matcher, resolution)
    m3b_run.check_meta(matcher, meta, resolution)
    n_raw = len(u_q)
    if matcher in LOFTR_MATCHERS:
        sel = conf >= conf_th
        u_q, u_r = u_q[sel], u_r[sel]
    diag = {"k": k, "tau_px": p["tau_px"], "ransac_px": p["ransac_px"],
            "n_raw": n_raw, "matches": len(u_q),
            "alpha_cov": float((alpha > GATE_ALPHA).mean())}
    if len(u_q) == 0:
        diag["fail"] = "マッチ0件"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 4: ゲート(run_iteration_real_m4 と同一条件・同一分類順)
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
    diag.update({
        "n_disp": int(cond_disp.sum()),
        "excl_depth_nan": int((cond_disp & ~cond_depth).sum()),
        "excl_alpha": int((cond_disp & cond_depth & ~cond_alpha).sum()),
        "excl_z": int((cond_disp & cond_depth & cond_alpha & ~cond_z).sum()),
        "excl_var": int((cond_disp & cond_depth & cond_alpha & cond_z & ~cond_var).sum()),
    })
    if N < N_MIN:
        diag["fail"] = f"ゲート生存 N={N} < {N_MIN}"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 5: 深度3D化(run_iteration_real_m4 と同一)
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack([u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec_cur).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_cur).reshape(1, 3)) @ R_cur

    # Stage 6: PnP-RANSAC + 精密化(run_iteration_real_m4 と同一)
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


def run_sampling_chain_matcher(gaussians, cam, query_png, matcher, conf_th, resolution,
                               rvec0, tvec0, seed_series, lines):
    """1チェーン。m4_run.run_sampling_chain の反復関数を差し替えた版(手順・seed 同一)"""
    rvec_cur = np.asarray(rvec0, dtype=np.float64).flatten()
    tvec_cur = np.asarray(tvec0, dtype=np.float64).flatten()
    diags = []
    cv2.setRNGSeed(seed_series)
    ok, (rvec_cur, tvec_cur), diag = run_iteration_matcher_m4(
        gaussians, cam, query_png, matcher, conf_th, resolution, rvec_cur, tvec_cur, 0)
    diags.append(diag)
    if not ok:
        lines.append(f"  j=0(k=0): 失敗({diag.get('fail')}) → 明示的失敗")
        return {"accepted": False, "fail": f"k=0: {diag.get('fail')}", "diags": diags}
    lines.append(f"  j=0(k=0): マッチ{diag['matches']} N={diag['N']} "
                 f"inl={diag['inliers']} RMSE={diag['rmse']:.3f}px")
    samples = []
    for j in range(1, N_S + 1):
        cv2.setRNGSeed(seed_series + j)
        ok, (rvec_new, tvec_new), diag = run_iteration_matcher_m4(
            gaussians, cam, query_png, matcher, conf_th, resolution, rvec_cur, tvec_cur, 1)
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


def setup_scene():
    """カメラ・画像・手動点・6点PnP 初期値(m4_run.main と同一の順序・経路)"""
    cams = load_cameras_toml(str(m4_run.INTRINSICS_PATH))
    cam = select_camera(cams, m4_run.CAMERA)
    img_q = cv2.imread(str(m4_run.IMAGE_PATH))
    if img_q is None:
        raise SystemExit(f"クエリ画像を読めません: {m4_run.IMAGE_PATH}")
    if img_q.shape[:2] != (cam["height"], cam["width"]):
        raise SystemExit("画像サイズが TOML と不一致")
    img_q_gray = cv2.cvtColor(img_q, cv2.COLOR_BGR2GRAY)
    p3, p2, names = m4_run.load_matched_points(m4_run.CAMERA)
    if len(names) != 6:
        raise SystemExit(f"手動点が6点ちょうどではありません: {len(names)}点")
    return cam, img_q, img_q_gray, p3, p2, names


def chain_inits(rvec0, tvec0):
    """3チェーンの初期ポーズ(m4_run.main §5-2 と同一の摂動・seed)"""
    inits = []
    for ci, (cname, dpos, dang, seed) in enumerate(m4_run.CHAINS):
        if dpos is None:
            rv, tv = rvec0.copy(), tvec0.copy()
        else:
            rng = np.random.default_rng(seed)
            rv, tv = perturb_pose(rvec0, tvec0, dpos, dang, rng)
        inits.append((ci, cname, rv, tv))
    return inits


def phase0(gaussians, lines: list) -> bool:
    """フェーズ0: 疎通(criteria §5)"""
    lines.append("=== フェーズ0: 疎通 ===")
    ok_all = True
    cam, img_q, img_q_gray, p3, p2, names = setup_scene()

    # 前提: マッチャー資産の同一性(M3b §3 の固定値と一致)
    for mname, (path, sha) in m3b_run.LOFTR_WEIGHTS.items():
        got = m3b_run.sha256_file(path) if path.exists() else "(ファイルなし)"
        lines.append(f"{mname} sha256: {'一致' if got == sha else '不一致: ' + got}")
        ok_all &= got == sha
    msize = m3b_run.MAST3R_CKPT.stat().st_size if m3b_run.MAST3R_CKPT.exists() else -1
    lines.append(f"mast3r ckpt サイズ: {msize} → "
                 f"{'一致' if msize == m3b_run.MAST3R_CKPT_BYTES else '不一致'}")
    ok_all &= msize == m3b_run.MAST3R_CKPT_BYTES

    # (a) SIFT 再現(6点PnP → 3チェーン k=0。M4 と同一 seed。N の完全一致が条件)
    res = m4_run.pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res is None:
        lines.append("フェーズ0 判定: 不成立(6点PnP 失敗)")
        return False
    rvec0, tvec0 = res
    lines.append("(a) SIFT 再現(M4 実測 k=0 の N との一致):")
    for ci, cname, rv, tv in chain_inits(rvec0, tvec0):
        cv2.setRNGSeed(SEED_SAMP_BASE + ci * 25)  # m4_run.run_sampling_chain j=0 と同一
        _, _, diag = m4_run.run_iteration_real_m4(gaussians, cam, img_q_gray, rv, tv, 0)
        exp_n = SIFT_BASELINE_N[cname]
        same = diag.get("N", 0) == exp_n
        lines.append(f"  チェーン{cname}: N={diag.get('N', 0)}(期待{exp_n}) "
                     f"ratio後={diag.get('matches', 0)} → {'一致' if same else '不一致'}")
        ok_all &= same

    # (b) マッチャー疎通(チェーンA 初期ポーズのペア)+ (c) 再現性
    cam_a = dict(cam)
    cam_a["rvec"], cam_a["tvec"] = rvec0, tvec0
    bgr = render_image(gaussians, cam_a, near_plane=NEAR_PLANE, distort=True)
    render_png = TMP_DIR / "render_chainA.png"
    cv2.imwrite(str(render_png), bgr)
    query_png = m4_run.IMAGE_PATH
    results = {}
    for matcher in ALL_MATCHERS:
        res_m = LOFTR_RESOLUTION if matcher in LOFTR_MATCHERS else "full"
        u_q, u_r, conf, meta = call_matcher(query_png, render_png, matcher, res_m)
        results[matcher] = (u_q, u_r)
        got = len(u_q) > 0 and meta["in_range_rate"] == m3b_run.IN_RANGE_REQ
        rt_ok = meta["roundtrip_max_px"] <= m3b_run.RT_MAX_PX
        lines.append(f"(b) {matcher}: {len(u_q)} マッチ, 範囲内率 "
                     f"{meta['in_range_rate']:.3f}, round-trip {meta['roundtrip_max_px']:.4f}px"
                     f" → {'OK' if got and rt_ok else 'NG'}")
        ok_all &= got and rt_ok
        m3b_run.save_overlay(query_png, render_png, u_q, u_r,
                             OUT_DIR / f"m4_2_overlay_{matcher}.png")
    lines.append("(c) 再現性(同一入力2回目との差。記録のみ):")
    for matcher in ALL_MATCHERS:
        res_m = LOFTR_RESOLUTION if matcher in LOFTR_MATCHERS else "full"
        u_q2, u_r2, _, _ = call_matcher(query_png, render_png, matcher, res_m)
        u_q1, u_r1 = results[matcher]
        if len(u_q1) == len(u_q2):
            dmax = float(max(np.abs(u_q1 - u_q2).max(initial=0),
                             np.abs(u_r1 - u_r2).max(initial=0)))
            lines.append(f"  {matcher}: マッチ数同一({len(u_q1)}) 座標最大差 {dmax:.4f}px")
        else:
            lines.append(f"  {matcher}: マッチ数差あり {len(u_q1)} vs {len(u_q2)}")

    lines.append(f"フェーズ0 判定: {'成立' if ok_all else '不成立(中断)'}")
    return ok_all


def phase1(gaussians, lines: list) -> dict:
    """フェーズ1: 3チェーン初期ポーズ × 9設定のオフライン比較(criteria §5)"""
    lines.append("=== フェーズ1: マッチング段のオフライン比較(3チェーン × 9設定) ===")
    cam, img_q, img_q_gray, p3, p2, names = setup_scene()
    res0 = m4_run.pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res0 is None:
        raise RuntimeError("6点PnP 失敗(フェーズ0 成立後の想定外事象。中断)")
    rvec0, tvec0 = res0
    tau_px = sched(0)["tau_px"]

    raw_matches = {}  # (matcher, chain) -> (u_q, u_r, conf)
    scene = {}        # chain -> (depth, alpha, var_map, tau_d)
    for ci, cname, rv, tv in chain_inits(rvec0, tvec0):
        cam_c = dict(cam)
        cam_c["rvec"], cam_c["tvec"] = rv, tv
        bgr = render_image(gaussians, cam_c, near_plane=NEAR_PLANE, distort=True)
        depth, alpha = render_depth_alpha_distorted(gaussians, cam_c, NEAR_PLANE)
        render_png = TMP_DIR / f"render_chain{cname}.png"
        cv2.imwrite(str(render_png), bgr)
        pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                       & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
        var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
        tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
        scene[cname] = (depth, alpha, var_map, tau_d)
        for matcher in ALL_MATCHERS:
            res_m = LOFTR_RESOLUTION if matcher in LOFTR_MATCHERS else "full"
            u_q, u_r, conf, meta = call_matcher(m4_run.IMAGE_PATH, render_png,
                                                matcher, res_m)
            m3b_run.check_meta(matcher, meta, res_m)
            raw_matches[(matcher, cname)] = (u_q, u_r, conf)

    chains = [c[1] for c in chain_inits(rvec0, tvec0)]
    rows = []
    per_config = {}  # (matcher, th) -> {chain: breakdown}
    for matcher in ALL_MATCHERS:
        ths = LOFTR_THRESHOLDS if matcher in LOFTR_MATCHERS else [None]
        for th in ths:
            cfg = (matcher, th)
            per_config[cfg] = {}
            for cname in chains:
                u_q, u_r, conf = raw_matches[(matcher, cname)]
                if th is not None:
                    sel = conf >= th
                    u_qs, u_rs = u_q[sel], u_r[sel]
                else:
                    u_qs, u_rs = u_q, u_r
                depth, alpha, var_map, tau_d = scene[cname]
                bd = m3b_run.gate_breakdown(u_qs, u_rs, depth, alpha, var_map,
                                            tau_d, tau_px)
                bd.pop("keep")
                bd.pop("d_bilin")
                per_config[cfg][cname] = bd
                rows.append({"config": f"{matcher}" + (f"@{th}" if th is not None else ""),
                             "chain": cname, **bd})
    for cname in chains:
        rows.append({"config": "sift_m4_baseline", "chain": cname, "raw": "",
                     "after_disp": "", "excl_nan_spread": "", "excl_alpha": "",
                     "excl_z": "", "excl_var": "", "N": SIFT_BASELINE_N[cname]})
    with open(TABLE_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["config", "chain", "raw", "after_disp",
                                          "excl_nan_spread", "excl_alpha", "excl_z",
                                          "excl_var", "N"])
        w.writeheader()
        w.writerows(rows)

    for cfg, perchain in per_config.items():
        matcher, th = cfg
        label = f"{matcher}" + (f"@{th}" if th is not None else "")
        ns = {c: perchain[c]["N"] for c in chains}
        n_ok = sum(1 for c in chains if ns[c] >= N_MIN)
        lines.append(f"  {label}: N(A/B/C)={[ns[c] for c in chains]} "
                     f"(N≥{N_MIN}: {n_ok}/3チェーン)")

    # 進出判定と採用設定(criteria §5 フェーズ1。3チェーン版の4段タイブレーク)
    loftr_qual = [(m, th, {c: per_config[(m, th)][c]["N"] for c in chains})
                  for m in LOFTR_MATCHERS for th in LOFTR_THRESHOLDS
                  if all(per_config[(m, th)][c]["N"] >= N_MIN for c in chains)]
    adopted = None
    if loftr_qual:
        def key(entry):
            m, th, ns = entry
            vals = [ns[c] for c in chains]
            return (min(vals), sum(vals), th, 1 if m == "loftr_indoor_new" else 0)
        m, th, ns = max(loftr_qual, key=key)
        adopted = (m, th)
        lines.append(f"LoFTR: 適格({len(loftr_qual)}設定) → 採用 {m}@{th} "
                     f"(minN={min(ns.values())})")
    else:
        lines.append("LoFTR: 不適格(全8設定で3チェーン N≥30 を満たさず)")
    ns_m = {c: per_config[("mast3r", None)][c]["N"] for c in chains}
    mast3r_ok = all(ns_m[c] >= N_MIN for c in chains)
    lines.append(f"MASt3R: {'適格' if mast3r_ok else '不適格'}"
                 f"(N(A/B/C)={[ns_m[c] for c in chains]})")
    return {"adopted": adopted, "mast3r_ok": mast3r_ok}


def run_e2e(gaussians, matcher: str, conf_th, lines: list, history_key: str) -> dict:
    """フェーズ2/3 共通: M4 手順の完全再実行(m4_run.main §5-2〜§7 と同一ロジック)"""
    resolution = LOFTR_RESOLUTION if matcher in LOFTR_MATCHERS else "full"
    label = f"{matcher}" + (f"@{conf_th}" if conf_th is not None else "")
    lines.append(f"--- E2E: {label} ---")
    cam, img_q, img_q_gray, p3, p2, names = setup_scene()
    res0 = m4_run.pnp_6pt(p3, p2, cam["K"], cam["D"], names, lines)
    if res0 is None:
        raise RuntimeError("6点PnP 失敗(フェーズ0 成立後の想定外事象。中断)")
    rvec0, tvec0 = res0
    th = conf_th if conf_th is not None else 0.0

    chain_results = []
    all_samples = []
    for ci, cname, rv, tv in chain_inits(rvec0, tvec0):
        dp, da = pose_diff(rv, tv, rvec0, tvec0)
        lines.append(f"チェーン {cname}: 初期値=6点PnP解"
                     + ("(無摂動)" if ci == 0 else f"+摂動(実摂動 {dp*100:.2f}cm/{da:.2f}°)"))
        seed_series = SEED_SAMP_BASE + ci * 25
        print(f"チェーン {cname} 実行中({label}, seed_series={seed_series})...")
        r = run_sampling_chain_matcher(gaussians, cam, m4_run.IMAGE_PATH, matcher, th,
                                       resolution, rv, tv, seed_series, lines)
        r["name"] = cname
        chain_results.append(r)
        if r["accepted"]:
            all_samples.extend(r["samples"])

    est_ok = all(r["accepted"] for r in chain_results)
    lines.append(f"§6.1 成立性: {'成立' if est_ok else '不成立'}"
                 f"(チェーン成立 {sum(r['accepted'] for r in chain_results)}/3, "
                 f"サンプル {len(all_samples)}/{3*N_S})")
    result = {"label": label, "est_ok": est_ok, "accepted": False,
              "precond_ok": False, "go": False}
    if not est_ok:
        for r in chain_results:
            if not r["accepted"]:
                lines.append(f"  チェーン {r['name']} 失敗段: {r['fail']}")
        m4_run.append_b_metrics(lines, chain_results)
        lines.append(f"{label} 判定: No-Go(成立性不成立)")
        np.savez(OUT_DIR / f"m4_2_history_{history_key}.npz",
                 results=np.array([chain_results], dtype=object),
                 rvec0=rvec0, tvec0=tvec0)
        return result

    cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(all_samples)
    n_pool = len(all_samples)
    lines.append(f"§5-3 pooled 合意: f_c={f_c:.3f}({len(cluster)}/{n_pool}) "
                 f"第2クラスタ={f2:.3f} RMS半径={rms_pos*1000:.2f}mm/{rms_ang:.4f}deg")

    accepted = False
    accept_mode = ""
    rvec_sol, tvec_sol = rvec_f, tvec_f
    if f_c >= F_C_MIN and f2 < F2_BIMODAL:
        accepted = True
        accept_mode = f"単峰高合意(f_c={f_c:.3f}≥{F_C_MIN}, 第2={f2:.3f}<{F2_BIMODAL})"
    elif f2 >= F2_BIMODAL:
        rest = [all_samples[i] for i in range(n_pool) if i not in set(cluster.tolist())]
        c2, _, _, (rvec_g, tvec_g), _, _ = cluster_analysis(rest)
        f12 = f_c + len(c2) / n_pool
        med_f = m4_run.median_reproj_px(p3, p2, rvec_f, tvec_f, cam["K"], cam["D"])
        med_g = m4_run.median_reproj_px(p3, p2, rvec_g, tvec_g, cam["K"], cam["D"])
        diff = abs(med_f - med_g)
        d_pk, a_pk = pose_diff(rvec_f, tvec_f, rvec_g, tvec_g)
        lines.append(f"二峰分類: 第1={f_c:.3f} 第2={len(c2)/n_pool:.3f} 合計={f12:.3f} "
                     f"峰間距離={d_pk*100:.2f}cm/{a_pk:.3f}°")
        lines.append(f"仲裁(手動6点再投影中央値): 第1峰={med_f:.3f}px 第2峰={med_g:.3f}px "
                     f"差={diff:.3f}px(閾値{m4_run.TH_ARBITRATE_PX}px)")
        if f12 >= m4_run.F12_MIN and diff >= m4_run.TH_ARBITRATE_PX:
            accepted = True
            if med_g < med_f:
                rvec_sol, tvec_sol = rvec_g, tvec_g
                accept_mode = f"仲裁成立の二峰(第2峰を採用, 差={diff:.3f}px)"
            else:
                accept_mode = f"仲裁成立の二峰(第1峰を採用, 差={diff:.3f}px)"
        elif f12 < m4_run.F12_MIN:
            accept_mode = f"不受理(二峰だが合計占有率 {f12:.3f} < {m4_run.F12_MIN})"
        else:
            accept_mode = f"不受理(仲裁不成立: 差 {diff:.3f}px < {m4_run.TH_ARBITRATE_PX})"
    else:
        accept_mode = f"不受理(f_c={f_c:.3f} < {F_C_MIN} の非二峰)"
    lines.append(f"§6.2 受理: {'受理' if accepted else '不受理'} — {accept_mode}")

    d_init, a_init = pose_diff(rvec_sol, tvec_sol, rvec0, tvec0)
    precond_ok = (d_init <= m4_run.PRECOND_POS_M) and (a_init <= m4_run.PRECOND_ANG_DEG)
    lines.append(f"§6.4 初期解→最終解の差: {d_init*100:.2f}cm/{a_init:.3f}° "
                 f"(前提条件 ≤{m4_run.PRECOND_POS_M*100:.0f}cm/{m4_run.PRECOND_ANG_DEG}° → "
                 f"{'満足' if precond_ok else '違反'})")

    med_sol = m4_run.median_reproj_px(p3, p2, rvec_sol, tvec_sol, cam["K"], cam["D"])
    med_init = m4_run.median_reproj_px(p3, p2, rvec0, tvec0, cam["K"], cam["D"])
    lines.append(f"手動6点再投影中央値: 初期解={med_init:.3f}px 最終解={med_sol:.3f}px"
                 "(記録のみ)")
    m4_run.append_b_metrics(lines, chain_results)

    go = est_ok and accepted and precond_ok
    lines.append(f"{label} 判定: {'Go' if go else 'No-Go'}"
                 f"(成立性={'○' if est_ok else '×'} 受理={'○' if accepted else '×'} "
                 f"前提条件={'○' if precond_ok else '×'})")

    cam_r = dict(cam)
    cam_r["rvec"] = np.asarray(rvec_sol).flatten()
    cam_r["tvec"] = np.asarray(tvec_sol).flatten()
    bgr = render_image(gaussians, cam_r, near_plane=NEAR_PLANE, distort=True)
    overlay = cv2.addWeighted(img_q, 0.5, bgr, 0.5, 0)
    cv2.imwrite(str(OUT_DIR / f"m4_2_final_overlay_{history_key}.png"), overlay)
    np.savez(OUT_DIR / f"m4_2_history_{history_key}.npz",
             results=np.array([chain_results], dtype=object),
             rvec0=rvec0, tvec0=tvec0,
             rvec_sol=np.asarray(rvec_sol).flatten(),
             tvec_sol=np.asarray(tvec_sol).flatten())
    result.update({"accepted": accepted, "precond_ok": precond_ok, "go": go})
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="all", choices=["all", "0", "1", "2", "3"])
    ap.add_argument("--rerun-defect", default=None,
                    help="criteria §7: 実装欠陥による再実行時に、欠陥の特定・修正内容を記す")
    args = ap.parse_args()
    TMP_DIR.mkdir(exist_ok=True)

    lines = [
        "feat-026 M4-2: 檻シーンへの学習ベースマッチャー適用",
        f"criteria: criteria_m4_2.md(locked 2026-08-02) / N_S={N_S} / N_MIN={N_MIN}",
        f"シーン: {m4_run.PLY_PATH}",
        f"クエリ: {m4_run.IMAGE_PATH}",
        f"LoFTR 解像度: {LOFTR_RESOLUTION}(criteria §3 固定)",
        "",
    ]

    print("PLY読み込み中...")
    gaussians = load_ply(str(m4_run.PLY_PATH))

    state = {}
    if STATE_PATH.exists():
        state = {k: v for k, v in np.load(STATE_PATH, allow_pickle=True).items()}

    # 試行回数管理は M3b と同一の attempted 方式(実行前に永続化)
    if args.phase in ("all", "0"):
        m3b_run._guard_phase(state, "phase0_attempted", args.rerun_defect, lines)
        state["phase0_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        ok0 = phase0(gaussians, lines)
        state["phase0_ok"] = np.array(ok0)
        np.savez(STATE_PATH, **state)
        if not ok0:
            _flush(lines)
            return

    if args.phase in ("all", "1"):
        if not m3b_run._flag(state, "phase0_ok"):
            raise RuntimeError("フェーズ0 が未成立(criteria §5: 順序固定)")
        rerun1 = m3b_run._flag(state, "phase1_attempted")
        m3b_run._guard_phase(state, "phase1_attempted", args.rerun_defect, lines)
        if rerun1:
            m3b_run._backup(TABLE_PATH)
        state["phase1_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        st1 = phase1(gaussians, lines)
        state["adopted"] = np.array(
            list(st1["adopted"]) if st1["adopted"] else [], dtype=object)
        state["mast3r_ok"] = np.array(st1["mast3r_ok"])
        np.savez(STATE_PATH, **state)
        if not st1["adopted"] and not st1["mast3r_ok"]:
            lines.append("=== M4-2 判定(criteria §6.1) === No-Go 確定"
                         "(遷移表: LoFTR 不適格かつ MASt3R 不適格)")
            _flush(lines)
            return

    adopted = state.get("adopted", np.empty(0)).tolist()
    mast3r_ok = m3b_run._flag(state, "mast3r_ok")

    if args.phase in ("all", "2"):
        if not adopted:
            lines.append("フェーズ2: LoFTR 不適格のためスキップ(遷移表: フェーズ3へ)")
        else:
            rerun2 = m3b_run._flag(state, "phase2_attempted")
            m3b_run._guard_phase(state, "phase2_attempted", args.rerun_defect, lines)
            if rerun2:
                m3b_run._backup(OUT_DIR / "m4_2_history_loftr.npz")
            state["phase2_attempted"] = np.array(True)
            np.savez(STATE_PATH, **state)
            lines.append("=== フェーズ2: LoFTR E2E ===")
            try:
                r2 = run_e2e(gaussians, str(adopted[0]), float(adopted[1]),
                             lines, "loftr")
            except m3b_run.MatcherOOM as e:
                state["phase2_aborted_oom"] = np.array(True)
                np.savez(STATE_PATH, **state)
                lines.append(f"フェーズ2: {e.matcher} で想定外の CUDA OOM。中断"
                             "(想定外事象として報告)")
                _flush(lines)
                raise
            # completed は run_e2e が正常終了した場合のみ立てる(OOM 中断と
            # 「判定を完了して未達」を区別する。codex-64 指摘)
            state["phase2_completed"] = np.array(True)
            state["phase2_go"] = np.array(r2["go"])
            np.savez(STATE_PATH, **state)
            if r2["go"]:
                lines.append("=== M4-2 判定(criteria §6.1) === Go(LoFTR で達成。"
                             "遷移表: フェーズ3 は実施しない)")
                _flush(lines)
                return

    if args.phase in ("all", "3"):
        # 遷移表(codex-64 指摘対応): 「LoFTR 未達」= run_e2e が正常完了して go=False。
        # OOM 中断(attempted のみ)や phase2 未実行はフェーズ3 に進めない
        if m3b_run._flag(state, "phase2_go"):
            return  # Go 終了済み(フェーズ3 は実施しない)
        if adopted and not m3b_run._flag(state, "phase2_completed"):
            raise RuntimeError(
                "フェーズ2 が完了していない(未実行または OOM 中断)。遷移表上フェーズ3 の"
                "実施条件は『LoFTR が判定を完了して未達』のみ。中断の場合は --rerun-defect "
                "でフェーズ2 を再実行すること")
        loftr_done_not_go = (m3b_run._flag(state, "phase2_completed")
                             and not m3b_run._flag(state, "phase2_go"))
        cond = (not adopted and mast3r_ok) or (loftr_done_not_go and mast3r_ok)
        if not cond:
            lines.append("=== M4-2 判定(criteria §6.1) === No-Go 確定"
                         "(遷移表: MASt3R 不適格のためフェーズ3 は実施しない)")
            _flush(lines)
            return
        rerun3 = m3b_run._flag(state, "phase3_attempted")
        m3b_run._guard_phase(state, "phase3_attempted", args.rerun_defect, lines)
        if rerun3:
            m3b_run._backup(OUT_DIR / "m4_2_history_mast3r.npz")
        state["phase3_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        lines.append("=== フェーズ3: MASt3R E2E ===")
        try:
            r3 = run_e2e(gaussians, "mast3r", None, lines, "mast3r")
        except m3b_run.MatcherOOM as e:
            state["phase3_aborted_oom"] = np.array(True)
            np.savez(STATE_PATH, **state)
            lines.append(f"フェーズ3: {e.matcher} で想定外の CUDA OOM。中断"
                         "(想定外事象として報告)")
            _flush(lines)
            raise
        state["phase3_go"] = np.array(r3["go"])
        np.savez(STATE_PATH, **state)
        lines.append(f"=== M4-2 判定(criteria §6.1) === {'Go' if r3['go'] else 'No-Go 確定'}"
                     f"(MASt3R E2E の結果)")

    _flush(lines)


def _flush(lines: list) -> None:
    with open(RESULT_PATH, "a") as f:
        f.write("\n".join(lines) + "\n\n")
    print(f"保存: {RESULT_PATH}")


if __name__ == "__main__":
    main()

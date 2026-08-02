"""feat-026 M3b: 学習ベースマッチャー(LoFTR/MASt3R)の病院シーン評価 実験ドライバ

criteria.md(locked 2026-08-02)の3フェーズを実装する。レンダ・ゲート・PnP は phase4
環境(本スクリプト)で実行し、マッチャー推論は matcher_lab 環境の matcher_cli.py を
subprocess で呼ぶ(criteria §5 の環境連携仕様)。

パイプライン定数・部品は m3_run / a1_synthetic から import して M3 との同一性を保証
する。run_iteration_matcher は m3_run.run_iteration_real の Stage 3 を CLI 呼び出しに
差し替えたもの(それ以外の処理・定数・順序は同一に保つこと)。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/m3b_run.py \
        [--phase all|0|1|2]

出力: m3b_result.txt(追記。コミット対象) / m3b_stage_table.csv(フェーズ1 内訳表) /
      m3b_overlay_*.png(フェーズ0(c) 目視重畳) / m3b_state.npz・m3b_history.npz(ローカル)
"""
import argparse
import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
M3_DIR = Path(__file__).resolve().parents[1] / "m3_realquery"
A1_DIR = Path(__file__).resolve().parents[1] / "a1_synthetic"
sys.path.insert(0, str(M3_DIR))

import m3_run  # noqa: E402  # 内部で phase4 / a1_synthetic を sys.path に積む
import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera, render_image  # noqa: E402
from stage2_smoke_test import (  # noqa: E402
    render_depth_alpha_distorted, PLY_PATH, TOML_PATH, NEAR_PLANE,
)
from stage3_perturb_match import sample_depth_bilinear, perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import depth_variance_map  # noqa: E402
from pipeline_loop import (  # noqa: E402
    sched, GATE_VAR_WINDOW, GATE_VAR_REL, GATE_ALPHA, GATE_Z_RANGE,
    N_MIN, EPS_ROT_DEG, EPS_POS_M,
)

OUT_DIR = Path(__file__).resolve().parent
TMP_DIR = OUT_DIR / "tmp"
IMAGES_DIR = ROOT / "phase0" / "data" / "osaka2" / "images"
RESULT_PATH = OUT_DIR / "m3b_result.txt"
STATE_PATH = OUT_DIR / "m3b_state.npz"
TABLE_PATH = OUT_DIR / "m3b_stage_table.csv"

CAMERAS = m3_run.CAMERAS
K_MAX = m3_run.K_MAX

# criteria §2: TOML の同一性固定
TOML_SHA256 = "df3b52184f5b489e99694a98814c2bd1acd3b2edd0370dc36930f4387b4bf0cc"

# criteria §3.1: LoFTR 重み(ローカルキャッシュ)の固定
LOFTR_WEIGHTS = {
    "loftr_outdoor": (
        Path.home() / ".cache/torch/hub/checkpoints/loftr_outdoor.ckpt",
        "21f5bec5968178e8bc8b7633441836fe5de4f47d861dd2cd7dc38e271b0479ec"),
    "loftr_indoor_new": (
        Path.home() / ".cache/torch/hub/checkpoints/loftr_indoor_ds_new.ckpt",
        "be9ff88b323ec27889114719f668ae41aff7034b56a4c4acbd46b8b180b87ed3"),
}
# criteria §3.2: MASt3R チェックポイント(サイズで固定)
MAST3R_CKPT = (Path.home() / "data/models/mast3r"
               / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")
MAST3R_CKPT_BYTES = 2754910614

# criteria §3.1: confidence 閾値(事後追加禁止)
LOFTR_THRESHOLDS = [0.0, 0.2, 0.5, 0.8]
LOFTR_MATCHERS = ["loftr_outdoor", "loftr_indoor_new"]
ALL_MATCHERS = LOFTR_MATCHERS + ["mast3r"]

# criteria §3.3: SIFT 比較ベースライン(M3 実測 P0 k=0。再計測せず固定値)
SIFT_BASELINE = {
    "cam05520126": (300, 234),
    "cam05520129": (446, 340),
    "cam41520554": (291, 244),
    "cam41520556": (380, 296),
    "cam41520557": (258, 231),
}

# criteria フェーズ0(c): 逆写像の数値基準
RT_MAX_PX = 0.5
IN_RANGE_REQ = 1.0

SEED_M3B = 200  # フェーズ2 RANSAC seed 基点(M3 の 100〜149 と重ならない)


class MatcherOOM(Exception):
    """matcher_cli が exit code 42(CUDA OOM)を返した。発生元マッチャーを保持する"""

    def __init__(self, matcher: str):
        super().__init__(matcher)
        self.matcher = matcher


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def call_matcher(query_png: Path, render_png: Path, matcher: str,
                 resolution: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """matcher_cli.py を matcher_lab 環境の subprocess で実行し結果を読む"""
    out_npz = TMP_DIR / f"matches_{matcher}.npz"
    cmd = ["uv", "run", "--project", str(ROOT / "matcher_lab"), "python",
           str(OUT_DIR / "matcher_cli.py"), str(query_png), str(render_png),
           "--matcher", matcher, "--out", str(out_npz), "--resolution", resolution]
    res = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
    if res.returncode == 42:
        raise MatcherOOM(matcher)
    if res.returncode != 0:
        raise RuntimeError(f"matcher_cli 失敗({matcher}): {res.stderr[-2000:]}")
    d = np.load(out_npz, allow_pickle=False)
    meta = json.loads(str(d["meta"]))
    return (d["u_q"].astype(np.float64), d["u_r"].astype(np.float64),
            d["conf"].astype(np.float64), meta)


def check_meta(matcher: str, meta: dict, resolution: str) -> None:
    """逆写像・範囲の実行時検証(criteria フェーズ0(c) の数値基準を全呼び出しに適用)。

    違反は座標変換の実装欠陥として即中断する(フェーズ途中で fallback に切替わった
    場合も、この検証が fallback 逆写像に対して必ず走る。codex-59 指摘)
    """
    if meta["in_range_rate"] != IN_RANGE_REQ:
        raise RuntimeError(f"{matcher}: 原寸範囲内率 {meta['in_range_rate']:.4f} < 1.0"
                           "(座標変換の実装欠陥として中断)")
    resized = (matcher == "mast3r") or resolution != "full"
    if resized and meta["roundtrip_max_px"] > RT_MAX_PX:
        raise RuntimeError(f"{matcher}: round-trip {meta['roundtrip_max_px']:.4f}px > "
                           f"{RT_MAX_PX}(座標変換の実装欠陥として中断)")


def _backup(path: Path) -> None:
    """再実行時に旧結果を上書きせず退避する(criteria §7。codex-59 指摘)"""
    if path.exists():
        i = 1
        while (b := path.with_name(path.name + f".bak{i}")).exists():
            i += 1
        path.rename(b)


def gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, tau_px) -> dict:
    """変位→深度系ゲートを適用し、段別内訳(逐次分類: depth_ok→α→z→分散)と keep を返す。

    ゲート条件は m3_run.run_iteration_real の Stage 4 と同一(順序も同じ結果になる
    AND 結合。内訳のみ逐次分類で記録する。M4-1 と同方式)
    """
    raw = len(u_r)
    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r)
    disp_ok = np.linalg.norm(u_q - u_r, axis=1) < tau_px
    alpha_ok = alpha[iy, ix] > GATE_ALPHA
    z_ok = (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1])
    var_ok = var_map[iy, ix] < tau_d ** 2
    keep = disp_ok & depth_ok & alpha_ok & z_ok & var_ok
    # 逐次分類(変位通過分のみを分母に、depth_ok→α→z→分散 の順で最初に落ちた段に計上)
    surv = disp_ok
    excl_nan = surv & ~depth_ok
    surv = surv & depth_ok
    excl_alpha = surv & ~alpha_ok
    surv = surv & alpha_ok
    excl_z = surv & ~z_ok
    surv = surv & z_ok
    excl_var = surv & ~var_ok
    return {"raw": raw, "after_disp": int(disp_ok.sum()),
            "excl_nan_spread": int(excl_nan.sum()), "excl_alpha": int(excl_alpha.sum()),
            "excl_z": int(excl_z.sum()), "excl_var": int(excl_var.sum()),
            "N": int(keep.sum()), "keep": keep, "d_bilin": d_bilin}


def run_iteration_matcher(gaussians, cam, query_png: Path, matcher: str, conf_th: float,
                          resolution: str, rvec_cur, tvec_cur, k):
    """反復1回分。m3_run.run_iteration_real の Stage 3 をマッチャー CLI に差し替えた版。

    Stage 2/4/5/6 の処理・定数・順序は run_iteration_real と同一に保つこと。
    """
    p = sched(k)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur
    K, D = cam["K"], cam["D"]

    # Stage 2: レンダ(方式A。run_iteration_real と同一)
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)

    # Stage 3: 学習ベースマッチャー(CLI 経由)
    render_png = TMP_DIR / "render_cur.png"
    cv2.imwrite(str(render_png), bgr)
    u_q, u_r, conf, meta = call_matcher(query_png, render_png, matcher, resolution)
    check_meta(matcher, meta, resolution)
    if matcher in LOFTR_MATCHERS:
        sel = conf >= conf_th
        u_q, u_r = u_q[sel], u_r[sel]
    diag = {"k": k, "tau_px": p["tau_px"], "ransac_px": p["ransac_px"],
            "matches": len(u_q)}
    if len(u_q) == 0:
        diag["fail"] = "マッチ0件"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 4: ゲート(run_iteration_real と同一条件)
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, p["tau_px"])
    keep, d_bilin = bd["keep"], bd["d_bilin"]
    N = bd["N"]
    diag["N"] = N
    diag["breakdown"] = {kk: bd[kk] for kk in
                         ("raw", "after_disp", "excl_nan_spread", "excl_alpha",
                          "excl_z", "excl_var", "N")}
    if N < N_MIN:
        diag["fail"] = f"ゲート生存 N={N} < {N_MIN}"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 5: 深度3D化(run_iteration_real と同一)
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack([u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec_cur).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_cur).reshape(1, 3)) @ R_cur

    # Stage 6: PnP-RANSAC + 精密化(run_iteration_real と同一)
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

    # 診断値(m3_run と同一。§5.5)
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
        (f"RMSE>{m3_run.WARN_RMSE_PX}px", diag["rmse"] > m3_run.WARN_RMSE_PX),
        (f"vol_ratio<{m3_run.WARN_VOL_RATIO}", diag["vol_ratio"] < m3_run.WARN_VOL_RATIO),
        (f"depth_range<{m3_run.WARN_DEPTH_RANGE_M}m",
         diag["depth_range"] < m3_run.WARN_DEPTH_RANGE_M),
        (f"area_ratio<{m3_run.WARN_AREA_RATIO}", diag["area_ratio"] < m3_run.WARN_AREA_RATIO),
    ] if bad]
    return True, (rvec, tvec), diag


def render_p0(gaussians, cam):
    """基準ポーズでの Stage 2 レンダ(フェーズ0/1 用)"""
    cam_cur = dict(cam)
    cam_cur["rvec"] = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    cam_cur["tvec"] = np.asarray(cam["tvec"], dtype=np.float64).flatten()
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)
    return bgr, depth, alpha


def save_overlay(query_png: Path, render_png: Path, u_q, u_r, out_path: Path) -> None:
    """フェーズ0(c) 目視重畳: クエリ|レンダ横並び+対応線(最大80本を等間隔抽出)"""
    img_q = cv2.imread(str(query_png), cv2.IMREAD_COLOR)
    img_r = cv2.imread(str(render_png), cv2.IMREAD_COLOR)
    h, w = img_q.shape[:2]
    canvas = np.hstack([img_q, img_r])
    n = len(u_q)
    idx = np.linspace(0, n - 1, min(n, 80)).astype(int) if n else []
    rng = np.random.default_rng(0)
    for i in idx:
        c = tuple(int(v) for v in rng.integers(64, 255, 3))
        p1 = (int(round(u_q[i, 0])), int(round(u_q[i, 1])))
        p2 = (int(round(u_r[i, 0])) + w, int(round(u_r[i, 1])))
        cv2.circle(canvas, p1, 4, c, 1)
        cv2.circle(canvas, p2, 4, c, 1)
        cv2.line(canvas, p1, p2, c, 1)
    cv2.imwrite(str(out_path), canvas)


def phase0(gaussians, cams_cfg, lines: list) -> dict:
    """フェーズ0: 疎通(criteria §5)。戻り値: {"ok": bool, "loftr_resolution": str}"""
    lines.append("=== フェーズ0: 疎通 ===")
    ok_all = True

    # 前提: TOML・重み・チェックポイントの同一性(criteria §2/§3、フェーズ0(b))
    toml_sha = sha256_file(Path(TOML_PATH))
    lines.append(f"TOML sha256: {toml_sha} → {'一致' if toml_sha == TOML_SHA256 else '不一致'}")
    ok_all &= toml_sha == TOML_SHA256
    for name, (path, sha) in LOFTR_WEIGHTS.items():
        got = sha256_file(path) if path.exists() else "(ファイルなし)"
        lines.append(f"{name} sha256: {got} → {'一致' if got == sha else '不一致'}")
        ok_all &= got == sha
    msize = MAST3R_CKPT.stat().st_size if MAST3R_CKPT.exists() else -1
    lines.append(f"mast3r ckpt サイズ: {msize} → "
                 f"{'一致' if msize == MAST3R_CKPT_BYTES else '不一致'}")
    ok_all &= msize == MAST3R_CKPT_BYTES

    # (a) SIFT 再現(全カメラ完全一致が条件)
    lines.append("(a) SIFT 再現(M3 実測 P0 k=0 との一致):")
    for ci, name in enumerate(CAMERAS):
        cam = select_camera(cams_cfg, name)
        img_q_gray = cv2.imread(str(IMAGES_DIR / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        cv2.setRNGSeed(m3_run.SEED_CV_BASE + ci * 10 + 0)  # M3 P0 と同一 seed
        _, _, diag = m3_run.run_iteration_real(
            gaussians, cam, img_q_gray,
            np.asarray(cam["rvec"], dtype=np.float64).flatten(),
            np.asarray(cam["tvec"], dtype=np.float64).flatten(), 0)
        exp_raw, exp_n = SIFT_BASELINE[name]
        same = (diag["matches"] == exp_raw and diag.get("N", 0) == exp_n)
        lines.append(f"  {name}: マッチ{diag['matches']}(期待{exp_raw}) "
                     f"N={diag.get('N', 0)}(期待{exp_n}) → {'一致' if same else '不一致'}")
        ok_all &= same

    # (b)(c)(d) マッチャー疎通・逆写像・再現性(cam05520126 の実ペア)
    name = "cam05520126"
    cam = select_camera(cams_cfg, name)
    bgr, _, _ = render_p0(gaussians, cam)
    render_png = TMP_DIR / f"render_{name}.png"
    cv2.imwrite(str(render_png), bgr)
    query_png = IMAGES_DIR / f"{name}.png"

    # 収集。OOM は発生元マッチャー別に処理する(codex-59 指摘):
    # - LoFTR の OOM(full 時) → 全設定・全カメラを fallback に統一して最初から取り直す(criteria §3.1)
    # - MASt3R の OOM → LoFTR の fallback とは無関係。MASt3R の疎通不成立として明示記録
    loftr_resolution = "full"
    while True:
        mast3r_oom = False
        try:
            results = {}
            for matcher in ALL_MATCHERS:
                res = loftr_resolution if matcher in LOFTR_MATCHERS else "full"
                try:
                    results[matcher] = call_matcher(query_png, render_png, matcher, res)
                except MatcherOOM as e:
                    if e.matcher in LOFTR_MATCHERS and loftr_resolution == "full":
                        raise  # 外側で fallback に切替えて全体をやり直す
                    if e.matcher == "mast3r":
                        mast3r_oom = True
                        results[matcher] = None
                    else:
                        raise RuntimeError(f"fallback 解像度でも OOM: {e.matcher}(中断)")
            break
        except MatcherOOM as e:
            loftr_resolution = "1280x720"
            lines.append(f"(b) {e.matcher}: full で OOM → LoFTR 全設定・全カメラを "
                         "1280x720 に統一")
    if mast3r_oom:
        lines.append("(b) mast3r: CUDA OOM → MASt3R の疎通不成立として記録"
                     "(LoFTR の fallback とは独立)")
        ok_all = False

    # チェックと記録(確定した解像度での結果に対して行う)
    for matcher in ALL_MATCHERS:
        if results[matcher] is None:
            continue
        u_q, u_r, conf, meta = results[matcher]
        got = len(u_q) > 0 and meta["in_range_rate"] == IN_RANGE_REQ
        lines.append(f"(b) {matcher}: {len(u_q)} マッチ, 範囲内率 {meta['in_range_rate']:.3f}"
                     f" → {'OK' if got else 'NG'}")
        ok_all &= got
        # (c) 逆写像の数値基準(縮小系のみ round-trip を判定。full は恒等写像)
        needs_rt = (matcher == "mast3r") or (matcher in LOFTR_MATCHERS
                                             and loftr_resolution != "full")
        if needs_rt:
            rt = meta["roundtrip_max_px"]
            lines.append(f"(c) {matcher}: round-trip 最大 {rt:.4f}px"
                         f"(基準 ≤{RT_MAX_PX}) → {'OK' if rt <= RT_MAX_PX else 'NG'}"
                         f" / 変換メタ {json.dumps(meta, ensure_ascii=False)}")
            ok_all &= rt <= RT_MAX_PX
        save_overlay(query_png, render_png, u_q, u_r,
                     OUT_DIR / f"m3b_overlay_{matcher}.png")

    lines.append("(d) 再現性(同一入力2回目との差。記録のみ):")
    for matcher in ALL_MATCHERS:
        if results[matcher] is None:
            continue
        res = loftr_resolution if matcher in LOFTR_MATCHERS else "full"
        u_q2, u_r2, _, _ = call_matcher(query_png, render_png, matcher, res)
        u_q1, u_r1 = results[matcher][0], results[matcher][1]
        if len(u_q1) == len(u_q2):
            dmax = float(max(np.abs(u_q1 - u_q2).max(initial=0),
                             np.abs(u_r1 - u_r2).max(initial=0)))
            lines.append(f"  {matcher}: マッチ数同一({len(u_q1)}) 座標最大差 {dmax:.4f}px")
        else:
            lines.append(f"  {matcher}: マッチ数差あり {len(u_q1)} vs {len(u_q2)}")

    lines.append(f"フェーズ0 判定: {'成立' if ok_all else '不成立(中断)'} "
                 f"/ LoFTR 解像度={loftr_resolution}")
    return {"ok": ok_all, "loftr_resolution": loftr_resolution}


def select_config(qual: list) -> tuple:
    """criteria §5 フェーズ2 採用設定の選択規則(一意に閉じる4段タイブレーク)。

    qual: [(matcher, th, {cam: N}), ...]。LoFTR 用(MASt3R は1設定のため不要)
    """
    def key(entry):
        matcher, th, ns = entry
        vals = [ns[c] for c in CAMERAS]
        # (1)min N 最大 → (2)合計最大 → (3)閾値高い → (4)indoor_new 優先
        return (min(vals), sum(vals), th, 1 if matcher == "loftr_indoor_new" else 0)
    return max(qual, key=key)


def phase1(gaussians, cams_cfg, lines: list, loftr_resolution: str) -> dict:
    """フェーズ1: マッチング段のオフライン比較(criteria §5)"""
    lines.append("=== フェーズ1: マッチング段のオフライン比較 ===")
    tau_px = sched(0)["tau_px"]
    rows = []  # CSV 行
    per_config = {}  # (matcher, th) -> {cam: breakdown}

    raw_matches = {}  # (matcher, cam) -> (u_q, u_r, conf)
    scene = {}       # cam -> (depth, alpha, var_map, tau_d)
    mast3r_oom = False
    for name in CAMERAS:
        cam = select_camera(cams_cfg, name)
        bgr, depth, alpha = render_p0(gaussians, cam)
        render_png = TMP_DIR / f"render_{name}.png"
        cv2.imwrite(str(render_png), bgr)
        pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                       & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
        var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
        tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
        scene[name] = (depth, alpha, var_map, tau_d)
        query_png = IMAGES_DIR / f"{name}.png"
        for matcher in ALL_MATCHERS:
            if matcher == "mast3r" and mast3r_oom:
                continue
            res = loftr_resolution if matcher in LOFTR_MATCHERS else "full"
            try:
                u_q, u_r, conf, meta = call_matcher(query_png, render_png, matcher, res)
            except MatcherOOM as e:
                # OOM は発生元別に処理(codex-59 指摘。LoFTR と MASt3R を混同しない)
                if e.matcher in LOFTR_MATCHERS and loftr_resolution == "full":
                    # criteria §3.1: 1ペアでも OOM → LoFTR 全設定・全カメラを統一して再取得
                    loftr_resolution = "1280x720"
                    lines.append(f"{name}/{e.matcher}: full で OOM → LoFTR 全体を "
                                 "1280x720 に統一して再取得(fallback 逆写像は"
                                 "各呼び出しの check_meta で検証)")
                    raw_matches = {k: v for k, v in raw_matches.items()
                                   if k[0] not in LOFTR_MATCHERS}
                    for pname in CAMERAS[:CAMERAS.index(name) + 1]:
                        for pm in LOFTR_MATCHERS:
                            pq, pr, pc, pmeta = call_matcher(
                                IMAGES_DIR / f"{pname}.png",
                                TMP_DIR / f"render_{pname}.png", pm, loftr_resolution)
                            check_meta(pm, pmeta, loftr_resolution)
                            raw_matches[(pm, pname)] = (pq, pr, pc)
                    continue
                if e.matcher == "mast3r":
                    mast3r_oom = True
                    lines.append(f"{name}/mast3r: CUDA OOM → MASt3R をフェーズ1 "
                                 "不成立(進出不可)として明示記録")
                    continue
                raise RuntimeError(f"fallback 解像度でも OOM: {e.matcher}(中断)")
            check_meta(matcher, meta, res)
            raw_matches[(matcher, name)] = (u_q, u_r, conf)

    # 段別内訳(LoFTR は閾値をオフライン適用、MASt3R は1設定)
    for matcher in ALL_MATCHERS:
        if matcher == "mast3r" and mast3r_oom:
            continue
        ths = LOFTR_THRESHOLDS if matcher in LOFTR_MATCHERS else [None]
        for th in ths:
            cfg = (matcher, th)
            per_config[cfg] = {}
            for name in CAMERAS:
                u_q, u_r, conf = raw_matches[(matcher, name)]
                if th is not None:
                    sel = conf >= th
                    u_qs, u_rs = u_q[sel], u_r[sel]
                else:
                    u_qs, u_rs = u_q, u_r
                depth, alpha, var_map, tau_d = scene[name]
                bd = gate_breakdown(u_qs, u_rs, depth, alpha, var_map, tau_d, tau_px)
                bd.pop("keep")
                bd.pop("d_bilin")
                per_config[cfg][name] = bd
                rows.append({"config": f"{matcher}" + (f"@{th}" if th is not None else ""),
                             "camera": name, **bd})

    # SIFT ベースライン行(criteria §3.3 固定値)
    for name in CAMERAS:
        raw, n = SIFT_BASELINE[name]
        rows.append({"config": "sift_m3_baseline", "camera": name, "raw": raw,
                     "after_disp": "", "excl_nan_spread": "", "excl_alpha": "",
                     "excl_z": "", "excl_var": "", "N": n})

    with open(TABLE_PATH, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["config", "camera", "raw", "after_disp",
                                          "excl_nan_spread", "excl_alpha", "excl_z",
                                          "excl_var", "N"])
        w.writeheader()
        w.writerows(rows)

    # 表のログ出力と進出判定
    for cfg, percam in per_config.items():
        matcher, th = cfg
        label = f"{matcher}" + (f"@{th}" if th is not None else "")
        ns = {c: percam[c]["N"] for c in CAMERAS}
        n_ok = sum(1 for c in CAMERAS if ns[c] >= N_MIN)
        lines.append(f"  {label}: N={[ns[c] for c in CAMERAS]} "
                     f"(N≥{N_MIN}: {n_ok}/5カメラ)")

    adopted = []
    # LoFTR(8設定から選択規則で1つ)
    loftr_qual = [(m, th, {c: per_config[(m, th)][c]["N"] for c in CAMERAS})
                  for m in LOFTR_MATCHERS for th in LOFTR_THRESHOLDS
                  if sum(1 for c in CAMERAS
                         if per_config[(m, th)][c]["N"] >= N_MIN) >= 3]
    if loftr_qual:
        m, th, ns = select_config(loftr_qual)
        adopted.append(("loftr", m, th))
        lines.append(f"LoFTR: 進出({len(loftr_qual)}設定が適格) → 採用 {m}@{th} "
                     f"(選択規則 §5: minN={min(ns.values())})")
    else:
        lines.append("LoFTR: 進出条件未達(全8設定で N≥30 が3カメラ未満)")
    # MASt3R(1設定)
    if mast3r_oom:
        lines.append("MASt3R: フェーズ1 で CUDA OOM のため進出不可(明示的不成立として記録)")
    else:
        ns_m = {c: per_config[("mast3r", None)][c]["N"] for c in CAMERAS}
        n_ok_m = sum(1 for c in CAMERAS if ns_m[c] >= N_MIN)
        if n_ok_m >= 3:
            adopted.append(("mast3r", "mast3r", None))
            lines.append(f"MASt3R: 進出(N≥{N_MIN} が {n_ok_m}/5)")
        else:
            lines.append("MASt3R: 進出条件未達")

    lines.append(f"フェーズ1 判定: 進出マッチャー {[a[0] for a in adopted] or 'なし(打ち切り)'}")
    return {"adopted": adopted, "loftr_resolution": loftr_resolution,
            "table": per_config}


def phase2(gaussians, cams_cfg, lines: list, adopted: list, loftr_resolution: str) -> dict:
    """フェーズ2: E2E(P0/P1/P2)。判定は M3 §5.1/5.2/5.3 と同一(criteria §5/§6)"""
    lines.append("=== フェーズ2: E2E(P0/P1/P2) ===")
    series_def = [("P0", None, None, None), ("P1", 0.05, 2.0, 11), ("P2", 0.05, 2.0, 12)]
    all_results = {}
    for mi, (mkey, matcher, th) in enumerate(adopted):
        label = f"{matcher}" + (f"@{th}" if th is not None else "")
        lines.append(f"--- マッチャー: {label} ---")
        results = {}
        for ci, name in enumerate(CAMERAS):
            cam = select_camera(cams_cfg, name)
            rvec_base = np.asarray(cam["rvec"], dtype=np.float64).flatten()
            tvec_base = np.asarray(cam["tvec"], dtype=np.float64).flatten()
            query_png = IMAGES_DIR / f"{name}.png"
            res = loftr_resolution if matcher in LOFTR_MATCHERS else "full"
            lines.append(f"=== {name} ===")
            cam_res = {"series": {}}
            for si, (sname, ppos, pang, seed) in enumerate(series_def):
                cv2.setRNGSeed(SEED_M3B + mi * 100 + ci * 10 + si)
                if ppos is None:
                    rvec0, tvec0 = rvec_base, tvec_base
                    lines.append(f"[{sname}] 摂動なし(基準ポーズ)")
                else:
                    rng = np.random.default_rng(seed)
                    rvec0, tvec0 = perturb_pose(rvec_base, tvec_base, ppos, pang, rng)
                    lines.append(f"[{sname}] 摂動 {ppos*100:.0f}cm/{pang:.0f}deg seed={seed}")
                rvec_cur, tvec_cur = rvec0.copy(), tvec0.copy()
                history, converged = [], False
                for k in range(K_MAX):
                    ok, (rv, tv), diag = run_iteration_matcher(
                        gaussians, cam, query_png, matcher, th if th is not None else 0.0,
                        res, rvec_cur, tvec_cur, k)
                    dpos, dang = m3_run.pose_diff(rv, tv, rvec_cur, tvec_cur)
                    diag.update({"step_pos_m": dpos, "step_ang_deg": dang})
                    history.append(diag)
                    line = (f"  k={k}: " + ("OK" if ok else f"失敗({diag.get('fail')})")
                            + f" | マッチ{diag.get('matches', 0)} N={diag.get('N', 0)}"
                            + f" inl={diag.get('inliers', 0)}"
                            + (f" RMSE={diag['rmse']:.3f}px" if "rmse" in diag else "")
                            + f" | 更新量 {dpos*1000:.2f}mm/{dang:.4f}deg"
                            + (f" | 警告: {','.join(diag['warns'])}"
                               if diag.get("warns") else ""))
                    print(line)
                    lines.append(line)
                    if not ok:
                        break
                    rvec_cur, tvec_cur = rv, tv
                    if dang < EPS_ROT_DEG and dpos < EPS_POS_M:
                        converged = True
                        lines.append(f"  → 収束(更新量 < {EPS_ROT_DEG}deg / "
                                     f"{EPS_POS_M*1000:.0f}mm)")
                        break
                established = converged and all("fail" not in h for h in history)
                d_base, a_base = m3_run.pose_diff(rvec_cur, tvec_cur, rvec_base, tvec_base)
                cam_res["series"][sname] = {
                    "established": established, "rvec": rvec_cur, "tvec": tvec_cur,
                    "history": history, "diff_base_pos_m": d_base,
                    "diff_base_ang_deg": a_base}
                lines.append(f"  最終解と基準ポーズの差: {d_base*100:.3f}cm / {a_base:.4f}deg")
            # 判定(M3 §5.1/5.2/5.3 と同一)
            p0 = cam_res["series"]["P0"]
            cam_res["established"] = p0["established"]
            trio = [cam_res["series"][s] for s in ("P0", "P1", "P2")]
            if all(s["established"] for s in trio):
                max_dp = max(m3_run.pose_diff(trio[i]["rvec"], trio[i]["tvec"],
                                              trio[j]["rvec"], trio[j]["tvec"])[0]
                             for i in range(3) for j in range(i + 1, 3))
                max_da = max(m3_run.pose_diff(trio[i]["rvec"], trio[i]["tvec"],
                                              trio[j]["rvec"], trio[j]["tvec"])[1]
                             for i in range(3) for j in range(i + 1, 3))
                cam_res["same_solution"] = (max_dp <= m3_run.SAME_SOL_POS_M
                                            and max_da <= m3_run.SAME_SOL_ANG_DEG)
                lines.append(f"  同一解判定: 最大ペア間 {max_dp*100:.3f}cm/{max_da:.4f}deg → "
                             f"{'同一解' if cam_res['same_solution'] else '不一致'}")
            else:
                cam_res["same_solution"] = False
                lines.append("  同一解判定: P0/P1/P2 に不成立系列があるため不合格")
            th_p, th_a = m3_run.P95_THRESHOLDS[name]
            cam_res["agree"] = (p0["established"]
                                and p0["diff_base_pos_m"] <= th_p
                                and p0["diff_base_ang_deg"] <= th_a)
            lines.append(f"  一致性判定: P0差 {p0['diff_base_pos_m']*100:.3f}cm/"
                         f"{p0['diff_base_ang_deg']:.4f}deg vs 閾値 "
                         f"{th_p*100:.2f}cm/{th_a:.2f}deg → "
                         f"{'一致' if cam_res['agree'] else '不一致'}")
            cam_res["pass"] = (cam_res["established"] and cam_res["same_solution"]
                               and cam_res["agree"])
            lines.append(f"  カメラ判定: 成立={cam_res['established']} "
                         f"同一解={cam_res['same_solution']} 一致={cam_res['agree']} → "
                         f"{'PASS' if cam_res['pass'] else 'FAIL'}")
            results[name] = cam_res
        n_pass = sum(1 for r in results.values() if r["pass"])
        lines.append(f"{label} 判定: PASS {n_pass}/5")
        all_results[label] = {"results": results, "n_pass": n_pass}
    go = any(v["n_pass"] >= 3 for v in all_results.values())
    lines.append(f"=== M3b 判定(criteria §6.2) === {'Go' if go else 'No-Go'}")
    return all_results


def _flag(state: dict, key: str) -> bool:
    return bool(np.asarray(state.get(key, False)))


def _guard_phase(state: dict, key: str, rerun_defect, lines: list) -> None:
    """criteria §7: 各フェーズ1回。再実行は実装欠陥の特定・修正・記録がある場合のみ許可"""
    if _flag(state, key):
        if not rerun_defect:
            raise RuntimeError(
                f"{key} は実行済み(criteria §7: 各フェーズ1回)。実装欠陥による再実行は "
                "--rerun-defect '<欠陥の特定・修正内容>' を付けて明示すること")
        lines.append(f"※{key} 再実行(criteria §7 実装欠陥の記録): {rerun_defect}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="all", choices=["all", "0", "1", "2"])
    ap.add_argument("--rerun-defect", default=None,
                    help="criteria §7: 実装欠陥による再実行時に、欠陥の特定・修正内容を記す")
    args = ap.parse_args()
    TMP_DIR.mkdir(exist_ok=True)

    lines = [
        "feat-026 M3b: 学習ベースマッチャー(LoFTR/MASt3R)の病院シーン評価",
        f"criteria: criteria.md(locked 2026-08-02) / K_MAX={K_MAX} / N_MIN={N_MIN}",
        f"入力: PLY={PLY_PATH}",
        f"      TOML={TOML_PATH}",
        f"バージョン: OpenCV {cv2.__version__} / NumPy {np.__version__}",
        "",
    ]

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    cams_cfg = load_cameras_toml(TOML_PATH)

    state = {}
    if STATE_PATH.exists():
        state = {k: v for k, v in np.load(STATE_PATH, allow_pickle=True).items()}

    # 試行回数管理(criteria §7)は attempted フラグを「実行前」に永続化する:
    # 中断(OOM 等)でも1回の試行として消費され、--rerun-defect なしの再実行は
    # どの終わり方でも拒否される(codex-60 指摘)
    if args.phase in ("all", "0"):
        _guard_phase(state, "phase0_attempted", args.rerun_defect, lines)
        state["phase0_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        st0 = phase0(gaussians, cams_cfg, lines)
        state["phase0_ok"] = np.array(st0["ok"])
        state["loftr_resolution"] = np.array(st0["loftr_resolution"])
        np.savez(STATE_PATH, **state)
        if not st0["ok"]:
            lines.append("フェーズ0 不成立のため中断(criteria §5)")
            _flush(lines)
            return

    if args.phase in ("all", "1"):
        if not _flag(state, "phase0_ok"):
            raise RuntimeError("フェーズ0 が未成立(criteria §5: 順序固定)")
        rerun1 = _flag(state, "phase1_attempted")
        _guard_phase(state, "phase1_attempted", args.rerun_defect, lines)
        if rerun1:
            _backup(TABLE_PATH)  # 旧結果を上書きしない(codex-59 指摘)
        state["phase1_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        st1 = phase1(gaussians, cams_cfg, lines,
                     str(state["loftr_resolution"]))
        state["loftr_resolution"] = np.array(st1["loftr_resolution"])
        state["adopted"] = np.array(st1["adopted"], dtype=object)
        np.savez(STATE_PATH, **state)
        if not st1["adopted"]:
            lines.append("フェーズ1: 進出マッチャーなしのため打ち切り(criteria §5)")
            _flush(lines)
            return

    if args.phase in ("all", "2"):
        adopted = [tuple(a) for a in state.get("adopted", np.empty(0)).tolist()]
        if not adopted:
            raise RuntimeError("フェーズ1 の進出マッチャーがない(criteria §5: 順序固定)")
        rerun2 = _flag(state, "phase2_attempted")
        _guard_phase(state, "phase2_attempted", args.rerun_defect, lines)
        if rerun2:
            _backup(OUT_DIR / "m3b_history.npz")  # 旧結果を上書きしない
        state["phase2_attempted"] = np.array(True)
        np.savez(STATE_PATH, **state)
        adopted = [(m, w, (float(t) if t is not None else None)) for m, w, t in adopted]
        try:
            res2 = phase2(gaussians, cams_cfg, lines, adopted,
                          str(state["loftr_resolution"]))
        except MatcherOOM as e:
            # フェーズ2 の OOM はフェーズ1 と解像度整合が取れなくなるため、
            # 途中切替せず想定外事象として中断・報告する(codex-59 指摘)。
            # attempted は実行前に立っているため、この後の安易な再実行は拒否される
            state["phase2_aborted_oom"] = np.array(True)
            np.savez(STATE_PATH, **state)
            lines.append(f"フェーズ2: {e.matcher} で想定外の CUDA OOM。フェーズ1 との"
                         "解像度整合が取れないため中断(想定外事象として報告)")
            _flush(lines)
            raise
        np.savez(STATE_PATH, **state)
        np.savez(OUT_DIR / "m3b_history.npz", results=np.array([res2], dtype=object))

    _flush(lines)


def _flush(lines: list) -> None:
    with open(RESULT_PATH, "a") as f:
        f.write("\n".join(lines) + "\n\n")
    print(f"保存: {RESULT_PATH}")


if __name__ == "__main__":
    main()

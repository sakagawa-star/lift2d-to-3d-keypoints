"""feat-041: レンダ＋マッチング反復による内部パラメータ（K・歪み）と外部パラメータの同時推定

処理の流れ:
  FR-002: 手動点PnPで初期外部パラメータ
  FR-003〜FR-005: 反復（レンダ → マッチ（i=0 は MASt3R、i≥1 は LoFTR） → ゲート → 深度リフト → 外れ値除去 →
                  同時最小二乗）を内部パラメータが収束するまで回す
  FR-006: 収束後のパラメータで refine_extrinsics.py を subprocess で呼び外部パラメータを精緻化
  FR-007: 実行レポート
  FR-008: 最終TOMLの重ね合わせ評価

実行方法（プロジェクトルートで）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py \\
        --init-toml <初期TOML> --camera <カメラ名> --ply <PLY> --images-dir <前処理後画像ディレクトリ> \\
        --points-3d <3D基準点CSV> --points-2d <前処理後座標の2D点CSV> --out-dir <出力ディレクトリ> \\
        [--tmp-dir <中間ファイル置き場>]
    設定YAMLで指定する場合（feat-042）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py --config run.yaml
    前処理（prepare_phone_photo.py）と共用の設定YAMLも渡せる（前処理専用のキーは無視する。feat-043）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py --config job.yaml

終了コード: 0=成功、1=入力エラー・環境異常・出力済み、2=引数エラー、
            3=ループ失敗（点数不足・RANSAC失敗・OOM・未収束・範囲外）、4=最終精緻化で不受理

設計根拠: docs/issues/feat-041-intrinsic-loop-render-match/design.md §4.2
"""
import argparse
import datetime
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import scipy.optimize
import tomli

from evaluate_overlay import eval_output_paths, evaluate_and_write
from refine_extrinsics import (
    GATE_ALPHA,
    GATE_VAR_REL,
    GATE_VAR_WINDOW,
    GATE_Z_RANGE,
    MATCHER_LAB_DIR,
    NEAR_PLANE,
    ROOT,
    LOFTR_CONF_TH,
    InputError,
    MatcherEnvError,
    MatcherOOM,
    _format_toml_value,
    _yaml_str,
    call_matcher,
    check_matcher_meta,
    depth_variance_map,
    gate_breakdown,
    load_points_2d_csv,
    load_points_3d_csv,
    median_reproj_px,
    parse_args_with_config,
    pnp_manual,
    render_depth_alpha_distorted,
    validate_camera,
)
from render_keypoints import load_cameras_toml, render_image, select_camera

IMG_W, IMG_H = 1920, 1080
TAU_PX = {0: 250.0, 1: 30.0}
RANSAC_PX = {0: 8.0, 1: 3.0}
PNP_RANSAC_ITER = 1000
N_MIN_INTR = 300
LSQ_LOSS = "soft_l1"
LSQ_F_SCALE = 1.0
LSQ_MAX_NFEV = 200
CONV_PX = 2.0
LOFTR_FROM_ITER = 1
MAX_ITER = 8
F_RANGE = (0.3 * 1920, 3.0 * 1920)
CX_TOL, CY_TOL = 192.0, 108.0
K_ABS_MAX = 1.0
GRID_CELLS = (4, 4)
CELL_MIN_PTS = 10
COVER_WARN = 12
SEED_BASE = 7000
DELTA_GRID_N = 5
UNDIST_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-10)
LIFT_ROUNDTRIP_PX = 0.01
REFINE_UNDIST_GRID = (33, 19)
REFINE_UNDIST_MAX_PX = 0.1


class LoopFailure(Exception):
    """ループ失敗（終了コード3）"""


@dataclass
class Params:
    f: float
    cx: float
    cy: float
    k1: float
    k2: float
    rvec: np.ndarray
    tvec: np.ndarray

    def K(self) -> np.ndarray:
        return np.array([[self.f, 0.0, self.cx], [0.0, self.f, self.cy], [0.0, 0.0, 1.0]],
                        dtype=np.float64)

    def D(self) -> np.ndarray:
        return np.array([self.k1, self.k2, 0.0, 0.0], dtype=np.float64)

    def to_cam(self) -> dict:
        return {"K": self.K(), "D": self.D(),
                "rvec": np.asarray(self.rvec, dtype=np.float64).reshape(3),
                "tvec": np.asarray(self.tvec, dtype=np.float64).reshape(3),
                "width": IMG_W, "height": IMG_H}

    def to_vector(self) -> np.ndarray:
        return np.concatenate([[self.f, self.cx, self.cy, self.k1, self.k2],
                               np.asarray(self.rvec, dtype=np.float64).reshape(3),
                               np.asarray(self.tvec, dtype=np.float64).reshape(3)])

    @staticmethod
    def from_vector(x: np.ndarray) -> "Params":
        x = np.asarray(x, dtype=np.float64).reshape(-1)
        return Params(float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]),
                      x[5:8].copy(), x[8:11].copy())


# ============================================================
# MASt3R 呼び出し
# ============================================================

def call_mast3r(query_png: Path, render_png: Path, out_npz: Path):
    """matcher_lab 環境の mast3r_cli.py を subprocess で実行し NPZ を読む。
    画像Aに写真、画像Bにレンダを渡す。戻り値: (u_q, u_r, meta)"""
    import json

    def _run(res: str):
        cmd = ["uv", "run", "--project", str(MATCHER_LAB_DIR), "python",
               str(MATCHER_LAB_DIR / "mast3r_cli.py"), str(query_png), str(render_png),
               "--out", str(out_npz), "--resolution", res]
        return subprocess.run(cmd, capture_output=True, text=True)

    r = _run("full")
    if r.returncode == 42:
        print("  MASt3R: CUDA OOM。1280x720 で再試行します")
        r = _run("1280x720")
        if r.returncode == 42:
            raise LoopFailure("MASt3R OOM")
    if r.returncode != 0:
        raise MatcherEnvError(f"mast3r_cli.py が異常終了しました（環境異常）: {r.stderr[-2000:]}")
    d = np.load(out_npz, allow_pickle=False)
    meta = json.loads(str(d["meta"]))
    return d["u_a"].astype(np.float64), d["u_b"].astype(np.float64), meta


# ============================================================
# 深度リフト
# ============================================================

def lift_to_world(u_r: np.ndarray, d: np.ndarray, params: Params):
    """レンダ側のマッチ点と深度を3DGS座標系の3D点に戻す。戻り値: (X_w, 逆歪み検査の合格マスク)"""
    K, D = params.K(), params.D()
    pts = np.asarray(u_r, dtype=np.float64).reshape(-1, 1, 2)
    u_pin = cv2.undistortPointsIter(pts, K, D, None, K, UNDIST_CRITERIA).reshape(-1, 2)
    n = (np.linalg.inv(K) @ np.hstack([u_pin, np.ones((len(u_pin), 1))]).T).T
    proj, _ = cv2.projectPoints(n.reshape(-1, 3), np.zeros(3), np.zeros(3), K, D)
    ok_undist = np.linalg.norm(proj.reshape(-1, 2) - np.asarray(u_r).reshape(-1, 2),
                               axis=1) <= LIFT_ROUNDTRIP_PX
    X_cam = n * np.asarray(d, dtype=np.float64).reshape(-1, 1)
    R, _ = cv2.Rodrigues(np.asarray(params.rvec, dtype=np.float64).reshape(3, 1))
    X_w = (X_cam - np.asarray(params.tvec, dtype=np.float64).reshape(1, 3)) @ R
    return X_w, ok_undist


def refine_undistort_error_px(K: np.ndarray, D: np.ndarray) -> float:
    """refine_extrinsics.py が使う既定の cv2.undistortPoints の逆歪み誤差（格子点の最大値）"""
    gx = np.linspace(0, IMG_W - 1, REFINE_UNDIST_GRID[0])
    gy = np.linspace(0, IMG_H - 1, REFINE_UNDIST_GRID[1])
    g = np.array([[x, y] for y in gy for x in gx], dtype=np.float64)
    u = cv2.undistortPoints(g.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)
    n = (np.linalg.inv(K) @ np.hstack([u, np.ones((len(u), 1))]).T).T
    p, _ = cv2.projectPoints(n.reshape(-1, 3), np.zeros(3), np.zeros(3), K, D)
    return float(np.max(np.linalg.norm(p.reshape(-1, 2) - g, axis=1)))


# ============================================================
# 同時最小二乗
# ============================================================

def _residuals(x: np.ndarray, X: np.ndarray, u: np.ndarray) -> np.ndarray:
    f, cx, cy, k1, k2 = x[0:5]
    K = np.array([[f, 0.0, cx], [0.0, f, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([k1, k2, 0.0, 0.0], dtype=np.float64)
    proj, _ = cv2.projectPoints(X, x[5:8].reshape(3, 1), x[8:11].reshape(3, 1), K, D)
    return (proj.reshape(-1, 2) - u).ravel()


def joint_refine(X: np.ndarray, u: np.ndarray, x0: np.ndarray):
    """f, cx, cy, k1, k2, rvec, tvec の11変数の同時最小二乗。戻り値: (Params, status)"""
    assert X.shape == (len(X), 3) and u.shape == (len(X), 2)
    X = np.ascontiguousarray(X, dtype=np.float64)
    u = np.ascontiguousarray(u, dtype=np.float64)
    res = scipy.optimize.least_squares(
        _residuals, np.asarray(x0, dtype=np.float64), args=(X, u), method="trf",
        loss=LSQ_LOSS, f_scale=LSQ_F_SCALE, x_scale="jac", max_nfev=LSQ_MAX_NFEV)
    if not np.all(np.isfinite(res.x)):
        raise LoopFailure("最小二乗の結果が非有限")
    return Params.from_vector(res.x), int(res.status)


# ============================================================
# 収束判定・範囲チェック
# ============================================================

def delta_intr(old: Params, new: Params) -> float:
    """内部パラメータ変化量 Δ_intr[px]（5x5格子点の移動量の最大値）"""
    gx = np.linspace(0, IMG_W - 1, DELTA_GRID_N)
    gy = np.linspace(0, IMG_H - 1, DELTA_GRID_N)
    K_old, D_old, K_new, D_new = old.K(), old.D(), new.K(), new.D()
    zero = np.zeros(3)
    dmax = 0.0
    for y in gy:
        for x in gx:
            nx = (x - old.cx) / old.f
            ny = (y - old.cy) / old.f
            pt = np.array([[nx, ny, 1.0]], dtype=np.float64)
            p_old, _ = cv2.projectPoints(pt, zero, zero, K_old, D_old)
            p_new, _ = cv2.projectPoints(pt, zero, zero, K_new, D_new)
            dmax = max(dmax, float(np.linalg.norm(p_new.reshape(2) - p_old.reshape(2))))
    return dmax


def is_converged(i: int, delta: float) -> bool:
    """収束判定。LoFTR でマッチを求めた反復（i >= LOFTR_FROM_ITER）だけが収束と判定できる"""
    return i >= LOFTR_FROM_ITER and delta < CONV_PX


def check_ranges(p: Params) -> list[str]:
    errs: list[str] = []
    if p.f < F_RANGE[0] or p.f > F_RANGE[1]:
        errs.append(f"f={p.f:.2f} が範囲 {F_RANGE[0]}〜{F_RANGE[1]} 外")
    if abs(p.cx - IMG_W / 2) > CX_TOL:
        errs.append(f"cx={p.cx:.2f} が画像中心から {CX_TOL}px 超")
    if abs(p.cy - IMG_H / 2) > CY_TOL:
        errs.append(f"cy={p.cy:.2f} が画像中心から {CY_TOL}px 超")
    if abs(p.k1) > K_ABS_MAX:
        errs.append(f"|k1|={abs(p.k1):.4f} > {K_ABS_MAX}")
    if abs(p.k2) > K_ABS_MAX:
        errs.append(f"|k2|={abs(p.k2):.4f} > {K_ABS_MAX}")
    return errs


def count_covered_cells(u: np.ndarray) -> int:
    """1920x1080 を横4x縦4に等分し、CELL_MIN_PTS 点以上入っているセルの数"""
    u = np.asarray(u, dtype=np.float64).reshape(-1, 2)
    cw, ch = IMG_W / GRID_CELLS[0], IMG_H / GRID_CELLS[1]
    ix = np.clip(np.floor(u[:, 0] / cw).astype(int), 0, GRID_CELLS[0] - 1)
    iy = np.clip(np.floor(u[:, 1] / ch).astype(int), 0, GRID_CELLS[1] - 1)
    counts = np.zeros(GRID_CELLS[::-1], dtype=int)
    np.add.at(counts, (iy, ix), 1)
    return int((counts >= CELL_MIN_PTS).sum())


# ============================================================
# 反復1回分
# ============================================================

def _cv_wrap(name: str, fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except cv2.error as e:
        raise LoopFailure(f"OpenCV 例外: {name}: {str(e)[:200]}") from e


def run_loop_iteration(gaussians, params: Params, query_png: Path, i: int, tmp_dir: Path) -> dict:
    """反復1回分。失敗は LoopFailure。"""
    t0 = time.time()
    cam = params.to_cam()
    print(f"反復 i={i}: レンダ中...")
    bgr = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)
    render_png = Path(tmp_dir) / f"iter{i:02d}_render.png"
    cv2.imwrite(str(render_png), bgr)
    out_npz = Path(tmp_dir) / f"iter{i:02d}_matches.npz"
    if i < LOFTR_FROM_ITER:
        matcher_name = "MASt3R"
        print(f"反復 i={i}: {matcher_name} 実行中...")
        u_q, u_r, meta = call_mast3r(Path(query_png), render_png, out_npz)
    else:
        matcher_name = "LoFTR"
        print(f"反復 i={i}: {matcher_name} 実行中...")
        try:
            u_q, u_r, conf, meta = call_matcher(Path(query_png), render_png, out_npz)
        except MatcherOOM as e:
            raise LoopFailure("LoFTR OOM") from e
        sel = conf >= LOFTR_CONF_TH
        u_q, u_r = u_q[sel], u_r[sel]
    check_matcher_meta(meta)
    n_match = len(u_q)
    if n_match == 0:
        raise LoopFailure("マッチ0件")

    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, TAU_PX[min(i, 1)])
    keep, d = bd["keep"], bd["d_bilin"]
    if bd["N"] < N_MIN_INTR:
        raise LoopFailure(f"ゲート通過 N={bd['N']} < {N_MIN_INTR}")

    idx = np.flatnonzero(keep)
    ur, uq, dk = u_r[idx], u_q[idx], d[idx]
    n0 = len(idx)

    X_w_all, ok_undist = _cv_wrap("undistortPointsIter", lift_to_world, ur, dk, params)
    X_w = X_w_all[ok_undist]
    uq = uq[ok_undist]
    n1 = int(ok_undist.sum())
    excl_undist = n0 - n1
    if n1 < N_MIN_INTR:
        raise LoopFailure(f"逆歪み検査後 N={n1} < {N_MIN_INTR}")

    cv2.setRNGSeed(SEED_BASE + i)
    ok, rv, tv, inl = _cv_wrap(
        "solvePnPRansac", cv2.solvePnPRansac, X_w, uq, params.K(), params.D(),
        iterationsCount=PNP_RANSAC_ITER, reprojectionError=RANSAC_PX[min(i, 1)],
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None:
        raise LoopFailure("solvePnPRansac 失敗")
    inl = np.asarray(inl).flatten()
    if len(inl) < N_MIN_INTR:
        raise LoopFailure(f"外れ値除去後 {len(inl)} < {N_MIN_INTR}")
    X = X_w[inl].reshape(-1, 3)
    u = uq[inl].reshape(-1, 2)

    x0 = np.concatenate([[params.f, params.cx, params.cy, params.k1, params.k2],
                         np.asarray(rv, dtype=np.float64).reshape(3),
                         np.asarray(tv, dtype=np.float64).reshape(3)])
    new, status = _cv_wrap("projectPoints(joint_refine)", joint_refine, X, u, x0)
    proj, _ = _cv_wrap("projectPoints", cv2.projectPoints, X, new.rvec.reshape(3, 1),
                       new.tvec.reshape(3, 1), new.K(), new.D())
    rmse = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - u) ** 2, axis=1))))
    cover = count_covered_cells(u)
    diag = {"matcher": matcher_name, "matches": n_match, "N": bd["N"], "excl_undist": excl_undist,
            "inl": int(len(inl)), "rmse": rmse, "cover": cover,
            "lsq_status": status, "elapsed": time.time() - t0}
    return {"ok": True, "params": new, "diag": diag}


# ============================================================
# TOML・レポート
# ============================================================

def format_loop_toml(camera: str, p: Params) -> str:
    lines = [
        f"[{camera}]",
        f"name = {_format_toml_value(camera)}",
        f"size = {_format_toml_value([float(IMG_W), float(IMG_H)])}",
        "matrix = " + _format_toml_value(
            [[float(p.f), 0.0, float(p.cx)], [0.0, float(p.f), float(p.cy)], [0.0, 0.0, 1.0]]),
        "distortions = " + _format_toml_value([float(p.k1), float(p.k2), 0.0, 0.0]),
        "rotation = " + _format_toml_value([float(v) for v in np.asarray(p.rvec).reshape(3)]),
        "translation = " + _format_toml_value([float(v) for v in np.asarray(p.tvec).reshape(3)]),
        "fisheye = false",
    ]
    return "\n".join(lines) + "\n"


def refine_accepted(report_text: str, camera: str) -> bool:
    prefix = f"  {camera}: 受理("
    return any(line.startswith(prefix) for line in report_text.splitlines())


def _format_iter_line(i: int, diag: dict, p: Params, delta: float) -> str:
    line = (f"i={i} | {diag['matcher']} マッチ{diag['matches']} N={diag['N']} inl={diag['inl']} "
            f"RMSE={diag['rmse']:.3f}px | f={p.f:.2f} cx={p.cx:.2f} cy={p.cy:.2f} "
            f"k1={p.k1:.4f} k2={p.k2:.4f} | Δ={delta:.3f}px | 被覆{diag['cover']}/16 | "
            f"lsq_status={diag['lsq_status']} | {diag['elapsed']:.1f}s")
    if diag["cover"] < COVER_WARN:
        line += " 警告: 被覆不足"
    return line


# ============================================================
# CLI
# ============================================================

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="レンダ＋マッチング反復による内部・外部パラメータの同時推定（feat-041）",
        allow_abbrev=False)
    p.add_argument("--init-toml", default=None, help="初期TOML（--config でも指定可）")
    p.add_argument("--camera", default=None, help="カメラ名（--config でも指定可）")
    p.add_argument("--ply", default=None, help="3DGS PLYファイル（--config でも指定可）")
    p.add_argument("--images-dir", default=None, help="前処理後画像ディレクトリ（<camera>.png）（--config でも指定可）")
    p.add_argument("--points-3d", default=None, help="3D基準点CSV（--config でも指定可）")
    p.add_argument("--points-2d", default=None, help="前処理後座標の2D点CSV（--config でも指定可）")
    p.add_argument("--out-dir", default=None, help="出力ディレクトリ（--config でも指定可）")
    p.add_argument("--tmp-dir", default=None, help="中間ファイル置き場（省略時は一時ディレクトリ）")
    p.add_argument("--config", default=None,
                   help="設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。"
                        "優先順位は CLI > 設定YAML > 既定値。"
                        "前処理（prepare_phone_photo.py）と共用の設定YAMLを渡してよい（前処理専用のキーは無視する）")
    return p


CONFIG_CONVERTERS: dict[str, Callable[[str], object]] = {
    "init_toml": _yaml_str, "camera": _yaml_str, "ply": _yaml_str, "images_dir": _yaml_str,
    "points_3d": _yaml_str, "points_2d": _yaml_str, "out_dir": _yaml_str, "tmp_dir": _yaml_str,
}
REQUIRED_OPTIONS = [("init_toml", "--init-toml"), ("camera", "--camera"), ("ply", "--ply"),
                    ("images_dir", "--images-dir"), ("points_3d", "--points-3d"),
                    ("points_2d", "--points-2d"), ("out_dir", "--out-dir")]
# 共用 YAML の相手（前処理 prepare_phone_photo.py）のキー（feat-043）
IGNORED_KEYS = frozenset({"image", "photo_info", "points_2d_original", "overwrite"})


def _out_paths(out_dir: Path, camera: str) -> dict:
    return {
        "loop_toml": out_dir / f"{camera}_loop.toml",
        "final_toml": out_dir / f"{camera}_final.toml",
        "loop_report": out_dir / f"{camera}_loop_report.txt",
        "refine_report": out_dir / f"{camera}_refine_report.txt",
        "eval_prefix": str(out_dir / f"{camera}_eval"),
    }


def _pose_summary(p: Params) -> str:
    return f"f={p.f:.2f} cx={p.cx:.2f} cy={p.cy:.2f} k1={p.k1:.4f} k2={p.k2:.4f}"


def _run(args, out: dict, tmp_dir: Path, rep: list[str]) -> int:
    camera = args.camera
    rep.append("feat-041: レンダ＋マッチング反復による内部・外部パラメータ同時推定")
    rep.append(f"実行日時: {datetime.datetime.now().isoformat()}")
    for k in ("init_toml", "camera", "ply", "images_dir", "points_3d", "points_2d", "out_dir"):
        rep.append(f"{k}: {getattr(args, k)}")
    rep.append("")

    try:
        cams = load_cameras_toml(args.init_toml)
        cam0 = select_camera(cams, camera)
    except (OSError, tomli.TOMLDecodeError, ValueError, KeyError) as e:
        raise InputError(f"入力TOMLまたはカメラを読み込めません: {e}") from e
    p3d = load_points_3d_csv(Path(args.points_3d))
    p2d = load_points_2d_csv(Path(args.points_2d))
    v = validate_camera(camera, cam0, args.images_dir, p3d, p2d)
    if not v["ok"]:
        raise InputError(f"入力検証に失敗しました: {v['reason']}")

    print("PLY読み込み中...")
    try:
        from render import load_ply  # 遅延import
        gaussians = load_ply(args.ply)
    except Exception as e:
        raise InputError(f"PLYを読み込めません: {args.ply}（{e}）") from e

    # FR-002
    res = pnp_manual(v["p3"], v["p2"], cam0["K"], cam0["D"])
    if res is None:
        raise InputError("手動点PnPに失敗しました")
    rvec0, tvec0, inl0 = res
    med0 = median_reproj_px(v["p3"], v["p2"], rvec0, tvec0, cam0["K"], cam0["D"])
    rep.append(f"手動点PnP: 手動点数={len(v['names'])} inlier={len(inl0)} "
               f"median_reproj_px={med0:.3f}")
    params = Params(f=float(cam0["K"][0, 0]), cx=float(cam0["K"][0, 2]),
                    cy=float(cam0["K"][1, 2]), k1=float(cam0["D"][0]), k2=float(cam0["D"][1]),
                    rvec=rvec0, tvec=tvec0)
    rep.append(f"初期パラメータ: {_pose_summary(params)}")
    rep.append("")

    converged = False
    try:
        for i in range(MAX_ITER):
            it = run_loop_iteration(gaussians, params, v["image_path"], i, tmp_dir)
            new = it["params"]
            delta = delta_intr(params, new)
            line = _format_iter_line(i, it["diag"], new, delta)
            print(line)
            rep.append(line)
            errs = check_ranges(new)
            if errs:
                rep.append(f"終了状態: 範囲外（{'; '.join(errs)}）")
                return 3
            params = new
            if is_converged(i, delta):
                converged = True
                break
    except LoopFailure as e:
        rep.append(f"終了状態: ループ失敗（{e}）")
        print(f"エラー: ループ失敗: {e}", file=sys.stderr)
        return 3
    if not converged:
        out["loop_toml"].write_text(format_loop_toml(camera, params), encoding="utf-8")
        rep.append("終了状態: 未収束（ループ結果 TOML は最後の反復の値）")
        print("エラー: 未収束（ループ結果 TOML は最後の反復の値で書き出した）", file=sys.stderr)
        return 3

    out["loop_toml"].write_text(format_loop_toml(camera, params), encoding="utf-8")
    e_und = refine_undistort_error_px(params.K(), params.D())
    rep.append(f"最終精緻化前の逆歪み誤差: {e_und:.5f}px（上限 {REFINE_UNDIST_MAX_PX}px）")
    if e_und > REFINE_UNDIST_MAX_PX:
        rep.append(f"終了状態: ループ失敗（最終精緻化の前提違反: 逆歪み誤差 {e_und}px）")
        print(f"エラー: 最終精緻化の前提違反: 逆歪み誤差 {e_und}px", file=sys.stderr)
        return 3

    # 最終精緻化
    print("最終精緻化（refine_extrinsics.py）実行中...")
    t0 = time.time()
    cmd = [sys.executable, str(ROOT / "phase4" / "refine_extrinsics.py"),
           "--toml", str(out["loop_toml"]), "--ply", args.ply,
           "--images-dir", args.images_dir, "--points-3d", args.points_3d,
           "--points-2d", args.points_2d, "--out-toml", str(out["final_toml"]),
           "--out-report", str(out["refine_report"]), "--cameras", camera,
           "--tmp-dir", str(tmp_dir / "refine"), "--overwrite"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    t_refine = time.time() - t0
    rep.append(f"refine_extrinsics.py 所要時間: {t_refine:.1f}s")
    if r.returncode != 0:
        rep.append(f"終了状態: refine_extrinsics.py が異常終了（終了コード {r.returncode}）")
        print(f"エラー: refine_extrinsics.py が異常終了: {r.stderr[-2000:]}", file=sys.stderr)
        return 1
    report_text = Path(out["refine_report"]).read_text(encoding="utf-8")
    if not refine_accepted(report_text, camera):
        rep.append("終了状態: 精緻化不受理")
        print("エラー: refine_extrinsics.py が不受理", file=sys.stderr)
        return 4

    # 評価
    print("重ね合わせ評価中...")
    t0 = time.time()
    final_cam = select_camera(load_cameras_toml(str(out["final_toml"])), camera)
    photo = cv2.imread(str(v["image_path"]), cv2.IMREAD_COLOR)
    st = evaluate_and_write(
        gaussians, final_cam, photo, out["eval_prefix"],
        f"toml={out['final_toml']} camera={camera} ply={args.ply} image={v['image_path']}")
    t_eval = time.time() - t0
    rep.append(f"評価 所要時間: {t_eval:.1f}s")
    rep.append(f"評価: n_valid={st['n_valid']} valid_ratio={st['valid_ratio']:.3f} "
               f"median={st['median']:.3f}px p95={st['p95']:.3f}px mean={st['mean']:.3f}px "
               f"max={st['max']:.3f}px 判定={st['judge']}")
    rep.append("終了状態: 成功")
    print(rep[-2])
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)
    camera = args.camera
    out_dir = Path(args.out_dir)
    out = _out_paths(out_dir, camera)

    if (camera in ("", ".", "..") or Path(camera).name != camera
            or "/" in camera or "\\" in camera):
        print(f"エラー: カメラ名にはパス区切りを含まない名前を指定してください: {camera}", file=sys.stderr)
        return 1
    # bug-006: 書き込み先のディレクトリは存在しないパスとして新しく作る（既存ディレクトリ内の
    # リンクや入力を、一時ファイル {camera}_final.toml.tmp や中間ファイルで上書きしないため）
    try:
        out_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"エラー: 出力ディレクトリが既に存在します（存在しない新しいパスを指定してください）: {out_dir}",
              file=sys.stderr)
        return 1
    except OSError as e:
        print(f"エラー: 出力ディレクトリを作成できません: {out_dir}（{e}）", file=sys.stderr)
        return 1
    tmp_ctx = None
    if args.tmp_dir:
        tmp_dir = Path(args.tmp_dir)
        try:
            tmp_dir.mkdir(parents=True, exist_ok=False)
        except OSError as e:
            try:
                out_dir.rmdir()   # 直前に作った出力ディレクトリを消す（空でなければ失敗する）
            except OSError:
                print(f"エラー: 作成したディレクトリが残っています（手で削除してください）: {out_dir}",
                      file=sys.stderr)
            if isinstance(e, FileExistsError):
                print(f"エラー: 中間ファイル置き場が既に存在します（存在しない新しいパスを指定してください）: {tmp_dir}",
                      file=sys.stderr)
            else:
                print(f"エラー: 中間ファイル置き場を作成できません: {tmp_dir}（{e}）", file=sys.stderr)
            return 1
    else:
        tmp_ctx = tempfile.TemporaryDirectory(prefix="feat041_")
        tmp_dir = Path(tmp_ctx.name)

    rep: list[str] = []
    try:
        try:
            code = _run(args, out, tmp_dir, rep)
        except (InputError, MatcherEnvError) as e:
            print(f"エラー: {e}", file=sys.stderr)
            rep.append(f"終了状態: エラー（{e}）")
            code = 1
        except LoopFailure as e:
            print(f"エラー: ループ失敗: {e}", file=sys.stderr)
            rep.append(f"終了状態: ループ失敗（{e}）")
            code = 3
        out["loop_report"].write_text("\n".join(rep) + "\n", encoding="utf-8")
    finally:
        if tmp_ctx is not None:
            tmp_ctx.cleanup()
    print(f"レポート: {out['loop_report']}")
    return code


if __name__ == "__main__":
    sys.exit(main())

"""feat-026: レンダリング＋自動マッチングによるカメラ外部パラメータ精緻化（メインCLI）

3DGSシーン（PLY）と実写画像から、手動プロット6点以上の2D-3D対応を入力に、
レンダ画像と実写画像の自動マッチング（LoFTR）・深度リフト・PnPの反復でカメラ外部
パラメータ（R, t）を精緻化するバッチツール。K既知モード（内部パラメータは入力TOML
の値を使用し推定しない）。

処理の流れ（1カメラあたり）:
  Stage 0: 入力検証（FR-001）
  Stage 1: 手動点PnP（入力全点、6点以上）で初期外部パラメータを求める（FR-002）
  Stage 2〜6: チェーンA/B/Cごとに反復リファイン（レンダ→マッチング→ゲート→
             深度リフト→PnP）をサンプリング型で実行する（FR-003, FR-004）
  合意判定・仲裁（FR-005）→ 前提条件チェック（FR-006）→ 結果出力（FR-007）
  複数カメラをTOML記載順に逐次処理する（FR-008）

実行方法（プロジェクトルートで）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/refine_extrinsics.py \\
        --toml <入力TOML> --ply <PLY> --images-dir <画像ディレクトリ> \\
        --points-3d <3D基準点CSV> --points-2d <2D点CSV> \\
        --out-toml <出力TOML> --out-report <レポートtxt> \\
        [--cameras cam1 cam2 ...] [--overwrite] [--seed-base 5000] [--tmp-dir <中間PNG/NPZ置き場>]

設計根拠: docs/issues/feat-026-render-match-extrinsic-refinement/design.md
本ファイルの定数はすべて design.md §3.1 の表の値をそのまま使用する（実装判断ゼロの原則）。
"""
import argparse
import csv
import datetime
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import tomli

from render_keypoints import (
    camera_to_viewmat,
    distortions_to_gsplat,
    load_cameras_toml,
    render_image,
    select_camera,
)

ROOT = Path(__file__).resolve().parents[1]
MATCHER_LAB_DIR = ROOT / "matcher_lab"


# ============================================================
# 例外クラス
# ============================================================

class InputError(Exception):
    """FR-001: 全体エラー（exit 1）として扱う入力不整合・ファイル欠落"""


class MatcherOOM(Exception):
    """FR-009/design §4.3: マッチャーCLIがCUDA OOM（exit 42）を返した（当該カメラ失敗）"""


class MatcherEnvError(RuntimeError):
    """FR-009/design §4.3: マッチャーCLIが想定外の非0終了コード、または座標変換の
    実装欠陥（round-trip/範囲内率違反）を検出した（環境異常。全体エラーとしてexit 1）"""


# ============================================================
# design.md §3.1 パイプライン定数（値の変更・追加は禁止）
# ============================================================

NEAR_PLANE = 0.5

SCHEDULE = {
    0: {"tau_px": 250.0, "ransac_px": 8.0},
    1: {"tau_px": 10.0, "ransac_px": 2.0},
}
GATE_ALPHA = 0.5
GATE_Z_RANGE = (0.5, 10.0)
GATE_VAR_WINDOW = 5
GATE_VAR_REL = 0.02
N_MIN = 30

LOFTR_CONF_TH = 0.2
ROUNDTRIP_MAX_PX = 0.5
IN_RANGE_REQ = 1.0

PNP_RANSAC_ITER = 1000
PNP_MANUAL_REPROJ_PX = 8.0

# チェーン定義: (名前, 位置摂動[m], 角度摂動[deg], 摂動seed)
CHAINS = [("A", None, None, None), ("B", 0.05, 2.0, 11), ("C", 0.05, 2.0, 12)]
N_S = 20
R_POS = 0.02
R_ANG = 0.30
F_C_MIN = 0.7
F2_BIMODAL = 0.2
F12_MIN = 0.9
TH_ARBITRATE_PX = 1.0

PRECOND_POS_M = 0.50
PRECOND_ANG_DEG = 10.0

WARN_RMSE_PX = 2.0
WARN_VOL_RATIO = 0.09
WARN_DEPTH_RANGE_M = 1.4
WARN_AREA_RATIO = 0.22

ORIG_W, ORIG_H = 1920, 1080

SEED_BASE_DEFAULT = 5000


def sched(k: int) -> dict:
    """反復番号kに対応するスケジュール条件を返す（k>=1はすべてk=1条件）"""
    return SCHEDULE[min(k, max(SCHEDULE.keys()))]


# ============================================================
# FR-001 入力検証（design §4.1）
# ============================================================

def load_points_3d_csv(path: Path) -> dict[str, np.ndarray]:
    """3D基準点CSV（kijunten_locations.csv形式）を読み込む。

    ObjectNameの重複・必須列の欠損・非有限値（NaN/inf）は全体エラー（InputError）とする。
    """
    required_cols = {"ObjectName", "X", "Y", "Z"}
    points: dict[str, np.ndarray] = {}
    dup_names: list[str] = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing_cols = required_cols - set(reader.fieldnames or [])
        if missing_cols:
            raise InputError(
                f"3D基準点CSVに必須列がありません: {sorted(missing_cols)}（{path}）")
        for row in reader:
            name = row["ObjectName"]
            try:
                x, y, z = float(row["X"]), float(row["Y"]), float(row["Z"])
            except (TypeError, ValueError):
                raise InputError(
                    f"3D基準点CSV: {name} の座標が数値として解釈できません（{path}）")
            if not all(np.isfinite([x, y, z])):
                raise InputError(
                    f"3D基準点CSV: {name} の座標に非有限値(NaN/inf)があります（{path}）")
            if name in points:
                dup_names.append(name)
            points[name] = np.array([x, y, z], dtype=np.float64)
    if dup_names:
        raise InputError(
            f"3D基準点CSV: ObjectNameが重複しています: {sorted(set(dup_names))}（{path}）")
    return points


def load_points_2d_csv(path: Path) -> dict[tuple[str, str], np.ndarray]:
    """2D点CSV（points_2d.csv形式）を読み込む。

    (camera_name, ObjectName)の重複・必須列の欠損・非有限値は全体エラー（InputError）とする。
    """
    required_cols = {"ObjectName", "camera_name", "X", "Y"}
    points: dict[tuple[str, str], np.ndarray] = {}
    dup_keys: list[tuple[str, str]] = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing_cols = required_cols - set(reader.fieldnames or [])
        if missing_cols:
            raise InputError(
                f"2D点CSVに必須列がありません: {sorted(missing_cols)}（{path}）")
        for row in reader:
            cam_name, name = row["camera_name"], row["ObjectName"]
            try:
                x, y = float(row["X"]), float(row["Y"])
            except (TypeError, ValueError):
                raise InputError(
                    f"2D点CSV: {cam_name}/{name} の座標が数値として解釈できません（{path}）")
            if not all(np.isfinite([x, y])):
                raise InputError(
                    f"2D点CSV: {cam_name}/{name} の座標に非有限値(NaN/inf)があります（{path}）")
            key = (cam_name, name)
            if key in points:
                dup_keys.append(key)
            points[key] = np.array([x, y], dtype=np.float64)
    if dup_keys:
        raise InputError(
            f"2D点CSV: (camera_name, ObjectName)が重複しています: "
            f"{sorted(set(dup_keys))}（{path}）")
    return points


def validate_camera(cam_name: str, cam_toml: dict, images_dir: str,
                    points_3d: dict, points_2d: dict) -> dict:
    """1カメラ分の入力検証（design §4.1）。

    実写画像の欠落・読込失敗は全体エラー（InputError）とする。画像サイズ不一致・
    点数不足（一意なObjectNameが6個未満）・座標範囲外は「カメラ単位の
    不整合」としてこの関数の戻り値（ok=False）で表現し、当該カメラのみ失敗扱いにする。
    一意なObjectNameが6個以上あれば、部分選択せず全点を使用する。
    """
    img_path = Path(images_dir) / f"{cam_name}.png"
    if not img_path.is_file():
        raise InputError(f"{cam_name}: 実写画像が見つかりません: {img_path}")
    img = cv2.imread(str(img_path))
    if img is None:
        raise InputError(f"{cam_name}: 実写画像を読み込めません: {img_path}")

    h, w = img.shape[:2]
    toml_w, toml_h = cam_toml["width"], cam_toml["height"]
    if (w, h) != (toml_w, toml_h):
        return {"ok": False,
                "reason": f"画像サイズがTOMLと不一致（実測{w}x{h}、TOML{toml_w}x{toml_h}）"}
    if (w, h) != (ORIG_W, ORIG_H):
        return {"ok": False,
                "reason": f"対応外の画像サイズです（実測{w}x{h}、対応は{ORIG_W}x{ORIG_H}のみ）"}

    names_2d = {n for (c, n) in points_2d.keys() if c == cam_name}
    names_3d = set(points_3d.keys())
    common = sorted(names_2d & names_3d)
    if len(common) < 6:
        return {"ok": False,
                "reason": f"一意なObjectNameが6個以上ありません（{len(common)}個: {common}）"}

    p3 = np.array([points_3d[n] for n in common], dtype=np.float64)
    p2 = np.array([points_2d[(cam_name, n)] for n in common], dtype=np.float64)
    out_of_range = [n for n, (x, y) in zip(common, p2)
                    if not (0 <= x <= toml_w - 1 and 0 <= y <= toml_h - 1)]
    if out_of_range:
        return {"ok": False, "reason": f"2D座標が画像範囲外です: {out_of_range}"}

    return {"ok": True, "reason": None, "p3": p3, "p2": p2, "names": common,
            "image_path": img_path}


# ============================================================
# FR-002 手動点PnP初期値（design §4.2）
# ============================================================

def pnp_manual(p3: np.ndarray, p2: np.ndarray, K: np.ndarray, D: np.ndarray):
    """手動点PnP（design §3.1「手動点PnP」行）。入力された全手動点（N≥6）で解く。
    戻り値: (rvec, tvec, inlier_idx) または None"""
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        p3, p2, K, D, iterationsCount=PNP_RANSAC_ITER,
        reprojectionError=PNP_MANUAL_REPROJ_PX, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None:
        return None
    inl = sorted(inl.flatten().tolist())
    if len(inl) < 4:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        p3[inl], p2[inl], K, D, rvec=rvec, tvec=tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return (np.asarray(rvec, dtype=np.float64).flatten(),
            np.asarray(tvec, dtype=np.float64).flatten(), inl)


# ============================================================
# design §4.3-1 方式A深度レンダ（擬似コードに固定）
# ============================================================

def render_depth_alpha_distorted(gaussians: dict, cam: dict, near_plane: float):
    """方式A: gsplat 1.5.3 の3DGUT経路で歪み付き期待深度・αマップを得る（2パスレンダ）。

    design §4.3-1 の擬似コードに固定する。`render_image(return_depth=True)` や
    `render_mode="RGB+ED"` は3DGUT経路が非対応のため使用しない（各ガウシアンの
    カメラ座標奥行きzを「色」として流し、αブレンド結果をαで正規化して期待深度を得る）。

    Returns:
        (depth_map, alpha_map): いずれも (H, W) float32。alpha≈0の画素の深度はNaN。
    """
    import torch
    from gsplat import rasterization

    device = gaussians["means"].device
    viewmat_np = camera_to_viewmat(cam)
    viewmat = torch.from_numpy(viewmat_np).to(device)
    K = torch.tensor(cam["K"], dtype=torch.float32, device=device)

    R = viewmat[:3, :3]
    t = viewmat[:3, 3]
    z_cam = (gaussians["means"] @ R.T + t)[:, 2]
    depth_colors = z_cam.unsqueeze(-1).expand(-1, 3).contiguous()

    radial, tangential = distortions_to_gsplat(cam["D"])
    with torch.no_grad():
        rendered, alphas, _meta = rasterization(
            means=gaussians["means"],
            quats=gaussians["quats"],
            scales=gaussians["scales"],
            opacities=gaussians["opacities"],
            colors=depth_colors,
            viewmats=viewmat.unsqueeze(0),
            Ks=K.unsqueeze(0),
            width=cam["width"],
            height=cam["height"],
            near_plane=near_plane,
            far_plane=1e10,
            camera_model="pinhole",
            with_ut=True,
            with_eval3d=True,
            packed=False,
            radial_coeffs=torch.tensor(np.asarray([radial]), dtype=torch.float32, device=device),
            tangential_coeffs=torch.tensor(np.asarray([tangential]), dtype=torch.float32, device=device),
            render_mode="RGB",
        )
    accum = rendered.squeeze(0)[..., 0]
    alpha = alphas.squeeze(0).squeeze(-1)
    depth = torch.where(alpha > 1e-6, accum / alpha.clamp(min=1e-6),
                        torch.full_like(accum, float("nan")))
    return depth.cpu().numpy().astype(np.float32), alpha.cpu().numpy().astype(np.float32)


# ============================================================
# design §4.3 反復リファインの補助関数（実験コードは流用せず書き直し）
# ============================================================

def sample_depth_bilinear(depth: np.ndarray, uv: np.ndarray, rel_spread_th: float = 0.05):
    """サブピクセル座標uv (M,2) の深度をbilinear補間で取得する。

    2x2近傍が画像外／NaNを含む／深度の広がり(max-min)が中心深度のrel_spread_thを
    超える（深度不連続=シルエット上）場合は無効（NaN）とする（design §3.1 深度参照）。

    Returns:
        (d, valid): d は (M,) float64（無効はNaN）、valid は (M,) bool
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


def depth_variance_map(depth: np.ndarray, valid_mask: np.ndarray, window: int) -> np.ndarray:
    """window×window局所深度分散（有効画素のみで計算。design §3.1 GATE_VAR_*）。

    有効画素が窓の半数未満の位置は inf（判定不能=ゲート不通過）。
    """
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


def perturb_pose(rvec: np.ndarray, tvec: np.ndarray, dpos_m: float, dang_deg: float,
                 rng: np.random.Generator):
    """チェーンB/Cの摂動生成（design §3.1「チェーン」行と同一の方式・大きさ）。

    位置: カメラ中心C=-R^T tを乱数方向にdpos_m移動。
    角度: カメラ座標系で乱数軸まわりにdang_deg回転（R'=R_δ @ R）。
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


def pose_diff(rvec_a, tvec_a, rvec_b, tvec_b) -> tuple[float, float]:
    """2ポーズの差: (カメラ中心距離[m], 回転角[deg])"""
    Ra, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64).reshape(3, 1))
    Rb, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64).reshape(3, 1))
    Ca = -Ra.T @ np.asarray(tvec_a, dtype=np.float64).reshape(3)
    Cb = -Rb.T @ np.asarray(tvec_b, dtype=np.float64).reshape(3)
    dpos = float(np.linalg.norm(Ca - Cb))
    cos_t = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    dang = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    return dpos, dang


def median_reproj_px(p3, p2, rvec, tvec, K, D) -> float:
    """入力された全手動点の再投影誤差の中央値（design §4.5 仲裁指標）"""
    proj, _ = cv2.projectPoints(p3, np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                                np.asarray(tvec, dtype=np.float64).reshape(3, 1), K, D)
    return float(np.median(np.linalg.norm(proj.reshape(-1, 2) - p2, axis=1)))


def gate_breakdown(u_q: np.ndarray, u_r: np.ndarray, depth: np.ndarray, alpha: np.ndarray,
                   var_map: np.ndarray, tau_d: float, tau_px: float) -> dict:
    """Stage 4ゲート（design §4.3-3）。AND条件は変位・深度有効性・α・z範囲・分散。

    除外内訳はdepth_ok→α→z→分散の順の逐次分類で記録する（design §4.3 accept基準6）。
    """
    ix = u_r[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = u_r[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    d_bilin, depth_ok = sample_depth_bilinear(depth, u_r)
    cond_disp = np.linalg.norm(u_q - u_r, axis=1) < tau_px
    cond_alpha = alpha[iy, ix] > GATE_ALPHA
    cond_z = (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1])
    cond_var = var_map[iy, ix] < tau_d ** 2
    keep = cond_disp & depth_ok & cond_alpha & cond_z & cond_var

    surv = cond_disp
    excl_depth_nan = surv & ~depth_ok
    surv = surv & depth_ok
    excl_alpha = surv & ~cond_alpha
    surv = surv & cond_alpha
    excl_z = surv & ~cond_z
    surv = surv & cond_z
    excl_var = surv & ~cond_var

    return {"n_disp": int(cond_disp.sum()),
            "excl_depth_nan": int(excl_depth_nan.sum()),
            "excl_alpha": int(excl_alpha.sum()),
            "excl_z": int(excl_z.sum()),
            "excl_var": int(excl_var.sum()),
            "N": int(keep.sum()), "keep": keep, "d_bilin": d_bilin}


# ============================================================
# FR-009 マッチャーCLI呼び出し（design §4.3-2, §6.3）
# ============================================================

def call_matcher(query_png: Path, render_png: Path, out_npz: Path):
    """matcher_lab環境のloftr_cli.pyをsubprocessで実行しNPZを読む。

    戻り値: (u_q, u_r, conf, meta)。exit 42はMatcherOOM、その他非0はMatcherEnvError。
    """
    cmd = ["uv", "run", "--project", str(MATCHER_LAB_DIR), "python",
           str(MATCHER_LAB_DIR / "loftr_cli.py"), str(query_png), str(render_png),
           "--out", str(out_npz)]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 42:
        raise MatcherOOM()
    if res.returncode != 0:
        raise MatcherEnvError(f"loftr_cli.py が異常終了しました（環境異常）: {res.stderr[-2000:]}")
    d = np.load(out_npz, allow_pickle=False)
    meta = json.loads(str(d["meta"]))
    return (d["u_q"].astype(np.float64), d["u_r"].astype(np.float64),
            d["conf"].astype(np.float64), meta)


def check_matcher_meta(meta: dict) -> None:
    """マッチ座標の逆写像検証（design §4.3 accept基準3）。違反は座標変換の実装欠陥として即中断"""
    if meta.get("in_range_rate") != IN_RANGE_REQ:
        raise MatcherEnvError(
            f"マッチ座標の原寸範囲内率 {meta.get('in_range_rate')} が {IN_RANGE_REQ} 未満です"
            "（座標変換の実装欠陥。中断）")
    if meta.get("roundtrip_max_px", 0.0) > ROUNDTRIP_MAX_PX:
        raise MatcherEnvError(
            f"逆写像round-trip誤差 {meta.get('roundtrip_max_px')}px が {ROUNDTRIP_MAX_PX}px "
            "を超えています（座標変換の実装欠陥。中断）")


# ============================================================
# design §4.3 反復リファイン1回分（FR-003）
# ============================================================

def run_iteration(gaussians, cam: dict, query_png: Path, rvec_cur, tvec_cur, k: int,
                  tmp_dir: Path, tag: str):
    """反復1回分（Stage 2〜6。design §4.3 の順序に固定）。

    戻り値: (成否, (rvec, tvec), diag dict)
    """
    p = sched(k)
    cam_cur = dict(cam)
    cam_cur["rvec"] = rvec_cur
    cam_cur["tvec"] = tvec_cur
    K, D = cam["K"], cam["D"]

    # Stage 2: レンダ（方式A。near_plane=0.5、distort=True）
    bgr = render_image(gaussians, cam_cur, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam_cur, NEAR_PLANE)

    # Stage 3: マッチング（matcher_lab環境のCLIをsubprocess呼び出し）
    render_png = tmp_dir / f"{tag}_render.png"
    cv2.imwrite(str(render_png), bgr)
    out_npz = tmp_dir / f"{tag}_matches.npz"
    u_q, u_r, conf, meta = call_matcher(query_png, render_png, out_npz)
    check_matcher_meta(meta)
    n_raw = len(u_q)
    sel = conf >= LOFTR_CONF_TH
    u_q, u_r = u_q[sel], u_r[sel]
    diag = {"k": k, "tau_px": p["tau_px"], "ransac_px": p["ransac_px"],
            "n_raw": n_raw, "matches": len(u_q),
            "alpha_cov": float((alpha > GATE_ALPHA).mean())}
    if len(u_q) == 0:
        diag["fail"] = "マッチ0件"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 4: ゲート
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                   & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, p["tau_px"])
    keep, d_bilin, N = bd["keep"], bd["d_bilin"], bd["N"]
    diag["N"] = N
    diag.update({key: bd[key] for key in
                ("n_disp", "excl_depth_nan", "excl_alpha", "excl_z", "excl_var")})
    if N < N_MIN:
        diag["fail"] = f"ゲート生存 N={N} < {N_MIN}"
        return False, (rvec_cur, tvec_cur), diag

    # Stage 5: 深度リフト（3D化）
    pts = u_r[keep].reshape(-1, 1, 2).astype(np.float64)
    u_pin = cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)
    rays = (np.linalg.inv(K) @ np.hstack([u_pin, np.ones((len(u_pin), 1))]).T).T
    X_cam = rays * d_bilin[keep].reshape(-1, 1)
    R_cur, _ = cv2.Rodrigues(np.asarray(rvec_cur).reshape(3, 1))
    X_w = (X_cam - np.asarray(tvec_cur).reshape(1, 3)) @ R_cur

    # Stage 6: PnP-RANSAC + 精密化
    u_q_keep = u_q[keep]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        X_w, u_q_keep, K, D, iterationsCount=PNP_RANSAC_ITER,
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


# ============================================================
# FR-004 サンプリング型リファイン（design §4.4）
# ============================================================

def run_chain(gaussians, cam: dict, query_png: Path, rvec0, tvec0, seed_series: int,
             tmp_dir: Path, tag_prefix: str) -> dict:
    """1チェーン: k=0で反復1回 + k=1条件でN_S=20回サンプリング（design §4.4）。

    seed規則: cv2.setRNGSeed(seed_series + j)（j=0がk=0の反復、j=1..N_Sがk=1の反復）。
    """
    rvec_cur = np.asarray(rvec0, dtype=np.float64).flatten()
    tvec_cur = np.asarray(tvec0, dtype=np.float64).flatten()
    diags = []
    cv2.setRNGSeed(seed_series)
    ok, (rvec_cur, tvec_cur), diag = run_iteration(
        gaussians, cam, query_png, rvec_cur, tvec_cur, 0, tmp_dir, f"{tag_prefix}_j000")
    diags.append(diag)
    if not ok:
        return {"accepted": False, "fail": f"k=0: {diag.get('fail')}", "diags": diags}
    samples = []
    for j in range(1, N_S + 1):
        cv2.setRNGSeed(seed_series + j)
        ok, (rvec_new, tvec_new), diag = run_iteration(
            gaussians, cam, query_png, rvec_cur, tvec_cur, 1, tmp_dir,
            f"{tag_prefix}_j{j:03d}")
        diags.append(diag)
        if not ok:
            return {"accepted": False, "fail": f"j={j}: {diag.get('fail')}", "diags": diags}
        rvec_cur, tvec_cur = rvec_new, tvec_new
        samples.append((rvec_cur.copy(), tvec_cur.copy()))
    return {"accepted": True, "samples": samples, "diags": diags}


# ============================================================
# FR-005 合意判定と受理（design §4.5）
# ============================================================

def _rot_of(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return R


def _center_of(rvec, tvec):
    R = _rot_of(rvec)
    return -R.T @ np.asarray(tvec, dtype=np.float64).reshape(3)


def cluster_analysis(samples: list):
    """pooledサンプルのクラスタ分析（design §4.5。アルゴリズムを固定）。

    samples: [(rvec, tvec), ...]
    戻り値: (第1クラスタのindex配列, f_c, f2, (rvec_final, tvec_final), rms_pos, rms_ang)
    """
    n = len(samples)
    Cs = np.array([_center_of(r, t) for r, t in samples])
    Rs = [_rot_of(r) for r, t in samples]
    dpos = np.linalg.norm(Cs[:, None, :] - Cs[None, :, :], axis=2)
    dang = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            cos_t = (np.trace(Rs[i].T @ Rs[j]) - 1.0) / 2.0
            dang[i, j] = dang[j, i] = np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0)))
    within = (dpos <= R_POS) & (dang <= R_ANG)
    counts = within.sum(axis=1)
    anchor = int(np.argmax(counts))
    cluster = np.where(within[anchor])[0]
    f_c = len(cluster) / n

    rest = np.array([i for i in range(n) if i not in set(cluster.tolist())])
    if len(rest) > 0:
        counts2 = within[np.ix_(rest, rest)].sum(axis=1)
        f2 = counts2.max() / n
    else:
        f2 = 0.0

    C_mean = Cs[cluster].mean(axis=0)
    R_anchor = Rs[anchor]
    ws = []
    for i in cluster:
        w, _ = cv2.Rodrigues(R_anchor.T @ Rs[i])
        ws.append(w.flatten())
    w_mean = np.mean(ws, axis=0)
    dR, _ = cv2.Rodrigues(w_mean.reshape(3, 1))
    R_final = R_anchor @ dR
    rvec_final, _ = cv2.Rodrigues(R_final)
    tvec_final = -R_final @ C_mean

    rms_pos = float(np.sqrt(np.mean(np.linalg.norm(Cs[cluster] - C_mean, axis=1) ** 2)))
    angs = []
    for i in cluster:
        cos_t = (np.trace(R_final.T @ Rs[i]) - 1.0) / 2.0
        angs.append(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    rms_ang = float(np.sqrt(np.mean(np.array(angs) ** 2)))
    return cluster, f_c, f2, (rvec_final.flatten(), tvec_final.flatten()), rms_pos, rms_ang


def decide_acceptance(chain_results: list, p3, p2, K, D) -> dict:
    """pooled合意判定+仲裁（design §4.5。受理条件は§3.1「受理」行のとおり）。

    戻り値: {"est_ok", "accepted", "mode", ["rvec","tvec","f_c","f2","rms_pos","rms_ang"]}
    """
    est_ok = all(r["accepted"] for r in chain_results)
    if not est_ok:
        return {"est_ok": False, "accepted": False, "mode": "不受理(成立性不成立)"}

    all_samples = []
    for r in chain_results:
        all_samples.extend(r["samples"])
    cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(all_samples)
    n_pool = len(all_samples)

    accepted = False
    mode = ""
    rvec_sol, tvec_sol = rvec_f, tvec_f
    if f_c >= F_C_MIN and f2 < F2_BIMODAL:
        accepted = True
        mode = f"単峰高合意(f_c={f_c:.3f}≥{F_C_MIN}, 第2={f2:.3f}<{F2_BIMODAL})"
    elif f2 >= F2_BIMODAL:
        rest = [all_samples[i] for i in range(n_pool) if i not in set(cluster.tolist())]
        c2, _, _, (rvec_g, tvec_g), _, _ = cluster_analysis(rest)
        f12 = f_c + len(c2) / n_pool
        med_f = median_reproj_px(p3, p2, rvec_f, tvec_f, K, D)
        med_g = median_reproj_px(p3, p2, rvec_g, tvec_g, K, D)
        diff = abs(med_f - med_g)
        if f12 >= F12_MIN and diff >= TH_ARBITRATE_PX:
            accepted = True
            if med_g < med_f:
                rvec_sol, tvec_sol = rvec_g, tvec_g
                mode = f"仲裁成立の二峰(第2峰採用, 差={diff:.3f}px)"
            else:
                mode = f"仲裁成立の二峰(第1峰採用, 差={diff:.3f}px)"
        elif f12 < F12_MIN:
            mode = f"不受理(二峰だが合計占有率{f12:.3f}<{F12_MIN})"
        else:
            mode = f"不受理(仲裁不成立: 差{diff:.3f}px<{TH_ARBITRATE_PX})"
    else:
        mode = f"不受理(f_c={f_c:.3f}<{F_C_MIN}の非二峰)"

    return {"est_ok": True, "accepted": accepted, "mode": mode,
            "rvec": rvec_sol, "tvec": tvec_sol,
            "f_c": f_c, "f2": f2, "rms_pos": rms_pos, "rms_ang": rms_ang}


# ============================================================
# FR-010 実行ログ用フォーマッタ
# ============================================================

def warn_list(diag: dict) -> list[str]:
    """診断値の暫定警告閾値の点灯一覧（design §3.1「警告閾値」行。合否には使わない）"""
    warns = []
    if diag.get("rmse") is not None and diag["rmse"] > WARN_RMSE_PX:
        warns.append(f"RMSE>{WARN_RMSE_PX}px")
    if diag.get("vol_ratio") is not None and diag["vol_ratio"] < WARN_VOL_RATIO:
        warns.append(f"vol_ratio<{WARN_VOL_RATIO}")
    if diag.get("depth_range") is not None and diag["depth_range"] < WARN_DEPTH_RANGE_M:
        warns.append(f"depth_range<{WARN_DEPTH_RANGE_M}m")
    if diag.get("area_ratio") is not None and diag["area_ratio"] < WARN_AREA_RATIO:
        warns.append(f"area_ratio<{WARN_AREA_RATIO}")
    return warns


def format_iter_line(diag: dict) -> str:
    """design §7のログ行形式: k=<k>: OK|失敗(...) | マッチ<生> 選別後<n> N=<N> inl=<inl> RMSE=<x>px | 警告: <一覧>"""
    status = "OK" if "fail" not in diag else f"失敗({diag['fail']})"
    n_raw = diag.get("n_raw", 0)
    matches = diag.get("matches", 0)
    N = diag.get("N", 0)
    inl = diag.get("inliers", 0)
    line = f"k={diag.get('k', '?')}: {status} | マッチ{n_raw} 選別後{matches} N={N} inl={inl}"
    if "rmse" in diag:
        line += f" RMSE={diag['rmse']:.3f}px"
    warns = warn_list(diag)
    if warns:
        line += f" | 警告: {','.join(warns)}"
    return line


# ============================================================
# 1カメラ分の処理全体（FR-002〜FR-006）
# ============================================================

def process_camera(cam_name: str, cam_toml: dict, validated: dict, gaussians,
                   tmp_dir: Path, seed_base_cam: int, report_lines: list) -> dict:
    """1カメラの精緻化処理全体。戻り値: {"status": "accepted"|"rejected"|"failed", ...}"""
    p3, p2, names = validated["p3"], validated["p2"], validated["names"]
    query_png = validated["image_path"]
    K, D = cam_toml["K"], cam_toml["D"]

    res_manual = pnp_manual(p3, p2, K, D)
    if res_manual is None:
        report_lines.append("手動点PnP: 失敗（solvePnPRansac失敗またはinlier<4）")
        return {"status": "failed", "stage": "手動点PnP"}
    rvec0, tvec0, inl = res_manual
    outlier_names = [names[i] for i in range(len(names)) if i not in inl]
    report_lines.append(
        f"手動点PnP: 成功 点数={len(names)} inlier={len(inl)}/{len(names)}"
        + (f" outlier={outlier_names}" if outlier_names else ""))

    chain_results = []
    try:
        for ci, (cname, dpos, dang, seed) in enumerate(CHAINS):
            if dpos is None:
                rv, tv = rvec0.copy(), tvec0.copy()
                report_lines.append(f"チェーン{cname}: 初期値=手動点PnP解(無摂動)")
            else:
                rng = np.random.default_rng(seed)
                rv, tv = perturb_pose(rvec0, tvec0, dpos, dang, rng)
                report_lines.append(
                    f"チェーン{cname}: 初期値=手動点PnP解+摂動{dpos*100:.0f}cm/{dang}°(seed={seed})")
            seed_series = seed_base_cam + ci * 25
            print(f"  チェーン{cname} 実行中(seed_series={seed_series})...")
            r = run_chain(gaussians, cam_toml, query_png, rv, tv, seed_series, tmp_dir,
                         f"{cam_name}_{cname}")
            r["name"] = cname
            chain_results.append(r)
            for d in r["diags"]:
                line = "  " + format_iter_line(d)
                print(line)
                report_lines.append(line)
                if d.get("k") == 0 and "n_disp" in d:
                    report_lines.append(
                        f"    除外内訳(k=0): depth_nan={d.get('excl_depth_nan')} "
                        f"alpha={d.get('excl_alpha')} z={d.get('excl_z')} var={d.get('excl_var')}")
            if not r["accepted"]:
                report_lines.append(f"  チェーン{cname}: 不成立（{r['fail']}）")
    except MatcherOOM:
        report_lines.append("マッチャーOOM: 当該カメラを失敗として記録")
        return {"status": "failed", "stage": "マッチャーOOM"}

    decision = decide_acceptance(chain_results, p3, p2, K, D)
    if "f_c" in decision:
        report_lines.append(
            f"pooled合意: f_c={decision['f_c']:.3f} 第2クラスタ={decision['f2']:.3f} "
            f"RMS半径={decision['rms_pos']*1000:.2f}mm/{decision['rms_ang']:.4f}deg")
    if not decision["accepted"]:
        report_lines.append(f"受理判定: 不受理 — {decision.get('mode', '')}")
        return {"status": "rejected", "mode": decision.get("mode", "")}
    report_lines.append(f"受理判定: 受理 — {decision['mode']}")

    d_init, a_init = pose_diff(decision["rvec"], decision["tvec"], rvec0, tvec0)
    precond_ok = (d_init <= PRECOND_POS_M) and (a_init <= PRECOND_ANG_DEG)
    report_lines.append(
        f"前提条件: 初期解→最終解の差={d_init*100:.2f}cm/{a_init:.3f}° "
        f"(上限{PRECOND_POS_M*100:.0f}cm/{PRECOND_ANG_DEG}°) → {'満足' if precond_ok else '違反'}")
    if not precond_ok:
        return {"status": "failed", "stage": "前提条件違反"}

    med_init = median_reproj_px(p3, p2, rvec0, tvec0, K, D)
    med_final = median_reproj_px(p3, p2, decision["rvec"], decision["tvec"], K, D)
    report_lines.append(f"手動点再投影中央値: 初期解={med_init:.3f}px 最終解={med_final:.3f}px")

    return {"status": "accepted", "rvec_final": decision["rvec"],
            "tvec_final": decision["tvec"], "mode": decision["mode"]}


# ============================================================
# FR-007 結果出力（design §4.7）
# ============================================================

def _format_toml_value(v) -> str:
    """TOML値をPythonオブジェクトから文字列化する（安全側: 未対応型はValueError）"""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return str(v)
    if isinstance(v, int):
        return str(v)
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, list):
        return "[" + ", ".join(_format_toml_value(e) for e in v) + "]"
    raise ValueError(f"転記方法が定義できない型です: {type(v).__name__}")


def _format_camera_section(camera_name: str, cam_data: dict, rvec=None, tvec=None) -> str:
    """1カメラ分のTOMLセクション文字列を生成する（phase0 _format_toml_section互換の書式）。

    rvec/tvecが与えられた場合のみrotation/translationをその値に置換し、他のキーは
    入力TOMLの値をそのまま転記する。
    """
    lines = [f"[{camera_name}]"]
    for key, value in cam_data.items():
        if key == "rotation" and rvec is not None:
            value = [float(rvec[0]), float(rvec[1]), float(rvec[2])]
        elif key == "translation" and tvec is not None:
            value = [float(tvec[0]), float(tvec[1]), float(tvec[2])]
        try:
            v_str = _format_toml_value(value)
        except ValueError as e:
            raise ValueError(f"[{camera_name}].{key}: {e}") from e
        lines.append(f"{key} = {v_str}")
    return "\n".join(lines) + "\n"


def write_output_toml(input_toml_path: str, out_path: str, results: dict) -> None:
    """入力TOMLの全カメラを元順で含む出力TOMLを書く（design §4.7 / FR-007）。

    受理カメラのみrotation/translationを最終解に置換し、それ以外（不受理・失敗・
    --cameras対象外）は全キーを入力値のまま転記する。metadataセクションがあれば
    転記した上でrefined_by/refined_dateを追記する。原子的書き込み（.tmp経由）。
    """
    with open(input_toml_path, "rb") as f:
        data = tomli.load(f)

    today = datetime.date.today().isoformat()
    sections = []
    for key, value in data.items():
        if not isinstance(value, dict):
            raise ValueError(f"未対応のTOML構造です: [{key}]")
        if key == "metadata":
            lines = ["[metadata]"]
            for mk, mv in value.items():
                lines.append(f"{mk} = {_format_toml_value(mv)}")
            lines.append('refined_by = "feat-026"')
            lines.append(f'refined_date = "{today}"')
            sections.append("\n".join(lines) + "\n")
        elif "matrix" in value:
            r = results.get(key)
            rvec = r["rvec_final"] if r and r.get("accepted") else None
            tvec = r["tvec_final"] if r and r.get("accepted") else None
            sections.append(_format_camera_section(key, value, rvec, tvec))
        else:
            raise ValueError(f"未対応のTOMLセクション形式です: [{key}]")

    tmp_path = Path(out_path).with_suffix(Path(out_path).suffix + ".tmp")
    with open(tmp_path, "w", encoding="utf-8") as f:
        f.write("\n".join(sections))
    tmp_path.replace(out_path)


def _write_report(path: str, lines: list[str]) -> None:
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def _check_can_write(path: str, overwrite: bool, label: str) -> None:
    if Path(path).exists() and not overwrite:
        raise InputError(f"{label}が既に存在します（--overwrite で上書き可）: {path}")


# ============================================================
# CLI（design §6.2）
# ============================================================

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="レンダリング＋自動マッチングによるカメラ外部パラメータ精緻化（feat-026）")
    p.add_argument("--toml", required=True, help="入力TOML（Calib_scene.toml型。K既知）")
    p.add_argument("--ply", required=True, help="3DGS PLYファイル")
    p.add_argument("--images-dir", required=True, help="実写画像ディレクトリ（<camera_name>.png）")
    p.add_argument("--points-3d", required=True, help="3D基準点CSV（kijunten_locations.csv形式）")
    p.add_argument("--points-2d", required=True, help="2D点CSV（points_2d.csv形式）")
    p.add_argument("--out-toml", required=True, help="出力TOMLパス")
    p.add_argument("--out-report", required=True, help="診断レポートtxtパス")
    p.add_argument("--cameras", nargs="+", default=None, help="対象カメラ名（省略時は自動選択）")
    p.add_argument("--overwrite", action="store_true", help="出力ファイルの上書きを許可する")
    p.add_argument("--seed-base", type=int, default=SEED_BASE_DEFAULT,
                   help=f"サンプリング乱数のseed基点（既定{SEED_BASE_DEFAULT}）")
    p.add_argument("--tmp-dir", default=None,
                   help="中間PNG/NPZ置き場（省略時はシステム一時ディレクトリを使い、正常終了時に削除）")
    return p


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    report_lines: list[str] = []
    summary_counts = {"受理(単峰)": 0, "受理(仲裁)": 0, "不受理": 0, "失敗": 0}
    per_camera_class: dict[str, str] = {}
    results: dict[str, dict] = {}
    target_names: list[str] = []

    tmp_dir_ctx = None
    if args.tmp_dir:
        tmp_dir = Path(args.tmp_dir)
        tmp_dir.mkdir(parents=True, exist_ok=True)
    else:
        tmp_dir_ctx = tempfile.TemporaryDirectory(prefix="feat026_")
        tmp_dir = Path(tmp_dir_ctx.name)

    try:
        _check_can_write(args.out_toml, args.overwrite, "出力TOML")
        _check_can_write(args.out_report, args.overwrite, "診断レポート")

        report_lines.append("feat-026: レンダリング＋自動マッチングによるカメラ外部パラメータ精緻化")
        report_lines.append(f"実行日時: {datetime.datetime.now().isoformat()}")
        report_lines.append(f"入力TOML: {args.toml}")
        report_lines.append(f"PLY: {args.ply}")
        report_lines.append(f"画像ディレクトリ: {args.images_dir}")
        report_lines.append(f"3D基準点CSV: {args.points_3d}")
        report_lines.append(f"2D点CSV: {args.points_2d}")
        report_lines.append(f"seed_base: {args.seed_base}")
        report_lines.append("")

        try:
            cams_toml = load_cameras_toml(args.toml)
        except (OSError, tomli.TOMLDecodeError) as e:
            raise InputError(f"入力TOMLを読み込めません: {args.toml}（{e}）") from e

        points_3d = load_points_3d_csv(Path(args.points_3d))
        points_2d = load_points_2d_csv(Path(args.points_2d))

        if args.cameras:
            target_names = list(dict.fromkeys(args.cameras))
            for name in target_names:
                if name not in cams_toml:
                    raise InputError(f"--cameras で指定されたカメラがTOMLにありません: {name}")
        else:
            cams_with_2d = {c for (c, _n) in points_2d.keys()}
            target_names = [name for name in cams_toml.keys() if name in cams_with_2d]

        print(f"対象カメラ: {target_names}")
        report_lines.append(f"対象カメラ: {target_names}")
        report_lines.append("")

        print("PLY読み込み中...")
        try:
            from render import load_ply  # 遅延import(root pytestではtorch非依存を保つ)
            gaussians = load_ply(args.ply)
        except Exception as e:
            raise InputError(f"PLYを読み込めません: {args.ply}（{e}）") from e

        for mi, cam_name in enumerate(target_names):
            print(f"=== {cam_name} ===")
            report_lines.append(f"=== {cam_name} ===")
            cam_toml = select_camera(cams_toml, cam_name)

            validated = validate_camera(cam_name, cam_toml, args.images_dir, points_3d, points_2d)
            if not validated["ok"]:
                report_lines.append(f"入力検証: 失敗（{validated['reason']}）")
                per_camera_class[cam_name] = "失敗(入力検証)"
                summary_counts["失敗"] += 1
                report_lines.append("")
                continue

            seed_base_cam = args.seed_base + mi * 100
            cam_result = process_camera(cam_name, cam_toml, validated, gaussians,
                                        tmp_dir, seed_base_cam, report_lines)
            status = cam_result["status"]
            if status == "accepted":
                results[cam_name] = {"accepted": True,
                                     "rvec_final": cam_result["rvec_final"],
                                     "tvec_final": cam_result["tvec_final"]}
                cls = "受理(仲裁)" if "仲裁" in cam_result.get("mode", "") else "受理(単峰)"
                per_camera_class[cam_name] = cls
                summary_counts[cls] += 1
            elif status == "rejected":
                per_camera_class[cam_name] = "不受理"
                summary_counts["不受理"] += 1
            else:
                stage = cam_result.get("stage", "?")
                per_camera_class[cam_name] = f"失敗({stage})"
                summary_counts["失敗"] += 1
            report_lines.append("")

    except InputError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1
    except MatcherEnvError as e:
        print(f"エラー: {e}", file=sys.stderr)
        report_lines.append(f"環境異常により中断: {e}")
        _write_report(args.out_report, report_lines)
        return 1

    try:
        write_output_toml(args.toml, args.out_toml, results)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    n_total = len(target_names)
    n_accepted = summary_counts["受理(単峰)"] + summary_counts["受理(仲裁)"]
    summary_line = (
        f"=== 全体サマリ === 受理(単峰)={summary_counts['受理(単峰)']} "
        f"受理(仲裁)={summary_counts['受理(仲裁)']} 不受理={summary_counts['不受理']} "
        f"失敗={summary_counts['失敗']}（全{n_total}カメラ中 受理{n_accepted}）")
    print(summary_line)
    report_lines.append(summary_line)
    for cam_name in target_names:
        report_lines.append(f"  {cam_name}: {per_camera_class.get(cam_name, '?')}")
    _write_report(args.out_report, report_lines)
    print(f"出力TOML: {args.out_toml}")
    print(f"診断レポート: {args.out_report}")

    if tmp_dir_ctx is not None:
        tmp_dir_ctx.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""feat-035: 8台多視点同時外部パラメータ調整（MASt3Rクロスカメラ + 3DGSアンカー）

3DGSモデル（PLY）へのアンカー対応点とカメラ間クロスマッチング対応点の再投影誤差を
同時最小化して、同時撮影した複数カメラの外部パラメータ（R, t）を一括調整するバッチ
ツール。内部パラメータ・歪み係数は入力TOMLの値に固定する（K既知）。

処理ステージ（design.md §1.2）:
    Stage P: ペアマッチング     … 全カメラペアに mast3r_cli.py を実行、採用ペア選定 (FR-004)
    Stage B: ブートストラップ   … 未較正カメラを貪欲に PnP 較正 (FR-002)
    Stage A: アンカー対応点     … 較正済み各カメラで レンダ→LoFTR→深度リフト (FR-003)
    Stage D: バンドル調整       … scipy least_squares による同時最適化 (FR-005)
    Stage E: 評価・出力         … ホールドアウト評価、レポート、TOML出力 (FR-006/007)

現段階（Step 3）では本体骨格、Stage P（ペアマッチング）、Stage B（ブートストラップ）
までを実装する。Stage A/D/E は未実装であり、main() は入力検証・カメラ辞書構築・
Stage P 実行・Stage B 実行・結果表示までで終了する。

実行方法（プロジェクトルートで）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/adjust_extrinsics_multiview.py \\
        --toml <intrinsics_all.toml> --ply <point_cloud.ply> --images-dir <dir> \\
        --init-toml <cage_refined.toml> --init-cameras cam05520125 cam05520126 \\
        --out-toml <out.toml> --out-report <report.txt> \\
        [--cameras <name>...] [--overwrite] [--seed 5000] [--tmp-dir <dir>] [--fresh-match]

設計根拠: docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import tomli

from refine_extrinsics import (  # noqa: F401 (Stage A/D/E で使用予定。design §2.0 参照)
    GATE_ALPHA,
    GATE_VAR_REL,
    GATE_VAR_WINDOW,
    GATE_Z_RANGE,
    NEAR_PLANE,
    depth_variance_map,
    render_depth_alpha_distorted,
    sample_depth_bilinear,
)
from render_keypoints import load_cameras_toml

ROOT = Path(__file__).resolve().parents[1]
MATCHER_LAB_DIR = ROOT / "matcher_lab"


# ============================================================
# 例外クラス
# ============================================================

class InputError(Exception):
    """全体エラー（exit 1）として扱う入力不整合・処理不能な状態"""


# ============================================================
# design.md §2.0 共通定数（FR-008 の実験で見直す前提の初期値）
# ============================================================

N_PAIR_MIN = 50        # 採用ペアの最小マッチ数（初期値。実験で決定）
N_BOOT_MIN = 30         # ブートストラップPnPの最小 2D-3D 対応数（refine_extrinsics N_MIN と同値）
BOOT_RANSAC_PX = 8.0    # ブートストラップPnP RANSAC 閾値（refine_extrinsics sched(0) と同値）
BOOT_INLIER_MIN = 30    # ブートストラップPnP の最小 inlier 数
HUBER_PX = 2.0          # Huber f_scale [px]（初期値。実験で決定）
W_ANCHOR = 1.0          # アンカー項重み（初期値。実験で決定）
W_CROSS = 1.0           # クロス項重み（初期値。実験で決定）
HOLDOUT_RATIO = 0.2     # ホールドアウト比率（初期値。実験で決定）
SEED_DEFAULT = 5000     # ホールドアウト分割の乱数シード（--seed で変更可）
ANCHOR_MAX_PTS = 3000   # カメラあたりアンカー対応点の上限（超過時は conf 降順で切詰め）
MAX_NFEV = 200          # least_squares の最大評価回数
TRI_MIN_ANGLE_DEG = 2.0     # 三角測量点の最小交会角[度]。未満の点は BA・評価から除外
Z_TRI_RANGE = (0.1, 10.0)   # 三角測量点の許容深度[m]（両カメラとも）。範囲外は除外
N_ANCHOR_MIN = 30       # BA参加に必要な最小アンカー対応点数（FR-005 のBA参加条件）
THETA_OPT_DEG = 30.0    # 交会角重みの最適角[度]（先行研究 VISAPP2026 の最適角30°）
THETA_SIGMA_DEG = 15.0  # 交会角重みのガウス幅[度]（初期値。実験で決定）


# ============================================================
# design §2.0 カメラ辞書マージ規則
# ============================================================

def build_camera_dict(toml_path: str, init_toml_path: str | None,
                      init_cameras: list[str] | None = None) -> dict[str, dict]:
    """`--toml`（intrinsics_all.toml）と `--init-toml` からカメラ辞書をマージする（design §2.0）。

    `--toml` からは K・D・画像サイズのみ採用し、同TOMLに記載されている rotation/
    translation は読み捨てる（別座標系の値である可能性があるため初期値に使わない）。
    `rvec`/`tvec` は「`--init-toml` に存在し、かつ `--init-cameras` に指定された」
    カメラについてのみ `--init-toml` の値をコピーし、それ以外のカメラは必ず None
    とする（None のカメラが Stage B の対象）。`init_cameras` が None（未指定）の場合は
    `--init-toml` の全カメラを指定したものとみなす。`init_cameras` に指定されたカメラが
    `--init-toml` に存在しない場合は InputError を送出する。

    feat-026 の出力TOML（`write_output_toml` 生成）は入力TOMLの全カメラのセクションを
    含み、精緻化されなかったカメラには無効なポーズが残るため、これを `--init-toml` に
    使う場合は `--init-cameras` の明示指定が必須（requirements FR-002、2026-08-17
    Step 3 実測で顕在化した事象への対策）。
    """
    cams_toml = load_cameras_toml(toml_path)
    init_cams = load_cameras_toml(init_toml_path) if init_toml_path else {}

    if init_cameras is not None:
        missing = [name for name in init_cameras if name not in init_cams]
        if missing:
            raise InputError(
                "--init-cameras で指定されたカメラが --init-toml に存在しません: "
                f"{missing}")
        trusted = set(init_cameras)
    else:
        trusted = set(init_cams.keys())

    merged: dict[str, dict] = {}
    for name, cam in cams_toml.items():
        if name in init_cams and name in trusted:
            rvec = init_cams[name]["rvec"].copy()
            tvec = init_cams[name]["tvec"].copy()
        else:
            rvec = None
            tvec = None
        merged[name] = {
            "name": name,
            "K": cam["K"].copy(),
            "D": cam["D"].copy(),
            "width": cam["width"],
            "height": cam["height"],
            "rvec": rvec,
            "tvec": tvec,
        }
    return merged


# ============================================================
# Stage P: ペアマッチングとペア選定（design §2.2, FR-004）
# ============================================================

def enumerate_pairs(camera_names: list[str]) -> list[tuple[str, str]]:
    """カメラ名の全ペアを辞書順（camA < camB）で列挙する（design §2.2）。

    n台のカメラから C(n,2) 件のペアを返す。
    """
    names = sorted(set(camera_names))
    return [(names[i], names[j]) for i in range(len(names)) for j in range(i + 1, len(names))]


def is_pair_adopted(n: int) -> bool:
    """マッチ数 n が採用ペア条件（N >= N_PAIR_MIN）を満たすか判定する（design §2.2 手順5）。"""
    return n >= N_PAIR_MIN


def _run_mast3r_cli(img_a: Path, img_b: Path, out_npz: Path,
                    resolution: str) -> subprocess.CompletedProcess:
    """matcher_lab環境の mast3r_cli.py をsubprocessで実行する（design §2.2 手順2）。"""
    cmd = ["uv", "run", "--project", str(MATCHER_LAB_DIR), "python",
           str(MATCHER_LAB_DIR / "mast3r_cli.py"), str(img_a), str(img_b),
           "--out", str(out_npz), "--resolution", resolution]
    return subprocess.run(cmd, capture_output=True, text=True)


def _load_pair_npz(out_npz: Path) -> dict:
    """mast3r_cli.py が出力したNPZを読み、採用/不採用のstatusを付けて返す。"""
    d = np.load(out_npz, allow_pickle=False)
    u_a = d["u_a"].astype(np.float64)
    u_b = d["u_b"].astype(np.float64)
    n = len(u_a)
    status = "adopted" if is_pair_adopted(n) else "rejected"
    return {"u_a": u_a, "u_b": u_b, "n": n, "status": status}


def run_pair_matching(camera_names: list[str], images_dir: Path, tmp_dir: Path,
                      fresh: bool) -> dict[tuple[str, str], dict]:
    """Stage P: 全カメラペアに mast3r_cli.py を実行し、採用ペアを選定する（design §2.2, FR-004）。

    処理ロジック（design §2.2）:
      1. 画像ファイルが存在するカメラのみを対象カメラとする（1台以下はエラー）
      2. 対象カメラの全ペア（辞書順）について mast3r_cli.py をsubprocess実行する。
         既存NPZがあれば再利用する（--fresh-match指定時は無視して再実行）
      3. 終了コード42（OOM）は --resolution 1280x720 で1回だけ再試行し、
         再度失敗したらマッチ0点扱い（status="OOM"）
      4. 終了コード1はマッチ0点扱い（status="ERROR"）で継続
      5. マッチ数 N >= N_PAIR_MIN のペアを採用ペア（status="adopted"）とする。
         採用ペアが0件ならエラー

    戻り値: {(camA, camB): {"u_a","u_b","n","status"}}。
        status は "adopted"（採用） / "rejected"（不採用） / "OOM" / "ERROR"。
    """
    images_dir = Path(images_dir)
    tmp_dir = Path(tmp_dir)
    target = [name for name in camera_names if (images_dir / f"{name}.png").is_file()]
    if len(target) <= 1:
        raise InputError(
            f"ペアマッチング対象の実写画像が1枚以下です（{len(target)}枚）。"
            "実写画像を2枚以上用意してください。")

    tmp_dir.mkdir(parents=True, exist_ok=True)
    pairs = enumerate_pairs(target)
    results: dict[tuple[str, str], dict] = {}
    empty = np.zeros((0, 2), dtype=np.float64)

    for cam_a, cam_b in pairs:
        img_a = images_dir / f"{cam_a}.png"
        img_b = images_dir / f"{cam_b}.png"
        out_npz = tmp_dir / f"pair_{cam_a}_{cam_b}.npz"

        if out_npz.exists() and not fresh:
            results[(cam_a, cam_b)] = _load_pair_npz(out_npz)
            continue

        res = _run_mast3r_cli(img_a, img_b, out_npz, "full")
        if res.returncode == 42:
            res2 = _run_mast3r_cli(img_a, img_b, out_npz, "1280x720")
            if res2.returncode != 0:
                results[(cam_a, cam_b)] = {"u_a": empty, "u_b": empty, "n": 0, "status": "OOM"}
            else:
                results[(cam_a, cam_b)] = _load_pair_npz(out_npz)
        elif res.returncode != 0:
            results[(cam_a, cam_b)] = {"u_a": empty, "u_b": empty, "n": 0, "status": "ERROR"}
        else:
            results[(cam_a, cam_b)] = _load_pair_npz(out_npz)

    n_adopted = sum(1 for r in results.values() if r["status"] == "adopted")
    if n_adopted == 0:
        raise InputError(
            "採用ペア（マッチ数 N >= N_PAIR_MIN）が0件です。"
            "N_PAIR_MIN を見直すかカメラ配置を確認してください。")
    return results


# ============================================================
# Stage B: ブートストラップ（design §2.3, FR-002）
# ============================================================

def gate_depth_quality(uv: np.ndarray, depth: np.ndarray,
                       alpha: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """ブートストラップ専用ゲート（design §2.3-3）。

    feat-026 の gate_breakdown は query↔render の画素変位条件（cond_disp）を含むが、
    視点の異なるカメラ間マッチには適用できないため流用しない。本関数は深度品質の
    4条件（depth_ok, alpha, z範囲, 局所分散）のみで判定する。

    判定式（design §2.3-3 に厳密準拠）:
        pixel_valid = isfinite(depth) & (alpha > GATE_ALPHA)
                      & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1])
        pixel_valid が0件の場合は全点 reject とする。
        d_bilin, depth_ok = sample_depth_bilinear(depth, uv)
        cond_alpha = alpha[iy, ix] > GATE_ALPHA
        cond_z = GATE_Z_RANGE[0] < d_bilin < GATE_Z_RANGE[1]
        var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
        tau_d = GATE_VAR_REL * median(depth[pixel_valid])
        cond_var = var_map[iy, ix] < tau_d ** 2
        keep = depth_ok & cond_alpha & cond_z & cond_var

    Returns:
        (keep_mask, depths): keep_mask は (N,) bool。depths は (N,) float64
        （uv位置のバイリニア深度 d_bilin。無効箇所はNaN）。
    """
    ix = uv[:, 0].round().astype(int).clip(0, depth.shape[1] - 1)
    iy = uv[:, 1].round().astype(int).clip(0, depth.shape[0] - 1)
    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                  & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    d_bilin, depth_ok = sample_depth_bilinear(depth, uv)
    if not pixel_valid.any():
        return np.zeros(len(uv), dtype=bool), d_bilin

    cond_alpha = alpha[iy, ix] > GATE_ALPHA
    cond_z = (d_bilin > GATE_Z_RANGE[0]) & (d_bilin < GATE_Z_RANGE[1])
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    cond_var = var_map[iy, ix] < tau_d ** 2
    keep = depth_ok & cond_alpha & cond_z & cond_var
    return keep, d_bilin


def _lift_points_to_world(uv: np.ndarray, depths: np.ndarray, K: np.ndarray,
                          D: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """歪み画素座標uvと深度depthsを3DGS座標系のワールド3D点に変換する（design §2.3-3）。

    cv2.undistortPoints で正規化座標を得て深度を乗じてカメラ座標を求め、
    カメラの外部パラメータ（world-to-camera規約）でワールド座標に変換する。
    """
    pts = uv.reshape(-1, 1, 2).astype(np.float64)
    norm = cv2.undistortPoints(pts, K, D).reshape(-1, 2)
    X_cam = np.hstack([norm, np.ones((len(norm), 1))]) * depths.reshape(-1, 1)
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return (X_cam - np.asarray(tvec, dtype=np.float64).reshape(1, 3)) @ R


def bootstrap_cameras(cams: dict[str, dict], pairs: dict, gaussians: dict,
                      images_dir: Path) -> tuple[dict[str, dict], list[dict]]:
    """Stage B: 未較正カメラを貪欲反復でPnP較正する（design §2.3, FR-002）。

    貪欲反復（design §2.3 手順1〜6）:
      1. S×U の採用ペアのうちマッチ数最大の未試行ペア (c∈S, u∈U) を選ぶ。
         存在しなければ終了
      2. c のポーズで render_depth_alpha_distorted(gaussians, cam_c, NEAR_PLANE) を
         実行し深度・αマップを得る（同一cは反復内でキャッシュし再レンダしない）
      3. c側マッチ座標を gate_depth_quality で選別し、3DGS座標系3D点に変換する
      4. 3D点とu側マッチ座標で cv2.solvePnPRansac を解く
      5. 対応数 < N_BOOT_MIN（対応点数不足）または inlier数 < BOOT_INLIER_MIN
         （PnP inlier不足）は試行失敗として (c, u) を試行済みに記録するのみとする。
         uの確定失敗は「S×{u}の採用ペアのうち未試行のものが存在しなくなった時点」で
         のみ判定し（毎周の選択時に判定）、それまでは試行失敗を繰り返し許す
      6. 成功したらuをSに移す

    境界条件: U が最初から空なら Stage B をスキップ（空の記録を返す）。
    S が1台のみで終わる場合のエラー終了コード1判定は main() 側で行う。

    Args:
        images_dir: design §1.7 のインターフェースに合わせて受け取るが、
            ペアマッチ座標は Stage P で既にNPZから読込済みのため本実装では
            直接使用しない（画像の再読込は不要）。

    Returns:
        (cams, records): cams は入力cams（成功したカメラのrvec/tvecを書き込み済み）。
        records は Stage B で処理した未較正カメラ全件の記録（"status" が
        "success"（"used_pair","n_gated","n_inliers","rmse_px" を含む）または
        "fail"（"attempts": 試行した(パートナー, 理由)のリスト）。
    """
    calibrated = {name for name, c in cams.items() if c["rvec"] is not None}
    uncalibrated = {name for name, c in cams.items() if c["rvec"] is None}
    if not uncalibrated:
        return cams, []

    adopted_by_cam: dict[str, list[tuple[str, int, tuple]]] = {}
    for key, r in pairs.items():
        if r.get("status") != "adopted":
            continue
        a, b = key
        adopted_by_cam.setdefault(a, []).append((b, r["n"], key))
        adopted_by_cam.setdefault(b, []).append((a, r["n"], key))

    tried: set[frozenset] = set()
    attempt_log: dict[str, list[dict]] = {u: [] for u in uncalibrated}
    records: list[dict] = []
    depth_cache: dict[str, tuple] = {}
    active_u = set(uncalibrated)

    while active_u:
        # 確定失敗判定（design §2.3 手順5。Sは反復中に増えるため毎周の選択時に行う）
        newly_failed = []
        for u in sorted(active_u):
            has_untried = any(
                (other in calibrated) and (frozenset((other, u)) not in tried)
                for other, _n, _key in adopted_by_cam.get(u, []))
            if not has_untried:
                newly_failed.append(u)
        for u in newly_failed:
            active_u.discard(u)
            records.append({"camera": u, "status": "fail", "attempts": list(attempt_log[u])})

        if not active_u:
            break

        # 手順1: S×Uの採用ペアのうちマッチ数最大の未試行ペアを選ぶ
        # （同数は決定性のためペア名の辞書順でタイブレークする）
        candidates = []
        for u in active_u:
            for other, n, key in adopted_by_cam.get(u, []):
                if other in calibrated and frozenset((other, u)) not in tried:
                    candidates.append((n, key, other, u))
        if not candidates:
            break  # 直前の確定失敗判定で全て除去済みのはずだが念のため

        candidates.sort(key=lambda x: (-x[0], x[1]))
        _n, key, c, u = candidates[0]
        tried.add(frozenset((c, u)))

        # 手順2: cのレンダ深度・αマップ（反復内キャッシュ）
        if c not in depth_cache:
            depth_cache[c] = render_depth_alpha_distorted(gaussians, cams[c], NEAR_PLANE)
        depth, alpha = depth_cache[c]

        a, b = key
        uv_c = pairs[key]["u_a"] if a == c else pairs[key]["u_b"]
        uv_u = pairs[key]["u_b"] if a == c else pairs[key]["u_a"]

        # 手順3: ゲート + 3D化
        keep, depths = gate_depth_quality(uv_c, depth, alpha)
        n_gated = int(keep.sum())
        if n_gated < N_BOOT_MIN:
            attempt_log[u].append({"partner": c, "reason": "対応点数不足", "n_gated": n_gated})
            continue

        X_w = _lift_points_to_world(uv_c[keep], depths[keep],
                                    cams[c]["K"], cams[c]["D"],
                                    cams[c]["rvec"], cams[c]["tvec"])
        uv_u_keep = uv_u[keep]

        # 手順4: PnP-RANSAC
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            X_w, uv_u_keep, cams[u]["K"], cams[u]["D"],
            reprojectionError=BOOT_RANSAC_PX, iterationsCount=2000,
            flags=cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess=False)
        n_inliers = 0 if (not ok or inliers is None) else len(inliers)

        # 手順5: 失敗判定
        if n_inliers < BOOT_INLIER_MIN:
            attempt_log[u].append({"partner": c, "reason": "PnP inlier不足",
                                   "n_gated": n_gated, "n_inliers": n_inliers})
            continue

        # 手順6: 成功
        rvec = np.asarray(rvec, dtype=np.float64).flatten()
        tvec = np.asarray(tvec, dtype=np.float64).flatten()
        inl = inliers.flatten()
        proj, _ = cv2.projectPoints(X_w[inl], rvec.reshape(3, 1), tvec.reshape(3, 1),
                                    cams[u]["K"], cams[u]["D"])
        resid = np.linalg.norm(proj.reshape(-1, 2) - uv_u_keep[inl], axis=1)
        rmse = float(np.sqrt(np.mean(resid ** 2)))

        cams[u]["rvec"] = rvec
        cams[u]["tvec"] = tvec
        calibrated.add(u)
        active_u.discard(u)
        records.append({"camera": u, "status": "success", "used_pair": c,
                        "n_gated": n_gated, "n_inliers": n_inliers, "rmse_px": rmse})

    return cams, records


# ============================================================
# 出力ファイルの上書きチェック（refine_extrinsics.py と同一方針）
# ============================================================

def _check_can_write(path: str, overwrite: bool, label: str) -> None:
    if Path(path).exists() and not overwrite:
        raise InputError(f"{label}が既に存在します（--overwrite で上書き可）: {path}")


# ============================================================
# CLI（design §2.7）
# ============================================================

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="8台多視点同時外部パラメータ調整"
                    "（MASt3Rクロスカメラ + 3DGSアンカー、feat-035）")
    p.add_argument("--toml", required=True, help="内部パラメータTOML（intrinsics_all.toml型。K既知）")
    p.add_argument("--ply", required=True, help="3DGS PLYファイル")
    p.add_argument("--images-dir", required=True, help="実写画像ディレクトリ（<camera_name>.png）")
    p.add_argument("--init-toml", default=None,
                  help="初期ポーズTOML（1台分以上。未指定はエラー）")
    p.add_argument("--init-cameras", nargs="+", default=None,
                  help="初期ポーズを信頼するカメラ名（省略時は --init-toml の全カメラ）")
    p.add_argument("--out-toml", required=True, help="出力TOMLパス")
    p.add_argument("--out-report", required=True, help="診断レポートtxtパス")
    p.add_argument("--cameras", nargs="+", default=None,
                  help="対象カメラ名（省略時はTOML×画像の積集合全て）")
    p.add_argument("--overwrite", action="store_true", help="出力ファイルの上書きを許可する")
    p.add_argument("--seed", type=int, default=SEED_DEFAULT,
                  help=f"ホールドアウト分割の乱数シード（既定{SEED_DEFAULT}）")
    p.add_argument("--tmp-dir", default=None,
                  help="中間PNG/NPZ置き場（省略時は<out-tomlの親>/tmp_feat035/）")
    p.add_argument("--fresh-match", action="store_true",
                  help="既存のペアマッチングNPZを無視して再実行する")
    return p


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    try:
        _check_can_write(args.out_toml, args.overwrite, "出力TOML")
        _check_can_write(args.out_report, args.overwrite, "診断レポート")

        if not args.init_toml:
            raise InputError(
                "--init-toml が指定されていません（1台以上の初期ポーズが必要です）")

        if not Path(args.ply).is_file():
            raise InputError(f"PLYファイルが見つかりません: {args.ply}")

        images_dir = Path(args.images_dir)
        if not images_dir.is_dir():
            raise InputError(f"画像ディレクトリが見つかりません: {images_dir}")

        try:
            cams = build_camera_dict(args.toml, args.init_toml, args.init_cameras)
        except (OSError, tomli.TOMLDecodeError) as e:
            raise InputError(f"TOMLを読み込めません（{e}）") from e

        if args.cameras:
            target_names = list(dict.fromkeys(args.cameras))
            for name in target_names:
                if name not in cams:
                    raise InputError(f"--cameras で指定されたカメラがTOMLにありません: {name}")
        else:
            target_names = [name for name in cams
                            if (images_dir / f"{name}.png").is_file()]

        print(f"対象カメラ: {target_names}")

        if args.tmp_dir:
            tmp_dir = Path(args.tmp_dir)
        else:
            tmp_dir = Path(args.out_toml).resolve().parent / "tmp_feat035"
        tmp_dir.mkdir(parents=True, exist_ok=True)

        print("=== Stage P: ペアマッチング ===")
        pair_results = run_pair_matching(target_names, images_dir, tmp_dir, args.fresh_match)

        print("--- ペアマッチ数一覧 ---")
        n_adopted = 0
        for (cam_a, cam_b), r in sorted(pair_results.items()):
            mark = "採用" if r["status"] == "adopted" else "不採用"
            if r["status"] == "adopted":
                n_adopted += 1
            print(f"{cam_a} - {cam_b}: N={r['n']} status={r['status']}（{mark}）")
        print(f"採用ペア数: {n_adopted}/{len(pair_results)}")

        print("=== Stage B: ブートストラップ ===")
        uncalibrated = [name for name in target_names if cams[name]["rvec"] is None]
        if not uncalibrated:
            print("全カメラが初期ポーズを持つため Stage B をスキップします。")
        else:
            import render  # 遅延import（torch非依存維持。design §1.2）
            gaussians = render.load_ply(args.ply)
            target_cams = {name: cams[name] for name in target_names}
            target_cams, boot_records = bootstrap_cameras(
                target_cams, pair_results, gaussians, images_dir)
            cams.update(target_cams)

            for rec in sorted(boot_records, key=lambda r: r["camera"]):
                if rec["status"] == "success":
                    print(f"{rec['camera']}: 成功（使用ペア={rec['used_pair']}, "
                         f"ゲート通過対応点数={rec['n_gated']}, inlier数={rec['n_inliers']}, "
                         f"inlier再投影RMSE={rec['rmse_px']:.3f}px）")
                else:
                    if rec["attempts"]:
                        reasons = "; ".join(
                            f"{a['partner']}:{a['reason']}" for a in rec["attempts"])
                    else:
                        reasons = "採用ペアなし"
                    print(f"{rec['camera']}: 失敗（試行={reasons}）")

        n_calibrated = sum(1 for name in target_names if cams[name]["rvec"] is not None)
        print(f"較正済みカメラ数: {n_calibrated}/{len(target_names)}")
        if n_calibrated <= 1:
            raise InputError(
                "ブートストラップ後の較正済みカメラが1台以下です。"
                "クロス項を構築できません。")

        # Stage A 以降は Step 4 以降で実装

    except InputError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

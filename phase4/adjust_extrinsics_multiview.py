"""feat-035: 8台多視点同時外部パラメータ調整（MASt3Rクロスカメラ + 3DGSアンカー）

3DGSモデル（PLY）へのアンカー対応点とカメラ間クロスマッチング対応点の再投影誤差を
同時最小化して、同時撮影した複数カメラの外部パラメータ（R, t）を一括調整するバッチ
ツール。内部パラメータ・歪み係数は入力TOMLの値に固定する（K既知）。

処理ステージ（design.md §1.2）:
    Stage P: ペアマッチング     … 全カメラペアに mast3r_cli.py を実行、採用ペア選定 (FR-004)
    Stage F: エピポーラフィルタ … 確定ポーズでクロス対応点の偽マッチを除去、採用ペア再判定 (FR-010)
    Stage A: アンカー対応点     … ポーズ確定カメラで レンダ→LoFTR→深度リフト (FR-003)
    Stage D: バンドル調整       … scipy least_squares による同時最適化 (FR-005)
    Stage E: 評価・出力         … ホールドアウト評価、レポート、TOML出力 (FR-006/007)
    （Stage B〔ブートストラップ〕は案Aにより主経路外。コードは温存するが main() から呼ばない。Stage R は廃止）

本体は Step 6（Stage E: ホールドアウト評価・レポート・TOML出力）実装によりMVPが完成
している。main() は入力検証・カメラ辞書構築（--init-cameras 全カメラ必須チェック含む）
→ Stage P → Stage F → Stage A → Stage D（BAの初期/最終コスト・非悪化フォールバック判定
の表示含む）→ Stage E（ホールドアウト評価、診断レポート (a)〜(i) の出力、出力TOML
書き出し）までを一括実行する。Stage B（bootstrap_cameras）・Stage R は主経路から
呼ばれない（非対称シーン向けの温存コード。design §2.3/2.35 参照）。

実行方法（プロジェクトルートで）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/adjust_extrinsics_multiview.py \\
        --toml <intrinsics_all.toml> --ply <point_cloud.ply> --images-dir <dir> \\
        --init-toml <cage_refined.toml> \\
        --init-cameras cam05520125 cam05520126 cam05520128 cam05520129 \\
                       cam41520554 cam41520556 cam41520557 cam41520558 \\
        --out-toml <out.toml> --out-report <report.txt> \\
        [--cameras <name>...] [--overwrite] [--seed 5000] [--tmp-dir <dir>] [--fresh-match]

設計根拠: docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md
"""
import argparse
import datetime
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import tomli
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from refine_extrinsics import (
    GATE_ALPHA,
    GATE_VAR_REL,
    GATE_VAR_WINDOW,
    GATE_Z_RANGE,
    LOFTR_CONF_TH,
    NEAR_PLANE,
    _format_camera_section,
    call_matcher,
    depth_variance_map,
    gate_breakdown,
    pose_diff,
    render_depth_alpha_distorted,
    sample_depth_bilinear,
    sched,
)
from render_keypoints import load_cameras_toml, render_image

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
HUBER_PX = 2.0          # Huber f_scale [px]（p1_param_sweep で決定。当初 1.0 採用も反映後再検証で
                        # W_CROSS=2.0 との相互作用による BA 停滞が発覚し、既定値 2.0 に改訂。
                        # 2026-08-25 ユーザー承認済み。criteria §4b・experiment_log 参照）
W_ANCHOR = 1.0          # アンカー項重み（p1_param_sweep で既定値維持を確認）
W_CROSS = 2.0           # クロス項重み（p1_param_sweep で決定。2026-08-25 採用・ユーザー承認済み）
HOLDOUT_RATIO = 0.2     # ホールドアウト比率（初期値。実験で決定）
SEED_DEFAULT = 5000     # ホールドアウト分割の乱数シード（--seed で変更可）
ANCHOR_MAX_PTS = 3000   # カメラあたりアンカー対応点の上限（超過時は conf 降順で切詰め）
MAX_NFEV = 1000         # least_squares の最大評価回数（改訂 2026-08-24: x_scale='jac' 前提でも
                        # inuyama 実測 nfev=270 で収束するため、収束余裕を持って 200→1000 に引き上げ）
TRI_MIN_ANGLE_DEG = 2.0     # 三角測量点の最小交会角[度]。未満の点は BA・評価から除外
Z_TRI_RANGE = (0.1, 10.0)   # 三角測量点の許容深度[m]（両カメラとも）。範囲外は除外
N_ANCHOR_MIN = 30       # BA参加に必要な最小アンカー対応点数（FR-005 のBA参加条件）
BA_DEGRADE_TOL_PX = 0.3  # 非悪化フォールバックの許容悪化幅[px]（FR-005）
THETA_OPT_DEG = 30.0    # 交会角重みの最適角[度]（VISAPP2026。交会角重み無効化採用により主経路では未使用。
                        # crossing_angle_weight 関数と単体テストは温存）
THETA_SIGMA_DEG = 15.0  # 交会角重みのガウス幅[度]（同上。主経路では未使用）
EPI_TOL_PX = 3.0        # エピポーラ整合フィルタの許容誤差[px]（初期値。実験で決定。FR-010）


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
# Stage F: エピポーラ整合フィルタ（design §2.45, FR-010）
# ============================================================

def _relative_pose(cam_a: dict, cam_b: dict) -> tuple[np.ndarray, np.ndarray]:
    """確定ポーズから相対ポーズ R_rel, t_rel を作る（design §2.45 手順2）。

    world-to-camera規約（x_cam = R @ x_w + t）を前提に、
    x_cam_b = R_rel @ x_cam_a + t_rel を満たす R_rel, t_rel を返す。
    """
    R_a, _ = cv2.Rodrigues(np.asarray(cam_a["rvec"], dtype=np.float64).reshape(3, 1))
    R_b, _ = cv2.Rodrigues(np.asarray(cam_b["rvec"], dtype=np.float64).reshape(3, 1))
    t_a = np.asarray(cam_a["tvec"], dtype=np.float64).reshape(3)
    t_b = np.asarray(cam_b["tvec"], dtype=np.float64).reshape(3)
    R_rel = R_b @ R_a.T
    t_rel = t_b - R_rel @ t_a
    return R_rel, t_rel


def _skew(v: np.ndarray) -> np.ndarray:
    """3次元ベクトルの歪対称行列 [v]_x を返す。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=np.float64)


def _sampson_epipolar_px(u_a: np.ndarray, u_b: np.ndarray, cam_a: dict,
                         cam_b: dict) -> np.ndarray:
    """クロス対応点のSampson距離を画素換算した値 d_px を返す（design §2.45 手順1〜3）。

    処理:
      1. 両側のマッチ座標を cv2.undistortPoints(uv, K, D, P=None) で正規化座標にする
      2. 確定ポーズから R_rel, t_rel を作り、E = [t_rel]_x @ R_rel を計算する
      3. sd = (x_b^T E x_a)^2 / ((E x_a)_1^2 + (E x_a)_2^2 + (E^T x_b)_1^2 + (E^T x_b)_2^2)
         を正規化座標系で計算し、d_px = sqrt(sd) * f_mean（f_mean=(f_a+f_b)/2、
         f は各カメラの(fx+fy)/2）で画素換算する（sdは二乗量なので平方根を先に取る）。

    Returns:
        d_px: (N,) float64
    """
    pts_a = u_a.reshape(-1, 1, 2).astype(np.float64)
    pts_b = u_b.reshape(-1, 1, 2).astype(np.float64)
    norm_a = cv2.undistortPoints(pts_a, cam_a["K"], cam_a["D"], P=None).reshape(-1, 2)
    norm_b = cv2.undistortPoints(pts_b, cam_b["K"], cam_b["D"], P=None).reshape(-1, 2)
    x_a = np.hstack([norm_a, np.ones((len(norm_a), 1))])
    x_b = np.hstack([norm_b, np.ones((len(norm_b), 1))])

    R_rel, t_rel = _relative_pose(cam_a, cam_b)
    E = _skew(t_rel) @ R_rel

    e_xa = x_a @ E.T   # 各行 = E @ x_a（正規化座標の同次3ベクトル）
    et_xb = x_b @ E    # 各行 = E^T @ x_b
    numerator = np.sum(x_b * e_xa, axis=1)  # x_b^T E x_a
    denom = e_xa[:, 0] ** 2 + e_xa[:, 1] ** 2 + et_xb[:, 0] ** 2 + et_xb[:, 1] ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        sd = numerator ** 2 / denom

    f_a = (cam_a["K"][0, 0] + cam_a["K"][1, 1]) / 2.0
    f_b = (cam_b["K"][0, 0] + cam_b["K"][1, 1]) / 2.0
    f_mean = (f_a + f_b) / 2.0
    return np.sqrt(sd) * f_mean


def filter_cross_matches(cams: dict[str, dict], pairs: dict) -> dict:
    """Stage F: 確定ポーズでクロス対応点の偽マッチを除去する（design §2.45, FR-010）。

    対象は Stage P の採用ペア（status=="adopted"）のうち、両端がポーズ確定カメラ
    （cams[name]["rvec"] is not None）のペア。片端でもポーズ確定でないペアは
    不採用に降格する。

    処理ロジック（design §2.45 手順1〜5）:
      1〜3. _sampson_epipolar_px で d_px を計算し、d_px <= EPI_TOL_PX の対応のみ残す
      4. 生存数 < N_PAIR_MIN のペアは不採用に降格する。フィルタ後の採用ペアで
         グラフを構築し、全ポーズ確定カメラが単一連結成分に入らない場合は
         InputError（連結成分の内訳つき）を送出する（全ペア不採用も同様）
      5. ペアごとの（フィルタ前対応数、生存数、生存率、採用再判定）を返り値に含める

    境界条件: 生存0点のペアも「生存数0・不採用」として記録して継続する
    （エラーになるのは全ペア不採用時、または連結性不成立時のみ）。

    Returns:
        {(camA, camB): {"u_a", "u_b"（生存対応点のみ）, "n_before", "n_survive",
                        "survival_rate", "status"（"adopted"/"rejected"/
                        "excluded_not_pose_confirmed"）}}
    """
    pose_confirmed = {name for name, c in cams.items() if c.get("rvec") is not None}

    results: dict[tuple[str, str], dict] = {}
    for key, r in pairs.items():
        if r.get("status") != "adopted":
            continue
        cam_a, cam_b = key
        n_before = r["n"]

        if cam_a not in pose_confirmed or cam_b not in pose_confirmed:
            results[key] = {
                "u_a": np.zeros((0, 2), dtype=np.float64),
                "u_b": np.zeros((0, 2), dtype=np.float64),
                "n_before": n_before, "n_survive": 0, "survival_rate": 0.0,
                "status": "excluded_not_pose_confirmed",
            }
            continue

        d_px = _sampson_epipolar_px(r["u_a"], r["u_b"], cams[cam_a], cams[cam_b])
        keep = d_px <= EPI_TOL_PX
        n_survive = int(keep.sum())
        survival_rate = (n_survive / n_before) if n_before > 0 else 0.0
        status = "adopted" if n_survive >= N_PAIR_MIN else "rejected"
        results[key] = {
            "u_a": r["u_a"][keep], "u_b": r["u_b"][keep],
            "n_before": n_before, "n_survive": n_survive,
            "survival_rate": survival_rate, "status": status,
        }

    adopted_keys = [key for key, r in results.items() if r["status"] == "adopted"]
    if not adopted_keys:
        raise InputError(
            "エピポーラ整合フィルタ後の採用ペアが0件です。"
            "EPI_TOL_PX を見直すかカメラ配置を確認してください。")

    # 連結性判定（design §2.45 手順4。フィルタ後採用ペアのグラフで
    # 全ポーズ確定カメラが単一連結成分に入るかをUnion-Findで確認する）
    parent = {name: name for name in pose_confirmed}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x: str, y: str) -> None:
        rx, ry = find(x), find(y)
        if rx != ry:
            parent[rx] = ry

    for cam_a, cam_b in adopted_keys:
        union(cam_a, cam_b)

    components: dict[str, list[str]] = {}
    for name in pose_confirmed:
        components.setdefault(find(name), []).append(name)

    if len(components) > 1:
        detail = "; ".join(
            "{" + ", ".join(sorted(members)) + "}" for members in components.values())
        raise InputError(
            "エピポーラ整合フィルタ後の採用ペアグラフで全ポーズ確定カメラが単一連結成分に"
            f"入りません（連結成分: {detail}）。")

    return results


# ============================================================
# Stage A: アンカー対応点（design §2.4, FR-003）
# ============================================================

# design §2.4 は gate_breakdown（cond_disp を含む）を「アンカー取得〔§2.4〕専用」ゲートと
# 明示指定する（design §2.3-3 末尾の記載）が、cond_disp の許容画素変位 tau_px は §2.4 に
# 新規定義がなく、design §2.0 の定数一覧にも FR-008 の実験対象パラメータ一覧にも含まれない。
# ポーズ確定カメラは feat-026（refine_extrinsics.py）で収束済みのため、同スクリプトの反復
# スケジュール sched(k) のうち収束後の最終値 sched(1)["tau_px"]（=10.0px）を援用する
# （新規定数を追加しない解釈。design のレビューで要確認）。
ANCHOR_DISP_TAU_PX = sched(1)["tau_px"]


def _load_anchor_npz(out_npz: Path) -> tuple[np.ndarray, np.ndarray]:
    """collect_anchor_points が保存したアンカー対応点NPZを読み込む（再利用時）。"""
    d = np.load(out_npz, allow_pickle=False)
    return d["X_w"].astype(np.float64), d["u_i"].astype(np.float64)


def collect_anchor_points(cam: dict, gaussians: dict, query_png: Path,
                          tmp_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """Stage A: 1台のポーズ確定カメラのアンカー対応点を取得する（design §2.4 手順1〜3）。

    処理ロジック（design §2.4）:
      1. render_image(gaussians, cam, NEAR_PLANE, distort=True) でレンダPNGを作り
         （<tmp_dir>/render_<cam>.png に保存）、refine_extrinsics.call_matcher（LoFTR）で
         実写とマッチする。call_matcher は raw マッチ（全confidence）を返すので、
         conf >= LOFTR_CONF_TH（=0.2）の選別を明示的に適用する
      2. render_depth_alpha_distorted の深度・αマップと gate_breakdown 相当のゲートで
         レンダ側画素を選別する（design §2.3-3「gate_breakdown はアンカー取得〔§2.4〕専用」
         に基づき、Stage B の gate_depth_quality ではなく gate_breakdown を使う。判定式は
         feat-026 実装（run_iteration Stage 4）と同一: pixel_valid, var_map, tau_d を作り
         gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, ANCHOR_DISP_TAU_PX) を呼ぶ）。
         採択されたレンダ側画素は _lift_points_to_world で3DGS座標系3D点に変換する
         （3D化手順は §2.3-3 と同一）
      3. 対応点が ANCHOR_MAX_PTS を超える場合は LoFTR conf 降順に切り詰める

    アンカー対応点は <tmp_dir>/anchor_<cam>.npz に保存する（既存NPZの再利用判定は
    呼び出し側の責務。Stage P の run_pair_matching と同じ方針）。

    Returns:
        (X_w, u_i): X_w は (N_A,3) float64 3DGS座標系3D点。u_i は (N_A,2) float64 実写2D点
        （X_w[k] と u_i[k] が対応する）。
    """
    name = cam["name"]
    K, D = cam["K"], cam["D"]

    bgr = render_image(gaussians, cam, NEAR_PLANE, distort=True)
    render_png = tmp_dir / f"render_{name}.png"
    cv2.imwrite(str(render_png), bgr)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)

    match_npz = tmp_dir / f"anchor_loftr_{name}.npz"
    u_q, u_r, conf, _meta = call_matcher(query_png, render_png, match_npz)
    sel = conf >= LOFTR_CONF_TH
    u_q, u_r, conf = u_q[sel], u_r[sel], conf[sel]

    pixel_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                  & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))
    var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
    tau_d = GATE_VAR_REL * float(np.median(depth[pixel_valid]))
    bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, ANCHOR_DISP_TAU_PX)
    keep, d_bilin = bd["keep"], bd["d_bilin"]

    u_r_keep = u_r[keep]
    u_q_keep = u_q[keep]
    conf_keep = conf[keep]
    depths_keep = d_bilin[keep]

    X_w = _lift_points_to_world(u_r_keep, depths_keep, K, D, cam["rvec"], cam["tvec"])
    u_i = u_q_keep

    if len(X_w) > ANCHOR_MAX_PTS:
        order = np.argsort(-conf_keep)[:ANCHOR_MAX_PTS]
        X_w = X_w[order]
        u_i = u_i[order]

    np.savez(tmp_dir / f"anchor_{name}.npz", X_w=X_w, u_i=u_i)
    return X_w, u_i


# ============================================================
# Stage D: バンドル調整（design §2.5, FR-005）
# ============================================================

def select_ba_camera_names(target_names: list[str],
                           anchor_results: dict[str, tuple[np.ndarray, np.ndarray]]
                           ) -> list[str]:
    """BA参加カメラ（N_A >= N_ANCHOR_MIN）を選定する（design §2.4 手順4、FR-005 BA参加条件）。

    N_A < N_ANCHOR_MIN のカメラはクロス項のみでは並進スケールが観測不能になり得るため
    BA に参加させない（信頼初期ポーズを最終値とする。当該カメラを含む採用ペアのクロス
    対応点はBA・評価の対象から除外する）。
    """
    return [name for name in target_names if len(anchor_results[name][0]) >= N_ANCHOR_MIN]


def split_holdout(filter_results: dict[tuple[str, str], dict], ratio: float,
                  seed: int) -> dict[tuple[str, str], dict]:
    """ホールドアウト分割（design §2.6 前半）。Stage D の三角測量の前に行う。

    Stage F の採用ペア（status=="adopted"）ごとに `numpy.random.default_rng(seed)`
    でマッチをシャッフルし、`floor(N*ratio)` 点を評価専用（holdout）に、残りを
    学習用（train）に分割する。

    Returns:
        {(camA,camB): {"train": {"u_a","u_b"}, "holdout": {"u_a","u_b"}}}
        （status=="adopted" のペアのみを含む）
    """
    rng = np.random.default_rng(seed)
    result: dict[tuple[str, str], dict] = {}
    for key, r in filter_results.items():
        if r.get("status") != "adopted":
            continue
        u_a, u_b = r["u_a"], r["u_b"]
        n = len(u_a)
        perm = rng.permutation(n)
        n_holdout = int(np.floor(n * ratio))
        holdout_idx = perm[:n_holdout]
        train_idx = perm[n_holdout:]
        result[key] = {
            "train": {"u_a": u_a[train_idx], "u_b": u_b[train_idx]},
            "holdout": {"u_a": u_a[holdout_idx], "u_b": u_b[holdout_idx]},
        }
    return result


def _triangulate_normalized(u_a: np.ndarray, u_b: np.ndarray, cam_a: dict,
                            cam_b: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """正規化座標 + `[R|t]` によるクロス対応点の三角測量（design §2.5「クロス3D点の初期化」）。

    cv2.undistortPoints(uv, K, D, P=None) で正規化座標を得て、cv2.triangulatePoints には
    K を掛けない射影行列 P_i=[R_i|t_i] を渡す（ピクセル座標＋K[R|t]方式は使わない）。

    Returns:
        (Y, angle_deg, depth_a, depth_b): Y は (N,3) 三角測量された3DGS座標系3D点。
        angle_deg は (N,) 交会角[度]。depth_a/depth_b は (N,) 各カメラのカメラ座標系Z深度[m]。
    """
    pts_a = u_a.reshape(-1, 1, 2).astype(np.float64)
    pts_b = u_b.reshape(-1, 1, 2).astype(np.float64)
    norm_a = cv2.undistortPoints(pts_a, cam_a["K"], cam_a["D"], P=None).reshape(-1, 2).T
    norm_b = cv2.undistortPoints(pts_b, cam_b["K"], cam_b["D"], P=None).reshape(-1, 2).T

    R_a, _ = cv2.Rodrigues(np.asarray(cam_a["rvec"], dtype=np.float64).reshape(3, 1))
    R_b, _ = cv2.Rodrigues(np.asarray(cam_b["rvec"], dtype=np.float64).reshape(3, 1))
    t_a = np.asarray(cam_a["tvec"], dtype=np.float64).reshape(3, 1)
    t_b = np.asarray(cam_b["tvec"], dtype=np.float64).reshape(3, 1)
    P_a = np.hstack([R_a, t_a])
    P_b = np.hstack([R_b, t_b])

    Y_h = cv2.triangulatePoints(P_a, P_b, norm_a, norm_b)
    Y = (Y_h[:3] / Y_h[3]).T

    C_a = (-R_a.T @ t_a).reshape(3)
    C_b = (-R_b.T @ t_b).reshape(3)
    v_a = Y - C_a
    v_b = Y - C_b
    cos_ang = np.sum(v_a * v_b, axis=1) / (np.linalg.norm(v_a, axis=1) * np.linalg.norm(v_b, axis=1))
    cos_ang = np.clip(cos_ang, -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cos_ang))

    depth_a = (Y @ R_a.T + t_a.reshape(1, 3))[:, 2]
    depth_b = (Y @ R_b.T + t_b.reshape(1, 3))[:, 2]
    return Y, angle_deg, depth_a, depth_b


def crossing_angle_weight(theta_deg: np.ndarray) -> np.ndarray:
    """交会角重み w_k = exp(-(θ-THETA_OPT_DEG)^2/(2*THETA_SIGMA_DEG^2))（design §2.5）。

    先行研究 VISAPP2026 準拠のガウス型スコア（最適角30°）。初期三角測量時の交会角
    θ_k から1回だけ計算し、BA中は定数として扱う。
    """
    theta_deg = np.asarray(theta_deg, dtype=np.float64)
    return np.exp(-(theta_deg - THETA_OPT_DEG) ** 2 / (2.0 * THETA_SIGMA_DEG ** 2))


def build_cross_points(cams: dict[str, dict],
                       cross_pairs: dict[tuple[str, str], dict]) -> list[dict]:
    """Stage D: フィルタ・ホールドアウト分割後の学習側マッチを三角測量し、クロス3D点を作る
    （design §2.5「クロス3D点の初期化」）。トラック統合はしない（ペア単位2視点点）。

    Args:
        cams: カメラ名 -> {"K","D","rvec","tvec",...}（三角測量に使う確定ポーズ）
        cross_pairs: {(camA,camB): {"u_a","u_b"}}（ホールドアウト分割後の学習側マッチ）

    Returns:
        交会角 >= TRI_MIN_ANGLE_DEG かつ深度が両カメラとも Z_TRI_RANGE 内の点のみ。
        各要素: {"cam_a","cam_b","u_a"(2,),"u_b"(2,),"Y0"(3,),"weight"(float)}。
        weight は交会角重み無効化採用（p1_param_sweep 2026-08-25。design §2.5）により
        常に 1.0 固定（crossing_angle_weight は使わない。関数・単体テストは温存）。
    """
    points: list[dict] = []
    for (cam_a, cam_b), m in cross_pairs.items():
        u_a, u_b = m["u_a"], m["u_b"]
        if len(u_a) == 0:
            continue
        Y, angle_deg, depth_a, depth_b = _triangulate_normalized(u_a, u_b, cams[cam_a], cams[cam_b])
        keep = (
            (angle_deg >= TRI_MIN_ANGLE_DEG)
            & (depth_a >= Z_TRI_RANGE[0]) & (depth_a <= Z_TRI_RANGE[1])
            & (depth_b >= Z_TRI_RANGE[0]) & (depth_b <= Z_TRI_RANGE[1])
        )
        for i in np.nonzero(keep)[0]:
            points.append({
                "cam_a": cam_a, "cam_b": cam_b,
                "u_a": u_a[i].copy(), "u_b": u_b[i].copy(),
                "Y0": Y[i].copy(), "weight": 1.0,
            })
    return points


def _projected_residual(points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
                        K: np.ndarray, D: np.ndarray, uv: np.ndarray,
                        weight: float) -> np.ndarray:
    """3D点を投影し観測uvとの重み付き残差 (2N,) を返す（design §2.5 残差ベクトル）。

    残差NaN置換バリア（design §2.5 エラーハンドリング）: 3D点がカメラ背面に回った場合等に
    残差にNaNが生じ得るため、当該点の残差を1e3px（定数）に置換する。
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 1, 3)
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    with np.errstate(divide="ignore", invalid="ignore"):
        proj, _ = cv2.projectPoints(pts, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    resid = weight * (proj - np.asarray(uv, dtype=np.float64).reshape(-1, 2))
    resid = np.where(np.isfinite(resid), resid, 1e3)
    return resid.ravel()


@dataclass
class BAProblem:
    """build_ba_problem の戻り値（design §1.7）。

    x0: 未知数ベクトル（BA参加カメラの6パラメータ×n_cam + クロス3D点3パラメータ×M）
    sparsity: 残差数×未知数のヤコビアン疎パターン（lil_matrix）
    unpack: x を (poses: {cam_name:(rvec,tvec)}, Y:(M,3)) に復元する関数
    residual_fn: least_squares に渡す残差関数（NaNバリア適用済み）
    camera_names: BA参加カメラ名（x0のカメラブロック順、昇順ソート）
    cams: build_ba_problem に渡されたカメラ辞書（K・D固定値の参照用）
    anchor: build_ba_problem に渡されたアンカー対応点辞書
    cross_train: build_ba_problem に渡されたクロス3D点リスト
    """
    x0: np.ndarray
    sparsity: lil_matrix
    unpack: Callable[[np.ndarray], tuple[dict[str, tuple[np.ndarray, np.ndarray]], np.ndarray]]
    residual_fn: Callable[[np.ndarray], np.ndarray]
    camera_names: list[str]
    cams: dict[str, dict]
    anchor: dict[str, tuple[np.ndarray, np.ndarray]]
    cross_train: list[dict]


def build_ba_problem(cams: dict[str, dict], anchor: dict[str, tuple[np.ndarray, np.ndarray]],
                     cross_train: list[dict]) -> BAProblem:
    """Stage D: BA問題を構築する（design §2.5、§1.7）。

    未知数はBA参加カメラ（cams のキー全て）の (rvec,tvec) とクロス3D点 Y_k。K・Dは固定。
    残差ベクトルはこの順で連結する: アンカー項（カメラ名の昇順、各カメラ内は対応点順）→
    クロス項（cross_train のリスト順、各点内は cam_a→cam_b の順）。

    Args:
        cams: BA参加カメラのみ（キー=カメラ名、値の rvec/tvec が初期値=x0になる）
        anchor: {cam_name: (X_w(N_A,3), u_i(N_A,2))}（cams と同じカメラ集合）
        cross_train: build_cross_points の戻り値（BA参加カメラ間のペアに限定済み）
    """
    camera_names = sorted(cams.keys())
    n_cam = len(camera_names)
    m_points = len(cross_train)
    cam_index = {name: i for i, name in enumerate(camera_names)}

    x0 = np.zeros(6 * n_cam + 3 * m_points, dtype=np.float64)
    for i, name in enumerate(camera_names):
        x0[6 * i:6 * i + 3] = np.asarray(cams[name]["rvec"], dtype=np.float64).reshape(3)
        x0[6 * i + 3:6 * i + 6] = np.asarray(cams[name]["tvec"], dtype=np.float64).reshape(3)
    for k, pt in enumerate(cross_train):
        x0[6 * n_cam + 3 * k:6 * n_cam + 3 * k + 3] = pt["Y0"]

    def unpack(x: np.ndarray) -> tuple[dict[str, tuple[np.ndarray, np.ndarray]], np.ndarray]:
        poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for i, name in enumerate(camera_names):
            rvec = x[6 * i:6 * i + 3]
            tvec = x[6 * i + 3:6 * i + 6]
            poses[name] = (rvec, tvec)
        Y = x[6 * n_cam:].reshape(m_points, 3) if m_points else np.zeros((0, 3))
        return poses, Y

    anchor_blocks = [(name, anchor[name][0], anchor[name][1]) for name in camera_names]
    n_res_anchor = sum(2 * len(X_w) for _name, X_w, _u_i in anchor_blocks)
    n_res_cross = 4 * m_points
    n_res = n_res_anchor + n_res_cross

    sparsity = lil_matrix((n_res, len(x0)), dtype=np.int8)
    row = 0
    for name, X_w, _u_i in anchor_blocks:
        ci = cam_index[name]
        n = len(X_w)
        sparsity[row:row + 2 * n, 6 * ci:6 * ci + 6] = 1
        row += 2 * n
    for k, pt in enumerate(cross_train):
        ca, cb = cam_index[pt["cam_a"]], cam_index[pt["cam_b"]]
        pcol = 6 * n_cam + 3 * k
        sparsity[row:row + 2, 6 * ca:6 * ca + 6] = 1
        sparsity[row:row + 2, pcol:pcol + 3] = 1
        row += 2
        sparsity[row:row + 2, 6 * cb:6 * cb + 6] = 1
        sparsity[row:row + 2, pcol:pcol + 3] = 1
        row += 2

    def residual_fn(x: np.ndarray) -> np.ndarray:
        poses, Y = unpack(x)
        parts = []
        for name, X_w, u_i in anchor_blocks:
            rvec, tvec = poses[name]
            K, D = cams[name]["K"], cams[name]["D"]
            parts.append(_projected_residual(X_w, rvec, tvec, K, D, u_i, W_ANCHOR))
        for k, pt in enumerate(cross_train):
            w = W_CROSS * pt["weight"]
            rvec_a, tvec_a = poses[pt["cam_a"]]
            rvec_b, tvec_b = poses[pt["cam_b"]]
            Yk = Y[k:k + 1]
            Ka, Da = cams[pt["cam_a"]]["K"], cams[pt["cam_a"]]["D"]
            Kb, Db = cams[pt["cam_b"]]["K"], cams[pt["cam_b"]]["D"]
            parts.append(_projected_residual(Yk, rvec_a, tvec_a, Ka, Da,
                                             pt["u_a"].reshape(1, 2), w))
            parts.append(_projected_residual(Yk, rvec_b, tvec_b, Kb, Db,
                                             pt["u_b"].reshape(1, 2), w))
        if parts:
            return np.concatenate(parts)
        return np.zeros(0, dtype=np.float64)

    return BAProblem(x0=x0, sparsity=sparsity, unpack=unpack, residual_fn=residual_fn,
                     camera_names=camera_names, cams=cams, anchor=anchor,
                     cross_train=cross_train)


@dataclass
class BAResult:
    """run_bundle_adjustment の戻り値（design §1.7）。

    poses: BA後の全参加カメラのポーズ {cam_name: (rvec,tvec)}
    status: scipy.optimize.least_squares の終了ステータス（<1は非収束。中断はしない）
    cost: least_squares の最終コスト（0.5*sum(residual**2)）
    cost_initial: x0時点のコスト（design §1.8「BAの初期/最終コスト」ログ用）
    nfev: 関数評価回数（design §1.8「反復数」ログ用）
    """
    poses: dict[str, tuple[np.ndarray, np.ndarray]]
    status: int
    cost: float
    cost_initial: float
    nfev: int


def run_bundle_adjustment(problem: BAProblem) -> BAResult:
    """Stage D: scipy.optimize.least_squares によるBA本体（design §2.5「最適化」）。

    least_squares(residual_fn, x0, jac_sparsity=sparsity, method='trf', loss='huber',
    f_scale=HUBER_PX, max_nfev=MAX_NFEV, x_scale='jac')。status<1（非収束）でも中断せず、最終反復解を
    そのまま出力する（design §2.5 エラーハンドリング）。
    """
    x0 = problem.x0
    cost_initial = 0.5 * float(np.sum(problem.residual_fn(x0) ** 2))

    result = least_squares(problem.residual_fn, x0, jac_sparsity=problem.sparsity,
                           method="trf", loss="huber", f_scale=HUBER_PX, max_nfev=MAX_NFEV,
                           x_scale="jac")

    if result.status < 1:
        print(f"警告: BA が収束しませんでした（status={result.status}）。"
             "最終反復解をそのまま出力します。")

    poses, _Y = problem.unpack(result.x)
    return BAResult(poses=poses, status=int(result.status), cost=float(result.cost),
                    cost_initial=cost_initial, nfev=int(result.nfev))


def _anchor_residual_median_px(X_w: np.ndarray, u_i: np.ndarray, K: np.ndarray, D: np.ndarray,
                               rvec: np.ndarray, tvec: np.ndarray) -> float:
    """アンカー対応点への再投影残差（L2ノルム）の中央値[px]を返す（design §2.5 非悪化フォールバック）。"""
    proj, _ = cv2.projectPoints(X_w.reshape(-1, 1, 3),
                                np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                                np.asarray(tvec, dtype=np.float64).reshape(3, 1), K, D)
    resid = np.linalg.norm(proj.reshape(-1, 2) - u_i, axis=1)
    return float(np.median(resid))


def apply_ba_fallback(cams_before: dict[str, dict], ba_result: BAResult,
                      anchor: dict[str, tuple[np.ndarray, np.ndarray]]
                      ) -> tuple[dict[str, tuple[np.ndarray, np.ndarray]], list[dict]]:
    """非悪化フォールバック（design §2.5「非悪化フォールバック」、FR-005）。

    BA参加カメラごとに自カメラのアンカー対応点への再投影残差中央値を「BA前（信頼初期）
    ポーズ」と「BA後ポーズ」で比較し、悪化幅が BA_DEGRADE_TOL_PX(=0.3px) を超えたカメラは
    BA前ポーズに戻す。比較に使うアンカー対応点集合は Stage A で取得したものを固定して使う
    （BA前後で同一集合）。

    Returns:
        (final_poses, records): final_poses は {cam_name: (rvec,tvec)}（フォールバック発動
        カメラはBA前ポーズ、それ以外はBA後ポーズ）。records は
        [{"camera","median_before","median_after","degrade_px","fallback"}]。
    """
    final_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    records: list[dict] = []
    for name, (rvec_after, tvec_after) in ba_result.poses.items():
        X_w, u_i = anchor[name]
        K, D = cams_before[name]["K"], cams_before[name]["D"]
        median_before = _anchor_residual_median_px(
            X_w, u_i, K, D, cams_before[name]["rvec"], cams_before[name]["tvec"])
        median_after = _anchor_residual_median_px(X_w, u_i, K, D, rvec_after, tvec_after)
        degrade_px = median_after - median_before
        fallback = degrade_px > BA_DEGRADE_TOL_PX
        if fallback:
            final_poses[name] = (cams_before[name]["rvec"], cams_before[name]["tvec"])
        else:
            final_poses[name] = (rvec_after, tvec_after)
        records.append({
            "camera": name, "median_before": median_before, "median_after": median_after,
            "degrade_px": degrade_px, "fallback": fallback,
        })
    return final_poses, records


# ============================================================
# Stage E: ホールドアウト評価・レポート・TOML出力（design §2.6, FR-006/007）
# ============================================================

def _reprojection_residuals_px(X_w: np.ndarray, u_i: np.ndarray, K: np.ndarray, D: np.ndarray,
                               rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """3D点X_wを投影し観測u_iとの再投影残差（L2ノルム、px）を返す（重み・NaNバリアなし）。"""
    proj, _ = cv2.projectPoints(np.asarray(X_w, dtype=np.float64).reshape(-1, 1, 3),
                                np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                                np.asarray(tvec, dtype=np.float64).reshape(3, 1), K, D)
    return np.linalg.norm(proj.reshape(-1, 2) - np.asarray(u_i, dtype=np.float64).reshape(-1, 2), axis=1)


def _residual_stats_px(resid: np.ndarray) -> tuple[float, float]:
    """残差ノルム配列から (中央値, RMSE) を返す。空配列は (nan, nan)。"""
    if len(resid) == 0:
        return float("nan"), float("nan")
    return float(np.median(resid)), float(np.sqrt(np.mean(resid ** 2)))


def _triangulate_gated(cams_opt: dict[str, dict],
                       cross_pairs: dict[tuple[str, str], dict]) -> dict[tuple[str, str], dict]:
    """Stage E 共通処理: 最適化後ポーズでクロス対応点を三角測量し、ゲート通過後の点数と
    両カメラへの再投影残差を返す（design §2.6「評価」。§2.5「クロス3D点の初期化」と同一の
    正規化座標方式・同一ゲート〔TRI_MIN_ANGLE_DEG/Z_TRI_RANGE〕）。

    Args:
        cams_opt: {cam_name: {"K","D","rvec","tvec",...}}（最適化後＝BA後/フォールバック後の
            確定ポーズ）。
        cross_pairs: {(camA,camB): {"u_a","u_b"}}。

    Returns:
        {(camA,camB): {"resid": (2*n,) float64 両カメラの再投影残差ノルム[px]（cam_a側→cam_b側の順）,
                        "n_points": ゲート通過後の点数}}
        （マッチ0点またはゲート全滅のペアも "n_points"=0, "resid"=空配列で含む）
    """
    out: dict[tuple[str, str], dict] = {}
    for (cam_a, cam_b), m in cross_pairs.items():
        u_a, u_b = m["u_a"], m["u_b"]
        if len(u_a) == 0:
            out[(cam_a, cam_b)] = {"resid": np.zeros(0, dtype=np.float64), "n_points": 0}
            continue

        Y, angle_deg, depth_a, depth_b = _triangulate_normalized(u_a, u_b, cams_opt[cam_a], cams_opt[cam_b])
        keep = (
            (angle_deg >= TRI_MIN_ANGLE_DEG)
            & (depth_a >= Z_TRI_RANGE[0]) & (depth_a <= Z_TRI_RANGE[1])
            & (depth_b >= Z_TRI_RANGE[0]) & (depth_b <= Z_TRI_RANGE[1])
        )
        if not keep.any():
            out[(cam_a, cam_b)] = {"resid": np.zeros(0, dtype=np.float64), "n_points": 0}
            continue

        Yk = Y[keep]
        resid_a = _reprojection_residuals_px(Yk, u_a[keep], cams_opt[cam_a]["K"], cams_opt[cam_a]["D"],
                                             cams_opt[cam_a]["rvec"], cams_opt[cam_a]["tvec"])
        resid_b = _reprojection_residuals_px(Yk, u_b[keep], cams_opt[cam_b]["K"], cams_opt[cam_b]["D"],
                                             cams_opt[cam_b]["rvec"], cams_opt[cam_b]["tvec"])
        out[(cam_a, cam_b)] = {
            "resid": np.concatenate([resid_a, resid_b]),
            "n_points": int(keep.sum()),
        }
    return out


def evaluate_holdout(cams_opt: dict[str, dict],
                     cross_holdout: dict[tuple[str, str], dict]) -> dict:
    """Stage E: ホールドアウト評価（design §2.6「評価」、§1.7、FR-006(c)）。

    最適化後ポーズでホールドアウトのクロス対応点を三角測量（§2.5と同一の正規化座標方式・
    同一ゲート〔TRI_MIN_ANGLE_DEG/Z_TRI_RANGE〕）し、両カメラへの再投影残差を全点分集計する。

    Args:
        cams_opt: {cam_name: {"K","D","rvec","tvec",...}}（最適化後の確定ポーズ。
            cross_holdout に現れるカメラを全て含むこと）。
        cross_holdout: {(camA,camB): {"u_a","u_b"}}（Stage Fフィルタ後・ホールドアウト
            分割後の評価専用マッチ）。

    Returns:
        {"n_points": ゲート通過後の総点数, "n_residuals": 集計残差数(=2*n_points),
         "median_px": 残差ノルムの中央値[px], "rmse_px": 残差ノルムのRMSE[px]}
        （対象点が0件の場合 median_px/rmse_px は nan）
    """
    by_pair = _triangulate_gated(cams_opt, cross_holdout)
    resid_all = (np.concatenate([r["resid"] for r in by_pair.values()])
                if by_pair else np.zeros(0, dtype=np.float64))
    n_points = sum(r["n_points"] for r in by_pair.values())
    median_px, rmse_px = _residual_stats_px(resid_all)
    return {"n_points": n_points, "n_residuals": len(resid_all),
           "median_px": median_px, "rmse_px": rmse_px}


def write_multiview_toml(intrinsics_toml_path: str, out_path: str,
                         poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
    """Stage E: 出力TOMLの専用ライタ（design §2.6「TOML」、§1.7、FR-007）。

    poses（camera_name -> (rvec, tvec)）に含まれるカメラのみを、入力TOML（intrinsics_all.toml、
    K・D・サイズの出典）の値と poses の最終ポーズから refine_extrinsics._format_camera_section
    で書き出す。既存の write_output_toml は入力TOMLの全カメラを書き出す仕様のため流用しない
    （poses に含まれないカメラは出力TOMLに一切含めない）。

    Raises:
        InputError: poses に指定されたカメラ名が入力TOMLに存在しない場合。
        ValueError: 対応するTOMLセクションがカメラセクション（matrixキー付き）でない場合。
    """
    with open(intrinsics_toml_path, "rb") as f:
        data = tomli.load(f)

    missing = [name for name in poses if name not in data]
    if missing:
        raise InputError(
            f"write_multiview_toml: poses に指定されたカメラが入力TOMLに存在しません: {missing}")

    sections = []
    for key, value in data.items():
        if key not in poses:
            continue
        if not isinstance(value, dict) or "matrix" not in value:
            raise ValueError(f"未対応のTOMLセクション形式です: [{key}]")
        rvec, tvec = poses[key]
        sections.append(_format_camera_section(key, value, rvec, tvec))

    tmp_path = Path(out_path).with_suffix(Path(out_path).suffix + ".tmp")
    with open(tmp_path, "w", encoding="utf-8") as f:
        f.write("\n".join(sections))
    tmp_path.replace(out_path)


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

        # --init-cameras 全カメラ必須チェック（design §2.7。案A: 全対象カメラの
        # 初期ポーズが必須。build_camera_dict のマージ規則により、--init-cameras
        # で信頼指定されなかったカメラは rvec が None のままになっている）
        missing_pose = [name for name in target_names if cams[name]["rvec"] is None]
        if missing_pose:
            raise InputError(
                "対象カメラに --init-cameras で初期ポーズが確定していないカメラが"
                f"含まれています（案A: 全対象カメラの初期ポーズが必須です）: {missing_pose}")

        # ポーズ確定カメラ（= target_names。上のチェックにより全て確定済み）が
        # 2台未満の場合はクロス項を構成できないため Stage F に進まずエラー（design §2.7/2.5）
        if len(target_names) < 2:
            raise InputError(
                "ポーズ確定カメラが2台未満です。クロス項を構成できないため "
                "Stage F を実行できません。")

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

        print("=== Stage F: エピポーラ整合フィルタ ===")
        target_cams = {name: cams[name] for name in target_names}
        filter_results = filter_cross_matches(target_cams, pair_results)

        print("--- エピポーラフィルタ結果一覧 ---")
        n_f_adopted = 0
        for (cam_a, cam_b), r in sorted(filter_results.items()):
            mark = "採用" if r["status"] == "adopted" else "不採用"
            if r["status"] == "adopted":
                n_f_adopted += 1
            print(f"{cam_a} - {cam_b}: フィルタ前対応数={r['n_before']} "
                 f"生存数={r['n_survive']} 生存率={r['survival_rate']:.3f} "
                 f"status={r['status']}（{mark}）")
        print(f"フィルタ後採用ペア数: {n_f_adopted}/{len(filter_results)}")
        print("連結性判定: OK（全ポーズ確定カメラが単一連結成分に属します）")

        print("=== Stage A: アンカー対応点 ===")
        from render import load_ply  # phase4 venv のみ。遅延 import（design §1.2）
        gaussians = load_ply(args.ply)

        anchor_results: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name in target_names:
            cam = cams[name]
            query_png = images_dir / f"{name}.png"
            anchor_npz = tmp_dir / f"anchor_{name}.npz"
            if anchor_npz.exists() and not args.fresh_match:
                X_w, u_i = _load_anchor_npz(anchor_npz)
            else:
                X_w, u_i = collect_anchor_points(cam, gaussians, query_png, tmp_dir)
            anchor_results[name] = (X_w, u_i)

        ba_camera_names = select_ba_camera_names(target_names, anchor_results)
        ba_camera_set = set(ba_camera_names)

        print("--- カメラ別アンカー対応点数一覧 ---")
        for name in target_names:
            n_a = len(anchor_results[name][0])
            status = "OK" if name in ba_camera_set else "アンカー不足（BA未参加）"
            print(f"{name}: N_A={n_a} status={status}")
        print(f"BA参加カメラ数: {len(ba_camera_names)}/{len(target_names)}")

        print("=== Stage D: バンドル調整 ===")
        if len(ba_camera_names) < 2:
            raise InputError(
                "BA参加カメラ（ポーズ確定かつ N_A >= N_ANCHOR_MIN）が2台未満です。"
                "クロス項を構成できないため BA を実行できません。")

        holdout_split = split_holdout(filter_results, HOLDOUT_RATIO, args.seed)
        cross_train_pairs = {
            key: v["train"] for key, v in holdout_split.items()
            if key[0] in ba_camera_set and key[1] in ba_camera_set
        }
        ba_cams = {name: cams[name] for name in ba_camera_names}
        cross_train = build_cross_points(ba_cams, cross_train_pairs)
        print(f"クロス3D点数（学習用、三角測量ゲート通過後）: {len(cross_train)}")
        if len(cross_train) == 0:
            raise InputError(
                "クロス対応点が全て三角測量ゲート（TRI_MIN_ANGLE_DEG/Z_TRI_RANGE）で"
                "除外されました（M=0）。BA を実行できません。")

        ba_anchor = {name: anchor_results[name] for name in ba_camera_names}
        problem = build_ba_problem(ba_cams, ba_anchor, cross_train)
        ba_result = run_bundle_adjustment(problem)
        print(f"BA コスト: 初期={ba_result.cost_initial:.6g} 最終={ba_result.cost:.6g} "
             f"status={ba_result.status} nfev={ba_result.nfev}")

        # 非悪化フォールバック（design §2.5）。final_poses は Step 6 のTOML出力で使用する
        final_poses, fallback_records = apply_ba_fallback(ba_cams, ba_result, ba_anchor)
        print("--- 非悪化フォールバック判定（アンカー残差中央値のBA前後比較） ---")
        n_fallback = 0
        for rec in fallback_records:
            if rec["fallback"]:
                n_fallback += 1
                mark = "フォールバック発動（BA前ポーズに復帰）"
            else:
                mark = "BA後ポーズを維持"
            print(f"{rec['camera']}: 残差中央値 BA前={rec['median_before']:.4f}px "
                 f"BA後={rec['median_after']:.4f}px 悪化幅={rec['degrade_px']:+.4f}px "
                 f"（{mark}）")
        print(f"フォールバック発動カメラ数: {n_fallback}/{len(fallback_records)}")

        print("=== Stage E: ホールドアウト評価・レポート・TOML出力 ===")
        non_ba_names = [name for name in target_names if name not in ba_camera_set]
        # ポーズ確定カメラ全員分の最終ポーズ（design §2.6 TOML: (1)BA参加カメラのBA後
        # ポーズ〔フォールバック後含む〕, (2)アンカー不足でBA未参加のポーズ確定カメラの
        # 信頼初期ポーズ）
        final_pose_all: dict[str, tuple[np.ndarray, np.ndarray]] = dict(final_poses)
        for name in non_ba_names:
            final_pose_all[name] = (cams[name]["rvec"], cams[name]["tvec"])

        # (b)(c) 用: BA参加カメラの最終ポーズを持つカメラ辞書（K・Dはcams由来で不変）
        cams_final = {
            name: {**ba_cams[name], "rvec": final_pose_all[name][0], "tvec": final_pose_all[name][1]}
            for name in ba_camera_names
        }

        excluded_cameras = sorted(set(cams.keys()) - set(target_names))

        report_lines: list[str] = []
        report_lines.append("feat-035 多視点外部パラメータ調整 診断レポート")
        report_lines.append("=" * 60)

        # (a) カメラ別アンカー残差（中央値・RMSE。BA前/BA後の両方）
        report_lines.append("")
        report_lines.append("(a) カメラ別アンカー残差（中央値・RMSE、BA前/BA後）")
        report_lines.append("-" * 60)
        for name in target_names:
            X_w, u_i = anchor_results[name]
            K, D = cams[name]["K"], cams[name]["D"]
            resid_before = _reprojection_residuals_px(X_w, u_i, K, D,
                                                       cams[name]["rvec"], cams[name]["tvec"])
            med_before, rmse_before = _residual_stats_px(resid_before)
            rvec_after, tvec_after = final_pose_all[name]
            resid_after = _reprojection_residuals_px(X_w, u_i, K, D, rvec_after, tvec_after)
            med_after, rmse_after = _residual_stats_px(resid_after)
            report_lines.append(
                f"{name}: N_A={len(X_w)} "
                f"BA前 median={med_before:.4f}px rmse={rmse_before:.4f}px / "
                f"BA後 median={med_after:.4f}px rmse={rmse_after:.4f}px")

        # (b) ペア別クロス残差（中央値・RMSE。学習用マッチ、BA後〔フォールバック後含む〕ポーズで再三角測量）
        report_lines.append("")
        report_lines.append("(b) ペア別クロス残差（中央値・RMSE、学習用マッチ、最終ポーズで再三角測量）")
        report_lines.append("-" * 60)
        cross_train_resid = _triangulate_gated(cams_final, cross_train_pairs)
        for cam_a, cam_b in sorted(cross_train_resid.keys()):
            r = cross_train_resid[(cam_a, cam_b)]
            med, rmse = _residual_stats_px(r["resid"])
            report_lines.append(
                f"{cam_a} - {cam_b}: n_points={r['n_points']} "
                f"median={med:.4f}px rmse={rmse:.4f}px")

        # (c) ホールドアウト三角測量再投影残差（中央値・RMSE）
        report_lines.append("")
        report_lines.append("(c) ホールドアウト三角測量再投影残差（中央値・RMSE）")
        report_lines.append("-" * 60)
        cross_holdout_pairs = {
            key: v["holdout"] for key, v in holdout_split.items()
            if key[0] in ba_camera_set and key[1] in ba_camera_set
        }
        holdout_eval = evaluate_holdout(cams_final, cross_holdout_pairs)
        report_lines.append(
            f"n_points={holdout_eval['n_points']} n_residuals={holdout_eval['n_residuals']} "
            f"median={holdout_eval['median_px']:.4f}px rmse={holdout_eval['rmse_px']:.4f}px")

        # (d) カメラ別の初期→最終ポーズ変化量（cm・度）
        report_lines.append("")
        report_lines.append("(d) カメラ別の初期→最終ポーズ変化量（cm・度）")
        report_lines.append("-" * 60)
        for name in target_names:
            rvec_after, tvec_after = final_pose_all[name]
            dpos_m, dang_deg = pose_diff(cams[name]["rvec"], cams[name]["tvec"], rvec_after, tvec_after)
            report_lines.append(f"{name}: 位置変化={dpos_m * 100:.4f}cm 回転変化={dang_deg:.4f}deg")

        # (e) ポーズ確定カメラ一覧と除外カメラ一覧
        report_lines.append("")
        report_lines.append("(e) ポーズ確定カメラ一覧と除外カメラ一覧")
        report_lines.append("-" * 60)
        report_lines.append(f"ポーズ確定カメラ(n={len(target_names)}): {', '.join(target_names)}")
        excluded_str = ", ".join(excluded_cameras) if excluded_cameras else "(なし)"
        report_lines.append(f"除外カメラ(n={len(excluded_cameras)}): {excluded_str}")

        # (f) 28ペアのマッチ数一覧
        report_lines.append("")
        report_lines.append("(f) 全ペアのマッチ数一覧（Stage P）")
        report_lines.append("-" * 60)
        for cam_a, cam_b in sorted(pair_results.keys()):
            r = pair_results[(cam_a, cam_b)]
            report_lines.append(f"{cam_a} - {cam_b}: N={r['n']} status={r['status']}")

        # (g) 実行条件（入力パス、パラメータ値、シード、日時）
        report_lines.append("")
        report_lines.append("(g) 実行条件")
        report_lines.append("-" * 60)
        report_lines.append(f"日時: {datetime.datetime.now().isoformat()}")
        report_lines.append(f"--toml: {args.toml}")
        report_lines.append(f"--ply: {args.ply}")
        report_lines.append(f"--images-dir: {args.images_dir}")
        report_lines.append(f"--init-toml: {args.init_toml}")
        report_lines.append(f"--init-cameras: {args.init_cameras}")
        report_lines.append(f"--out-toml: {args.out_toml}")
        report_lines.append(f"--out-report: {args.out_report}")
        report_lines.append(f"--cameras: {args.cameras}")
        report_lines.append(f"--seed: {args.seed}")
        report_lines.append(f"--tmp-dir: {tmp_dir}")
        report_lines.append(f"--fresh-match: {args.fresh_match}")
        report_lines.append(f"N_PAIR_MIN={N_PAIR_MIN}")
        report_lines.append(f"HUBER_PX={HUBER_PX}")
        report_lines.append(f"W_ANCHOR={W_ANCHOR}")
        report_lines.append(f"W_CROSS={W_CROSS}")
        report_lines.append(f"HOLDOUT_RATIO={HOLDOUT_RATIO}")
        report_lines.append(f"ANCHOR_MAX_PTS={ANCHOR_MAX_PTS}")
        report_lines.append(f"MAX_NFEV={MAX_NFEV}")
        report_lines.append(f"TRI_MIN_ANGLE_DEG={TRI_MIN_ANGLE_DEG}")
        report_lines.append(f"Z_TRI_RANGE={Z_TRI_RANGE}")
        report_lines.append(f"N_ANCHOR_MIN={N_ANCHOR_MIN}")
        report_lines.append(f"BA_DEGRADE_TOL_PX={BA_DEGRADE_TOL_PX}")
        report_lines.append(f"THETA_OPT_DEG={THETA_OPT_DEG}")
        report_lines.append(f"THETA_SIGMA_DEG={THETA_SIGMA_DEG}")
        report_lines.append(f"EPI_TOL_PX={EPI_TOL_PX}")
        report_lines.append(f"ANCHOR_DISP_TAU_PX={ANCHOR_DISP_TAU_PX}")

        # (h) エピポーラ整合フィルタのペア別生存数・生存率と採用再判定（FR-010）
        report_lines.append("")
        report_lines.append("(h) エピポーラ整合フィルタのペア別生存数・生存率と採用再判定")
        report_lines.append("-" * 60)
        for cam_a, cam_b in sorted(filter_results.keys()):
            r = filter_results[(cam_a, cam_b)]
            report_lines.append(
                f"{cam_a} - {cam_b}: フィルタ前対応数={r['n_before']} "
                f"生存数={r['n_survive']} 生存率={r['survival_rate']:.4f} status={r['status']}")

        # (i) BAフォールバックの有無（カメラ別）
        report_lines.append("")
        report_lines.append("(i) BAフォールバックの有無（カメラ別）")
        report_lines.append("-" * 60)
        for rec in fallback_records:
            report_lines.append(
                f"{rec['camera']}: 悪化幅={rec['degrade_px']:+.4f}px "
                f"フォールバック={'あり' if rec['fallback'] else 'なし'}")
        if non_ba_names:
            report_lines.append(
                f"BA未参加（アンカー不足。フォールバック判定なし・信頼初期ポーズを採用）: "
                f"{', '.join(non_ba_names)}")

        Path(args.out_report).write_text("\n".join(report_lines) + "\n", encoding="utf-8")
        write_multiview_toml(args.toml, args.out_toml, final_pose_all)
        print(f"レポートを書き出しました: {args.out_report}")
        print(f"出力TOMLを書き出しました: {args.out_toml}")

    except InputError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

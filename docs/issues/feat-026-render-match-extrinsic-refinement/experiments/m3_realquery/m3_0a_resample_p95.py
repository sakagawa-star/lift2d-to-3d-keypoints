"""feat-026 M3-0a: Config_scene の K・歪みでの n0=10 再サンプリング再計測

criteria.md §5.3 の一致性閾値（n0=10 再サンプリング分布の p95）を数値確定するための
決定的計算。M1 スクリプト（../m1/run_m1_experiment.py）の内部パラメータ入力を
intrinsics_all.toml から Config_scene.toml（M3 で実際に使う K・歪み）に差し替えたもの。
それ以外のロジック（PnP 経路・seed・試行数・差の定義）は M1 と同一に保つ。

実行方法（プロジェクトルートで）:
    uv run python docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_resample_p95.py

出力: このスクリプトと同じディレクトリに m3_0a_results_trials.csv / m3_0a_results_summary.txt
"""
import csv
import re
import sys
from pathlib import Path

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # phase0 環境（Python 3.10）

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase0"))
from estimate_camera_params import estimate_extrinsic  # noqa: E402

DATA_DIR = ROOT / "phase0" / "data"
OUT_DIR = Path(__file__).resolve().parent

# M3 で使う K・歪み（criteria.md §2。M1 の intrinsics_all.toml から差し替え）
TOML_PATH = Path("/home/sakagawa/git/pose2sim/Pose2Sim/20260418_osaka-hosp1/calibration/Config_scene.toml")
POINTS_3D_PATH = DATA_DIR / "kijunten_locations_osaka2.csv"
POINTS_2D_PATH = DATA_DIR / "points_2d_osaka2.csv"

N_TRIALS = 1000     # M1 と同一
N0_LIST = [10]      # criteria.md §5.3 の物差しは n0=10 のみ
SEED = 42           # M1 と同一

# 閾値確定に必須の判定対象5カメラ（criteria.md §3）。欠けたまま完走させない
REQUIRED_CAMS = {"cam05520126", "cam05520129", "cam41520554", "cam41520556", "cam41520557"}


def load_intrinsics_toml(path: Path) -> dict:
    """Calib_scene TOML から カメラ名→(K, dist) を読む。

    [metadata] セクションが存在する場合は除去してからパースする（M1 と同一処理。
    Config_scene.toml に [metadata] はないが、除去処理は無害なので共通のまま残す）。
    """
    text = path.read_text()
    text = re.sub(r"^\[metadata\]\n(?:^(?!\[).*\n?)*", "", text, flags=re.M)
    data = tomllib.loads(text)
    cams = {}
    for key, v in data.items():
        if isinstance(v, dict) and "matrix" in v:
            cams[key] = {
                "K": np.array(v["matrix"], dtype=np.float64),
                "dist": np.array(v["distortions"], dtype=np.float64),
            }
    return cams


def load_matched_points(cam_name: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """ObjectName で 3D-2D をマッチし (points_3d, points_2d, names) を返す。"""
    pts3d = {}
    with open(POINTS_3D_PATH) as f:
        for row in csv.DictReader(f):
            pts3d[row["ObjectName"]] = [float(row["X"]), float(row["Y"]), float(row["Z"])]
    p3, p2, names = [], [], []
    with open(POINTS_2D_PATH) as f:
        for row in csv.DictReader(f):
            if row["camera_name"] == cam_name and row["ObjectName"] in pts3d:
                p3.append(pts3d[row["ObjectName"]])
                p2.append([float(row["X"]), float(row["Y"])])
                names.append(row["ObjectName"])
    return (np.array(p3, dtype=np.float64), np.array(p2, dtype=np.float64), names)


def pose_diff(rvec_a, tvec_a, rvec_b, tvec_b) -> tuple[float, float]:
    """2ポーズの差: (カメラ中心距離[m], 回転角[deg])"""
    import cv2
    Ra, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64))
    Rb, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64))
    Ca = -Ra.T @ np.asarray(tvec_a, dtype=np.float64).reshape(3)
    Cb = -Rb.T @ np.asarray(tvec_b, dtype=np.float64).reshape(3)
    dpos = float(np.linalg.norm(Ca - Cb))
    cos_t = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    dang = float(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    return dpos, dang


def main():
    import cv2
    cv2.setRNGSeed(SEED)  # solvePnPRansac の RANSAC 抽選も固定（抽出乱数の seed とは別系統）
    rng = np.random.default_rng(SEED)
    cams = load_intrinsics_toml(TOML_PATH)
    print(f"内部パラメータTOML: {TOML_PATH}")
    print(f"カメラ: {sorted(cams.keys())}")

    trial_rows = []
    summary_lines = [
        "feat-026 M3-0a: Config_scene の K・歪みでの n0=10 再サンプリング再計測",
        f"目的: criteria.md §5.3 一致性閾値（p95）の数値確定",
        f"データ: osaka2 / 試行: {N_TRIALS}回 x n0={N0_LIST} / seed={SEED}（numpy 抽出・cv2.setRNGSeed の両方）",
        f"内部パラメータ: {TOML_PATH}",
        "PnP実装: phase0/estimate_camera_params.py estimate_extrinsic（実運用と同一、M1 と同一）",
        f"バージョン: OpenCV {cv2.__version__} / NumPy {np.__version__}",
        "",
    ]
    done_cams = set()

    for cam_name, cam in sorted(cams.items()):
        p3, p2, names = load_matched_points(cam_name)
        n_all = len(p3)
        print(f"\n=== {cam_name}: マッチ対応 {n_all} 点")
        if n_all < 15:
            if cam_name in REQUIRED_CAMS:
                raise RuntimeError(f"{cam_name}（判定対象カメラ）の対応点が {n_all} 点しかない")
            print("  対応点が少ないためスキップ")
            continue

        # 基準ポーズ: 全点で推定（実運用と同じ経路: RANSAC + ITERATIVE）
        ref = estimate_extrinsic(p3, p2, cam["K"], cam["dist"], names)
        if ref is None:
            if cam_name in REQUIRED_CAMS:
                raise RuntimeError(f"{cam_name}（判定対象カメラ）の基準ポーズ推定に失敗")
            print("  基準ポーズ推定に失敗。スキップ")
            continue
        n_inlier = len(ref["inliers"])
        summary_lines.append(
            f"[{cam_name}] 全点={n_all}, 基準ポーズinlier={n_inlier}, "
            f"outlier={ref['outliers']}"
        )

        for n0 in N0_LIST:
            dposs, dangs, fails = [], [], 0
            for _ in range(N_TRIALS):
                idx = rng.choice(n_all, size=n0, replace=False)
                est = estimate_extrinsic(p3[idx], p2[idx], cam["K"], cam["dist"],
                                         [names[i] for i in idx])
                if est is None:
                    fails += 1
                    continue
                dpos, dang = pose_diff(est["rvec"], est["tvec"], ref["rvec"], ref["tvec"])
                dposs.append(dpos)
                dangs.append(dang)
                trial_rows.append([cam_name, n0, f"{dpos:.6f}", f"{dang:.4f}"])
            dposs = np.array(dposs)
            dangs = np.array(dangs)
            if len(dposs) == 0:
                if cam_name in REQUIRED_CAMS:
                    raise RuntimeError(f"{cam_name}（判定対象カメラ）で n0={n0} の全試行が失敗")
                summary_lines.append(f"  n0={n0}: 全試行失敗")
                continue
            line = (
                f"  n0={n0:2d}: 位置差[cm] 中央値={np.median(dposs)*100:6.2f} "
                f"p90={np.percentile(dposs, 90)*100:7.2f} p95={np.percentile(dposs, 95)*100:7.2f} "
                f"最大={dposs.max()*100:8.2f} | "
                f"角度差[deg] 中央値={np.median(dangs):5.2f} "
                f"p90={np.percentile(dangs, 90):6.2f} p95={np.percentile(dangs, 95):6.2f} "
                f"最大={dangs.max():7.2f} | 失敗={fails}"
            )
            print(line)
            summary_lines.append(line)
            done_cams.add(cam_name)
        summary_lines.append("")

    missing = REQUIRED_CAMS - done_cams
    if missing:
        raise RuntimeError(f"判定対象カメラの結果が欠けている: {sorted(missing)}")

    with open(OUT_DIR / "m3_0a_results_trials.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["camera", "n0", "dpos_m", "dang_deg"])
        w.writerows(trial_rows)
    (OUT_DIR / "m3_0a_results_summary.txt").write_text("\n".join(summary_lines) + "\n")
    print(f"\n保存: {OUT_DIR / 'm3_0a_results_trials.csv'}, {OUT_DIR / 'm3_0a_results_summary.txt'}")


if __name__ == "__main__":
    main()

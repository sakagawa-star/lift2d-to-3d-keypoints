"""feat-026 M1（実験B）: 少点数PnPの初期値精度検証

既存データ（osaka2、対象環境と同一カメラ機種）を用い、全点PnPを基準ポーズとして
5〜6点ランダム抽出PnPとの差（位置・角度）の分布を測る。
PnP は phase0/estimate_camera_params.py の estimate_extrinsic をそのまま使用する
（6点以上: solvePnPRansac+ITERATIVE精密化 / 4〜5点: SQPnP。実運用と同一挙動）。

実行方法（プロジェクトルートで）:
    uv run python docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m1/run_m1_experiment.py

出力: このスクリプトと同じディレクトリに results_trials.csv / results_summary.txt
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

TOML_PATH = DATA_DIR / "osaka2" / "intrinsics_all.toml"  # チェスボード較正済み内部パラメータ（ユーザー指定）
POINTS_3D_PATH = DATA_DIR / "kijunten_locations_osaka2.csv"
POINTS_2D_PATH = DATA_DIR / "points_2d_osaka2.csv"

N_TRIALS = 1000          # n0 ごとの抽出試行回数
N0_LIST = [4, 5, 6, 8, 10]  # 主対象は 5, 6。4/8/10 は文脈用
SEED = 42

# 収束域の判定基準（pipeline.md §4.2(b-1): 位置 数cm・角度 数度）
THRESHOLDS = [(0.05, 2.0), (0.10, 5.0)]  # (位置[m], 角度[deg])


def load_intrinsics_toml(path: Path) -> dict:
    """Calib_scene TOML から カメラ名→(K, dist) を読む。

    既存ファイルは [metadata] セクションが重複しており標準TOMLとして不正なため、
    [metadata] セクションを除去してからパースする。
    """
    text = path.read_text()
    # [metadata] から次のセクションヘッダまでを除去
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
    rng = np.random.default_rng(SEED)
    cams = load_intrinsics_toml(TOML_PATH)
    print(f"内部パラメータTOML: {sorted(cams.keys())}")

    trial_rows = []
    summary_lines = [
        "feat-026 M1（実験B）: 少点数PnPの初期値精度検証",
        f"データ: osaka2（対象環境と同一カメラ機種） / 試行: {N_TRIALS}回 x n0={N0_LIST} / seed={SEED}",
        f"内部パラメータ: {TOML_PATH.name}（チェスボード較正済み、5係数）",
        f"PnP実装: phase0/estimate_camera_params.py estimate_extrinsic（実運用と同一）",
        "",
    ]

    for cam_name, cam in sorted(cams.items()):
        p3, p2, names = load_matched_points(cam_name)
        n_all = len(p3)
        print(f"\n=== {cam_name}: マッチ対応 {n_all} 点")
        if n_all < 15:
            print("  対応点が少ないためスキップ")
            continue

        # 基準ポーズ: 全点で推定（実運用と同じ経路: RANSAC + ITERATIVE）
        ref = estimate_extrinsic(p3, p2, cam["K"], cam["dist"], names)
        if ref is None:
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
            for th_pos, th_ang in THRESHOLDS:
                ok = np.mean((dposs < th_pos) & (dangs < th_ang)) * 100
                line += f" | <{th_pos*100:.0f}cm&{th_ang:.0f}deg: {ok:5.1f}%"
            print(line)
            summary_lines.append(line)
        summary_lines.append("")

    with open(OUT_DIR / "results_trials.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["camera", "n0", "dpos_m", "dang_deg"])
        w.writerows(trial_rows)
    (OUT_DIR / "results_summary.txt").write_text("\n".join(summary_lines) + "\n")
    print(f"\n保存: {OUT_DIR / 'results_trials.csv'}, {OUT_DIR / 'results_summary.txt'}")


if __name__ == "__main__":
    main()

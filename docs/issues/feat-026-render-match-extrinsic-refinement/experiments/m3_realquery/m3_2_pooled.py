"""feat-026 M3-2: チェーン統合型合意判定の検証(保存済みサンプルの再分析)

criteria_m3_2.md(locked 2026-07-31)に基づき、M3-1 で保存した各カメラ
P0/P1/P2 の全60サンプル(m3_1_history.npz)を統合(pooled)して合意クラスタ
分析を行い、§3 の3判定条件を照合する。新規レンダは行わない(決定的な再計算)。

クラスタ分析は m3_1_sampling.cluster_analysis をそのまま使う(同一ロジック保証)。
※ m3_1_sampling の import は m3_run 経由で gsplat/torch を読み込むため、
   phase4 環境で実行する(GPU は使用しない)。

実行方法(プロジェクトルートで):
    uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_2_pooled.py

出力: m3_2_result.txt(コミット対象)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from m3_1_sampling import (  # noqa: E402  # M3-1 と同一のクラスタ分析・定数
    cluster_analysis, N_S, R_POS, R_ANG, F_C_MIN, F2_BIMODAL,
)
from m3_run import pose_diff, CAMERAS, P95_THRESHOLDS  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402
from stage2_smoke_test import TOML_PATH  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
HISTORY_NPZ = OUT_DIR / "m3_1_history.npz"

POOL_SERIES = ("P0", "P1", "P2")
N_POOL = N_S * len(POOL_SERIES)          # 60
F_C_MIN_COUNT = int(np.ceil(F_C_MIN * N_POOL))  # 42
TARGET_FAIL_CAM = "cam41520554"          # 検出されるべきカメラ(criteria §3 条件1)


def main():
    hist = np.load(HISTORY_NPZ, allow_pickle=True)
    results = hist["results"][0]
    neg = hist["neg"][0]
    cams_cfg = load_cameras_toml(TOML_PATH)

    lines = [
        "feat-026 M3-2: チェーン統合型合意判定の検証(criteria_m3_2.md locked 2026-07-31)",
        f"pooled N={N_POOL}(3チェーン×{N_S}) r_pos={R_POS*100:.0f}cm r_ang={R_ANG}deg "
        f"受理=f_c≥{F_C_MIN}(={F_C_MIN_COUNT}/{N_POOL}) 二峰閾値={F2_BIMODAL}",
        f"バージョン: OpenCV {cv2.__version__} / NumPy {np.__version__}",
        "",
    ]

    cond1_ok = None
    cond2_ok = True
    for name in CAMERAS:
        cam = select_camera(cams_cfg, name)
        rvec_base = np.asarray(cam["rvec"], dtype=np.float64).flatten()
        tvec_base = np.asarray(cam["tvec"], dtype=np.float64).flatten()
        pooled = []
        for sname in POOL_SERIES:
            pooled.extend(results[name][sname]["samples"])
        if len(pooled) != N_POOL:
            raise RuntimeError(f"{name}: サンプル数 {len(pooled)} != {N_POOL}")
        cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(pooled)
        accepted = f_c >= F_C_MIN
        bimodal = f2 >= F2_BIMODAL
        d_b, a_b = pose_diff(rvec_f, tvec_f, rvec_base, tvec_base)
        th_p, th_a = P95_THRESHOLDS[name]
        # M3-1 の各チェーン中心との差
        chain_diffs = []
        for sname in POOL_SERIES:
            s = results[name][sname]
            d_c, a_c = pose_diff(rvec_f, tvec_f, s["rvec"], s["tvec"])
            chain_diffs.append((sname, d_c, a_c))
        lines.append(
            f"[{name}] pooled f_c={f_c:.3f}({len(cluster)}/{N_POOL}) 第2クラスタ={f2:.3f}"
            f"{'【二峰】' if bimodal else ''} RMS半径={rms_pos*1000:.2f}mm/{rms_ang:.4f}deg → "
            f"{'受理' if accepted else '不受理(合意なし)'}")
        lines.append(
            f"  最終解と基準ポーズの差: {d_b*100:.3f}cm/{a_b:.4f}deg (p95閾値 {th_p*100:.2f}cm/{th_a:.2f}deg)")
        lines.append("  M3-1チェーン中心との差: " + " | ".join(
            f"{s}: {d*100:.3f}cm/{a:.4f}deg" for s, d, a in chain_diffs))
        print("\n".join(lines[-3:]))

        if name == TARGET_FAIL_CAM:
            # 条件1: 554 が不受理かつ二峰として分類される
            cond1_ok = (not accepted) and bimodal
        else:
            # 条件2: 受理・第2クラスタ<0.2・p95以内・M3-1チェーン中心と r 以内
            ok = (accepted and not bimodal
                  and d_b <= th_p and a_b <= th_a
                  and all(d <= R_POS and a <= R_ANG for _, d, a in chain_diffs))
            if not ok:
                cond2_ok = False
        lines.append("")

    # 条件3: 負対照(M3-1 でサンプル自体が存在しない=明示的失敗のまま)
    cond3_ok = not neg["accepted"] and "samples" not in neg
    lines.append(f"負対照: M3-1 で明示的失敗(受理={neg['accepted']}, "
                 f"サンプル無し={'samples' not in neg}) → 本方式でも不受理: {'○' if cond3_ok else '×'}")

    go = bool(cond1_ok) and cond2_ok and cond3_ok
    lines += [
        "",
        "=== M3-2 判定(criteria_m3_2.md §3) ===",
        f"1. 554 の検出(不受理かつ二峰): {'○' if cond1_ok else '×'}",
        f"2. 他4カメラの受理(f_c≥0.7・第2クラスタ<0.2・p95以内・M3-1整合): {'○' if cond2_ok else '×'}",
        f"3. 負対照の不受理: {'○' if cond3_ok else '×'}",
        f"総合: {'Go' if go else 'No-Go'}",
    ]
    print("\n".join(lines[-6:]))
    (OUT_DIR / "m3_2_result.txt").write_text("\n".join(lines) + "\n")
    print(f"保存: {OUT_DIR / 'm3_2_result.txt'}")


if __name__ == "__main__":
    main()

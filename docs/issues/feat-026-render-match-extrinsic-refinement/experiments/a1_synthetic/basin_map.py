"""案1 収束域マップ: 摂動グリッドで pipeline_loop の反復を回し、収束/発散を実測する

グリッド: 位置 {5, 10, 20, 30, 50 cm} × 角度 {2, 5, 10, 20, 30 deg}（各1シード）。
成功の定義: 収束フラグ成立 かつ 最終ポーズ誤差 < 5mm / 0.05°（真値既知）。
「収束フラグ成立だが最終誤差が大きい」= silent failure として別カウントする。

実行方法（プロジェクトルートで）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
    docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/basin_map.py

出力: basin_map_result.txt / basin_map_history.npz
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from pipeline_loop import (  # noqa: E402
    run_pipeline, PLY_PATH, TOML_PATH, TRUE_CAM,
)
from stage3_perturb_match import perturb_pose  # noqa: E402
from stage4_6_pnp_iteration import pose_error  # noqa: E402

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "phase4"))
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent

POS_LIST_M = [0.05, 0.10, 0.20, 0.30, 0.50]
ANG_LIST_DEG = [2, 5, 10, 20, 30]
SEED = 1
K_MAX = 5
SUCCESS_POS_M = 0.005    # 成功判定: 最終位置誤差 < 5mm
SUCCESS_ANG_DEG = 0.05   # 成功判定: 最終角度誤差 < 0.05°


def main():
    cams = load_cameras_toml(TOML_PATH)
    cam = select_camera(cams, TRUE_CAM)
    rvec_true = np.asarray(cam["rvec"], dtype=np.float64).flatten()
    tvec_true = np.asarray(cam["tvec"], dtype=np.float64).flatten()

    img_q_gray = cv2.imread(str(OUT_DIR / "query_cam05520126.png"), cv2.IMREAD_GRAYSCALE)
    if img_q_gray is None:
        raise FileNotFoundError("クエリ画像がありません（Stage 2 未実行?）")

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)

    results = []
    lines = [f"収束域マップ（seed={SEED}, k_max={K_MAX}, "
             f"成功判定: <{SUCCESS_POS_M*1000:.0f}mm & <{SUCCESS_ANG_DEG}deg）", ""]
    n_silent = 0
    for pos in POS_LIST_M:
        for ang in ANG_LIST_DEG:
            rng = np.random.default_rng(SEED)
            rvec0, tvec0 = perturb_pose(rvec_true, tvec_true, pos, ang, rng)
            converged, (rvec_f, tvec_f), history, _ = run_pipeline(
                gaussians, cam, img_q_gray, rvec0, tvec0,
                rvec_true, tvec_true, k_start=0, k_max=K_MAX, verbose=False)
            dpos, dang = pose_error(rvec_f, tvec_f, rvec_true, tvec_true)
            accurate = (dpos < SUCCESS_POS_M) and (dang < SUCCESS_ANG_DEG)
            if converged and accurate:
                status = "成功"
            elif converged and not accurate:
                status = "SILENT_FAILURE"   # 収束フラグは立ったが真値から遠い
                n_silent += 1
            else:
                fail_reason = next((h.get("fail") for h in history if "fail" in h),
                                   "収束せず(k_max到達)")
                status = f"失敗({fail_reason})"
            iters = len(history)
            line = (f"pos={pos*100:3.0f}cm ang={ang:2.0f}deg: {status:<20} "
                    f"反復{iters} 最終誤差 {dpos*1000:8.2f}mm / {dang:7.4f}deg")
            print(line)
            lines.append(line)
            results.append({"pos": pos, "ang": ang, "status": status,
                            "converged": bool(converged), "accurate": bool(accurate),
                            "final_pos_m": dpos, "final_ang_deg": dang,
                            "iters": iters, "history": history})

    n_ok = sum(1 for r in results if r["status"] == "成功")
    lines += ["", f"成功 {n_ok}/{len(results)}, silent failure {n_silent} 件"]
    print(lines[-1])
    (OUT_DIR / "basin_map_result.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT_DIR / "basin_map_history.npz",
             results=np.array(results, dtype=object))


if __name__ == "__main__":
    main()

"""実験 a1 フェーズ3: 旧経路と新経路の c2w を照合する（criteria.md §3・§4）。

前提チェック P1/P2 を確認したうえで、比較ウィンドウ（Blender フレーム 41〜340 ↔
NPZ インデックス 40〜339）の300フレームについて位置差・回転角度差を計算し、
criteria.md §4 の閾値（位置 max < 1mm、回転 max < 0.1°）で合否判定する。

実行（リポジトリルート）:
    uv run --project phase4 python docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/compare_poses.py
"""
import json
import sys
from pathlib import Path

import numpy as np

EXP_DIR = Path(__file__).resolve().parent
FRAME_START = 41
FRAME_END = 340
POS_TOL_M = 0.001          # 位置差の閾値 [m]（criteria §4。変更禁止）
ROT_TOL_DEG = 0.1          # 回転角度差の閾値 [deg]（criteria §4。変更禁止）


def main() -> int:
    old = json.load(open(EXP_DIR / "old_poses.json"))
    new = json.load(open(EXP_DIR / "new_poses.json"))

    # 前提チェック P1
    frames_old = [f["frame"] for f in old]
    if frames_old != list(range(FRAME_START, FRAME_END + 1)):
        print(f"P1 不成立: 旧JSONのフレームが {FRAME_START}〜{FRAME_END} の連番ではありません "
              f"(n={len(old)}, first={frames_old[:1]}, last={frames_old[-1:]})")
        return 1
    print(f"P1 成立: 旧JSON {len(old)} フレーム（{FRAME_START}〜{FRAME_END} 連番）")

    # 前提チェック P2
    if new["num_frames"] != 194279:
        print(f"P2 不成立: 新JSON num_frames={new['num_frames']}（期待 194279）")
        return 1
    window = new["frames"][FRAME_START - 1:FRAME_END]
    invalid = [fr["frame_id"] for fr in window if not fr["valid"]]
    if invalid:
        print(f"P2 不成立: 比較ウィンドウに invalid フレームがあります: {invalid[:10]}")
        return 1
    print(f"P2 成立: 新JSON num_frames=194279、ウィンドウ {len(window)} フレーム全件 valid")

    # 照合（criteria §4 の指標）
    d_pos = np.empty(len(old))
    d_rot = np.empty(len(old))
    for i, (fo, fn) in enumerate(zip(old, window)):
        assert fo["frame"] - 1 == fn["frame_id"], "フレーム対応の不整合"
        m_old = np.array(fo["c2w"], dtype=np.float64)
        m_new = np.array(fn["c2w"], dtype=np.float64)
        d_pos[i] = np.linalg.norm(m_old[:3, 3] - m_new[:3, 3])
        r_rel_trace = np.trace(m_old[:3, :3].T @ m_new[:3, :3])
        cos_theta = np.clip((r_rel_trace - 1.0) / 2.0, -1.0, 1.0)
        d_rot[i] = np.degrees(np.arccos(cos_theta))

    i_pos = int(np.argmax(d_pos))
    i_rot = int(np.argmax(d_rot))
    print(f"位置差    : max = {d_pos.max():.3e} m (frame {frames_old[i_pos]}), "
          f"mean = {d_pos.mean():.3e} m")
    print(f"回転角度差: max = {d_rot.max():.3e} deg (frame {frames_old[i_rot]}), "
          f"mean = {d_rot.mean():.3e} deg")

    ok_pos = d_pos.max() < POS_TOL_M
    ok_rot = d_rot.max() < ROT_TOL_DEG
    print(f"判定: 位置 {'合格' if ok_pos else '不合格'} (max {d_pos.max():.3e} < {POS_TOL_M}), "
          f"回転 {'合格' if ok_rot else '不合格'} (max {d_rot.max():.3e} < {ROT_TOL_DEG})")
    print("総合判定:", "合格" if (ok_pos and ok_rot) else "不合格")
    return 0 if (ok_pos and ok_rot) else 1


if __name__ == "__main__":
    sys.exit(main())

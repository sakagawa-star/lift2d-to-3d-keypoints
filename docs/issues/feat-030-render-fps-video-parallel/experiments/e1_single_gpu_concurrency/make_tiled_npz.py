"""実験 e1 用: 300フレームNPZを時間方向に10回タイルして3000フレームの実験用NPZを生成する。

criteria.md §4.1 で lock された生成手順の実体。リポジトリルートで実行する:

    uv run --project phase4 python \
        docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/make_tiled_npz.py

入力・出力・タイル回数は固定（引数なし）。生成後に検証値（shape・frame_ids範囲・NaN数）を表示する。
"""
import os

import numpy as np

TILE = 10
SRC = "phase4/data/session001_f145749_world300.npz"
DST = ("docs/issues/feat-030-render-fps-video-parallel/experiments/"
       "e1_single_gpu_concurrency/tmp/tiled_3000.npz")


def main() -> int:
    data = np.load(SRC, allow_pickle=False)
    x3d = data["x3d_world"]
    frame_ids = data["frame_ids"]
    joint_names = data["joint_names"]

    # 入力の前提検証（criteria.md §6 Phase 0）
    assert x3d.shape == (300, 22, 3), f"入力 shape 不一致: {x3d.shape}"
    assert int(frame_ids[0]) == 145599 and int(frame_ids[-1]) == 145898, \
        f"入力 frame_ids 範囲不一致: {int(frame_ids[0])}..{int(frame_ids[-1])}"

    x3d_tiled = np.tile(x3d, (TILE, 1, 1))
    frame_ids_tiled = np.arange(
        int(frame_ids[0]), int(frame_ids[0]) + 300 * TILE, dtype=frame_ids.dtype
    )

    os.makedirs(os.path.dirname(DST), exist_ok=True)
    np.savez(DST, x3d_world=x3d_tiled, frame_ids=frame_ids_tiled,
             joint_names=joint_names)

    nan_count = int(np.isnan(x3d_tiled).sum())
    print(f"生成: {DST}")
    print(f"shape={x3d_tiled.shape}, "
          f"frame_ids={int(frame_ids_tiled[0])}..{int(frame_ids_tiled[-1])}, "
          f"NaN数={nan_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

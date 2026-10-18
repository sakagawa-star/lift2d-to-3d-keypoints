"""
bug-005: Pose2Sim 出力C3Dの欠損（座標 (0,0,0)・residual 0）を無効点として扱うテスト

実行:
  uv run pytest tests/test_bug005_zero_coord_missing.py -v
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (  # noqa: E402
    HALPE26_NAMES,
    HEAD_JOINT_NAMES,
    KEYPOINT_NAMES,
    collect_head_points,
    compute_fps_poses,
    extract_keypoints,
)


def _base_data():
    """全点が非ゼロ座標・residual 0 の (26,3) データを返す。"""
    n = len(HALPE26_NAMES)
    rng = np.random.default_rng(0)
    data = rng.uniform(100.0, 1000.0, size=(n, 3))
    residual = np.zeros(n, dtype=np.float64)
    return list(HALPE26_NAMES), data, residual


def test_t01_zero_coord_is_invalid():
    labels, data, residual = _base_data()
    data[3] = (0.0, 0.0, 0.0)
    _, valid = extract_keypoints(labels, data, residual)
    k = KEYPOINT_NAMES.index(labels[3])
    assert not valid[k]
    others = np.arange(len(KEYPOINT_NAMES)) != k
    assert valid[others][: len(HALPE26_NAMES) - 1].all()


def test_t02_negative_zero_is_invalid():
    labels, data, residual = _base_data()
    data[3] = (-0.0, 0.0, -0.0)
    _, valid = extract_keypoints(labels, data, residual)
    assert not valid[KEYPOINT_NAMES.index(labels[3])]


def test_t03_partial_zero_is_valid():
    labels, data, residual = _base_data()
    data[3] = (0.0, 0.0, 5.0)
    data[4] = (0.0, 3.0, 0.0)
    _, valid = extract_keypoints(labels, data, residual)
    assert valid[KEYPOINT_NAMES.index(labels[3])]
    assert valid[KEYPOINT_NAMES.index(labels[4])]


def test_t04_residual_and_nan_still_invalid():
    labels, data, residual = _base_data()
    residual[3] = -1.0
    data[4] = (np.nan, 1.0, 2.0)
    _, valid = extract_keypoints(labels, data, residual)
    assert not valid[KEYPOINT_NAMES.index(labels[3])]
    assert not valid[KEYPOINT_NAMES.index(labels[4])]


def test_t05_all_zero_frame_all_invalid():
    labels, data, residual = _base_data()
    data[:] = 0.0
    _, valid = extract_keypoints(labels, data, residual)
    assert not valid.any()


def test_t06_head_point_zero_invalidates_fps_pose():
    labels, data, residual = _base_data()
    data0 = data.copy()
    data1 = data.copy()
    nose_c3d_idx = labels.index("Nose")
    data1[nose_c3d_idx] = (0.0, 0.0, 0.0)
    frames_data = [
        {"frame_no": 1, "data": data0, "residual": residual},
        {"frame_no": 2, "data": data1, "residual": residual},
    ]
    head_arr = collect_head_points(labels, frames_data)
    nose_head_idx = HEAD_JOINT_NAMES.index("Nose")
    assert not np.isnan(head_arr[0]).any()
    assert np.isnan(head_arr[1, nose_head_idx]).all()

    head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}
    _, valid, _ = compute_fps_poses(head_arr, head_idx)
    assert valid[0] == True  # noqa: E712
    assert valid[1] == False  # noqa: E712

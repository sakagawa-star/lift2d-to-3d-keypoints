"""visualize_points_2d.py のテスト"""

import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from visualize_points_2d import (
    resolve_target_camera,
    resolve_image_path,
    draw_points_on_image,
)


# ========================================
# resolve_target_camera
# ========================================

def test_resolve_target_camera_with_arg():
    config = {'target_camera': 'cam01'}
    assert resolve_target_camera(config, 'cam99') == 'cam99'


def test_resolve_target_camera_single_from_config():
    config = {'target_camera': 'cam01'}
    assert resolve_target_camera(config, None) == 'cam01'


def test_resolve_target_camera_multi_without_arg_exits():
    config = {'target_camera': 'cam01,cam02'}
    with pytest.raises(SystemExit):
        resolve_target_camera(config, None)


def test_resolve_target_camera_multi_with_arg():
    config = {'target_camera': 'cam01,cam02'}
    assert resolve_target_camera(config, 'cam01') == 'cam01'


def test_resolve_target_camera_missing_exits():
    with pytest.raises(SystemExit):
        resolve_target_camera({}, None)


# ========================================
# resolve_image_path
# ========================================

def test_resolve_image_path_absolute(tmp_path):
    img = tmp_path / "img.jpg"
    img.write_bytes(b"x")
    config = {'image_cam01': str(img)}
    other_dir = tmp_path / "other"
    other_dir.mkdir()
    # 絶対パスは config_dir に依存せずそのまま採用される
    assert resolve_image_path(config, 'cam01', other_dir) == Path(str(img))


def test_resolve_image_path_relative(tmp_path):
    img = tmp_path / "img.jpg"
    img.write_bytes(b"x")
    config = {'image_cam01': 'img.jpg'}
    # 相対パスは config_dir 基準で解決される
    assert resolve_image_path(config, 'cam01', tmp_path) == tmp_path / 'img.jpg'


def test_resolve_image_path_missing_key_exits(tmp_path):
    with pytest.raises(SystemExit):
        resolve_image_path({}, 'cam01', tmp_path)


def test_resolve_image_path_missing_file_exits(tmp_path):
    config = {'image_cam01': 'nope.jpg'}
    with pytest.raises(SystemExit):
        resolve_image_path(config, 'cam01', tmp_path)


# ========================================
# draw_points_on_image
# ========================================

def _make_image(path: Path, w: int = 100, h: int = 80):
    img = np.full((h, w, 3), 255, dtype=np.uint8)
    cv2.imwrite(str(path), img)


def test_draw_points_on_image_writes_file(tmp_path):
    img_path = tmp_path / "in.png"
    _make_image(img_path, 100, 80)
    out_path = tmp_path / "out.png"
    points = {'p1': [10.0, 10.0], 'p2': [50.0, 40.0]}
    config = {'image_width': '100', 'image_height': '80'}

    drawn, skipped = draw_points_on_image(img_path, points, out_path, config)

    assert drawn == 2
    assert skipped == 0
    assert out_path.exists()
    out = cv2.imread(str(out_path))
    assert out.shape == (80, 100, 3)


def test_draw_points_on_image_counts_out_of_range(tmp_path):
    img_path = tmp_path / "in.png"
    _make_image(img_path, 100, 80)
    out_path = tmp_path / "out.png"
    points = {
        'inside': [10.0, 10.0],
        'neg_x': [-1.0, 10.0],
        'over_y': [10.0, 80.0],
    }
    config = {'image_width': '100', 'image_height': '80'}

    drawn, skipped = draw_points_on_image(img_path, points, out_path, config)

    assert drawn == 1
    assert skipped == 2


def test_draw_points_on_image_resolution_mismatch_warns(tmp_path, capsys):
    img_path = tmp_path / "in.png"
    _make_image(img_path, 100, 80)
    out_path = tmp_path / "out.png"
    points = {'p1': [10.0, 10.0]}
    config = {'image_width': '999', 'image_height': '999'}

    drawn, skipped = draw_points_on_image(img_path, points, out_path, config)

    captured = capsys.readouterr()
    assert "警告" in captured.err
    assert drawn == 1
    assert skipped == 0

"""
feat-011: visualize_points_2d.py 基準点番号ラベル表示オプションのテスト
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from visualize_points_2d import (
    draw_points_on_image,
    extract_point_number,
)


# ========================================
# extract_point_number のユニットテスト
# ========================================

class TestExtractPointNumber:
    def test_leading_zero_preserved(self):
        """先頭ゼロが保持されること"""
        assert extract_point_number("基準_01") == "01"

    def test_two_digit(self):
        assert extract_point_number("基準_12") == "12"

    def test_last_number_group(self):
        """最後の連続数字列が採用されること"""
        assert extract_point_number("pt3_cam01_05") == "05"

    def test_no_digits_returns_full_name(self):
        """数字なしの場合は ObjectName 全体を返すこと"""
        assert extract_point_number("marker") == "marker"


# ========================================
# draw_points_on_image の動作テスト
# ========================================

IMG_W, IMG_H = 200, 150


@pytest.fixture
def test_image(tmp_path):
    """テスト用PNG画像（可逆形式、ピクセル一致比較のため）"""
    img = np.full((IMG_H, IMG_W, 3), 128, dtype=np.uint8)
    path = tmp_path / "test.png"
    cv2.imwrite(str(path), img)
    return path


CONFIG = {"image_width": str(IMG_W), "image_height": str(IMG_H)}


def _reference_image(points: dict) -> np.ndarray:
    """従来実装（circle 2行のみ）の参照描画を再現する"""
    img = np.full((IMG_H, IMG_W, 3), 128, dtype=np.uint8)
    for name, (x, y) in points.items():
        ix, iy = int(round(x)), int(round(y))
        if ix < 0 or ix >= IMG_W or iy < 0 or iy >= IMG_H:
            continue
        cv2.circle(img, (ix, iy), radius=8, color=(0, 0, 255), thickness=2)
        cv2.circle(img, (ix, iy), radius=1, color=(0, 255, 255), thickness=-1)
    return img


class TestDrawPointsOnImage:
    def test_no_label_matches_reference(self, test_image, tmp_path):
        """show_label=False の出力が従来実装とピクセル一致すること"""
        points = {"基準_01": [50.0, 60.0], "基準_02": [120.0, 90.0]}
        out = tmp_path / "out_nolabel.png"
        drawn, skipped = draw_points_on_image(
            test_image, points, out, CONFIG, show_label=False
        )
        assert drawn == 2 and skipped == 0
        result = cv2.imread(str(out))
        assert np.array_equal(result, _reference_image(points))

    def test_default_is_no_label(self, test_image, tmp_path):
        """show_label 省略時のデフォルトがラベルなし（後方互換）であること"""
        points = {"基準_01": [50.0, 60.0]}
        out = tmp_path / "out_default.png"
        draw_points_on_image(test_image, points, out, CONFIG)
        result = cv2.imread(str(out))
        assert np.array_equal(result, _reference_image(points))

    def test_label_changes_output(self, test_image, tmp_path):
        """show_label=True の出力がラベルなしと異なる（ラベルが描画される）こと"""
        points = {"基準_01": [50.0, 60.0]}
        out = tmp_path / "out_label.png"
        drawn, skipped = draw_points_on_image(
            test_image, points, out, CONFIG, show_label=True
        )
        assert drawn == 1 and skipped == 0
        result = cv2.imread(str(out))
        assert not np.array_equal(result, _reference_image(points))

    def test_labels_at_four_corners(self, test_image, tmp_path):
        """画像4隅付近の点でも例外なく出力されること"""
        points = {
            "基準_01": [2.0, 2.0],                      # 左上
            "基準_02": [IMG_W - 3.0, 2.0],              # 右上
            "基準_03": [2.0, IMG_H - 3.0],              # 左下
            "基準_04": [IMG_W - 3.0, IMG_H - 3.0],      # 右下
        }
        out = tmp_path / "out_corners.png"
        drawn, skipped = draw_points_on_image(
            test_image, points, out, CONFIG, show_label=True
        )
        assert drawn == 4 and skipped == 0
        assert out.exists()
        assert cv2.imread(str(out)) is not None

    def test_out_of_range_point_skipped(self, test_image, tmp_path):
        """範囲外の点はラベル有効時もスキップされること"""
        points = {"基準_01": [50.0, 60.0], "基準_99": [9999.0, 9999.0]}
        out = tmp_path / "out_skip.png"
        drawn, skipped = draw_points_on_image(
            test_image, points, out, CONFIG, show_label=True
        )
        assert drawn == 1 and skipped == 1

    def test_no_digit_name_does_not_raise(self, test_image, tmp_path):
        """数字を含まない ObjectName でもエラーにならないこと"""
        points = {"marker": [50.0, 60.0]}
        out = tmp_path / "out_nodigit.png"
        drawn, skipped = draw_points_on_image(
            test_image, points, out, CONFIG, show_label=True
        )
        assert drawn == 1 and skipped == 0

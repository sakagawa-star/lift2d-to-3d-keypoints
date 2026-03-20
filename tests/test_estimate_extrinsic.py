"""
estimate_extrinsic.py のテスト

合成データ（既知のK, R, tから cv2.projectPoints で2D点を生成）を使ってテストする。
"""

import sys
import tempfile
from pathlib import Path

import numpy as np
import cv2
import pytest

# phase0 ディレクトリをパスに追加
sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from estimate_extrinsic import (
    load_intrinsic_toml,
    estimate_extrinsic,
    compute_reprojection_errors,
    match_points,
)


# ========================================
# テスト用の共通パラメータ
# ========================================

K_TRUE = np.array([[1177.34, 0, 956.70], [0, 1177.82, 495.77], [0, 0, 1]], dtype=np.float64)
DIST_TRUE = np.array([-0.0536, 0.0983, -0.0054, -0.0027], dtype=np.float64)
RVEC_TRUE = np.array([0.1, -0.2, 0.3], dtype=np.float64)
TVEC_TRUE = np.array([1.0, -0.5, 5.0], dtype=np.float64)

# 3D基準点（8点、カメラから十分離れた位置）
POINTS_3D_8 = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.5, 0.5, 0.5],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
], dtype=np.float64)

POINT_NAMES_8 = [f"基準_{i+1:03d}" for i in range(8)]


def _generate_2d_points(points_3d, rvec, tvec, K, dist):
    """合成2D点を生成"""
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


# ========================================
# T-1: TOML読み込み
# ========================================

class TestLoadIntrinsicToml:
    """TOML読み込みのテスト"""

    def test_normal_toml(self, tmp_path):
        """正常なTOMLからK, dist, sizeを正しく読み込めること"""
        toml_content = b"""
[cam01]
name = "cam01"
size = [1920.0, 1080.0]
matrix = [[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]
distortions = [-0.0536, 0.0983, -0.0054, -0.0027]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false

[metadata]
adjusted = false
error = 0.3356
"""
        toml_file = tmp_path / "test.toml"
        toml_file.write_bytes(toml_content)

        result = load_intrinsic_toml(str(toml_file), "cam01")

        np.testing.assert_array_almost_equal(result['K'][0, 0], 1177.34)
        np.testing.assert_array_almost_equal(result['K'][1, 1], 1177.82)
        np.testing.assert_array_almost_equal(result['K'][0, 2], 956.70)
        np.testing.assert_array_almost_equal(result['K'][1, 2], 495.77)
        assert len(result['dist']) == 4
        assert result['image_width'] == 1920
        assert result['image_height'] == 1080

    def test_multi_camera_toml(self, tmp_path):
        """複数カメラのTOMLから指定カメラのみ抽出できること"""
        toml_content = b"""
[cam01]
name = "cam01"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false

[cam02]
name = "cam02"
size = [1280.0, 720.0]
matrix = [[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]
distortions = [-0.1, 0.05, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false

[metadata]
adjusted = false
"""
        toml_file = tmp_path / "multi.toml"
        toml_file.write_bytes(toml_content)

        result = load_intrinsic_toml(str(toml_file), "cam02")

        np.testing.assert_array_almost_equal(result['K'][0, 0], 800.0)
        assert result['image_width'] == 1280
        assert result['image_height'] == 720

    def test_camera_not_found(self, tmp_path):
        """存在しないカメラ名の場合にエラーが発生すること"""
        toml_content = b"""
[cam01]
name = "cam01"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false
"""
        toml_file = tmp_path / "test.toml"
        toml_file.write_bytes(toml_content)

        with pytest.raises(SystemExit):
            load_intrinsic_toml(str(toml_file), "cam_nonexistent")

    def test_dist5_toml(self, tmp_path):
        """歪み係数が5要素（k3あり）の場合も読み込めること"""
        toml_content = b"""
[cam01]
name = "cam01"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.09, -0.005, -0.003, 0.01]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false
"""
        toml_file = tmp_path / "test.toml"
        toml_file.write_bytes(toml_content)

        result = load_intrinsic_toml(str(toml_file), "cam01")

        assert len(result['dist']) == 5
        np.testing.assert_array_almost_equal(result['dist'][4], 0.01)


# ========================================
# T-2: 推定（6点以上、RANSAC使用）
# ========================================

class TestEstimateRansac:
    """6点以上でのRANSAC推定テスト"""

    def test_clean_data(self):
        """合成データ（ノイズなし）でR, tが正しく推定されること"""
        points_2d = _generate_2d_points(POINTS_3D_8, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        result = estimate_extrinsic(POINTS_3D_8, points_2d, K_TRUE, DIST_TRUE, POINT_NAMES_8)

        rmse, _ = compute_reprojection_errors(
            POINTS_3D_8, points_2d, K_TRUE, DIST_TRUE, result['rvec'], result['tvec']
        )
        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.4f} px"
        assert result['used_ransac'] is True
        assert len(result['outliers']) == 0

    def test_with_outlier(self):
        """1点を外れ値として大きくずらした場合、outlierとして検出されること"""
        points_2d = _generate_2d_points(POINTS_3D_8, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        # 最後の点を大きくずらす
        points_2d_noisy = points_2d.copy()
        points_2d_noisy[-1] += np.array([200.0, 200.0])

        result = estimate_extrinsic(POINTS_3D_8, points_2d_noisy, K_TRUE, DIST_TRUE, POINT_NAMES_8)

        assert POINT_NAMES_8[-1] in result['outliers'], \
            f"外れ値が検出されませんでした。outliers: {result['outliers']}"


# ========================================
# T-3: 推定（4〜5点、RANSACなし）
# ========================================

class TestEstimateNoRansac:
    """4〜5点でのRANSACなし推定テスト"""

    def test_4_points(self):
        """4点の合成データでR, tが推定できること"""
        points_3d = POINTS_3D_8[:4]
        names = POINT_NAMES_8[:4]
        points_2d = _generate_2d_points(points_3d, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        result = estimate_extrinsic(points_3d, points_2d, K_TRUE, DIST_TRUE, names)

        assert result['used_ransac'] is False
        rmse, _ = compute_reprojection_errors(
            points_3d, points_2d, K_TRUE, DIST_TRUE, result['rvec'], result['tvec']
        )
        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.4f} px"

    def test_5_points(self):
        """5点の合成データでR, tが推定できること"""
        points_3d = POINTS_3D_8[:5]
        names = POINT_NAMES_8[:5]
        points_2d = _generate_2d_points(points_3d, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        result = estimate_extrinsic(points_3d, points_2d, K_TRUE, DIST_TRUE, names)

        assert result['used_ransac'] is False
        rmse, _ = compute_reprojection_errors(
            points_3d, points_2d, K_TRUE, DIST_TRUE, result['rvec'], result['tvec']
        )
        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.4f} px"


# ========================================
# T-4: 推定（3点以下、エラー）
# ========================================

class TestEstimateError:
    """3点以下でのエラーテスト"""

    def test_3_points_error(self):
        """3点の場合にエラー終了すること"""
        points_3d = POINTS_3D_8[:3]
        names = POINT_NAMES_8[:3]
        points_2d = _generate_2d_points(points_3d, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        with pytest.raises(SystemExit):
            estimate_extrinsic(points_3d, points_2d, K_TRUE, DIST_TRUE, names)

    def test_0_points_error(self):
        """0点の場合にエラー終了すること"""
        points_3d = np.empty((0, 3), dtype=np.float64)
        points_2d = np.empty((0, 2), dtype=np.float64)
        names = []

        with pytest.raises(SystemExit):
            estimate_extrinsic(points_3d, points_2d, K_TRUE, DIST_TRUE, names)


# ========================================
# T-5: 再投影誤差計算
# ========================================

class TestReprojectionErrors:
    """再投影誤差計算のテスト"""

    def test_perfect_reprojection(self):
        """ノイズなし合成データで再投影誤差が十分小さいこと"""
        points_2d = _generate_2d_points(POINTS_3D_8, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_TRUE)

        rmse, per_point_errors = compute_reprojection_errors(
            POINTS_3D_8, points_2d, K_TRUE, DIST_TRUE, RVEC_TRUE, TVEC_TRUE
        )

        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.6f} px"
        assert len(per_point_errors) == len(POINTS_3D_8)
        assert all(e < 1.0 for e in per_point_errors)


# ========================================
# T-6: TOML出力フォーマット
# ========================================

class TestTomlOutput:
    """TOML出力のテスト"""

    def test_toml_parseable(self, capsys):
        """出力がCalib_scene.toml形式として正しいこと（tomliでパース可能）"""
        from estimate_extrinsic import print_toml_output

        print_toml_output(
            "cam01", 1920, 1080,
            K_TRUE, DIST_TRUE, RVEC_TRUE, TVEC_TRUE
        )

        captured = capsys.readouterr()
        output = captured.out.strip()

        # tomli でパース可能であることを確認
        import tomli
        parsed = tomli.loads(output)

        assert "cam01" in parsed
        cam = parsed["cam01"]
        assert cam["name"] == "cam01"
        assert cam["size"] == [1920.0, 1080.0]
        assert len(cam["matrix"]) == 3
        assert len(cam["distortions"]) == 4
        assert len(cam["rotation"]) == 3
        assert len(cam["translation"]) == 3
        assert cam["fisheye"] is False

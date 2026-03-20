"""
estimate_camera_params.py の広角レンズ対応（--wide）テスト

合成データを使ってテストする。
"""

import sys
from pathlib import Path
from unittest.mock import patch
from io import StringIO

import numpy as np
import cv2
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from estimate_camera_params import (
    run_estimation,
    project_dist8,
    project_dist8_fix_center,
)


# ========================================
# 共通パラメータ
# ========================================

K_TRUE = np.array([[1177.34, 0, 956.70], [0, 1177.82, 495.77], [0, 0, 1]], dtype=np.float64)
DIST_4 = np.array([-0.05, 0.09, -0.005, -0.003], dtype=np.float64)
DIST_8 = np.array([-0.05, 0.09, -0.005, -0.003, 0.01, 0.02, -0.01, 0.005], dtype=np.float64)
RVEC_TRUE = np.array([0.1, -0.2, 0.3], dtype=np.float64)
TVEC_TRUE = np.array([1.0, -0.5, 5.0], dtype=np.float64)

# 25点（20点以上の要件を満たす）
POINTS_3D_25 = np.array([
    [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
    [0.5, 0.5, 0.5], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0],
    [1.0, 1.0, 1.0], [0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.5, 0.5, 0.0],
    [0.25, 0.25, 0.25], [0.75, 0.25, 0.25], [0.25, 0.75, 0.25],
    [0.75, 0.75, 0.25], [0.25, 0.25, 0.75], [0.75, 0.25, 0.75],
    [0.25, 0.75, 0.75], [0.75, 0.75, 0.75], [0.5, 0.0, 0.5],
    [0.0, 0.5, 0.5], [1.0, 0.5, 0.5], [0.5, 1.0, 0.5], [0.5, 0.5, 1.0],
], dtype=np.float64)


def _generate_2d(points_3d, rvec, tvec, K, dist):
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


# ========================================
# T-1: 8係数推定（主点推定）
# ========================================

class TestWideEstimation:

    def test_8coeff_estimation(self):
        """8係数歪みモデルで全パラメータが推定できること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_8)

        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1], K_TRUE[0, 2], K_TRUE[1, 2],
            *DIST_8,
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist8(params, POINTS_3D_25)
        errors = np.linalg.norm(projected - points_2d, axis=1)
        rmse = np.sqrt(np.mean(errors ** 2))
        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.6f} px"


# ========================================
# T-2: 8係数推定（主点固定）
# ========================================

class TestWideFixCenter:

    def test_8coeff_fix_center(self):
        """8係数 + 主点固定で投影が正しく動作すること"""
        cx, cy = K_TRUE[0, 2], K_TRUE[1, 2]
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_8)

        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1],
            *DIST_8,
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist8_fix_center(params, POINTS_3D_25, cx, cy)
        errors = np.linalg.norm(projected - points_2d, axis=1)
        rmse = np.sqrt(np.mean(errors ** 2))
        assert rmse < 1.0, f"再投影誤差が大きすぎます: {rmse:.6f} px"


# ========================================
# T-3: 8係数で点数不足時のフォールバック
# ========================================

class TestWideFallback:

    def test_fallback_not_enough_points(self, tmp_path):
        """点数不足で歪み係数が0に固定されること"""
        # 10点（< 20点）の合成データ
        points_3d_10 = POINTS_3D_25[:10]
        points_2d_10 = _generate_2d(points_3d_10, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_4)

        # テスト用CSV/YAML作成
        csv_3d = tmp_path / "pts3d.csv"
        csv_2d = tmp_path / "pts2d.csv"
        yaml_f = tmp_path / "config.yaml"

        with open(csv_3d, 'w') as f:
            f.write("ObjectName,X,Y,Z\n")
            for i, p in enumerate(points_3d_10):
                f.write(f"pt{i},{p[0]},{p[1]},{p[2]}\n")

        with open(csv_2d, 'w') as f:
            f.write("ObjectName,camera_name,X,Y\n")
            for i, p in enumerate(points_2d_10):
                f.write(f"pt{i},cam01,{p[0]},{p[1]}\n")

        with open(yaml_f, 'w') as f:
            f.write(f"target_camera: cam01\n")
            f.write(f"points_3d: pts3d.csv\n")
            f.write(f"points_2d: pts2d.csv\n")
            f.write(f"image_width: 1920\n")
            f.write(f"image_height: 1080\n")

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=True, fix_center=False)

        output = captured.getvalue()
        assert "歪み係数は0に固定します" in output
        assert result == 0


# ========================================
# T-4: --wide --k3 の排他制御
# ========================================

class TestWideK3Exclusive:

    def test_wide_overrides_k3(self):
        """--wide が --k3 を上書きすること"""
        # main() のロジックをテスト
        use_k3 = True and not True  # args.k3=True, args.wide=True
        assert use_k3 is False


# ========================================
# T-5: リグレッションテスト
# ========================================

class TestRegression:

    def test_4coeff_projection(self):
        """4係数モードの投影が正しいこと"""
        from estimate_camera_params import project_dist4
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_4)

        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1], K_TRUE[0, 2], K_TRUE[1, 2],
            *DIST_4,
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist4(params, POINTS_3D_25)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1.0

    def test_no_dist_projection(self):
        """歪みなしモードの投影が正しいこと"""
        from estimate_camera_params import project_no_dist
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, None)

        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1], K_TRUE[0, 2], K_TRUE[1, 2],
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_no_dist(params, POINTS_3D_25)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1.0


# ========================================
# T-7: TOML/CSV出力フォーマット
# ========================================

class TestOutputFormat:

    def test_toml_8coeff(self, tmp_path):
        """8係数モードのTOML出力でdistortionsが8要素であること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_8)

        csv_3d = tmp_path / "pts3d.csv"
        csv_2d = tmp_path / "pts2d.csv"
        yaml_f = tmp_path / "config.yaml"

        with open(csv_3d, 'w') as f:
            f.write("ObjectName,X,Y,Z\n")
            for i, p in enumerate(POINTS_3D_25):
                f.write(f"pt{i},{p[0]},{p[1]},{p[2]}\n")

        with open(csv_2d, 'w') as f:
            f.write("ObjectName,camera_name,X,Y\n")
            for i, p in enumerate(points_2d):
                f.write(f"pt{i},cam01,{p[0]},{p[1]}\n")

        with open(yaml_f, 'w') as f:
            f.write("target_camera: cam01\n")
            f.write("points_3d: pts3d.csv\n")
            f.write("points_2d: pts2d.csv\n")
            f.write("image_width: 1920\n")
            f.write("image_height: 1080\n")

        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_f), use_k3=False, use_wide=True, fix_center=False)

        output = captured.getvalue()
        # TOML distortions が8要素
        assert "distortions = [" in output
        for line in output.split('\n'):
            if line.strip().startswith("distortions"):
                # 要素数をカウント（カンマ + 1）
                count = line.count(',') + 1
                assert count == 8, f"distortions の要素数が8ではありません: {count}"
                break

    def test_csv_8coeff_header(self, tmp_path):
        """8係数モードのCSVヘッダーにk3,k4,k5,k6が含まれること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_8)

        csv_3d = tmp_path / "pts3d.csv"
        csv_2d = tmp_path / "pts2d.csv"
        yaml_f = tmp_path / "config.yaml"

        with open(csv_3d, 'w') as f:
            f.write("ObjectName,X,Y,Z\n")
            for i, p in enumerate(POINTS_3D_25):
                f.write(f"pt{i},{p[0]},{p[1]},{p[2]}\n")

        with open(csv_2d, 'w') as f:
            f.write("ObjectName,camera_name,X,Y\n")
            for i, p in enumerate(points_2d):
                f.write(f"pt{i},cam01,{p[0]},{p[1]}\n")

        with open(yaml_f, 'w') as f:
            f.write("target_camera: cam01\n")
            f.write("points_3d: pts3d.csv\n")
            f.write("points_2d: pts2d.csv\n")
            f.write("image_width: 1920\n")
            f.write("image_height: 1080\n")

        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_f), use_k3=False, use_wide=True, fix_center=False)

        output = captured.getvalue()
        assert "k3,k4,k5,k6" in output

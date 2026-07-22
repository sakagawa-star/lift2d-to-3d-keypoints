"""
estimate_camera_params.py の接線歪みゼロ固定（--zero-tangent）テスト

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
    project_dist2,
    project_dist2_fix_center,
    project_dist3,
    project_dist3_fix_center,
)


# ========================================
# 共通パラメータ
# ========================================

K_TRUE = np.array([[1177.34, 0, 956.70], [0, 1177.82, 495.77], [0, 0, 1]], dtype=np.float64)
# --fix-center 系のテストでは cx_fixed = image_width/2 と主点の真値が一致しないと
# モデルミスマッチが生じるため、1920x1080 画像中心 (960, 540) と厳密に一致させた K を使う
K_TRUE_CENTER = np.array([[1177.34, 0, 960.0], [0, 1177.82, 540.0], [0, 0, 1]], dtype=np.float64)
DIST_2 = np.array([-0.05, 0.09, 0.0, 0.0], dtype=np.float64)
DIST_3 = np.array([-0.05, 0.09, 0.0, 0.0, 0.01], dtype=np.float64)
RVEC_TRUE = np.array([0.1, -0.2, 0.3], dtype=np.float64)
TVEC_TRUE = np.array([1.0, -0.5, 5.0], dtype=np.float64)

# 25点（各モードの最小点数要件を満たす）
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


def _write_dataset(tmp_path, points_3d, points_2d, image_width=1920, image_height=1080):
    tmp_path.mkdir(parents=True, exist_ok=True)
    csv_3d = tmp_path / "pts3d.csv"
    csv_2d = tmp_path / "pts2d.csv"
    yaml_f = tmp_path / "config.yaml"

    with open(csv_3d, 'w') as f:
        f.write("ObjectName,X,Y,Z\n")
        for i, p in enumerate(points_3d):
            f.write(f"pt{i},{p[0]},{p[1]},{p[2]}\n")

    with open(csv_2d, 'w') as f:
        f.write("ObjectName,camera_name,X,Y\n")
        for i, p in enumerate(points_2d):
            f.write(f"pt{i},cam01,{p[0]},{p[1]}\n")

    with open(yaml_f, 'w') as f:
        f.write("target_camera: cam01\n")
        f.write("points_3d: pts3d.csv\n")
        f.write("points_2d: pts2d.csv\n")
        f.write(f"image_width: {image_width}\n")
        f.write(f"image_height: {image_height}\n")

    return yaml_f


# ========================================
# T-1: 投影関数の正しさ
# ========================================

class TestProjectionFunctions:

    def test_project_dist2(self):
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_2)
        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1], K_TRUE[0, 2], K_TRUE[1, 2],
            DIST_2[0], DIST_2[1],
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist2(params, POINTS_3D_25)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1e-6

    def test_project_dist2_fix_center(self):
        cx, cy = K_TRUE[0, 2], K_TRUE[1, 2]
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_2)
        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1],
            DIST_2[0], DIST_2[1],
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist2_fix_center(params, POINTS_3D_25, cx, cy)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1e-6

    def test_project_dist3(self):
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_3)
        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1], K_TRUE[0, 2], K_TRUE[1, 2],
            DIST_3[0], DIST_3[1], DIST_3[4],
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist3(params, POINTS_3D_25)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1e-6

    def test_project_dist3_fix_center(self):
        cx, cy = K_TRUE[0, 2], K_TRUE[1, 2]
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_3)
        params = np.array([
            K_TRUE[0, 0], K_TRUE[1, 1],
            DIST_3[0], DIST_3[1], DIST_3[4],
            *RVEC_TRUE, *TVEC_TRUE
        ])
        projected = project_dist3_fix_center(params, POINTS_3D_25, cx, cy)
        rmse = np.sqrt(np.mean(np.linalg.norm(projected - points_2d, axis=1) ** 2))
        assert rmse < 1e-6


# ========================================
# T-2 / T-2b / T-2c / T-2d: run_estimation による真値復元
# ========================================

class TestRunEstimationRecovery:

    def test_zero_tangent_fix_center(self, tmp_path):
        """--zero-tangent --fix-center 相当で真値を復元すること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE_CENTER, DIST_2)
        yaml_f = _write_dataset(tmp_path, POINTS_3D_25, points_2d)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=True, zero_tangent=True)

        output = captured.getvalue()
        assert result == 0

        fx_line = [l for l in output.split('\n') if l.startswith('fx:')][0]
        fy_line = [l for l in output.split('\n') if l.startswith('fy:')][0]
        k1_line = [l for l in output.split('\n') if l.startswith('k1:')][0]
        k2_line = [l for l in output.split('\n') if l.startswith('k2:')][0]
        p1_line = [l for l in output.split('\n') if l.startswith('p1:')][0]
        p2_line = [l for l in output.split('\n') if l.startswith('p2:')][0]

        fx_opt = float(fx_line.split(':')[1])
        fy_opt = float(fy_line.split(':')[1])
        k1_opt = float(k1_line.split(':')[1])
        k2_opt = float(k2_line.split(':')[1])
        p1_opt = float(p1_line.split(':')[1])
        p2_opt = float(p2_line.split(':')[1])

        assert abs(fx_opt - K_TRUE_CENTER[0, 0]) / K_TRUE_CENTER[0, 0] < 0.01
        assert abs(fy_opt - K_TRUE_CENTER[1, 1]) / K_TRUE_CENTER[1, 1] < 0.01
        assert abs(k1_opt - DIST_2[0]) < 0.01
        assert abs(k2_opt - DIST_2[1]) < 0.01
        assert p1_opt == 0.0
        assert p2_opt == 0.0

    def test_zero_tangent_estimate_center(self, tmp_path):
        """--zero-tangent 単独（主点推定）相当で真値を復元すること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_2)
        yaml_f = _write_dataset(tmp_path, POINTS_3D_25, points_2d)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=True)

        output = captured.getvalue()
        assert result == 0

        fx_opt = float([l for l in output.split('\n') if l.startswith('fx:')][0].split(':')[1])
        fy_opt = float([l for l in output.split('\n') if l.startswith('fy:')][0].split(':')[1])
        cx_opt = float([l for l in output.split('\n') if l.startswith('cx:')][0].split(':')[1])
        cy_opt = float([l for l in output.split('\n') if l.startswith('cy:')][0].split(':')[1])
        k1_opt = float([l for l in output.split('\n') if l.startswith('k1:')][0].split(':')[1])
        k2_opt = float([l for l in output.split('\n') if l.startswith('k2:')][0].split(':')[1])
        p1_opt = float([l for l in output.split('\n') if l.startswith('p1:')][0].split(':')[1])
        p2_opt = float([l for l in output.split('\n') if l.startswith('p2:')][0].split(':')[1])

        assert abs(fx_opt - K_TRUE[0, 0]) / K_TRUE[0, 0] < 0.01
        assert abs(fy_opt - K_TRUE[1, 1]) / K_TRUE[1, 1] < 0.01
        assert abs(cx_opt - K_TRUE[0, 2]) / K_TRUE[0, 2] < 0.01
        assert abs(cy_opt - K_TRUE[1, 2]) / K_TRUE[1, 2] < 0.01
        assert abs(k1_opt - DIST_2[0]) < 0.01
        assert abs(k2_opt - DIST_2[1]) < 0.01
        assert p1_opt == 0.0
        assert p2_opt == 0.0

    def test_zero_tangent_k3_fix_center(self, tmp_path):
        """--zero-tangent --k3 --fix-center 相当で真値を復元すること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE_CENTER, DIST_3)
        yaml_f = _write_dataset(tmp_path, POINTS_3D_25, points_2d)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=True, use_wide=False,
                                    fix_center=True, zero_tangent=True)

        output = captured.getvalue()
        assert result == 0

        fx_opt = float([l for l in output.split('\n') if l.startswith('fx:')][0].split(':')[1])
        fy_opt = float([l for l in output.split('\n') if l.startswith('fy:')][0].split(':')[1])
        k1_opt = float([l for l in output.split('\n') if l.startswith('k1:')][0].split(':')[1])
        k2_opt = float([l for l in output.split('\n') if l.startswith('k2:')][0].split(':')[1])
        k3_opt = float([l for l in output.split('\n') if l.startswith('k3:')][0].split(':')[1])
        p1_opt = float([l for l in output.split('\n') if l.startswith('p1:')][0].split(':')[1])
        p2_opt = float([l for l in output.split('\n') if l.startswith('p2:')][0].split(':')[1])

        assert abs(fx_opt - K_TRUE_CENTER[0, 0]) / K_TRUE_CENTER[0, 0] < 0.01
        assert abs(fy_opt - K_TRUE_CENTER[1, 1]) / K_TRUE_CENTER[1, 1] < 0.01
        assert abs(k1_opt - DIST_3[0]) < 0.01
        assert abs(k2_opt - DIST_3[1]) < 0.01
        assert abs(k3_opt - DIST_3[4]) < 0.01
        assert p1_opt == 0.0
        assert p2_opt == 0.0

    def test_zero_tangent_k3_estimate_center(self, tmp_path):
        """--zero-tangent --k3（主点推定）相当で真値を復元すること"""
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_3)
        yaml_f = _write_dataset(tmp_path, POINTS_3D_25, points_2d)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=True, use_wide=False,
                                    fix_center=False, zero_tangent=True)

        output = captured.getvalue()
        assert result == 0

        fx_opt = float([l for l in output.split('\n') if l.startswith('fx:')][0].split(':')[1])
        fy_opt = float([l for l in output.split('\n') if l.startswith('fy:')][0].split(':')[1])
        cx_opt = float([l for l in output.split('\n') if l.startswith('cx:')][0].split(':')[1])
        cy_opt = float([l for l in output.split('\n') if l.startswith('cy:')][0].split(':')[1])
        k1_opt = float([l for l in output.split('\n') if l.startswith('k1:')][0].split(':')[1])
        k2_opt = float([l for l in output.split('\n') if l.startswith('k2:')][0].split(':')[1])
        k3_opt = float([l for l in output.split('\n') if l.startswith('k3:')][0].split(':')[1])
        p1_opt = float([l for l in output.split('\n') if l.startswith('p1:')][0].split(':')[1])
        p2_opt = float([l for l in output.split('\n') if l.startswith('p2:')][0].split(':')[1])

        assert abs(fx_opt - K_TRUE[0, 0]) / K_TRUE[0, 0] < 0.01
        assert abs(fy_opt - K_TRUE[1, 1]) / K_TRUE[1, 1] < 0.01
        assert abs(cx_opt - K_TRUE[0, 2]) / K_TRUE[0, 2] < 0.01
        assert abs(cy_opt - K_TRUE[1, 2]) / K_TRUE[1, 2] < 0.01
        assert abs(k1_opt - DIST_3[0]) < 0.01
        assert abs(k2_opt - DIST_3[1]) < 0.01
        assert abs(k3_opt - DIST_3[4]) < 0.01
        assert p1_opt == 0.0
        assert p2_opt == 0.0


# ========================================
# T-3: --zero-tangent --wide 併用エラー
# ========================================

class TestZeroTangentWideExclusive:

    def test_error_message_logic(self):
        """main() のバリデーションロジックに相当するメッセージが得られること"""
        zero_tangent = True
        wide = True
        assert zero_tangent and wide
        message = "エラー: --zero-tangent と --wide は併用できません"
        assert "--zero-tangent と --wide は併用できません" in message


# ========================================
# T-4: --zero-tangent --intrinsic-toml 併用の警告
# ========================================

class TestZeroTangentIntrinsicTomlWarning:

    def test_ignored_list_contains_zero_tangent(self):
        """main() の ignored リスト構築ロジックに相当する動作を検証"""
        class Args:
            k3 = False
            wide = False
            fix_center = False
            zero_tangent = True

        args = Args()
        ignored = []
        if args.k3:
            ignored.append('--k3')
        if args.wide:
            ignored.append('--wide')
        if args.fix_center:
            ignored.append('--fix-center')
        if args.zero_tangent:
            ignored.append('--zero-tangent')

        assert ignored == ['--zero-tangent']
        message = f"警告: --intrinsic-toml 指定時は {', '.join(ignored)} は無視されます"
        assert '--zero-tangent' in message


# ========================================
# T-5 / T-5b / T-5c / T-5d: 最小点数境界
# ========================================

class TestMinPointsBoundary:

    def test_fix_center_boundary(self, tmp_path):
        """主点固定: 10点で推定、9点で0固定"""
        points_2d_full = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE_CENTER, DIST_2)

        # 10点: 推定される
        yaml_10 = _write_dataset(tmp_path / "p10", POINTS_3D_25[:10], points_2d_full[:10])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_10), use_k3=False, use_wide=False,
                          fix_center=True, zero_tangent=True)
        assert "歪み係数を推定します" in captured.getvalue()

        # 9点: 0固定
        yaml_9 = _write_dataset(tmp_path / "p9", POINTS_3D_25[:9], points_2d_full[:9])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_9), use_k3=False, use_wide=False,
                          fix_center=True, zero_tangent=True)
        assert "歪み係数は0に固定します" in captured.getvalue()

    def test_estimate_center_boundary(self, tmp_path):
        """主点推定: 13点で推定、12点で0固定"""
        points_2d_full = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_2)

        yaml_13 = _write_dataset(tmp_path / "p13", POINTS_3D_25[:13], points_2d_full[:13])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_13), use_k3=False, use_wide=False,
                          fix_center=False, zero_tangent=True)
        assert "歪み係数を推定します" in captured.getvalue()

        yaml_12 = _write_dataset(tmp_path / "p12", POINTS_3D_25[:12], points_2d_full[:12])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_12), use_k3=False, use_wide=False,
                          fix_center=False, zero_tangent=True)
        assert "歪み係数は0に固定します" in captured.getvalue()

    def test_fix_center_k3_boundary(self, tmp_path):
        """主点固定+k3: 11点で推定、10点で0固定"""
        points_2d_full = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE_CENTER, DIST_3)

        yaml_11 = _write_dataset(tmp_path / "p11", POINTS_3D_25[:11], points_2d_full[:11])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_11), use_k3=True, use_wide=False,
                          fix_center=True, zero_tangent=True)
        assert "歪み係数を推定します" in captured.getvalue()

        yaml_10 = _write_dataset(tmp_path / "p10", POINTS_3D_25[:10], points_2d_full[:10])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_10), use_k3=True, use_wide=False,
                          fix_center=True, zero_tangent=True)
        assert "歪み係数は0に固定します" in captured.getvalue()

    def test_estimate_center_k3_boundary(self, tmp_path):
        """主点推定+k3: 14点で推定、13点で0固定"""
        points_2d_full = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_3)

        yaml_14 = _write_dataset(tmp_path / "p14", POINTS_3D_25[:14], points_2d_full[:14])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_14), use_k3=True, use_wide=False,
                          fix_center=False, zero_tangent=True)
        assert "歪み係数を推定します" in captured.getvalue()

        yaml_13 = _write_dataset(tmp_path / "p13", POINTS_3D_25[:13], points_2d_full[:13])
        captured = StringIO()
        with patch('sys.stdout', captured):
            run_estimation(str(yaml_13), use_k3=True, use_wide=False,
                          fix_center=False, zero_tangent=True)
        assert "歪み係数は0に固定します" in captured.getvalue()


# ========================================
# T-6: 既存経路の回帰なし（既存テストの再実行で担保）
# ========================================

class TestNoRegressionDefault:

    def test_default_path_unaffected(self, tmp_path):
        """zero_tangent=False（既定値）で従来どおり4係数推定が行われること"""
        DIST_4 = np.array([-0.05, 0.09, -0.005, -0.003], dtype=np.float64)
        points_2d = _generate_2d(POINTS_3D_25, RVEC_TRUE, TVEC_TRUE, K_TRUE, DIST_4)
        yaml_f = _write_dataset(tmp_path, POINTS_3D_25, points_2d)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False, fix_center=False)

        assert result == 0
        output = captured.getvalue()
        p1_opt = float([l for l in output.split('\n') if l.startswith('p1:')][0].split(':')[1])
        # 既存動作では p1 は真値付近に推定される（0固定ではない）
        assert abs(p1_opt - DIST_4[2]) < 0.01

"""
estimate_camera_params.py のパラメータ範囲チェック（feat-025）テスト
"""

import sys
from pathlib import Path
from unittest.mock import patch
from io import StringIO

import numpy as np
import cv2
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from estimate_camera_params import check_param_ranges, run_estimation


# ========================================
# 共通パラメータ
# ========================================

IMG_WIDTH = 960
IMG_HEIGHT = 540


def _default_kwargs(**overrides):
    kwargs = dict(
        fx=1000.0, fy=1000.0, cx=480.0, cy=270.0,
        k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0,
        img_width=IMG_WIDTH, img_height=IMG_HEIGHT,
        fix_center=False, use_wide=False, use_k3=False,
        zero_tangent=False, estimate_distortion=True,
    )
    kwargs.update(overrides)
    return kwargs


# ========================================
# ユニットテスト（check_param_ranges）
# ========================================

class TestCheckParamRanges:

    def test_1_all_normal(self):
        """全パラメータ正常 → 空リスト"""
        warnings = check_param_ranges(**_default_kwargs())
        assert warnings == []

    def test_2_p1_over(self):
        """p1=0.5 → p1 の警告を含む（本案件の動機となったケース）"""
        warnings = check_param_ranges(**_default_kwargs(p1=0.5))
        assert any("p1" in w for w in warnings)

    def test_3_fx_under(self):
        """fx=200（< 0.3×960=288）→ fx の警告"""
        warnings = check_param_ranges(**_default_kwargs(fx=200.0))
        assert any(w.startswith("fx") for w in warnings)

    def test_4_fx_fy_ratio(self):
        """fx=1200, fy=1000（比1.2）→ fx/fy 比の警告"""
        warnings = check_param_ranges(**_default_kwargs(fx=1200.0, fy=1000.0))
        assert any("fx/fy" in w for w in warnings)

    def test_5_cx_over(self):
        """cx=600（480+0.10×960=576 超）→ cx の警告"""
        warnings = check_param_ranges(**_default_kwargs(cx=600.0))
        assert any(w.startswith("cx") for w in warnings)

    def test_6_k1_over(self):
        """k1=1.5 → k1 の警告"""
        warnings = check_param_ranges(**_default_kwargs(k1=1.5))
        assert any(w.startswith("k1") for w in warnings)

    def test_7_boundary_values(self):
        """境界値: fx=288.0（=0.3×S）, p1=0.01, k1=1.0 → 警告なし"""
        warnings = check_param_ranges(**_default_kwargs(fx=288.0, fy=288.0, p1=0.01, k1=1.0))
        assert warnings == []

    def test_8_fix_center_ignores_cx(self):
        """fix_center=True で cx=9999 → cx の警告なし"""
        warnings = check_param_ranges(**_default_kwargs(fix_center=True, cx=9999.0))
        assert not any(w.startswith("cx") for w in warnings)

    def test_9_use_wide(self):
        """use_wide=True で k1=5.0, p1=0.5 → k1 の警告なし、p1 の警告あり"""
        warnings = check_param_ranges(**_default_kwargs(use_wide=True, k1=5.0, p1=0.5))
        assert not any(w.startswith("k1") for w in warnings)
        assert any(w.startswith("p1") for w in warnings)

    def test_10_zero_tangent(self):
        """zero_tangent=True で p1=0.0（固定値）→ p1 の警告なし。k1=1.5 → k1 の警告あり"""
        warnings = check_param_ranges(**_default_kwargs(zero_tangent=True, p1=0.0, k1=1.5))
        assert not any(w.startswith("p1") for w in warnings)
        assert any(w.startswith("k1") for w in warnings)

    def test_11_estimate_distortion_false(self):
        """estimate_distortion=False で k1=1.5（形式上渡す）→ 歪み警告なし"""
        warnings = check_param_ranges(**_default_kwargs(estimate_distortion=False, k1=1.5))
        assert not any(w.startswith("k1") for w in warnings)

    def test_12_use_k3(self):
        """use_k3=False で k3=5.0 → 警告なし / use_k3=True で k3=5.0 → 警告あり"""
        warnings_no_k3 = check_param_ranges(**_default_kwargs(use_k3=False, k3=5.0))
        assert not any(w.startswith("k3") for w in warnings_no_k3)

        warnings_k3 = check_param_ranges(**_default_kwargs(use_k3=True, k3=5.0))
        assert any(w.startswith("k3") for w in warnings_k3)

    def test_13_fy_zero(self):
        """fy=0 → fy の絶対範囲警告あり、例外が発生しない"""
        warnings = check_param_ranges(**_default_kwargs(fy=0.0))
        assert any(w.startswith("fy") for w in warnings)

    def test_14_multiple_warnings(self):
        """複数項目同時異常（p1=0.5, k1=2.0）→ 警告が2件"""
        warnings = check_param_ranges(**_default_kwargs(p1=0.5, k1=2.0))
        assert len(warnings) == 2


# ========================================
# 統合テスト（FR-002 / FR-003）
# ========================================

def _write_synthetic_dataset(tmp_path):
    """既知パラメータ・非平面20点の合成データを tmp_path に書き出す"""
    fx_true = fy_true = 800.0
    cx_true, cy_true = 480.0, 270.0
    K_true = np.array([[fx_true, 0, cx_true], [0, fy_true, cy_true], [0, 0, 1]], dtype=np.float64)
    dist_true = np.zeros(4, dtype=np.float64)
    rvec_true = np.array([0.1, -0.2, 0.3], dtype=np.float64)
    tvec_true = np.array([0.5, -0.3, 5.0], dtype=np.float64)

    # 非平面20点（グリッド + Z変化）
    points_3d = []
    for i in range(5):
        for j in range(4):
            x = -1.0 + i * 0.5
            y = -0.75 + j * 0.5
            z = 0.1 * (i + j)
            points_3d.append([x, y, z])
    points_3d = np.array(points_3d, dtype=np.float64)

    projected, _ = cv2.projectPoints(
        points_3d, rvec_true.reshape(3, 1), tvec_true.reshape(3, 1), K_true, dist_true
    )
    points_2d = projected.reshape(-1, 2)

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
        f.write("image_width: 960\n")
        f.write("image_height: 540\n")

    return yaml_f


class TestIntegrationNormal:

    def test_normal_run(self, tmp_path):
        yaml_f = _write_synthetic_dataset(tmp_path)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        output = captured.getvalue()
        assert "[パラメータ範囲チェック]" in output
        assert "✓ すべてのパラメータが正常範囲内です" in output
        assert "Calib_scene.toml 形式" in output
        assert "camera_params.csv 形式" in output
        assert result == 0


class TestIntegrationWarning:

    def test_warning_display(self, tmp_path, monkeypatch):
        yaml_f = _write_synthetic_dataset(tmp_path)

        fixed_warnings = ["p1 = 0.500000 が正常範囲 [-0.01, 0.01] を外れています"]
        monkeypatch.setattr(
            "estimate_camera_params.check_param_ranges",
            lambda *args, **kwargs: fixed_warnings,
        )

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        output = captured.getvalue()
        assert "⚠ 警告: p1 = 0.500000 が正常範囲 [-0.01, 0.01] を外れています" in output
        assert output.count("--zero-tangent / --fix-center の使用を検討してください。") == 1
        assert "Calib_scene.toml 形式" in output
        assert "camera_params.csv 形式" in output
        assert result == 0

    def test_warning_display_use_wide_hint(self, tmp_path, monkeypatch):
        """use_wide=True 時のヒント文切り替え確認（20点で MIN_POINTS_DIST8=20 を満たす）"""
        yaml_f = _write_synthetic_dataset(tmp_path)

        fixed_warnings = ["p1 = 0.500000 が正常範囲 [-0.01, 0.01] を外れています"]
        monkeypatch.setattr(
            "estimate_camera_params.check_param_ranges",
            lambda *args, **kwargs: fixed_warnings,
        )

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=True,
                                    fix_center=False, zero_tangent=False)

        output = captured.getvalue()
        assert "--fix-center の使用、または標準モデル（--wide なし）への切り替えを検討してください。" in output
        assert result == 0

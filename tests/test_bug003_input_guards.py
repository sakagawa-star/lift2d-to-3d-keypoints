"""
estimate_camera_params.py の入力ガード・収束判定テスト（bug-003）
"""

import sys
from pathlib import Path
from io import StringIO
from unittest.mock import patch

import numpy as np
import cv2
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

import estimate_camera_params
from estimate_camera_params import run_estimation, main, _run_extrinsic_estimation


# ========================================
# 共通ヘルパー
# ========================================

def _write_dataset(tmp_path, num_points=8, target_camera='cam01',
                    points_3d_name='pts3d.csv', points_2d_name='pts2d.csv',
                    config_name='config.yaml', extra_lines=None,
                    omit_keys=None, blank_keys=None):
    """最小の3D/2D CSVとconfig.yamlをtmp_path上に生成する"""
    fx_true = fy_true = 800.0
    cx_true, cy_true = 480.0, 270.0
    K_true = np.array([[fx_true, 0, cx_true], [0, fy_true, cy_true], [0, 0, 1]], dtype=np.float64)
    dist_true = np.zeros(4, dtype=np.float64)
    rvec_true = np.array([0.1, -0.2, 0.3], dtype=np.float64)
    tvec_true = np.array([0.5, -0.3, 5.0], dtype=np.float64)

    points_3d = []
    i = 0
    while len(points_3d) < num_points:
        x = -1.0 + (i % 5) * 0.5
        y = -0.75 + (i // 5) * 0.5
        z = 0.1 * i
        points_3d.append([x, y, z])
        i += 1
    points_3d = np.array(points_3d[:num_points], dtype=np.float64)

    projected, _ = cv2.projectPoints(
        points_3d, rvec_true.reshape(3, 1), tvec_true.reshape(3, 1), K_true, dist_true
    )
    points_2d = projected.reshape(-1, 2)

    tmp_path.mkdir(parents=True, exist_ok=True)
    csv_3d = tmp_path / points_3d_name
    csv_2d = tmp_path / points_2d_name
    yaml_f = tmp_path / config_name

    with open(csv_3d, 'w') as f:
        f.write("ObjectName,X,Y,Z\n")
        for i, p in enumerate(points_3d):
            f.write(f"pt{i},{p[0]},{p[1]},{p[2]}\n")

    with open(csv_2d, 'w') as f:
        f.write("ObjectName,camera_name,X,Y\n")
        for i, p in enumerate(points_2d):
            f.write(f"pt{i},{target_camera},{p[0]},{p[1]}\n")

    omit_keys = omit_keys or []
    blank_keys = blank_keys or []

    kv = {
        'target_camera': target_camera,
        'points_3d': points_3d_name,
        'points_2d': points_2d_name,
        'image_width': '960',
        'image_height': '540',
    }
    with open(yaml_f, 'w') as f:
        for key, value in kv.items():
            if key in omit_keys:
                continue
            if key in blank_keys:
                f.write(f"{key}: \n")
            else:
                f.write(f"{key}: {value}\n")
        if extra_lines:
            for line in extra_lines:
                f.write(line + "\n")

    return yaml_f


def _write_intrinsic_toml(tmp_path, camera_name='cam01', omit_key=None, name='intrinsic.toml'):
    """K既知モード用TOMLを生成する。omit_keyでセクション内キーを欠落させる"""
    lines = [
        f'[{camera_name}]',
        f'name = "{camera_name}"',
        'size = [1920.0, 1080.0]',
        'matrix = [[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]',
        'distortions = [-0.0536, 0.0983, -0.0054, -0.0027]',
        'rotation = [0.0, 0.0, 0.0]',
        'translation = [0.0, 0.0, 0.0]',
        'fisheye = false',
    ]
    key_to_line_prefix = {
        'matrix': 'matrix = ',
        'distortions': 'distortions = ',
        'size': 'size = ',
    }
    if omit_key:
        prefix = key_to_line_prefix[omit_key]
        lines = [l for l in lines if not l.startswith(prefix)]

    toml_f = tmp_path / name
    toml_f.write_text("\n".join(lines) + "\n")
    return toml_f


# ========================================
# 1. H-1: 5点でエラー終了
# ========================================

class TestMinPointsGuard:

    def test_5_points_returns_1(self, tmp_path):
        yaml_f = _write_dataset(tmp_path, num_points=5)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "最低6点必要" in captured.getvalue()

    def test_6_points_returns_0(self, tmp_path):
        """境界値: 6点で歪みなしモードが正常終了する"""
        yaml_f = _write_dataset(tmp_path, num_points=6)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 0


# ========================================
# 3. M-3-1: config必須キー欠落・空文字（関数レベル）
# ========================================

class TestConfigRequiredKeysFunctionLevel:

    @pytest.mark.parametrize("key", ['target_camera', 'points_3d', 'points_2d'])
    def test_missing_key(self, tmp_path, key):
        yaml_f = _write_dataset(tmp_path, omit_keys=[key])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "がありません" in captured.getvalue()

    @pytest.mark.parametrize("key", ['target_camera', 'points_3d', 'points_2d'])
    def test_blank_key(self, tmp_path, key):
        yaml_f = _write_dataset(tmp_path, blank_keys=[key])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "がありません" in captured.getvalue()


# ========================================
# 4. M-3-1: main() 経由（CLIレベル）
# ========================================

class TestConfigRequiredKeysCliLevel:

    def test_missing_target_camera_via_main(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path, omit_keys=['target_camera'])

        monkeypatch.setattr(sys, 'argv', ['estimate_camera_params.py', str(yaml_f)])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "target_camera がありません" in captured.getvalue()


# ========================================
# 5. M-3-2: CSVファイル不在・ディレクトリ指定
# ========================================

class TestCsvFileExistence:

    def test_points_2d_not_found(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        # points_2d を存在しないファイル名に書き換える
        content = yaml_f.read_text()
        content = content.replace("points_2d: pts2d.csv", "points_2d: nonexistent.csv")
        yaml_f.write_text(content)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "入力ファイルが見つかりません" in captured.getvalue()

    def test_points_3d_is_directory(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        dir_path = tmp_path / "somedir"
        dir_path.mkdir()
        content = yaml_f.read_text()
        content = content.replace("points_3d: pts3d.csv", "points_3d: somedir")
        yaml_f.write_text(content)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "入力ファイルが見つかりません" in captured.getvalue()


# ========================================
# 5b. M-3-1/M-3-2: K既知経路（intrinsic_toml指定）
# ========================================

class TestConfigGuardsExtrinsicPath:

    @pytest.mark.parametrize("key", ['target_camera', 'points_3d', 'points_2d'])
    def test_missing_key(self, tmp_path, key):
        yaml_f = _write_dataset(tmp_path, omit_keys=[key])
        toml_f = _write_intrinsic_toml(tmp_path)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f))

        assert result == 1
        assert "がありません" in captured.getvalue()

    @pytest.mark.parametrize("key", ['target_camera', 'points_3d', 'points_2d'])
    def test_blank_key(self, tmp_path, key):
        yaml_f = _write_dataset(tmp_path, blank_keys=[key])
        toml_f = _write_intrinsic_toml(tmp_path)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f))

        assert result == 1
        assert "がありません" in captured.getvalue()

    def test_csv_not_found(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        content = yaml_f.read_text()
        content = content.replace("points_2d: pts2d.csv", "points_2d: nonexistent.csv")
        yaml_f.write_text(content)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f))

        assert result == 1
        assert "入力ファイルが見つかりません" in captured.getvalue()


# ========================================
# 6. M-3-3: TOMLセクション内キー欠落
# ========================================

class TestIntrinsicTomlSectionKeys:

    def test_broken_camera_causes_overall_failure(self, tmp_path):
        """2カメラ構成でcam02のmatrixキー欠落 → 全体を終了コード1にする"""
        yaml_f = _write_dataset(tmp_path, target_camera='cam01')

        # config.yaml の target_camera を複数カメラ指定に書き換える
        content = yaml_f.read_text()
        content = content.replace("target_camera: cam01", "target_camera: cam01,cam02")
        yaml_f.write_text(content)

        # cam02用の2D観測データも追加（cam01と同じ2D座標を使う）
        pts2d_content = (tmp_path / 'pts2d.csv').read_text()
        extra_lines = []
        for line in pts2d_content.strip().split('\n')[1:]:
            name, cam, x, y = line.split(',')
            extra_lines.append(f"{name},cam02,{x},{y}")
        with open(tmp_path / 'pts2d.csv', 'a') as f:
            for line in extra_lines:
                f.write(line + "\n")

        # cam01は正常、cam02はmatrixキー欠落
        toml_f = tmp_path / "intrinsic.toml"
        lines_cam01 = [
            '[cam01]',
            'name = "cam01"',
            'size = [1920.0, 1080.0]',
            'matrix = [[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]',
            'distortions = [-0.0536, 0.0983, -0.0054, -0.0027]',
            'rotation = [0.0, 0.0, 0.0]',
            'translation = [0.0, 0.0, 0.0]',
            'fisheye = false',
            '',
            '[cam02]',
            'name = "cam02"',
            'size = [1920.0, 1080.0]',
            'distortions = [-0.0536, 0.0983, -0.0054, -0.0027]',
            'rotation = [0.0, 0.0, 0.0]',
            'translation = [0.0, 0.0, 0.0]',
            'fisheye = false',
        ]
        toml_f.write_text("\n".join(lines_cam01) + "\n")

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "matrix がありません" in captured.getvalue()

    def test_missing_section_still_skips(self, tmp_path):
        """セクション自体の不在は従来どおりスキップされ、正常カメラのみで成功する"""
        yaml_f = _write_dataset(tmp_path, target_camera='cam01')

        # config.yaml の target_camera を複数カメラ指定に書き換える
        content = yaml_f.read_text()
        content = content.replace("target_camera: cam01", "target_camera: cam01,cam02")
        yaml_f.write_text(content)

        pts2d_content = (tmp_path / 'pts2d.csv').read_text()
        extra_lines = []
        for line in pts2d_content.strip().split('\n')[1:]:
            name, cam, x, y = line.split(',')
            extra_lines.append(f"{name},cam02,{x},{y}")
        with open(tmp_path / 'pts2d.csv', 'a') as f:
            for line in extra_lines:
                f.write(line + "\n")

        # cam02のセクション自体が存在しない
        toml_f = _write_intrinsic_toml(tmp_path, camera_name='cam01')

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 0
        assert "スキップします" in captured.getvalue()


# ========================================
# 7. M-2: 収束判定
# ========================================

class TestConvergenceWarning:

    def test_non_converged_warning(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path, num_points=8)

        class DummyResult:
            def __init__(self, x):
                self.x = x
                self.status = 0
                self.message = "反復回数の上限に達しました"

        def fake_least_squares(residual_func, params_init, args, method, verbose):
            return DummyResult(np.array(params_init, dtype=np.float64))

        monkeypatch.setattr(estimate_camera_params, "least_squares", fake_least_squares)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        output = captured.getvalue()
        assert "収束しませんでした" in output
        assert result == 0

"""
estimate_camera_params.py の --output 出力ガード・パス種別検証・
TOML値検証・オプション排他・fisheye未対応化のテスト（bug-004）
"""

import sys
from pathlib import Path
from io import StringIO
from unittest.mock import patch

import numpy as np
import cv2
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase0'))

from estimate_camera_params import run_estimation, main, _run_extrinsic_estimation, _write_toml_output


# ========================================
# 共通ヘルパー（test_bug003_input_guards.py のスタイルを踏襲）
# ========================================

def _write_dataset(tmp_path, num_points=8, target_camera='cam01',
                    points_3d_name='pts3d.csv', points_2d_name='pts2d.csv',
                    config_name='config.yaml'):
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

    with open(yaml_f, 'w') as f:
        f.write(f"target_camera: {target_camera}\n")
        f.write(f"points_3d: {points_3d_name}\n")
        f.write(f"points_2d: {points_2d_name}\n")
        f.write("image_width: 960\n")
        f.write("image_height: 540\n")

    return yaml_f


def _write_intrinsic_toml(tmp_path, camera_name='cam01', name='intrinsic.toml',
                           matrix_line=None, distortions_line=None, size_line=None,
                           fisheye=False):
    """K既知モード用TOMLを生成する。各行を差し替え可能にして不正値テストに使う"""
    lines = [
        f'[{camera_name}]',
        f'name = "{camera_name}"',
        size_line if size_line is not None else 'size = [1920.0, 1080.0]',
        matrix_line if matrix_line is not None else
        'matrix = [[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]',
        distortions_line if distortions_line is not None else
        'distortions = [-0.0536, 0.0983, -0.0054, -0.0027]',
        'rotation = [0.0, 0.0, 0.0]',
        'translation = [0.0, 0.0, 0.0]',
        f'fisheye = {"true" if fisheye else "false"}',
    ]
    toml_f = tmp_path / name
    toml_f.write_text("\n".join(lines) + "\n")
    return toml_f


# ========================================
# 1. H-1: --output と --intrinsic-toml の同一パス防止
# ========================================

class TestSamePathGuard:

    def test_same_path_via_run_estimation(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        original_content = toml_f.read_text()

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f),
                                    output_path=str(toml_f))

        assert result == 1
        assert "同じパスは指定できません" in captured.getvalue()
        assert toml_f.read_text() == original_content

    def test_same_path_via_main(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        original_content = toml_f.read_text()

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--intrinsic-toml', str(toml_f),
            '--output', str(toml_f),
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "同じパスは指定できません" in captured.getvalue()
        assert toml_f.read_text() == original_content

    def test_same_path_relative_vs_absolute(self, tmp_path, monkeypatch):
        """相対/絶対の表記違いでも同一パスとして検出される"""
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        original_content = toml_f.read_text()

        monkeypatch.chdir(tmp_path)
        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f),
                                    output_path=toml_f.name)

        assert result == 1
        assert "同じパスは指定できません" in captured.getvalue()
        assert toml_f.read_text() == original_content


# ========================================
# 2. H-1（原子性）: .tmp が残らない・出力内容が従来と同一
# ========================================

class TestAtomicWrite:

    def test_no_tmp_file_left_and_content_matches(self, tmp_path):
        output_f = tmp_path / "output.toml"
        results = {
            'cam01': {
                'name': 'cam01',
                'size': [1920.0, 1080.0],
                'K': np.array([[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]),
                'dist': np.array([-0.0536, 0.0983, -0.0054, -0.0027]),
                'rvec': np.array([0.1, 0.2, 0.3]),
                'tvec': np.array([0.4, 0.5, 5.0]),
            }
        }

        captured = StringIO()
        with patch('sys.stdout', captured):
            _write_toml_output(str(output_f), results)

        tmp_path_candidate = output_f.with_suffix(output_f.suffix + '.tmp')
        assert not tmp_path_candidate.exists()
        assert output_f.exists()
        content = output_f.read_text()
        assert '[cam01]' in content
        assert 'matrix = ' in content
        assert 'distortions = ' in content


# ========================================
# 3. H-2: config / --intrinsic-toml / --output のパス種別検証
# ========================================

class TestPathTypeGuards:

    def test_config_is_directory_via_run_estimation(self, tmp_path):
        dir_path = tmp_path / "somedir"
        dir_path.mkdir()

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(dir_path), use_k3=False, use_wide=False,
                                    fix_center=False, zero_tangent=False)

        assert result == 1
        assert "設定ファイルが見つかりません" in captured.getvalue()

    def test_config_is_directory_via_main(self, tmp_path, monkeypatch):
        dir_path = tmp_path / "somedir"
        dir_path.mkdir()

        monkeypatch.setattr(sys, 'argv', ['estimate_camera_params.py', str(dir_path)])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "設定ファイルが見つかりません" in captured.getvalue()

    def test_intrinsic_toml_is_directory_via_run_estimation(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        dir_path = tmp_path / "somedir"
        dir_path.mkdir()

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(dir_path))

        assert result == 1
        assert "TOMLファイルが見つかりません" in captured.getvalue()

    def test_intrinsic_toml_is_directory_via_main(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)
        dir_path = tmp_path / "somedir"
        dir_path.mkdir()

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--intrinsic-toml', str(dir_path),
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "TOMLファイルが見つかりません" in captured.getvalue()

    def test_output_is_existing_directory(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        output_dir = tmp_path / "outdir"
        output_dir.mkdir()

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f),
                                    output_path=str(output_dir))

        assert result == 1
        assert "ディレクトリです" in captured.getvalue()

    def test_output_is_existing_directory_via_main(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        output_dir = tmp_path / "outdir"
        output_dir.mkdir()

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--intrinsic-toml', str(toml_f),
            '--output', str(output_dir),
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "ディレクトリです" in captured.getvalue()

    def test_output_parent_not_exist(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        output_f = tmp_path / "nonexistent_dir" / "out.toml"

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f),
                                    output_path=str(output_f))

        assert result == 1
        assert "出力先ディレクトリが存在しません" in captured.getvalue()

    def test_output_parent_not_exist_via_main(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        output_f = tmp_path / "nonexistent_dir" / "out.toml"

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--intrinsic-toml', str(toml_f),
            '--output', str(output_f),
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "出力先ディレクトリが存在しません" in captured.getvalue()

    def test_output_parent_is_existing_file(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)
        parent_file = tmp_path / "parent_is_file"
        parent_file.write_text("dummy")
        output_f = parent_file / "out.toml"

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = run_estimation(str(yaml_f), use_k3=False, use_wide=False,
                                    fix_center=False, intrinsic_toml=str(toml_f),
                                    output_path=str(output_f))

        assert result == 1
        assert "出力先ディレクトリが存在しません" in captured.getvalue()


# ========================================
# 4. H-3: TOML値の形状・長さ検証
# ========================================

class TestTomlValueShapeValidation:

    def test_matrix_not_3x3(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(
            tmp_path, matrix_line='matrix = [[1177.34, 0.0], [0.0, 1177.82]]'
        )

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "matrix が不正です" in captured.getvalue()

    def test_distortions_length_2(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(
            tmp_path, distortions_line='distortions = [-0.0536, 0.0983]'
        )

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "distortions が不正です" in captured.getvalue()

    def test_size_1_element(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path, size_line='size = [1920.0]')

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "size が不正です" in captured.getvalue()

    def test_size_negative(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path, size_line='size = [-1920.0, 1080.0]')

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "size が不正です" in captured.getvalue()

    def test_size_non_numeric(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path, size_line='size = ["a", "b"]')

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "size が不正です" in captured.getvalue()


# ========================================
# 5. M-1: --intrinsic-toml 指定時は --wide --zero-tangent 併用が警告のみ
# ========================================

class TestExclusiveOptionsWithIntrinsicToml:

    def test_wide_zero_tangent_with_intrinsic_toml_warns_not_errors(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path)

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--intrinsic-toml', str(toml_f),
            '--wide', '--zero-tangent',
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        output = captured.getvalue()
        assert "併用できません" not in output
        assert "無視されます" in output
        assert result == 0

    def test_wide_zero_tangent_without_intrinsic_toml_still_errors(self, tmp_path, monkeypatch):
        yaml_f = _write_dataset(tmp_path)

        monkeypatch.setattr(sys, 'argv', [
            'estimate_camera_params.py', str(yaml_f),
            '--wide', '--zero-tangent',
        ])

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = main()

        assert result == 1
        assert "併用できません" in captured.getvalue()


# ========================================
# 6. M-2: fisheye = true の未対応エラー
# ========================================

class TestFisheyeUnsupported:

    def test_fisheye_true_raises_error(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path, fisheye=True)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 1
        assert "fisheye = true です" in captured.getvalue()
        assert "未対応です" in captured.getvalue()

    def test_fisheye_false_works_as_before(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = _write_intrinsic_toml(tmp_path, fisheye=False)

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 0

    def test_fisheye_key_absent_works_as_before(self, tmp_path):
        yaml_f = _write_dataset(tmp_path)
        toml_f = tmp_path / "intrinsic_no_fisheye.toml"
        toml_f.write_text(
            "\n".join([
                '[cam01]',
                'name = "cam01"',
                'size = [1920.0, 1080.0]',
                'matrix = [[1177.34, 0.0, 956.70], [0.0, 1177.82, 495.77], [0.0, 0.0, 1.0]]',
                'distortions = [-0.0536, 0.0983, -0.0054, -0.0027]',
                'rotation = [0.0, 0.0, 0.0]',
                'translation = [0.0, 0.0, 0.0]',
            ]) + "\n"
        )

        captured = StringIO()
        with patch('sys.stdout', captured):
            result = _run_extrinsic_estimation(str(yaml_f), str(toml_f))

        assert result == 0

"""
feat-012: camera_pose.py カメラ名・出力先のCLIオプション化のテスト

bpy 非依存の純粋ロジック（引数切り出し・引数解析・デフォルト出力パス生成）のみを対象とする。
bpy 依存部分（export_camera_poses, main）は実機 Blender での手動テストで確認する。
"""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase4'))

from camera_pose import (
    default_output_path,
    extract_script_args,
    parse_args,
)


# ========================================
# extract_script_args のユニットテスト
# ========================================

class TestExtractScriptArgs:
    def test_with_separator(self):
        """'--' 以降のスクリプト引数が取り出されること"""
        argv = ['blender', '-b', 'x.blend', '--python', 'camera_pose.py',
                '--', '--camera', 'cam1']
        assert extract_script_args(argv) == ['--camera', 'cam1']

    def test_without_separator(self):
        """'--' がない場合は空リストを返すこと"""
        argv = ['blender', '-b', 'x.blend', '--python', 'camera_pose.py']
        assert extract_script_args(argv) == []

    def test_separator_at_end(self):
        """'--' が末尾の場合は空リストを返すこと"""
        argv = ['blender', '-b', 'x.blend', '--python', 'camera_pose.py', '--']
        assert extract_script_args(argv) == []

    def test_multiple_separators(self):
        """'--' が複数ある場合は最初の '--' を区切りとすること"""
        argv = ['blender', '--', '--camera', 'cam1', '--', 'x']
        assert extract_script_args(argv) == ['--camera', 'cam1', '--', 'x']


# ========================================
# default_output_path のユニットテスト
# ========================================

class TestDefaultOutputPath:
    def test_default_path(self):
        """data/<カメラ名>_poses.json が返ること"""
        assert default_output_path('cam41520554') == os.path.join(
            'data', 'cam41520554_poses.json')


# ========================================
# parse_args のユニットテスト
# ========================================

class TestParseArgs:
    def test_camera_only_uses_default_output(self):
        """--camera のみ指定時、output がデフォルトパスになること"""
        args = parse_args(['--camera', 'cam1'])
        assert args.camera == 'cam1'
        assert args.output == default_output_path('cam1')

    def test_output_override(self):
        """--output 指定時、指定値が使われること"""
        args = parse_args(['--camera', 'cam1', '--output', 'out/poses.json'])
        assert args.camera == 'cam1'
        assert args.output == 'out/poses.json'

    def test_missing_camera_exits(self):
        """--camera 未指定時、終了コード2で SystemExit すること"""
        with pytest.raises(SystemExit) as e:
            parse_args([])
        assert e.value.code == 2

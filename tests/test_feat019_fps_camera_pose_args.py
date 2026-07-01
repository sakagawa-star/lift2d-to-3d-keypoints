"""
feat-019: fps_camera_pose.py 頭部追従カメラポーズ書き出しのテスト

bpy / mathutils 非依存の純粋ロジック（引数切り出し・引数解析・デフォルト出力パス生成、
および定数定義）のみを対象とする。
bpy / mathutils 依存部分（compute_anchor_rotation_euler, validate_*,
export_camera_poses, main）は実機 Blender でのヘッドレステストで確認する。
"""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / 'phase4'))

from fps_camera_pose import (
    DEFAULT_ANCHOR_NAME,
    DEFAULT_ARMATURE_NAME,
    REQUIRED_BONES,
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
        argv = ['blender', '-b', 'x.blend', '--python', 'fps_camera_pose.py',
                '--', '--camera', 'Cam_FPS']
        assert extract_script_args(argv) == ['--camera', 'Cam_FPS']

    def test_without_separator(self):
        """'--' がない場合は空リストを返すこと"""
        argv = ['blender', '-b', 'x.blend', '--python', 'fps_camera_pose.py']
        assert extract_script_args(argv) == []

    def test_separator_at_end(self):
        """'--' が末尾の場合は空リストを返すこと"""
        argv = ['blender', '-b', 'x.blend', '--python', 'fps_camera_pose.py', '--']
        assert extract_script_args(argv) == []

    def test_multiple_separators(self):
        """'--' が複数ある場合は最初の '--' を区切りとすること"""
        argv = ['blender', '--', '--camera', 'Cam_FPS', '--', 'x']
        assert extract_script_args(argv) == ['--camera', 'Cam_FPS', '--', 'x']


# ========================================
# default_output_path のユニットテスト
# ========================================

class TestDefaultOutputPath:
    def test_default_path(self):
        """data/<カメラ名>_poses.json が返ること"""
        assert default_output_path('Cam_FPS') == os.path.join(
            'data', 'Cam_FPS_poses.json')


# ========================================
# parse_args のユニットテスト
# ========================================

class TestParseArgs:
    def test_camera_only_uses_defaults(self):
        """--camera のみ指定時、armature/anchor/output がデフォルトになること"""
        args = parse_args(['--camera', 'Cam_FPS'])
        assert args.camera == 'Cam_FPS'
        assert args.armature == DEFAULT_ARMATURE_NAME
        assert args.anchor == DEFAULT_ANCHOR_NAME
        assert args.output == default_output_path('Cam_FPS')

    def test_armature_anchor_override(self):
        """--armature / --anchor 指定時、指定値が使われること"""
        args = parse_args(['--camera', 'Cam_FPS',
                           '--armature', 'ArmX', '--anchor', 'AnchorY'])
        assert args.armature == 'ArmX'
        assert args.anchor == 'AnchorY'

    def test_output_override(self):
        """--output 指定時、指定値が使われること"""
        args = parse_args(['--camera', 'Cam_FPS', '--output', 'out/poses.json'])
        assert args.camera == 'Cam_FPS'
        assert args.output == 'out/poses.json'

    def test_missing_camera_exits(self):
        """--camera 未指定時、終了コード2で SystemExit すること"""
        with pytest.raises(SystemExit) as e:
            parse_args([])
        assert e.value.code == 2


# ========================================
# 定数定義のユニットテスト
# ========================================

class TestConstants:
    def test_required_bones(self):
        """姿勢計算に必要な7ボーンが定義されていること"""
        assert REQUIRED_BONES == ['LEye', 'REye', 'LEar', 'REar', 'Nose', 'Head', 'Neck']

    def test_default_names(self):
        """デフォルトのアーマチュア名・アンカー名が対象.blend構成であること"""
        assert DEFAULT_ARMATURE_NAME == 'session001_f145749_world300'
        assert DEFAULT_ANCHOR_NAME == 'Cam_Anchor'

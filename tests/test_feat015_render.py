"""
feat-015: ピンホール3DGSレンダリング（PNG出力）のテスト

bpy/CUDA非依存の純粋ロジック（カメラ行列・カメラ選択・TOML読込）と、
main のエラー終了経路（PNG保存失敗・カメラ不在）を対象とする。
gsplatレンダリング本体（render_image）は実機GPUでの手動テストで確認する。

純粋ロジックは合成データのみで完結させる。実データ（TOML）依存テストは
ファイル非存在時に skip する（gitignore対象で他環境に無いため）。
"""

import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (
    HALPE26_NAMES,
    camera_to_viewmat,
    load_cameras_toml,
    main,
    select_camera,
)

# 実データパス（phase4/data/Blender、gitignore対象）
_DATA = Path(__file__).parent.parent / "phase4" / "data" / "Blender"
_TOML = _DATA / "Config_scene.toml"

# 合成 Pose2Sim TOML（matrix を持つ camA のみカメラ。metadata は除外される）
_SYNTH_TOML = """\
[metadata]
adjusted = false

[camA]
name = "camA"
size = [1920, 1080]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""


def _write_synth_toml(tmp_path) -> str:
    """合成TOMLを tmp_path に書き、そのパスを返す。"""
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _install_render_stub(monkeypatch):
    """render.py（torch を top-level import）の代わりにダミーを sys.modules に仕込む。

    render.py はルートvenv（pytest）では import できないため、main 内の
    `from render import load_ply, print_ply_summary` がダミーを使うようにする。
    CUDA依存の render_image・phase4のみの load_c3d_all_frames もスタブ化し、
    PNG保存直前まで到達させる（feat-016 で c3d_path 必須化・return_depth 追加、
    feat-017 で全フレーム化・連番PNG出力に対応）。
    """
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: {"means": "stub"}
    dummy.print_ply_summary = lambda path, gaussians: None
    monkeypatch.setitem(sys.modules, "render", dummy)

    H, W = 1080, 1920

    def _stub_render_image(gaussians, cam, near_plane,
                           background=(0.0, 0.0, 0.0), return_depth=False):
        bgr = np.zeros((H, W, 3), dtype=np.uint8)
        if return_depth:
            depth_map = np.full((H, W), 100.0, dtype=np.float32)
            alpha_map = np.zeros((H, W), dtype=np.float32)  # 全画素 3DGSなし扱い
            return bgr, depth_map, alpha_map
        return bgr

    monkeypatch.setattr("render_keypoints.render_image", _stub_render_image)

    # C3Dスタブ: Halpe26 全26マーカーを原点付近に置き、全て有効（residual>=0）。
    # feat-017: (labels, frames_data, point_rate) を返す。先頭1フレームのみ。
    def _stub_load_c3d(path):
        labels = list(HALPE26_NAMES)
        frames_data = [{
            "frame_no": 1,
            "data": np.zeros((26, 3), dtype=np.float64),      # mm
            "residual": np.zeros(26, dtype=np.float64),
        }]
        return labels, frames_data, 30.0

    monkeypatch.setattr("render_keypoints.load_c3d_all_frames", _stub_load_c3d)


# ========================================
# camera_to_viewmat（合成データ）
# ========================================

class TestCameraToViewmat:
    def test_identity_rotation(self):
        """ゼロ回転では R=I、t がそのまま並進に入ること（4x4・float32）"""
        cam = {"rvec": np.zeros(3), "tvec": np.array([1.0, 2.0, 3.0])}
        vm = camera_to_viewmat(cam)
        assert vm.shape == (4, 4)
        assert vm.dtype == np.float32
        np.testing.assert_allclose(vm[:3, :3], np.eye(3), atol=1e-6)
        np.testing.assert_allclose(vm[:3, 3], [1.0, 2.0, 3.0], atol=1e-6)

    def test_bottom_row(self):
        """最下行が [0,0,0,1] であること"""
        cam = {"rvec": np.array([0.1, -0.2, 0.3]), "tvec": np.array([0.5, 0.5, 2.0])}
        vm = camera_to_viewmat(cam)
        np.testing.assert_allclose(vm[3, :], [0.0, 0.0, 0.0, 1.0])

    def test_rotation_matches_rodrigues(self):
        """R がOpenCV Rodriguesと一致すること"""
        rvec = np.array([1.2, -0.7, 0.4])
        cam = {"rvec": rvec, "tvec": np.array([0.0, 0.0, 1.0])}
        vm = camera_to_viewmat(cam)
        R_expected, _ = cv2.Rodrigues(rvec)
        np.testing.assert_allclose(vm[:3, :3], R_expected, atol=1e-6)

    def test_rotation_is_orthonormal(self):
        """回転部分が直交行列であること"""
        cam = {"rvec": np.array([0.5, 0.5, 0.5]), "tvec": np.zeros(3)}
        R = camera_to_viewmat(cam)[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-5)


# ========================================
# select_camera（合成データ・非skip）
# ========================================

class TestSelectCamera:
    def test_returns_existing(self):
        """存在名のカメラ辞書を返すこと"""
        cams = {"camA": {"name": "camA"}}
        assert select_camera(cams, "camA")["name"] == "camA"

    def test_missing_raises_with_available(self):
        """不在名で ValueError、メッセージに利用可能カメラ名を含むこと"""
        cams = {"camA": {"name": "camA"}, "camB": {"name": "camB"}}
        with pytest.raises(ValueError, match="camA"):
            select_camera(cams, "no_such")


# ========================================
# load_cameras_toml（合成データ・非skip）
# ========================================

class TestLoadCamerasTomlSynth:
    def test_loads_only_matrix_tables(self, tmp_path):
        """matrix を持つテーブルのみカメラとして読み、metadata を除外すること"""
        cams = load_cameras_toml(_write_synth_toml(tmp_path))
        assert set(cams.keys()) == {"camA"}  # metadata は除外される
        cam = cams["camA"]
        assert cam["K"].shape == (3, 3)
        assert cam["D"].shape[0] == 5
        assert cam["rvec"].shape == (3,)
        assert cam["tvec"].shape == (3,)
        assert cam["width"] == 1920
        assert cam["height"] == 1080


# ========================================
# main エラー終了経路（合成データ・非skip）
# ========================================

class TestMainErrorPaths:
    def test_imwrite_false_returns_1(self, tmp_path, monkeypatch, capsys):
        """cv2.imwrite が False を返すと終了コード1・stderrにエラー（要求仕様 3.4）"""
        _install_render_stub(monkeypatch)
        monkeypatch.setattr(cv2, "imwrite", lambda path, img: False)
        toml = _write_synth_toml(tmp_path)
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(tmp_path / "out")])
        assert rc == 1
        assert "保存に失敗" in capsys.readouterr().err

    def test_imwrite_cv2error_returns_1(self, tmp_path, monkeypatch, capsys):
        """cv2.imwrite が cv2.error を送出しても traceback クラッシュせず終了コード1（要求仕様 3.4）"""
        _install_render_stub(monkeypatch)

        def _raise(path, img):
            raise cv2.error("could not find a writer for the specified extension")

        monkeypatch.setattr(cv2, "imwrite", _raise)
        toml = _write_synth_toml(tmp_path)
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(tmp_path / "out")])
        assert rc == 1
        assert "保存に失敗" in capsys.readouterr().err

    def test_missing_camera_returns_1(self, tmp_path, capsys):
        """存在しないカメラ名で終了コード1・stderrに利用可能カメラ名一覧（要求3.1）

        select_camera が from render import より前なので render スタブ不要。
        """
        toml = _write_synth_toml(tmp_path)
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "no_such",
                   "--output-dir", str(tmp_path / "out")])
        assert rc == 1
        assert "camA" in capsys.readouterr().err  # 利用可能カメラ名一覧


# ========================================
# 実データ依存（skipif）
# ========================================

@pytest.mark.skipif(not _TOML.exists(), reason="実データTOMLが無い環境ではスキップ")
class TestRealData:
    def test_loads_five_cameras(self):
        """実データTOMLから5カメラが読め、各フィールド形状が正しいこと"""
        cams = load_cameras_toml(str(_TOML))
        assert len(cams) == 5
        for cam in cams.values():
            assert cam["K"].shape == (3, 3)
            assert cam["D"].shape[0] == 5
            assert cam["rvec"].shape == (3,)
            assert cam["tvec"].shape == (3,)
            assert cam["width"] == 1920
            assert cam["height"] == 1080

    def test_select_existing(self):
        cams = load_cameras_toml(str(_TOML))
        assert select_camera(cams, "cam41520554")["name"] == "cam41520554"

    def test_select_missing_raises(self):
        cams = load_cameras_toml(str(_TOML))
        with pytest.raises(ValueError, match="見つかりません"):
            select_camera(cams, "no_such_cam")

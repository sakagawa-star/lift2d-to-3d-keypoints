"""
feat-017: render_keypoints.py 全フレーム対応（連番PNG + MP4）のテスト

bpy/CUDA/c3d 非依存の純粋ロジック（フレーム範囲フィルタ・fps決定・連番PNG出力・
ffmpegコマンド構築・CLIパーサー）を合成データ／モックで検証する。
c3d/torch/gsplat は render_keypoints の関数内 import なので本テストでは import 不要。
実データ依存テストはファイル非存在時に skip する。
"""

import os
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (
    HALPE26_NAMES,
    _build_parser,
    main,
    start_ffmpeg,
)

_DATA = Path(__file__).parent.parent / "phase4" / "data" / "Blender"
_TOML = _DATA / "Config_scene.toml"

_SYNTH_TOML = """\
[camA]
name = "camA"
size = [1920, 1080]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _install_render_stub(monkeypatch, n_frames=3, point_rate=29.97):
    """render.py（torch top-level import）をダミー化し、CUDA依存の render_image と
    load_c3d_all_frames をスタブ化して、main を PNG/MP4 出力直前まで到達させる。

    n_frames フレーム分の Halpe26（全マーカー原点・有効）を frame_no=1..n_frames で返す。
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

    def _stub_load_c3d(path):
        labels = list(HALPE26_NAMES)
        frames_data = [{
            "frame_no": i + 1,
            "data": np.zeros((26, 3), dtype=np.float64),
            "residual": np.zeros(26, dtype=np.float64),
        } for i in range(n_frames)]
        return labels, frames_data, point_rate

    monkeypatch.setattr("render_keypoints.load_c3d_all_frames", _stub_load_c3d)


# ========================================
# CLIパーサー
# ========================================

class TestParser:
    def test_output_dir_and_new_flags(self):
        """--output-dir / --start-frame / --end-frame / --mp4 / --mp4-fps が定義される"""
        args = _build_parser().parse_args(
            ["a.ply", "b.toml", "c.c3d", "--camera", "camA",
             "--output-dir", "out", "--start-frame", "5", "--end-frame", "10",
             "--mp4", "--mp4-fps", "59.94"]
        )
        assert args.output_dir == "out"
        assert args.start_frame == 5
        assert args.end_frame == 10
        assert args.mp4 is True
        assert args.mp4_fps == pytest.approx(59.94)

    def test_mp4_fps_is_float(self):
        """--mp4-fps は float（小数を丸めず保持）"""
        args = _build_parser().parse_args(
            ["a.ply", "b.toml", "c.c3d", "--camera", "camA", "--mp4-fps", "29.97"]
        )
        assert isinstance(args.mp4_fps, float)

    def test_output_flag_removed(self):
        """旧 --output は廃止されており指定するとエラー"""
        with pytest.raises(SystemExit):
            _build_parser().parse_args(
                ["a.ply", "b.toml", "c.c3d", "--camera", "camA", "--output", "x.png"]
            )

    def test_defaults(self):
        """範囲・mp4 のデフォルトは None / False"""
        args = _build_parser().parse_args(
            ["a.ply", "b.toml", "c.c3d", "--camera", "camA"]
        )
        assert args.output_dir is None
        assert args.start_frame is None
        assert args.end_frame is None
        assert args.mp4 is False
        assert args.mp4_fps is None


# ========================================
# start_ffmpeg（ffmpeg依存をモック）
# ========================================

class TestStartFfmpeg:
    def test_missing_ffmpeg_raises(self, monkeypatch, tmp_path):
        """ffmpeg 不在で FileNotFoundError"""
        monkeypatch.setattr("shutil.which", lambda name: None)
        with pytest.raises(FileNotFoundError):
            start_ffmpeg(str(tmp_path), 1920, 1080, 29.97)

    def test_float_fps_passed_as_string(self, monkeypatch, tmp_path):
        """fps=29.97 が丸められず -r 引数に '29.97' として渡る"""
        captured = {}

        class _DummyProc:
            stdin = None
            stderr = None

        def _fake_run(cmd, capture_output, text):
            out = types.SimpleNamespace(stdout="h264_nvenc")
            return out

        def _fake_popen(cmd, stdin, stderr):
            captured["cmd"] = cmd
            return _DummyProc()

        monkeypatch.setattr("shutil.which", lambda name: "/usr/bin/ffmpeg")
        monkeypatch.setattr("subprocess.run", _fake_run)
        monkeypatch.setattr("subprocess.Popen", _fake_popen)

        proc, mp4_path = start_ffmpeg(str(tmp_path), 1920, 1080, 29.97)
        cmd = captured["cmd"]
        r_idx = cmd.index("-r")
        assert cmd[r_idx + 1] == "29.97"      # 丸めず小数保持
        s_idx = cmd.index("-s")
        assert cmd[s_idx + 1] == "1920x1080"
        assert mp4_path == os.path.join(str(tmp_path), "output.mp4")

    def test_nvenc_fallback_to_libx264(self, monkeypatch, tmp_path):
        """h264_nvenc 非対応環境では libx264 にフォールバック"""
        captured = {}

        def _fake_run(cmd, capture_output, text):
            return types.SimpleNamespace(stdout="libx264 only")

        def _fake_popen(cmd, stdin, stderr):
            captured["cmd"] = cmd
            return types.SimpleNamespace(stdin=None, stderr=None)

        monkeypatch.setattr("shutil.which", lambda name: "/usr/bin/ffmpeg")
        monkeypatch.setattr("subprocess.run", _fake_run)
        monkeypatch.setattr("subprocess.Popen", _fake_popen)

        start_ffmpeg(str(tmp_path), 640, 480, 30.0)
        cmd = captured["cmd"]
        cv_idx = cmd.index("-c:v")
        assert cmd[cv_idx + 1] == "libx264"


# ========================================
# main: 連番PNG出力（フレーム範囲・全フレーム）
# ========================================

class TestMainPngOutput:
    def test_all_frames_png(self, tmp_path, monkeypatch):
        """全フレーム（3枚）が frame_NNNNNN.png 連番で出力される"""
        _install_render_stub(monkeypatch, n_frames=3)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir)])
        assert rc == 0
        pngs = sorted(p.name for p in outdir.glob("*.png"))
        assert pngs == ["frame_000001.png", "frame_000002.png", "frame_000003.png"]

    def test_frame_range_filter(self, tmp_path, monkeypatch):
        """--start-frame/--end-frame で範囲内フレームのみ出力（両端含む）"""
        _install_render_stub(monkeypatch, n_frames=5)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir), "--start-frame", "2", "--end-frame", "4"])
        assert rc == 0
        pngs = sorted(p.name for p in outdir.glob("*.png"))
        assert pngs == ["frame_000002.png", "frame_000003.png", "frame_000004.png"]

    def test_frame_range_empty_returns_1(self, tmp_path, monkeypatch, capsys):
        """範囲に該当フレームが0件なら終了コード1・stderrにメッセージ"""
        _install_render_stub(monkeypatch, n_frames=3)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir), "--start-frame", "100"])
        assert rc == 1
        assert "該当するフレームがありません" in capsys.readouterr().err

    def test_mp4_missing_ffmpeg_returns_1(self, tmp_path, monkeypatch, capsys):
        """--mp4 で ffmpeg 不在なら終了コード1・PNGも未出力（ループ前に弾く）"""
        _install_render_stub(monkeypatch, n_frames=3)
        monkeypatch.setattr("shutil.which", lambda name: None)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir), "--mp4"])
        assert rc == 1
        assert "ffmpeg" in capsys.readouterr().err
        # ループ前に終了するためPNGは1枚も出ない
        assert list(outdir.glob("*.png")) == []


# ========================================
# main + ffmpeg をモックした両出力（PNG + MP4）
# ========================================

class TestMainMp4Output:
    def test_png_and_mp4_written(self, tmp_path, monkeypatch):
        """--mp4 時、連番PNGとMP4の両方が出力され、各フレームが ffmpeg stdin に書かれる"""
        _install_render_stub(monkeypatch, n_frames=3, point_rate=50.0)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"

        written = {"frames": 0, "fps_arg": None, "closed": False}

        class _FakeStdin:
            closed = False

            def write(self, data):
                written["frames"] += 1

            def close(self):
                self.closed = True
                written["closed"] = True

        class _FakeProc:
            def __init__(self):
                self.stdin = _FakeStdin()
                self.stderr = types.SimpleNamespace(read=lambda: b"")
                self.returncode = 0

            def wait(self):
                return 0

        def _fake_start_ffmpeg(output_dir, width, height, fps):
            written["fps_arg"] = fps
            return _FakeProc(), os.path.join(output_dir, "output.mp4")

        monkeypatch.setattr("render_keypoints.start_ffmpeg", _fake_start_ffmpeg)

        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir), "--mp4"])
        assert rc == 0
        # 3フレーム分PNGが出力され、ffmpeg にも3フレーム書き込まれた
        assert len(list(outdir.glob("*.png"))) == 3
        assert written["frames"] == 3
        assert written["closed"] is True
        # point_rate=50.0 が fps として使われる（--mp4-fps 未指定）
        assert written["fps_arg"] == pytest.approx(50.0)

    def test_mp4_fps_override(self, tmp_path, monkeypatch):
        """--mp4-fps 指定時はC3D rateより優先される"""
        _install_render_stub(monkeypatch, n_frames=1, point_rate=50.0)
        toml = _write_synth_toml(tmp_path)
        outdir = tmp_path / "out"
        captured = {}

        def _fake_start_ffmpeg(output_dir, width, height, fps):
            captured["fps"] = fps
            proc = types.SimpleNamespace(
                stdin=types.SimpleNamespace(closed=False,
                                            write=lambda d: None,
                                            close=lambda: None),
                stderr=types.SimpleNamespace(read=lambda: b""),
                returncode=0,
                wait=lambda: 0,
            )
            return proc, os.path.join(output_dir, "output.mp4")

        monkeypatch.setattr("render_keypoints.start_ffmpeg", _fake_start_ffmpeg)
        rc = main(["dummy.ply", toml, "dummy.c3d", "--camera", "camA",
                   "--output-dir", str(outdir), "--mp4", "--mp4-fps", "24.0"])
        assert rc == 0
        assert captured["fps"] == pytest.approx(24.0)


# ========================================
# 実データ依存（skipif）
# ========================================

@pytest.mark.skipif(not _TOML.exists(), reason="実データTOMLが無い環境ではスキップ")
class TestRealData:
    def test_load_all_frames_smoke(self):
        """実C3Dがあれば全フレーム読み込みが (labels, frames_data, rate) を返す"""
        from render_keypoints import load_c3d_all_frames
        c3d = _DATA / "keypoints.c3d"
        if not c3d.exists():
            pytest.skip("実C3Dが無い")
        labels, frames_data, rate = load_c3d_all_frames(str(c3d))
        assert isinstance(labels, list)
        assert len(frames_data) >= 1
        assert "frame_no" in frames_data[0]
        assert frames_data[0]["data"].shape[1] == 3
        assert isinstance(rate, float)

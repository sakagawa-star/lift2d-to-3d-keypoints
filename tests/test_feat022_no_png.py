"""
feat-022: render_keypoints.py --no-png オプション（MP4のみ出力）のテスト

GPU/ffmpeg/実データ非依存。design.md「テスト方針」のとおり、
パース・バリデーション系（1〜3）とフレームループ実体系（4〜7）を
合成TOML＋スタブ（render モジュール・render_image・load_c3d_all_frames・
start_ffmpeg・cv2.imwrite）で検証する。
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import HALPE26_NAMES, _build_parser, main

_SYNTH_TOML = """\
[camA]
name = "camA"
size = [64, 48]
matrix = [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""

N_FRAMES = 3


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _install_stubs(monkeypatch):
    """torch/CUDA/c3d/ffmpeg 依存を全て遮断し main をフレームループまで到達させる。

    Returns:
        (imwrite_mock, ffmpeg_stdin_mock): 呼び出し回数検証用モック
    """
    H, W = 48, 64

    # render モジュール（torch top-level import）をダミー化
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: {"means": "stub"}
    dummy.print_ply_summary = lambda path, gaussians: None
    monkeypatch.setitem(sys.modules, "render", dummy)

    # 背景レンダリング（全画素 3DGSなし扱い → キーポイントは隠れない）
    def _stub_render_image(gaussians, cam, near_plane,
                           background=(0.0, 0.0, 0.0), return_depth=False):
        bgr = np.zeros((H, W, 3), dtype=np.uint8)
        if return_depth:
            depth_map = np.full((H, W), 100.0, dtype=np.float32)
            alpha_map = np.zeros((H, W), dtype=np.float32)
            return bgr, depth_map, alpha_map
        return bgr

    monkeypatch.setattr("render_keypoints.render_image", _stub_render_image)

    # C3D（Halpe26 全マーカー原点・有効、frame_no=1..N_FRAMES）
    def _stub_load_c3d(path):
        labels = list(HALPE26_NAMES)
        frames_data = [{
            "frame_no": i + 1,
            "data": np.zeros((26, 3), dtype=np.float64),
            "residual": np.zeros(26, dtype=np.float64),
        } for i in range(N_FRAMES)]
        return labels, frames_data, 30.0

    monkeypatch.setattr("render_keypoints.load_c3d_all_frames", _stub_load_c3d)

    # ffmpeg プロセス（stdin.write を記録、正常終了）
    proc = MagicMock()
    proc.stdin.closed = False
    proc.returncode = 0

    def _stub_start_ffmpeg(output_dir, width, height, fps):
        return proc, f"{output_dir}/output.mp4"

    monkeypatch.setattr("render_keypoints.start_ffmpeg", _stub_start_ffmpeg)

    # cv2.imwrite（呼び出し回数記録、成功扱い）
    imwrite = MagicMock(return_value=True)
    monkeypatch.setattr("render_keypoints.cv2.imwrite", imwrite)

    return imwrite, proc.stdin.write


def _base_args(toml_path, out_dir):
    return ["a.ply", toml_path, "c.c3d", "--camera", "camA",
            "--output-dir", str(out_dir)]


# ========================================
# パース・バリデーション系（design.md テスト 1〜3）
# ========================================

class TestParserAndValidation:
    def test_no_png_flag_parsed(self):
        """--no-png を受理し args.no_png == True"""
        args = _build_parser().parse_args(
            ["a.ply", "b.toml", "c.c3d", "--camera", "camA", "--mp4", "--no-png"]
        )
        assert args.no_png is True

    def test_no_png_without_mp4_exits_2(self, capsys):
        """--no-png 単独指定は SystemExit(2)。重い処理（ファイル読み込み）より前に終了"""
        with pytest.raises(SystemExit) as excinfo:
            main(["nonexistent.ply", "nonexistent.toml", "nonexistent.c3d",
                  "--camera", "camA", "--no-png"])
        assert excinfo.value.code == 2
        err = capsys.readouterr().err
        assert "--mp4" in err and "--no-png" in err

    def test_no_png_default_false(self):
        """--no-png なしのデフォルトは False"""
        args = _build_parser().parse_args(
            ["a.ply", "b.toml", "c.c3d", "--camera", "camA"]
        )
        assert args.no_png is False


# ========================================
# フレームループ実体系（design.md テスト 4〜7）
# ========================================

class TestFrameLoopSkip:
    def test_no_png_skips_imwrite_and_writes_mp4(self, monkeypatch, tmp_path):
        """--no-png --mp4: imwrite 0回、ffmpeg 書き込みはフレーム数と同数（FR-001）"""
        imwrite, ffmpeg_write = _install_stubs(monkeypatch)
        toml = _write_synth_toml(tmp_path)
        rc = main(_base_args(toml, tmp_path / "out") + ["--mp4", "--no-png"])
        assert rc == 0
        assert imwrite.call_count == 0
        assert ffmpeg_write.call_count == N_FRAMES

    def test_without_no_png_writes_png(self, monkeypatch, tmp_path):
        """対照: --mp4 のみでは imwrite がフレーム数と同数呼ばれる"""
        imwrite, ffmpeg_write = _install_stubs(monkeypatch)
        toml = _write_synth_toml(tmp_path)
        rc = main(_base_args(toml, tmp_path / "out") + ["--mp4"])
        assert rc == 0
        assert imwrite.call_count == N_FRAMES
        assert ffmpeg_write.call_count == N_FRAMES

    def test_progress_lines_without_png_path(self, monkeypatch, tmp_path, capsys):
        """--no-png --mp4: 進捗行に .png が含まれず frame と -> mp4 を含む（FR-003）"""
        _install_stubs(monkeypatch)
        toml = _write_synth_toml(tmp_path)
        rc = main(_base_args(toml, tmp_path / "out") + ["--mp4", "--no-png"])
        assert rc == 0
        out = capsys.readouterr().out
        progress = [l for l in out.splitlines() if l.startswith("  [")]
        assert len(progress) == N_FRAMES
        for line in progress:
            assert ".png" not in line
            assert "frame" in line and "-> mp4" in line

    def test_existing_pngs_untouched(self, monkeypatch, tmp_path):
        """--no-png --mp4: 出力先の既存 frame_*.png を削除・上書きしない（受け入れ基準4）"""
        _install_stubs(monkeypatch)
        toml = _write_synth_toml(tmp_path)
        out_dir = tmp_path / "out"
        out_dir.mkdir()
        old = out_dir / "frame_000001.png"
        old.write_bytes(b"old-content")
        rc = main(_base_args(toml, out_dir) + ["--mp4", "--no-png"])
        assert rc == 0
        pngs = sorted(out_dir.glob("frame_*.png"))
        assert pngs == [old]
        assert old.read_bytes() == b"old-content"

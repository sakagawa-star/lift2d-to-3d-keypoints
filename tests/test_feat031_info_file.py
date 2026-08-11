"""feat-031 render_fps_video.py のレンダリング情報テキスト自動保存のテスト。

GPU・torch・gsplat 不要。純関数（format_frame_ranges / info_file_path /
build_info_text / write_info_file）と、_run_mp4_mode の完了時書き出し経路を
フェイクの torch・チャンク完成済みセットアップで検証する
（tests/test_feat030_parallel_render.py・tests/test_feat027_render_fps_video.py と
同じ流儀）。実GPU上の実挙動は手動テストで確認する。

実行:
  uv run pytest -v   （root環境。torch非依存のテストはそのまま走る）
"""

import argparse
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import render_fps_video  # noqa: E402

from render_fps_video import (  # noqa: E402
    build_info_text,
    format_frame_ranges,
    info_file_path,
    write_info_file,
)


# ========================================
# 1〜5: format_frame_ranges
# ========================================

def test_format_frame_ranges_empty():
    assert format_frame_ranges([]) == "なし"


def test_format_frame_ranges_single():
    assert format_frame_ranges([5]) == "5"


def test_format_frame_ranges_consecutive():
    assert format_frame_ranges([1, 2, 3]) == "1-3"


def test_format_frame_ranges_mixed():
    fids = list(range(1200, 1240)) + [35001] + list(range(47800, 47990))
    assert format_frame_ranges(fids) == "1200-1239, 35001, 47800-47989"


def test_format_frame_ranges_adjacent_singles():
    assert format_frame_ranges([1, 3, 5]) == "1, 3, 5"


# ========================================
# 6: info_file_path
# ========================================

def test_info_file_path():
    assert info_file_path("/a/b/test_fps.mp4") == "/a/b/test_fps_info.txt"


# ========================================
# 7: build_info_text（完全一致）
# ========================================

def test_build_info_text_full():
    text = build_info_text(
        video_name="test_fps.mp4",
        elapsed_sec=123.456,
        render_avg="0.500秒/フレーム",
        total_frames=1000,
        fps=30.0,
        nan_fids=[10, 11, 12, 13, 14, 100],
        degen_fids=[500],
        width=1920,
        height=1080,
    )
    expected = (
        "動画ファイル: test_fps.mp4\n"
        "総所要時間: 123.5秒\n"
        "描画フレーム平均: 0.500秒/フレーム\n"
        "総フレーム数: 1000\n"
        "フレームレート: 30.0\n"
        "NaN黒フレーム数: 6\n"
        "NaN黒フレーム番号: 10-14, 100\n"
        "縮退黒フレーム数: 1\n"
        "縮退黒フレーム番号: 500\n"
        "解像度: 1920x1080\n"
    )
    assert text == expected


# ========================================
# 8: build_info_text（0件・描画0フレーム）
# ========================================

def test_build_info_text_zero_black():
    text = build_info_text(
        video_name="v.mp4",
        elapsed_sec=1.0,
        render_avg="N/A（描画0フレーム）",
        total_frames=10,
        fps=30.0,
        nan_fids=[],
        degen_fids=[],
        width=64,
        height=64,
    )
    assert "NaN黒フレーム数: 0\n" in text
    assert "NaN黒フレーム番号: なし\n" in text
    assert "縮退黒フレーム数: 0\n" in text
    assert "縮退黒フレーム番号: なし\n" in text
    assert "描画フレーム平均: N/A（描画0フレーム）\n" in text


# ========================================
# 9: write_info_file（内容一致・上書き・.tmp が残らない）
# ========================================

def test_write_info_file_content_and_overwrite(tmp_path):
    path = tmp_path / "out_info.txt"
    path.write_text("old content", encoding="utf-8")

    write_info_file(str(path), "line1\nline2\n")

    assert path.read_text(encoding="utf-8") == "line1\nline2\n"
    assert not os.path.exists(str(path) + ".tmp")


# ========================================
# 10: write_info_file が durable_replace を使う
# ========================================

def test_write_info_file_uses_durable_replace(monkeypatch, tmp_path):
    calls = []

    def fake_durable_replace(tmp_path_arg, dst_path):
        calls.append((tmp_path_arg, dst_path))
        os.replace(tmp_path_arg, dst_path)

    monkeypatch.setattr(render_fps_video, "durable_replace", fake_durable_replace)

    path = tmp_path / "out_info.txt"
    write_info_file(str(path), "text\n")

    assert calls == [(str(path) + ".tmp", str(path))]
    assert path.read_text(encoding="utf-8") == "text\n"


# ========================================
# 11〜13: _run_mp4_mode 統合テスト
# ========================================

class _FakeCudaModule:
    @staticmethod
    def set_device(idx):
        pass


class _FakeTorchModule:
    cuda = _FakeCudaModule()


def _make_mp4_mode_setup(tmp_path, output_name="test_fps.mp4"):
    """全チャンク完成済みの _run_mp4_mode 呼び出しセットアップを構築する
    （design.md テスト設計「テスト11〜13の _run_mp4_mode 統合セットアップ」）。

    Returns:
        (args, output_path, frame_ids, intrinsics, viewmats, valid)
    """
    F = 10
    chunk_size = 5
    frame_ids = np.arange(F)
    valid = np.ones(F, dtype=bool)
    viewmats = np.zeros((F, 4, 4), dtype=np.float32)
    intrinsics = {"width": 64, "height": 64, "fx": 1.0, "fy": 1.0, "cx": 32.0, "cy": 32.0}

    ply_path = tmp_path / "scene.ply"
    ply_path.write_bytes(b"")
    npz_path = tmp_path / "in.npz"
    npz_path.write_bytes(b"")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text("", encoding="utf-8")

    args = argparse.Namespace(
        gpus=None, gpu=0, chunk_size=chunk_size, crf=18, preset="medium", fps=30.0,
        overwrite=False, keep_chunks=True,
        ply_path=str(ply_path), npz_path=str(npz_path), toml=str(toml_path), camera="cam",
    )

    output_path = str(tmp_path / output_name)
    chunk_dir = output_path + ".chunks"
    os.makedirs(chunk_dir)

    chunks = render_fps_video.plan_chunks(F, chunk_size, frame_ids)
    manifest = render_fps_video.build_manifest(
        args, F, int(frame_ids[0]), int(frame_ids[-1]),
        intrinsics["width"], intrinsics["height"],
    )
    render_fps_video.write_manifest(chunk_dir, manifest)
    for chunk in chunks:
        chunk_path = os.path.join(chunk_dir, render_fps_video.chunk_filename(chunk))
        with open(chunk_path, "wb"):
            pass

    return args, output_path, frame_ids, intrinsics, viewmats, valid


def test_mp4_mode_writes_info_on_success(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "torch", _FakeTorchModule())
    monkeypatch.setattr(render_fps_video, "verify_chunk_mp4", lambda path, expected: (True, ""))

    def fake_concat_chunks(chunk_dir, chunk_names, output_path):
        with open(output_path, "wb"):
            pass

    monkeypatch.setattr(render_fps_video, "concat_chunks", fake_concat_chunks)

    args, output_path, frame_ids, intrinsics, viewmats, valid = _make_mp4_mode_setup(tmp_path)

    rc = render_fps_video._run_mp4_mode(
        args, output_path, frame_ids, intrinsics, viewmats, valid, [3, 4], [7]
    )

    assert rc == 0
    info_path = render_fps_video.info_file_path(output_path)
    assert os.path.exists(info_path)
    text = open(info_path, encoding="utf-8").read()
    assert "動画ファイル: test_fps.mp4\n" in text
    assert "描画フレーム平均: N/A（描画0フレーム）\n" in text
    assert "総フレーム数: 10\n" in text
    assert "フレームレート: 30.0\n" in text
    assert "NaN黒フレーム数: 2\n" in text
    assert "NaN黒フレーム番号: 3-4\n" in text
    assert "縮退黒フレーム数: 1\n" in text
    assert "縮退黒フレーム番号: 7\n" in text
    assert "解像度: 64x64\n" in text


def test_mp4_mode_no_info_on_concat_failure(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "torch", _FakeTorchModule())
    monkeypatch.setattr(render_fps_video, "verify_chunk_mp4", lambda path, expected: (True, ""))

    def fake_concat_chunks_fail(chunk_dir, chunk_names, output_path):
        raise RuntimeError("concat failed")

    monkeypatch.setattr(render_fps_video, "concat_chunks", fake_concat_chunks_fail)

    args, output_path, frame_ids, intrinsics, viewmats, valid = _make_mp4_mode_setup(tmp_path)

    rc = render_fps_video._run_mp4_mode(
        args, output_path, frame_ids, intrinsics, viewmats, valid, [3], [7]
    )

    assert rc == 1
    info_path = render_fps_video.info_file_path(output_path)
    assert not os.path.exists(info_path)


def test_mp4_mode_rc1_on_info_write_failure(tmp_path, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "torch", _FakeTorchModule())
    monkeypatch.setattr(render_fps_video, "verify_chunk_mp4", lambda path, expected: (True, ""))

    def fake_concat_chunks(chunk_dir, chunk_names, output_path):
        with open(output_path, "wb"):
            pass

    monkeypatch.setattr(render_fps_video, "concat_chunks", fake_concat_chunks)

    def fake_write_info_file(path, text):
        raise OSError("disk full")

    monkeypatch.setattr(render_fps_video, "write_info_file", fake_write_info_file)

    args, output_path, frame_ids, intrinsics, viewmats, valid = _make_mp4_mode_setup(tmp_path)

    rc = render_fps_video._run_mp4_mode(
        args, output_path, frame_ids, intrinsics, viewmats, valid, [], []
    )

    assert rc == 1
    err = capsys.readouterr().err
    assert "情報ファイルの書き出しに失敗しました" in err

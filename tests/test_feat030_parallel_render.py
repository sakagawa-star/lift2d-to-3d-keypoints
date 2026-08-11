"""feat-030 render_fps_video.py の並列モード（--gpus）のテスト。

GPU・torch・gsplat 不要。純ロジック（--gpus パース・CLI/YAML検証・
select_pending_chunks・cleanup_tmp_files・_dispatch_loop）を、フェイクの
ワーカー（procs）・キュー（queue.Queue）を注入して検証する
（tests/test_feat027_render_fps_video.py・tests/test_feat029_config_yaml.py と
同じ流儀）。実GPU上の実挙動（ワーカープロセスの実際のkill等）は手動テストで確認する。

実行:
  uv run pytest -v   （root環境。torch非依存のテストはそのまま走る）
"""

import argparse
import os
import queue
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import render_fps_video  # noqa: E402（テスト14でstart_ffmpeg_chunkをモンキーパッチするため）

from render_fps_video import (  # noqa: E402
    HEAD_JOINT_NAMES,
    _build_parser,
    _dispatch_loop,
    _gpus_list,
    cleanup_tmp_files,
    main,
    parse_args_with_config,
    parse_config_yaml,
    select_pending_chunks,
)


# === 共通ヘルパ（feat-027テストと同一配置の「標準姿勢」） ===

_DEFAULT_POSE = {
    "LEye": (-1.0, 0.0, 0.0), "REye": (1.0, 0.0, 0.0),
    "LEar": (-1.0, 0.0, 0.0), "REar": (1.0, 0.0, 0.0),
    "Nose": (0.0, 1.0, 0.0), "Head": (0.0, 0.0, 1.0), "Neck": (0.0, 0.0, 0.0),
}


def _pose_row() -> np.ndarray:
    return np.array([_DEFAULT_POSE[name] for name in HEAD_JOINT_NAMES], dtype=np.float64)


def _write_npz(path, rows: list[np.ndarray], frame_ids) -> None:
    x3d_world = np.stack(rows, axis=0)
    np.savez(
        str(path),
        x3d_world=x3d_world,
        frame_ids=np.asarray(frame_ids, dtype=np.int64),
        joint_names=np.array(HEAD_JOINT_NAMES, dtype="<U16"),
    )


_TOML_TEMPLATE = """[cam01]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
size = [{w}, {h}]
"""

_BASE_CLI = ["dummy.ply", "dummy.npz", "--toml", "dummy.toml", "--camera", "cam01"]

# 必須5項目を満たす最小の設定YAML断片（dummy パス。ファイル存在チェックより前の
# 検証を確認するテストで使う）。
_REQUIRED_YAML = (
    "ply_path: dummy.ply\n"
    "npz_path: dummy.npz\n"
    "toml: dummy.toml\n"
    "camera: cam01\n"
    "fps: 30\n"
)


# ========================================
# 1: test_gpus_list_valid
# ========================================

def test_gpus_list_valid():
    assert _gpus_list("0") == [0]
    assert _gpus_list("0,0") == [0, 0]
    assert _gpus_list("0,1,2") == [0, 1, 2]
    assert _gpus_list(" 0 , 1 ") == [0, 1]


# ========================================
# 2: test_gpus_list_invalid
# ========================================

@pytest.mark.parametrize("value", ["", "0,", "a", "-1", "0,,1"])
def test_gpus_list_invalid(value):
    with pytest.raises(argparse.ArgumentTypeError):
        _gpus_list(value)


# ========================================
# 3: test_cli_gpu_and_gpus_exclusive_exits_2
# ========================================

def test_cli_gpu_and_gpus_exclusive_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--gpu", "0", "--gpus", "0,0"])
    assert exc_info.value.code == 2


# ========================================
# 4: test_cli_gpus_with_still_range_exits_2 / test_cli_gpus_with_dump_poses_exits_2
# ========================================

def test_cli_gpus_with_still_range_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--gpus", "0,0",
                          "--still-range", "1", "2", "--still-dir", "out"])
    assert exc_info.value.code == 2


def test_cli_gpus_with_dump_poses_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--gpus", "0,0", "--dump-poses", "poses.json"])
    assert exc_info.value.code == 2


# ========================================
# 5: test_cli_gpu_default_normalized
# ========================================

def test_cli_gpu_default_normalized():
    # --gpu/--gpus とも未指定 → parse_args_with_config 内の正規化で args.gpu == 0
    # になる経路を、dump-poses モードの必須項目のみで確認する（ファイルI/O不要）。
    args = parse_args_with_config(_BASE_CLI + ["--fps", "30", "--dump-poses", "poses.json"])
    assert args.gpu == 0
    assert args.gpus is None


# ========================================
# 6: test_yaml_gpus_key
# ========================================

def test_yaml_gpus_key(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "gpus: 0,1\n")
    result = parse_config_yaml(str(cfg), _build_parser())
    assert result["gpus"] == [0, 1]


# ========================================
# 7: test_yaml_gpu_plus_cli_gpus_exclusive_exits_2
# ========================================

def test_yaml_gpu_plus_cli_gpus_exclusive_exits_2(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "gpu: 0\n")
    with pytest.raises(SystemExit) as exc_info:
        main(["--config", str(cfg), "--gpus", "0,0"])
    assert exc_info.value.code == 2


# ========================================
# 8: test_gpus_out_of_range_exits_1
# ========================================

class _FakeCudaModule:
    @staticmethod
    def device_count():
        return 1


class _FakeTorchModule:
    cuda = _FakeCudaModule()


def test_gpus_out_of_range_exits_1(tmp_path, monkeypatch, capsys):
    pytest.importorskip("tomli")
    monkeypatch.setitem(sys.modules, "torch", _FakeTorchModule())

    ply_path = tmp_path / "scene.ply"
    ply_path.write_bytes(b"")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))
    npz_path = tmp_path / "in.npz"
    _write_npz(npz_path, [_pose_row()], np.array([1000]))

    rc = main([
        str(ply_path), str(npz_path), "--toml", str(toml_path), "--camera", "cam01",
        "--fps", "30", "--gpus", "0,5",
    ])
    assert rc == 1

    err = capsys.readouterr().err
    assert "範囲外" in err
    assert "5" in err


# ========================================
# 9: test_select_pending_chunks
# ========================================

def test_select_pending_chunks():
    chunks = [{"index": i} for i in range(5)]
    pending = select_pending_chunks(chunks, {1, 3})
    assert [c["index"] for c in pending] == [0, 2, 4]


# ========================================
# 10: test_cleanup_tmp_files
# ========================================

def test_cleanup_tmp_files(tmp_path):
    (tmp_path / "chunk_00000_00000000_00000004.mp4.tmp").write_bytes(b"x")
    (tmp_path / "chunk_00001_00000005_00000009.mp4.tmp").write_bytes(b"y")
    (tmp_path / "chunk_00000_00000000_00000004.mp4").write_bytes(b"z")
    (tmp_path / "manifest.json").write_text("{}")

    cleanup_tmp_files(str(tmp_path))

    remaining = sorted(os.listdir(tmp_path))
    assert remaining == ["chunk_00000_00000000_00000004.mp4", "manifest.json"]


# ========================================
# 11〜13: _dispatch_loop（フェイク procs + queue.Queue）
# ========================================

class _FakeProc:
    """.exitcode / .is_alive() / .terminate() / .join(timeout) を持つ最小フェイク。"""

    def __init__(self, exitcode=None, alive=True):
        self.exitcode = exitcode
        self._alive = alive
        self.terminate_called = 0
        self.kill_called = 0
        self.join_calls: list = []

    def is_alive(self):
        return self._alive

    def terminate(self):
        self.terminate_called += 1
        self._alive = False

    def kill(self):
        self.kill_called += 1
        self._alive = False

    def join(self, timeout=None):
        self.join_calls.append(timeout)


_CHUNKS_2 = [
    {"index": 0, "i0": 0, "i1": 3, "fid0": 100, "fid1": 102},
    {"index": 1, "i0": 3, "i1": 6, "fid0": 103, "fid1": 105},
]


def test_dispatch_loop_all_done(capsys):
    result_q = queue.Queue()
    result_q.put(("done", 0, 0, 3, 1.5))
    result_q.put(("done", 1, 1, 3, 2.0))
    procs = [_FakeProc(exitcode=0, alive=False), _FakeProc(exitcode=0, alive=False)]

    rc, total_rendered, total_sec = _dispatch_loop(
        procs, result_q, 2, _CHUNKS_2, "dummy_chunk_dir"
    )

    assert rc == 0
    assert total_rendered == 6
    assert total_sec == pytest.approx(3.5)

    out = capsys.readouterr().out
    assert "[W0] チャンク 00000 (fid 100-102) 完了: 描画 3/3 フレーム, 1.5秒" in out
    assert "[W1] チャンク 00001 (fid 103-105) 完了: 描画 3/3 フレーム, 2.0秒" in out


def test_dispatch_loop_error_aborts(tmp_path, capsys):
    chunk_dir = tmp_path / "chunks"
    chunk_dir.mkdir()
    (chunk_dir / "chunk_00000_00000100_00000102.mp4.tmp").write_bytes(b"x")
    (chunk_dir / "chunk_00001_00000103_00000105.mp4").write_bytes(b"done")

    result_q = queue.Queue()
    result_q.put(("error", 0, 0, "RuntimeError: ffmpeg終了コード 1"))
    procs = [_FakeProc(exitcode=None, alive=True), _FakeProc(exitcode=None, alive=True)]

    rc, total_rendered, total_sec = _dispatch_loop(
        procs, result_q, 2, _CHUNKS_2, str(chunk_dir)
    )

    assert rc == 1
    assert total_rendered == 0
    assert all(p.terminate_called == 1 for p in procs)

    remaining = sorted(os.listdir(chunk_dir))
    assert remaining == ["chunk_00001_00000103_00000105.mp4"]

    err = capsys.readouterr().err
    assert "ワーカー W0" in err
    assert "00000" in err


def test_dispatch_loop_dead_worker_aborts(tmp_path, capsys):
    chunk_dir = tmp_path / "chunks"
    chunk_dir.mkdir()
    chunks = [{"index": 0, "i0": 0, "i1": 3, "fid0": 100, "fid1": 102}]
    result_q: queue.Queue = queue.Queue()
    procs = [_FakeProc(exitcode=1, alive=False)]

    rc, total_rendered, total_sec = _dispatch_loop(
        procs, result_q, 1, chunks, str(chunk_dir)
    )

    assert rc == 1
    assert total_rendered == 0

    err = capsys.readouterr().err
    assert "ワーカー W0" in err
    assert "exit code 1" in err


# ========================================
# 14: test_render_chunk_keyboardinterrupt_closes_stdin_then_kills
# ========================================

class _KeyboardInterruptStdin:
    """render_chunk の proc.stdin フェイク。write() で KeyboardInterrupt を送出する。"""

    def __init__(self):
        self.closed = False
        self.close_called = 0

    def write(self, buf):
        raise KeyboardInterrupt

    def close(self):
        self.close_called += 1
        self.closed = True


class _FakeFfmpegProc:
    """§4.7 の後始末順序（stdin close → terminate → wait(timeout=10) →
    [TimeoutExpiredなら kill → wait]）を検証するフェイク proc。

    calls に terminate/wait/kill の呼び出し順を記録する。
    wait_timeout_count 回だけ subprocess.TimeoutExpired を送出してから成功する。
    """

    def __init__(self, wait_timeout_count=0):
        self.stdin = _KeyboardInterruptStdin()
        self.stderr = None
        self.returncode = 0
        self.calls: list = []
        self._wait_timeout_count = wait_timeout_count

    def terminate(self):
        self.calls.append("terminate")

    def wait(self, timeout=None):
        self.calls.append(("wait", timeout))
        if self._wait_timeout_count > 0:
            self._wait_timeout_count -= 1
            raise subprocess.TimeoutExpired(cmd=["ffmpeg"], timeout=timeout)
        return 0

    def kill(self):
        self.calls.append("kill")


_CHUNK_1F = {"index": 0, "i0": 0, "i1": 1, "fid0": 100, "fid1": 100}

_CONFIG_1F = {
    "width": 4, "height": 4, "fx": 1.0, "fy": 1.0, "cx": 2.0, "cy": 2.0,
    "fps": 30, "crf": 18, "preset": "medium", "chunk_dir": "dummy_chunk_dir",
    "black_frame": b"\x00" * (4 * 4 * 3),
}


class _FakeTorchModuleForRenderChunk:
    """render_chunk 冒頭の `import torch` 用フェイク。全フレーム黒のチャンクを
    使うため、torch.zeros()（背景色生成）以外は呼ばれない想定。"""

    float32 = "float32"

    @staticmethod
    def zeros(*args, **kwargs):
        return "background"

    @staticmethod
    def no_grad():
        raise AssertionError("全フレーム黒のため呼ばれないはず")


class _FakeRenderModule:
    """render_chunk の `from render import render_frame` 用フェイク。"""

    @staticmethod
    def render_frame(*args, **kwargs):
        raise AssertionError("全フレーム黒のため呼ばれないはず")


def test_render_chunk_keyboardinterrupt_closes_stdin_then_kills(monkeypatch):
    monkeypatch.setitem(sys.modules, "torch", _FakeTorchModuleForRenderChunk())
    monkeypatch.setitem(sys.modules, "render", _FakeRenderModule())

    valid = np.array([False])
    viewmats = np.zeros((1, 4, 4), dtype=np.float32)

    # (a)(c): terminate 後に wait が即座に成功するケース
    proc_ok = _FakeFfmpegProc(wait_timeout_count=0)
    monkeypatch.setattr(render_fps_video, "start_ffmpeg_chunk", lambda *a, **k: proc_ok)
    with pytest.raises(KeyboardInterrupt):
        render_fps_video.render_chunk(_CHUNK_1F, viewmats, valid, _CONFIG_1F, None)
    assert proc_ok.stdin.close_called == 1
    assert proc_ok.stdin.closed is True
    assert proc_ok.calls == ["terminate", ("wait", 10)]

    # (b): wait が TimeoutExpired を送出 → kill → wait のエスカレーション
    proc_kill = _FakeFfmpegProc(wait_timeout_count=1)
    monkeypatch.setattr(render_fps_video, "start_ffmpeg_chunk", lambda *a, **k: proc_kill)
    with pytest.raises(KeyboardInterrupt):
        render_fps_video.render_chunk(_CHUNK_1F, viewmats, valid, _CONFIG_1F, None)
    assert proc_kill.stdin.close_called == 1
    assert proc_kill.calls == ["terminate", ("wait", 10), "kill", ("wait", None)]

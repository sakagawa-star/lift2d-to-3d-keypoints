"""feat-028 filter_npz.py のテスト。

実行（phase4 venv + pytest。ルート venv には scipy がないため）:
  uv run --project phase4 --with pytest pytest tests/test_feat028_filter_npz.py -v
"""

import os
import sys

import numpy as np
import pytest

pytest.importorskip("scipy")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import filter_npz  # noqa: E402
from filter_npz import (  # noqa: E402
    MIN_FILTER_SAMPLES, filter_joint, filter_all_joints, load_npz_keypoints,
    lowpass_filtfilt, split_segments, main,
)

FPS = 100.0
CUTOFF = 2.0


def _noisy_walk(n, seed=0):
    rng = np.random.default_rng(seed)
    return np.cumsum(rng.normal(0, 1, (n, 3)), axis=0)


# === FR-002: フィルタ特性 ===

def test_constant_series_preserved():
    series = np.full((50, 3), 1.23456)
    out = lowpass_filtfilt(series, CUTOFF, FPS)
    np.testing.assert_allclose(out, series, atol=1e-6)


def test_lowfreq_preserved_highfreq_attenuated():
    t = np.arange(1000) / FPS
    low = np.sin(2 * np.pi * (CUTOFF / 10) * t)     # 0.2 Hz
    high = np.sin(2 * np.pi * (CUTOFF * 5) * t)      # 10 Hz < Nyquist(50)
    out_low = lowpass_filtfilt(low, CUTOFF, FPS)
    out_high = lowpass_filtfilt(high, CUTOFF, FPS)
    # 端点のパディング影響を避けて中央部で評価
    mid = slice(100, 900)
    assert abs(out_low[mid].max() - 1.0) < 0.05          # 低周波は振幅保存（5%以内）
    assert np.abs(out_high[mid]).max() < 0.10            # 高周波は10%未満に減衰
    # ゼロ位相: 低周波のピーク位置がずれない（許容1フレーム）
    assert abs(int(np.argmax(out_low[mid])) - int(np.argmax(low[mid]))) <= 1


# === FR-003: 無効サンプル・セグメント ===

def test_all_valid_joint_filtered():
    x3d_j = _noisy_walk(100)
    out, skipped = filter_joint(x3d_j, CUTOFF, FPS, max_gap=10)
    assert skipped == []
    assert not np.allclose(out, x3d_j)   # 全フレーム平滑化される


def test_gap_interpolation_and_nan_kept():
    x3d_j = _noisy_walk(100)
    x3d_j[40:43] = np.nan   # ギャップ長3（max_gap以下）
    out, skipped = filter_joint(x3d_j, CUTOFF, FPS, max_gap=10)
    assert skipped == []
    valid = np.isfinite(x3d_j).all(axis=1)
    assert not np.allclose(out[valid], x3d_j[valid])              # 有効位置は平滑化
    assert np.all(np.isnan(out[~valid]))                          # 無効位置は NaN のまま


def test_long_gap_splits_segments():
    x3d_j = _noisy_walk(100)
    x3d_j[40:60] = np.nan   # ギャップ長20 > max_gap=10 → セグメント分割
    out1, _ = filter_joint(x3d_j, CUTOFF, FPS, max_gap=10)
    # 後半セグメントの値を大きく変えても前半セグメントの結果は不変（独立フィルタ）
    x3d_j2 = x3d_j.copy()
    x3d_j2[60:] += 1000.0
    out2, _ = filter_joint(x3d_j2, CUTOFF, FPS, max_gap=10)
    np.testing.assert_array_equal(out1[:40], out2[:40])


def test_edge_nan_untouched():
    x3d_j = _noisy_walk(100)
    x3d_j[:5] = np.nan
    x3d_j[-5:] = np.nan
    out, skipped = filter_joint(x3d_j, CUTOFF, FPS, max_gap=10)
    assert skipped == []
    assert np.all(np.isnan(out[:5]))
    assert np.all(np.isnan(out[-5:]))
    assert not np.allclose(out[5:-5], x3d_j[5:-5])


def test_short_segment_skipped():
    n = MIN_FILTER_SAMPLES - 1   # 長さ9のセグメント
    x3d_j = _noisy_walk(n)
    out, skipped = filter_joint(x3d_j, CUTOFF, FPS, max_gap=10)
    assert skipped == [(0, n - 1)]
    np.testing.assert_array_equal(out, x3d_j)


# === FR-001/004/005: NPZ入出力・CLI ===

JOINT_NAMES = ["Nose", "Neck", "Head"]


def _make_npz(path, F=60, J=None, seed=1, x3d_dtype=np.float32, extras=None,
              frame_ids=None, joint_names=None):
    if J is None:
        J = len(JOINT_NAMES)
    rng = np.random.default_rng(seed)
    x3d_world = rng.normal(0, 1, (F, J, 3)).astype(x3d_dtype)
    if frame_ids is None:
        frame_ids = np.arange(F, dtype=np.int64)
    if joint_names is None:
        joint_names = np.array(JOINT_NAMES[:J], dtype="<U16")
    kwargs = dict(extras) if extras else {}
    np.savez(str(path), x3d_world=x3d_world, frame_ids=frame_ids,
              joint_names=joint_names, **kwargs)
    return x3d_world, frame_ids, joint_names


def test_roundtrip_npz(tmp_path):
    src = tmp_path / "in.npz"
    dst = tmp_path / "out.npz"
    x3d_world, frame_ids, joint_names = _make_npz(src, F=60)
    assert main([str(src), "--fps", "30", "--output", str(dst)]) == 0

    d_in = np.load(str(src))
    d_out = np.load(str(dst))
    assert set(d_out.files) == set(d_in.files)
    assert d_out["x3d_world"].shape == x3d_world.shape
    np.testing.assert_array_equal(d_out["frame_ids"], frame_ids)
    np.testing.assert_array_equal(d_out["joint_names"], joint_names)
    invalid_in = ~np.isfinite(x3d_world)
    invalid_out = ~np.isfinite(d_out["x3d_world"])
    np.testing.assert_array_equal(invalid_out, invalid_in)


def test_extras_passthrough(tmp_path):
    src = tmp_path / "in.npz"
    dst = tmp_path / "out.npz"
    F = 60
    pnp_ok = np.zeros(F, dtype=bool)
    pnp_ok[::2] = True
    coord_system = np.array("world", dtype="<U16")
    _make_npz(src, F=F, extras={"pnp_ok": pnp_ok, "coord_system": coord_system})
    assert main([str(src), "--fps", "30", "--output", str(dst)]) == 0

    d_out = np.load(str(dst))
    np.testing.assert_array_equal(d_out["pnp_ok"], pnp_ok)
    assert str(d_out["coord_system"]) == "world"


def test_dtype_preserved(tmp_path):
    src = tmp_path / "in.npz"
    dst = tmp_path / "out.npz"
    _make_npz(src, F=60, x3d_dtype=np.float32)
    assert main([str(src), "--fps", "30", "--output", str(dst)]) == 0

    d_out = np.load(str(dst))
    assert d_out["x3d_world"].dtype == np.float32


def test_output_same_as_input_rejected(tmp_path):
    src = tmp_path / "in.npz"
    _make_npz(src)
    assert main([str(src), "--fps", "30", "--output", str(src)]) == 1


def test_output_symlink_to_input_rejected(tmp_path):
    real = tmp_path / "real.npz"
    _make_npz(real)
    link = tmp_path / "link.npz"
    link.symlink_to(real)
    assert main([str(link), "--fps", "30", "--output", str(real)]) == 1


def test_cutoff_validation(tmp_path):
    src = tmp_path / "in.npz"
    _make_npz(src)
    assert main([str(src), "--fps", "30", "--cutoff", "0"]) == 1
    assert main([str(src), "--fps", "30", "--cutoff", "-1"]) == 1
    assert main([str(src), "--fps", "30", "--cutoff", "15.0"]) == 1   # == Nyquist(30/2)


def test_fps_validation(tmp_path):
    src = tmp_path / "in.npz"
    _make_npz(src)
    assert main([str(src), "--fps", "0"]) == 1
    assert main([str(src), "--fps", "-1"]) == 1


def test_missing_key_rejected(tmp_path):
    path = tmp_path / "missing.npz"
    F, J = 60, len(JOINT_NAMES)
    rng = np.random.default_rng(1)
    x3d_world = rng.normal(0, 1, (F, J, 3)).astype(np.float32)
    frame_ids = np.arange(F, dtype=np.int64)
    # joint_names を欠落させる
    np.savez(str(path), x3d_world=x3d_world, frame_ids=frame_ids)
    assert main([str(path), "--fps", "30"]) == 1


def test_nonconsecutive_frame_ids_rejected(tmp_path):
    path = tmp_path / "nonconsecutive.npz"
    F = 60
    frame_ids = np.arange(F, dtype=np.int64)
    frame_ids[10] = 999   # 連番を崩す
    _make_npz(path, F=F, frame_ids=frame_ids)
    assert main([str(path), "--fps", "30"]) == 1


def test_noninteger_frame_ids_rejected(tmp_path):
    F = 60

    path_float = tmp_path / "float_ids.npz"
    _make_npz(path_float, F=F, frame_ids=np.arange(F, dtype=np.float64))
    assert main([str(path_float), "--fps", "30"]) == 1

    path_str = tmp_path / "str_ids.npz"
    _make_npz(path_str, F=F, frame_ids=np.array([str(i) for i in range(F)]))
    assert main([str(path_str), "--fps", "30"]) == 1


def test_bad_extension_rejected(tmp_path):
    src = tmp_path / "in.npz"
    _make_npz(src)
    assert main([str(src), "--fps", "30", "--output", str(tmp_path / "out.txt")]) == 1

"""feat-020 filter_c3d.py のテスト。

実行（phase4 venv + pytest。ルート venv には scipy/c3d がないため）:
  uv run --project phase4 --with pytest pytest tests/test_filter_c3d.py -v
"""

import os
import sys

import numpy as np
import pytest

pytest.importorskip("scipy")
pytest.importorskip("c3d")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import filter_c3d  # noqa: E402
from filter_c3d import (  # noqa: E402
    MIN_FILTER_SAMPLES, filter_marker, load_c3d, lowpass_filtfilt,
    split_segments, write_c3d, main,
)

RATE = 100.0
CUTOFF = 2.0


# === FR-002: フィルタ特性 ===

def test_constant_series_preserved():
    series = np.full((50, 3), 123.456)
    out = lowpass_filtfilt(series, CUTOFF, RATE)
    np.testing.assert_allclose(out, series, atol=1e-6)


def test_lowfreq_preserved_highfreq_attenuated():
    t = np.arange(1000) / RATE
    low = np.sin(2 * np.pi * (CUTOFF / 10) * t)     # 0.2 Hz
    high = np.sin(2 * np.pi * (CUTOFF * 5) * t)     # 10 Hz < Nyquist(50)
    out_low = lowpass_filtfilt(low, CUTOFF, RATE)
    out_high = lowpass_filtfilt(high, CUTOFF, RATE)
    # 端点のパディング影響を避けて中央部で評価
    mid = slice(100, 900)
    assert abs(out_low[mid].max() - 1.0) < 0.05          # 低周波は振幅保存（5%以内）
    assert np.abs(out_high[mid]).max() < 0.10            # 高周波は10%未満に減衰
    # ゼロ位相: 低周波のピーク位置がずれない（許容1フレーム）
    assert abs(int(np.argmax(out_low[mid])) - int(np.argmax(low[mid]))) <= 1


# === FR-003: 無効サンプル・セグメント ===

def _noisy_walk(n, seed=0):
    rng = np.random.default_rng(seed)
    return np.cumsum(rng.normal(0, 1, (n, 3)), axis=0)


def test_split_segments():
    valid = np.ones(20, dtype=bool)
    valid[5:8] = False   # ギャップ長3
    assert split_segments(valid, max_gap=3) == [(0, 19)]
    assert split_segments(valid, max_gap=2) == [(0, 4), (8, 19)]
    assert split_segments(np.zeros(5, dtype=bool), max_gap=10) == []


def test_gap_interpolation_and_invalid_kept():
    points = _noisy_walk(100)
    residuals = np.zeros(100)
    residuals[40:43] = -1.0   # ギャップ長3（max_gap以下）
    out, skipped = filter_marker(points, residuals, CUTOFF, RATE, max_gap=10)
    assert skipped == []
    valid = residuals >= 0
    assert not np.allclose(out[valid], points[valid])            # 有効位置は平滑化
    np.testing.assert_array_equal(out[~valid], points[~valid])   # 無効位置は入力のまま


def test_long_gap_splits_segments():
    points = _noisy_walk(100)
    residuals = np.zeros(100)
    residuals[40:60] = -1.0   # ギャップ長20 > max_gap=10 → セグメント分割
    out1, _ = filter_marker(points, residuals, CUTOFF, RATE, max_gap=10)
    # 後半セグメントの値を大きく変えても前半セグメントの結果は不変（独立フィルタ）
    points2 = points.copy()
    points2[60:] += 1000.0
    out2, _ = filter_marker(points2, residuals, CUTOFF, RATE, max_gap=10)
    np.testing.assert_array_equal(out1[:40], out2[:40])


def test_edge_invalid_untouched():
    points = _noisy_walk(100)
    residuals = np.zeros(100)
    residuals[:5] = -1.0
    residuals[-5:] = -1.0
    out, skipped = filter_marker(points, residuals, CUTOFF, RATE, max_gap=10)
    assert skipped == []
    np.testing.assert_array_equal(out[:5], points[:5])
    np.testing.assert_array_equal(out[-5:], points[-5:])
    assert not np.allclose(out[5:-5], points[5:-5])


def test_short_segment_skipped():
    n = MIN_FILTER_SAMPLES - 1   # 長さ9のセグメント
    points = _noisy_walk(n)
    residuals = np.zeros(n)
    out, skipped = filter_marker(points, residuals, CUTOFF, RATE, max_gap=10)
    assert skipped == [(0, n - 1)]
    np.testing.assert_array_equal(out, points)


# === FR-001/004/005: C3D入出力・CLI ===

LABELS = ["Nose", "Neck", "Head"]


def _make_c3d(path, F=60, rate=30.0, first_frame=1, seed=1):
    rng = np.random.default_rng(seed)
    points = rng.normal(0, 100, (F, len(LABELS), 3))
    residuals = np.zeros((F, len(LABELS)))
    write_c3d(str(path), points, residuals, LABELS, rate, first_frame)
    return points, residuals


def test_roundtrip_c3d(tmp_path):
    src = tmp_path / "in.c3d"
    dst = tmp_path / "out.c3d"
    _make_c3d(src, F=60, rate=30.0, first_frame=1)
    assert main([str(src), "--output", str(dst)]) == 0
    points, residuals, labels, rate, first_frame = load_c3d(str(dst))
    assert points.shape == (60, 3, 3)
    assert labels == LABELS
    assert rate == 30.0
    assert first_frame == 1


def test_output_same_as_input_rejected(tmp_path):
    src = tmp_path / "in.c3d"
    _make_c3d(src)
    assert main([str(src), "--output", str(src)]) == 1


def test_output_symlink_to_input_rejected(tmp_path):
    real = tmp_path / "real.c3d"
    _make_c3d(real)
    link = tmp_path / "link.c3d"
    link.symlink_to(real)
    assert main([str(link), "--output", str(real)]) == 1


def test_cutoff_validation(tmp_path):
    src = tmp_path / "in.c3d"
    _make_c3d(src, rate=30.0)
    assert main([str(src), "--cutoff", "0"]) == 1
    assert main([str(src), "--cutoff", "-1"]) == 1
    assert main([str(src), "--cutoff", "15.0"]) == 1   # == Nyquist(30/2)
    assert main([str(src), "--max-gap", "0"]) == 1


def test_rate_conflict_rejected(tmp_path):
    src = tmp_path / "in.c3d"
    _make_c3d(src, rate=30.0)
    assert main([str(src), "--rate", "60.0"]) == 1


def test_wrong_units_rejected(tmp_path):
    import c3d
    path = tmp_path / "in.c3d"
    writer = c3d.Writer(point_rate=30.0, point_units="m")
    writer.set_point_labels(LABELS)
    writer.set_screen_axis(filter_c3d.X_SCREEN, filter_c3d.Y_SCREEN)
    writer.set_start_frame(1)
    frame = (np.zeros((len(LABELS), 5), dtype=np.float32),
             np.zeros((0, 1), dtype=np.float32))
    for _ in range(20):
        writer.add_frames([frame])
    with open(path, "wb") as h:
        writer.write(h)
    assert main([str(path)]) == 1


def test_screen_axis_missing_or_wrong_rejected(tmp_path):
    import c3d
    frame = (np.zeros((len(LABELS), 5), dtype=np.float32),
             np.zeros((0, 1), dtype=np.float32))

    # 欠落（set_screen_axis を呼ばない）
    path1 = tmp_path / "missing.c3d"
    writer = c3d.Writer(point_rate=30.0, point_units="mm")
    writer.set_point_labels(LABELS)
    writer.set_start_frame(1)
    for _ in range(20):
        writer.add_frames([frame])
    with open(path1, "wb") as h:
        writer.write(h)
    assert main([str(path1)]) == 1

    # 規約外（+Y/+X）
    path2 = tmp_path / "wrong.c3d"
    writer = c3d.Writer(point_rate=30.0, point_units="mm")
    writer.set_point_labels(LABELS)
    writer.set_screen_axis("+Y", "+X")
    writer.set_start_frame(1)
    for _ in range(20):
        writer.add_frames([frame])
    with open(path2, "wb") as h:
        writer.write(h)
    assert main([str(path2)]) == 1

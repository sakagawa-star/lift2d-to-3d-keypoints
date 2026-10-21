"""
feat-039: C3D→NPZ 変換スクリプト（phase4/c3d_to_npz.py）のテスト

テスト用 C3D は npz_to_c3d.write_c3d または py-c3d の c3d.Writer で tmp_path に作る。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import c3d  # noqa: E402
import c3d_to_npz  # noqa: E402
from c3d_to_npz import convert_to_world, main  # noqa: E402
from npz_to_c3d import load_npz, world_to_c3d_raw, write_c3d  # noqa: E402
from render_keypoints import c3d_to_calib, load_c3d_all_frames  # noqa: E402

HEAD7 = ["LEye", "REye", "LEar", "REar", "Nose", "Head", "Neck"]


def _write_raw_c3d(path, raw_mm, residual, labels, start_frame=1, rate=30.0) -> str:
    """c3d.Writer で raw 座標(mm)・residual 指定の C3D を書く。raw_mm: (F,J,3)。"""
    raw_mm = np.asarray(raw_mm, dtype=np.float64)
    F, J = raw_mm.shape[0], raw_mm.shape[1]
    writer = c3d.Writer(point_rate=rate, point_units="mm")
    writer.set_point_labels(labels)
    writer.set_start_frame(start_frame)
    empty_analog = np.zeros((0, 1), dtype=np.float32)
    for f in range(F):
        pts = np.zeros((J, 5), dtype=np.float32)
        pts[:, 0:3] = raw_mm[f]
        pts[:, 3] = residual[f]
        writer.add_frames([(pts, empty_analog)])
    with open(path, "wb") as h:
        writer.write(h)
    return str(path)


def _synth_world(F=5, J=4):
    rng = np.random.default_rng(0)
    return rng.normal(size=(F, J, 3)).astype(np.float64)


def _labels(J):
    return [f"J{i}" for i in range(J)]


def _make_c3d_from_world(tmp_path, W, labels=None, name="in.c3d") -> str:
    labels = labels or _labels(W.shape[1])
    p = str(tmp_path / name)
    write_c3d(p, world_to_c3d_raw(W), labels, 30.0)
    return p


def _tmp_leftovers(d: Path) -> list:
    return sorted(p.name for p in d.glob("*.tmp.npz"))


# T-01
def test_t01_roundtrip(tmp_path):
    W = _synth_world(5, 4)
    src = _make_c3d_from_world(tmp_path, W)
    out = str(tmp_path / "out.npz")
    assert main([src, "--output", out]) == 0
    x, f, n = load_npz(out)
    assert x.shape == (5, 4, 3)
    assert np.allclose(x, W, atol=1e-6)
    assert n == _labels(4)
    assert f.tolist() == [1, 2, 3, 4, 5]


# T-02
def test_t02_missing_by_residual(tmp_path):
    W = _synth_world(5, 4)
    W[2, 1, :] = np.nan
    src = _make_c3d_from_world(tmp_path, W)
    out = str(tmp_path / "out.npz")
    assert main([src, "--output", out]) == 0
    x, _, _ = load_npz(out)
    assert np.isnan(x[2, 1]).all()
    mask = np.ones(x.shape[:2], dtype=bool)
    mask[2, 1] = False
    assert np.isfinite(x[mask]).all()


# T-03
def test_t03_missing_by_zero(tmp_path):
    raw = np.random.default_rng(1).normal(size=(3, 3, 3)) * 100 + 10
    raw[1, 2] = 0.0
    res = np.zeros((3, 3))
    src = _write_raw_c3d(tmp_path / "z.c3d", raw, res, _labels(3))
    out = str(tmp_path / "out.npz")
    assert main([src, "--output", out]) == 0
    x, _, _ = load_npz(out)
    assert np.isnan(x[1, 2]).all()
    mask = np.ones(x.shape[:2], dtype=bool)
    mask[1, 2] = False
    assert np.isfinite(x[mask]).all()


# T-04
def test_t04_single_component_nonzero(tmp_path):
    raw = np.full((2, 2, 3), 10.0)
    raw[0, 0] = [0.0, 0.0, 5.0]
    res = np.zeros((2, 2))
    src = _write_raw_c3d(tmp_path / "p.c3d", raw, res, _labels(2))
    out = str(tmp_path / "out.npz")
    assert main([src, "--output", out]) == 0
    x, _, _ = load_npz(out)
    assert np.allclose(x[0, 0], [0.005, 0.0, 0.0], atol=1e-9)


# T-05
def test_t05_convert_to_world_unit():
    raw = np.array([[[1.0, 2.0, 3.0],     # (a) residual -1
                     [np.nan, 2.0, 3.0],  # (b)
                     [0.0, 0.0, 0.0],     # (c)
                     [4.0, 5.0, 6.0],     # residual NaN
                     [7.0, 8.0, 9.0]]])   # 有効
    res = np.array([[-1.0, 0.0, 0.0, np.nan, 0.0]])
    x, counts = convert_to_world(raw, res)
    assert np.isnan(x[0, :4]).all()
    assert np.array_equal(x[0, 4], c3d_to_calib(raw[0, 4]))
    assert counts == {"missing": 4, "residual": 2, "zero": 1, "nan": 1, "total": 5}


# T-06
def test_t06_start_frame_100(tmp_path):
    raw = np.full((3, 2, 3), 10.0)
    src = _write_raw_c3d(tmp_path / "s.c3d", raw, np.zeros((3, 2)), _labels(2), start_frame=100)
    out = str(tmp_path / "out.npz")
    assert main([src, "--output", out]) == 0
    _, f, _ = load_npz(out)
    assert f.tolist() == [100, 101, 102]


# T-07
def test_t07_default_output(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world(), name="a.c3d")
    assert main([src]) == 0
    assert (tmp_path / "a.npz").exists()


# T-08
def test_t08_output_option(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "sub_out.npz"
    assert main([src, "--output", str(out)]) == 0
    assert out.exists()


# T-09
def test_t09_bad_extension(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "b.txt"
    assert main([src, "--output", str(out)]) == 1
    assert not out.exists()


# T-10
def test_t10_same_path(tmp_path, capsys):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    before = Path(src).read_bytes()
    # 拡張子 .npz の入力ファイルと同一パスを指定
    npz_in = tmp_path / "same.npz"
    npz_in.write_bytes(before)
    assert main([str(npz_in), "--output", str(npz_in)]) == 1
    assert npz_in.read_bytes() == before
    assert "同一" in capsys.readouterr().err


# T-11
def test_t11_missing_input(tmp_path, capsys):
    assert main([str(tmp_path / "nothing.c3d")]) == 1
    assert "エラー:" in capsys.readouterr().err


# T-12
def test_t12_not_c3d(tmp_path, capsys):
    bad = tmp_path / "bad.c3d"
    bad.write_text("this is not a c3d file")
    assert main([str(bad)]) == 1
    assert "C3Dとして読み込めません" in capsys.readouterr().err


# T-13
def test_t13_zero_frames(tmp_path, monkeypatch):
    def fake(p):
        raise ValueError(f"C3Dにフレームがありません: {p}")
    monkeypatch.setattr(c3d_to_npz, "load_c3d_all_frames", fake)
    assert main([str(tmp_path / "x.c3d")]) == 1


def _patch_labels(monkeypatch, labels):
    frames = [{"frame_no": 1, "data": np.ones((max(len(labels), 1), 3)),
               "residual": np.zeros(max(len(labels), 1))}]
    monkeypatch.setattr(c3d_to_npz, "load_c3d_all_frames", lambda p: (labels, frames, 30.0))


# T-14
def test_t14_zero_markers(tmp_path, monkeypatch, capsys):
    _patch_labels(monkeypatch, [])
    assert main([str(tmp_path / "x.c3d")]) == 1
    assert "マーカーがありません" in capsys.readouterr().err


# T-15
def test_t15_empty_marker_name(tmp_path, monkeypatch, capsys):
    _patch_labels(monkeypatch, ["Head", ""])
    assert main([str(tmp_path / "x.c3d")]) == 1
    assert "空のマーカー名" in capsys.readouterr().err


# T-16
def test_t16_duplicate_marker_name(tmp_path, monkeypatch, capsys):
    _patch_labels(monkeypatch, ["Head", "Neck", "Head"])
    assert main([str(tmp_path / "x.c3d")]) == 1
    err = capsys.readouterr().err
    assert "重複" in err and "Head" in err


# T-17
def test_t17_output_dir_missing(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    outdir = tmp_path / "nodir"
    assert main([src, "--output", str(outdir / "o.npz")]) == 1
    assert not outdir.exists()


# T-18
def test_t18_verify_failure(tmp_path, monkeypatch):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "out.npz"
    real = c3d_to_npz.load_npz

    def fake(path):
        x, f, n = real(path)
        return x, f + 1, n
    monkeypatch.setattr(c3d_to_npz, "load_npz", fake)
    assert main([src, "--output", str(out)]) == 1
    assert not out.exists()
    assert _tmp_leftovers(tmp_path) == []


# T-19
def test_t19_overwrite(tmp_path):
    W = _synth_world()
    src = _make_c3d_from_world(tmp_path, W)
    out = tmp_path / "out.npz"
    out.write_bytes(b"old content")
    assert main([src, "--output", str(out)]) == 0
    x, _, _ = load_npz(str(out))
    assert np.allclose(x, W, atol=1e-6)


# T-20
def test_t20_head_markers_missing(tmp_path, capsys):
    raw = np.full((2, 3, 3), 10.0)
    src = _write_raw_c3d(tmp_path / "h.c3d", raw, np.zeros((2, 3)), ["LEye", "Nose", "Neck"])
    assert main([src, "--output", str(tmp_path / "o.npz")]) == 0
    err = capsys.readouterr().err
    assert "警告: 頭部マーカー" in err and "Head" in err


# T-21
def test_t21_head_markers_complete(tmp_path, capsys):
    raw = np.full((2, 7, 3), 10.0)
    src = _write_raw_c3d(tmp_path / "h.c3d", raw, np.zeros((2, 7)), HEAD7)
    assert main([src, "--output", str(tmp_path / "o.npz")]) == 0
    assert "警告" not in capsys.readouterr().err


# T-22
def test_t22_stdout(tmp_path, capsys):
    src = _make_c3d_from_world(tmp_path, _synth_world(5, 4))
    assert main([src, "--output", str(tmp_path / "o.npz")]) == 0
    out = capsys.readouterr().out
    assert "frame_ids 範囲: 1〜5" in out
    assert "欠損点（NaN にした点）:" in out


# T-23
def test_t23_npz_keys_and_dtypes(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "o.npz"
    assert main([src, "--output", str(out)]) == 0
    d = np.load(out)
    assert sorted(d.files) == sorted(["x3d_world", "frame_ids", "joint_names"])
    assert d["joint_names"].dtype.kind == "U"
    assert d["frame_ids"].dtype == np.int64
    assert d["x3d_world"].dtype == np.float64


# T-25
def test_t25_truncated_c3d(tmp_path, capsys):
    raw = np.random.default_rng(2).normal(size=(10, 3, 3)) * 100 + 10
    src = Path(_write_raw_c3d(tmp_path / "t.c3d", raw, np.zeros((10, 3)), _labels(3)))
    data = src.read_bytes()
    # ヘッダ（バイト16-17）のデータ開始ブロック（1始まり、512バイト単位）から数フレーム分だけ残す
    data_start = (int.from_bytes(data[16:18], "little") - 1) * 512
    src.write_bytes(data[:data_start + 150])
    try:
        _, fr, _ = load_c3d_all_frames(str(src))
        n_read = len(fr)
    except ValueError:
        n_read = 0
    assert 1 <= n_read < 10
    out = tmp_path / "out.npz"
    out.write_bytes(b"previous")
    assert main([str(src), "--output", str(out)]) == 1
    assert "途中で欠けています" in capsys.readouterr().err
    assert out.read_bytes() == b"previous"
    assert _tmp_leftovers(tmp_path) == []


# T-26
def test_t26_output_is_directory(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "result.npz"
    out.mkdir()
    assert main([src, "--output", str(out)]) == 1
    assert _tmp_leftovers(tmp_path) == []
    assert out.is_dir()


# T-27
def test_t27_existing_tmp_names_untouched(tmp_path):
    src = _make_c3d_from_world(tmp_path, _synth_world())
    out = tmp_path / "out.npz"
    pre = tmp_path / "out.npz.tmp.npz"
    pre.write_bytes(b"precious")
    victim = tmp_path / "victim.npz"
    victim.write_bytes(b"victim content")
    link = tmp_path / "out.npz.x.tmp.npz"
    link.symlink_to(victim)
    assert main([src, "--output", str(out)]) == 0
    assert pre.read_bytes() == b"precious"
    assert victim.read_bytes() == b"victim content"
    assert link.is_symlink() and link.resolve() == victim.resolve()
    assert _tmp_leftovers(tmp_path) == ["out.npz.tmp.npz", "out.npz.x.tmp.npz"]

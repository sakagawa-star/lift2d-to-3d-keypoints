"""feat-040 render_fps_video.py のフレーム範囲指定（start_frame / end_frame）のテスト。

GPU・ffmpeg 不要。MP4 モードの main は torch / which / libx264 / _run_mp4_mode のスタブで通す。

実行:
  uv run pytest tests/test_feat040_frame_range.py -v
"""

import json
import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import render_fps_video  # noqa: E402
from render_fps_video import (  # noqa: E402
    HEAD_JOINT_NAMES,
    _build_parser,
    build_manifest,
    main,
    parse_args_with_config,
    range_output_name,
    select_frame_range,
    write_manifest,
)

_DEFAULT_POSE = {
    "LEye": (-1.0, 0.0, 0.0), "REye": (1.0, 0.0, 0.0),
    "LEar": (-1.0, 0.0, 0.0), "REar": (1.0, 0.0, 0.0),
    "Nose": (0.0, 1.0, 0.0), "Head": (0.0, 0.0, 1.0), "Neck": (0.0, 0.0, 0.0),
}

_TOML_TEMPLATE = """[cam01]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
size = [{w}, {h}]
"""


def _pose_row(overrides: dict | None = None) -> np.ndarray:
    pose = dict(_DEFAULT_POSE)
    if overrides:
        pose.update(overrides)
    return np.array([pose[name] for name in HEAD_JOINT_NAMES], dtype=np.float64)


def _write_npz(path, rows: list[np.ndarray], frame_ids: np.ndarray):
    np.savez(
        str(path),
        x3d_world=np.stack(rows, axis=0),
        frame_ids=frame_ids.astype(np.int64),
        joint_names=np.array(HEAD_JOINT_NAMES, dtype="<U16"),
    )


def _setup(tmp_path, nan_fid: int | None = None):
    """frame_ids 1〜10 の NPZ・TOML・PLY を作る。"""
    pytest.importorskip("tomli")
    ply = tmp_path / "scene.ply"
    ply.write_bytes(b"")
    toml = tmp_path / "calib.toml"
    toml.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))
    rows = []
    for fid in range(1, 11):
        row = _pose_row()
        if nan_fid is not None and fid == nan_fid:
            row[HEAD_JOINT_NAMES.index("Nose")] = np.nan
        rows.append(row)
    npz = tmp_path / "a.npz"
    _write_npz(npz, rows, np.arange(1, 11))
    return str(ply), str(npz), str(toml)


def _base(ply, npz, toml):
    return [ply, npz, "--toml", toml, "--camera", "cam01", "--fps", "30"]


def _stub_mp4(monkeypatch, record: dict, stub_run=True):
    monkeypatch.setitem(
        sys.modules, "torch",
        types.SimpleNamespace(cuda=types.SimpleNamespace(device_count=lambda: 1)),
    )
    monkeypatch.setattr(render_fps_video.shutil, "which", lambda name: "/usr/bin/fake")
    monkeypatch.setattr(render_fps_video, "ffmpeg_has_libx264", lambda b: True)
    if stub_run:
        def fake_run(args, output_path, frame_ids, intrinsics, viewmats, valid,
                     nan_fids, degen_fids):
            record["output_path"] = output_path
            record["frame_ids"] = [int(f) for f in frame_ids]
            record["nan_fids"] = list(nan_fids)
            record["degen_fids"] = list(degen_fids)
            return 0
        monkeypatch.setattr(render_fps_video, "_run_mp4_mode", fake_run)


# T-01
def test_select_frame_range():
    fids = np.arange(1, 11)

    def sel(s, e):
        return fids[select_frame_range(fids, s, e)].tolist()

    assert sel(3, 5) == [3, 4, 5]
    assert sel(None, 8) == list(range(1, 9))
    assert sel(4, None) == list(range(4, 11))
    assert sel(None, None) == list(range(1, 11))
    assert sel(0, 100) == list(range(1, 11))
    assert sel(20, 30) == []
    assert sel(5, 3) == []


# T-02
def test_range_output_name():
    assert range_output_name("/d/a.npz", 1, 900) == "/d/a_fps_1-900.mp4"
    assert range_output_name("rel/b.npz", -5, 3) == os.path.join(
        os.path.dirname(os.path.abspath("rel/b.npz")), "b_fps_-5-3.mp4")


# T-03
def test_parser_start_end():
    args = _build_parser().parse_args(["--start-frame", "10", "--end-frame", "20"])
    assert args.start_frame == 10 and args.end_frame == 20


# T-04
def test_parser_non_int_exits_2():
    with pytest.raises(SystemExit) as e:
        _build_parser().parse_args(["--start-frame", "1.5"])
    assert e.value.code == 2


# T-05
_REQUIRED_YAML = (
    "ply_path: p.ply\nnpz_path: n.npz\ntoml: t.toml\ncamera: cam01\nfps: 30\n"
)


def test_config_yaml_start_end(tmp_path):
    cfg = tmp_path / "c.yaml"
    cfg.write_text(_REQUIRED_YAML + "start_frame: 10\nend_frame: 20\n")
    args = parse_args_with_config(["--config", str(cfg)])
    assert args.start_frame == 10 and args.end_frame == 20


# T-06
def test_config_yaml_non_int_exits_2(tmp_path, capsys):
    cfg = tmp_path / "c.yaml"
    cfg.write_text("start_frame: 1.5\n")
    with pytest.raises(SystemExit) as e:
        parse_args_with_config(["--config", str(cfg)])
    assert e.value.code == 2
    assert "start_frame" in capsys.readouterr().err


# T-07
def test_cli_overrides_config_yaml(tmp_path):
    cfg = tmp_path / "c.yaml"
    cfg.write_text(_REQUIRED_YAML + "start_frame: 10\n")
    args = parse_args_with_config(["--config", str(cfg), "--start-frame", "5"])
    assert args.start_frame == 5


# T-08
@pytest.mark.parametrize("opt", ["--start-frame", "--end-frame"])
def test_still_range_with_range_exits_2(tmp_path, capsys, opt):
    ply, npz, toml = _setup(tmp_path)
    with pytest.raises(SystemExit) as e:
        main(_base(ply, npz, toml) + ["--still-range", "1", "2", "--still-dir", "d", opt, "1"])
    assert e.value.code == 2
    assert "--still-range" in capsys.readouterr().err


def _dump(tmp_path, extra, name="p.json"):
    ply, npz, toml = _setup(tmp_path)
    out = tmp_path / name
    rc = main(_base(ply, npz, toml) + extra + ["--dump-poses", str(out)])
    return rc, out


# T-09
def test_dump_range(tmp_path):
    rc, out = _dump(tmp_path, ["--start-frame", "3", "--end-frame", "5"])
    assert rc == 0
    payload = json.loads(out.read_text())
    assert payload["num_frames"] == 3
    assert [f["frame_id"] for f in payload["frames"]] == [3, 4, 5]


# T-10
def test_dump_range_matches_full(tmp_path):
    rc1, out_full = _dump(tmp_path, [], "full.json")
    rc2, out_rng = _dump(tmp_path, ["--start-frame", "3", "--end-frame", "5"], "rng.json")
    assert rc1 == 0 and rc2 == 0
    full = {f["frame_id"]: f for f in json.loads(out_full.read_text())["frames"]}
    for f in json.loads(out_rng.read_text())["frames"]:
        assert f["valid"] == full[f["frame_id"]]["valid"]
        assert f["c2w"] == full[f["frame_id"]]["c2w"]


# T-11
def test_dump_range_overhang(tmp_path, capsys):
    rc, out = _dump(tmp_path, ["--start-frame", "0", "--end-frame", "3"])
    assert rc == 0
    payload = json.loads(out.read_text())
    assert payload["num_frames"] == 3
    assert [f["frame_id"] for f in payload["frames"]] == [1, 2, 3]
    assert "フレーム範囲: 10 フレーム中 3 フレームを対象（frame_ids 1〜3）" in capsys.readouterr().out


# T-12
@pytest.mark.parametrize("extra", [["--start-frame", "20"],
                                   ["--start-frame", "5", "--end-frame", "3"]])
def test_dump_range_empty(tmp_path, capsys, extra):
    rc, out = _dump(tmp_path, extra)
    assert rc == 1
    err = capsys.readouterr().err
    assert "該当するフレームがありません" in err and "1〜10" in err
    assert not out.exists()


# T-13
def test_dump_no_range_no_message(tmp_path, capsys):
    rc, _ = _dump(tmp_path, [])
    assert rc == 0
    assert "フレーム範囲" not in capsys.readouterr().out


# T-14
def test_mp4_default_name_with_range(tmp_path, monkeypatch):
    ply, npz, toml = _setup(tmp_path)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    rc = main(_base(ply, npz, toml) + ["--start-frame", "3", "--end-frame", "5"])
    assert rc == 0
    assert rec["output_path"] == str(tmp_path / "a_fps_3-5.mp4")
    assert rec["frame_ids"] == [3, 4, 5]


# T-15
def test_mp4_default_name_overhang(tmp_path, monkeypatch):
    ply, npz, toml = _setup(tmp_path)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    rc = main(_base(ply, npz, toml) + ["--start-frame", "0", "--end-frame", "3"])
    assert rc == 0
    assert rec["output_path"] == str(tmp_path / "a_fps_1-3.mp4")


# T-16
def test_mp4_no_range_unchanged(tmp_path, monkeypatch):
    ply, npz, toml = _setup(tmp_path)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    rc = main(_base(ply, npz, toml))
    assert rc == 0
    assert rec["output_path"] == str(tmp_path / "a_fps.mp4")
    assert rec["frame_ids"] == list(range(1, 11))


# T-17
def test_mp4_explicit_output_wins(tmp_path, monkeypatch):
    ply, npz, toml = _setup(tmp_path)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    rc = main(_base(ply, npz, toml) + ["--start-frame", "3", "--output", "x.mp4"])
    assert rc == 0
    assert rec["output_path"] == "x.mp4"


# T-18
def test_mp4_bad_extension(tmp_path, monkeypatch, capsys):
    ply, npz, toml = _setup(tmp_path)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    rc = main(_base(ply, npz, toml) + ["--start-frame", "3", "--output", "x.avi"])
    assert rc == 1
    assert "拡張子が .mp4 ではありません" in capsys.readouterr().err


# T-19
def test_mp4_nan_fids_within_range(tmp_path, monkeypatch):
    ply, npz, toml = _setup(tmp_path, nan_fid=4)
    rec: dict = {}
    _stub_mp4(monkeypatch, rec)
    assert main(_base(ply, npz, toml) + ["--start-frame", "3", "--end-frame", "5"]) == 0
    assert rec["nan_fids"] == [4]
    rec2: dict = {}
    _stub_mp4(monkeypatch, rec2)
    assert main(_base(ply, npz, toml) + ["--start-frame", "6"]) == 0
    assert rec2["nan_fids"] == []


# T-21
def test_rerun_with_different_range_manifest_mismatch(tmp_path, monkeypatch, capsys):
    ply, npz, toml = _setup(tmp_path)
    out = str(tmp_path / "r.mp4")
    chunk_dir = out + ".chunks"
    os.makedirs(chunk_dir)
    parser = _build_parser()
    args = parser.parse_args(_base(ply, npz, toml))
    args.gpu = 0
    write_manifest(chunk_dir, build_manifest(args, 3, 1, 3, 1920, 1080))

    _stub_mp4(monkeypatch, {}, stub_run=False)

    def no_render(*a, **k):
        raise AssertionError("render_chunk が呼ばれた")
    monkeypatch.setattr(render_fps_video, "render_chunk", no_render)

    rc = main(_base(ply, npz, toml) + ["--start-frame", "4", "--end-frame", "6",
                                       "--output", out])
    assert rc == 1
    err = capsys.readouterr().err
    assert "パラメータが変更されています" in err and "fid_first" in err

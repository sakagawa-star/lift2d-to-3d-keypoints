"""feat-043: 前処理と推定で 1 つの設定 YAML を共用するテスト（GPU・実データ不要）。

設計: docs/issues/feat-043-shared-job-yaml-phone-photo/design.md §6.1
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(Path(__file__).parent))

import estimate_intrinsics_loop as eil  # noqa: E402
import prepare_phone_photo as ppp  # noqa: E402
import refine_extrinsics as rex  # noqa: E402
from refine_extrinsics import parse_args_with_config  # noqa: E402
from test_feat042_config_yaml import _loop_setup  # noqa: E402

PREP_REQ = {"image": "o.jpg", "camera": "camA", "images_dir": "imgs", "init_toml": "i.toml",
            "points_2d_original": "in.csv", "points_2d": "out.csv"}
ALL_KEYS = {
    **PREP_REQ, "expect_model": "Galaxy S25", "expect_focal_mm": "5.4", "overwrite": "true",
    "points_3d": "p3.csv", "ply": "s.ply", "out_dir": "outd", "tmp_dir": "tmpd",
}
PREP_ONLY = {"image", "points_2d_original", "expect_model", "expect_focal_mm", "overwrite"}
EST_ONLY = {"points_3d", "ply", "out_dir", "tmp_dir"}


def _yaml(path: Path, d: dict) -> str:
    path.write_text("".join(f"{k}: {v}\n" for k, v in d.items()), encoding="utf-8")
    return str(path)


def _parse(mod, argv):
    return parse_args_with_config(argv, mod.build_arg_parser(), mod.CONFIG_CONVERTERS,
                                  mod.REQUIRED_OPTIONS, mod.IGNORED_KEYS)


def _prep_cli(**over) -> list[str]:
    d = {**PREP_REQ, **over}
    out = []
    for k, v in d.items():
        out += ["--" + k.replace("_", "-"), str(v)]
    return out


def _mock_exif(monkeypatch):
    monkeypatch.setattr(ppp, "read_exif", lambda path: {
        "Model": "Galaxy S25", "Orientation": 1, "FocalLength": 5.4,
        "FocalLengthIn35mmFilm": 24, "DigitalZoomRatio": 1.0, "Size": (64, 48)})


def _make_prep_inputs(tmp_path):
    src = tmp_path / "orig.jpg"
    cv2.imwrite(str(src), np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8))
    csv_in = tmp_path / "p_in.csv"
    lines = ["ObjectName,camera_name,X,Y"] + [f"P{k},camA,{10 + k * 8},{12 + k * 4}" for k in range(6)]
    csv_in.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return src, csv_in


def test_1_prep_all_keys(tmp_path):
    d = {**PREP_REQ, "expect_model": "Galaxy S25", "expect_focal_mm": "5.4", "overwrite": "true"}
    a = _parse(ppp, ["--config", _yaml(tmp_path / "c.yaml", d)])
    for k, v in PREP_REQ.items():
        assert getattr(a, k) == v
    assert a.expect_model == "Galaxy S25" and a.expect_focal_mm == 5.4 and a.overwrite is True


def test_2_prep_priority(tmp_path):
    d = {**PREP_REQ, "init_toml": "a.toml"}
    a = _parse(ppp, ["--config", _yaml(tmp_path / "c.yaml", d), "--init-toml", "b.toml"])
    assert a.init_toml == "b.toml"


def test_3_prep_missing_required(tmp_path, capsys):
    conf = _yaml(tmp_path / "e.yaml", {})
    with pytest.raises(SystemExit) as ei:
        _parse(ppp, ["--config", conf])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    for label in ("--image", "--camera", "--images-dir", "--init-toml",
                  "--points-2d-original", "--points-2d"):
        assert label in err


def test_4_shared_yaml_both(tmp_path):
    conf = _yaml(tmp_path / "j.yaml", ALL_KEYS)
    p = _parse(ppp, ["--config", conf])
    e = _parse(eil, ["--config", conf])
    for k in EST_ONLY:
        assert not hasattr(p, k)
    for k in PREP_ONLY:
        assert not hasattr(e, k)
    for k in ("camera", "images_dir", "init_toml", "points_2d"):
        assert getattr(p, k) == getattr(e, k) == ALL_KEYS[k]


def test_5_other_keys_not_checked(tmp_path, capsys):
    conf = _yaml(tmp_path / "j.yaml", {**ALL_KEYS, "expect_focal_mm": "abc"})
    _parse(eil, ["--config", conf])   # (a) エラーにならない
    with pytest.raises(SystemExit) as ei:   # (b)
        _parse(ppp, ["--config", conf])
    assert ei.value.code == 2
    assert "expect_focal_mm" in capsys.readouterr().err


@pytest.mark.parametrize("mod", [ppp, eil])
def test_6_unknown_key(mod, tmp_path, capsys):
    conf = _yaml(tmp_path / "j.yaml", {**ALL_KEYS, "foo": "1"})
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--config", conf])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert "未知のキー" in err and "foo" in err


def test_7_old_names_rejected(tmp_path, capsys):
    for extra in (["--out-toml", "x"], ["--points-2d-in", "x"], ["--points-2d-out", "x"]):
        with pytest.raises(SystemExit) as ei:
            _parse(ppp, _prep_cli() + extra)
        assert ei.value.code == 2
        assert "unrecognized arguments" in capsys.readouterr().err
    est_cli = ["--camera", "camA", "--ply", "x.ply", "--images-dir", "i", "--points-3d", "a",
               "--points-2d", "b", "--out-dir", "o"]
    with pytest.raises(SystemExit) as ei:
        _parse(eil, est_cli + ["--toml", "x"])
    assert ei.value.code == 2
    assert "unrecognized arguments" in capsys.readouterr().err
    conf = _yaml(tmp_path / "e.yaml", {"toml": "x", "camera": "camA", "ply": "x.ply",
                                       "images_dir": "i", "points_3d": "a", "points_2d": "b",
                                       "out_dir": "o"})
    with pytest.raises(SystemExit) as ei:
        _parse(eil, ["--config", conf])
    assert ei.value.code == 2
    assert "未知のキー" in capsys.readouterr().err
    # 省略形（既存の出力ファイルは変わらない）
    outs = {"images_dir": str(tmp_path / "imgs"), "init_toml": str(tmp_path / "i.toml"),
            "points_2d": str(tmp_path / "out.csv")}
    for p in (outs["init_toml"], outs["points_2d"]):
        Path(p).write_text("old", encoding="utf-8")
    base = _prep_cli(**outs)
    cam_idx = base.index("--camera")
    no_cam = base[:cam_idx] + base[cam_idx + 2:]
    for argv in (base + ["--o"], base + ["--points-2d-o", "x"], no_cam + ["--cam", "camA"]):
        with pytest.raises(SystemExit) as ei:
            ppp.main(argv)
        assert ei.value.code == 2
        assert "unrecognized arguments" in capsys.readouterr().err
    for p in (outs["init_toml"], outs["points_2d"]):
        assert Path(p).read_text(encoding="utf-8") == "old"


def test_8_other_tool_cli_rejected(capsys):
    est_cli = ["--camera", "camA", "--ply", "x.ply", "--images-dir", "i", "--points-3d", "a",
               "--points-2d", "b", "--out-dir", "o", "--init-toml", "t"]
    with pytest.raises(SystemExit) as ei:
        _parse(eil, est_cli + ["--image", "x"])
    assert ei.value.code == 2
    assert "unrecognized arguments" in capsys.readouterr().err
    with pytest.raises(SystemExit) as ei:
        _parse(ppp, _prep_cli() + ["--ply", "x"])
    assert ei.value.code == 2
    assert "unrecognized arguments" in capsys.readouterr().err


def test_18_estimate_abbrev_rejected(capsys):
    full = {"--camera": "camA", "--ply": "x.ply", "--images-dir": "i", "--points-3d": "a",
            "--points-2d": "b", "--out-dir": "o", "--init-toml": "t"}

    def cli_without(opt):
        out = []
        for k, v in full.items():
            if k != opt:
                out += [k, v]
        return out

    cases = [
        cli_without("--images-dir") + ["--images", "photo.jpg"],
        cli_without("--camera") + ["--cam", "camA"],
        cli_without("--out-dir") + ["--ou", "x"],
    ]
    for argv in cases:
        with pytest.raises(SystemExit) as ei:
            _parse(eil, argv)
        assert ei.value.code == 2
        assert "unrecognized arguments" in capsys.readouterr().err


def test_9_points_2d_exact():
    a = _parse(ppp, _prep_cli(points_2d="out.csv", points_2d_original="in.csv"))
    assert a.points_2d == "out.csv" and a.points_2d_original == "in.csv"


@pytest.mark.parametrize("bad", ["sub/cam", ".."])
def test_10_camera_name(bad, monkeypatch, tmp_path):
    _mock_exif(monkeypatch)
    src, csv_in = _make_prep_inputs(tmp_path)
    outs = {"images_dir": tmp_path / "imgs", "init_toml": tmp_path / "i.toml",
            "points_2d": tmp_path / "out.csv"}
    code = ppp.main(_prep_cli(image=src, camera=bad, points_2d_original=csv_in, **outs))
    assert code == 1
    for p in outs.values():
        assert not p.exists()


def _shared_job(tmp_path, src, csv_in):
    return _yaml(tmp_path / "job.yaml", {
        **ALL_KEYS, "image": src, "points_2d_original": csv_in,
        "images_dir": tmp_path / "imgs", "init_toml": tmp_path / "i.toml",
        "points_2d": tmp_path / "out.csv", "out_dir": tmp_path / "outd",
        "tmp_dir": tmp_path / "tmpd", "expect_focal_mm": "5.4"})


def test_11_12_prep_run_then_estimate_parse(monkeypatch, tmp_path):
    _mock_exif(monkeypatch)
    src, csv_in = _make_prep_inputs(tmp_path)
    conf = _shared_job(tmp_path, src, csv_in)
    assert ppp.main(["--config", conf]) == 0
    assert (tmp_path / "imgs" / "camA.png").exists()
    assert (tmp_path / "i.toml").exists() and (tmp_path / "out.csv").exists()
    assert not (tmp_path / "outd").exists()
    e = _parse(eil, ["--config", conf])
    assert Path(e.images_dir) == tmp_path / "imgs"
    assert Path(e.init_toml) == tmp_path / "i.toml"
    assert Path(e.points_2d) == tmp_path / "out.csv"


def test_13_default_ignored_keys(tmp_path, capsys):
    conf = _yaml(tmp_path / "u.yaml", {**{k: "v" for k in ("toml", "ply", "images_dir", "points_3d",
                                                             "points_2d", "out_toml", "out_report")},
                                       "foo": "1"})
    with pytest.raises(SystemExit) as ei:
        parse_args_with_config(["--config", conf], rex.build_arg_parser(),
                               rex.CONFIG_CONVERTERS, rex.REQUIRED_OPTIONS)
    assert ei.value.code == 2
    assert "未知のキー" in capsys.readouterr().err


def test_14_overlap_assert():
    with pytest.raises(AssertionError):
        parse_args_with_config([], ppp.build_arg_parser(), ppp.CONFIG_CONVERTERS,
                               ppp.REQUIRED_OPTIONS, frozenset({"camera"}))


def test_15_help():
    h = ppp.build_arg_parser().format_help()
    for s in ("--init-toml", "--points-2d-original", "--points-2d", "--config"):
        assert s in h
    for s in ("--out-toml", "--points-2d-in", "--points-2d-out"):
        assert s not in h
    h = eil.build_arg_parser().format_help()
    assert "--init-toml" in h and "--toml " not in h


def test_16_estimate_main_ignores_other_keys(monkeypatch, tmp_path):
    calls = _loop_setup(monkeypatch, tmp_path)
    conf = _yaml(tmp_path / "j.yaml", {
        **ALL_KEYS, "init_toml": "x.toml", "ply": "x.ply", "images_dir": tmp_path,
        "points_3d": "a.csv", "points_2d": "b.csv", "out_dir": tmp_path / "out",
        "tmp_dir": tmp_path / "tmp"})
    assert eil.main(["--config", conf]) == 3
    assert calls == list(range(8))
    a = _parse(eil, ["--config", conf])
    assert not hasattr(a, "overwrite")


@pytest.mark.parametrize("empty", ["images_dir", "init_toml", "points_2d"])
def test_17_empty_output_path(empty, monkeypatch, tmp_path):
    _mock_exif(monkeypatch)
    monkeypatch.chdir(tmp_path)
    src, csv_in = _make_prep_inputs(tmp_path)
    work = tmp_path / "work"
    work.mkdir()
    monkeypatch.chdir(work)
    d = {"image": src, "camera": "camA", "points_2d_original": csv_in,
         "images_dir": tmp_path / "imgs", "init_toml": tmp_path / "i.toml",
         "points_2d": tmp_path / "out.csv", "overwrite": "true"}
    d[empty] = ""
    conf = tmp_path / "e.yaml"
    conf.write_text("".join(f"{k}:{(' ' + str(v)) if v != '' else ''}\n" for k, v in d.items()),
                    encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        ppp.main(["--config", str(conf)])
    assert ei.value.code == 2
    assert list(work.iterdir()) == []
    for name in ("imgs", "i.toml", "out.csv"):
        assert not (tmp_path / name).exists()

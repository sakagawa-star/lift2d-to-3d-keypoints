"""feat-042 精緻化ツール 3 本の --config（設定YAML）対応テスト。

GPU・torch 不要（重い処理はモック）。設計: docs/issues/feat-042-config-yaml-refine-tools/design.md §6

実行:
  uv run pytest tests/test_feat042_config_yaml.py -v
"""

import os
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import adjust_extrinsics_multiview as aem  # noqa: E402
import estimate_intrinsics_loop as eil  # noqa: E402
import refine_extrinsics as rex  # noqa: E402
import render_fps_video  # noqa: E402
from estimate_intrinsics_loop import Params  # noqa: E402
from refine_extrinsics import parse_args_with_config  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# === ツール別の定義 ===

REQ_REFINE = {"toml": "a.toml", "ply": "b.ply", "images_dir": "imgs", "points_3d": "p3.csv",
              "points_2d": "p2.csv", "out_toml": "o.toml", "out_report": "r.txt"}
OPT_REFINE = {"cameras": ["camA", "camB"], "overwrite": True, "seed_base": 77, "tmp_dir": "tmpd"}

REQ_ADJUST = {"toml": "a.toml", "ply": "b.ply", "images_dir": "imgs",
              "out_toml": "o.toml", "out_report": "r.txt"}
OPT_ADJUST = {"init_toml": "init.toml", "init_cameras": ["camA", "camB"],
              "cameras": ["camA", "camB", "camC"], "overwrite": True, "seed": 9,
              "tmp_dir": "tmpd", "fresh_match": True}

REQ_LOOP = {"init_toml": "a.toml", "camera": "camA", "ply": "b.ply", "images_dir": "imgs",
            "points_3d": "p3.csv", "points_2d": "p2.csv", "out_dir": "outd"}
OPT_LOOP = {"tmp_dir": "tmpd"}

TOOLS = {
    "refine": (rex, REQ_REFINE, OPT_REFINE),
    "adjust": (aem, REQ_ADJUST, OPT_ADJUST),
    "loop": (eil, REQ_LOOP, OPT_LOOP),
}
TOOL_IDS = list(TOOLS)


def _cli(d: dict) -> list[str]:
    """dest 名→値の辞書を CLI 引数列にする。"""
    out: list[str] = []
    for k, v in d.items():
        opt = "--" + k.replace("_", "-")
        if v is True:
            out.append(opt)
        elif isinstance(v, list):
            out += [opt] + v
        else:
            out += [opt, str(v)]
    return out


def _yaml_text(d: dict) -> str:
    lines = []
    for k, v in d.items():
        if v is True:
            v = "true"
        elif isinstance(v, list):
            v = " ".join(v)
        lines.append(f"{k}: {v}")
    return "\n".join(lines) + "\n"


def _write_yaml(path: Path, d: dict) -> str:
    path.write_text(_yaml_text(d), encoding="utf-8")
    return str(path)


def _parse(mod, argv):
    parser = mod.build_arg_parser()
    return parse_args_with_config(argv, parser, mod.CONFIG_CONVERTERS, mod.REQUIRED_OPTIONS,
                                  getattr(mod, "IGNORED_KEYS", frozenset()))


def _vars_wo_config(ns) -> dict:
    d = dict(vars(ns))
    d.pop("config")
    return d


# === 1. 全キーの適用 ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_1_all_keys(tool, tmp_path):
    mod, req, opt = TOOLS[tool]
    full = {**req, **opt}
    conf = _write_yaml(tmp_path / "c.yaml", full)
    a = _parse(mod, ["--config", conf])
    b = mod.build_arg_parser().parse_args(_cli(full))
    assert _vars_wo_config(a) == _vars_wo_config(b)
    assert a.config == conf
    for k, v in full.items():
        assert getattr(a, k) == v


# === 2. 優先順位 ===

def test_2_priority(tmp_path):
    base = {k: v for k, v in REQ_REFINE.items()}
    conf = _write_yaml(tmp_path / "c.yaml", {**base, "seed_base": 100, "cameras": ["a", "b"]})
    a = _parse(rex, ["--config", conf, "--seed-base", "200", "--cameras", "c"])
    assert a.seed_base == 200
    assert a.cameras == ["c"]
    b = _parse(rex, ["--config", conf])
    assert b.seed_base == 100
    assert b.cameras == ["a", "b"]
    conf2 = _write_yaml(tmp_path / "c2.yaml", base)
    c = _parse(rex, ["--config", conf2])
    assert c.seed_base == rex.SEED_BASE_DEFAULT
    assert c.cameras is None


# === 3. フラグの非対称 ===

def test_3_flag_asymmetry(tmp_path):
    conf = _write_yaml(tmp_path / "c.yaml", {**REQ_ADJUST, "fresh_match": True})
    a = _parse(aem, ["--config", conf])
    assert a.fresh_match is True
    b = _parse(aem, ["--config", conf, "--overwrite"])
    assert b.fresh_match is True and b.overwrite is True


# === 4. 必須項目の欠落 ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_4_missing_required(tool, tmp_path, capsys):
    mod, req, _ = TOOLS[tool]
    for k in req:
        d = {kk: vv for kk, vv in req.items() if kk != k}
        conf = _write_yaml(tmp_path / f"c_{k}.yaml", d)
        with pytest.raises(SystemExit) as ei:
            _parse(mod, ["--config", conf])
        assert ei.value.code == 2
        err = capsys.readouterr().err
        assert "必須項目が未指定です" in err
        assert "--" + k.replace("_", "-") in err
    # 空の YAML: 必須項目すべて
    conf = tmp_path / "empty.yaml"
    conf.write_text("", encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--config", str(conf)])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert "必須項目が未指定です" in err
    for k in req:
        assert "--" + k.replace("_", "-") in err


# === 4b. CLI と YAML に分けて渡す ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_4b_split_cli_yaml(tool, tmp_path):
    mod, req, _ = TOOLS[tool]
    key = "init_toml" if tool == "loop" else "toml"   # feat-043: 推定は init_toml
    rest = {k: v for k, v in req.items() if k != key}
    conf = _write_yaml(tmp_path / "c.yaml", rest)
    a = _parse(mod, ["--config", conf, "--" + key.replace("_", "-"), "cli.toml"])
    assert getattr(a, key) == "cli.toml"
    for k, v in rest.items():
        assert getattr(a, k) == v


# === 5. 未知のキー ===

def test_5_unknown_key(tmp_path, capsys):
    conf = _write_yaml(tmp_path / "a.yaml", {**REQ_LOOP, "seed_base": 5})
    with pytest.raises(SystemExit) as ei:
        _parse(eil, ["--config", conf])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert "未知のキー" in err and "seed_base" in err
    conf = _write_yaml(tmp_path / "b.yaml", {**REQ_REFINE, "camera": "x"})
    with pytest.raises(SystemExit) as ei:
        _parse(rex, ["--config", conf])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert "未知のキー" in err and "camera" in err


# === 6. config キー ===

def test_6_config_key(tmp_path, capsys):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(REQ_REFINE) + "config: other.yaml\n", encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        _parse(rex, ["--config", str(conf)])
    assert ei.value.code == 2
    assert "config キー" in capsys.readouterr().err


# === 7. bool の不正 ===

@pytest.mark.parametrize("val", ["True", "1"])
def test_7_bad_bool(val, tmp_path, capsys):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(REQ_REFINE) + f"overwrite: {val}\n", encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        _parse(rex, ["--config", str(conf)])
    assert ei.value.code == 2
    assert "overwrite" in capsys.readouterr().err


# === 8. 複数値の不正 ===

@pytest.mark.parametrize("mod,req,line", [
    (rex, REQ_REFINE, "cameras:"),
    (aem, REQ_ADJUST, "init_cameras:   "),
])
def test_8_bad_multi(mod, req, line, tmp_path, capsys):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(req) + line + "\n", encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--config", str(conf)])
    assert ei.value.code == 2
    assert line.split(":")[0] in capsys.readouterr().err


# === 9. 複数値の区切り ===

def test_9_multi_separator(tmp_path):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(REQ_REFINE) + "cameras: cam01  cam02\tcam03\n", encoding="utf-8")
    a = _parse(rex, ["--config", str(conf)])
    assert a.cameras == ["cam01", "cam02", "cam03"]


# === 9b. インデントした行 ===

def test_9b_indented_line(tmp_path):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(REQ_REFINE) + "tmp_dir:\n  overwrite: true\n", encoding="utf-8")
    a = _parse(rex, ["--config", str(conf)])
    assert a.tmp_dir == ""
    assert a.overwrite is True


# === 10. 数値の不正 ===

@pytest.mark.parametrize("mod,req,line", [
    (rex, REQ_REFINE, "seed_base: abc"),
    (aem, REQ_ADJUST, "seed: 1.5"),
])
def test_10_bad_int(mod, req, line, tmp_path, capsys):
    conf = tmp_path / "c.yaml"
    conf.write_text(_yaml_text(req) + line + "\n", encoding="utf-8")
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--config", str(conf)])
    assert ei.value.code == 2
    assert line.split(":")[0] in capsys.readouterr().err


# === 10b. NUL 文字 ===

@pytest.mark.parametrize("key,val", [("points_3d", "a\x00b.csv"), ("cameras", "cam\x00x")])
def test_10b_nul(key, val, tmp_path, capsys):
    d = dict(REQ_REFINE)
    d[key] = val
    conf = tmp_path / "c.yaml"
    conf.write_bytes((("\n".join(f"{k}: {v}" for k, v in d.items())) + "\n").encode("utf-8"))
    with pytest.raises(SystemExit) as ei:
        _parse(rex, ["--config", str(conf)])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert key in err and "NUL 文字" in err


# === 10c. 引数エラーの優先順位 ===

@pytest.mark.parametrize("tool", TOOL_IDS)
@pytest.mark.parametrize("bad", [["--bogus"], ["extra"]])
def test_10c_unrecognized_first(tool, bad, capsys):
    mod = TOOLS[tool][0]
    with pytest.raises(SystemExit) as ei:
        _parse(mod, bad)
    assert ei.value.code == 2
    assert "unrecognized arguments" in capsys.readouterr().err


# === 11. 設定YAMLが無い ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_11_missing_yaml(tool, tmp_path, capsys):
    mod = TOOLS[tool][0]
    p = str(tmp_path / "nothing.yaml")
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--config", p])
    assert ei.value.code == 2
    assert p in capsys.readouterr().err


# === 11b. UTF-8 でない ===

def test_11b_not_utf8(tmp_path, capsys):
    p = tmp_path / "bad.yaml"
    p.write_bytes(b"toml: \xff\xfe\n")
    with pytest.raises(SystemExit) as ei:
        _parse(rex, ["--config", str(p)])
    assert ei.value.code == 2
    err = capsys.readouterr().err
    assert "UTF-8 として読めません" in err and str(p) in err


# === 11c. 省略形 ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_11c_abbrev(tool, tmp_path, monkeypatch, capsys):
    mod, req, _ = TOOLS[tool]
    calls = []
    orig = render_fps_video.load_yaml_flat

    def spy(path):
        calls.append(path)
        return orig(path)

    monkeypatch.setattr(render_fps_video, "load_yaml_flat", spy)
    y1 = _write_yaml(tmp_path / "y1.yaml", {"tmp_dir": "t1"})
    y2 = _write_yaml(tmp_path / "y2.yaml", {"tmp_dir": "t2"})
    base = _cli(req)

    def run(extra):
        with pytest.raises(SystemExit) as ei:
            _parse(mod, base + extra)
        return ei.value.code, capsys.readouterr()

    code, cap = run(["--c", "camA"])
    if tool == "loop":
        assert code == 2 and "unrecognized arguments" in cap.err
    else:
        assert code == 2 and "ambiguous" in cap.err
    code, cap = run(["--conf", y1])
    assert code == 2 and "--config は省略せずに指定してください" in cap.err
    code, cap = run(["--conf", y1, "--config", y2])
    assert code == 2 and "--config は省略せずに指定してください" in cap.err
    code, cap = run(["--config", y1, "--c", "camA"])
    if tool == "loop":
        assert code == 2 and "unrecognized arguments" in cap.err
    else:
        assert code == 2 and "ambiguous" in cap.err
    code, cap = run([f"--co={y1}"])
    assert code == 2 and "--config は省略せずに指定してください" in cap.err
    code, cap = run(["--conf", y1, "--help"])
    assert code == 2 and "--config は省略せずに指定してください" in cap.err
    assert "show this help message" not in cap.out + cap.err
    assert calls == []
    # (h) --help -- --conf は従来どおりヘルプ表示で終了コード 0
    with pytest.raises(SystemExit) as ei:
        _parse(mod, ["--help", "--", "--conf"])
    assert ei.value.code == 0
    assert "show this help message" in capsys.readouterr().out
    assert calls == []
    # (f) 対照: --ca / --cam は従来どおり通る
    if tool == "loop":
        base_f = _cli({k: v for k, v in req.items() if k != "camera"})
        with pytest.raises(SystemExit) as ei:
            _parse(mod, base_f + ["--cam", "camA"])
        assert ei.value.code == 2
        capsys.readouterr()
        a = _parse(mod, base_f + ["--camera", "camA"])
        assert a.camera == "camA"
    else:
        a = _parse(mod, base + ["--ca", "camA"])
        assert a.cameras == ["camA"]


# === 12. 後方互換 ===

EXPECTED_DEFAULTS = {
    "refine": {"cameras": None, "overwrite": False, "seed_base": rex.SEED_BASE_DEFAULT,
               "tmp_dir": None},
    "adjust": {"init_toml": None, "init_cameras": None, "cameras": None, "overwrite": False,
               "seed": aem.SEED_DEFAULT, "tmp_dir": None, "fresh_match": False},
    "loop": {"tmp_dir": None},
}


@pytest.mark.parametrize("tool", TOOL_IDS)
def test_12_backward_compat(tool):
    mod, req, _ = TOOLS[tool]
    a = _parse(mod, _cli(req))
    expected = {**req, **EXPECTED_DEFAULTS[tool]}
    assert _vars_wo_config(a) == expected
    assert a.config is None


# === 13a. main が設定値で処理に入る（refine_extrinsics） ===

def test_13a_refine_main(monkeypatch, tmp_path, capsys):
    rec: dict = {}
    proc_calls: list = []

    cams = {"camA": {"name": "camA"}, "camB": {"name": "camB"}}

    def load_cameras_toml(path):
        rec["toml"] = str(path)
        return cams

    def load3(path):
        rec["p3"] = str(path)
        return {}

    def load2(path):
        rec["p2"] = str(path)
        return {}

    def process_camera(cam_name, cam_toml, validated, gaussians, tmp_dir, seed_base_cam, report_lines):
        proc_calls.append((cam_name, seed_base_cam, Path(tmp_dir)))
        raise rex.MatcherEnvError("stop")

    def load_ply(path):
        rec["ply"] = str(path)
        return object()

    monkeypatch.setattr(rex, "load_cameras_toml", load_cameras_toml)
    monkeypatch.setattr(rex, "select_camera", lambda c, n: c[n])
    monkeypatch.setattr(rex, "load_points_3d_csv", load3)
    monkeypatch.setattr(rex, "load_points_2d_csv", load2)
    monkeypatch.setattr(rex, "validate_camera", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(rex, "process_camera", process_camera)
    dummy = types.ModuleType("render")
    dummy.load_ply = load_ply
    monkeypatch.setitem(sys.modules, "render", dummy)

    common = {"toml": "in.toml", "ply": "s.ply", "images_dir": "imgs",
              "points_3d": "p3.csv", "points_2d": "p2.csv"}
    y_tmp = tmp_path / "y_tmp"
    y_out = tmp_path / "y_out.toml"
    y_rep = tmp_path / "y_rep.txt"
    conf = _write_yaml(tmp_path / "c.yaml", {
        **common, "out_toml": str(y_out), "out_report": str(y_rep),
        "cameras": ["camB"], "seed_base": 123, "tmp_dir": str(y_tmp)})
    assert rex.main(["--config", conf]) == 1
    rec_yaml = dict(rec)
    calls_yaml = list(proc_calls)
    assert rec_yaml == {"toml": "in.toml", "p3": "p3.csv", "p2": "p2.csv", "ply": "s.ply"}
    assert calls_yaml == [("camB", 123, y_tmp)]
    rep = y_rep.read_text(encoding="utf-8")
    assert "seed_base: 123" in rep and "=== camB ===" in rep and "環境異常により中断: stop" in rep

    rec.clear()
    proc_calls.clear()
    c_tmp = tmp_path / "c_tmp"
    cli = _cli({**common, "out_toml": str(tmp_path / "c_out.toml"),
                "out_report": str(tmp_path / "c_rep.txt"),
                "cameras": ["camB"], "seed_base": 123, "tmp_dir": str(c_tmp)})
    assert rex.main(cli) == 1
    assert rec == rec_yaml
    assert proc_calls == [("camB", 123, c_tmp)]


# === 13b. main が設定値で処理に入る（adjust_extrinsics_multiview） ===

def test_13b_adjust_main(monkeypatch, tmp_path, capsys):
    work = tmp_path / "work"
    work.mkdir()
    (work / "scene.ply").write_bytes(b"")
    (work / "imgs").mkdir()
    confdir = tmp_path / "conf"
    confdir.mkdir()
    monkeypatch.chdir(work)

    calls: list = []

    def fake_build(toml, init_toml, init_cameras=None):
        calls.append((toml, init_toml, init_cameras))
        raise aem.InputError("stop")

    monkeypatch.setattr(aem, "build_camera_dict", fake_build)

    # (1) 最小構成
    conf1 = _write_yaml(confdir / "min.yaml", {
        "toml": "t.toml", "ply": "scene.ply", "images_dir": "imgs", "init_toml": "init.toml",
        "out_toml": str(tmp_path / "o1.toml"), "out_report": str(tmp_path / "r1.txt")})
    assert aem.main(["--config", conf1]) == 1
    assert "stop" in capsys.readouterr().err
    assert calls == [("t.toml", "init.toml", None)]

    # (2) 全キー
    calls.clear()
    full = {"toml": "t.toml", "ply": "scene.ply", "images_dir": "imgs", "init_toml": "init.toml",
            "init_cameras": ["camA", "camB"], "cameras": ["camA", "camB"], "seed": 9,
            "tmp_dir": str(tmp_path / "tmpd"), "fresh_match": True, "overwrite": True}
    conf2 = _write_yaml(confdir / "full.yaml", {
        **full, "out_toml": str(tmp_path / "o2.toml"), "out_report": str(tmp_path / "r2.txt")})
    assert aem.main(["--config", conf2]) == 1
    assert calls == [("t.toml", "init.toml", ["camA", "camB"])]
    rec_yaml = list(calls)
    calls.clear()
    assert aem.main(_cli({**full, "out_toml": str(tmp_path / "o3.toml"),
                          "out_report": str(tmp_path / "r3.txt")})) == 1
    assert calls == rec_yaml
    capsys.readouterr()

    # init_toml なし
    calls.clear()
    conf3 = _write_yaml(confdir / "noinit.yaml", {
        "toml": "t.toml", "ply": "scene.ply", "images_dir": "imgs",
        "out_toml": str(tmp_path / "o4.toml"), "out_report": str(tmp_path / "r4.txt")})
    assert aem.main(["--config", conf3]) == 1
    assert "--init-toml が指定されていません" in capsys.readouterr().err
    assert calls == []


# === 14〜17 の共通モック（loop） ===

def _loop_setup(monkeypatch, tmp_path, constant_params=False, camera="camA"):
    K0 = np.array([[1280.0, 0, 960.0], [0, 1280.0, 540.0], [0, 0, 1]])
    cam0 = {"K": K0, "D": np.zeros(4), "width": 1920, "height": 1080,
            "rvec": np.zeros(3), "tvec": np.zeros(3)}
    img_path = tmp_path / "camA.png"
    calls: list = []

    def mock_params(i):
        if constant_params:
            return Params(1280.0, 960.0, 540.0, 0.0, 0.0, np.zeros(3), np.zeros(3))
        return Params(1280.0 + 20.0 * (i % 2) + 0.1 * i, 960.0 + 0.01 * i, 540.0 - 0.01 * i,
                      0.001 * i, -0.001 * i,
                      np.array([0.01 * i, 0.02, 0.03]), np.array([0.1 * i, 0.2, 0.3]))

    def mock_run_loop_iteration(gaussians, params, image_path, i, tmp_dir):
        calls.append(i)
        diag = {"matcher": "LoFTR", "matches": 1000, "N": 800, "inl": 700, "rmse": 1.0,
                "cover": 16, "lsq_status": 1, "elapsed": 0.1}
        return {"ok": True, "params": mock_params(i), "diag": diag}

    monkeypatch.setattr(eil, "load_cameras_toml", lambda path: {camera: cam0})
    monkeypatch.setattr(eil, "select_camera", lambda cams, name: cams[name])
    monkeypatch.setattr(eil, "load_points_3d_csv", lambda path: {})
    monkeypatch.setattr(eil, "load_points_2d_csv", lambda path: {})
    monkeypatch.setattr(eil, "validate_camera", lambda *a, **k: {
        "ok": True, "p3": np.zeros((6, 3)), "p2": np.zeros((6, 2)),
        "names": list("abcdef"), "image_path": img_path})
    monkeypatch.setattr(eil, "pnp_manual",
                        lambda *a, **k: (np.zeros(3), np.zeros(3), np.arange(6)))
    monkeypatch.setattr(eil, "median_reproj_px", lambda *a, **k: 0.0)
    monkeypatch.setattr(eil, "run_loop_iteration", mock_run_loop_iteration)
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: object()
    monkeypatch.setitem(sys.modules, "render", dummy)
    return calls


# === 14. main の完走（loop） ===

def test_14_loop_main(monkeypatch, tmp_path):
    calls = _loop_setup(monkeypatch, tmp_path)
    conf = _write_yaml(tmp_path / "c.yaml", {
        "init_toml": "x.toml", "camera": "camA", "ply": "x.ply", "images_dir": str(tmp_path),
        "points_3d": "a.csv", "points_2d": "b.csv", "out_dir": str(tmp_path / "out"),
        "tmp_dir": str(tmp_path / "tmp")})
    assert eil.main(["--config", conf]) == 3
    assert calls == list(range(8))


# === 15. --help ===

@pytest.mark.parametrize("tool", TOOL_IDS)
def test_15_help(tool):
    mod = TOOLS[tool][0]
    h = mod.build_arg_parser().format_help()
    assert "--config" in h
    if tool in ("refine", "adjust"):
        # argparse の折り返しで空白位置が変わるため、空白を除いて比較する
        assert "true方向" in "".join(h.split())


# === 16. 内部呼び出しの不変 ===

def test_16_subprocess_command_unchanged(monkeypatch, tmp_path):
    _loop_setup(monkeypatch, tmp_path, constant_params=True)
    out_dir = tmp_path / "out"
    work_tmp = tmp_path / "wtmp"
    y = {"init_toml": "x.toml", "camera": "camA", "ply": "x.ply", "images_dir": str(tmp_path),
         "points_3d": "a.csv", "points_2d": "b.csv", "out_dir": str(out_dir),
         "tmp_dir": str(work_tmp)}
    conf = _write_yaml(tmp_path / "c.yaml", y)
    recorded: list = []

    def fake_run(cmd, **kwargs):
        recorded.append((list(cmd), kwargs))
        Path(cmd[cmd.index("--out-report") + 1]).write_text("  camA: 受理(単峰)\n", encoding="utf-8")
        src = Path(cmd[cmd.index("--toml") + 1]).read_text(encoding="utf-8")
        Path(cmd[cmd.index("--out-toml") + 1]).write_text(src, encoding="utf-8")
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(eil.subprocess, "run", fake_run)
    monkeypatch.setattr(eil, "evaluate_and_write", lambda *a, **k: {
        "n_valid": 100, "valid_ratio": 0.9, "median": 1.0, "p95": 2.0,
        "mean": 1.1, "max": 3.0, "judge": "OK"})
    assert eil.main(["--config", conf]) == 0
    assert len(recorded) == 1
    cmd, kwargs = recorded[0]
    expected = [sys.executable, str(ROOT / "phase4/refine_extrinsics.py"),
                "--toml", str(out_dir / "camA_loop.toml"), "--ply", "x.ply",
                "--images-dir", str(tmp_path), "--points-3d", "a.csv", "--points-2d", "b.csv",
                "--out-toml", str(out_dir / "camA_final.toml"),
                "--out-report", str(out_dir / "camA_refine_report.txt"),
                "--cameras", "camA", "--tmp-dir", str(work_tmp / "refine"), "--overwrite"]
    assert cmd == expected
    assert "--config" not in cmd
    assert kwargs["cwd"] == str(ROOT)


# === 17. YAML 由来の既存 out_dir の拒否 ===

def test_17_existing_out_dir_rejected(monkeypatch, tmp_path):
    calls = _loop_setup(monkeypatch, tmp_path)
    out_dir = tmp_path / "existing"
    out_dir.mkdir()
    keep = out_dir / "keep.txt"
    keep.write_text("old", encoding="utf-8")
    conf = _write_yaml(tmp_path / "c.yaml", {
        "init_toml": "x.toml", "camera": "camA", "ply": "x.ply", "images_dir": str(tmp_path),
        "points_3d": "a.csv", "points_2d": "b.csv", "out_dir": str(out_dir)})
    assert eil.main(["--config", conf]) == 1
    assert keep.read_text(encoding="utf-8") == "old"
    assert [p.name for p in out_dir.iterdir()] == ["keep.txt"]
    assert calls == []

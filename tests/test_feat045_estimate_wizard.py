"""feat-045: 推定ウィザードのテスト（GPU・実データ不要）。

設計: docs/issues/feat-045-estimate-wizard/design.md §6.1
"""
import shlex
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(Path(__file__).parent))

import estimate_intrinsics_loop as eil  # noqa: E402
import estimate_wizard as ew  # noqa: E402
import extract_photo_info as epi  # noqa: E402
import prepare_phone_photo as ppp  # noqa: E402
from refine_extrinsics import parse_args_with_config  # noqa: E402
from render_fps_video import load_yaml_flat  # noqa: E402

OK_EXIF = {"Orientation": 1, "DigitalZoomRatio": None, "Size": (64, 48)}


def _inputs(responses):
    """答えのリストを順に返す input_fn。尽きたら EOFError。"""
    it = iter(responses)

    def fn(prompt):
        try:
            return next(it)
        except StopIteration:
            raise EOFError
    return fn


def _patch_exif(monkeypatch, exif=None, focal=23):
    monkeypatch.setattr(ew, "read_exif", lambda path: dict(exif if exif is not None else OK_EXIF))
    monkeypatch.setattr(ew, "read_focal_35mm", lambda path: focal)


def _make_files(tmp_path, names2d=("A", "B", "C", "D", "E", "F"), names3d="ABCDEFG", f_y=None):
    src = tmp_path / "orig.png"
    cv2.imwrite(str(src), np.full((48, 64, 3), 200, np.uint8))
    lines = ["ObjectName,camera_name,X,Y"]
    for k, n in enumerate(names2d):
        y = 10 + k * 5
        if n == "F" and f_y is not None:
            y = f_y
        lines.append(f"{n},cam,{10 + k * 8},{y}")
    lines.append("Z,other,5,20")
    c2 = tmp_path / "p2.csv"
    c2.write_text("\n".join(lines) + "\n", encoding="utf-8")
    c3 = tmp_path / "p3.csv"
    c3.write_text("ObjectName,X,Y,Z\n" + "".join(f"{n},{i},{i},{i}\n" for i, n in enumerate(names3d)),
                  encoding="utf-8")
    ply = tmp_path / "x.ply"
    ply.write_bytes(b"ply\x00\x01\x02binary")
    return src, c2, c3, ply


def _jobs(tmp_path, monkeypatch):
    """作業フォルダの親を一時ディレクトリに差し替え、作業フォルダ job1 のパスを返す。"""
    monkeypatch.setattr(ew, "JOBS_DIR", tmp_path / "EIL")
    return tmp_path / "EIL" / "job1"


def _answers(src, name, c2, c3, ply, last="n"):
    return [str(src), name, "cam", str(c2), str(c3), "y", str(ply), last]


def test_1_draw_crop_area_small():
    img = np.full((48, 64, 3), 200, np.uint8)
    orig = img.copy()
    out = ew.draw_crop_area(img, (0, 63, 6, 41))
    assert out[0, 0, 0] == 100 and out[47, 0, 0] == 100 and out[20, 0, 0] == 200
    assert np.array_equal(img, orig)


def test_2_draw_crop_area_large():
    img = np.full((3000, 4000, 3), 200, np.uint8)
    out = ew.draw_crop_area(img, (1, 3998, 376, 2623))
    assert out[0, 0, 0] == 100 and out[2999, 0, 0] == 100
    assert out[0, 3999, 0] == 100 and out[2999, 3999, 0] == 100
    assert out[1500, 0, 0] == 100 and out[1500, 3999, 0] == 100
    assert (out[1500, 1:3999] == 200).all()
    assert tuple(out[376, 1000]) == (0, 0, 255) and tuple(out[2623, 1000]) == (0, 0, 255)


def test_3_point_label():
    assert ew.point_label("基準_001") == "001"
    assert ew.point_label("P12a3") == "3"
    assert ew.point_label("abc") == "abc"


def test_4_check_names():
    assert ew.check_names({"A": 1, "B": 1}, {"B": 1, "C": 1}) == (["B"], ["A"], ["C"])


def test_5_draw_points():
    img = np.zeros((1080, 1920, 3), np.uint8)
    out, skipped = ew.draw_points(img, {"P1": (100, 100), "P2": (5000, 5)})
    assert skipped == ["P2"]
    assert tuple(out[100, 100]) == (0, 255, 255)
    assert tuple(out[100, 108]) == (0, 0, 255)
    assert not img.any()


def test_6_camera_name_error():
    names = ["cam1", "s25_2026-11", "a/b", "..", "cam 1", "写真", "cam.1", "a\nb", "", "-cam", "metadata"]
    res = [ew.camera_name_error(n) for n in names]
    assert res[0] is None and res[1] is None
    assert all(r is not None for r in res[2:])


def test_7_ask():
    assert ew.ask(lambda p: "", "L", default="d") == "d"
    seq = iter(["", "x"])
    assert ew.ask(lambda p: next(seq), "L") == "x"


def test_8_ask_yes_no():
    seq = iter(["maybe", "Y"])
    calls = []

    def fn(p):
        calls.append(p)
        return next(seq)
    assert ew.ask_yes_no(fn, "L") is True
    assert len(calls) == 2


def test_9_job_paths_and_yaml(tmp_path):
    job = Path("/tmp/j")
    p = ew.job_paths(job, ".jpg")
    assert p["original"] == job / "original.jpg"
    assert p["crop_area"] == job / "crop_area.png"
    assert p["points_check"] == job / "points_check.png"
    assert p["points_2d_original"] == job / "points_2d_original.csv"
    assert p["points_3d"] == job / "points_3d.csv"
    assert p["ply"] == job / "point_cloud.ply"
    assert p["job_yaml"] == job / "job.yaml"
    assert p["photo_info"] == job / "photo_info.yaml"
    assert p["images_dir"] == job / "images"
    assert p["init_toml"] == job / "init.toml"
    assert p["points_2d"] == job / "points_2d.csv"
    assert p["out_dir"] == job / "out"
    text = ew.format_job_yaml("cam", p)
    lines = text.splitlines()
    assert lines[0] == "# 共用設定YAML（estimate_wizard.py が作成。feat-045）"
    assert lines[1:] == [
        "camera: cam", "image: /tmp/j/original.jpg", "photo_info: /tmp/j/photo_info.yaml",
        "points_2d_original: /tmp/j/points_2d_original.csv", "images_dir: /tmp/j/images",
        "init_toml: /tmp/j/init.toml", "points_2d: /tmp/j/points_2d.csv",
        "points_3d: /tmp/j/points_3d.csv", "ply: /tmp/j/point_cloud.ply", "out_dir: /tmp/j/out"]
    y = tmp_path / "job.yaml"
    y.write_text(text, encoding="utf-8")
    assert len(load_yaml_flat(str(y))) == 10


def test_10_yaml_accepted_by_tools(tmp_path):
    y = tmp_path / "job.yaml"
    y.write_text(ew.format_job_yaml("cam", ew.job_paths(tmp_path / "j", ".jpg")), encoding="utf-8")
    for tool in (epi, ppp, eil):
        args = parse_args_with_config(["--config", str(y)], tool.build_arg_parser(),
                                      tool.CONFIG_CONVERTERS, tool.REQUIRED_OPTIONS, tool.IGNORED_KEYS)
        assert args is not None


def test_11_manual_commands():
    cmds = ew.manual_commands(Path("/tmp/j/job.yaml"))
    assert len(cmds) == 3
    assert cmds[2].startswith('TORCH_CUDA_ARCH_LIST="9.0+PTX" ')
    assert not cmds[0].startswith("TORCH") and not cmds[1].startswith("TORCH")
    assert all(c.endswith("--config /tmp/j/job.yaml") for c in cmds)


def _runner(codes, calls):
    it = iter(codes)

    def run(args):
        calls.append(args)
        return types.SimpleNamespace(returncode=next(it))
    return run


def test_12_run_tools_ok(tmp_path):
    calls = []
    y = tmp_path / "job.yaml"
    assert ew.run_tools(y, runner=_runner([0, 0, 0], calls)) == 0
    assert calls == [[sys.executable, str(ew.PHASE4_DIR / t), "--config", str(y)] for t in ew.TOOLS]


def test_13_run_tools_fail(tmp_path, capsys):
    calls = []
    y = tmp_path / "job.yaml"
    assert ew.run_tools(y, runner=_runner([0, 1, 0], calls)) == 1
    assert len(calls) == 2
    cap = capsys.readouterr()
    assert "prepare_phone_photo.py" in cap.err and "終了コード 1" in cap.err
    cmds = [ln for ln in cap.out.splitlines() if ln.startswith(("uv ", "TORCH_"))]
    assert len(cmds) == 2
    assert "--overwrite" in shlex.split(cmds[0])
    assert shlex.split(cmds[1])[-2:] == ["--out-dir", str(tmp_path / "out_2")]


def test_13b_recovery_out_dir(tmp_path):
    (tmp_path / "out").mkdir()
    (tmp_path / "out_2").mkdir()
    y = tmp_path / "job.yaml"
    rec = ew.recovery_commands(y)
    assert shlex.split(rec[2])[-2:] == ["--out-dir", str(tmp_path / "out_3")]
    assert rec[0] == ew.manual_commands(y)[0]


def test_13c_photo_info_exists(tmp_path, capsys):
    (tmp_path / "photo_info.yaml").write_text("focal_length_35mm: 23\n", encoding="utf-8")
    calls = []
    assert ew.run_tools(tmp_path / "job.yaml", runner=_runner([1], calls)) == 1
    cap = capsys.readouterr()
    assert "写真情報ファイルは既にあります" in cap.out
    cmds = [ln for ln in cap.out.splitlines() if ln.startswith(("uv ", "TORCH_"))]
    assert len(cmds) == 2
    assert "extract_photo_info.py" not in "\n".join(cmds)
    assert "prepare_phone_photo.py" in cmds[0] and "estimate_intrinsics_loop.py" in cmds[1]


def test_14_main_no_run(tmp_path, monkeypatch):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    job = _jobs(tmp_path, monkeypatch)
    assert ew.main([], input_fn=_inputs(_answers(src, "job1", c2, c3, ply))) == 0
    for name in ["original.png", "crop_area.png", "points_check.png", "points_2d_original.csv",
                 "points_3d.csv", "point_cloud.ply", "job.yaml"]:
        assert (job / name).is_file(), name
    assert (job / "original.png").read_bytes() == src.read_bytes()
    assert (job / "points_2d_original.csv").read_bytes() == c2.read_bytes()
    assert (job / "points_3d.csv").read_bytes() == c3.read_bytes()
    assert (job / "point_cloud.ply").read_bytes() == ply.read_bytes()
    assert cv2.imread(str(job / "crop_area.png")) is not None
    assert cv2.imread(str(job / "points_check.png")) is not None


@pytest.mark.parametrize("rc", [0, 1])
def test_15_main_run(tmp_path, monkeypatch, rc):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    job = _jobs(tmp_path, monkeypatch)
    calls = []
    monkeypatch.setattr(ew, "run_tools", lambda y: calls.append(y) or rc)
    assert ew.main([], input_fn=_inputs(_answers(src, "job1", c2, c3, ply, last="y"))) == rc
    assert calls == [job / "job.yaml"]


def test_16_reask_original_job_camera(tmp_path, monkeypatch):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    job = _jobs(tmp_path, monkeypatch)
    (tmp_path / "EIL" / "old").mkdir(parents=True)
    ans = [str(tmp_path / "none.png"), str(src), "old", "job1", "a/b", "cam",
           str(c2), str(c3), "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert (job / "job.yaml").is_file()


def test_17_reask_names(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    bad2 = tmp_path / "bad2.csv"
    bad2.write_text(c2.read_text(encoding="utf-8") + "Q,cam,30,30\n", encoding="utf-8")
    _jobs(tmp_path, monkeypatch)
    ans = [str(src), "job1", "cam", str(bad2), str(c3), str(c2), "", "n", "", "", "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert "3D 基準点 CSV に無い名前" in capsys.readouterr().err


def test_17b_duplicate(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    dup = tmp_path / "dup.csv"
    dup.write_text(c2.read_text(encoding="utf-8") + "A,cam,30,30\n", encoding="utf-8")
    _jobs(tmp_path, monkeypatch)
    ans = [str(src), "job1", "cam", str(dup), str(c3), str(c2), "", "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert "エラー" in capsys.readouterr().err


def test_18_no_camera_rows(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    nocam = tmp_path / "nocam.csv"
    nocam.write_text("ObjectName,camera_name,X,Y\nA,other,10,10\n", encoding="utf-8")
    _jobs(tmp_path, monkeypatch)
    ans = [str(src), "job1", "cam", str(nocam), str(c3), str(c2), "", "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert "camera_name=cam の行がありません" in capsys.readouterr().err


def test_19_interrupt(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    job = _jobs(tmp_path, monkeypatch)
    assert ew.main([], input_fn=_inputs([str(src), "job1", "cam"])) == 1
    assert job.is_dir()
    assert "中断しました" in capsys.readouterr().out


def test_27_job_name_reask(tmp_path, monkeypatch, capsys, request):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    _jobs(tmp_path, monkeypatch)
    (tmp_path / "EIL" / "old").mkdir(parents=True)
    monkeypatch.chdir(tmp_path)
    before = {p.name for p in tmp_path.iterdir()}
    ans = [str(src), "a/b", "../x", "a b", "old", "20261126-01", "cam", str(c2), str(c3), "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert (tmp_path / "EIL" / "20261126-01" / "job.yaml").is_file()
    assert sorted(p.name for p in (tmp_path / "EIL").iterdir()) == ["20261126-01", "old"]
    assert {p.name for p in tmp_path.iterdir()} == before
    err = capsys.readouterr().err
    assert err.count("作業フォルダの名前は英数字") == 3
    assert "その名前の作業フォルダは既にあります" in err


def test_28_explanations(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    _jobs(tmp_path, monkeypatch)
    prompts = []
    inner = _inputs(_answers(src, "job1", c2, c3, ply))

    def fn(prompt):
        prompts.append((prompt, capsys.readouterr().out))
        return inner(prompt)
    assert ew.main([], input_fn=fn) == 0
    outs = {p: o for p, o in prompts}
    job_q = next(p for p in outs if p.startswith("作業フォルダの名前"))
    cam_q = next(p for p in outs if p.startswith("カメラ名"))
    ply_q = next(p for p in outs if "PLY" in p)
    assert "推定用の作業フォルダを" in outs[job_q] and str(tmp_path / "EIL") in outs[job_q]
    assert "この写真のカメラ名を決めます" in outs[cam_q]
    assert ply_q.startswith("3DGS の PLY ファイルのパス")


def test_29_job_place_error(tmp_path):
    assert ew.job_place_error(tmp_path / "EIL" / "j") is None
    assert "改行または前後の空白" in ew.job_place_error(tmp_path / "a\nb" / "j")
    assert "改行または前後の空白" in ew.job_place_error(Path(str(tmp_path / "j") + " "))


def test_29b_stops_before_create(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    monkeypatch.setattr(ew, "JOBS_DIR", tmp_path / "a\nb" / "EIL")
    assert ew.main([], input_fn=_inputs(_answers(src, "job1", c2, c3, ply))) == 1
    assert "改行または前後の空白" in capsys.readouterr().err
    assert not (tmp_path / "a\nb" / "EIL" / "job1").exists()


def test_31_csv_encoding(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    _jobs(tmp_path, monkeypatch)
    cp = tmp_path / "cp932.csv"
    cp.write_bytes("ObjectName,camera_name,X,Y\n基準_001,cam,10,10\n".encode("cp932"))
    ans = [str(src), "job1", "cam", str(cp), str(c3), str(c2), "", "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert "UTF-8 として読めません" in capsys.readouterr().err


def test_32_run_tools_keyboard_interrupt(tmp_path, capsys):
    n = []

    def run(args):
        n.append(args)
        if len(n) == 2:
            raise KeyboardInterrupt
        return types.SimpleNamespace(returncode=0)
    assert ew.run_tools(tmp_path / "job.yaml", runner=run) == 1
    out = capsys.readouterr().out
    assert "中断しました（prepare_phone_photo.py の実行中）" in out
    cmds = [ln for ln in out.splitlines() if ln.startswith(("uv ", "TORCH_"))]
    assert len(cmds) == 2
    assert "--overwrite" in shlex.split(cmds[0])
    assert shlex.split(cmds[1])[-2:] == ["--out-dir", str(tmp_path / "out_2")]


def test_33_interrupt_before_yaml(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    _jobs(tmp_path, monkeypatch)
    assert ew.main([], input_fn=_inputs([str(src), "job1", "cam"])) == 1
    assert "新しい作業フォルダ名で最初からやり直してください" in capsys.readouterr().out


def test_34_interrupt_at_run_confirm(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    job = _jobs(tmp_path, monkeypatch)
    ans = _answers(src, "job1", c2, c3, ply)[:-1]
    assert ew.main([], input_fn=_inputs(ans)) == 1
    out = capsys.readouterr().out
    assert "次のコマンドで続けられます" in out
    assert "最初からやり直してください" not in out
    for c in ew.manual_commands(job / "job.yaml"):
        assert c in out


def test_20_args():
    with pytest.raises(SystemExit) as e:
        ew.main(["--foo"])
    assert e.value.code == 2


@pytest.mark.parametrize("w,h,y0", [(4000, 3000, 375), (4000, 2252, 1)])
def test_21_valid_range(w, h, y0):
    x_min, x_max, y_min, y_max = ew.valid_range(w, h, y0)
    if h == 3000:
        assert (x_min, x_max, y_min, y_max) == (1, 3998, 376, 2623)
    xc, yc = w // 2, (y_min + y_max) // 2
    assert ew.in_range(x_min, yc, y0, w) and ew.in_range(x_max, yc, y0, w)
    assert ew.in_range(xc, y_min, y0, w) and ew.in_range(xc, y_max, y0, w)
    assert not ew.in_range(x_min - 1, yc, y0, w) and not ew.in_range(x_max + 1, yc, y0, w)
    assert not ew.in_range(xc, y_min - 1, y0, w) and not ew.in_range(xc, y_max + 1, y0, w)
    scale = ppp.OUT_W / w
    for x, y in [(x_min, yc), (x_min - 1, yc), (xc, y_min), (xc, y_min - 1), (xc, y_max), (xc, y_max + 1)]:
        u, v = ppp.convert_point(x, y, y0, scale)
        assert ew.in_range(x, y, y0, w) == (0 <= u <= ppp.OUT_W - 1 and 0 <= v <= ppp.OUT_H - 1)


def test_22_check_original(tmp_path, monkeypatch):
    src = tmp_path / "o.png"
    cv2.imwrite(str(src), np.full((48, 64, 3), 200, np.uint8))
    cases = {
        "a": ({"Orientation": 6, "DigitalZoomRatio": None, "Size": (64, 48)}, 23),
        "b": ({"Orientation": 1, "DigitalZoomRatio": 2.0, "Size": (64, 48)}, 23),
        "c": (OK_EXIF, None),
        "d": (OK_EXIF, 0),
        "e": ({"Orientation": 1, "DigitalZoomRatio": None, "Size": (32, 24)}, 23),
    }
    for key, (exif, focal) in cases.items():
        _patch_exif(monkeypatch, exif, focal)
        errors, info = ew.check_original(src)
        assert errors and info is None, key
        if key == "c":
            assert any("Exif の無い画像はウィザードでは扱えません" in e for e in errors)
    _patch_exif(monkeypatch)
    errors, info = ew.check_original(src)
    assert errors == [] and info["rng"] == (0, 63, 6, 41)
    txt = tmp_path / "t.png"
    txt.write_text("not an image", encoding="utf-8")
    errors, info = ew.check_original(txt)
    assert errors and info is None


def test_23_manual_commands_quote():
    y = Path("/tmp/camera jobs/job/job.yaml")
    cmds = ew.manual_commands(y)
    assert shlex.split(cmds[0])[-1] == str(y)
    assert shlex.split(cmds[1])[-1] == str(y)
    prefix = 'TORCH_CUDA_ARCH_LIST="9.0+PTX" '
    assert cmds[2].startswith(prefix)
    assert shlex.split(cmds[2][len(prefix):])[-1] == str(y)


def test_24_not_enough_points(tmp_path, monkeypatch, capsys):
    _patch_exif(monkeypatch)
    src, c2, c3, ply = _make_files(tmp_path)
    (tmp_path / "sub").mkdir()
    _, c2b, _, _ = _make_files(tmp_path / "sub", f_y=0)
    _jobs(tmp_path, monkeypatch)
    ans = [str(src), "job1", "cam", str(c2b), str(c3), str(c2), "", "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    cap = capsys.readouterr()
    assert "範囲外のため前処理で除かれる点: F" in cap.out
    assert "範囲内の点が 5 点です" in cap.err


def test_25_reask_original(tmp_path, monkeypatch, capsys):
    src, c2, c3, ply = _make_files(tmp_path)
    monkeypatch.setattr(ew, "read_exif", lambda path: dict(OK_EXIF))
    seq = iter([None, 23])
    monkeypatch.setattr(ew, "read_focal_35mm", lambda path: next(seq))
    _jobs(tmp_path, monkeypatch)
    ans = [str(src), str(src), "job1", "cam", str(c2), str(c3), "y", str(ply), "n"]
    assert ew.main([], input_fn=_inputs(ans)) == 0
    assert "Exif の無い画像はウィザードでは扱えません" in capsys.readouterr().err

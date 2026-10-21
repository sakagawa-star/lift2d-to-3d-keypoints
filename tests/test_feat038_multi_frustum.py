"""
feat-038: render_keypoints.py 複数人全員のFPS視錐台の同時描画のテスト

gsplat/torch を import しない範囲で、render モジュール・render_image・load_c3d_all_frames を
スタブ化し、合成TOMLで main を通して検証する（feat-037 テストと同じ方針。ヘルパーは複製）。

実行:
  uv run pytest tests/test_feat038_multi_frustum.py -v
"""

import argparse
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import render_keypoints  # noqa: E402
from render_keypoints import (  # noqa: E402
    HALPE26_NAMES,
    HEAD_JOINT_NAMES,
    KEYPOINT_NAMES,
    _build_parser,
    _fps_person_value,
    build_skeleton,
    c3d_to_calib,
    collect_head_points,
    compute_frustum_vertices,
    compute_keypoint_depth,
    compute_label_anchor,
    compute_person_depth,
    compute_visibility,
    draw_frustum,
    draw_overlay,
    draw_person_labels,
    extract_keypoints,
    load_cameras_toml,
    main,
    make_absent_frame,
    parse_args_with_config,
    project_keypoints,
    select_camera,
)
from render_fps_video import compute_fps_poses  # noqa: E402


# ========================================
# 共通ヘルパ（test_feat037_multi_person.py から複製）
# ========================================

_SYNTH_TOML = """\
[camA]
name = "camA"
size = [64, 48]
matrix = [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""

H, W = 48, 64


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _required_yaml(toml_path, camera="camA") -> str:
    return f"ply_path: dummy.ply\ntoml_path: {toml_path}\ncamera: {camera}\n"


def _make_person(labels, frame_nos, depth_m, rate=30.0, x_off=0.0, seed=0):
    """合成 C3D 1人分 (labels, frames_data, rate) を返す。"""
    rng = np.random.default_rng(seed)
    n = len(labels)
    frames = []
    for f in frame_nos:
        calib = rng.normal(size=(n, 3)) * 0.1
        calib[:, 0] += x_off
        calib[:, 2] = (depth_m - 2.0) + rng.normal(size=n) * 0.01
        raw = np.stack([calib[:, 1], calib[:, 2], calib[:, 0]], axis=-1) * 1000.0
        frames.append({"frame_no": int(f), "data": raw, "residual": np.zeros(n)})
    return list(labels), frames, rate


def _install_stubs(monkeypatch, persons: dict, depth_map_value=100.0, alpha_value=0.0):
    """render.py と render_image、load_c3d_all_frames（パスごとのデータ）をスタブ化する。"""
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: {"means": "stub"}
    dummy.print_ply_summary = lambda path, gaussians: None
    monkeypatch.setitem(sys.modules, "render", dummy)

    def _stub_render_image(gaussians, cam, near_plane, background=(0.0, 0.0, 0.0),
                           return_depth=False, distort=False):
        bgr = np.zeros((H, W, 3), dtype=np.uint8)
        if return_depth:
            depth_map = np.full((H, W), depth_map_value, dtype=np.float32)
            alpha_map = np.full((H, W), alpha_value, dtype=np.float32)
            return bgr, depth_map, alpha_map
        return bgr

    monkeypatch.setattr("render_keypoints.render_image", _stub_render_image)

    def _stub_load(path):
        return persons[path]

    monkeypatch.setattr("render_keypoints.load_c3d_all_frames", _stub_load)


def _run(tmp_path, argv_tail, extra=()):
    toml = _write_synth_toml(tmp_path)
    out = tmp_path / "out"
    rc = main(["dummy.ply", toml, *argv_tail, "--camera", "camA",
               "--output-dir", str(out), *extra])
    return rc, out


def _frustum_args(person=None):
    extra = ["--fps-frustum", "--fps-camera", "camA"]
    if person is not None:
        extra += ["--fps-person", str(person)]
    return extra


def _two_persons(frames1=(1, 2), frames2=(1, 2), depth1=2.0, depth2=3.0):
    return {
        "a.c3d": _make_person(HALPE26_NAMES, list(frames1), depth1, seed=0),
        "b.c3d": _make_person(HALPE26_NAMES, list(frames2), depth2, seed=1),
    }


def _parse_yaml(tmp_path, body, argv_tail=()):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(_required_yaml(toml) + body)
    return parse_args_with_config(["--config", str(cfg), *argv_tail])


# ========================================
# T-01〜T-03 変換関数・パーサー
# ========================================

def test_t01_fps_person_value_valid():
    assert _fps_person_value("all") == "all"
    assert _fps_person_value("2") == 2
    assert _fps_person_value("-1") == -1


@pytest.mark.parametrize("value", ["ALL", "All", "x", "1,2", " all"])
def test_t02_fps_person_value_invalid(value):
    with pytest.raises(argparse.ArgumentTypeError):
        _fps_person_value(value)


def test_t03_parser_fps_person_all():
    args = _build_parser().parse_args(["a.ply", "b.toml", "a.c3d", "--fps-person", "all"])
    assert args.fps_person == "all"


# ========================================
# T-04〜T-06 引数検証
# ========================================

def test_t04_fps_person_upper_all_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", "b.c3d", "--camera", "camA",
              "--fps-frustum", "--fps-camera", "camA", "--fps-person", "ALL"])
    assert ei.value.code == 2
    assert "--fps-person" in capsys.readouterr().err


def test_t05_all_without_frustum_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", "b.c3d", "--camera", "camA", "--fps-person", "all"])
    assert ei.value.code == 2
    assert "--fps-frustum" in capsys.readouterr().err


def test_t06_no_keypoints_with_all_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "--camera", "camA", "--no-keypoints", "--fps-person", "all"])
    assert ei.value.code == 2


# ========================================
# T-07〜T-10 全員指定の描画
# ========================================

def test_t07_collect_head_points_called_per_person(monkeypatch, tmp_path):
    labels2 = list(reversed(HALPE26_NAMES))
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(labels2, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    seen = []
    real = render_keypoints.collect_head_points

    def _spy(labels, frames_data):
        seen.append(list(labels))
        return real(labels, frames_data)

    monkeypatch.setattr("render_keypoints.collect_head_points", _spy)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 0
    assert seen == [list(HALPE26_NAMES), labels2]


def _record_draw_frustum(monkeypatch):
    calls = []
    real = render_keypoints.draw_frustum

    def _spy(image, verts, *a, **kw):
        out = real(image, verts, *a, **kw)
        calls.append((image, out))
        return out

    monkeypatch.setattr("render_keypoints.draw_frustum", _spy)
    return calls


def test_t08_draw_frustum_chained(monkeypatch, tmp_path):
    _install_stubs(monkeypatch, _two_persons())
    bgs = []
    stub_render = render_keypoints.render_image

    def _wrap(*a, **kw):
        r = stub_render(*a, **kw)
        bgs.append(r[0] if isinstance(r, tuple) else r)
        return r

    monkeypatch.setattr("render_keypoints.render_image", _wrap)
    calls = _record_draw_frustum(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 0
    assert len(bgs) == 1
    assert len(calls) == 4          # 2フレーム x 2人
    for f in range(2):
        first, second = calls[2 * f], calls[2 * f + 1]
        assert first[0] is bgs[0] or np.array_equal(first[0], bgs[0])
        assert second[0] is first[1]


def test_t09_absent_person_frustum_skipped(monkeypatch, tmp_path, capsys):
    _install_stubs(monkeypatch, _two_persons(frames1=(1, 2, 3, 4), frames2=(1, 2)))
    calls = _record_draw_frustum(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 0
    assert len(calls) == 6          # フレーム1,2は2回、3,4は1回
    out = capsys.readouterr().out
    assert "FPS視錐台 P1: 4/4 フレームに描画" in out
    assert "FPS視錐台 P2: 2/4 フレームに描画" in out


def test_t10_target_person_line(monkeypatch, tmp_path, capsys):
    _install_stubs(monkeypatch, _two_persons())
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 0
    assert "FPS視錐台: 対象人物 P1, P2" in capsys.readouterr().out


# ========================================
# T-11〜T-13b 頭部マーカー構成チェック
# ========================================

_NO_HEAD = [n for n in HALPE26_NAMES if n != "Head"]


def test_t11_all_second_person_without_head(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(_NO_HEAD, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 1
    err = capsys.readouterr().err
    assert "P2" in err
    assert "P1:" not in err


def test_t12_all_both_without_head(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(_NO_HEAD, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(_NO_HEAD, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 1
    lines = [ln for ln in capsys.readouterr().err.splitlines()
             if "頭部マーカーが入力にありません" in ln]
    assert len(lines) == 2
    assert "P1" in lines[0] and "P2" in lines[1]


def test_t13_number_spec_unaffected_by_other_person(monkeypatch, tmp_path):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(_NO_HEAD, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(1))
    assert rc == 0


def test_t13a_unknown_marker_file_reported_first(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(["Foo", "Bar"], [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(_NO_HEAD, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 1
    err = capsys.readouterr().err
    assert "既知マーカーが1つもありません" in err
    assert "P1" in err
    assert "頭部マーカーが入力にありません" not in err


def test_t13b_single_person_all_matches_person_1(monkeypatch, tmp_path, capsys):
    _install_stubs(monkeypatch, {"a.c3d": _make_person(_NO_HEAD, [1, 2], 2.0)})
    rc, _ = _run(tmp_path, ["a.c3d"], _frustum_args("all"))
    err_all = capsys.readouterr().err
    rc1, _ = _run(tmp_path, ["a.c3d"], _frustum_args(1))
    err_1 = capsys.readouterr().err
    assert rc == 1 and rc1 == 1
    assert err_all == err_1
    assert "頭部マーカーが入力にありません（--fps-frustum に必須）: Head" in err_all
    assert "P1" not in err_all


# ========================================
# T-14〜T-15 後方互換
# ========================================

def _normalize_stdout(text: str) -> str:
    return "\n".join(ln for ln in text.splitlines() if not ln.startswith("完了:"))


def test_t14_single_person_all_equals_person_1(monkeypatch, tmp_path, capsys):
    import cv2

    _install_stubs(monkeypatch, {"a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0)})
    rc, out = _run(tmp_path, ["a.c3d"], _frustum_args("all"))
    assert rc == 0
    pngs_all = [cv2.imread(str(out / f"frame_{f:06d}.png")) for f in (1, 2)]
    stdout_all = _normalize_stdout(capsys.readouterr().out)

    rc, out = _run(tmp_path, ["a.c3d"], _frustum_args(1))
    assert rc == 0
    pngs_1 = [cv2.imread(str(out / f"frame_{f:06d}.png")) for f in (1, 2)]
    stdout_1 = _normalize_stdout(capsys.readouterr().out)

    for a, b in zip(pngs_all, pngs_1):
        assert np.array_equal(a, b)
    assert stdout_all == stdout_1


def test_t15_number_spec_output_unchanged(monkeypatch, tmp_path, capsys):
    _install_stubs(monkeypatch, _two_persons())
    calls = _record_draw_frustum(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(2))
    assert rc == 0
    assert len(calls) == 2          # フレームあたり1回
    out = capsys.readouterr().out
    assert "FPS視錐台 P" not in out
    assert "FPS視錐台: 2/2 フレームに描画" in out
    assert "対象人物" not in out


# ========================================
# T-16 描画の同一性
# ========================================

def test_t16_all_matches_direct_composition(monkeypatch, tmp_path):
    import cv2

    persons = _two_persons(frames1=(1,), frames2=(1,), depth1=2.0, depth2=3.0)
    _install_stubs(monkeypatch, persons)
    rc, out = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args("all"))
    assert rc == 0
    got = cv2.imread(str(out / "frame_000001.png"))

    toml = _write_synth_toml(tmp_path)
    cam = select_camera(load_cameras_toml(toml), "camA")
    fps_cam = cam
    bg = np.zeros((H, W, 3), dtype=np.uint8)
    depth_map = np.full((H, W), 100.0, dtype=np.float32)
    alpha_map = np.zeros((H, W), dtype=np.float32)
    margin = render_keypoints.OCCLUSION_MARGIN
    near = 0.1
    head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}

    img = bg
    for path in ("a.c3d", "b.c3d"):
        labels, frames, _ = persons[path]
        head_arr = collect_head_points(labels, frames)
        c2w_b, valid, _ = compute_fps_poses(head_arr, head_idx)
        assert valid[0]
        verts = compute_frustum_vertices(
            c2w_b, fps_cam["K"], fps_cam["width"], fps_cam["height"],
            render_keypoints.FRUSTUM_DEPTH_DEFAULT)
        img = draw_frustum(img, verts[0], cam, depth_map, alpha_map, margin, near,
                           occlusion=True)

    entries = []
    for idx, path in ((1, "a.c3d"), (2, "b.c3d")):
        labels, frames, _ = persons[path]
        kpts_mm, valid = extract_keypoints(labels, frames[0]["data"], frames[0]["residual"])
        kpts_calib = c3d_to_calib(kpts_mm)
        pts2d = project_keypoints(kpts_calib, cam)
        depth_cam = compute_keypoint_depth(kpts_calib, cam)
        kp_visible = compute_visibility(pts2d, depth_cam, valid, depth_map, alpha_map,
                                        margin, near)
        entries.append({"idx": idx, "labels": labels, "kpts_calib": kpts_calib,
                        "pts2d": pts2d, "valid": valid, "kp_visible": kp_visible,
                        "depth_cam": depth_cam})
    drawable = [(compute_person_depth(e["depth_cam"], e["valid"]), e) for e in entries]
    drawable.sort(key=lambda t: (-t[0], t[1]["idx"]))
    ordered = [e for _, e in drawable]
    assert [e["idx"] for e in ordered] == [2, 1]      # 奥の P2 から描く
    for e in ordered:
        img = draw_overlay(
            img, e["kpts_calib"], e["pts2d"], e["valid"], e["kp_visible"], cam,
            build_skeleton({n for n in KEYPOINT_NAMES if n in e["labels"]}),
            depth_map, alpha_map, margin, near, occlusion=True)
    items = []
    for e in ordered:
        anchor = compute_label_anchor(e["pts2d"], e["valid"], e["depth_cam"], near)
        if anchor is not None:
            items.append((f"P{e['idx']}", anchor))
    expected = draw_person_labels(img, items)
    assert np.array_equal(got, expected)


# ========================================
# T-17〜T-19 設定YAML
# ========================================

def test_t17_yaml_fps_person_all(tmp_path):
    args = _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\nfps_person: all\n")
    assert args.fps_person == "all"


def test_t18_yaml_fps_person_invalid(tmp_path, capsys):
    with pytest.raises(SystemExit) as ei:
        _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\nfps_person: x\n")
    assert ei.value.code == 2
    assert "fps_person" in capsys.readouterr().err


def test_t19_cli_overrides_yaml_all(tmp_path):
    args = _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\nfps_person: all\n",
                       ["--fps-person", "2"])
    assert args.fps_person == 2

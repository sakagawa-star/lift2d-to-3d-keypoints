"""
feat-037: render_keypoints.py 複数人の3Dキーポイント重ね描きのテスト

gsplat/torch を import しない範囲で、render モジュール・render_image・load_c3d_all_frames を
スタブ化し、合成TOMLで main を通して検証する（既存 feat-017/034 テストと同じ方針）。

実行:
  uv run pytest tests/test_feat037_multi_person.py -v
"""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import render_keypoints  # noqa: E402
from render_keypoints import (  # noqa: E402
    HALPE26_NAMES,
    KEYPOINT_NAMES,
    NAME_TO_IDX,
    _build_parser,
    build_skeleton,
    c3d_to_calib,
    compute_keypoint_depth,
    compute_label_anchor,
    compute_person_depth,
    draw_overlay,
    draw_person_labels,
    extract_keypoints,
    main,
    parse_args_with_config,
    project_keypoints,
    select_camera,
    load_cameras_toml,
)


# ========================================
# 共通ヘルパ
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
    """合成 C3D 1人分 (labels, frames_data, rate) を返す。

    観察カメラ camA（回転なし、t=(0,0,2)、f=100）に対し、各マーカーのカメラ深度が
    約 depth_m になるように配置する（キャリブ座標 Z = depth_m - 2。C3D raw の第2成分）。
    """
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


def _record_draw_overlay(monkeypatch):
    """draw_overlay の呼び出し順に、人物の平均カメラ深度を記録する（元の関数に委譲）。"""
    calls = []
    real = render_keypoints.draw_overlay

    def _spy(image, kpts_calib, pts2d, valid, kp_visible, cam, skeleton,
             depth_map, alpha_map, margin, near_plane, occlusion=True):
        calls.append(float(np.mean(compute_keypoint_depth(kpts_calib, cam))))
        return real(image, kpts_calib, pts2d, valid, kp_visible, cam, skeleton,
                    depth_map, alpha_map, margin, near_plane, occlusion=occlusion)

    monkeypatch.setattr("render_keypoints.draw_overlay", _spy)
    return calls


def _record_labels(monkeypatch):
    """draw_person_labels に渡る items をフレームごとに記録する（元の関数に委譲）。"""
    recorded = []
    real = render_keypoints.draw_person_labels

    def _spy(image, items):
        recorded.append(list(items))
        return real(image, items)

    monkeypatch.setattr("render_keypoints.draw_person_labels", _spy)
    return recorded


# ========================================
# T-01〜T-03 パーサー
# ========================================

def test_t01_parser_two_paths():
    args = _build_parser().parse_args(["a.ply", "b.toml", "a.c3d", "b.c3d", "--camera", "camA"])
    assert args.keypoints_path == ["a.c3d", "b.c3d"]


def test_t02_parser_one_path():
    args = _build_parser().parse_args(["a.ply", "b.toml", "a.c3d"])
    assert args.keypoints_path == ["a.c3d"]


def test_t03_parser_no_keypoints():
    args = _build_parser().parse_args(["a.ply", "b.toml", "--no-keypoints"])
    assert args.keypoints_path == []


# ========================================
# T-04〜T-09 引数検証
# ========================================

def test_t04_no_keypoints_with_path_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", "--camera", "camA", "--no-keypoints"])
    assert ei.value.code == 2


def test_t05_video_mode_without_keypoints_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "--camera", "camA"])
    assert ei.value.code == 2
    assert "keypoints_path" in capsys.readouterr().err


@pytest.mark.parametrize("npz_name", ["b.npz", "B.NPZ"])
def test_t06_multi_with_npz_exits_2(tmp_path, capsys, npz_name):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", npz_name, "--camera", "camA"])
    assert ei.value.code == 2
    assert npz_name in capsys.readouterr().err


def test_t07_fps_person_without_frustum_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", "b.c3d", "--camera", "camA", "--fps-person", "1"])
    assert ei.value.code == 2
    assert "--fps-frustum" in capsys.readouterr().err


@pytest.mark.parametrize("value", ["0", "3"])
def test_t08_fps_person_out_of_range_exits_2(tmp_path, value):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "a.c3d", "b.c3d", "--camera", "camA",
              "--fps-frustum", "--fps-camera", "camA", "--fps-person", value])
    assert ei.value.code == 2


def test_t09_no_keypoints_with_fps_person_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as ei:
        main(["dummy.ply", toml, "--camera", "camA", "--no-keypoints", "--fps-person", "1"])
    assert ei.value.code == 2


# ========================================
# T-10〜T-14 main: 読み込み・検査・和集合
# ========================================

def test_t10_point_rate_mismatch_returns_1(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, rate=30.0),
        "b.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, rate=60.0),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 1
    err = capsys.readouterr().err
    assert "30.0" in err and "60.0" in err


def test_t11_second_file_without_known_marker_returns_1(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0),
        "b.c3d": _make_person(["Foo", "Bar"], [1, 2], 2.0),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 1
    assert "P2" in capsys.readouterr().err


def test_t12_duplicate_frame_number_in_multi_returns_1(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0),
        "b.c3d": _make_person(HALPE26_NAMES, [1, 2, 2], 2.0),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 1
    err = capsys.readouterr().err
    assert "重複" in err and "P2" in err


def test_t13_union_of_frames(monkeypatch, tmp_path):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2, 3], 2.0),
        "b.c3d": _make_person(HALPE26_NAMES, [3, 4, 5], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, out = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 0
    assert sorted(p.name for p in out.glob("frame_*.png")) == [
        f"frame_{i:06d}.png" for i in range(1, 6)]


def test_t14_frame_range_applies_to_union(monkeypatch, tmp_path):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2, 3], 2.0),
        "b.c3d": _make_person(HALPE26_NAMES, [3, 4, 5], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, out = _run(tmp_path, ["a.c3d", "b.c3d"], ["--start-frame", "4", "--end-frame", "9"])
    assert rc == 0
    assert sorted(p.name for p in out.glob("frame_*.png")) == [
        "frame_000004.png", "frame_000005.png"]


# ========================================
# T-15, T-16 純関数
# ========================================

def test_t15_compute_person_depth():
    depth = np.array([1.0, 3.0, 100.0])
    assert compute_person_depth(depth, np.array([True, True, False])) == pytest.approx(2.0)
    assert compute_person_depth(depth, np.array([False, False, False])) is None
    assert compute_person_depth(np.array([1.0, np.inf, 2.0]),
                                np.array([True, True, False])) is None


def _label_inputs():
    n = len(KEYPOINT_NAMES)
    pts2d = np.zeros((n, 2))
    pts2d[:, 0] = np.arange(n) + 10.0
    pts2d[:, 1] = 100.0 + np.arange(n)       # 添字が大きいほど下
    depth = np.full(n, 3.0)
    valid = np.ones(n, dtype=bool)
    return pts2d, valid, depth


def test_t16_compute_label_anchor():
    pts2d, valid, depth = _label_inputs()
    head = NAME_TO_IDX["Head"]

    # Head 有効 -> Head の位置
    assert compute_label_anchor(pts2d, valid, depth, 0.5) == (
        int(round(pts2d[head, 0])), int(round(pts2d[head, 1])))

    # Head 無効 -> v 最小の点（添字0）
    v2 = valid.copy()
    v2[head] = False
    assert compute_label_anchor(pts2d, v2, depth, 0.5) == (10, 100)

    # Head が near_plane 以下 -> v 最小の別の点
    d2 = depth.copy()
    d2[head] = 0.3
    assert compute_label_anchor(pts2d, valid, d2, 0.5) == (10, 100)

    # v 最小の点が near_plane 以下なら次の点へ
    d3 = depth.copy()
    d3[head] = 0.3
    d3[0] = 0.3
    assert compute_label_anchor(pts2d, valid, d3, 0.5) == (11, 101)

    # 候補なし
    assert compute_label_anchor(pts2d, np.zeros_like(valid), depth, 0.5) is None
    assert compute_label_anchor(pts2d, valid, np.full_like(depth, 0.1), 0.5) is None


# ========================================
# T-17〜T-22 描画順・ラベル
# ========================================

def _two_persons(depth1, depth2, frames=(1, 2)):
    return {
        "a.c3d": _make_person(HALPE26_NAMES, list(frames), depth1, seed=0),
        "b.c3d": _make_person(HALPE26_NAMES, list(frames), depth2, seed=1),
    }


def test_t17_draw_order_far_first(monkeypatch, tmp_path):
    _install_stubs(monkeypatch, _two_persons(2.0, 4.0))
    calls = _record_draw_overlay(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], ["--start-frame", "1", "--end-frame", "1"])
    assert rc == 0
    assert len(calls) == 2
    assert calls[0] == pytest.approx(4.0, abs=0.2)   # P2（遠い）が先
    assert calls[1] == pytest.approx(2.0, abs=0.2)


def test_t17_draw_order_swapped(monkeypatch, tmp_path):
    _install_stubs(monkeypatch, _two_persons(4.0, 2.0))
    calls = _record_draw_overlay(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], ["--start-frame", "1", "--end-frame", "1"])
    assert rc == 0
    assert calls[0] == pytest.approx(4.0, abs=0.2)   # P1（遠い）が先
    assert calls[1] == pytest.approx(2.0, abs=0.2)


def test_t18_equal_depth_by_person_index(monkeypatch, tmp_path):
    same = _make_person(HALPE26_NAMES, [1], 3.0, seed=0)
    persons = {"a.c3d": same, "b.c3d": same}
    _install_stubs(monkeypatch, persons)
    recorded = _record_labels(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 0
    assert [t for t, _ in recorded[0]] == ["P1", "P2"]


def test_t19_absent_or_no_valid_person_not_drawn(monkeypatch, tmp_path):
    p1 = _make_person(HALPE26_NAMES, [1, 2, 3], 2.0, seed=0)
    p2 = _make_person(HALPE26_NAMES, [2, 3], 3.0, seed=1)
    # P2 の frame 3 は全マーカー無効（residual = -1）
    p2[1][1]["residual"] = np.full(len(HALPE26_NAMES), -1.0)
    _install_stubs(monkeypatch, {"a.c3d": p1, "b.c3d": p2})
    calls = _record_draw_overlay(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"])
    assert rc == 0
    # frame1: P1 のみ(1) / frame2: 両方(2) / frame3: P1 のみ(1) -> 合計 4
    assert len(calls) == 4


def test_t20_labels_in_draw_order(monkeypatch, tmp_path):
    _install_stubs(monkeypatch, _two_persons(2.0, 4.0))
    recorded = _record_labels(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], ["--start-frame", "1", "--end-frame", "1"])
    assert rc == 0
    assert [t for t, _ in recorded[0]] == ["P2", "P1"]


def test_t21_single_person_no_labels(monkeypatch, tmp_path):
    _install_stubs(monkeypatch, {"a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0)})
    recorded = _record_labels(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d"])
    assert rc == 0
    assert recorded == []


def test_t22_label_drawn_even_when_hidden_by_3dgs(monkeypatch, tmp_path):
    # 全画素で α=1・3DGS 深度 0.5 -> 深度 2m 以上の全点が隠れる
    _install_stubs(monkeypatch, _two_persons(2.0, 4.0), depth_map_value=0.5, alpha_value=1.0)
    recorded = _record_labels(monkeypatch)
    calls = _record_draw_overlay(monkeypatch)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], ["--start-frame", "1", "--end-frame", "1"])
    assert rc == 0
    assert len(calls) == 2
    assert sorted(t for t, _ in recorded[0]) == ["P1", "P2"]


# ========================================
# T-23 draw_person_labels
# ========================================

def test_t23_draw_person_labels():
    img = np.zeros((H * 4, W * 4, 3), dtype=np.uint8)
    assert draw_person_labels(img, []) is img
    out = draw_person_labels(img, [("P1", (W * 2, H * 2))])
    assert out is not img
    assert int(np.count_nonzero(np.all(out == 255, axis=-1))) > 0
    assert int(np.count_nonzero(img)) == 0          # 入力は変更しない


# ========================================
# T-24〜T-26 視錐台の対象人物
# ========================================

def _frustum_args(person=None):
    extra = ["--fps-frustum", "--fps-camera", "camA"]
    if person is not None:
        extra += ["--fps-person", str(person)]
    return extra


def test_t24_frustum_uses_selected_person_labels(monkeypatch, tmp_path):
    labels2 = list(reversed(HALPE26_NAMES))
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(labels2, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    captured = {}
    real = render_keypoints.collect_head_points

    def _spy(labels, frames_data):
        captured["labels"] = list(labels)
        return real(labels, frames_data)

    monkeypatch.setattr("render_keypoints.collect_head_points", _spy)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(2))
    assert rc == 0
    assert captured["labels"] == labels2

    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args())    # 省略時は P1
    assert rc == 0
    assert captured["labels"] == list(HALPE26_NAMES)


def test_t25_frustum_absent_person_not_drawn(monkeypatch, tmp_path, capsys):
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2, 3, 4], 2.0, seed=0),
        "b.c3d": _make_person(HALPE26_NAMES, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(2))
    assert rc == 0
    out = capsys.readouterr().out
    assert "FPS視錐台: 2/4 フレームに描画" in out
    assert "2 フレームは非描画" in out


def test_t26_frustum_person_without_head_markers(monkeypatch, tmp_path, capsys):
    no_head = [n for n in HALPE26_NAMES if n != "Head"]
    persons = {
        "a.c3d": _make_person(HALPE26_NAMES, [1, 2], 2.0, seed=0),
        "b.c3d": _make_person(no_head, [1, 2], 3.0, seed=1),
    }
    _install_stubs(monkeypatch, persons)
    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(2))
    assert rc == 1
    assert "P2" in capsys.readouterr().err

    rc, _ = _run(tmp_path, ["a.c3d", "b.c3d"], _frustum_args(1))
    assert rc == 0


# ========================================
# T-27〜T-31 設定YAML
# ========================================

def _parse_yaml(tmp_path, body, argv_tail=()):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(_required_yaml(toml) + body)
    return parse_args_with_config(["--config", str(cfg), *argv_tail])


def test_t27_yaml_two_paths(tmp_path):
    args = _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\n")
    assert args.keypoints_path == ["a.c3d", "b.c3d"]


def test_t28_yaml_path_with_space_is_single(tmp_path):
    args = _parse_yaml(tmp_path, "keypoints_path: dir/a b.npz\n")
    assert args.keypoints_path == ["dir/a b.npz"]


@pytest.mark.parametrize("value", ["a.c3d;", "a.c3d;;b.c3d", ""])
def test_t29_yaml_empty_element_exits_2(tmp_path, capsys, value):
    with pytest.raises(SystemExit) as ei:
        _parse_yaml(tmp_path, f"keypoints_path: {value}\n")
    assert ei.value.code == 2
    assert "keypoints_path" in capsys.readouterr().err


def test_t30_yaml_fps_person(tmp_path):
    args = _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\nfps_person: 2\n")
    assert args.fps_person == 2


def test_t31_cli_path_replaces_yaml_paths(tmp_path):
    # 位置引数は ply_path toml_path keypoints_path の順に埋まるため、先頭2つも明示する
    toml = _write_synth_toml(tmp_path)
    args = _parse_yaml(tmp_path, "keypoints_path: a.c3d; b.c3d\n", ["x.ply", toml, "c.c3d"])
    assert args.keypoints_path == ["c.c3d"]


# ========================================
# T-32 1人モードの後方互換
# ========================================

def test_t32_single_person_matches_direct_draw_overlay(monkeypatch, tmp_path):
    import cv2

    person = _make_person(HALPE26_NAMES, [1], 2.0, seed=3)
    _install_stubs(monkeypatch, {"a.c3d": person})
    rc, out = _run(tmp_path, ["a.c3d"])
    assert rc == 0
    got = cv2.imread(str(out / "frame_000001.png"))

    toml = _write_synth_toml(tmp_path)
    cam = select_camera(load_cameras_toml(toml), "camA")
    labels, frames, _ = person
    kpts_mm, valid = extract_keypoints(labels, frames[0]["data"], frames[0]["residual"])
    kpts_calib = c3d_to_calib(kpts_mm)
    pts2d = project_keypoints(kpts_calib, cam)
    expected = draw_overlay(
        np.zeros((H, W, 3), dtype=np.uint8), kpts_calib, pts2d, valid,
        np.ones(len(KEYPOINT_NAMES), dtype=bool), cam,
        build_skeleton({n for n in KEYPOINT_NAMES if n in labels}),
        np.full((H, W), 100.0, dtype=np.float32), np.zeros((H, W), dtype=np.float32),
        render_keypoints.OCCLUSION_MARGIN, 0.1, occlusion=True)
    assert np.array_equal(got, expected)

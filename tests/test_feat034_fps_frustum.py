"""
feat-034: render_keypoints.py の FPSカメラ視錐台ワイヤフレーム重ね描きのテスト

gsplat/torch を import しない範囲（幾何・描画・CLI検証・YAML）で合成データを使って検証する。
c3d/torch/gsplat は render_keypoints の関数内 import なので本テストでは import 不要
（既存 feat-032/033 テストと同じ方針）。

実行:
  uv run pytest tests/test_feat034_fps_frustum.py -v
"""

import os
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (  # noqa: E402
    HALPE26_NAMES,
    HEAD_JOINT_NAMES,
    FRUSTUM_COLOR,
    collect_head_points,
    compute_fps_poses,
    compute_frustum_vertices,
    draw_frustum,
    main,
    parse_args_with_config,
)
from render_fps_video import c2w_blender_to_viewmats  # noqa: E402


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

# camA/camB でK・解像度が異なる2カメラTOML（T-19用）
_SYNTH_TOML_TWO_CAMS = """\
[camA]
name = "camA"
size = [64, 48]
matrix = [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]

[camB]
name = "camB"
size = [32, 24]
matrix = [[50.0, 0.0, 16.0], [0.0, 50.0, 12.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 3.0]
"""

# camC のみを持つ別TOML（--fps-toml 用、T-20/T-22/T-23）。K・解像度は camA/camB と異なる
_SYNTH_TOML_FPS_ONLY = """\
[camC]
name = "camC"
size = [16, 12]
matrix = [[25.0, 0.0, 8.0], [0.0, 25.0, 6.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 4.0]
"""

# TOMLとして解釈できない不正な内容（T-22用）
_INVALID_TOML = "this is not valid toml [[[\n"


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _write_two_cam_toml(tmp_path) -> str:
    p = tmp_path / "synth_two.toml"
    p.write_text(_SYNTH_TOML_TWO_CAMS)
    return str(p)


def _write_fps_only_toml(tmp_path) -> str:
    p = tmp_path / "synth_fps.toml"
    p.write_text(_SYNTH_TOML_FPS_ONLY)
    return str(p)


def _write_invalid_toml(tmp_path) -> str:
    p = tmp_path / "invalid.toml"
    p.write_text(_INVALID_TOML)
    return str(p)


def _required_yaml(toml_path, camera="camA") -> str:
    """必須3項目（ply_path, toml_path, camera）を満たす最小の設定YAML断片。"""
    return f"ply_path: dummy.ply\ntoml_path: {toml_path}\ncamera: {camera}\n"


def _make_npz(tmp_path, name="synth.npz", F=3, joint_names=None, start_frame=1, seed=0):
    """合成 NPZ（既定: Halpe26 全26関節、頭部7点を含む）を tmp_path に書き出し、パスを返す。"""
    if joint_names is None:
        joint_names = list(HALPE26_NAMES)
    J = len(joint_names)
    rng = np.random.default_rng(seed)
    x3d_world = rng.normal(size=(F, J, 3)).astype(np.float64) * 0.1
    frame_ids = np.arange(start_frame, start_frame + F, dtype=np.int64)
    path = tmp_path / name
    np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=np.array(joint_names))
    return path


def _install_render_stub(monkeypatch, H=48, W=64):
    """render.py（torch top-level import）をダミー化し、render_image をスタブ化する。"""
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: {"means": "stub"}
    dummy.print_ply_summary = lambda path, gaussians: None
    monkeypatch.setitem(sys.modules, "render", dummy)

    def _stub_render_image(
        gaussians, cam, near_plane, background=(0.0, 0.0, 0.0),
        return_depth=False, distort=False,
    ):
        bgr = np.zeros((H, W, 3), dtype=np.uint8)
        if return_depth:
            depth_map = np.full((H, W), 100.0, dtype=np.float32)
            alpha_map = np.zeros((H, W), dtype=np.float32)  # 全画素 3DGSなし扱い
            return bgr, depth_map, alpha_map
        return bgr

    monkeypatch.setattr("render_keypoints.render_image", _stub_render_image)


# ========================================
# T-01〜T-05 compute_frustum_vertices（FR-001/003/004）
# ========================================

def test_t01_identity_pose_apex_and_corner():
    K = np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    c2w_b = np.eye(4, dtype=np.float64)[None, :, :]
    verts = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=0.5)

    assert verts.shape == (1, 5, 3)
    assert np.allclose(verts[0, 0], [0.0, 0.0, 0.0])
    # ピクセル(0,0)の隅（インデックス1） = 0.5・(−1.0, +0.5, −1.0)
    assert np.allclose(verts[0, 1], [-0.5, 0.25, -0.5], atol=1e-9)


def test_t02_apex_matches_translation():
    K = np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    c2w_b = np.eye(4, dtype=np.float64)[None, :, :].copy()
    c2w_b[0, :3, 3] = [1.0, 2.0, 3.0]
    verts = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=0.5)
    assert np.allclose(verts[0, 0], [1.0, 2.0, 3.0])


def test_t03_far_corners_at_depth_along_optical_axis():
    import cv2

    K = np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    rvec = np.array([0.3, -0.2, 0.5])
    R, _ = cv2.Rodrigues(rvec)
    c2w_b = np.eye(4, dtype=np.float64)[None, :, :].copy()
    c2w_b[0, :3, :3] = R
    c2w_b[0, :3, 3] = [1.0, 2.0, 3.0]
    depth = 0.7

    verts = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=depth)
    viewmats = c2w_blender_to_viewmats(c2w_b)  # (1,4,4) float32、world→camera(OpenCV)

    viewmat = viewmats[0].astype(np.float64)
    for corner in verts[0, 1:]:
        p_h = np.append(corner, 1.0)
        cam_coord = viewmat @ p_h
        assert cam_coord[2] == pytest.approx(depth, abs=1e-4)


def test_t04_invalid_frame_all_nan():
    K = np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    c2w_b = np.full((1, 4, 4), np.nan, dtype=np.float64)
    verts = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=0.5)
    assert verts.shape == (1, 5, 3)
    assert np.isnan(verts).all()


def test_t05_depth_doubling_doubles_displacement():
    K = np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]])
    c2w_b = np.eye(4, dtype=np.float64)[None, :, :]

    verts1 = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=0.5)
    verts2 = compute_frustum_vertices(c2w_b, K, width=200, height=100, depth=1.0)

    disp1 = verts1[0, 1:] - verts1[0, 0]
    disp2 = verts2[0, 1:] - verts2[0, 0]
    assert np.allclose(disp2, disp1 * 2.0, atol=1e-9)


# ========================================
# T-06〜T-11b CLI検証（FR-007）
# ========================================

def test_t06_fps_frustum_without_camera_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as exc_info:
        main(["a.ply", toml, "kp.npz", "--camera", "camA", "--fps-frustum"])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "--fps-camera" in err


@pytest.mark.parametrize("extra_args", [
    ["--fps-camera", "camA"],
    ["--frustum-depth", "1.0"],
])
def test_t07_fps_options_without_frustum_exits_2(tmp_path, capsys, extra_args):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as exc_info:
        main(["a.ply", toml, "kp.npz", "--camera", "camA", *extra_args])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "--fps-frustum" in err


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "-inf"])
def test_t08_frustum_depth_invalid_exits_2(tmp_path, value):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as exc_info:
        main(["a.ply", toml, "kp.npz", "--camera", "camA",
              "--fps-frustum", "--fps-camera", "camA", "--frustum-depth", value])
    assert exc_info.value.code == 2


def test_t09_no_keypoints_with_fps_frustum_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    with pytest.raises(SystemExit) as exc_info:
        main(["a.ply", toml, "--camera", "camA", "--no-keypoints",
              "--fps-frustum", "--fps-camera", "camA"])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "--no-keypoints" in err
    assert "--fps-frustum" in err


def test_t10_fps_camera_not_in_toml_returns_1(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    rc = main(["a.ply", toml, "kp.npz", "--camera", "camA",
               "--fps-frustum", "--fps-camera", "camZ"])
    assert rc == 1
    err = capsys.readouterr().err
    assert "camZ" in err
    assert "camA" in err  # 利用可能なカメラ名一覧に含まれる


def test_t11_missing_head_marker_returns_1(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    joint_names = [n for n in HALPE26_NAMES if n != "Nose"]
    npz_path = _make_npz(tmp_path, joint_names=joint_names, F=2)

    rc = main(["a.ply", toml, str(npz_path), "--camera", "camA",
               "--fps-frustum", "--fps-camera", "camA"])
    assert rc == 1
    err = capsys.readouterr().err
    assert "Nose" in err


def test_t11b_missing_head_marker_without_fps_frustum_ok(monkeypatch, tmp_path):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)
    joint_names = [n for n in HALPE26_NAMES if n != "Nose"]
    npz_path = _make_npz(tmp_path, joint_names=joint_names, F=2)
    outdir = tmp_path / "out"

    rc = main(["dummy.ply", toml, str(npz_path), "--camera", "camA",
               "--output-dir", str(outdir)])
    assert rc == 0
    pngs = list(outdir.glob("*.png"))
    assert len(pngs) == 2


# ========================================
# T-12〜T-13 設定YAML（FR-006）
# ========================================

def test_t12_yaml_fps_frustum_keys_applied(tmp_path):
    toml = _write_synth_toml(tmp_path)
    fps_toml = _write_fps_only_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        _required_yaml(toml)
        + f"fps_frustum: true\nfps_camera: camC\nfps_toml: {fps_toml}\nfrustum_depth: 0.3\n"
    )
    args = parse_args_with_config(["--config", str(cfg)])
    assert args.fps_frustum is True
    assert args.fps_camera == "camC"
    assert args.fps_toml == fps_toml
    assert args.frustum_depth == pytest.approx(0.3)


def test_t13_frustum_depth_cli_overrides_yaml(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "frustum_depth: 0.3\n")
    args = parse_args_with_config(["--config", str(cfg), "--frustum-depth", "1.0"])
    assert args.frustum_depth == pytest.approx(1.0)


# ========================================
# T-14 collect_head_points → compute_fps_poses の無効判定（FR-002）
# ========================================

def test_t14_collect_head_points_nan_propagates_to_invalid():
    labels = HALPE26_NAMES
    n = len(labels)
    data = np.zeros((n, 3), dtype=np.float64)
    residual = np.zeros(n, dtype=np.float64)
    nose_c3d_idx = labels.index("Nose")
    residual[nose_c3d_idx] = -1.0  # Noseを無効化（欠損想定）
    frames_data = [{"frame_no": 1, "data": data, "residual": residual}]

    head_arr = collect_head_points(labels, frames_data)
    nose_head_idx = HEAD_JOINT_NAMES.index("Nose")
    assert np.isnan(head_arr[0, nose_head_idx]).all()

    head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}
    c2w_b, valid, _ = compute_fps_poses(head_arr, head_idx)
    assert valid[0] == False  # noqa: E712


# ========================================
# T-15〜T-16 draw_frustum（FR-001/005）
# ========================================

def _frustum_test_verts():
    """draw_frustum テスト用の5頂点（apex z=1、遠端矩形 z=2 の小さな四角錐）。"""
    return np.array([
        [0.0, 0.0, 1.0],
        [-0.3, -0.3, 2.0],
        [0.3, -0.3, 2.0],
        [0.3, 0.3, 2.0],
        [-0.3, 0.3, 2.0],
    ])


def _frustum_test_cam(W=200, H=100):
    return {
        "K": np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]]),
        "rvec": np.zeros(3),
        "tvec": np.zeros(3),
        "width": W, "height": H,
    }


def _count_frustum_pixels(image: np.ndarray) -> int:
    return int(np.all(image == np.array(FRUSTUM_COLOR), axis=-1).sum())


def test_t15_draw_frustum_no_occlusion_adds_magenta():
    H, W = 100, 200
    image = np.zeros((H, W, 3), dtype=np.uint8)
    cam = _frustum_test_cam(W, H)
    verts = _frustum_test_verts()

    out = draw_frustum(image, verts, cam, None, None, 0.05, 0.1, occlusion=False)
    assert _count_frustum_pixels(out) > 0


def test_t16_draw_frustum_occlusion_hidden_and_visible():
    H, W = 100, 200
    image = np.zeros((H, W, 3), dtype=np.uint8)
    cam = _frustum_test_cam(W, H)
    verts = _frustum_test_verts()

    # 全サンプルが深度マップより奥（隠れる）→描画なし
    depth_map_near = np.full((H, W), 0.5, dtype=np.float32)
    alpha_map_high = np.ones((H, W), dtype=np.float32)
    out_hidden = draw_frustum(
        image, verts, cam, depth_map_near, alpha_map_high, 0.05, 0.1, occlusion=True
    )
    assert _count_frustum_pixels(out_hidden) == 0

    # 全サンプルが深度マップより手前→描画あり
    depth_map_far = np.full((H, W), 100.0, dtype=np.float32)
    out_visible = draw_frustum(
        image, verts, cam, depth_map_far, alpha_map_high, 0.05, 0.1, occlusion=True
    )
    assert _count_frustum_pixels(out_visible) > 0


# ========================================
# T-17 既定動作の不変（FR-009）
# ========================================

def test_t17_default_namespace_no_fps_frustum():
    args = parse_args_with_config(["a.ply", "b.toml", "c.npz", "--camera", "camA"])
    assert args.fps_frustum is False
    assert args.fps_camera is None
    assert args.frustum_depth is None


# ========================================
# T-19 FPSカメラのK・解像度が実際に使われる（FR-003）
# ========================================

def test_t19_fps_camera_intrinsics_used_not_observer(monkeypatch, tmp_path, capsys):
    _install_render_stub(monkeypatch)
    toml = _write_two_cam_toml(tmp_path)
    npz_path = _make_npz(tmp_path, F=2, joint_names=HALPE26_NAMES)
    outdir = tmp_path / "out"

    real_compute = compute_frustum_vertices
    captured = {}

    def _spy(c2w_b, K_fps, width, height, depth):
        captured["K"] = np.array(K_fps)
        captured["width"] = width
        captured["height"] = height
        return real_compute(c2w_b, K_fps, width, height, depth)

    monkeypatch.setattr("render_keypoints.compute_frustum_vertices", _spy)

    rc = main(["dummy.ply", toml, str(npz_path), "--camera", "camA",
               "--fps-frustum", "--fps-camera", "camB",
               "--output-dir", str(outdir)])
    assert rc == 0

    assert captured["width"] == 32
    assert captured["height"] == 24
    assert np.allclose(
        captured["K"], [[50.0, 0.0, 16.0], [0.0, 50.0, 12.0], [0.0, 0.0, 1.0]]
    )

    out = capsys.readouterr().out
    assert "camB" in out
    assert "32x24" in out


# ========================================
# T-20〜T-23 --fps-toml（イテレーション1、FR-003）
# ========================================

def test_t20_fps_toml_separate_file_intrinsics_used(monkeypatch, tmp_path, capsys):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)          # camA のみ（観察カメラ）
    fps_toml = _write_fps_only_toml(tmp_path)   # camC のみ（別TOML）
    npz_path = _make_npz(tmp_path, F=2, joint_names=HALPE26_NAMES)
    outdir = tmp_path / "out"

    real_compute = compute_frustum_vertices
    captured = {}

    def _spy(c2w_b, K_fps, width, height, depth):
        captured["K"] = np.array(K_fps)
        captured["width"] = width
        captured["height"] = height
        return real_compute(c2w_b, K_fps, width, height, depth)

    monkeypatch.setattr("render_keypoints.compute_frustum_vertices", _spy)

    rc = main(["dummy.ply", toml, str(npz_path), "--camera", "camA",
               "--fps-frustum", "--fps-toml", fps_toml, "--fps-camera", "camC",
               "--output-dir", str(outdir)])
    assert rc == 0

    assert captured["width"] == 16
    assert captured["height"] == 12
    assert np.allclose(
        captured["K"], [[25.0, 0.0, 8.0], [0.0, 25.0, 6.0], [0.0, 0.0, 1.0]]
    )

    out = capsys.readouterr().out
    assert "camC" in out
    assert "16x12" in out


def test_t21_fps_toml_without_fps_frustum_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    fps_toml = _write_fps_only_toml(tmp_path)
    with pytest.raises(SystemExit) as exc_info:
        main(["a.ply", toml, "kp.npz", "--camera", "camA", "--fps-toml", fps_toml])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "--fps-frustum" in err


def test_t22_fps_toml_not_found_returns_1(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    missing_path = str(tmp_path / "does_not_exist.toml")
    rc = main(["a.ply", toml, "kp.npz", "--camera", "camA",
               "--fps-frustum", "--fps-toml", missing_path, "--fps-camera", "camC"])
    assert rc == 1
    err = capsys.readouterr().err
    assert missing_path in err


def test_t22_fps_toml_unparseable_returns_1(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    invalid_path = _write_invalid_toml(tmp_path)
    rc = main(["a.ply", toml, "kp.npz", "--camera", "camA",
               "--fps-frustum", "--fps-toml", invalid_path, "--fps-camera", "camC"])
    assert rc == 1
    err = capsys.readouterr().err
    assert invalid_path in err


def test_t23_yaml_fps_toml_intrinsics_used(monkeypatch, tmp_path, capsys):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)          # camA のみ（観察カメラ）
    fps_toml = _write_fps_only_toml(tmp_path)   # camC のみ（別TOML）
    npz_path = _make_npz(tmp_path, F=2, joint_names=HALPE26_NAMES)
    outdir = tmp_path / "out"
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        _required_yaml(toml)
        + f"fps_frustum: true\nfps_camera: camC\nfps_toml: {fps_toml}\n"
        + f"output_dir: {outdir}\nkeypoints_path: {npz_path}\n"
    )

    real_compute = compute_frustum_vertices
    captured = {}

    def _spy(c2w_b, K_fps, width, height, depth):
        captured["K"] = np.array(K_fps)
        captured["width"] = width
        captured["height"] = height
        return real_compute(c2w_b, K_fps, width, height, depth)

    monkeypatch.setattr("render_keypoints.compute_frustum_vertices", _spy)

    rc = main(["--config", str(cfg)])
    assert rc == 0

    assert captured["width"] == 16
    assert captured["height"] == 12
    assert np.allclose(
        captured["K"], [[25.0, 0.0, 8.0], [0.0, 25.0, 6.0], [0.0, 0.0, 1.0]]
    )

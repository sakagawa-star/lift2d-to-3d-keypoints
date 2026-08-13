"""
feat-032: render_keypoints.py の NPZ 入力対応のテスト

bpy/CUDA/c3d 非依存の純粋ロジック（NPZ読み込み・検証・座標変換・residual合成・
拡張子ディスパッチ・CLIパーサー）を合成データ／モックで検証する。
c3d/torch/gsplat は render_keypoints の関数内 import なので本テストでは import 不要。
"""

import os
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (
    HALPE26_NAMES,
    KEYPOINT_NAMES,
    _build_parser,
    c3d_to_calib,
    extract_keypoints,
    load_npz_all_frames,
    main,
)


_SYNTH_TOML = """\
[camA]
name = "camA"
size = [64, 48]
matrix = [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


def _make_npz(
    tmp_path,
    name="synth.npz",
    F=3,
    J=22,
    start_frame=1,
    joint_names=None,
    pnp_ok_all_false=False,
    nan_at=None,
    seed=0,
):
    """合成 NPZ を tmp_path に書き出し、(path, x3d_world, frame_ids, joint_names) を返す。"""
    if joint_names is None:
        joint_names = list(KEYPOINT_NAMES[:J])
    rng = np.random.default_rng(seed)
    x3d_world = rng.normal(size=(F, J, 3)).astype(np.float64)
    if nan_at is not None:
        fi, ji = nan_at
        x3d_world[fi, ji, 0] = np.nan
    frame_ids = np.arange(start_frame, start_frame + F, dtype=np.int64)
    pnp_ok = np.zeros(F, dtype=bool) if pnp_ok_all_false else np.ones(F, dtype=bool)
    path = tmp_path / name
    np.savez(
        path,
        x3d_world=x3d_world,
        frame_ids=frame_ids,
        joint_names=np.array(joint_names),
        pnp_ok=pnp_ok,
    )
    return path, x3d_world, frame_ids, joint_names


def _write_broken_npz(path, kind):
    """異常系 NPZ を path に書き出す。"""
    if kind == "missing_key":
        x3d_world = np.zeros((2, 3, 3), dtype=np.float64)
        frame_ids = np.array([1, 2])
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids)  # joint_names 欠落
    elif kind == "bad_shape":
        x3d_world = np.zeros((2, 3), dtype=np.float64)  # 2次元（期待は3次元）
        frame_ids = np.array([1, 2])
        joint_names = np.array(["a", "b", "c"])
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
    elif kind == "frame_ids_len_mismatch":
        x3d_world = np.zeros((2, 3, 3), dtype=np.float64)
        frame_ids = np.array([1, 2, 3])  # F=2 だが長さ3
        joint_names = np.array(["a", "b", "c"])
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
    elif kind == "joint_names_len_mismatch":
        x3d_world = np.zeros((2, 3, 3), dtype=np.float64)
        frame_ids = np.array([1, 2])
        joint_names = np.array(["a", "b"])  # J=3 だが長さ2
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
    elif kind == "non_consecutive":
        x3d_world = np.zeros((3, 2, 3), dtype=np.float64)
        frame_ids = np.array([1, 3, 5])  # 非連番
        joint_names = np.array(["a", "b"])
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
    elif kind == "zero_frames":
        x3d_world = np.zeros((0, 2, 3), dtype=np.float64)
        frame_ids = np.zeros((0,), dtype=np.int64)
        joint_names = np.array(["a", "b"])
        np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
    elif kind == "corrupt":
        with open(path, "wb") as f:
            f.write(b"\x00\x01this is not a valid npz or pickle stream at all\xff\xfe" * 5)
    else:
        raise AssertionError(f"未知のkind: {kind}")


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
# #1 load_npz_all_frames の基本動作（FR-002/005）
# ========================================

def test_load_npz_all_frames_basic(tmp_path):
    path, x3d_world, frame_ids, joint_names = _make_npz(
        tmp_path, F=3, J=22, start_frame=145599
    )
    labels, frames_data, point_rate = load_npz_all_frames(str(path))

    assert labels == joint_names
    assert point_rate == 0.0
    assert len(frames_data) == 3
    for i, fr in enumerate(frames_data):
        assert fr["frame_no"] == int(frame_ids[i])
        assert fr["data"].shape == (22, 3)
        assert fr["data"].dtype == np.float64
        assert fr["residual"].shape == (22,)


# ========================================
# #2 world座標の往復等価性（FR-003）
# ========================================

def test_world_roundtrip_equivalence(tmp_path):
    path, x3d_world, frame_ids, joint_names = _make_npz(tmp_path, F=3, J=22)
    labels, frames_data, _ = load_npz_all_frames(str(path))

    for i, fr in enumerate(frames_data):
        calib = c3d_to_calib(fr["data"])
        assert np.allclose(calib, x3d_world[i], atol=1e-9)


# ========================================
# #3 NaN関節のresidual合成（FR-004）
# ========================================

def test_nan_joint_residual(tmp_path):
    J = 22
    path, x3d_world, frame_ids, joint_names = _make_npz(
        tmp_path, F=2, J=J, nan_at=(0, 5)
    )
    labels, frames_data, _ = load_npz_all_frames(str(path))

    fr0 = frames_data[0]
    assert fr0["residual"][5] == -1.0
    others = [i for i in range(J) if i != 5]
    assert np.all(fr0["residual"][others] == 0.0)
    assert np.all(frames_data[1]["residual"] == 0.0)

    kpts, valid = extract_keypoints(labels, fr0["data"], fr0["residual"])
    marker_name = labels[5]
    idx_in_keypoints = KEYPOINT_NAMES.index(marker_name)
    assert valid[idx_in_keypoints] == False  # noqa: E712 (bool型を明示比較)


# ========================================
# #4 pnp_ok を無視する（FR-004）
# ========================================

def test_pnp_ok_ignored(tmp_path):
    path, x3d_world, frame_ids, joint_names = _make_npz(
        tmp_path, F=3, J=22, pnp_ok_all_false=True
    )
    labels, frames_data, _ = load_npz_all_frames(str(path))

    for fr in frames_data:
        assert np.all(fr["residual"] == 0.0)


# ========================================
# #5 F=0 でエラー（FR-002）
# ========================================

def test_empty_frames_error(tmp_path):
    J = 5
    x3d_world = np.zeros((0, J, 3), dtype=np.float64)
    frame_ids = np.zeros((0,), dtype=np.int64)
    joint_names = np.array([f"j{i}" for i in range(J)])
    path = tmp_path / "empty.npz"
    np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)

    with pytest.raises(ValueError) as excinfo:
        load_npz_all_frames(str(path))
    assert "フレームがありません" in str(excinfo.value)


# ========================================
# #6 main() 経由の異常系NPZ（FR-002、parametrize 7ケース）
# ========================================

@pytest.mark.parametrize("kind", [
    "missing_key",
    "bad_shape",
    "frame_ids_len_mismatch",
    "joint_names_len_mismatch",
    "non_consecutive",
    "zero_frames",
    "corrupt",
])
def test_invalid_npz_exit_code(tmp_path, capsys, kind):
    toml = _write_synth_toml(tmp_path)
    npz_path = tmp_path / "broken.npz"
    _write_broken_npz(npz_path, kind)

    rc = main(["a.ply", toml, str(npz_path), "--camera", "camA"])
    assert rc == 1
    assert capsys.readouterr().err.strip() != ""


# ========================================
# #7 拡張子によるディスパッチ（FR-001）
# ========================================

def test_dispatch_by_extension(monkeypatch, tmp_path):
    toml = _write_synth_toml(tmp_path)
    calls = []

    def _fake_npz(path):
        calls.append(("npz", path))
        raise ValueError("SENTINEL")

    def _fake_c3d(path):
        calls.append(("c3d", path))
        raise ValueError("SENTINEL")

    monkeypatch.setattr("render_keypoints.load_npz_all_frames", _fake_npz)
    monkeypatch.setattr("render_keypoints.load_c3d_all_frames", _fake_c3d)

    rc = main(["a.ply", toml, "x.npz", "--camera", "camA"])
    assert rc == 1
    assert calls == [("npz", "x.npz")]

    calls.clear()
    rc = main(["a.ply", toml, "x.NPZ", "--camera", "camA"])
    assert rc == 1
    assert calls == [("npz", "x.NPZ")]

    calls.clear()
    rc = main(["a.ply", toml, "x.c3d", "--camera", "camA"])
    assert rc == 1
    assert calls == [("c3d", "x.c3d")]


# ========================================
# #8 main() NPZ end-to-end（FR-001/005）
# ========================================

def test_main_npz_end_to_end(monkeypatch, tmp_path):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)

    path1, *_ = _make_npz(
        tmp_path, name="a.npz", F=3, J=26, start_frame=145599, joint_names=HALPE26_NAMES
    )
    outdir1 = tmp_path / "out1"
    rc = main(["dummy.ply", toml, str(path1), "--camera", "camA", "--output-dir", str(outdir1)])
    assert rc == 0
    pngs = sorted(p.name for p in outdir1.glob("*.png"))
    assert pngs == ["frame_145599.png", "frame_145600.png", "frame_145601.png"]

    path2, *_ = _make_npz(
        tmp_path, name="b.npz", F=5, J=26, start_frame=145599, joint_names=HALPE26_NAMES
    )
    outdir2 = tmp_path / "out2"
    rc2 = main(["dummy.ply", toml, str(path2), "--camera", "camA", "--output-dir", str(outdir2),
                "--start-frame", "145600", "--end-frame", "145602"])
    assert rc2 == 0
    pngs2 = sorted(p.name for p in outdir2.glob("*.png"))
    assert pngs2 == ["frame_145600.png", "frame_145601.png", "frame_145602.png"]


# ========================================
# #9 MP4 fps のフォールバック・上書き（FR-006）
# ========================================

def test_mp4_fps_fallback_and_override(monkeypatch, tmp_path, capsys):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)
    path, *_ = _make_npz(tmp_path, F=2, J=26, start_frame=1, joint_names=HALPE26_NAMES)

    captured = {}

    def _fake_start_ffmpeg(output_dir, width, height, fps):
        captured["fps"] = fps
        proc = types.SimpleNamespace(
            stdin=types.SimpleNamespace(closed=False, write=lambda d: None, close=lambda: None),
            stderr=types.SimpleNamespace(read=lambda: b""),
            returncode=0,
            wait=lambda: 0,
        )
        return proc, os.path.join(output_dir, "output.mp4")

    monkeypatch.setattr("render_keypoints.start_ffmpeg", _fake_start_ffmpeg)

    outdir1 = tmp_path / "out1"
    rc = main(["dummy.ply", toml, str(path), "--camera", "camA", "--output-dir", str(outdir1),
               "--mp4", "--mp4-fps", "25"])
    assert rc == 0
    assert captured["fps"] == pytest.approx(25.0)

    captured.clear()
    outdir2 = tmp_path / "out2"
    rc2 = main(["dummy.ply", toml, str(path), "--camera", "camA", "--output-dir", str(outdir2),
                "--mp4"])
    assert rc2 == 0
    assert captured["fps"] == pytest.approx(30.0)
    err = capsys.readouterr().err
    assert "フレームレート情報がない" in err


# ========================================
# #10 ファイル不存在で終了コード1（FR-002）
# ========================================

def test_missing_file_returns_1(tmp_path):
    toml = _write_synth_toml(tmp_path)

    rc_npz = main(["a.ply", toml, str(tmp_path / "missing.npz"), "--camera", "camA"])
    assert rc_npz == 1

    rc_c3d = main(["a.ply", toml, str(tmp_path / "missing.c3d"), "--camera", "camA"])
    assert rc_c3d == 1


# ========================================
# #11 --help に keypoints_path が表示される（FR-007）
# ========================================

def test_help_shows_keypoints_path():
    help_text = _build_parser().format_help()
    assert "keypoints_path" in help_text

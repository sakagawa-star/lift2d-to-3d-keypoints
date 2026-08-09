"""feat-027 render_fps_video.py のテスト。

GPU・torch・gsplat 不要の純ロジック（ポーズ計算・チャンク計画・マニフェスト・
耐久書き出し・CLI検証・ダンプモード）を中心に検証する。ffmpeg/ffprobe を要する
スモークテスト（T14/T16/T17）は libx264 対応 ffmpeg（または ffprobe）不在時に skip する。

実行:
  uv run --project phase4 --with pytest pytest tests/test_feat027_render_fps_video.py -v
  uv run pytest -v   （root環境。torch/tomli非依存のテストはそのまま走る）
"""

import json
import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

import render_fps_video  # noqa: E402
from render_fps_video import (  # noqa: E402
    HEAD_JOINT_NAMES,
    build_manifest,
    c2w_blender_to_viewmats,
    chunk_filename,
    compute_fps_poses,
    concat_chunks,
    durable_replace,
    ffmpeg_has_libx264,
    format_render_average,
    load_intrinsics,
    load_manifest,
    main,
    manifest_mismatch_keys,
    plan_chunks,
    resolve_head_indices,
    start_ffmpeg_chunk,
    verify_chunk_mp4,
    write_manifest,
)


def _ffmpeg_available_with_libx264() -> bool:
    """ffmpegバイナリが存在し、かつlibx264エンコーダを実際に使えるかを確認する。

    ローカル環境によってはffmpegバイナリはあってもGPL非同梱ビルドでlibx264が
    使えないことがあるため、バイナリ存在確認（shutil.which）だけでは不十分。
    render_fps_video.ffmpeg_has_libx264（実装本体）をそのまま流用する。
    """
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin is None:
        return False
    try:
        return ffmpeg_has_libx264(ffmpeg_bin)
    except (OSError, subprocess.TimeoutExpired):
        return False


_FFMPEG_AVAILABLE = _ffmpeg_available_with_libx264()
_FFPROBE_AVAILABLE = shutil.which("ffprobe") is not None


# === 合成データヘルパ ===

# T1で手計算した「標準姿勢」: eyes_mid=(0,0,0), r0=(-1,0,0), Nose方向=+Y, Head-Neck=+Z
_DEFAULT_POSE = {
    "LEye": (-1.0, 0.0, 0.0), "REye": (1.0, 0.0, 0.0),
    "LEar": (-1.0, 0.0, 0.0), "REar": (1.0, 0.0, 0.0),
    "Nose": (0.0, 1.0, 0.0), "Head": (0.0, 0.0, 1.0), "Neck": (0.0, 0.0, 0.0),
}


def _pose_row(overrides: dict | None = None) -> np.ndarray:
    """_DEFAULT_POSE に overrides を適用し、HEAD_JOINT_NAMES順の(7,3)配列を返す。"""
    pose = dict(_DEFAULT_POSE)
    if overrides:
        pose.update(overrides)
    return np.array([pose[name] for name in HEAD_JOINT_NAMES], dtype=np.float64)


def _stack_frames(rows: list[np.ndarray]) -> np.ndarray:
    return np.stack(rows, axis=0)


def _head_idx() -> dict[str, int]:
    return resolve_head_indices(HEAD_JOINT_NAMES)


# ========================================
# T1: compute_fps_poses（既知配置）
# ========================================

def test_compute_fps_poses_known_pose():
    x3d = _stack_frames([_pose_row()])
    c2w_b, valid, reasons = compute_fps_poses(x3d, _head_idx())
    assert valid.tolist() == [True]
    assert reasons == []

    expected_R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])
    np.testing.assert_allclose(c2w_b[0, :3, :3], expected_R, atol=1e-12)
    np.testing.assert_allclose(c2w_b[0, :3, 3], [0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(c2w_b[0, 3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)


# ========================================
# T2: head_up 符号反転
# ========================================

def test_compute_fps_poses_head_up_flip():
    # Head/Neckを逆転させ、素朴なcross積のuがhead_upと逆向きになるケースを作る
    row_normal = _pose_row()
    row_flipped = _pose_row({"Head": (0.0, 0.0, -1.0), "Neck": (0.0, 0.0, 0.0)})
    x3d = _stack_frames([row_normal, row_flipped])
    head_idx = _head_idx()
    c2w_b, valid, reasons = compute_fps_poses(x3d, head_idx)
    assert valid.tolist() == [True, True]
    assert reasons == []

    Head = x3d[:, head_idx["Head"], :]
    Neck = x3d[:, head_idx["Neck"], :]
    head_up = Head - Neck
    head_up = head_up / np.linalg.norm(head_up, axis=1, keepdims=True)
    u = c2w_b[:, :3, 1]  # 列1 = u
    dot = np.sum(u * head_up, axis=1)
    assert np.all(dot >= -1e-9)  # 反転補正によりu・head_upは常に非負

    # 反転ケースのuは非反転ケースのuと逆符号になる（head_upが逆向きのため）
    np.testing.assert_allclose(u[1], -u[0], atol=1e-12)


# ========================================
# T3: 縮退4種
# ========================================

def test_compute_fps_poses_degenerate_reasons():
    frame_ok = _pose_row()
    frame_nose_eq_earmid = _pose_row({"Nose": (0.0, 0.0, 0.0)})  # ear_mid=(0,0,0)
    frame_eyes_eq = _pose_row({"LEye": (1.0, 0.0, 0.0), "REye": (1.0, 0.0, 0.0)})
    frame_f_degen = _pose_row({
        "LEar": (0.0, 0.0, 0.0), "REar": (0.0, 0.0, 0.0), "Nose": (5.0, 0.0, 0.0),
    })
    frame_head_eq_neck = _pose_row({"Head": (0.0, 0.0, 0.0), "Neck": (0.0, 0.0, 0.0)})

    x3d = _stack_frames([
        frame_ok, frame_nose_eq_earmid, frame_eyes_eq, frame_f_degen, frame_head_eq_neck,
    ])
    c2w_b, valid, reasons = compute_fps_poses(x3d, _head_idx())

    assert valid.tolist() == [True, False, False, False, False]
    reason_by_idx = dict(reasons)
    assert reason_by_idx[1] == "Nose-ear_mid 縮退"
    assert reason_by_idx[2] == "LEye-REye 縮退"
    assert reason_by_idx[3] == "f(p⊥r0) 縮退"
    assert reason_by_idx[4] == "Head-Neck 縮退"
    assert set(reason_by_idx.keys()) == {1, 2, 3, 4}


# ========================================
# T4: NaN（全点・部分）は警告なしでinvalid
# ========================================

def test_compute_fps_poses_nan_no_warning():
    frame_ok = _pose_row()
    frame_all_nan = np.full((7, 3), np.nan)
    frame_partial_nan = _pose_row()
    frame_partial_nan[HEAD_JOINT_NAMES.index("Nose")] = np.nan

    x3d = _stack_frames([frame_ok, frame_all_nan, frame_partial_nan])
    c2w_b, valid, reasons = compute_fps_poses(x3d, _head_idx())

    assert valid.tolist() == [True, False, False]
    reason_indices = {i for i, _ in reasons}
    assert 1 not in reason_indices
    assert 2 not in reason_indices
    assert np.isnan(c2w_b[1]).all()
    assert np.isnan(c2w_b[2]).all()


# ========================================
# T5: ベクトル化実装 vs スカラー参照実装
# ========================================

def _scalar_reference_pose(row: np.ndarray) -> np.ndarray | None:
    """1フレーム分をスカラーnumpy（mathutils不使用）で計算する素朴な参照実装。

    縮退時は None を返す。compute_fps_poses と同一の式を辿る。
    """
    idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}
    LEye, REye = row[idx["LEye"]], row[idx["REye"]]
    LEar, REar = row[idx["LEar"]], row[idx["REar"]]
    Nose, Head, Neck = row[idx["Nose"]], row[idx["Head"]], row[idx["Neck"]]
    eyes_mid = (LEye + REye) / 2.0
    ear_mid = (LEar + REar) / 2.0

    p_raw = Nose - ear_mid
    if np.linalg.norm(p_raw) < render_fps_video.EPS:
        return None
    r0_raw = LEye - REye
    if np.linalg.norm(r0_raw) < render_fps_video.EPS:
        return None
    p = p_raw / np.linalg.norm(p_raw)
    r0 = r0_raw / np.linalg.norm(r0_raw)
    f_raw = p - r0 * np.dot(p, r0)
    if np.linalg.norm(f_raw) < render_fps_video.EPS:
        return None
    f = f_raw / np.linalg.norm(f_raw)
    u = np.cross(f, r0)
    u = u / np.linalg.norm(u)
    hu_raw = Head - Neck
    if np.linalg.norm(hu_raw) < render_fps_video.EPS:
        return None
    head_up = hu_raw / np.linalg.norm(hu_raw)
    if np.dot(u, head_up) < 0:
        u = -u
    b = -f
    r = np.cross(u, b)
    r = r / np.linalg.norm(r)
    R = np.stack([r, u, b], axis=-1)

    if not np.isfinite(R).all():
        return None
    det = np.linalg.det(R)
    if abs(det - 1.0) > render_fps_video.ORTHO_TOL:
        return None
    max_dev = np.max(np.abs(R @ R.T - np.eye(3)))
    if max_dev > render_fps_video.ORTHO_TOL:
        return None

    c2w = np.eye(4)
    c2w[:3, :3] = R
    c2w[:3, 3] = eyes_mid
    return c2w


def test_compute_fps_poses_matches_scalar_reference():
    rng = np.random.default_rng(42)
    n = 30
    rows = [rng.normal(0.0, 1.0, size=(7, 3)) for _ in range(n)]
    x3d = _stack_frames(rows)
    c2w_b, valid, _reasons = compute_fps_poses(x3d, _head_idx())

    assert valid.all(), "ランダム連続データで縮退が発生しました（再現性のため再検討が必要）"

    for i, row in enumerate(rows):
        ref = _scalar_reference_pose(row)
        assert ref is not None
        np.testing.assert_allclose(c2w_b[i], ref, atol=1e-12)


# ========================================
# T6: c2w_blender_to_viewmats
# ========================================

def test_c2w_blender_to_viewmats_inverse_and_dtype():
    rng = np.random.default_rng(7)
    rows = [rng.normal(0.0, 1.0, size=(7, 3)) for _ in range(10)]
    x3d = _stack_frames(rows)
    c2w_b, valid, _reasons = compute_fps_poses(x3d, _head_idx())
    assert valid.all()

    viewmats = c2w_blender_to_viewmats(c2w_b)
    assert viewmats.dtype == np.float32

    convert = np.diag([1.0, -1.0, -1.0, 1.0])
    c2w_cv = c2w_b.astype(np.float64) @ convert
    viewmats64 = viewmats.astype(np.float64)
    product = np.matmul(viewmats64, c2w_cv)
    identity = np.eye(4)[None, :, :].repeat(len(rows), axis=0)
    np.testing.assert_allclose(product, identity, atol=1e-5)


def test_c2w_blender_to_viewmats_invalid_is_nan():
    c2w_b = np.full((2, 4, 4), np.nan)
    c2w_b[1] = np.eye(4)
    viewmats = c2w_blender_to_viewmats(c2w_b)
    assert np.isnan(viewmats[0]).all()
    assert np.isfinite(viewmats[1]).all()


# ========================================
# T7: plan_chunks / chunk_filename
# ========================================

def test_plan_chunks_f_equals_1():
    frame_ids = np.array([100])
    chunks = plan_chunks(1, 10, frame_ids)
    assert chunks == [{"index": 0, "i0": 0, "i1": 1, "fid0": 100, "fid1": 100}]
    assert chunk_filename(chunks[0]) == "chunk_00000_00000100_00000100.mp4"


def test_plan_chunks_f_equals_chunk_size():
    frame_ids = np.arange(10) + 50
    chunks = plan_chunks(10, 10, frame_ids)
    assert len(chunks) == 1
    assert chunks[0]["i0"] == 0 and chunks[0]["i1"] == 10
    assert chunks[0]["fid0"] == 50 and chunks[0]["fid1"] == 59


def test_plan_chunks_f_equals_chunk_size_plus_1():
    frame_ids = np.arange(11) + 200
    chunks = plan_chunks(11, 10, frame_ids)
    assert len(chunks) == 2
    assert chunks[0]["i0"] == 0 and chunks[0]["i1"] == 10
    assert chunks[0]["fid0"] == 200 and chunks[0]["fid1"] == 209
    assert chunks[1]["i0"] == 10 and chunks[1]["i1"] == 11
    assert chunks[1]["fid0"] == 210 and chunks[1]["fid1"] == 210
    assert chunk_filename(chunks[1]) == "chunk_00001_00000210_00000210.mp4"


# ========================================
# T8: manifest
# ========================================

class _Args:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _make_args(**overrides) -> _Args:
    base = dict(npz_path="in.npz", ply_path="scene.ply", toml="calib.toml",
               camera="cam01", fps=30.0, chunk_size=10000, crf=18, preset="medium")
    base.update(overrides)
    return _Args(**base)


def test_manifest_build_and_no_mismatch():
    args = _make_args()
    manifest = build_manifest(args, 1000, 100, 1099, 1920, 1080)
    assert manifest["schema"] == 1
    assert manifest["num_frames"] == 1000
    assert manifest["fid_first"] == 100 and manifest["fid_last"] == 1099
    assert manifest_mismatch_keys(manifest, manifest) == []


def test_manifest_mismatch_detects_changed_keys():
    args1 = _make_args()
    args2 = _make_args(chunk_size=5000, crf=20)
    m1 = build_manifest(args1, 1000, 100, 1099, 1920, 1080)
    m2 = build_manifest(args2, 1000, 100, 1099, 1920, 1080)
    assert set(manifest_mismatch_keys(m1, m2)) == {"chunk_size", "crf"}


def test_manifest_write_load_roundtrip(tmp_path):
    args = _make_args()
    manifest = build_manifest(args, 500, 10, 509, 640, 480)
    chunk_dir = tmp_path / "chunks"
    chunk_dir.mkdir()
    write_manifest(str(chunk_dir), manifest)
    loaded = load_manifest(str(chunk_dir))
    assert loaded == manifest


def test_load_manifest_missing_raises(tmp_path):
    chunk_dir = tmp_path / "chunks"
    chunk_dir.mkdir()
    with pytest.raises(FileNotFoundError):
        load_manifest(str(chunk_dir))


def test_load_manifest_corrupted_raises_valueerror(tmp_path):
    chunk_dir = tmp_path / "chunks"
    chunk_dir.mkdir()
    (chunk_dir / "manifest.json").write_text("{not valid json")
    with pytest.raises(ValueError, match="manifest.json が壊れています"):
        load_manifest(str(chunk_dir))


# ========================================
# T9: resolve_head_indices
# ========================================

def test_resolve_head_indices_success():
    names = ["Hip"] + list(HEAD_JOINT_NAMES) + ["Spine"]
    idx = resolve_head_indices(names)
    assert set(idx.keys()) == set(HEAD_JOINT_NAMES)
    assert idx["Nose"] == names.index("Nose")
    assert idx["Neck"] == names.index("Neck")


def test_resolve_head_indices_missing_raises():
    names = ["Hip", "LEye", "REye"]
    with pytest.raises(ValueError) as exc_info:
        resolve_head_indices(names)
    msg = str(exc_info.value)
    for missing in ["LEar", "REar", "Nose", "Head", "Neck"]:
        assert missing in msg


# ========================================
# T10: CLI検証（SystemExit(2)）
# ========================================

_BASE_CLI = ["dummy.ply", "dummy.npz", "--toml", "dummy.toml", "--camera", "cam01"]


def test_cli_missing_fps_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI)
    assert exc_info.value.code == 2


def test_cli_fps_not_positive_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "0"])
    assert exc_info.value.code == 2


def test_cli_crf_out_of_range_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--crf", "52"])
    assert exc_info.value.code == 2


def test_cli_gpu_negative_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--gpu", "-1"])
    assert exc_info.value.code == 2


def test_cli_exclusive_still_and_dump_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--still-range", "1", "2", "--still-dir", "out",
                          "--dump-poses", "poses.json"])
    assert exc_info.value.code == 2


def test_cli_still_range_without_dir_exits_2():
    with pytest.raises(SystemExit) as exc_info:
        main(_BASE_CLI + ["--fps", "30", "--still-range", "1", "2"])
    assert exc_info.value.code == 2


# ========================================
# T11: load_intrinsics
# ========================================

_TOML_TEMPLATE = """[cam01]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
size = [{w}, {h}]
"""


def test_load_intrinsics_success(tmp_path):
    pytest.importorskip("tomli")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))
    intr = load_intrinsics(str(toml_path), "cam01")
    assert intr["fx"] == 1000.0 and intr["fy"] == 1000.0
    assert intr["cx"] == 960.0 and intr["cy"] == 540.0
    assert intr["width"] == 1920 and intr["height"] == 1080


def test_load_intrinsics_odd_resolution_raises(tmp_path):
    pytest.importorskip("tomli")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1921, h=1080))
    with pytest.raises(ValueError, match="解像度が偶数ではありません"):
        load_intrinsics(str(toml_path), "cam01")


def test_load_intrinsics_unknown_camera_raises(tmp_path):
    pytest.importorskip("tomli")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))
    with pytest.raises(ValueError, match="cam99"):
        load_intrinsics(str(toml_path), "cam99")


# ========================================
# T12: dump-poses モード（main）
# ========================================

def _write_npz(path, rows: list[np.ndarray], frame_ids: np.ndarray):
    x3d_world = _stack_frames(rows)
    np.savez(
        str(path),
        x3d_world=x3d_world,
        frame_ids=frame_ids.astype(np.int64),
        joint_names=np.array(HEAD_JOINT_NAMES, dtype="<U16"),
    )


def test_dump_poses_mode(tmp_path):
    pytest.importorskip("tomli")
    ply_path = tmp_path / "scene.ply"
    ply_path.write_bytes(b"")  # 存在確認のみ・中身は読まれない

    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))

    npz_path = tmp_path / "in.npz"
    row_ok1 = _pose_row()
    row_nan = _pose_row()
    row_nan[HEAD_JOINT_NAMES.index("Nose")] = np.nan
    row_ok2 = _pose_row()
    _write_npz(npz_path, [row_ok1, row_nan, row_ok2], np.array([1000, 1001, 1002]))

    out_json = tmp_path / "poses.json"
    rc = main([
        str(ply_path), str(npz_path), "--toml", str(toml_path), "--camera", "cam01",
        "--fps", "30", "--dump-poses", str(out_json),
    ])
    assert rc == 0
    assert out_json.exists()

    payload = json.loads(out_json.read_text())
    assert payload["num_frames"] == 3
    frames = payload["frames"]
    assert len(frames) == 3
    assert frames[0]["frame_id"] == 1000 and frames[0]["valid"] is True
    assert len(frames[0]["c2w"]) == 4 and len(frames[0]["c2w"][0]) == 4
    assert frames[1]["frame_id"] == 1001 and frames[1]["valid"] is False
    assert frames[1]["c2w"] is None
    assert frames[2]["frame_id"] == 1002 and frames[2]["valid"] is True


# ========================================
# T13: F=0検証
# ========================================

def test_f_zero_rejected(tmp_path):
    pytest.importorskip("tomli")
    ply_path = tmp_path / "scene.ply"
    ply_path.write_bytes(b"")
    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))

    npz_path = tmp_path / "empty.npz"
    np.savez(
        str(npz_path),
        x3d_world=np.zeros((0, 7, 3)),
        frame_ids=np.zeros((0,), dtype=np.int64),
        joint_names=np.array(HEAD_JOINT_NAMES, dtype="<U16"),
    )

    rc = main([
        str(ply_path), str(npz_path), "--toml", str(toml_path), "--camera", "cam01",
        "--fps", "30", "--dump-poses", str(tmp_path / "poses.json"),
    ])
    assert rc == 1


# ========================================
# T14: start_ffmpeg_chunk（スモーク）
# ========================================

@pytest.mark.skipif(not _FFMPEG_AVAILABLE, reason="ffmpeg（libx264対応）が見つからない環境ではスキップ")
def test_start_ffmpeg_chunk_smoke(tmp_path):
    width, height, fps = 64, 48, 10.0
    out_tmp = str(tmp_path / "chunk_00000_00000000_00000004.mp4.tmp")
    proc = start_ffmpeg_chunk(out_tmp, width, height, fps, 28, "ultrafast")
    black = bytes(height * width * 3)
    for _ in range(5):
        proc.stdin.write(black)
    proc.stdin.close()
    proc.wait()
    assert proc.returncode == 0
    assert os.path.exists(out_tmp)
    assert os.path.getsize(out_tmp) > 0


# ========================================
# T15: format_render_average
# ========================================

def test_format_render_average_zero_frames():
    assert format_render_average(0, 0.0) == "N/A（描画0フレーム）"


def test_format_render_average_nonzero():
    assert format_render_average(10, 5.0) == "0.500秒/フレーム"


# ========================================
# T16: concat_chunks（スモーク）
# ========================================

def _write_chunk_mp4(path: str, width: int, height: int, fps: float, n_frames: int) -> None:
    proc = start_ffmpeg_chunk(path + ".tmp", width, height, fps, 28, "ultrafast")
    black = bytes(height * width * 3)
    for _ in range(n_frames):
        proc.stdin.write(black)
    proc.stdin.close()
    proc.wait()
    assert proc.returncode == 0
    os.replace(path + ".tmp", path)


@pytest.mark.skipif(not (_FFMPEG_AVAILABLE and _FFPROBE_AVAILABLE),
                    reason="ffmpeg（libx264対応）/ffprobe が見つからない環境ではスキップ")
def test_concat_chunks_smoke(tmp_path):
    width, height, fps = 64, 48, 10.0
    chunk_dir = tmp_path / "out.mp4.chunks"
    chunk_dir.mkdir()

    names = []
    for idx in range(2):
        name = f"chunk_{idx:05d}_{idx * 5:08d}_{idx * 5 + 4:08d}.mp4"
        _write_chunk_mp4(str(chunk_dir / name), width, height, fps, 5)
        names.append(name)

    output_path = str(tmp_path / "out.mp4")
    concat_chunks(str(chunk_dir), names, output_path)
    assert os.path.exists(output_path)
    assert os.path.getsize(output_path) > 0


# ========================================
# T17: verify_chunk_mp4（スモーク）
# ========================================

@pytest.mark.skipif(not (_FFMPEG_AVAILABLE and _FFPROBE_AVAILABLE),
                    reason="ffmpeg（libx264対応）/ffprobe が見つからない環境ではスキップ")
def test_verify_chunk_mp4(tmp_path):
    width, height, fps = 64, 48, 10.0
    n = 6
    path = tmp_path / "chunk.mp4"
    _write_chunk_mp4(str(path), width, height, fps, n)

    ok, reason = verify_chunk_mp4(str(path), n)
    assert ok is True
    assert reason == ""

    truncated = tmp_path / "trunc.mp4"
    data = path.read_bytes()
    truncated.write_bytes(data[: len(data) // 2])
    ok2, reason2 = verify_chunk_mp4(str(truncated), n)
    assert ok2 is False
    # ffprobeのバージョン・切断位置により「コンテナ読み取り不能」（非ゼロ終了）
    # または「フレーム数取得不能」（終了コード0だがnb_frames空出力）のいずれかになる
    # （design.md §4.7.1、2026-08-08 実機検証で確認）
    assert reason2 in ("コンテナ読み取り不能", "フレーム数取得不能")

    ok3, reason3 = verify_chunk_mp4(str(path), n + 1)
    assert ok3 is False
    assert "フレーム数不一致" in reason3


# ========================================
# T18: durable_replace
# ========================================

def test_durable_replace_order_and_result(monkeypatch, tmp_path):
    calls = []

    def fake_fsync_file(path):
        calls.append(("fsync_file", path))

    def fake_fsync_dir(path):
        calls.append(("fsync_dir", path))

    real_replace = os.replace

    def fake_replace(src, dst):
        calls.append(("replace", src, dst))
        real_replace(src, dst)

    monkeypatch.setattr(render_fps_video, "fsync_file", fake_fsync_file)
    monkeypatch.setattr(render_fps_video, "fsync_dir", fake_fsync_dir)
    monkeypatch.setattr(render_fps_video.os, "replace", fake_replace)

    src = tmp_path / "a.tmp"
    dst = tmp_path / "a.txt"
    src.write_text("hello")

    durable_replace(str(src), str(dst))

    assert [c[0] for c in calls] == ["fsync_file", "replace", "fsync_dir"]
    assert dst.exists()
    assert not src.exists()


def test_durable_replace_basename_dst_no_empty_dirname(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    dirs_synced = []
    monkeypatch.setattr(render_fps_video, "fsync_dir", lambda p: dirs_synced.append(p))
    monkeypatch.setattr(render_fps_video, "fsync_file", lambda p: None)

    src_name = "b.tmp"
    with open(src_name, "w") as f:
        f.write("x")

    durable_replace(src_name, "out.mp4")

    assert os.path.exists("out.mp4")
    assert len(dirs_synced) == 1
    assert dirs_synced[0] != ""
    assert os.path.samefile(dirs_synced[0], str(tmp_path))


# ========================================
# T19: ffmpeg_has_libx264（実ffmpeg不要、subprocess.run差し替え）
# ========================================

class _FakeCompletedProcess:
    def __init__(self, stdout: str):
        self.stdout = stdout
        self.returncode = 0


def test_ffmpeg_has_libx264_true(monkeypatch):
    def fake_run(cmd, **kwargs):
        assert cmd[0] == "ffmpeg"
        assert "-encoders" in cmd
        return _FakeCompletedProcess(
            " V..... libx264              libx264 H.264 / AVC / MPEG-4 AVC\n"
        )

    monkeypatch.setattr(render_fps_video.subprocess, "run", fake_run)
    assert ffmpeg_has_libx264("ffmpeg") is True


def test_ffmpeg_has_libx264_false(monkeypatch):
    def fake_run(cmd, **kwargs):
        return _FakeCompletedProcess(
            " V..... mpeg4                MPEG-4 part 2\n"
        )

    monkeypatch.setattr(render_fps_video.subprocess, "run", fake_run)
    assert ffmpeg_has_libx264("ffmpeg") is False

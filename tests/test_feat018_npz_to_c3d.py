"""
feat-018: NPZ→C3D変換スクリプト（phase4/npz_to_c3d.py）のテスト

bpy 非依存。c3d（py-c3d 0.6.0）は root venv にあるため import 可能。
T-1 座標ラウンドトリップ / T-2 メタデータ / T-2b 出力パス検証 /
T-3 軸解釈（Blender正立）/ T-4 検証エラー を合成データで検証する。
"""

import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import c3d  # noqa: E402
from npz_to_c3d import (  # noqa: E402
    X_SCREEN,
    Y_SCREEN,
    load_npz,
    main,
    world_to_c3d_raw,
    write_c3d,
)
from render_keypoints import c3d_to_calib  # noqa: E402


def _make_npz(tmp_path, x3d_world, frame_ids, joint_names, name="in.npz") -> str:
    """合成 NPZ を書き出してパスを返す。"""
    p = tmp_path / name
    np.savez(
        p,
        x3d_world=np.asarray(x3d_world, dtype=np.float32),
        frame_ids=np.asarray(frame_ids, dtype=np.int32),
        joint_names=np.asarray(joint_names),
    )
    return str(p)


def _synth_data(F=5, J=4):
    """既知の world座標・連番frame_ids・ラベルを生成。"""
    rng = np.random.default_rng(0)
    x3d_world = rng.normal(size=(F, J, 3)).astype(np.float64)
    frame_ids = np.arange(145599, 145599 + F, dtype=np.int64)  # 65535超を含む
    joint_names = [f"J{i}" for i in range(J)]
    return x3d_world, frame_ids, joint_names


# === T-1 座標ラウンドトリップ ===

def test_t1_coordinate_roundtrip_with_nan(tmp_path):
    """world→raw→C3D書き出し→読み戻し→c3d_to_calib で元 world に一致。NaN点はresidual<0。"""
    x3d_world, frame_ids, joint_names = _synth_data(F=5, J=4)
    # 1点を NaN（無効）にする: frame 2, joint 1
    x3d_world[2, 1, :] = np.nan

    npz = _make_npz(tmp_path, x3d_world, frame_ids, joint_names)
    out = str(tmp_path / "out.c3d")
    assert main([npz, "--output", out]) == 0

    with open(out, "rb") as f:
        reader = c3d.Reader(f)
        frames = list(reader.read_frames())

    assert len(frames) == 5
    for fi, (_frame_no, points, _analog) in enumerate(frames):
        raw = np.asarray(points[:, :3], dtype=np.float64)
        residual = np.asarray(points[:, 3], dtype=np.float64)
        recovered = c3d_to_calib(raw)  # raw mm → world m
        for ji in range(4):
            if fi == 2 and ji == 1:
                assert residual[ji] < 0  # NaN点は無効サンプル
            else:
                assert residual[ji] >= 0
                np.testing.assert_allclose(
                    recovered[ji], x3d_world[fi, ji], atol=1e-4
                )


def test_world_to_c3d_raw_inverse_of_c3d_to_calib():
    """world_to_c3d_raw と c3d_to_calib が逆変換であること。"""
    rng = np.random.default_rng(1)
    w = rng.normal(size=(7, 3))
    np.testing.assert_allclose(c3d_to_calib(world_to_c3d_raw(w)), w, atol=1e-9)


# === T-2 メタデータ ===

def test_t2_metadata(tmp_path):
    """読み戻した C3D の first_frame=1・フレーム数=F・ラベル・rate・UNITS・SCREEN が一致。"""
    x3d_world, frame_ids, joint_names = _synth_data(F=6, J=4)
    npz = _make_npz(tmp_path, x3d_world, frame_ids, joint_names)
    out = str(tmp_path / "out.c3d")
    assert main([npz, "--output", out, "--fps", "30.0"]) == 0

    with open(out, "rb") as f:
        reader = c3d.Reader(f)
        labels = [s.strip() for s in reader.point_labels]
        n_frames = sum(1 for _ in reader.read_frames())
        assert reader.first_frame == 1
        assert n_frames == 6
        assert labels == joint_names
        assert reader.point_rate == pytest.approx(30.0)
        units = reader.get("POINT:UNITS").string_value.strip()
        assert units == "mm"
        x_screen = reader.get("POINT:X_SCREEN").string_value.strip()
        y_screen = reader.get("POINT:Y_SCREEN").string_value.strip()
        assert x_screen == X_SCREEN
        assert y_screen == Y_SCREEN


# === T-2b 出力パス検証 ===

def test_t2b_reject_non_c3d_extension(tmp_path, capsys):
    """出力拡張子が .c3d 以外なら終了コード1・ファイル未生成。"""
    x3d_world, frame_ids, joint_names = _synth_data()
    npz = _make_npz(tmp_path, x3d_world, frame_ids, joint_names)
    bad = str(tmp_path / "out.txt")
    assert main([npz, "--output", bad]) == 1
    assert not os.path.exists(bad)


def test_t2b_reject_output_equals_input(tmp_path):
    """出力パスが入力NPZと同一なら終了コード1・NPZ破壊なし。"""
    x3d_world, frame_ids, joint_names = _synth_data()
    npz = _make_npz(tmp_path, x3d_world, frame_ids, joint_names)
    before = Path(npz).read_bytes()
    assert main([npz, "--output", npz]) == 1
    assert Path(npz).read_bytes() == before  # 入力は無傷


def test_default_output_path(tmp_path):
    """--output 省略時は入力と同じ basename で .c3d を生成。"""
    x3d_world, frame_ids, joint_names = _synth_data()
    npz = _make_npz(tmp_path, x3d_world, frame_ids, joint_names, name="foo.npz")
    assert main([npz]) == 0
    assert (tmp_path / "foo.c3d").exists()


# === T-3 軸解釈（Blender正立。実物アドオンコード + ボーン rest 行列） ===

_ADDON_DIR = Path("/home/sakagawa/git/io_anim_c3d")
_REAL_NPZ = Path(__file__).parent.parent / "phase4" / "data" / "session001_f145749_world300.npz"

# +Z向き pose bone の rest 行列（local→world, Rx(+90)）: (lx,ly,lz)→(lx,-lz,ly)
# io_anim_c3d は location を bone ローカルに書くため、表示ワールド = B @ global_orient @ p_raw。
BONE_REST = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])


def _load_real_parse_dict():
    """io_anim_c3d.c3d_parse_dictionary を bpy 非依存で読み込む（無ければ None）。"""
    import importlib.util
    import types

    pdict = _ADDON_DIR / "c3d_parse_dictionary.py"
    if not pdict.exists():
        return None
    pkg = types.ModuleType("io_anim_c3d")
    pkg.__path__ = [str(_ADDON_DIR)]
    sys.modules["io_anim_c3d"] = pkg
    spec = importlib.util.spec_from_file_location(
        "io_anim_c3d.c3d_parse_dictionary", str(pdict)
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["io_anim_c3d.c3d_parse_dictionary"] = mod
    spec.loader.exec_module(mod)
    return mod.C3DParseDictionary


def _displayed_world(c3d_path: str):
    """実物アドオンで C3D を取り込んだときに画面表示されるワールド座標(先頭フレーム)を返す。

    実物 axis_interpretation で global_orient を取得 → ボーン rest 行列 B を適用。
    アドオンが無ければ None。
    """
    ParseDict = _load_real_parse_dict()
    if ParseDict is None:
        return None, None
    with ParseDict(c3d_path) as parser:
        g, _parsed = parser.axis_interpretation([0, 0, 1], [0, 1, 0])
        # 実物importerと同じく単位換算(mm→m)を global_orient に掛ける（c3d_importer.load）
        g = g * parser.unit_conversion("POINT", sys_unit="m")
        labels = [s.strip() for s in parser.point_labels()]
        for _i, points, _analog in parser.reader.read_frames():
            p_local = (g @ points[:, :3].T).T          # bone ローカル座標（location fcurve値）
            displayed = (BONE_REST @ p_local.T).T      # rest行列でワールドへ
            return displayed, {n: k for k, n in enumerate(labels)}
    return None, None


@pytest.mark.skipif(not _ADDON_DIR.exists(), reason="io_anim_c3dアドオンが無い")
def test_t3_synthetic_identity_with_bone_matrix(tmp_path):
    """合成データ: 実物アドオン + ボーン rest 行列で、表示ワールド = world そのまま（恒等＝正立）。"""
    # 既知 world 座標（Head を上=+Z 側、Ankle を下に配置）
    names = ["Head", "RAnkle", "LAnkle"]
    x3d = np.array([[[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.1, 0.0, -1.0]]])  # (1,3,3)
    npz = _make_npz(tmp_path, x3d, [1], names)
    out = str(tmp_path / "syn.c3d")
    assert main([npz, "--output", out]) == 0

    displayed, idx = _displayed_world(out)
    # 表示ワールド == 元 world（恒等）
    np.testing.assert_allclose(displayed[idx["Head"]], x3d[0, 0], atol=1e-6)
    # Head の表示Z が Ankle より上
    assert displayed[idx["Head"], 2] > displayed[idx["RAnkle"], 2]


@pytest.mark.skipif(
    not _REAL_NPZ.exists() or not _ADDON_DIR.exists(),
    reason="実データNPZまたはアドオンが無い",
)
def test_t3_real_data_upright(tmp_path):
    """実データ: 実物アドオン + ボーン rest 行列で、Head の表示Z が両Ankle より上（正立）。"""
    out = str(tmp_path / "real.c3d")
    assert main([str(_REAL_NPZ), "--output", out]) == 0

    displayed, idx = _displayed_world(out)
    head_z = displayed[idx["Head"], 2]
    ankle_z = (displayed[idx["RAnkle"], 2] + displayed[idx["LAnkle"], 2]) / 2
    assert head_z > ankle_z  # 頭が上 = 正立


# === T-4 検証エラー ===

def test_t4_missing_key(tmp_path):
    """必須キー欠落で終了コード1。"""
    p = tmp_path / "bad.npz"
    np.savez(p, x3d_world=np.zeros((3, 4, 3), dtype=np.float32))  # frame_ids/joint_names欠落
    assert main([str(p)]) == 1


def test_t4_non_consecutive_frame_ids(tmp_path):
    """frame_ids が非連番で ValueError（load_npz）。"""
    x3d_world, _frame_ids, joint_names = _synth_data(F=4, J=4)
    bad_ids = np.array([10, 11, 13, 14], dtype=np.int64)  # 12 が欠ける
    npz = _make_npz(tmp_path, x3d_world, bad_ids, joint_names)
    with pytest.raises(ValueError):
        load_npz(npz)
    assert main([npz, "--output", str(tmp_path / "o.c3d")]) == 1


def test_t4_shape_mismatch(tmp_path):
    """joint_names の長さ不一致で ValueError。"""
    x3d_world, frame_ids, _names = _synth_data(F=4, J=4)
    npz = _make_npz(tmp_path, x3d_world, frame_ids, ["A", "B"])  # J=4 に対し2個
    with pytest.raises(ValueError):
        load_npz(npz)


def test_t4_bad_x3d_world_shape(tmp_path):
    """x3d_world の末尾次元が3でないと ValueError。"""
    p = tmp_path / "bad.npz"
    np.savez(
        p,
        x3d_world=np.zeros((3, 4, 2), dtype=np.float32),
        frame_ids=np.arange(3, dtype=np.int32),
        joint_names=np.asarray(["A", "B", "C", "D"]),
    )
    with pytest.raises(ValueError):
        load_npz(str(p))

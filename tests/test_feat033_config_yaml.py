"""feat-033 render_keypoints.py の --config（YAML設定ファイル読み込み）テスト。

GPU・c3d/torch不要。parse_args_with_config の戻り値（Namespace）検証を基本とし、
E2Eは render スタブ + 合成 NPZ で main を完走させる1件（#1）。

実行:
  uv run pytest tests/test_feat033_config_yaml.py -v
"""

import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

from render_keypoints import (  # noqa: E402
    HALPE26_NAMES,
    KEYPOINT_NAMES,
    _build_parser,
    main,
    parse_args_with_config,
)
from render_fps_video import parse_config_yaml as fps_parse_config_yaml  # noqa: E402
from render_fps_video import CONFIG_CONVERTERS as FPS_CONFIG_CONVERTERS  # noqa: E402


# === 共通ヘルパ ===

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


def _make_npz(tmp_path, name="synth.npz", F=3, J=26, start_frame=1, seed=0):
    """合成 NPZ（Halpe26 全26関節）を tmp_path に書き出し、パスを返す。"""
    rng = np.random.default_rng(seed)
    x3d_world = rng.normal(size=(F, J, 3)).astype(np.float64)
    frame_ids = np.arange(start_frame, start_frame + F, dtype=np.int64)
    joint_names = np.array(HALPE26_NAMES[:J])
    path = tmp_path / name
    np.savez(path, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=joint_names)
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
            alpha_map = np.zeros((H, W), dtype=np.float32)
            return bgr, depth_map, alpha_map
        return bgr

    monkeypatch.setattr("render_keypoints.render_image", _stub_render_image)


# 必須3項目（ply_path, toml_path, camera）を満たす最小の設定YAML断片。
def _required_yaml(toml_path, camera="camA") -> str:
    return f"ply_path: dummy.ply\ntoml_path: {toml_path}\ncamera: {camera}\n"


# ========================================
# #1 E2E（動画モード、render スタブ + 合成NPZ）
# ========================================

def test_e2e_video_mode_config_only(monkeypatch, tmp_path):
    _install_render_stub(monkeypatch)
    toml = _write_synth_toml(tmp_path)
    npz_path = _make_npz(tmp_path, F=3, start_frame=100)
    outdir = tmp_path / "out"

    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        f"ply_path: dummy.ply\n"
        f"toml_path: {toml}\n"
        f"keypoints_path: {npz_path}\n"
        f"camera: camA\n"
        f"output_dir: {outdir}\n"
    )

    rc = main(["--config", str(cfg)])
    assert rc == 0
    pngs = sorted(p.name for p in outdir.glob("*.png"))
    assert len(pngs) == 3


# ========================================
# #2 優先順位（CLI > YAML > デフォルト）
# ========================================

def test_priority_cli_overrides_yaml(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "near_plane: 0.3\n")

    args_yaml_only = parse_args_with_config(["--config", str(cfg)])
    assert args_yaml_only.near_plane == pytest.approx(0.3)

    args_overridden = parse_args_with_config(
        ["--config", str(cfg), "--near-plane", "0.5"]
    )
    assert args_overridden.near_plane == pytest.approx(0.5)

    cfg_no_np = tmp_path / "run_no_np.yaml"
    cfg_no_np.write_text(_required_yaml(toml))
    args_default = parse_args_with_config(["--config", str(cfg_no_np)])
    assert args_default.near_plane == pytest.approx(0.1)


# ========================================
# #3 全16キーの適用確認
# ========================================

def test_all_keys_apply(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        "ply_path: a.ply\n"
        f"toml_path: {toml}\n"
        "keypoints_path: kp.npz\n"
        "camera: camA\n"
        "near_plane: 0.7\n"
        "output_dir: outdir\n"
        "background: 0.1 0.2 0.3\n"
        "no_occlusion: true\n"
        "occlusion_margin: 0.02\n"
        "start_frame: 10\n"
        "end_frame: 20\n"
        "mp4: true\n"
        "mp4_fps: 24.0\n"
        "no_png: true\n"
        "no_keypoints: false\n"
        "distort: false\n"
    )

    args_yaml = parse_args_with_config(["--config", str(cfg)])

    argv_cli = [
        "a.ply", toml, "kp.npz",
        "--camera", "camA",
        "--near-plane", "0.7",
        "--output-dir", "outdir",
        "--background", "0.1", "0.2", "0.3",
        "--no-occlusion",
        "--occlusion-margin", "0.02",
        "--start-frame", "10",
        "--end-frame", "20",
        "--mp4",
        "--mp4-fps", "24.0",
        "--no-png",
    ]
    args_cli = parse_args_with_config(argv_cli)

    for key in [
        "ply_path", "toml_path", "keypoints_path", "camera", "near_plane",
        "output_dir", "no_occlusion", "occlusion_margin", "start_frame",
        "end_frame", "mp4", "mp4_fps", "no_png", "no_keypoints", "distort",
    ]:
        assert getattr(args_yaml, key) == getattr(args_cli, key), key
    assert list(args_yaml.background) == pytest.approx(list(args_cli.background))


# ========================================
# #4 未知キー
# ========================================

def test_unknown_key_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "near_planee: 0.5\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2

    err = capsys.readouterr().err
    assert "near_planee" in err
    assert "有効なキー" in err


# ========================================
# #5 config キーの連鎖禁止
# ========================================

def test_config_chaining_forbidden_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "config: other.yaml\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# #6 bool変換の成功・失敗
# ========================================

def test_bool_conversion_and_invalid(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)

    cfg_true = tmp_path / "true.yaml"
    cfg_true.write_text(_required_yaml(toml) + "mp4: true\n")
    args_true = parse_args_with_config(["--config", str(cfg_true)])
    assert args_true.mp4 is True

    cfg_false = tmp_path / "false.yaml"
    cfg_false.write_text(_required_yaml(toml) + "mp4: false\n")
    args_false = parse_args_with_config(["--config", str(cfg_false)])
    assert args_false.mp4 is False

    cfg_bad = tmp_path / "bad.yaml"
    cfg_bad.write_text(_required_yaml(toml) + "mp4: True\n")
    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg_bad)])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "mp4" in err

    cfg_bad2 = tmp_path / "bad2.yaml"
    cfg_bad2.write_text(_required_yaml(toml) + "mp4: 1\n")
    with pytest.raises(SystemExit) as exc_info2:
        parse_args_with_config(["--config", str(cfg_bad2)])
    assert exc_info2.value.code == 2
    err2 = capsys.readouterr().err
    assert "mp4" in err2


# ========================================
# #7 background 変換
# ========================================

def test_background_conversion(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "background: 0.1 0.2 0.3\n")

    args = parse_args_with_config(["--config", str(cfg)])
    assert list(args.background) == pytest.approx([0.1, 0.2, 0.3])


# ========================================
# #8 background 要素数不正
# ========================================

def test_background_wrong_count_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "background: 0.1 0.2\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# #9 background 非float
# ========================================

def test_background_non_float_exits_2(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "background: a b c\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# #10 数値変換失敗（parametrize）
# ========================================

@pytest.mark.parametrize("bad_line,key", [
    ("near_plane: abc", "near_plane"),
    ("start_frame: 1.5", "start_frame"),
    ("mp4_fps: x", "mp4_fps"),
])
def test_numeric_validation_exits_2(tmp_path, capsys, bad_line, key):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + bad_line + "\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2

    err = capsys.readouterr().err
    assert key in err


# ========================================
# #11 必須項目欠落
# ========================================

def test_required_missing_exits_2(tmp_path, capsys):
    toml = _write_synth_toml(tmp_path)

    cfg_no_camera = tmp_path / "no_camera.yaml"
    cfg_no_camera.write_text(f"ply_path: dummy.ply\ntoml_path: {toml}\n")
    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg_no_camera)])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "--camera" in err

    cfg_empty = tmp_path / "empty.yaml"
    cfg_empty.write_text("# コメントのみ\n")
    with pytest.raises(SystemExit) as exc_info2:
        parse_args_with_config(["--config", str(cfg_empty)])
    assert exc_info2.value.code == 2
    err2 = capsys.readouterr().err
    assert "ply_path" in err2
    assert "toml_path" in err2
    assert "--camera" in err2


# ========================================
# #12 後方互換（--config なし）
# ========================================

def test_backward_compat_no_config():
    toml = "dummy.toml"
    argv = ["dummy.ply", toml, "dummy.npz", "--camera", "camA"]
    args = parse_args_with_config(argv)

    assert args.ply_path == "dummy.ply"
    assert args.toml_path == toml
    assert args.keypoints_path == "dummy.npz"
    assert args.camera == "camA"
    assert args.near_plane == pytest.approx(0.1)
    assert args.output_dir is None
    assert list(args.background) == pytest.approx([0.0, 0.0, 0.0])
    assert args.no_occlusion is False
    assert args.occlusion_margin is None
    assert args.start_frame is None
    assert args.end_frame is None
    assert args.mp4 is False
    assert args.mp4_fps is None
    assert args.no_png is False
    assert args.no_keypoints is False
    assert args.distort is False
    assert args.config is None


# ========================================
# #13 --config 不在
# ========================================

def test_config_file_not_found_exits_2(tmp_path, capsys):
    missing = tmp_path / "nope.yaml"

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(missing)])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert str(missing) in err


# ========================================
# #14 フラグの非対称性（true方向のみCLI上書き可）
# ========================================

def test_flag_asymmetry_yaml_true_stays_true(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_required_yaml(toml) + "mp4: true\n")

    args = parse_args_with_config(["--config", str(cfg)])
    assert args.mp4 is True


# ========================================
# #15 組み合わせ検証が統合後に効く（main経由）
# ========================================

def test_combination_check_after_merge(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        _required_yaml(toml)
        + "keypoints_path: kp.npz\n"
        + "no_keypoints: true\n"
    )

    with pytest.raises(SystemExit) as exc_info:
        main(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# #16 位置引数の部分上書き
# ========================================

def test_positional_partial_override(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        "ply_path: dummy.ply\n"
        f"toml_path: {toml}\n"
        "keypoints_path: dummy.npz\n"
        "camera: camA\n"
    )

    args = parse_args_with_config(["--config", str(cfg), "override.ply"])
    assert args.ply_path == "override.ply"
    assert args.toml_path == toml  # YAML由来のまま
    assert args.keypoints_path == "dummy.npz"  # YAML由来のまま


# ========================================
# #17 空YAML + CLIなし → 必須項目エラー
# ========================================

def test_empty_yaml_required_missing_only(tmp_path, capsys):
    cfg = tmp_path / "empty.yaml"
    cfg.write_text("# コメントのみ\n\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "ply_path" in err
    assert "toml_path" in err
    assert "--camera" in err


# ========================================
# #18 同一キー重複 → 後の値
# ========================================

def test_duplicate_key_last_wins(tmp_path):
    toml = _write_synth_toml(tmp_path)
    cfg = tmp_path / "dup.yaml"
    cfg.write_text(
        _required_yaml(toml) + "near_plane: 0.2\nnear_plane: 0.4\n"
    )

    args = parse_args_with_config(["--config", str(cfg)])
    assert args.near_plane == pytest.approx(0.4)


# ========================================
# #19 render_fps_video.parse_config_yaml の従来挙動確認（converters省略）
# ========================================

def test_fps_video_parse_config_yaml_unchanged(tmp_path, capsys):
    import argparse

    parser = argparse.ArgumentParser(add_help=False)

    # crf は render_fps_video.CONFIG_CONVERTERS に存在するキー → 従来どおり変換できる
    cfg_ok = tmp_path / "ok.yaml"
    cfg_ok.write_text("crf: 20\n")
    result = fps_parse_config_yaml(str(cfg_ok), parser)
    assert result == {"crf": 20}
    assert "crf" in FPS_CONFIG_CONVERTERS

    # toml_path は render_keypoints.CONFIG_CONVERTERS のキーだが
    # render_fps_video.CONFIG_CONVERTERS には存在しない（fps 側は "toml"）。
    # converters省略時に render_fps_video 自身の変換表が使われることの確認（未知キーエラー）。
    cfg_ng = tmp_path / "ng.yaml"
    cfg_ng.write_text("toml_path: x.toml\n")
    with pytest.raises(SystemExit) as exc_info:
        fps_parse_config_yaml(str(cfg_ng), parser)
    assert exc_info.value.code == 2
    err = capsys.readouterr().err
    assert "toml_path" in err

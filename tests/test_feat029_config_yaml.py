"""feat-029 render_fps_video.py の --config（YAML設定ファイル読み込み）テスト。

GPU・ffmpeg不要。parse_args_with_config の戻り値（Namespace）検証を基本とし、
E2Eは dump-poses モード1件のみ（T1）。

実行:
  uv run --project phase4 --with pytest pytest tests/test_feat029_config_yaml.py -v
"""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "phase4"))

from render_fps_video import (  # noqa: E402
    HEAD_JOINT_NAMES,
    load_yaml_flat,
    main,
    parse_args_with_config,
)


# === 共通ヘルパ ===

# 必須5項目（ply_path, npz_path, toml, camera, fps）を満たす最小の設定YAML断片。
# 各テストで追加行を連結して使う。
_REQUIRED_YAML = (
    "ply_path: dummy.ply\n"
    "npz_path: dummy.npz\n"
    "toml: dummy.toml\n"
    "camera: cam01\n"
    "fps: 30\n"
)

_TOML_TEMPLATE = """[cam01]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
size = [{w}, {h}]
"""

# fps_camera_pose.py 由来の「標準姿勢」（縮退なし。feat-027テストと同一配置）
_STANDARD_POSE_ROW = np.array(
    [
        [-1.0, 0.0, 0.0],  # LEye
        [1.0, 0.0, 0.0],   # REye
        [-1.0, 0.0, 0.0],  # LEar
        [1.0, 0.0, 0.0],   # REar
        [0.0, 1.0, 0.0],   # Nose
        [0.0, 0.0, 1.0],   # Head
        [0.0, 0.0, 0.0],   # Neck
    ],
    dtype=np.float64,
)


def _write_min_npz(path) -> None:
    """1フレームのみの有効なNPZ（頭部7点、非縮退）を書き出す。"""
    x3d_world = np.stack([_STANDARD_POSE_ROW], axis=0)  # (1, 7, 3)
    frame_ids = np.array([1000], dtype=np.int64)
    np.savez(
        str(path),
        x3d_world=x3d_world,
        frame_ids=frame_ids,
        joint_names=np.array(HEAD_JOINT_NAMES, dtype="<U16"),
    )


# ========================================
# T1: E2E（dump-poses モード、GPU不要）
# ========================================

def test_e2e_dump_poses_config_only(tmp_path):
    pytest.importorskip("tomli")
    ply_path = tmp_path / "scene.ply"
    ply_path.write_bytes(b"")  # 存在確認のみ・中身は読まれない

    toml_path = tmp_path / "calib.toml"
    toml_path.write_text(_TOML_TEMPLATE.format(w=1920, h=1080))

    npz_path = tmp_path / "in.npz"
    _write_min_npz(npz_path)

    out_json = tmp_path / "poses.json"
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        f"ply_path: {ply_path}\n"
        f"npz_path: {npz_path}\n"
        f"toml: {toml_path}\n"
        "camera: cam01\n"
        "fps: 30\n"
        f"dump_poses: {out_json}\n"
    )

    rc = main(["--config", str(cfg)])
    assert rc == 0
    assert out_json.exists()

    payload = json.loads(out_json.read_text())
    assert payload["num_frames"] == 1
    assert payload["frames"][0]["frame_id"] == 1000
    assert payload["frames"][0]["valid"] is True


# ========================================
# T2: 優先順位（CLI > YAML > デフォルト）
# ========================================

def test_priority_cli_overrides_yaml(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "crf: 20\n")

    args_yaml_only = parse_args_with_config(["--config", str(cfg)])
    assert args_yaml_only.crf == 20

    args_overridden = parse_args_with_config(["--config", str(cfg), "--crf", "23"])
    assert args_overridden.crf == 23

    cfg_no_crf = tmp_path / "run_no_crf.yaml"
    cfg_no_crf.write_text(_REQUIRED_YAML)
    args_default = parse_args_with_config(["--config", str(cfg_no_crf)])
    assert args_default.crf == 18


# ========================================
# T3: 未知キー
# ========================================

def test_unknown_key_exits_2(tmp_path, capsys):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "crff: 18\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2

    err = capsys.readouterr().err
    assert "crff" in err
    assert "有効なキー" in err


# ========================================
# T4: bool変換
# ========================================

def test_bool_conversion_true_and_false(tmp_path):
    cfg_true = tmp_path / "true.yaml"
    cfg_true.write_text(_REQUIRED_YAML + "overwrite: true\n")
    args_true = parse_args_with_config(["--config", str(cfg_true)])
    assert args_true.overwrite is True

    cfg_false = tmp_path / "false.yaml"
    cfg_false.write_text(_REQUIRED_YAML + "overwrite: false\n")
    args_false = parse_args_with_config(["--config", str(cfg_false)])
    assert args_false.overwrite is False


def test_bool_invalid_value_exits_2(tmp_path):
    cfg = tmp_path / "bad.yaml"
    cfg.write_text(_REQUIRED_YAML + "overwrite: True\n")  # 大文字は不可

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# T5: still_range
# ========================================

def test_still_range_conversion(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "still_range: 100 200\n")

    args = parse_args_with_config(["--config", str(cfg)])
    assert args.still_range == [100, 200]


def test_still_range_wrong_count_exits_2(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "still_range: 100\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


def test_still_range_non_integer_exits_2(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "still_range: abc def\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# T6: 必須欠落（統合後）
# ========================================

def test_required_missing_exits_2(tmp_path, capsys):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        "ply_path: dummy.ply\n"
        "npz_path: dummy.npz\n"
        "toml: dummy.toml\n"
        "camera: cam01\n"
    )  # fps を CLI にも YAML にも与えない

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2

    err = capsys.readouterr().err
    assert "fps" in err


# ========================================
# T7: 後方互換（--config なし）
# ========================================

def test_backward_compat_no_config():
    argv = [
        "dummy.ply", "dummy.npz", "--toml", "dummy.toml", "--camera", "cam01",
        "--fps", "30",
    ]
    args = parse_args_with_config(argv)

    assert args.ply_path == "dummy.ply"
    assert args.npz_path == "dummy.npz"
    assert args.toml == "dummy.toml"
    assert args.camera == "cam01"
    assert args.fps == 30.0
    assert args.output is None
    assert args.gpu == 0
    assert args.chunk_size == 10000
    assert args.crf == 18
    assert args.preset == "medium"
    assert args.overwrite is False
    assert args.keep_chunks is False
    assert args.still_range is None
    assert args.still_dir is None
    assert args.dump_poses is None
    assert args.config is None


# ========================================
# T8: --config 不在
# ========================================

def test_config_file_not_found_exits_2(tmp_path):
    missing = tmp_path / "nope.yaml"

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(missing)])
    assert exc_info.value.code == 2


# ========================================
# T9: config キーの連鎖禁止
# ========================================

def test_config_chaining_forbidden_exits_2(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "config: other.yaml\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# T10: 型検証（範囲違反・型変換失敗）
# ========================================

@pytest.mark.parametrize("bad_line,key", [
    ("crf: 99", "crf"),
    ("fps: 0", "fps"),
    ("gpu: -1", "gpu"),
    ("crf: abc", "crf"),
    ("fps: abc", "fps"),
    ("gpu: abc", "gpu"),
    ("chunk_size: abc", "chunk_size"),
])
def test_numeric_validation_exits_2(tmp_path, capsys, bad_line, key):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + bad_line + "\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2

    err = capsys.readouterr().err
    assert key in err


# ========================================
# T11: フラグの非対称性（true方向のみCLI上書き可）
# ========================================

def test_flag_asymmetry_yaml_true_stays_true_without_cli_flag(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML + "overwrite: true\n")

    # CLIに --overwrite を付けなくてもYAMLのtrueがそのまま残る
    # （falseに戻すCLI手段は存在しない仕様の確認）
    args = parse_args_with_config(["--config", str(cfg)])
    assert args.overwrite is True


# ========================================
# T12: 排他制約が統合後に効く（main経由）
# ========================================

def test_exclusive_still_and_dump_via_config_exits_2(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(
        _REQUIRED_YAML
        + "dump_poses: poses.json\n"
        + "still_range: 1 2\n"
    )

    with pytest.raises(SystemExit) as exc_info:
        main(["--config", str(cfg)])
    assert exc_info.value.code == 2


# ========================================
# T13: 位置引数の上書き（1個のみCLI指定）
# ========================================

def test_positional_argument_partial_override(tmp_path):
    cfg = tmp_path / "run.yaml"
    cfg.write_text(_REQUIRED_YAML)

    args = parse_args_with_config(["--config", str(cfg), "override.ply"])
    assert args.ply_path == "override.ply"
    assert args.npz_path == "dummy.npz"  # YAML由来のまま


# ========================================
# T14: 空YAML・重複キー
# ========================================

def test_empty_yaml_required_missing_only(tmp_path):
    cfg = tmp_path / "empty.yaml"
    cfg.write_text("# コメントのみ\n\n")

    with pytest.raises(SystemExit) as exc_info:
        parse_args_with_config(["--config", str(cfg)])
    assert exc_info.value.code == 2


def test_duplicate_key_last_wins(tmp_path):
    cfg = tmp_path / "dup.yaml"
    cfg.write_text("crf: 10\ncrf: 20\n")

    raw = load_yaml_flat(str(cfg))
    assert raw["crf"] == "20"

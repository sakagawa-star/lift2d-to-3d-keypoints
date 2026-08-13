# feat-033 機能設計書: render_keypoints.py の YAML 設定ファイル読み込み

## 1. 対応要求マッピング

要求仕様書: `docs/issues/feat-033-render-keypoints-yaml-config/requirements.md`

| 要求ID | 設計セクション |
|---|---|
| FR-001（--config 読み込み） | 4.1（`parse_args_with_config`）、4.4（エラーハンドリング） |
| FR-002（設定キーと型変換） | 4.2（`CONFIG_CONVERTERS` / `_yaml_floats3`） |
| FR-003（優先順位） | 4.1（統合パース）、4.3（パーサー変更） |
| FR-004（必須項目の統合後検証） | 4.1（必須チェック）、4.3（パーサー変更） |
| FR-005（設定YAMLのエラー検出） | 4.2、4.4 |

## 2. システム構成

### モジュール構成

```
phase4/
├── render_keypoints.py    # 変更対象。--config 追加、CONFIG_CONVERTERS / _yaml_floats3 /
│                          #   parse_args_with_config を新設、_build_parser の必須引数を統合後検証化
├── render_fps_video.py    # 最小変更。parse_config_yaml に省略可能引数 converters を追加
│                          #   （省略時は従来どおり自身の CONFIG_CONVERTERS。挙動変更なし）
└── npz_to_c3d.py          # 変更なし
tests/
└── test_feat033_config_yaml.py   # 新規テスト
```

### 依存関係

- `render_keypoints.py` → `render_fps_video.py` の `load_yaml_flat`, `_yaml_bool`, `parse_config_yaml` を**トップレベル import** で再利用する。`render_fps_video.py` のトップレベル import は argparse/json/multiprocessing/os/queue/shutil/signal/subprocess/sys/time/typing/numpy/npz_to_c3d のみ（torch 非依存）であることを確認済みのため、root venv のテストでも import できる
- 逆方向（render_fps_video → render_keypoints）の import は存在しない（循環依存なし）

## 3. 技術スタック

- Python 3.10（phase4 venv、uv 管理）。テストは root venv の `uv run pytest` で実行（既存方式: `sys.path.insert` で `phase4/` を追加）
- 新規ライブラリ追加なし（PyYAML 不採用は feat-029 で確定済み）→ `docs/TECH_STACK.md` の更新は不要

## 4. 詳細設計

### 4.1 統合パース `parse_args_with_config`（FR-001/003/004）

`render_keypoints.py` に新設する。`main()` 冒頭の `args = parser.parse_args(argv)` をこの関数の呼び出しに置き換える。

#### インターフェース

```python
def parse_args_with_config(
    argv=None, parser: argparse.ArgumentParser | None = None
) -> argparse.Namespace:
```

- `argv`: コマンドライン引数（None なら sys.argv）。既存テストとの互換のため `main(argv)` の argv をそのまま渡す
- `parser`: 使用するパーサー（None なら内部で `_build_parser()` を呼ぶ）。`main()` は自作の parser を渡し、以降の `parser.error` 呼び出しで同一の parser オブジェクトを使い続ける（feat-029 と同一の設計）

#### 処理ロジック

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def parse_args_with_config(argv=None, parser=None):
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default=None)
    pre_args, _ = pre_parser.parse_known_args(argv)

    if parser is None:
        parser = _build_parser()

    if pre_args.config is not None:
        defaults = parse_config_yaml(pre_args.config, parser,
                                     converters=CONFIG_CONVERTERS)  # 本スクリプト用の変換表を渡す
        parser.set_defaults(**defaults)

    args = parser.parse_args(argv)

    required_dests = [("ply_path", "ply_path"), ("toml_path", "toml_path"),
                      ("camera", "--camera")]
    missing = [label for dest, label in required_dests if getattr(args, dest) is None]
    if missing:
        parser.error(
            f"必須項目が未指定です（CLIまたは--configで指定）: {', '.join(missing)}"
        )
    return args
```

- 優先順位の実現方法は feat-029 と同一: 設定YAMLの値を `parser.set_defaults` でデフォルトに昇格させ、CLI 明示指定があれば argparse が上書きする
- 必須項目は `ply_path` / `toml_path` / `camera` の3つ（FR-004）。`keypoints_path` は含めない（`--no-keypoints` との既存の組み合わせ検証が統合後の値に対してそのまま働く）
- `main()` 内の既存の組み合わせ検証（`--no-keypoints` 時の禁止オプション、`keypoints_path` 要否、`--distort` 制約、`--no-png`+`--mp4` 必須）は**変更しない**。統合後の `args` に対して従来どおり実行される（設定YAML由来の値も「与えられた」扱い。FR-004）

### 4.2 設定キーと変換表（FR-002/005）

`render_keypoints.py` に新設する。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def _yaml_floats3(value: str) -> list[float]:
    """設定YAMLの background 値（スペース区切り float 3個）を変換する。"""
    parts = value.split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"スペース区切りの数値3個である必要があります: {value}")
    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise argparse.ArgumentTypeError(f"数値に変換できません: {value}") from e


CONFIG_CONVERTERS = {
    "ply_path": str, "toml_path": str, "keypoints_path": str,
    "camera": str, "output_dir": str,
    "near_plane": float, "occlusion_margin": float, "mp4_fps": float,
    "start_frame": int, "end_frame": int,
    "background": _yaml_floats3,
    "no_occlusion": _yaml_bool, "mp4": _yaml_bool, "no_png": _yaml_bool,
    "no_keypoints": _yaml_bool, "distort": _yaml_bool,
}
```

- キー名は argparse の dest 名と完全一致させる（`parser.set_defaults(**defaults)` にそのまま渡すため）。全16キー、FR-002 の表と1対1対応
- 数値系は CLI と同じ素の `float` / `int` 変換のみ（CLI 側に値域検証が無いため YAML 側にも追加しない。FR-002）。変換失敗（`ValueError`）は `parse_config_yaml` の既存 except 節（`ArgumentTypeError, ValueError, TypeError`）が捕捉し、キー名つき `parser.error` になる
- `_yaml_bool`（小文字 true/false のみ許可）は `render_fps_video.py` から import して再利用する

#### `render_fps_video.parse_config_yaml` の最小変更

呼び出し側から変換表を注入できるよう、省略可能引数を追加する（**これ以外は一切変更しない**）:

```python
# 変更前
def parse_config_yaml(yaml_path: str, parser: argparse.ArgumentParser) -> dict:
    ...
    unknown = [k for k in raw if k not in CONFIG_CONVERTERS]
    ...
    converter = CONFIG_CONVERTERS[k]

# 変更後（意図伝達用。converters 省略時の挙動は従来と完全に同一）
def parse_config_yaml(yaml_path, parser, converters=None) -> dict:
    if converters is None:
        converters = CONFIG_CONVERTERS   # render_fps_video 自身の変換表
    ...
    unknown = [k for k in raw if k not in converters]
    ...
    converter = converters[k]
```

- `render_fps_video.py` 内の既存呼び出し（`parse_args_with_config` 内）は無変更（converters を渡さない → 従来挙動）
- docstring に converters 引数の説明を追記する
- エラーメッセージ（「--config が読み込めません」「config キーは書けません」「未知のキーがあります…有効なキー一覧」「設定YAML キー '{k}': …」）は共通のまま流用する

### 4.3 パーサー変更（FR-003/004）

`_build_parser()` の変更点（これ以外の引数定義は変更しない）:

| 引数 | 変更前 | 変更後 |
|---|---|---|
| `ply_path` | 必須位置引数 | `nargs="?", default=None`、help 末尾に「（--config でも指定可）」を追記 |
| `toml_path` | 必須位置引数 | `nargs="?", default=None`、help 末尾に「（--config でも指定可）」を追記 |
| `keypoints_path` | `nargs="?", default=None`（feat-032） | 変更なし（help 末尾に「（--config でも指定可）」を追記のみ） |
| `--camera` | `required=True` | `default=None`（統合後検証に移行）。help 末尾に「（--config でも指定可）」を追記 |
| `--config` | （なし） | 新設: `default=None`、help =「設定YAML（フラット key: value）。優先順位は CLI > 設定YAML > デフォルト。フラグ系（--no-occlusion/--mp4/--no-png/--no-keypoints/--distort）は CLI から true 方向のみ上書き可（設定YAMLの true を false に戻すには設定YAMLを編集する）」 |

- モジュール docstring の実行例の下に、`--config run_keypoints.yaml` の1行例を追記する
- **挙動変更（意図的、FR-004 に明記済み）**: `ply_path` / `toml_path` / `--camera` の欠落時エラーが、argparse 標準の required エラーから `必須項目が未指定です（CLIまたは--configで指定）: ...` に変わる（終了コード 2 は維持）。既存テストはこれらを常に与えており文言に依存しないことを確認済み（2026-08-13 調査）

### 4.4 エラーハンドリング

すべて `parser.error`（usage + メッセージを stderr に出力し終了コード 2）で処理する。リトライなし。feat-029 実装の共通部をそのまま流用する。

| エラー | 検出方法 | 出力（stderr） |
|---|---|---|
| 設定YAMLが存在しない・読めない | `load_yaml_flat` の `OSError` を `parse_config_yaml` が捕捉 | `--config が読み込めません: <path>: <原因>` |
| `config` キー（連鎖） | `parse_config_yaml` の既存チェック | `設定YAMLに config キーは書けません（連鎖読み込み不可）` |
| 未知のキー | `parse_config_yaml` の既存チェック（converters に基づく） | `設定YAMLに未知のキーがあります: <キー>。有効なキー: <一覧>` |
| bool 値不正 / background 要素数・変換不正 / 数値変換失敗 | 各コンバータの `ArgumentTypeError` / `ValueError` を `parse_config_yaml` が捕捉 | `設定YAML キー '<キー>': <原因>` |
| 必須項目欠落（統合後） | `parse_args_with_config` の明示チェック | `必須項目が未指定です（CLIまたは--configで指定): <項目一覧>` |
| 組み合わせ違反（--no-keypoints 系、--no-png 単独ほか） | `main()` の既存チェック（無変更） | 既存メッセージのまま |

### 4.5 境界条件

| 条件 | 振る舞い |
|---|---|
| 空の設定YAML（有効行なし） | `parse_config_yaml` が空 dict を返し、デフォルト昇格なし。必須項目が CLI にも無ければ統合後検証でエラー |
| 同一キーの重複行 | 後の行が勝つ（`load_yaml_flat` の dict 上書き仕様。feat-029 と同一） |
| `:` を含まない非空行 | スキップ（`load_yaml_flat` の既存仕様） |
| 値が空文字（`camera:`） | str キーは空文字として受理され、後段の既存検証（カメラ名不一致等）で検出される。数値・bool キーは変換失敗でエラー |
| `--config` だけ指定し YAML に全必須項目がある | 位置引数なしで実行成功（FR-001 受け入れ基準） |

## 5. ログ・デバッグ設計

- 既存方式を踏襲する（stdout=進捗 print、stderr=エラー・警告）。logging モジュールは導入しない
- 設定YAML読み込み成功時のログ出力は追加しない（feat-029 と同一。エラー時のみ表示）

## 6. テスト設計

新規 `tests/test_feat033_config_yaml.py`。`tests/test_feat029_config_yaml.py` の構成・`tests/test_feat032_npz_input.py` の render スタブパターンを踏襲する（root venv 実行、`sys.path.insert` で `phase4/` 追加）。

| # | テスト | 対応FR | 概要 |
|---|---|---|---|
| 1 | `test_e2e_video_mode_config_only` | FR-001 | 全必須項目 + NPZ を書いた設定YAMLで `main(["--config", path])` を render スタブ + 合成 NPZ で完走（戻り値 0） |
| 2 | `test_priority_cli_overrides_yaml` | FR-003 | YAML `near_plane: 0.3` + CLI `--near-plane 0.5` → 0.5。YAML のみ → 0.3。両方なし → 0.1 |
| 3 | `test_all_keys_apply` | FR-002 | 全16キーを書いた YAML で `parse_args_with_config` の結果が対応 CLI 指定と同値 |
| 4 | `test_unknown_key_exits_2` | FR-005 | 未知キー → SystemExit(2)、stderr に有効キー一覧 |
| 5 | `test_config_chaining_forbidden_exits_2` | FR-005 | `config:` キー → SystemExit(2) |
| 6 | `test_bool_conversion_and_invalid` | FR-002/005 | `true`/`false` 変換成功、`True`/`1` は SystemExit(2)（キー名を含む） |
| 7 | `test_background_conversion` | FR-002 | `background: 0.1 0.2 0.3` → [0.1, 0.2, 0.3] |
| 8 | `test_background_wrong_count_exits_2` | FR-005 | 要素2個 → SystemExit(2) |
| 9 | `test_background_non_float_exits_2` | FR-005 | `background: a b c` → SystemExit(2) |
| 10 | `test_numeric_validation_exits_2`（parametrize） | FR-005 | `near_plane: abc` / `start_frame: 1.5` / `mp4_fps: x` → SystemExit(2)（キー名を含む） |
| 11 | `test_required_missing_exits_2` | FR-004 | camera 欠落 → SystemExit(2)、stderr に `--camera`。全欠落 → 3項目列挙 |
| 12 | `test_backward_compat_no_config` | FR-001 | `--config` なしの従来引数でパース結果が従来どおり（near_plane=0.1 等のデフォルト維持） |
| 13 | `test_config_file_not_found_exits_2` | FR-001 | 存在しないパス → SystemExit(2)、stderr にパス |
| 14 | `test_flag_asymmetry_yaml_true_stays_true` | FR-003 | YAML `mp4: true` は CLI フラグなしでも true のまま |
| 15 | `test_combination_check_after_merge` | FR-004 | YAML `no_keypoints: true` + `keypoints_path` あり → SystemExit(2)（既存の組み合わせエラー） |
| 16 | `test_positional_partial_override` | FR-003 | YAML に3パス + CLI に位置引数1個 → `ply_path` のみ上書き |
| 17 | `test_empty_yaml_required_missing_only` | FR-004 | 空 YAML + CLI なし → 必須項目エラー |
| 18 | `test_duplicate_key_last_wins` | FR-003 | 同一キー2行 → 後の値 |
| 19 | `test_fps_video_parse_config_yaml_unchanged` | 制約 | `render_fps_video.parse_config_yaml` を converters 省略で呼び、従来の変換表が使われる（feat-029 回帰の直接確認。既存 test_feat029 全件パスと併せて検証） |

- 既存テスト（test_feat015〜032、test_feat029 を含む）は**全件無変更でパス**すること
- テスト結果は `tests/results/feat-033_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する

### 実データでの動作確認（実装ステップ内で実施）

ローカル機（gtune2、RTX 5060 Ti）で実施する。コマンド例の `TORCH_CUDA_ARCH_LIST="9.0+PTX"` はローカル機向けの値（要求仕様書1章参照。A100 等の他環境では環境に応じた値を使う）。

feat-032 の動作確認と同一データで、設定YAML経由の実行が成功することを確認する。まず以下の内容で `phase4/data/run_keypoints.yaml` を作成する:

```yaml
ply_path: /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply
toml_path: data/Blender/handai-hosp1_20251024.toml
keypoints_path: data/session001_f145749_world300_filtered.npz
camera: int_cam01_img
near_plane: 0.5
output_dir: data/feat033_check
mp4: true
no_png: true
```

次に実行する（パスは phase4/ ディレクトリ基準。ルートから実行する場合は YAML 内の相対パスを `phase4/...` に読み替えて作成する）:

```bash
# phase4/ ディレクトリで実行
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py --config data/run_keypoints.yaml
```

期待値: feat-032 の動作確認と同一の出力（NPZ 300フレーム、22/28マーカー、fps=30 警告、`data/feat033_check/output.mp4` 生成）。さらに `--mp4-fps 25` を CLI に足して fps=25 に上書きされることを確認する。

## 7. 設計判断（ADR 簡易版）

### D-1: 汎用部品は `render_fps_video.py` から import 再利用し、`parse_config_yaml` に省略可能引数 `converters` を追加（採用）

- **採用**: `load_yaml_flat` / `_yaml_bool` / `parse_config_yaml` を import。`parse_config_yaml` は変換表を引数注入できるよう省略可能引数を追加（省略時は従来挙動、既存呼び出し・既存テスト無変更）
- **却下案1**: render_keypoints.py 内に全部を複製する（feat-029 の phase0/common.py 非共有と同じ扱い）
- **却下案2**: 共通モジュール（例: phase4/config_yaml.py）に切り出す
- **理由**: 却下案1は同一パッケージ内（phase4、同一 venv）での重複であり、feat-029 当時の「独立 venv 間で import できない」事情が当てはまらない。既知の重複負債を増やさない（feat-032 D-2 と同方針）。却下案2は feat-029 実装の移動を伴い変更範囲が広がる（両スクリプトのテスト影響）。省略可能引数の追加は挙動変更ゼロで最小

### D-2: 位置引数の省略可能化と統合後検証（採用）

- **採用**: `ply_path` / `toml_path` を `nargs="?"` 化、`--camera` の required を外し、統合後に3項目まとめて検証（feat-029 と同一方式）
- **却下案**: required のまま「YAML があるときだけ後付けで required を外す」動的パーサー構築
- **理由**: feat-029 で確立済みの方式で、エラーメッセージも統一される。動的構築はパーサーの姿が実行パスで変わり、テスト・ヘルプ表示が複雑になる。欠落時エラー文言の変化は要求（FR-004）に明記して許容

### D-3: `background` はスペース区切り float 3個（採用）

- **採用**: `background: 0 0 0` 形式（feat-029 の `still_range: 145599 145650` と同形式）
- **却下案**: YAML リスト構文 `[0, 0, 0]`
- **理由**: 簡易フラットパーサーはリスト構文を解さない（feat-029 で Won't 確定）。既存の前例と表記を揃える

### D-4: フラグの CLI 上書きは true 方向のみ（採用）

- **採用**: feat-029 と同一の非対称性を許容し、`--config` の help に明記する
- **却下案**: `--no-mp4` のような打ち消しオプションの追加
- **理由**: feat-029 で確定済みの運用。打ち消しオプションの追加は CLI 肥大とスコープ拡大になる

### D-5: 組み合わせ検証は統合後の値に適用（採用）

- **採用**: `main()` の既存検証を無変更のまま、統合後の args に適用する（設定YAML由来の値も「与えられた」扱い）
- **却下案**: YAML 由来か CLI 由来かで検証を分ける
- **理由**: feat-029 FR-004 と同一方針。由来で挙動を変えると「同じ最終値なのに結果が違う」状態になり予測しにくい

## 8. 完了時の更新対象（ステップ8で本体が実施）

- `docs/BACKLOG.md`: feat-033 を Closed に
- `docs/CHANGELOG.md`: 完了内容を記録
- `CLAUDE.md`: ディレクトリ構成の `render_keypoints.py` 説明に `--config` 対応を追記（ファイル追加・削除はなし）
- `README.md`: `render_keypoints.py` の節に `--config` の説明・設定YAML例・オプション表の行を追記

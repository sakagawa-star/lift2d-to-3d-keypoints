# feat-043 機能設計書: スマートフォン静止画の前処理と推定で 1 つの設定 YAML を共用する

作成日: 2026-11-25
基準: `docs/DESIGN_STANDARD.md`
本書のコード例は意図を伝えるためのもので、そのままコピーして使うものではない。用語は requirements.md §2 に従う。

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 | §4.2（前処理のパーサー）、§4.3（推定のパーサー） |
| FR-002 | §4.2 |
| FR-003 | §4.1（`ignored_keys`）、§4.2・§4.3 の `IGNORED_KEYS` |
| FR-004 | §4.2（カメラ名の検査） |
| FR-005 | §4.3 |
| 非機能（出力の同一性） | §6.2 |

## 2. システム構成

### 2.1 変更対象

| ファイル | 変更内容 |
|---|---|
| `phase4/refine_extrinsics.py` | `parse_args_with_config` に省略可能な引数 `ignored_keys` を追加（既定は空。既存の呼び出しの動作は変えない）。それ以外は変えない |
| `phase4/prepare_phone_photo.py` | パーサーの名前の変更・`--config` の追加、`CONFIG_CONVERTERS`・`REQUIRED_OPTIONS`・`IGNORED_KEYS` の追加、カメラ名の検査、`main` の置き換え |
| `phase4/estimate_intrinsics_loop.py` | `--toml` → `--init-toml`、`IGNORED_KEYS` の追加、`args.toml` の参照とレポートのヘッダの表示名の変更 |
| `tests/test_feat043_shared_yaml.py` | 新規（§6.1） |
| `tests/test_feat041_intrinsics_loop.py` | 名前の置き換えだけ（§6.3） |
| `tests/test_feat042_config_yaml.py` | 名前の置き換えだけ（§6.3） |
| `tests/results/feat-043_test_result.txt` | 新規 |
| `docs/issues/feat-043-shared-job-yaml-phone-photo/verification.md` | 新規（§6.2 の実データでの確認の記録。実装担当がステップ6で作る。codex-04 指摘1） |

変更しないファイル: `phase4/render_fps_video.py`（`load_yaml_flat`・`parse_config_yaml` を使うだけ）、`phase4/adjust_extrinsics_multiview.py`（`parse_args_with_config` を `ignored_keys` なしで呼び続ける）、`phase4/evaluate_overlay.py`、`matcher_lab/*`。

### 2.2 依存関係

```
prepare_phone_photo.py ──import──> refine_extrinsics（既存: InputError, _format_toml_value, load_points_2d_csv。
                                                     追加: parse_args_with_config, _yaml_str）
                       ──import──> render_fps_video（追加: _yaml_bool）
estimate_intrinsics_loop.py ──import──> refine_extrinsics（既存の import のまま）
```

`prepare_phone_photo.py` は既に `refine_extrinsics` を import しており（`refine_extrinsics` は `render_keypoints` → `render_fps_video` を読み込む）、ルート環境の pytest からの import 可否は変わらない。

## 3. 技術スタック

Python 3.10、phase4 の uv 環境。新規ライブラリなし（`docs/TECH_STACK.md` の更新は不要）。

## 4. 詳細設計

### 4.1 `parse_args_with_config` の拡張（FR-003）

`phase4/refine_extrinsics.py` の既存関数に、省略可能な引数を 1 つ追加する。

```python
def parse_args_with_config(argv, parser, converters, required_options,
                           ignored_keys: frozenset[str] = frozenset()) -> argparse.Namespace:
```

- `ignored_keys`: 設定YAMLに書かれていても黙って無視するキー（共用 YAML の相手のキー）。`converters` のキーと重ならないこと（重なりは呼び出し側のプログラムの誤りなので `assert` で検出する）
- 処理の変更点は手順 3（設定YAMLの読み込み）だけ:

```python
# 意図伝達用の擬似コード
all_converters = {**converters, **{k: _yaml_ignore for k in ignored_keys}}
defaults = parse_config_yaml(first.config, parser, converters=all_converters)  # 既存の except はそのまま
defaults = {k: v for k, v in defaults.items() if k not in ignored_keys}       # 相手のキーを捨てる
parser.set_defaults(**defaults)
```

- `_yaml_ignore(value: str) -> str`: 値をそのまま返す（検査しない）。`refine_extrinsics.py` に新設する。相手のキーの値は型変換・NUL 検査を行わずに捨てる（requirements.md FR-003）
- 未知のキーのエラーメッセージ（`parse_config_yaml` の既存処理）の「有効なキー」一覧には、相手のキーも含まれる
- `ignored_keys` を渡さない呼び出し（`refine_extrinsics.py`・`adjust_extrinsics_multiview.py`）は、`all_converters == converters` となり、従来と同じ動作になる
- docstring に `ignored_keys` の説明を追記する

### 4.2 前処理 `prepare_phone_photo.py`（FR-001・002・004）

> **注記（feat-044、2026-11-26）**: 本節の `--expect-model`・`--expect-focal-mm` とその変換表のキーは feat-044 で削除され、`--photo-info`（キー `photo_info`、必須）が追加された。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

#### パーサー（`build_arg_parser`）

| CLI | dest | 変更 | help |
|---|---|---|---|
| `--image` | `image` | `required=True` を外し `default=None` | 原本JPEG（--config でも指定可） |
| `--camera` | `camera` | 同上 | カメラ名（--config でも指定可） |
| `--images-dir` | `images_dir` | 同上 | 前処理後画像の出力ディレクトリ（画像は <images-dir>/<camera>.png）（--config でも指定可） |
| `--init-toml` | `init_toml` | 新名（旧 `--out-toml`） | 初期TOMLの出力パス（--config でも指定可） |
| `--points-2d-original` | `points_2d_original` | 新名（旧 `--points-2d-in`） | 原本座標の2D点CSV（--config でも指定可） |
| `--points-2d` | `points_2d` | 新名（旧 `--points-2d-out`） | 前処理後座標の2D点CSVの出力パス（--config でも指定可） |
| `--expect-model` | `expect_model` | 変更なし（help に追記） | 期待する機種名（Exif Model）（--config でも指定可） |
| `--expect-focal-mm` | `expect_focal_mm` | 変更なし（help に追記） | 期待する実焦点距離[mm]（--config でも指定可） |
| `--overwrite` | `overwrite` | 変更なし（help に追記） | 出力ファイルの上書きを許可する（--config でも指定可） |
| `--config` | `config` | 新設 | 設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。優先順位は CLI > 設定YAML > 既定値。フラグ（--overwrite）は CLI から true 方向にだけ上書きできる（設定YAMLの true を false に戻すには設定YAMLを編集する）。推定（estimate_intrinsics_loop.py）と共用の設定YAMLを渡してよい（推定専用のキーは無視する） |

パーサーは `argparse.ArgumentParser(..., allow_abbrev=False)` で作り、オプション名の省略形を受け付けない（requirements.md §4 の後方互換の例外。改名により、従来の省略形が別のオプションとして解釈される危険を避けるため。例: `--o` が `--overwrite` に、`--points-2d-o` が `--points-2d-original` に解釈される。codex-09）。省略形は、`parse_args_with_config` の 1 回目の解釈（設定YAMLを読む前）で `unrecognized arguments` として終了コード 2 になる。旧名（`--out-toml`、`--points-2d-in`、`--points-2d-out`）は定義しない。`--points-2d` は `--points-2d-original` の前方一致でもあるが、完全一致なので `points_2d` になる。

#### 変換表・必須項目・相手のキー

```python
CONFIG_CONVERTERS = {
    "image": _yaml_str, "camera": _yaml_str, "images_dir": _yaml_str, "init_toml": _yaml_str,
    "points_2d_original": _yaml_str, "points_2d": _yaml_str,
    "expect_model": _yaml_str, "expect_focal_mm": float, "overwrite": _yaml_bool,
}
REQUIRED_OPTIONS = [("image", "--image"), ("camera", "--camera"), ("images_dir", "--images-dir"),
                    ("init_toml", "--init-toml"), ("points_2d_original", "--points-2d-original"),
                    ("points_2d", "--points-2d")]
IGNORED_KEYS = frozenset({"points_3d", "ply", "out_dir", "tmp_dir"})   # 推定のキー
```

#### `main` と `_run`

```python
def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)
    output_options = [("images_dir", "--images-dir"), ("init_toml", "--init-toml"), ("points_2d", "--points-2d")]
    empty = [label for dest, label in output_options if getattr(args, dest) == ""]
    if empty:   # 空のパスは書き込み先が実行ディレクトリになる・置換で途中失敗する（codex-04 指摘2）
        parser.error(f"空の値は指定できません: {', '.join(empty)}")
    return _run(args)
```

`main` では、統合パースの後、書き込みより前に、3 つの出力パス（`images_dir`・`init_toml`・`points_2d`）の値が空文字でないことを検査する。空文字があれば、項目名を列挙して終了コード 2（`parser.error`）。`camera` の空文字はこの検査の対象外で、`_run` のカメラ名の検査で終了コード 1 になる（FR-004。codex-05）。入力パス（`image`・`points_2d_original`）の空文字は、従来どおり読み込みのエラーになる

`_run` の変更点（それ以外は変えない）:

1. 最初に（出力済み検査より前に）カメラ名を検査する（FR-004）。条件とメッセージは `estimate_intrinsics_loop.py` の `main` と同じ:
   ```python
   if (camera in ("", ".", "..") or Path(camera).name != camera or "/" in camera or "\\" in camera):
       print(f"エラー: カメラ名にはパス区切りを含まない名前を指定してください: {camera}", file=sys.stderr)
       return 1
   ```
2. `args.out_toml` → `args.init_toml`、`args.points_2d_out` → `args.points_2d`、`args.points_2d_in` → `args.points_2d_original`（変数名 `out_toml`・`out_csv` などはそのままでよい）
3. 完了時の表示は従来どおり

モジュール docstring の使用例を新しいオプション名に書き換え、`--config job.yaml` の 1 行例を追記する。

### 4.3 推定 `estimate_intrinsics_loop.py`（FR-001・003・005）

> **注記（feat-044、2026-11-26）**: 推定の `IGNORED_KEYS` は feat-044 で `{"image", "photo_info", "points_2d_original", "overwrite"}` に変わった（`expect_model`・`expect_focal_mm` は削除され、未知のキーとして終了コード 2）。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

| 変更 | 内容 |
|---|---|
| パーサーの省略形 | `argparse.ArgumentParser(..., allow_abbrev=False)` にして、オプション名の省略形を受け付けない（requirements.md §4 の後方互換の例外。共用 YAML の前処理の `--image` を推定に渡したとき、`--images-dir` の省略形として受け付けられて画像ディレクトリに使われるのを防ぐ。実装時の発見 2026-11-25） |
| パーサー | `--toml` を `--init-toml`（dest `init_toml`、help「初期TOML（--config でも指定可）」）に置き換える。他のオプションは変えない。`--config` の help の末尾に「前処理（prepare_phone_photo.py）と共用の設定YAMLを渡してよい（前処理専用のキーは無視する）」を追記 |
| `CONFIG_CONVERTERS` | キー `toml` を `init_toml` に置き換える（他は変えない） |
| `REQUIRED_OPTIONS` | `("toml", "--toml")` を `("init_toml", "--init-toml")` に置き換える |
| `IGNORED_KEYS` | 新設: `frozenset({"image", "points_2d_original", "expect_model", "expect_focal_mm", "overwrite"})`（前処理のキー） |
| `main` | `parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)` |
| `_run` | `load_cameras_toml(args.toml)` → `load_cameras_toml(args.init_toml)`。レポートのヘッダの入力パスの列挙 `("toml", "camera", ...)` を `("init_toml", "camera", ...)` に変える（表示名が `init_toml:` になる） |
| 最終精緻化の呼び出し | 変えない（`refine_extrinsics.py` の `--toml` にはループ結果 TOML を渡す。これは `refine_extrinsics.py` 自身のオプション名） |
| docstring | 使用例を `--init-toml` に書き換え、共用 YAML の 1 行例を追記 |

### 4.4 エラーと終了コード

| 場面 | 終了コード | 出力 |
|---|---|---|
| 旧名のオプション（`--toml`、`--out-toml`、`--points-2d-in`、`--points-2d-out`） | 2 | argparse の `unrecognized arguments` |
| 旧名の設定キー（推定の `toml` など） | 2 | `設定YAMLに未知のキーがあります: ...` |
| 相手のキー（共用 YAML） | エラーなし | （無視） |
| 相手のオプションを CLI で指定（例: 推定に `--image`） | 2 | argparse の `unrecognized arguments` |
| 前処理のカメラ名の違反 | 1 | `エラー: カメラ名にはパス区切りを含まない名前を指定してください: ...` |
| その他（設定YAMLの誤り、必須項目の欠落など） | feat-042 と同じ | feat-042 と同じ |

### 4.5 境界条件

| 条件 | 振る舞い |
|---|---|
| 共用 YAML に `overwrite: true` | 前処理では上書き許可。推定では無視（推定は `--overwrite` を持たず、出力先は新規ディレクトリ限定のまま） |
| 共用 YAML に `tmp_dir` | 推定では従来どおり。前処理では無視 |
| 共用 YAML の相手のキーの値が不正（例: 推定に渡した `expect_focal_mm: abc`） | エラーにならない（FR-003）。同じ YAML を前処理に渡すと、キー名つきのエラー（終了コード 2） |
| 前処理の `images_dir` に既存の他のカメラの画像がある | 従来どおり（`{camera}.png` だけを出力済み検査の対象にする） |

## 5. ログ・デバッグ設計

既存方式のまま。相手のキーを無視したことは表示しない（ユーザー決定「黙って無視する」）。

## 6. テスト設計

### 6.1 新規 `tests/test_feat043_shared_yaml.py`

`tests/test_feat042_config_yaml.py`・`tests/test_feat041_intrinsics_loop.py`（テスト 7d の前処理のモック、`_bug006_setup` の推定のモック）の書き方に従う。

| # | テスト | 対応FR | 内容 | 期待 |
|---|---|---|---|---|
| 1 | 前処理の全キー | FR-002 | 前処理の全キー（相手のキーなし）を書いた YAML で `parse_args_with_config` | 各 dest が YAML の値（`expect_focal_mm` は float、`overwrite` は True） |
| 2 | 前処理の優先順位 | FR-002 | YAML `init_toml: a.toml` と CLI `--init-toml b.toml` | `b.toml` |
| 3 | 前処理の必須項目の欠落 | FR-002 | 空の YAML、CLI なし | 終了コード 2、stderr に 6 つの表示名（`--image` … `--points-2d`） |
| 4 | 共用 YAML（両方） | FR-003 | FR-001 の表の全キー（13 キー）を書いた 1 つの YAML を、前処理と推定の `parse_args_with_config` にそれぞれ渡す | どちらもエラーにならない。前処理の args に `ply`・`points_3d`・`out_dir`・`tmp_dir` の属性が無い。推定の args に `image`・`points_2d_original`・`expect_model`・`expect_focal_mm`・`overwrite` の属性が無い。共通のキー（`camera`・`images_dir`・`init_toml`・`points_2d`）は両方で同じ値 |
| 5 | 相手のキーの値は検査しない | FR-003 | 共用 YAML に `expect_focal_mm: abc` を書き、(a) 推定、(b) 前処理に渡す | (a) エラーにならない。(b) 終了コード 2、stderr に `expect_focal_mm` |
| 6 | どちらにも無いキー | FR-003 | 共用 YAML に `foo: 1` | 前処理・推定とも終了コード 2、stderr に「未知のキー」と `foo` |
| 7 | 旧名の拒否 | FR-001 | (a) 前処理に `--out-toml x`、(b) `--points-2d-in x`、(c) `--points-2d-out x`、(d) 推定に `--toml x`、(e) 推定に YAML キー `toml: x`。さらに前処理に、必須項目をすべて CLI でそろえたうえで (f) `--o`、(g) `--points-2d-o x`、(h) `--cam camA`（`--camera` の代わり）を加える（出力パスには既存のファイル（内容 "old"）を置く） | すべて終了コード 2（(a)〜(d)・(f)〜(h) は `unrecognized arguments`、(e) は「未知のキー」）。(f)〜(h) で既存の出力ファイルは内容 "old" のまま（codex-09） |
| 8 | 相手のオプションを CLI で指定 | FR-003 | 必須項目をそろえたうえで、推定に `--image x`、前処理に `--ply x` | どちらも終了コード 2（`unrecognized arguments`）。推定の `--image` が `--images-dir` として受け付けられないこと（実装時の発見。設計どおり `--image` で確認する） |
| 9 | `--points-2d` の完全一致 | FR-001 | 前処理に必須項目をすべて CLI で渡す（`--points-2d out.csv --points-2d-original in.csv`） | `args.points_2d == "out.csv"`、`args.points_2d_original == "in.csv"` |
| 10 | 前処理のカメラ名 | FR-004 | Exif 読み込みをモックし、`--camera sub/cam`（および `..`）で `main` | 終了コード 1、`images_dir`・`init_toml`・`points_2d` のどれも作られない |
| 11 | 前処理の完走（共用 YAML） | FR-002・003 | テスト 7d（test_feat041）と同じく Exif 読み込みをモックした小さな画像で、共用 YAML（推定のキーも含む）だけを渡して `main(["--config", path])` | 終了コード 0。`images_dir/{camera}.png`、`init_toml`、`points_2d` が YAML に書いたパスに作られる。`out_dir` は作られない |
| 12 | 推定が前処理の出力を読む（共用 YAML） | FR-003・005 | テスト 11 と同じ共用 YAML を推定の `parse_args_with_config` に渡す | `args.init_toml`・`args.images_dir`・`args.points_2d` が、テスト 11 で前処理が書いたパスと一致する |
| 13 | `parse_args_with_config` の既定動作 | 制約 | `ignored_keys` を渡さずに、未知のキーを書いた YAML で呼ぶ（`refine_extrinsics` の変換表） | 終了コード 2（従来どおり）。`refine_extrinsics.py`・`adjust_extrinsics_multiview.py` の既存テスト（feat-042）がそのまま通ることと合わせて確認 |
| 14 | `ignored_keys` と変換表の重なり | §4.1 | `converters` と重なる `ignored_keys` で呼ぶ | `AssertionError` |
| 15 | `--help` | FR-001 | 前処理と推定の `format_help()` | 前処理: `--init-toml`・`--points-2d-original`・`--points-2d`・`--config` を含み、`--out-toml`・`--points-2d-in`・`--points-2d-out` を含まない。推定: `--init-toml` を含み、`--toml ` を含まない |
| 16 | 推定の `main` が相手のキーを無視して処理に進む | FR-003 | `tests/test_feat041_intrinsics_loop.py` の `_bug006_setup` と同じモック（推定のテスト 19 と同じ）を使う。推定の必須項目に加えて前処理のキー（`image`・`points_2d_original`・`expect_model`・`expect_focal_mm: 5.4`・`overwrite: true`）も書いた共用 YAML で、推定の `main(["--config", path])` を呼ぶ。あわせて、同じ YAML を推定の `parse_args_with_config`（`IGNORED_KEYS` つき）に渡す | `main` はテスト 19 と同じ終了コード 3 で、`run_loop_iteration` が 8 回呼ばれる（処理に進む）。パース結果の args に `overwrite` 属性が無い（codex-02） |
| 17 | 前処理の空の値 | §4.2 | Exif 読み込みをモックし、必須項目をそろえた YAML のうち (a) `images_dir:`、(b) `init_toml:`、(c) `points_2d:` をそれぞれ空にし、`overwrite: true` も書いて前処理の `main` を呼ぶ | (a)(b)(c) とも終了コード 2、stderr に「空の値は指定できません」と該当の表示名。実行ディレクトリ（`monkeypatch.chdir(tmp_path)`）にも、他の出力パスにも、何も作られない |
| 18 | 推定の省略形の拒否 | §4.3 | 必須項目をすべて CLI でそろえたうえで、推定に (a) `--images photo.jpg`（`--images-dir` の省略形）、(b) `--cam` を `--camera` の代わりに使う、(c) `--ou x` を `--out-dir` の代わりに使う | すべて終了コード 2（`unrecognized arguments`）。設定YAMLは読まれない |

### 6.2 実データでの確認（実装担当がステップ6で実行）

新しい名前と共用 YAML で前処理 → 推定を実行し、本案件の前と同じ結果になることを確かめる（requirements.md 非機能要求「出力の同一性」）。判定はファイルの一致だけで、閾値の判定は行わない。

1. 事前確認: 次のパスが存在しないこと（`test ! -e` かつ `test ! -L`）。満たさなければ、設定YAMLも作らずに報告する（codex-07 指摘3）
   - `phase4/data/10F/job_feat043.yaml`（設定YAML自体）
   - `phase4/data/10F/feat043_images`
   - `phase4/data/10F/feat043_init.toml`
   - `phase4/data/10F/feat043_points_2d.csv`
   - `phase4/data/10F/feat043_out`
   - `phase4/data/10F/feat043_tmp`
2. 共用 YAML `phase4/data/10F/job_feat043.yaml` を作る（requirements.md §7 の例に `overwrite: true` を加え（推定が無視することも実データで通す）、そのうえで、`images_dir: phase4/data/10F/feat043_images`、`init_toml: phase4/data/10F/feat043_init.toml`、`points_2d: phase4/data/10F/feat043_points_2d.csv`、`out_dir: phase4/data/10F/feat043_out`、`tmp_dir: phase4/data/10F/feat043_tmp` にする）
3. プロジェクトルートで次を実行する:
   ```
   uv run --project phase4 python phase4/prepare_phone_photo.py --config phase4/data/10F/job_feat043.yaml
   TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py --config phase4/data/10F/job_feat043.yaml
   ```
4. 合格条件（すべて満たすこと）:
   - 前処理の終了コード 0。`cmp` で次がバイト単位で一致する: `feat043_images/s25_20261124_152436.png` と `s25_images/s25_20261124_152436.png`、`feat043_init.toml` と `s25_20261124_152436_init.toml`、`feat043_points_2d.csv` と `points_2d_s25_20261124_152436.csv`（いずれも `phase4/data/10F/` 配下。既存ファイルは feat-041 の前処理の出力）
   - 推定の終了コード 0。`cmp` で `feat043_out/s25_20261124_152436_loop.toml` と `feat041_out_it2/s25_20261124_152436_loop.toml`、`feat043_out/s25_20261124_152436_final.toml` と `feat041_out_it2/s25_20261124_152436_final.toml` がバイト単位で一致する（推定の主成果物。codex-01 指摘2）。`feat043_out/s25_20261124_152436_loop_report.txt` の反復行（`i=` で始まる行。末尾の所要時間 `| *.s` を除く）が `feat041_out_it2/s25_20261124_152436_loop_report.txt` と一致する。`feat043_out/s25_20261124_152436_eval.txt` の `median_raw`・`valid_ratio_raw`・判定が `feat041_out_it2` と一致する
5. 結果（各コマンドの終了コード、`cmp` の結果、比較した行）を `docs/issues/feat-043-shared-job-yaml-phone-photo/verification.md` に記録する。不一致があれば原因を調べずに報告して中断する

### 6.3 既存テストの変更（名前の置き換えだけ。期待値は変えない）

- `tests/test_feat041_intrinsics_loop.py`:
  - 推定の `main` / パーサーに渡す `"--toml"` を `"--init-toml"` に置き換える（テスト 7c、19、20、`_bug006_args`、その他 `eil.main` を呼ぶ箇所すべて）
  - 前処理（テスト 7d）の `"--out-toml"` を `"--init-toml"`、`"--points-2d-in"` を `"--points-2d-original"`、`"--points-2d-out"` を `"--points-2d"` に置き換える
- `tests/test_feat042_config_yaml.py`:
  - 推定用の設定キー `"toml"` を `"init_toml"` に、推定に渡す CLI の `"--toml"` を `"--init-toml"` に置き換える（`REQ_LOOP`、テスト 4b・12・14・16・17 など推定を対象とする箇所）
  - テスト 12 の「本案件の前のパーサーの dest 一覧」は、推定について `toml` を `init_toml` に読み替える
  - `refine_extrinsics.py`・`adjust_extrinsics_multiview.py` を対象とする箇所の `toml` / `--toml` は変えない（それらのツールの名前は変わらない）
  - テスト 16 の期待コマンドの `--toml`（`refine_extrinsics.py` の引数）は変えない
  - テスト 11c（省略形）の推定（`loop`）の部分は、推定が省略形を受け付けなくなることに合わせて期待値を変える（requirements.md §4 の後方互換の例外。意図的な変更）: `--c camA`（(a)・(d)）は stderr に `ambiguous` ではなく `unrecognized arguments` が出る（終了コード 2 は同じ）。(f) の対照は、`--cam camA` が終了コード 2 になり、完全な名前の `--camera camA` で `a.camera == "camA"` になることを確認する形に変える。`refine_extrinsics.py`・`adjust_extrinsics_multiview.py` の部分は変えない
  - テスト用の補助関数 `_parse(mod, argv)` は、`parse_args_with_config` に `getattr(mod, "IGNORED_KEYS", frozenset())` を渡すように変える（推定の `main` と同じ条件でパースするため。`refine_extrinsics.py`・`adjust_extrinsics_multiview.py` には `IGNORED_KEYS` が無いので従来どおり。codex-01 指摘1）
  - テスト 5（未知のキー）の推定の部分は、本案件で「相手のキー」になる `overwrite` ではなく、前処理にも推定にも無いキー（`seed_base`）で未知のキーのエラーを確認するように変える。`overwrite` が推定で無視されることは feat-043 のテスト 4 で確認する（requirements.md FR-003。この変更は期待値の変更を伴うが、FR-003 で仕様が変わるための意図的な変更。codex-01 指摘1）
- 上記（名前の置き換え、`_parse` の変更、テスト 5 の推定の部分、テスト 11c の推定の部分）以外の変更が必要になった場合は、実装担当は中断して報告する

テスト結果は `tests/results/feat-043_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する。

## 7. 設計判断（ADR 簡易版）

| 判断 | 採用 | 却下案と理由 |
|---|---|---|
| 重複の解消方法 | 同じ中身に CLI・YAML・両ツールで同じ名前を付け、共用 YAML に 1 回だけ書く（ユーザー決定） | 前処理ディレクトリの中に決まったファイル名で置く: 暗黙の規則で分かりにくい（ユーザーが却下）。キー名をどちらか一方に合わせるだけ: CLI と YAML で名前が食い違う（ユーザーが却下） |
| 相手のキー | 黙って無視（値も検査しない）（ユーザー決定） | 無視して一覧を表示: ユーザー選択外。値も検査する: 相手のツールの変換表に依存し、相手のツールを直したときに壊れやすい |
| 旧名 | 残さない | 互換用エイリアス: 名前が 2 つになり分かりにくい（ユーザー指示「同じ名前を使え」）。両ツールとも作ったばかりで利用者は限られる |
| 相手のキーの実装 | `parse_args_with_config` に省略可能な `ignored_keys` を追加 | 前処理・推定ごとに別のパース関数: feat-042 の共通関数と重複する。`parse_config_yaml`（`render_fps_video.py`）の変更: 他の 4 本に影響する |
| `{images_dir}/{camera}.png` | 既存の約束事として残す | `refine_extrinsics.py` に画像パスを直接渡すオプションを足す: 範囲が広がる（requirements.md §6 Won't） |

## 8. 完了時の更新対象（ステップ8で Claude Code 本体が行う）

- `docs/BACKLOG.md`: feat-043 を Closed にする
- `docs/CHANGELOG.md`: 完了内容を記録する
- `CLAUDE.md`: ディレクトリ構成の `prepare_phone_photo.py`・`estimate_intrinsics_loop.py` の説明を更新する（`--config`、共用 YAML、名前の変更）
- `README.md`: `prepare_phone_photo.py`・`estimate_intrinsics_loop.py` の節（feat-041 の節）のコマンド例・設定キー・設定YAMLの例を新しい名前に更新し、共用 YAML の使い方をその節の中に書く（独立した節は作らない）。スクリプト一覧の表の該当行も更新する
- `docs/issues/feat-041-intrinsic-loop-render-match/design.md` の CLI の節（§4.1.1、§4.2.1）、`docs/issues/feat-042-config-yaml-refine-tools/requirements.md` の FR-002（推定の表）と非機能要求「後方互換」の例外 2（推定の `--c` などの省略形）に、「feat-043 で名前を変更し、前処理・推定は省略形を受け付けない」と注記する

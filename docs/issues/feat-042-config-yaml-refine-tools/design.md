# feat-042 機能設計書: 精緻化ツール 3 本の設定 YAML（--config）対応

作成日: 2026-11-25
基準: `docs/DESIGN_STANDARD.md`
本書のコード例は意図を伝えるためのもので、そのままコピーして使うものではない。用語は requirements.md §2 に従う。

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 | §4.1、§4.4（パーサーへの `--config` 追加と `main` の置き換え） |
| FR-002 | §4.2（変換表）、§4.3（`_yaml_names`） |
| FR-003 | §4.1（`set_defaults` による優先順位） |
| FR-004 | §4.1（統合後の必須項目検証）、§4.4（`required=True` の解除） |
| FR-005 | §4.1（`parse_config_yaml` の既存チェック）、§4.3 |
| 非機能（後方互換・内部呼び出しの不変） | §4.4、§4.6 |

## 2. システム構成

### 2.1 モジュール構成（変更対象）

| ファイル | 種別 | 変更内容 |
|---|---|---|
| `phase4/refine_extrinsics.py` | 既存・変更 | 共通関数 `_yaml_names`、`parse_args_with_config` を新設。変換表 `CONFIG_CONVERTERS`、必須項目 `REQUIRED_OPTIONS` を新設。`build_arg_parser` と `main` 冒頭を変更 |
| `phase4/adjust_extrinsics_multiview.py` | 既存・変更 | 変換表 `CONFIG_CONVERTERS`、必須項目 `REQUIRED_OPTIONS` を新設。`build_arg_parser` と `main` 冒頭を変更。共通関数は `refine_extrinsics` から import |
| `phase4/estimate_intrinsics_loop.py` | 既存・変更 | 同上 |
| `tests/test_feat042_config_yaml.py` | 新規 | §6 のテスト |
| `tests/results/feat-042_test_result.txt` | 新規 | テスト結果 |

変更しないファイル:

| ファイル | 理由 |
|---|---|
| `phase4/render_fps_video.py` | `load_yaml_flat`、`_yaml_bool`、`parse_config_yaml`（`converters` 引数つき。feat-033）を import して使うだけ |
| `phase4/render_keypoints.py` | 関係しない |
| `matcher_lab/*`、`phase4/prepare_phone_photo.py`、`phase4/evaluate_overlay.py` | 対象外（requirements.md §6） |

### 2.2 依存関係

```
refine_extrinsics.py ──import──> render_fps_video（load_yaml_flat は parse_config_yaml 内で使用、_yaml_bool, parse_config_yaml）
                     （既存: ──import──> render_keypoints ──import──> render_fps_video。追加の依存は増えない）
adjust_extrinsics_multiview.py ──import──> refine_extrinsics（既存の import に _yaml_names, parse_args_with_config を追加）
                               ──import──> render_fps_video（_yaml_bool）
estimate_intrinsics_loop.py ──import──> refine_extrinsics（既存の import に parse_args_with_config を追加）
estimate_intrinsics_loop.py ──subprocess──> refine_extrinsics.py（CLI 引数のまま。変更しない）
```

`render_fps_video.py` のトップレベル import は numpy と `npz_to_c3d` だけで、torch に依存しない（feat-033 で確認済み）。3 本とも既に `render_keypoints` 経由で読み込んでいるので、ルート環境の pytest からの import 可否は変わらない。

## 3. 技術スタック

- Python 3.10、phase4 の uv 環境。新規ライブラリなし（`docs/TECH_STACK.md` の更新は不要）
- argparse（標準ライブラリ）の `set_defaults` で設定YAMLの値を既定値に昇格させる（feat-029 / feat-033 と同じ方式）

## 4. 詳細設計

### 4.1 統合パース `parse_args_with_config`（FR-001/003/004/005）

`phase4/refine_extrinsics.py` に新設し、3 本で共用する。

```python
def parse_args_with_config(
    argv: list[str] | None,
    parser: argparse.ArgumentParser,
    converters: dict[str, Callable[[str], object]],
    required_options: list[tuple[str, str]],
) -> argparse.Namespace:
```

- `argv`: コマンドライン引数（None なら `sys.argv[1:]`）。`main(argv)` の argv をそのまま渡す
- `parser`: 各ツールの `build_arg_parser()` が返すパーサー（`--config` を含む。§4.4）。この関数は `parser.set_defaults` でパーサーを書き換えるので、**呼び出しごとに新しく生成したパーサーを渡す**（同じパーサーを使い回すと、前回の設定YAMLの値が既定値として残る。各ツールの `main` は毎回 `build_arg_parser()` を呼ぶ。codex-16 指摘1）
- `converters`: そのツールの変換表（§4.2）
- `required_options`: `(dest 名, 表示名)` のリスト。表示名は CLI のオプション名（例: `("images_dir", "--images-dir")`）

処理（意図伝達用の擬似コード）:

```python
def parse_args_with_config(argv, parser, converters, required_options):
    tokens = sys.argv[1:] if argv is None else list(argv)

    # 1. 実際に書かれたオプション名で、--config の省略形を検出する（最初に行う。--help の処理や設定YAMLの読み込みより前）
    #    対象は --co〜--confi（=値 付きを含む）だけ。--c は --camera / --cameras と区別できないので本パーサーに判定させる。
    #    "--" より後ろは位置引数扱いなので検査しない
    for tok in tokens:
        if tok == "--":
            break
        name = tok.split("=", 1)[0]
        if name.startswith("--co") and name != "--config" and "--config".startswith(name):
            parser.error(f"--config は省略せずに指定してください: {name}")

    # 2. 本パーサーで 1 回解釈する（設定YAMLはまだ読まない）。省略形は従来どおり許可。
    #    --c のような曖昧な省略形や CLI の型エラーは、ここで終了コード 2 になる。--help はここで表示して終了コード 0
    first = parser.parse_args(tokens)

    # 3. 設定YAMLを読み、既定値に昇格させてから、もう一度解釈する（CLI の明示指定が優先）
    args = first
    if first.config is not None:
        try:
            defaults = parse_config_yaml(first.config, parser, converters=converters)
        except UnicodeDecodeError as e:   # load_yaml_flat は UTF-8 固定。この例外は parse_config_yaml が捕まえない
            parser.error(f"--config が UTF-8 として読めません: {first.config}: {e}")
        parser.set_defaults(**defaults)
        args = parser.parse_args(tokens)

    missing = [label for dest, label in required_options if getattr(args, dest) is None]
    if missing:
        parser.error(f"必須項目が未指定です（CLIまたは--configで指定）: {', '.join(missing)}")
    return args
```

- 優先順位（FR-003）: 設定YAMLの値を `parser.set_defaults` で既定値に昇格させる。CLI で明示指定された値は argparse が上書きする。`nargs="+"` の引数（`--cameras` 等）も、CLI で指定すれば CLI のリストで丸ごと置き換わる
- `parse_config_yaml`（`render_fps_video.py`。変更しない）が次を行う:
  - 読み込み失敗: `--config が読み込めません: <path>: <原因>`
  - `config` キー: `設定YAMLに config キーは書けません（連鎖読み込み不可）`
  - 未知のキー: `設定YAMLに未知のキーがあります: <キー>。有効なキー: <一覧>`
  - 型変換の失敗: `設定YAML キー '<キー>': <原因>`
  
  いずれも `parser.error`（終了コード 2）
- 必須項目の検証（FR-004）: 統合した後の値で行う。欠けた項目は表示名で列挙する
- 省略形の扱い（codex-01 指摘2、codex-02、codex-03）: 本パーサーは従来どおり省略形を許可する（既存の 3 本の挙動を変えない。feat-029 / feat-033 の本パーサーは `allow_abbrev=False` だが、ここで合わせると `--image` などの既存の省略形がすべて使えなくなるため合わせない）。設定YAMLを読む前に、次の 2 段で省略形を判定する:
  - 手順 2（本パーサーでの 1 回目の解釈）: `--c camA` は `--camera` / `--cameras` / `--config` の曖昧な省略形として終了コード 2（argparse の `ambiguous option`）。`--config a.yaml --c camA` も同じく、設定YAMLを読む前に終了コード 2。従来通っていた `--c` は使えなくなる（requirements.md 非機能要求「後方互換」の例外 2）
  - 手順 1（トークンの検査。最初に行う）: `--co`〜`--confi`（`=値` つきを含む）のように `--config` の省略形として書かれたオプション名があれば、終了コード 2（「--config は省略せずに指定してください」）。`--conf a.yaml --config b.yaml` のように完全形と併用しても、トークンを見るので検出できる。`--help` と併用しても（`--conf a.yaml --help`）、ヘルプ表示より前に終了コード 2 になる（codex-04）。`--c` はこの検査の対象外で、手順 2 の本パーサーが曖昧な省略形として判定する。`--` より後ろのトークンは検査しない（`--help -- --conf` は従来どおりヘルプ表示で終了コード 0。codex-05）
  - 1 回目の解釈で必須項目が欠けていてもエラーにならない（`required=True` を外しているため。必須項目の検証は手順 3 の後に行う）
  - `--help`（省略形のトークンが無い場合）は手順 2 でヘルプを表示して終了コード 0。設定YAMLは読まない（従来と同じ）
  - 以前の設計（`--config` だけを拾う事前パーサー → 本パーサー、の 2 段）は、2 つのパーサーの解釈の差から抜け道が続けて見つかったため取り下げた（codex-02、codex-03）

### 4.2 変換表と必須項目（FR-002/004）

キー名は argparse の dest 名と完全に一致させる（`set_defaults(**defaults)` にそのまま渡すため）。各表は requirements.md FR-002 の表と 1 対 1 で対応する。

`refine_extrinsics.py`:

```python
CONFIG_CONVERTERS = {
    "toml": str, "ply": str, "images_dir": str, "points_3d": str, "points_2d": str,
    "out_toml": str, "out_report": str, "cameras": _yaml_names,
    "overwrite": _yaml_bool, "seed_base": int, "tmp_dir": str,
}
REQUIRED_OPTIONS = [("toml", "--toml"), ("ply", "--ply"), ("images_dir", "--images-dir"),
                    ("points_3d", "--points-3d"), ("points_2d", "--points-2d"),
                    ("out_toml", "--out-toml"), ("out_report", "--out-report")]
```

`adjust_extrinsics_multiview.py`:

```python
CONFIG_CONVERTERS = {
    "toml": str, "ply": str, "images_dir": str, "init_toml": str, "init_cameras": _yaml_names,
    "out_toml": str, "out_report": str, "cameras": _yaml_names,
    "overwrite": _yaml_bool, "seed": int, "tmp_dir": str, "fresh_match": _yaml_bool,
}
REQUIRED_OPTIONS = [("toml", "--toml"), ("ply", "--ply"), ("images_dir", "--images-dir"),
                    ("out_toml", "--out-toml"), ("out_report", "--out-report")]
```

`estimate_intrinsics_loop.py`:

```python
CONFIG_CONVERTERS = {
    "toml": str, "camera": str, "ply": str, "images_dir": str,
    "points_3d": str, "points_2d": str, "out_dir": str, "tmp_dir": str,
}
REQUIRED_OPTIONS = [("toml", "--toml"), ("camera", "--camera"), ("ply", "--ply"),
                    ("images_dir", "--images-dir"), ("points_3d", "--points-3d"),
                    ("points_2d", "--points-2d"), ("out_dir", "--out-dir")]
```

- 上の変換表の `str` は、実装では `_yaml_str`（§4.3a。NUL 文字を拒否する）にする
- 数値（`seed_base`、`seed`）は CLI と同じ素の `int` 変換だけを行う。`ValueError` は `parse_config_yaml` の既存の except 節が捕まえ、キー名つきのエラーにする
- `_yaml_bool`（小文字の true / false のみ）は `render_fps_video.py` から import する

### 4.3a 文字列の変換 `_yaml_str`（FR-002/005）

`phase4/refine_extrinsics.py` に新設し、3 本で共用する。§4.2 の変換表で `str` と書いた箇所は、すべて `_yaml_str` を使う（codex-12 指摘2）。

```python
def _yaml_str(value: str) -> str:
    """設定YAMLの文字列値（パス・カメラ名）。NUL 文字を含む値は拒否する。"""
    if "\x00" in value:
        raise argparse.ArgumentTypeError("NUL 文字を含む値は指定できません")
    return value
```

- 空文字はそのまま受け付ける（§4.6）
- `_yaml_names` も、分割の前に同じ検査を行う

### 4.3 複数値の変換 `_yaml_names`（FR-002/005）

`phase4/refine_extrinsics.py` に新設する（`adjust_extrinsics_multiview.py` から import して共用）。

```python
def _yaml_names(value: str) -> list[str]:
    """設定YAMLの複数カメラ名（スペース区切り、1 個以上）を変換する。"""
    if "\x00" in value:
        raise argparse.ArgumentTypeError("NUL 文字を含む値は指定できません")
    names = value.split()
    if not names:
        raise argparse.ArgumentTypeError("カメラ名を 1 個以上、スペース区切りで指定してください")
    return names
```

- `str.split()`（引数なし）で、連続する空白やタブも区切りとして扱う
- 重複の除去は行わない（CLI の `nargs="+"` と同じ。既存の `main` の扱いに任せる）

### 4.4 パーサーと `main` の変更（FR-001/004）

3 本の `build_arg_parser` の変更点（これ以外の引数定義は変えない）:

| 変更 | 内容 |
|---|---|
| `required=True` の引数 | `required=True` を外して `default=None` にする（§4.2 の `REQUIRED_OPTIONS` のもの）。help の末尾に「（--config でも指定可）」を追記する |
| `--config` | 新設。`default=None`。help は「設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。優先順位は CLI > 設定YAML > 既定値。フラグ（{フラグ名}）は CLI から true 方向にだけ上書きできる（設定YAMLの true を false に戻すには設定YAMLを編集する）」。`{フラグ名}` は `refine_extrinsics.py` が `--overwrite`、`adjust_extrinsics_multiview.py` が `--overwrite/--fresh-match`。`estimate_intrinsics_loop.py` はフラグが無いので、フラグの一文を省く |

`main` 冒頭の変更（3 本とも）:

```python
# 変更前
args = build_arg_parser().parse_args(argv)
# 変更後
parser = build_arg_parser()
args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS)
```

- `main` のそれ以降は変えない。既存の検証（出力の上書き確認、`--init-toml` 未指定のエラー、bug-006 のカメラ名の制限と `--out-dir` / `--tmp-dir` の新規作成など）は、統合後の `args` に従来どおり適用される
- モジュールの docstring の使用例の下に、`--config run.yaml` の 1 行例を追記する
- **挙動の変更（意図的。FR-004 に明記）**: 必須項目が欠けたときのエラーが、argparse 標準の `the following arguments are required: ...` から `必須項目が未指定です（CLIまたは--configで指定）: ...` に変わる。終了コード 2 は変わらない

### 4.5 エラーハンドリング

設定YAMLの読み込みと統合パースのエラー（表の最終行以外）は、すべて `parser.error`（usage とメッセージを標準エラー出力に出し、終了コード 2）で処理する。表の最終行（統合パースより後の、各ツールの `main` の既存のエラー）は従来どおりで、終了コードも既存のまま（例: `--init-toml` の未指定や、bug-006 の既存の `out_dir` は終了コード 1）。

| エラー | 検出 | 出力 |
|---|---|---|
| 設定YAMLが存在しない・読めない | `parse_config_yaml` | `--config が読み込めません: <path>: <原因>` |
| 設定YAMLが UTF-8 として読めない | `parse_args_with_config`（`UnicodeDecodeError` を捕まえる） | `--config が UTF-8 として読めません: <path>: <原因>` |
| `config` キー | `parse_config_yaml` | `設定YAMLに config キーは書けません（連鎖読み込み不可）` |
| 未知のキー（他のツールのキーを含む） | `parse_config_yaml` | `設定YAMLに未知のキーがあります: <キー>。有効なキー: <一覧>` |
| bool 値の不正 / 複数値が 0 個 / 数値の変換失敗 | 各変換関数 → `parse_config_yaml` | `設定YAML キー '<キー>': <原因>` |
| 必須項目の欠落 | `parse_args_with_config` | `必須項目が未指定です（CLIまたは--configで指定）: <表示名の一覧>` |
| それ以降の入力エラー | 各ツールの `main` の既存処理（変更しない） | 既存のメッセージ・終了コードのまま |

### 4.6 境界条件

| 条件 | 振る舞い |
|---|---|
| 空の設定YAML（有効な行なし） | 既定値の昇格なし。必須項目が CLI にも無ければ統合後の検証でエラー |
| 同じキーの重複行 | 後の行が勝つ（`load_yaml_flat` の仕様。feat-029 と同じ） |
| `:` を含まない空でない行 | 読み飛ばす（`load_yaml_flat` の仕様） |
| 文字列キーの値が空（`camera:`） | 空文字として受け付け、後段の既存の検証に任せる |
| 値にスペースを含むパス | 最初の `:` より後ろ全体（前後の空白を除く）が値になる。複数値キー（`cameras`、`init_cameras`）だけは §4.3 の規則でスペースで分割する |
| インデントした行（ネストのつもりの記述。例: `tmp_dir:` の次の行に `  overwrite: true`） | `load_yaml_flat`（feat-029 以来の共通パーサー。変更しない）は行頭の空白を除いてから読むので、インデントした行も通常のキーとして適用される（この例では `tmp_dir` は空文字、`overwrite` は True）。ネストは検出しない。feat-029 / feat-033 と同じ既存仕様を引き継ぐ（codex-10 指摘1）。設定YAMLはフラットに書く（requirements.md §2） |
| `--config` だけを指定し、YAML に処理の開始に必要な項目がすべてある | 実行を始められる（FR-001）。`adjust_extrinsics_multiview.py` は必須項目 5 つに加えて `init_toml` が必要（本案件の前から `main` が未指定をエラーにする。終了コード 1） |
| `estimate_intrinsics_loop.py` の最終精緻化 | `refine_extrinsics.py` を従来どおり CLI 引数で subprocess 実行する（`--config` は渡さない） |

## 5. ログ・デバッグ設計

- 既存方式に従う（標準出力に進捗、標準エラー出力にエラー）
- 設定YAMLの読み込みに成功したときのログは追加しない（feat-029 / feat-033 と同じ。エラー時だけ表示する）

## 6. テスト設計

新規 `tests/test_feat042_config_yaml.py`。`tests/test_feat033_config_yaml.py` と `tests/test_feat041_intrinsics_loop.py` の構成に従う（ルートの uv 環境で実行、`sys.path` に `phase4/` を追加）。設定YAMLは `tmp_path` に書く。終了コード 2 のエラーは `pytest.raises(SystemExit)` と `capsys` で確かめる。

| # | テスト | 対応FR | 内容 | 期待 |
|---|---|---|---|---|
| 1 | 全キーの適用（3 本それぞれ） | FR-002 | §4.2 の全キーを書いた YAML で `parse_args_with_config` を呼ぶ | 各 dest が、対応する CLI 指定で `parse_args` した結果と同じ値（`cameras` 等はリスト、bool は True、数値は int） |
| 2 | 優先順位 | FR-003 | `refine_extrinsics.py`: YAML `seed_base: 100` と CLI `--seed-base 200`、YAML だけ、両方なし。YAML `cameras: a b` と CLI `--cameras c` | 200 / 100 / `SEED_BASE_DEFAULT`。`cameras` は `["c"]` |
| 3 | フラグの非対称 | FR-003 | `adjust_extrinsics_multiview.py`: YAML `fresh_match: true`、CLI にフラグなし | `fresh_match` は True のまま |
| 4 | 必須項目の欠落（3 本それぞれ） | FR-004 | 必須項目を 1 つ除いた YAML、および空の YAML で CLI なし | 終了コード 2。stderr に「必須項目が未指定です」と、欠けた表示名（例: `--images-dir`）。空の YAML ではそのツールの必須項目すべて |
| 4b | 必須項目を CLI と YAML に分けて渡す（3 本それぞれ） | FR-004 | 必須項目のうち 1 つ（`--toml`）だけを CLI で渡し、残りを YAML に書く | `parse_args_with_config` がエラーにならずに戻り、全必須項目に値が入る（`toml` は CLI の値、残りは YAML の値） |
| 5 | 未知のキー | FR-005 | `estimate_intrinsics_loop.py` に `overwrite: true`、`refine_extrinsics.py` に `camera: x` | 終了コード 2。stderr に「未知のキー」とキー名 |
| 6 | `config` キー | FR-005 | `config: other.yaml` | 終了コード 2 |
| 7 | bool の不正 | FR-005 | `overwrite: True`、`overwrite: 1` | 終了コード 2。stderr に `overwrite` |
| 8 | 複数値の不正 | FR-005 | `cameras:`（値が空）、`init_cameras:   `（空白だけ） | 終了コード 2。stderr にキー名 |
| 9 | 複数値の区切り | FR-002 | `cameras: cam01  cam02\tcam03` | `["cam01", "cam02", "cam03"]` |
| 9b | インデントした行（既存仕様の固定） | 境界条件 | `refine_extrinsics.py`: 必須項目をそろえた YAML に `tmp_dir:` と、その次の行に `  overwrite: true`（行頭に空白 2 個）を書く | 終了コード 0 でパースされ、`args.tmp_dir == ""`、`args.overwrite is True`（`load_yaml_flat` の既存仕様。§4.6） |
| 10 | 数値の不正 | FR-005 | `seed_base: abc`、`seed: 1.5` | 終了コード 2。stderr にキー名 |
| 10b | NUL 文字 | FR-005 | 別々のケースで (a) `points_3d` だけに NUL 文字を含む値（`"a\x00b.csv"`）、(b) `cameras` だけに NUL 文字を含む値（`"cam\x00x"`）を書いた YAML（UTF-8 で書き出す）を渡す | (a)(b) それぞれ終了コード 2。stderr に、そのケースのキー名（`points_3d` / `cameras`）と「NUL 文字」 |
| 10c | 引数エラーの優先順位 | 非機能（後方互換の例外 1） | 3 本それぞれで、別々のケースとして (a) `--bogus` だけ、(b) 余分な位置引数 `extra` だけを渡す | (a)(b) とも終了コード 2。stderr に `unrecognized arguments`（必須項目の欠落より先に報告される） |
| 11 | 設定YAMLが無い | FR-001 | 存在しないパスを `--config` に指定 | 終了コード 2。stderr にパス |
| 11b | 設定YAMLが UTF-8 でない | FR-001 | 不正なバイト列（例: `b"toml: \xff\xfe\n"`）を書いたファイルを `--config` に指定 | 終了コード 2。stderr に「UTF-8 として読めません」とパス |
| 11c | 省略形（3 本それぞれ） | 非機能（後方互換の例外） | `render_fps_video.load_yaml_flat` を、呼ばれたら記録する関数で包む（monkeypatch）。必須項目をすべて CLI でそろえたうえで、(a) `--c camA`、(b) `--conf {存在する YAML}`、(c) `--conf {YAML1} --config {YAML2}`、(d) `--config {YAML} --c camA`、(e) `--co={YAML}`、(g) `--conf {YAML} --help`、(h) `--help -- --conf` を加える。対照として (f) `--ca camA`（`estimate_intrinsics_loop.py` は `--cam camA`）を加える（`adjust_extrinsics_multiview.py` では `--ca` は `--cameras` と解釈される） | (a)(d) 終了コード 2、stderr に `ambiguous`。(b)(c)(e)(g) 終了コード 2、stderr に「--config は省略せずに指定してください」（(g) はヘルプを表示しない）。(a)〜(e)・(g) のどれもローダーは呼ばれない。(h) ヘルプを表示して終了コード 0、ローダーは呼ばれない。(f) 従来どおり通る（`--camera` / `--cameras` に値が入る） |
| 12 | 後方互換（3 本それぞれ） | 非機能 | `--config` なしで、従来どおり全必須項目を CLI で渡す | 本案件の前のパーサーにあった dest（`config` 以外の全 dest）の値が、本案件の前の `parse_args` の結果と同じ（既定値も含む。比較する dest の一覧はテストに書き出す）。`args.config` は None |
| 13a | `main` が設定値で処理に入る（`refine_extrinsics.py`） | FR-001 | モック（`refine_extrinsics` のモジュール属性を monkeypatch）: `load_cameras_toml` は受け取ったパスを記録し、カメラ `camA`・`camB` を持つ辞書を返す。`select_camera` は辞書から取り出す。`load_points_3d_csv`・`load_points_2d_csv` は受け取ったパスを記録してダミーを返す。`validate_camera` は `{"ok": True}` を返す。`process_camera` は `(cam_name, seed_base_cam, tmp_dir)` を記録して `MatcherEnvError("stop")` を送出する。`sys.modules["render"]` に、受け取ったパスを記録してダミーを返す `load_ply` を持つダミーモジュールを入れる。全キー（`cameras: camB`、`seed_base: 123`、`tmp_dir`・`out_toml`・`out_report` は `tmp_path` 配下の新しいパス）を書いた YAML で `main(["--config", path])` を呼ぶ。続けて、同じ値を CLI だけで渡す（`out_toml`・`out_report`・`tmp_dir` だけ別の新しいパスにする） | 両方とも戻り値 1（`MatcherEnvError` の既存処理）。記録したパス（TOML・CSV 2 つ・PLY）が YAML の値と一致する。`process_camera` は 1 回だけ、`("camB", 123, Path(tmp_dir))` で呼ばれる。`out_report` のレポートに `seed_base: 123`、`=== camB ===`、`環境異常により中断: stop` がある。CLI だけの場合と、記録した値（出力・中間パスを除く）が同じ |
| 13b | `main` が設定値で処理に入る（`adjust_extrinsics_multiview.py`） | FR-001 | `tmp_path/work` に空の PLY ファイル `scene.ply` と画像ディレクトリ `imgs/` を作り、`monkeypatch.chdir(tmp_path/work)` で実行ディレクトリにする。YAML は別のディレクトリ `tmp_path/conf/` に置き、`ply: scene.ply`、`images_dir: imgs` のように実行ディレクトリ基準の相対パスで書く（`tmp_path/conf/` には PLY も画像ディレクトリも置かない。相対パスの基準が実行ディレクトリであることを判別するため。codex-10 指摘4）。モック: `adjust_extrinsics_multiview.build_camera_dict` を、受け取った引数（`toml`、`init_toml`、`init_cameras`）を記録して `InputError("stop")` を送出する関数に置き換える。(1) 処理の開始に必要な最小構成（必須 5 項目 + `init_toml`）だけを書いた YAML、(2) 全キー（`init_cameras: camA camB` を含む）を書いた YAML のそれぞれで `main(["--config", path])` を呼び、(2) は同じ値を CLI だけで渡した場合とも比べる。別のケースとして、`init_toml` を書かない YAML で呼ぶ | (1): PLY・画像ディレクトリの存在検査を通り（相対パスが実行ディレクトリ基準で解決される）、戻り値 1（`InputError("stop")`）で、`build_camera_dict` が `(toml, init_toml, None)` で呼ばれる（処理を開始できる）。(2): 両方とも戻り値 1。記録した引数が YAML の値（`init_cameras` は `["camA", "camB"]`）と一致し、CLI だけの場合と同じ。`init_toml` なしの場合: 既存の「--init-toml が指定されていません」で戻り値 1、`build_camera_dict` は呼ばれない |
| 14 | `main` の完走（`estimate_intrinsics_loop.py`） | FR-001 | `tests/test_feat041_intrinsics_loop.py` のテスト 19 と同じモック（`run_loop_iteration` 等）を使い、全必須項目を書いた YAML で `main(["--config", path])` を呼ぶ | テスト 19 と同じ終了コード 3。`run_loop_iteration` が 8 回呼ばれる |
| 15 | `--help` | FR-001/003 | 3 本の `build_arg_parser().format_help()` | `--config` を含む。`refine_extrinsics.py` と `adjust_extrinsics_multiview.py` は「true 方向」の説明を含む |
| 16 | 内部呼び出しの不変 | 非機能 | `estimate_intrinsics_loop.py` を設定YAMLだけで呼び、最終精緻化まで進める。YAML には必須項目に加えて `tmp_dir`（`tmp_path` 配下の未作成のパス）を書く（期待コマンドの `--tmp-dir` を組み立てるため）。モック: テスト 14 のモックに加え、(1) `run_loop_iteration` は毎回同じパラメータ（f=1280、cx=960、cy=540、k1=k2=0、任意の姿勢）を返して i=1 で収束させる、(2) `estimate_intrinsics_loop.subprocess.run` を、受け取ったコマンドを記録し、コマンド中の `--out-report` のパスに `"  {camera}: 受理(単峰)\n"` を書き、コマンド中の `--toml` の直後のパス（親が呼び出し直前に書いたループ結果 TOML。テスト 19 の `x.toml` ではない）の内容を読んで `--out-toml` の直後のパスに書いて、`returncode=0` の結果を返す関数に置き換える、(3) `estimate_intrinsics_loop.evaluate_and_write` を、統計値の辞書（`n_valid`・`valid_ratio`・`median`・`p95`・`mean`・`max`・`judge`）を返す関数に置き換える | 記録したコマンドの引数列全体が、期待するコマンドと完全に一致する。期待するコマンドは `[sys.executable, str(ROOT / "phase4/refine_extrinsics.py"), "--toml", <out_dir>/<camera>_loop.toml, "--ply", <YAML の ply>, "--images-dir", <YAML の images_dir>, "--points-3d", <YAML の points_3d>, "--points-2d", <YAML の points_2d>, "--out-toml", <out_dir>/<camera>_final.toml, "--out-report", <out_dir>/<camera>_refine_report.txt, "--cameras", <camera>, "--tmp-dir", <YAML の tmp_dir>/refine, "--overwrite"]`（feat-041 design.md §4.2.6 の形。`--config` を含まない）。`cwd` キーワードを渡さない（親のカレントディレクトリを継承する。bug-007 で変更）。終了コード 0 |
| 17 | YAML 由来の既存 `out_dir` の拒否（`estimate_intrinsics_loop.py`） | FR-004 | テスト 14 と同じモックで、`out_dir` に既存のディレクトリ（中にファイル `keep.txt`、内容 "old"）を書いた YAML で `main(["--config", path])` を呼ぶ | 戻り値 1（bug-006 の新規作成の検査）。`keep.txt` は内容 "old" のままで、ディレクトリ内に他のファイルが増えない。`run_loop_iteration` は呼ばれない |

- 既存テスト（`tests/test_feat026_refine.py`、`tests/test_feat035_multiview.py`、`tests/test_feat041_intrinsics_loop.py` を含む）は**変更せずに全件成功**すること
- テスト結果は `tests/results/feat-042_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する

### 実データでの動作確認

GPU を使う実行は行わない。`--config` は引数の受け渡しだけを変え、精緻化の処理は変えないため。テスト 13a・13b・14 で、`main` が設定YAMLの値で従来と同じ処理に入ることを確かめる。ユーザーの手動テスト（ステップ7）では、実データの設定YAMLで 3 本を起動し、従来の CLI と同じように始まることを確認する。

## 7. 設計判断（ADR 簡易版）

| 判断 | 採用 | 却下案と理由 |
|---|---|---|
| 汎用部品 | `render_fps_video.py` の `load_yaml_flat`・`_yaml_bool`・`parse_config_yaml` を import して再利用（変更しない） | 各ツールに複製: 同じパッケージ内の重複を増やす（feat-033 D-1 と同じ方針） |
| 統合パース関数の置き場所 | `refine_extrinsics.py` に 1 つ置き、他の 2 本は import して共用 | 各ツールに複製（feat-033 の方式）: 3 本で同じ 20 行が重複する。他の 2 本は既に `refine_extrinsics` から関数を import しており、依存は増えない。新しい共通モジュールを作る: 変更するファイルが増える |
| 複数値の書き方 | スペース区切り（ヒアリング 2026-11-25） | カンマ区切り・`;` 区切り: CLI の `--cameras cam01 cam02` と見た目が揃わない。カメラ名にスペースは使われていない |
| 相対パスの基準 | カレントディレクトリ（ヒアリング 2026-11-25） | YAML ファイルの場所基準: 既存 2 本と解釈が変わり、CLI で書いたときとも変わる |
| 必須項目 | `required=True` を外し、統合後に検証（feat-029 / feat-033 と同じ） | YAML があるときだけ required を外す動的なパーサー: 実行経路によってパーサーの形が変わり、テストと help が複雑になる |
| フラグの CLI 上書き | true 方向だけ（feat-029 / feat-033 と同じ） | `--no-overwrite` などの打ち消しオプションの追加: CLI が増え、範囲が広がる |
| 内部の subprocess 呼び出し | CLI 引数のまま | 設定YAMLを書き出して `--config` で渡す: 一時ファイルが増え、bug-006 で整理した書き込み先の扱いにも影響する |

## 8. 完了時の更新対象（ステップ8で Claude Code 本体が行う）

- `docs/BACKLOG.md`: feat-042 を Closed にする
- `docs/CHANGELOG.md`: 完了内容を記録する
- `CLAUDE.md`: ディレクトリ構成の 3 本の説明に `--config` 対応を追記する（ファイルの追加・削除はなし）
- `README.md`: 3 本の節のコマンド例に `[--config <設定YAML>]` を追記し、設定キーの一覧（requirements.md FR-002 の表）と設定YAMLの例を加える。`estimate_intrinsics_loop.py` の節には「bug-007 の修正まで、プロジェクトルートで実行するか、すべてのパスを絶対パスで指定する（設定YAMLに書くパスも同じ）」を明記する
- `docs/issues/feat-041-intrinsic-loop-render-match/design.md` §5 の「設定ファイルは使わない（CLI 引数のみ）」に「feat-042 で `--config` に対応」と注記する

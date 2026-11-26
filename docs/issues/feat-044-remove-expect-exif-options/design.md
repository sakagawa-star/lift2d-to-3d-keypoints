# 機能設計書: feat-044 前処理の Exif 依存の整理（写真情報ファイルの分離と expect 系オプションの削除）

## 1. 対応要求マッピング

| 要求 | 設計セクション |
|---|---|
| FR-001 書き出し | §4.1 |
| FR-002 前処理が写真情報ファイルから読む | §4.2.1、§4.2.2 |
| FR-003 削除対象の削除 | §4.2.3 |
| FR-004 共用設定YAMLの 3 ツールへの拡張 | §4.1（`IGNORED_KEYS`）、§4.2.4、§4.3 |

## 2. システム構成

### 2.1 変更対象

| ファイル | 変更 |
|---|---|
| `phase4/extract_photo_info.py` | 新規（§4.1） |
| `phase4/prepare_phone_photo.py` | `--photo-info` の追加、写真情報ファイルの読み込み、Exif 検査の整理、削除対象の削除（§4.2） |
| `phase4/estimate_intrinsics_loop.py` | `IGNORED_KEYS` の更新（§4.3） |
| `tests/test_feat041_intrinsics_loop.py` | 既存テストの変更（§6.1）、テスト 29 の追加（§6.2） |
| `tests/test_feat043_shared_yaml.py` | 既存テストの変更（§6.1）、テスト 19 の追加（§6.2） |
| `tests/test_feat044_photo_info.py` | 新規（§6.2） |

変更しないファイル: `phase4/refine_extrinsics.py`（`parse_args_with_config` は変えない）、`phase4/render_fps_video.py`（`load_yaml_flat` を import して使うだけ）、`phase4/evaluate_overlay.py`。

### 2.2 依存関係

- `extract_photo_info.py` は `prepare_phone_photo` の `PHOTO_INFO_KEY` と Exif タグ番号の定数 `_TAG_FOCAL_35MM`・`_IFD_EXIF`、`refine_extrinsics` の `parse_args_with_config`・`_yaml_str` を import する（bool のキーを持たないので `_yaml_bool` は使わない）。Exif の 35mm 換算焦点距離の読み込みは書き出しの中に持つ（§4.1.3。前処理の `read_exif` は使わない）
- `prepare_phone_photo.py` は追加で `render_fps_video.load_yaml_flat` を import する（写真情報ファイルの読み込み）

## 3. 技術スタック

変化なし（新規ライブラリなし）。

## 4. 詳細設計

### 4.1 書き出し `phase4/extract_photo_info.py`（新規。FR-001・FR-004）

#### 4.1.1 CLI

```
uv run --project phase4 python phase4/extract_photo_info.py \
    --image <原本画像> --photo-info <写真情報ファイルの出力パス。存在しない新しいパス> [--config <設定YAML>]
```

- パーサーは `argparse.ArgumentParser(description=..., allow_abbrev=False)`（省略形を受け付けない。feat-043 と同じ理由）
- 各オプションは `default=None`。help の末尾に「（--config でも指定可）」を付ける（前処理と同じ書き方）。`--photo-info` の help には「存在しない新しいパス（上書きしない）」と書く
- 上書きのオプションは持たない（ユーザー判断 2026-11-26。codex-06 の指摘を受け、上書きと安全な書き込みの両立を取り下げた）
- 終了コード: 0=成功、1=入力エラー・出力済み・Exif に値が無い・書き込み失敗、2=引数エラー
- モジュール docstring に、目的・実行方法（上記 CLI と `--config` での実行）・終了コード・設計根拠（本書 §4.1）を書く（前処理の docstring と同じ形）

モジュール定数:

```python
CONFIG_CONVERTERS = {"image": _yaml_str, "photo_info": _yaml_str}
REQUIRED_OPTIONS = [("image", "--image"), ("photo_info", "--photo-info")]
# 共用 YAML の相手（前処理・推定）のキー（feat-044）
IGNORED_KEYS = frozenset({"camera", "images_dir", "init_toml", "points_2d_original", "points_2d",
                          "overwrite", "points_3d", "ply", "out_dir", "tmp_dir"})
```

`PHOTO_INFO_KEY`（`"focal_length_35mm"`）は前処理に定義し（§4.2.1）、書き出しは import して使う（定義は 1 か所）。

#### 4.1.2 処理手順（`main(argv: list[str] | None = None) -> int`）

1. `args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)`
2. `args.photo_info == ""` なら `parser.error("空の値は指定できません: --photo-info")`（終了コード 2）
3. `out = Path(args.photo_info)`。`os.path.lexists(out)` なら「エラー: 出力先が既に存在します（上書きしません）。別の出力パスを指定してください: {out}」（削除を案内しない。原本画像や設定YAMLを誤って指定した場合に、案内に従って入力を消さないため。codex-07）を stderr に出して終了コード 1（リンク先の無いシンボリックリンクも存在するとみなす）。原本画像・設定YAMLは必ず存在するので、出力先に指定されてもここで止まり、書き換わらない
4. `f35 = read_focal_35mm(args.image)`（§4.1.3）。`OSError`（Pillow の `UnidentifiedImageError` は `OSError` の派生）なら「エラー: 画像を開けません: {image}（{e}）」で終了コード 1
5. `f35` が `None` または 0 以下なら、次のメッセージを stderr に出して終了コード 1（ファイルは書かない）:

   ```
   エラー: 画像の Exif に 35mm 換算焦点距離（FocalLengthIn35mmFilm）がありません: {image}
   写真情報ファイルを手で書いてください（書式: 1 行 "focal_length_35mm: <35mm換算焦点距離[mm]>"）: {out}
   ```

6. 写真情報ファイルを書く（§4.1.4）。内容 `text`（末尾に改行）:

   ```
   # 写真情報ファイル（extract_photo_info.py が Exif から書き出し。feat-044）
   focal_length_35mm: {f35}
   ```

   `{f35}` は `read_focal_35mm` が返す `int` を `str()` したもの（例: `23`）。`FileExistsError` なら手順 3 と同じメッセージで終了コード 1（同時実行で先に作られた場合）。それ以外の `OSError` なら「エラー: 写真情報ファイルを書けません: {out}（{e}）」で終了コード 1
7. 「35mm 換算焦点距離: {f35}」「写真情報ファイル: {out}」を標準出力に出して終了コード 0

`if __name__ == "__main__": sys.exit(main())`

#### 4.1.3 Exif の 35mm 換算焦点距離の読み込み（`read_focal_35mm(path: str) -> int | None`）

```python
def read_focal_35mm(path: str) -> int | None:
    """Exif の FocalLengthIn35mmFilm を int で返す。無い・変換できない場合は None。"""
    from PIL import Image  # 遅延import（ルート環境はPillowなし）

    with Image.open(path) as im:
        exif = im.getexif()
        merged = dict(exif)
        try:
            merged.update(exif.get_ifd(_IFD_EXIF))
        except Exception:
            pass
    v = merged.get(_TAG_FOCAL_35MM)
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return None
```

（Exif の取り出し方は前処理の `read_exif` と同じ。`int(inf)` の `OverflowError` も値なしとして扱う。codex-05）

#### 4.1.4 写真情報ファイルの書き込み（`write_photo_info(out: Path, text: str) -> None`）

全体を書き終えてから、出力先が存在しない場合だけ公開する（codex-06）:

1. `out.parent.mkdir(parents=True, exist_ok=True)`
2. `fd, tmp = tempfile.mkstemp(dir=out.parent, prefix=f".{out.name}.", suffix=".tmp")`（同じディレクトリに名前の重ならない一時ファイルを排他的に作る。既存のファイルやシンボリックリンクの先を開かない）
3. `try:` の中で `os.fdopen(fd, "w", encoding="utf-8")` で `text` を書いて閉じ、`os.link(tmp, out)` で公開する（`out` が既に存在すれば `FileExistsError`。確定時にも上書きしない）
4. `finally:` で `os.unlink(tmp)`（成功・失敗のどちらでも一時ファイルを残さない。`os.unlink` 自体の `FileNotFoundError` は無視する）
5. 例外はそのまま送出し、呼び出し元（§4.1.2 の 6）が終了コード 1 にする

`os.link` が使えないファイルシステム（ハードリンク非対応）では `OSError` になり、終了コード 1（「写真情報ファイルを書けません」）。前処理の `_write_atomic_text`（固定名 `{out}.tmp`）は使わない

### 4.2 前処理 `phase4/prepare_phone_photo.py`（FR-002・FR-003・FR-004）

#### 4.2.1 写真情報ファイルの読み込み（新設）

```python
PHOTO_INFO_KEY = "focal_length_35mm"


def load_photo_info(path: str) -> float:
    """写真情報ファイルから 35mm 換算焦点距離を読む（feat-044）。不正は InputError。"""
```

処理（すべて `InputError` で報告。メッセージにパスを含める）:

1. `d = load_yaml_flat(path)`。`OSError` または `UnicodeDecodeError` なら「写真情報ファイルを読み込めません: {path}（{e}）」
2. `PHOTO_INFO_KEY` 以外のキーがあれば「写真情報ファイルに未知のキーがあります: {キーをソートしてカンマ区切り}（使えるキーは focal_length_35mm だけ）: {path}」
3. `PHOTO_INFO_KEY` が無ければ「写真情報ファイルに focal_length_35mm がありません: {path}」
4. `v = float(d[PHOTO_INFO_KEY])`。`ValueError` なら、または `math.isfinite(v)` でないか `v <= 0` なら「写真情報ファイルの focal_length_35mm が正の数ではありません: {値}: {path}」
5. `v` を返す

#### 4.2.2 `_run` の変更

- カメラ名の検査の直後、「1. 出力済み検査」の前に、写真情報ファイルと出力の衝突の検査を入れる（codex-09）:

  ```python
  # 0b. 写真情報ファイルと出力の衝突（feat-044）
  outs = [out_png, out_toml, out_csv]
  outs += [Path(str(p) + ".tmp") for p in outs]   # _write_atomic_* の一時ファイル
  pi = Path(args.photo_info)
  for o in outs:
      same = os.path.realpath(pi) == os.path.realpath(o)
      if not same and os.path.exists(pi) and os.path.exists(o):
          same = os.path.samefile(pi, o)          # ハードリンクによる別名
      if same:
          print(f"エラー: --photo-info が前処理の出力（または書き込み途中の一時ファイル）と同じファイルを指しています: {o}",
                file=sys.stderr)
          return 1
  ```

  `args.photo_info` が空の値の場合はこの検査を通り過ぎ、1b で終了コード 1 になる（`os.path.realpath("")` はカレントディレクトリになるが、出力パスとは一致しない）。`import os` を足す

- 「1. 出力済み検査」の直後、「2. Exif 検査」の前に次を入れる:

  ```python
  # 1b. 写真情報ファイル（feat-044）
  try:
      f35 = load_photo_info(args.photo_info)
  except InputError as e:
      print(f"エラー: {e}", file=sys.stderr)
      return 1
  ```

- 「2. Exif 検査」の呼び出しを `errors = validate_exif(exif, width, height)` にする
- 「5. 初期焦点距離」を `f = initial_focal_px(f35, width, height, OUT_W)` にする（Exif の `FocalLengthIn35mmFilm` を使わない）
- これ以外の手順・メッセージ・出力は変えない。カメラ名の検査は従来どおり最初に行う

`validate_exif` の変更:

- シグネチャを `def validate_exif(exif: dict, width: int, height: int) -> list[str]:` にする
- `FocalLengthIn35mmFilm` の検査（`f35 is None or f35 <= 0`）を削除する
- 削除対象の 2 つの検査を削除する（§4.2.3）
- 残る検査は `Orientation`（None は可）、`width <= height`、`DigitalZoomRatio`（None は可）の 3 つで、メッセージは変えない

`read_exif` は `FocalLengthIn35mmFilm` を読まない（§4.2.3 の 4。タグに異常な値があっても前処理が止まらないようにする。codex-05）。

#### 4.2.3 削除対象の削除（FR-003）

1. モジュール docstring の実行方法: `        [--expect-model "Galaxy S25"] [--expect-focal-mm 5.4] [--overwrite]` の行を `        [--overwrite]` にし、`--image <原本JPEG> --camera <カメラ名>` の行を `--image <原本JPEG> --photo-info <写真情報ファイル> --camera <カメラ名>` にする。冒頭の説明の「Exifから初期内部パラメータTOML」を「写真情報ファイル（35mm換算焦点距離。extract_photo_info.py が Exif から書き出すか手で書く）から初期内部パラメータTOML」にする。終了コードの行は変えない
2. 定数 `FOCAL_MM_TOL = 0.05` を削除する
3. Exif タグ番号の定数 `_TAG_MODEL = 0x0110` と `_TAG_FOCAL_LENGTH = 0x920A` を削除する（他の `_TAG_*` と `_IFD_EXIF` は残す。`_TAG_FOCAL_35MM`・`_IFD_EXIF` は書き出しが import する）
4. `read_exif`: `model = merged.get(_TAG_MODEL)` から `model = model.decode(...)` までの 3 行を削除し、戻り値の辞書から `"Model"`・`"FocalLength"`・`"FocalLengthIn35mmFilm"` を削除する。戻り値は `{"Orientation", "DigitalZoomRatio", "Size"}` の 3 項目。定数 `_TAG_FOCAL_35MM` は書き出しが import するので残す
5. `validate_exif` から `if expect_model is not None:` の節と `if expect_focal_mm is not None:` の節を削除する
6. `build_arg_parser` から `--expect-model` と `--expect-focal-mm` を削除する
7. `CONFIG_CONVERTERS` から `"expect_model"` と `"expect_focal_mm"` を削除する

#### 4.2.4 `--photo-info` の追加（FR-002・FR-004）

- `build_arg_parser` の `--image` の直後に `p.add_argument("--photo-info", default=None, help="写真情報ファイル（35mm換算焦点距離。extract_photo_info.py が書き出すか手で書く）（--config でも指定可）")` を足す
- `CONFIG_CONVERTERS` に `"photo_info": _yaml_str` を足す。最終形は `image` `photo_info` `camera` `images_dir` `init_toml` `points_2d_original` `points_2d` `overwrite` の 8 キー
- `REQUIRED_OPTIONS` の `("image", "--image")` の直後に `("photo_info", "--photo-info")` を足す（7 項目）
- `IGNORED_KEYS`（推定のキー `points_3d` `ply` `out_dir` `tmp_dir`）は変えない（書き出しのキーは前処理のキーに含まれるため）
- `main` の空値の検査（`output_options`）は変えない。`--photo-info` が空の値の場合は、`load_photo_info("")` が `OSError` を受けて終了コード 1 になる（入力の誤りとして扱う）

### 4.3 推定 `phase4/estimate_intrinsics_loop.py`（FR-003・FR-004）

```python
IGNORED_KEYS = frozenset({"image", "points_2d_original", "expect_model", "expect_focal_mm", "overwrite"})
```

を

```python
IGNORED_KEYS = frozenset({"image", "photo_info", "points_2d_original", "overwrite"})
```

に変える。これ以外は変えない。

### 4.4 エラーと終了コード

| 入力 | ツール | 動作 |
|---|---|---|
| 写真情報ファイルの出力先が既に存在する（原本画像・設定YAMLを指定した場合を含む） | 書き出し | 終了コード 1、書かない（確定時にも保証） |
| 書き出しに `--overwrite` を指定 | 書き出し | argparse の `unrecognized arguments`、終了コード 2 |
| 写真情報ファイルの書き込み失敗 | 書き出し | 終了コード 1、不完全なファイル・一時ファイルを残さない |
| Exif に 35mm 換算焦点距離が無い・0 以下 | 書き出し | 終了コード 1、書かない、手で書く方法を表示 |
| 原本画像を開けない | 書き出し | 終了コード 1 |
| `--photo-info` が空 | 書き出し | 終了コード 2 |
| 写真情報ファイルが前処理の出力・一時ファイルと同じファイル（別名を含む） | 前処理 | 終了コード 1、何も書かない（`--overwrite` があっても） |
| 写真情報ファイルが無い・読めない・未知のキー・値が不正 | 前処理 | 終了コード 1、何も書かない |
| `--expect-model` / `--expect-focal-mm` を CLI で指定 | 前処理 | argparse の `unrecognized arguments`、終了コード 2 |
| `expect_model` / `expect_focal_mm` を設定YAMLに書く | 3 ツールとも | 未知のキー、終了コード 2 |

### 4.5 境界条件

- 写真情報ファイルの値が小数（例: `23.5`）でも前処理は受け付ける。書き出しは Exif の整数値をそのまま書く
- Exif の無い画像（例: Exif を消した JPEG、PNG）: 書き出しは終了コード 1。人が写真情報ファイルを書けば、前処理は `Orientation`・`DigitalZoomRatio` の検査をせずに進む（値が `None` のため）
- 写真情報ファイルのコメント行（`#` 始まり）と `:` を含まない行は読み飛ばす（`load_yaml_flat` の既存の挙動）

## 5. ログ・デバッグ設計

- 書き出しの標準出力: 「35mm 換算焦点距離: {値}」「写真情報ファイル: {パス}」
- 前処理の標準出力は変えない（「初期焦点距離 f = ...」はそのまま）

## 6. テスト設計

Exif の読み込みは、既存テストと同じく差し替えて試験する（前処理は `ppp.read_exif`、書き出しは `epi.read_focal_35mm`。ルート環境に Pillow が無いため。`read_focal_35mm` 自体の変換は §6.2 No.12 で偽の `PIL` を入れて試験する）。テストで写真情報ファイルが必要な場合は、`tmp_path` に `focal_length_35mm: 24\n` を書いたファイルを作って渡す（既存テストの Exif 差し替えの `FocalLengthIn35mmFilm: 24` と同じ値なので、初期焦点距離の期待値は変わらない）。

### 6.1 既存テストの変更

`tests/test_feat041_intrinsics_loop.py`:

- テスト 4（`test_4_validate_exif`）: `validate_exif` の呼び出しを `ppp.validate_exif(exif, w, h)` の 3 引数にする。`cases` から次の 5 件を削除する: `FocalLengthIn35mmFilm` が `None` の件と `0` の件、`(ok, 4000, 3000, "Pixel 9", None)`、`(ok, 4000, 3000, None, 6.0)`、`({**ok, "FocalLength": None}, 4000, 3000, None, 5.4)`。残る 3 件（Orientation 6、縦長、DigitalZoomRatio 2.0）は `(exif, w, h)` の 3 要素にする。ループは `for exif, w, h in cases: assert len(ppp.validate_exif(exif, w, h)) == 1, (exif, w, h)`。冒頭の合格 3 件も 3 引数にする。さらに `assert ppp.validate_exif({**ok, "FocalLengthIn35mmFilm": None}, 4000, 3000) == []` を足す（前処理は Exif の 35mm 換算焦点距離を検査しない）
- テスト 7d（`test_7d_prepare_png`）: `ppp.main([...])` の 2 回とも、`"--expect-model", "Galaxy S25", "--expect-focal-mm", "5.4"` があれば削除し、`"--photo-info", <focal_length_35mm: 24 の写真情報ファイル>` を足す。期待値は変えない

`tests/test_feat043_shared_yaml.py`:

- `PREP_REQ` に `"photo_info": "pi.yaml"` を足す
- `ALL_KEYS` から `"expect_model"`・`"expect_focal_mm"` を削除する
- `PREP_ONLY` を `{"image", "photo_info", "points_2d_original", "overwrite"}` にする
- テスト 1: `d` から `"expect_model"`・`"expect_focal_mm"` を削除し、最後の検査を `assert a.overwrite is True` にする
- テスト 5: 相手のキーの不正値の例を `"overwrite": "abc"` に変え、最後の検査を `assert "overwrite" in capsys.readouterr().err` にする（前処理では bool 変換エラー、推定では無視。テストの目的は変えない）
- `_shared_job`: 末尾の `"expect_focal_mm": "5.4"` を削除し、`"photo_info"` に `focal_length_35mm: 24` の実ファイルのパスを入れる
- 前処理の `main` を呼び、写真情報ファイルの読み込み（`_run` の 1b）まで進むテストでは、`photo_info` に `focal_length_35mm: 24` の実ファイルのパスを渡す（`_prep_cli(..., photo_info=<パス>)` など）。カメラ名の検査・出力済み検査・引数エラーで先に止まるテストは変えない
- テスト 17（`test_17_empty_output_path`）: YAML の辞書 `d` に `"photo_info"`（`focal_length_35mm: 24` の実ファイルのパス）を足し、`capsys` を受け取って、`SystemExit` の code が 2 であることに加えて stderr に `空の値は指定できません` と対象のオプション名（`--images-dir` / `--init-toml` / `--points-2d`）が含まれることを確認する（必須項目の検査で先に止まって合格するのを防ぐ。codex-09）
- `_mock_exif` の戻り値は変えない（`Model`・`FocalLength` が残っていても参照されない）

### 6.2 追加するテスト

新規 `tests/test_feat044_photo_info.py`（`sys.path` の設定と import は `tests/test_feat043_shared_yaml.py` と同じ方式。`import extract_photo_info as epi`、`import prepare_phone_photo as ppp`）:

| No | 対象 | 内容 | 合格条件 |
|---|---|---|---|
| 1 | 書き出しの正常系 | `epi.read_focal_35mm` を `23` を返す関数に差し替え、`epi.main(["--image", "x.jpg", "--photo-info", str(out)])` | 終了コード 0。`out` の内容が `"# 写真情報ファイル（extract_photo_info.py が Exif から書き出し。feat-044）\nfocal_length_35mm: 23\n"` と完全一致。`{out}.tmp` が残らない。`ppp.load_photo_info(str(out)) == 23.0` |
| 2 | 書き出しの値なし | 差し替えの戻り値を `None` と `0` の 2 通り | 終了コード 1、`out` が作られない、stderr に `focal_length_35mm:` が含まれる |
| 3 | 書き出しの出力済み | `out` を先に作っておく（内容 `x`） | 終了コード 1、内容が `x` のまま |
| 4 | 書き出しの画像を開けない | `epi.read_focal_35mm` を `OSError` を送出する関数に差し替え | 終了コード 1、`out` が作られない |
| 5 | 書き出しの空のパス | `--photo-info ""` | `SystemExit` の code が 2 |
| 6 | 書き出しの設定YAML | `test_feat043_shared_yaml.py` の `ALL_KEYS`（§6.1 の変更後。`overwrite: true` を含む）と同じ全キーに `photo_info` を `out` にした共用設定YAMLで `epi.main(["--config", conf])` | 終了コード 0（相手のキーは無視される。`overwrite` も無視） |
| 7 | 書き出しの省略形 | `--photo`（`--photo-info` の省略形）で指定 | `SystemExit` の code が 2 |
| 8 | 前処理の写真情報ファイルの読み込み | `ppp.load_photo_info` に、`focal_length_35mm: 23`、`focal_length_35mm: 23.5`、コメント行つきのファイル | それぞれ 23.0、23.5、23.0 |
| 9 | 前処理の写真情報ファイルの不正 | 存在しないパス、空ファイル、`foo: 1` を含む、値が `abc`・`0`・`-1`・`inf`・`nan` | どれも `InputError` |
| 10 | 前処理が Exif の値を必要としない | `ppp.read_exif` を `{"Orientation": None, "DigitalZoomRatio": None, "Size": (64, 48)}` を返す関数に差し替え（Exif の無い画像を模す）、写真情報ファイル `focal_length_35mm: 24` で前処理（画像・2D 点 CSV はテスト 7d と同じ作り方） | 終了コード 0。初期 TOML の fx が `24 * 80 / 43.27 * 30`（テスト 7d と同じ期待値） |
| 11 | 前処理の写真情報ファイル不正時は何も書かない | No.9 の `foo: 1` のファイルで前処理（入力は No.10 と同じ） | 終了コード 1、出力 3 つがどれも作られない |
| 12 | `read_focal_35mm` の変換 | `sys.modules["PIL"]` に偽の `PIL` パッケージ（`Image.open` が、`getexif()` で `get_ifd(_IFD_EXIF)` が `{_TAG_FOCAL_35MM: 値}` を返す辞書派生オブジェクトを返すコンテキストマネージャ）を `monkeypatch.setitem` で入れ、値を `23`、`23.0`、`float("inf")`、`"abc"`、タグなしの 5 通りにする | それぞれ 23、23、None、None、None（例外を送出しない） |
| 13 | 既存ファイルを出力先に指定 | (a) `--image` と `--photo-info` に同じパス（中身 `x`）、(b) 設定YAML `job.yaml` に `photo_info: <job.yaml 自身のパス>` を書いて `--config job.yaml`、(c) `--photo-info` に原本（中身 `x`）を指すシンボリックリンクのパス | どれも終了コード 1、原本・設定YAMLの中身が変わらない |
| 14 | 一時ファイルが既存のリンク先を上書きしない | `out` は存在しない。`{out}.tmp` という名前で原本（中身 `x`）を指すシンボリックリンクを作っておき、No.1 と同じ書き出し | 終了コード 0、`out` の内容が No.1 と同じ、原本の中身が `x` のまま、`out` のディレクトリに `.{out.name}.` で始まる一時ファイルが残らない |
| 15 | 公開の失敗で何も残さない | `epi.os.link` を `OSError` を送出する関数に差し替えて No.1 と同じ書き出し | 終了コード 1、`out` が作られない、一時ファイルが残らない |
| 16 | 書き出しの `--overwrite` | `--overwrite` を付ける | `SystemExit` の code が 2 |
| 18 | 前処理の写真情報ファイルと出力の衝突 | 写真情報ファイル（`focal_length_35mm: 24`）を用意し、入力は No.10 と同じで、(a) `--init-toml` に写真情報ファイルと同じパス・`--overwrite` あり、(b) `--photo-info` を `{init_toml}.tmp` と同じパスにする（写真情報ファイルをそのパスに置く）・`--overwrite` なし、(c) `--init-toml` のパスに写真情報ファイルへのハードリンクを作っておき `--overwrite` あり | どれも終了コード 1、写真情報ファイルの内容が変わらない、前処理後画像と 2D 点 CSV が作られない |
| 17 | 前処理の `read_exif` が焦点距離タグを扱わない | `sys.modules["PIL"]` に No.12 と同じ方式の偽の `PIL` を入れ（`Image.open` の戻り値は `size = (4000, 3000)` を持つ）、IFD に `{_TAG_FOCAL_35MM: float("inf"), _TAG_ORIENTATION: 1}` を入れて `ppp.read_exif("x.jpg")` | 例外を送出しない。戻り値のキーが `{"Orientation", "DigitalZoomRatio", "Size"}` で、`FocalLengthIn35mmFilm` を含まない |

`tests/test_feat041_intrinsics_loop.py` テスト 29（`test_29_expect_options_removed`、FR-003）:

1. `ppp.build_arg_parser().format_help()` に `--expect-model` と `--expect-focal-mm` が含まれず、`--photo-info` が含まれる
2. 前処理の必須引数（`--photo-info` を含む）に `--expect-model x` を足して `ppp.main` を呼ぶと `SystemExit` の code が 2。`--expect-focal-mm 5.4` についても同じ

`tests/test_feat043_shared_yaml.py` テスト 19（`test_19_removed_keys_rejected`、FR-003。既存の最大番号は 18）: `expect_model` と `expect_focal_mm` それぞれについて、`{**ALL_KEYS, キー: 値}`（値は `"Galaxy S25"` / `"5.4"`）を書いた設定YAMLを、書き出し・前処理・推定の 3 つで `parse_args_with_config` 経由（既存の `_parse` 補助関数。書き出しは `extract_photo_info` モジュールを渡す）で解釈すると `SystemExit` の code が 2、stderr にキー名が含まれる

### 6.3 実行と保存

- `uv run pytest -v` 全件。出力を `tests/results/feat-044_test_result.txt` に保存する

### 6.4 実データでの確認（実装担当がステップ6で実行。GPU 不要）

プロジェクトルートで実行する。入力は feat-043 の検証記録（`docs/issues/feat-043-shared-job-yaml-phone-photo/verification.md`）にある共用 YAML `phase4/data/10F/job_feat043.yaml` と、その前処理の出力（`phase4/data/10F/feat043_images/`、`phase4/data/10F/feat043_init.toml`、`phase4/data/10F/feat043_points_2d.csv`）。どれかが存在しない場合は中断して報告する。

1. 検証用の新しいディレクトリ `phase4/data/10F/feat044_check/` を `mkdir phase4/data/10F/feat044_check`（`-p` を付けない）で作る。既に存在すれば mkdir が失敗するので、中断して報告する（既存のファイルを上書きしないため）。以下の検証用ファイルと出力はすべてこのディレクトリの配下に置く
2. `job_feat043.yaml` を写して `phase4/data/10F/feat044_check/job.yaml` を作り、写しに対して次を行う（`job_feat043.yaml` 自体は編集しない）: `expect_model` と `expect_focal_mm` の 2 行を削除する。`tmp_dir` の行を削除する（推定は実行しないため）。`photo_info: phase4/data/10F/feat044_check/photo_info.yaml` を足す。`images_dir` / `init_toml` / `points_2d` / `out_dir` を `phase4/data/10F/feat044_check/images`、`phase4/data/10F/feat044_check/init.toml`、`phase4/data/10F/feat044_check/points_2d.csv`、`phase4/data/10F/feat044_check/out` に変える。それ以外の行（`overwrite: true` を含む）は変えない
3. `uv run --project phase4 python phase4/extract_photo_info.py --config phase4/data/10F/feat044_check/job.yaml` が終了コード 0。`photo_info.yaml` の `focal_length_35mm` が 23
4. `uv run --project phase4 python phase4/prepare_phone_photo.py --config phase4/data/10F/feat044_check/job.yaml` が終了コード 0
5. 出力 3 つが feat-043 の出力とバイト単位で一致する（`cmp phase4/data/10F/feat044_check/images/s25_20261124_152436.png phase4/data/10F/feat043_images/s25_20261124_152436.png`、`cmp phase4/data/10F/feat044_check/init.toml phase4/data/10F/feat043_init.toml`、`cmp phase4/data/10F/feat044_check/points_2d.csv phase4/data/10F/feat043_points_2d.csv`）
6. Exif の無い原本 JPEG での確認（FR-002 の受け入れ基準。codex-05）:
   1. `uv run --project phase4 python -c "import cv2; cv2.imwrite('phase4/data/10F/feat044_check/noexif.jpg', cv2.imread('phase4/data/10F/s25_original/20261124_152436.jpg', cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION))"` で、Exif を持たない 4000x3000 の JPEG を作る（OpenCV の `imwrite` は Exif を書かない）。`uv run --project phase4 python -c "from PIL import Image; print(len(Image.open('phase4/data/10F/feat044_check/noexif.jpg').getexif()))"` が `0` であることを確認する
   2. 書き出し: `--image phase4/data/10F/feat044_check/noexif.jpg --photo-info phase4/data/10F/feat044_check/noexif_photo_info.yaml` で終了コード 1、`noexif_photo_info.yaml` が作られない、stderr に手で書く方法が出る
   3. 人が書く場合を模して、`phase4/data/10F/feat044_check/noexif_photo_info.yaml` に `focal_length_35mm: 23` の 1 行を書く
   4. 前処理を CLI で実行する: `--image phase4/data/10F/feat044_check/noexif.jpg --photo-info phase4/data/10F/feat044_check/noexif_photo_info.yaml --camera s25_20261124_152436 --images-dir phase4/data/10F/feat044_check/noexif_images --init-toml phase4/data/10F/feat044_check/noexif_init.toml --points-2d-original phase0/data/points_2d_10F.csv --points-2d phase4/data/10F/feat044_check/noexif_points_2d.csv` が終了コード 0
   5. `cmp phase4/data/10F/feat044_check/noexif_init.toml phase4/data/10F/feat043_init.toml` と `cmp phase4/data/10F/feat044_check/noexif_points_2d.csv phase4/data/10F/feat043_points_2d.csv` が一致（初期焦点距離と 2D 点の変換は、写真情報ファイルの値と画像サイズだけで決まるため）。前処理後画像は JPEG の再圧縮で画素が変わるため比較しない
7. 削除前の `job_feat043.yaml`（2 行を含む。編集しない）を書き出し・前処理・推定の 3 つに `--config` で渡すと、どれも終了コード 2（引数の解釈で止まるので、何も書かない。推定は GPU 処理の前に止まる）
8. 結果を `docs/issues/feat-044-remove-expect-exif-options/verification.md` に記録する（コマンド、終了コード、`cmp` の結果、`photo_info.yaml` の内容）

推定（GPU）は実行しない（推定のコードの変更は `IGNORED_KEYS` だけで、§6.2 テスト 19 と本節 7 で確認できる。前処理の出力がバイト単位で同じなら推定の結果も変わらない）。

## 7. 設計判断（ADR 簡易版）

- **expect 系オプションをオプションごと削除する**: ユーザー判断（2026-11-26）。基準は「設定YAMLは人が手で書くので、人が決める必要がある値だけを置く。Exif の値を人が見て書き写す項目は間違えるので置かない」
- **Exif から読む値を写真情報ファイルに分ける**: ユーザー判断（2026-11-26）。Exif の無い画像でも前処理を使えるようにするため。項目は処理に必須の 35mm 換算焦点距離だけ（ユーザー判断）
- **写真情報ファイルは Exif から自動で書き出す**（Exif の無い画像のときだけ人が書く）: ユーザー判断（2026-11-26）。書き写しによる間違いを避ける
- **書き出しを別スクリプトにする**（前処理の中で「写真情報ファイルが無ければ Exif から作り、あれば読む」と切り替えない）: 入力の出どころが実行のたびに変わる暗黙の動作を避け、Exif が無いときに人が書く場所を明確にするため（ユーザー承認 2026-11-26）
- **書き出しは上書きしない**（出力先は存在しない新しいパスだけ）: ユーザー判断（2026-11-26）。上書きを持たせると、原本画像・設定YAMLとの衝突の検査と安全な置き換えの仕組みが必要になり、レビューで同じ論点の指摘が続いた（codex-05・06。行き詰まり検出で方針を取り下げ）。上書きしなければ既存のファイルを壊す経路が構造的になくなる。再実行では既存の写真情報ファイルを使い回す（書き出しを飛ばす）か手で削除する。`overwrite` は前処理だけのキーになる
- **書き出しは Exif が無ければ失敗する**（既定値で書かない）: 推測した値で写真情報ファイルを作ると、人が書いたものと区別できず、誤った初期値で推定が進むため
- **向き・デジタルズームの検査は Exif に値があるときだけ行う**: 写真情報ファイルに入れない（ユーザー判断）。Exif の無い画像では検査できないが、従来も値が無ければ検査しなかった
- **Exif の検査を期待値との突き合わせに置き換えない**: 突き合わせる期待値を人が書く必要があり、上の基準に反する
- **テストの Exif 差し替え辞書の `Model`・`FocalLength` は残す**: 削除しても期待値が変わらず、変更量を増やすだけのため

## 8. 完了時の更新対象（ステップ8で Claude Code 本体が行う）

- `README.md`:
  - 冒頭のスクリプト一覧の表に `extract_photo_info.py` の行を足す（`prepare_phone_photo.py` の行の前）。`prepare_phone_photo.py` の行の「Exif から初期 TOML」を「写真情報ファイルから初期 TOML」に変える
  - `estimate_intrinsics_loop.py ほか` の節: 手順を 3 コマンド（書き出し → 前処理 → 推定）に変え、書き出しのコマンド例・写真情報ファイルの書式と例・Exif の無い画像の場合（手で書いて書き出しを飛ばす）を足す。前処理のコマンド例から `[--expect-model "Galaxy S25"] [--expect-focal-mm 5.4]` を削除し `--photo-info` を足す。キー一覧に書き出しのキーを足し、前処理のキーから `expect_model`・`expect_focal_mm` を削除して `photo_info` を足し、推定が無視するキーを更新する。共用設定YAMLの例から 2 行を削除し `photo_info` を足す。書き出しの終了コードと、書き出しは上書きしない（出力先が既存なら終了コード 1）ことを足す。再実行の手順として「写真情報ファイルが既にあれば書き出しを飛ばし、そのファイルを使う。作り直す場合は、そのファイルが写真情報ファイルであることを確かめてから削除する」と書く（エラーメッセージでは削除を案内しないため、利用ガイドに書く。codex-07）。`photo_info` も `out_dir`・`tmp_dir` の配下に置けないことを、前処理の出力の配置の注意と並べて書く（すべてこの節の中に書く）
- `CLAUDE.md`: ディレクトリ構成に `extract_photo_info.py` の行を足す（`prepare_phone_photo.py` の前）。`prepare_phone_photo.py` の説明の「Exif から初期TOML」を「写真情報ファイル〔extract_photo_info.py が Exif から書き出すか手で書く〕から初期TOML」に変える
- feat-041 の requirements.md FR-001（入力と終了コード 1 の条件）と design.md §4.1.1・§4.1.3（検査項目・初期焦点距離の入力元）・§9（実データでの動作確認のコマンド例。`--expect-model` / `--expect-focal-mm` を含む）、feat-043 の requirements.md FR-001 の表と design.md §4.2・§4.3（`IGNORED_KEYS`）: 「feat-044 で変更」の注記を足す（本文は当時の記録として残す）
- 旧仕様への案内の更新（例どおりに書くと終了コード 2 になる記述を残さない）:
  - `docs/issues/feat-042-config-yaml-refine-tools/requirements.md` FR-002 の注記（feat-043 の注記。推定が `expect_model`・`expect_focal_mm` を無視すると書き、現在の仕様として feat-043 を案内している箇所）の末尾に「（feat-044 で `expect_model`・`expect_focal_mm` は削除され、3 ツールとも未知のキーとして終了コード 2。前処理には `photo_info` が必須になった。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照）」を足す
  - `docs/issues/feat-043-shared-job-yaml-phone-photo/README.md` の共用 YAML の例と、`docs/issues/feat-043-shared-job-yaml-phone-photo/requirements.md` §7 の共用 YAML の例の、それぞれ直前に「注記（feat-044、{完了日}）: 現行版では `expect_model`・`expect_focal_mm` は削除され、前処理には `photo_info`（写真情報ファイル）が必須になった。この例のままでは終了コード 2 になる。現行の例は README の `estimate_intrinsics_loop.py ほか` の節を参照」を足す（例そのものは当時の記録として残す）
- `docs/BACKLOG.md`（件名を本案件の範囲に合わせて更新し Closed）、`docs/CHANGELOG.md`、本案件 README のステータス。本案件 README の「概要」を範囲拡大後の内容（写真情報ファイルの分離・書き出しの新設・expect 系の削除）に書き直し、「合意した方式」が実装と一致している（書き出しは `--overwrite` を持たず既存パスを拒否、`overwrite` は前処理だけのキー）ことを確認する（codex-09）

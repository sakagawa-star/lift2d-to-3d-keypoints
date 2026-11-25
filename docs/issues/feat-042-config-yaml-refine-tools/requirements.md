# feat-042 要求仕様書: 精緻化ツール 3 本の設定 YAML（--config）対応

作成日: 2026-11-25
基準: `docs/REQUIREMENTS_STANDARD.md`
入力資料:
- feat-029 / feat-033 の要求仕様書。次の方式はそちらで確定済み: 方式は YAML、優先順位は CLI が YAML を上書き、パーサーは簡易フラットパーサー（新規ライブラリなし）
- 本案件のヒアリング（2026-11-25。README.md「ヒアリング結果」）

## 1. プロジェクト概要

- **何を作るのか**: 次の 3 本に `--config <YAMLパス>` オプションを追加し、CLI オプションの値を設定 YAML から読み込めるようにする
  - `phase4/refine_extrinsics.py`（feat-026）
  - `phase4/adjust_extrinsics_multiview.py`（feat-035）
  - `phase4/estimate_intrinsics_loop.py`（feat-041 / bug-006）
- **なぜ作るのか**: 3 本とも必須のパス引数が 5〜7 個あり、コマンドが長い。定型部分をファイルにまとめて、入力を短くし、実行条件を記録として残す（feat-029 / feat-033 と同じ動機）
- **誰が使うのか**: 本プロジェクトの研究者
- **どこで使うのか**: phase4 の uv 環境（torch / gsplat / CUDA が必要）。開発と動作確認はローカル機（gtune2、RTX 5060 Ti）で行う。コマンド例の `TORCH_CUDA_ARCH_LIST="9.0+PTX"` はローカル機向けの値である

## 2. 用語定義

feat-029 / feat-033 と同じ用語を使う。

| 用語 | 定義 |
|---|---|
| 設定YAML | `--config` で渡すフラット構造（`key: value` のみ。ネスト・リスト構文なし）の YAML ファイル。`#` で始まる行と空行は無視する。行頭の空白は無視されるので、インデントした行も通常のキーとして読まれる（ネストは検出しない。feat-029 / feat-033 と同じ） |
| 簡易フラットパーサー | `render_fps_video.py` の `load_yaml_flat`（feat-029 で実装）。行を最初の `:` で分割し、キーと値を文字列として読む |
| 設定キー | 設定YAMLに書けるキー。CLI オプション名の先頭の `--` を除き、ハイフンをアンダースコアに置き換えた名前（argparse の dest 名と同じ）。FR-002 の表ですべて列挙する |
| 明示指定 | コマンドラインでオプションを実際に与えること（デフォルト値の採用は含まない） |
| 対象ツール | 上記の 3 本 |

## 3. 機能要求一覧

### FR-001: `--config` オプションと設定YAMLの読み込み

- **機能名**: 設定YAMLの読み込み
- **概要**: 対象ツールのそれぞれに `--config <パス>` を追加し、設定YAMLを読み込んで CLI オプションの値として使えるようにする
- **入力**: 設定YAMLのパス
- **出力**: 読み込まれた設定値（以降の処理は本案件の前の実装と同じ）
- **詳細**:
  - 設定YAMLは簡易フラットパーサーで読む（新規ライブラリは追加しない）
  - `--config` を指定しない場合、動作は本案件の前の実装と同じにする（後方互換）。例外は非機能要求「後方互換」に列挙したものだけ（必須項目欠落時のエラー文言、`unrecognized arguments` との同時発生時の報告順、`--config` と重なる省略形）
  - `--config` は省略せずに書く。`--config` の省略形（`--co` など）や、`--config` と区別できない曖昧な省略形（`--c`）を書いた場合は、設定YAMLを読まずに終了コード 2 で終了する（非機能要求「後方互換」の例外 2）
  - 設定YAMLの文字コードは UTF-8 とする。指定したパスが存在しない、読めない、または UTF-8 として読めない場合は、パスを含むエラーメッセージを出して終了コード 2 で終了する
  - 設定YAML内の相対パスは、CLI で書いた場合と同じく、コマンドを実行したカレントディレクトリを基準に解釈する（ヒアリング 2026-11-25。既存 2 本と同じで、特別な処理はしない）。ただし `estimate_intrinsics_loop.py` の最終精緻化は、本案件の前から子プロセスをプロジェクトルート基準で起動するため、プロジェクトルート以外で相対パスを使うと最終精緻化でパスがずれる（§5.1、bug-007）。bug-007 の修正までは、プロジェクトルートで実行するか、すべてのパスを絶対パスで指定する
- **受け入れ基準**: 対象ツールのそれぞれについて、処理の開始に必要な項目（FR-004 の必須項目。`adjust_extrinsics_multiview.py` はそれに加えて、本案件の前から `main` 内で未指定をエラーにしている `init_toml`）をすべて設定YAMLに書けば、`--config <YAML>` だけで実行を始められる。これは、ツール本体の重い処理をスタブ化したユニットテストで確認する。`--config` を使わない既存テストは全件成功する
- **優先度**: Must

### FR-002: 設定キーと型変換・検証

- **機能名**: 設定キーの型変換・検証
- **概要**: 設定キーは対象ツールごとに次の表のものに限る。値には CLI と同じ型変換を適用する
- **入力**: 簡易フラットパーサーが返す文字列の値
- **出力**: 型変換済みの設定値
- **詳細**: 設定キーの全列挙（これ以外のキーはエラー。FR-005）

`refine_extrinsics.py`（11 キー）:

| 設定キー | 対応する CLI | 型・書式 |
|---|---|---|
| `toml` | `--toml` | 文字列（パス） |
| `ply` | `--ply` | 文字列（パス） |
| `images_dir` | `--images-dir` | 文字列（パス） |
| `points_3d` | `--points-3d` | 文字列（パス） |
| `points_2d` | `--points-2d` | 文字列（パス） |
| `out_toml` | `--out-toml` | 文字列（パス） |
| `out_report` | `--out-report` | 文字列（パス） |
| `cameras` | `--cameras` | スペース区切りのカメラ名 1 個以上（例: `cameras: cam01 cam02`） |
| `overwrite` | `--overwrite` | `true` または `false`（小文字のみ） |
| `seed_base` | `--seed-base` | int |
| `tmp_dir` | `--tmp-dir` | 文字列（パス） |

`--config` 自身は設定キーにならない（FR-005 の 2）。

`adjust_extrinsics_multiview.py`（12 キー）:

| 設定キー | 対応する CLI | 型・書式 |
|---|---|---|
| `toml` | `--toml` | 文字列（パス） |
| `ply` | `--ply` | 文字列（パス） |
| `images_dir` | `--images-dir` | 文字列（パス） |
| `init_toml` | `--init-toml` | 文字列（パス） |
| `init_cameras` | `--init-cameras` | スペース区切りのカメラ名 1 個以上 |
| `out_toml` | `--out-toml` | 文字列（パス） |
| `out_report` | `--out-report` | 文字列（パス） |
| `cameras` | `--cameras` | スペース区切りのカメラ名 1 個以上 |
| `overwrite` | `--overwrite` | `true` または `false`（小文字のみ） |
| `seed` | `--seed` | int |
| `tmp_dir` | `--tmp-dir` | 文字列（パス） |
| `fresh_match` | `--fresh-match` | `true` または `false`（小文字のみ） |

`estimate_intrinsics_loop.py`（8 キー）:

| 設定キー | 対応する CLI | 型・書式 |
|---|---|---|
| `toml` | `--toml` | 文字列（パス） |
| `camera` | `--camera` | 文字列 |
| `ply` | `--ply` | 文字列（パス） |
| `images_dir` | `--images-dir` | 文字列（パス） |
| `points_3d` | `--points-3d` | 文字列（パス） |
| `points_2d` | `--points-2d` | 文字列（パス） |
| `out_dir` | `--out-dir` | 文字列（パス） |
| `tmp_dir` | `--tmp-dir` | 文字列（パス） |

  - 複数のカメラ名（`cameras`、`init_cameras`）はスペース区切りで書く（ヒアリング 2026-11-25）。要素が 0 個（値が空）の場合はエラーとする（CLI の `nargs="+"` と同じく 1 個以上が必要）
  - 型変換に失敗した場合は、設定キー名を含むエラーメッセージを出して終了コード 2 で終了する
  - CLI 側に値域の検証が無い項目（`seed_base`、`seed`）には、YAML 側でも値域の検証を追加しない（CLI と同じ変換だけ行う）
  - 文字列キーの値が空（例: `camera:`）の場合は空文字として受け付け、後段の既存の検証に任せる（feat-033 と同じ）
  - 文字列キー（パス・カメラ名）の値に NUL 文字（`\x00`）が含まれる場合はエラーとする（コマンドライン引数では渡せない値で、設定YAMLで新たに入り得るため。codex-12 指摘2）
- **受け入れ基準**: 各キーが、対応する CLI 指定と同じ値を `args` に与えることをユニットテストで確認できる。`seed_base: abc` のような変換できない値は、キー名を含むエラーになる
- **優先度**: Must

### FR-003: 優先順位（CLI が YAML を上書き）

- **機能名**: CLI と設定YAMLの統合
- **概要**: 同じ項目が CLI と設定YAMLの両方にある場合は、CLI の明示指定を優先する
- **入力**: CLI 引数と設定YAML
- **出力**: 統合された設定値
- **詳細**:
  - 優先順位: **CLI の明示指定 > 設定YAML > 組み込みの既定値**
  - 複数値（`cameras`、`init_cameras`）は、CLI で指定した場合は CLI の値で丸ごと置き換える（結合しない）
  - フラグ（`overwrite`、`fresh_match`）の CLI からの上書きは true 方向だけ可能（CLI に false にするオプションが無いため）。設定YAMLの `true` を false に戻すには設定YAMLを編集する。この非対称性を `--help` に書く（feat-033 と同じ）
- **受け入れ基準**: YAML に `seed_base: 100`、CLI に `--seed-base 200` を与えると 200 が使われる。YAML だけなら 100、両方とも無ければ既定値。`cameras` も同様に、CLI 指定時は CLI の値だけになる
- **優先度**: Must

### FR-004: 必須項目の統合後検証

- **機能名**: 必須項目の統合後検証
- **概要**: 必須項目は、CLI か設定YAMLのどちらかで与えられていればよいとし、統合した後に欠けていないかを検証する
- **入力**: 統合された設定値
- **出力**: エラーメッセージ、または処理の続行
- **詳細**:
  - 必須項目（本案件の前の実装で `required=True` だったもの）:
    - `refine_extrinsics.py`: `toml`、`ply`、`images_dir`、`points_3d`、`points_2d`、`out_toml`、`out_report`
    - `adjust_extrinsics_multiview.py`: `toml`、`ply`、`images_dir`、`out_toml`、`out_report`
    - `estimate_intrinsics_loop.py`: `toml`、`camera`、`ply`、`images_dir`、`points_3d`、`points_2d`、`out_dir`
  - 統合した後に 1 つでも欠けていれば、欠けている項目を CLI のオプション名（例: `--toml`）で列挙したエラーメッセージを出して終了コード 2 で終了する。このため、これらの引数は `required=True` から統合後の検証に変わる。**欠けたときのエラーメッセージの文言は、argparse 標準の required エラーから「必須項目が未指定です（CLIまたは--configで指定）: …」の形に変わる**。終了コード 2 は変わらない
  - `adjust_extrinsics_multiview.py` の `--init-toml` は、本案件の前から省略可能（未指定は `main` 内でエラー）であり、その扱いを変えない
  - 既存の `main` 内の検証（出力ファイルの上書き確認、入力ファイルの存在確認、bug-006 の `--out-dir` / `--tmp-dir` の新規作成とカメラ名の制限など）は、統合後の値に対して従来どおり適用する。設定YAML由来の値も「与えられた」値として扱う
- **受け入れ基準**: 他の引数エラーが無く統合パースまで進む場合に、必須項目の 1 つを CLI にも YAML にも書かないと、その項目名を含むエラーで終了コード 2 になる。必須項目を CLI と YAML に分けて書き、両方を合わせて初めてそろう場合は、エラーにならない。`estimate_intrinsics_loop.py` で YAML に既存の `out_dir` を書いた場合、bug-006 の検査で終了コード 1 になる
- **優先度**: Must

### FR-005: 設定YAMLのエラー検出

- **機能名**: 設定YAMLの誤り検出
- **概要**: 設定YAMLの誤りを、はっきりしたエラーで検出する
- **入力**: 設定YAML
- **出力**: エラーメッセージ（終了コード 2）
- **詳細**: 次をすべてエラーとする
  1. FR-002 の表に無いキー（タイポの検出。有効なキーの一覧を表示する）。他の対象ツールのキー（例: `estimate_intrinsics_loop.py` に `overwrite`）も未知のキーとしてエラーにする
  2. `config` キー（設定YAMLから別の設定YAMLを参照する連鎖はできない）
  3. bool キーの `true` / `false` 以外の値（大文字・混在も不可）
  4. 複数値キーの要素が 0 個
  5. 数値キー（`seed_base`、`seed`）の型変換の失敗
  6. 文字列キー・複数値キーの値に NUL 文字が含まれる
- **受け入れ基準**: 上記 6 種類の誤りが、それぞれエラーメッセージ（キー名を含む）で検出される
- **優先度**: Must

## 4. 非機能要求

- **後方互換**: `--config` を使わない既存のコマンドラインの挙動を変えない（既存テストの全件成功で検証する）。例外は次の 2 つ:
  1. FR-004 に書いた、必須項目欠落時のエラー文言。また、必須項目の欠落と、argparse が `unrecognized arguments` として拒否する引数（未知のオプション `--bogus` や余分な位置引数 `extra` など）が同時にある場合は、従来は必須項目の欠落が報告されたが、本案件の後は先に `unrecognized arguments` が報告される（argparse の 1 回目の解釈で検出されるため。終了コード 2 は変わらない。codex-12 指摘1、codex-13）
  2. argparse のオプション名の省略形のうち、`--config` と前方一致が重なるもの（例: 従来は `--camera` / `--cameras` の省略形として通っていた `--c`）は、曖昧な指定として終了コード 2 になる。`--ca` 以上の省略形と完全なオプション名は従来どおり使える。他のオプションの省略形も従来どおり使える（既存の 3 本は省略形を許可しており、それを変えない）。`--config` 自体の省略形（`--co` など）は終了コード 2 にする。どの場合も設定YAMLは読まない
- **内部呼び出しの不変**: `estimate_intrinsics_loop.py` が最終精緻化で `refine_extrinsics.py` を subprocess で呼ぶときの引数（CLI 引数で渡す）は変えない
- **パフォーマンス**: 設定YAMLの読み込みにかかる時間は無視できる（数十行のテキストの読み込み）
- **対応環境**: 本案件の前と同じ。ユニットテストはルートの uv 環境の `uv run pytest` で実行できること（既存方式）
- **セキュリティ**: 該当なし（ローカルファイルの処理のみ）

## 5. 制約条件

- **新規ライブラリを追加しない**（PyYAML は使わない。feat-029 のヒアリングで確定した方針に従う）
- 汎用部品（`load_yaml_flat`、`_yaml_bool`、`parse_config_yaml`）は `render_fps_video.py` の実装を import して再利用する。`parse_config_yaml` は feat-033 で変換表を渡せる形（`converters` 引数）になっており、変更しない
- 実装での変更対象は機能設計書 §2 のファイルに限る
- 環境操作は uv だけで行う

## 5.1 範囲外とする既存の問題

- `estimate_intrinsics_loop.py` は最終精緻化の `refine_extrinsics.py` を `cwd=ROOT`（プロジェクトルート）で起動し、相対パスの引数をそのまま渡す。そのため、プロジェクトルート以外のディレクトリから相対パスで実行すると、子プロセスで入力が見つからない、または出力先がずれる。これは本案件の前から CLI でも起きる問題で、`--config` とは関係しない。本案件では扱わず、別案件 **bug-007** で扱う（codex-01 指摘1）。それまでは、従来どおりプロジェクトルートで実行するか、すべてのパスを絶対パスで指定する（bug-007 README の運用上の回避と同じ）

## 6. 優先順位

| 要求ID | MoSCoW |
|---|---|
| FR-001〜FR-005 | Must |
| 対象ツール以外（`prepare_phone_photo.py`、`evaluate_overlay.py`、`render.py` など）への展開 | Won't（本案件の対象外） |
| ネスト・リスト構文を含む正式な YAML 対応 | Won't（フラット構造のみ） |
| 設定YAML内の相対パスを YAML ファイルの場所基準で解釈する | Won't（ヒアリング 2026-11-25。カレントディレクトリ基準） |

**MVP の範囲**: FR-001〜FR-005 のすべて（3 本とも）。

## 7. CLI 仕様（要求レベル）

```bash
# 設定YAMLだけで実行
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/refine_extrinsics.py --config run_refine.yaml

# 一部だけ CLI で上書き
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/refine_extrinsics.py --config run_refine.yaml --seed-base 6000 \
  --out-toml phase4/data/10F/refined_seed6000.toml --out-report phase4/data/10F/refined_seed6000_report.txt
```

設定YAMLの例（`run_refine.yaml`）:

```yaml
# 10F の定型設定
toml: phase4/data/10F/s25_20261124_152436_init.toml
ply: phase4/data/10F/point_cloud.ply
images_dir: phase4/data/10F/s25_images
points_3d: phase0/data/kijunten_locations_10F.csv
points_2d: phase4/data/10F/points_2d_s25_20261124_152436.csv
out_toml: phase4/data/10F/refined.toml
out_report: phase4/data/10F/refined_report.txt
cameras: s25_20261124_152436
seed_base: 5100
```

上の 2 つ目のコマンドでは、YAML の `seed_base: 5100` を CLI の `--seed-base 6000` が上書きし、6000 が使われる。出力先（`--out-toml`、`--out-report`）も CLI で別のパスに上書きしている（1 つ目の出力が残っていると、上書きの確認で終了コード 1 になるため）。

## 8. 変更履歴

- 2026-11-25: 初版作成

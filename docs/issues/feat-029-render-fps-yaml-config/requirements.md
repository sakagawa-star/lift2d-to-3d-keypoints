# feat-029 要求仕様書: render_fps_video.py の YAML 設定ファイル読み込み

作成日: 2026-08-09
基準: `docs/REQUIREMENTS_STANDARD.md`
入力資料: 2026-08-09 ヒアリング（方式=YAML、優先順位=CLI が YAML を上書き、パーサー=簡易フラットパーサー〔新規ライブラリなし〕）

---

## 1.1 プロジェクト概要

- **何を作るのか**: `phase4/render_fps_video.py` に `--config <YAMLパス>` オプションを追加し、CLI オプションの内容を YAML ファイルから読み込めるようにする
- **なぜ作るのか**: 同スクリプトは必須引数が多く（PLY・NPZ・`--toml`・`--camera`・`--fps`）、毎回長いコマンドを打つ必要がある。定型部分をファイル化して入力を短くする
- **誰が使うのか**: 本プロジェクトの研究者
- **どこで使うのか**: ローカル機および A100 サーバー（feat-027 と同一環境）

## 1.2 用語定義

| 用語 | 定義 |
|---|---|
| 設定YAML | `--config` で渡すフラット構造（`key: value` のみ、ネスト・リスト構文なし）の YAML ファイル。`#` 始まりの行と空行は無視する |
| 簡易フラットパーサー | phase0 の `load_yaml_simple`（`phase0/common.py`）と同仕様のパーサー。行を最初の `:` で分割し、キーと値を文字列として読む |
| 設定キー | 設定YAMLに書けるキー。CLI オプション名のハイフンをアンダースコアに置換した名前（§FR-002 の表で全列挙） |
| 明示指定 | コマンドラインでオプション・位置引数を実際に与えること（デフォルト値の採用は含まない） |

## 1.3 機能要求一覧

### FR-001: `--config` オプションと設定YAMLの読み込み

- **概要**: `--config <パス>` で設定YAMLを読み込み、CLI オプションの値として使えるようにする
- **入力**: 設定YAMLのパス
- **出力**: 読み込まれた設定値（以降の処理は feat-027 と同一）
- **詳細**:
  - 設定YAMLは簡易フラットパーサーで読む（新規ライブラリは追加しない）
  - `--config` を指定しない場合、動作は feat-027 実装と完全に同一とする（後方互換）
  - 指定パスが存在しない・読めない場合はエラーメッセージ（パス表示）を出して終了コード2で終了する
- **受け入れ基準**: 全必須項目を設定YAMLに書き、`render_fps_video.py --config run.yaml` だけでポーズダンプモードが実行できる。`--config` なしの従来コマンドの挙動が変わらない（既存テスト全件成功）

### FR-002: 設定キーと型変換・検証

- **概要**: 設定キーは以下に限定し、値には CLI と同一の型変換・検証を適用する
- **入力**: 簡易パーサーが返す文字列値
- **出力**: 型変換済みの設定値
- **詳細**: 設定キーの全列挙（これ以外のキーはエラー。§FR-005）:

| 設定キー | 対応するCLI | 型・書式 |
|---|---|---|
| `ply_path` | 位置引数1 | 文字列（パス） |
| `npz_path` | 位置引数2 | 文字列（パス） |
| `toml` | `--toml` | 文字列（パス） |
| `camera` | `--camera` | 文字列 |
| `fps` | `--fps` | float、> 0 |
| `output` | `--output` | 文字列（パス） |
| `gpu` | `--gpu` | int、>= 0 |
| `chunk_size` | `--chunk-size` | int、>= 1 |
| `crf` | `--crf` | int、0〜51 |
| `preset` | `--preset` | 文字列 |
| `overwrite` | `--overwrite` | `true` または `false`（小文字のみ） |
| `keep_chunks` | `--keep-chunks` | `true` または `false`（小文字のみ） |
| `still_range` | `--still-range` | スペース区切りの整数2個（例: `still_range: 145599 145650`） |
| `still_dir` | `--still-dir` | 文字列（パス） |
| `dump_poses` | `--dump-poses` | 文字列（パス） |

  - 数値系（fps, gpu, chunk_size, crf, still_range）には CLI と同じ検証関数を適用し、違反時は設定キー名を含むエラーメッセージで終了コード2とする
- **受け入れ基準**: 各キーが対応する CLI 指定と同じ効果を持つ。`crf: 99` のような範囲外値がエラーになる

### FR-003: 優先順位（CLI が YAML を上書き）

- **概要**: 同じ項目が CLI と設定YAMLの両方にある場合、CLI の明示指定を優先する
- **入力**: CLI 引数と設定YAML
- **出力**: 統合された設定値
- **詳細**:
  - 優先順位: **CLI 明示指定 > 設定YAML > 組み込みデフォルト**（例: `--config run.yaml --output another.mp4` は出力先だけ差し替え）
  - 位置引数はコマンドラインに書いた個数分だけ先頭から上書きする（1個だけ書いた場合は `ply_path` として解釈される。argparse の位置引数の仕様どおり）
  - フラグ（`--overwrite` / `--keep-chunks`）の CLI からの上書きは true 方向のみ可能（CLI に「falseにするオプション」が存在しないため）。設定YAMLの `true` を false に戻すには設定YAMLを編集する。この非対称性を `--help` に明記する
- **受け入れ基準**: YAML に `crf: 18`、CLI に `--crf 23` を与えると 23 が使われる。YAML のみなら 18、両方なしなら組み込みデフォルト 18

### FR-004: 必須項目の統合後検証

- **概要**: 必須項目（`ply_path`, `npz_path`, `toml`, `camera`, `fps`）は「CLI か設定YAMLのどちらかで与えられていればよい」とし、統合後に欠落を検証する
- **入力**: 統合された設定値
- **出力**: エラーメッセージまたは処理続行
- **詳細**:
  - 統合後にいずれかが欠けていれば、欠けている項目名を列挙したエラーメッセージを出して終了コード2で終了する
  - 排他制約（`--still-range` と `--dump-poses` の同時指定不可、排他モード時の MP4 関連引数禁止）およびそれ以降の検証（feat-027 design §4.11 の手順2以降）は、統合後の値に対して従来どおり適用する
- **受け入れ基準**: fps を CLI にも YAML にも書かない場合にエラーになる。YAML で `dump_poses` と `still_range` を両方書いた場合に排他エラーになる

### FR-005: 設定YAMLのエラー検出

- **概要**: 設定YAMLの誤りを明確なエラーで検出する
- **入力**: 設定YAML
- **出力**: エラーメッセージ（終了コード2）
- **詳細**: 以下をすべてエラーとする
  1. FR-002 の表にないキー（タイポ検出。有効なキー一覧を表示する）
  2. `config` キー（設定YAMLから別の設定YAMLを参照する連鎖は不可）
  3. bool キーの `true` / `false` 以外の値
  4. `still_range` の要素数が2でない・整数に変換できない
  5. 数値キーの型変換失敗・範囲違反（FR-002）
- **受け入れ基準**: 上記5種の誤りがそれぞれエラーメッセージ（キー名を含む）で検出される

## 1.4 非機能要求

| ID | 項目 | 要求 |
|---|---|---|
| NFR-001 | 後方互換 | `--config` を使わない既存のコマンドライン・再開マニフェストの挙動を一切変えない（既存テスト全件成功で検証） |
| NFR-002 | 対応環境 | feat-027 と同一（追加の環境要件なし） |

## 1.5 制約条件

- **新規ライブラリを追加しない**（PyYAML 不採用。2026-08-09 ヒアリング確定）。簡易フラットパーサーは `render_fps_video.py` 内に実装する（phase4 は独立 uv 環境のため phase0/common.py は import しない。重複実装はプロジェクトの既存慣例）
- 変更対象は `phase4/render_fps_video.py` と `tests/test_feat027_render_fps_video.py` への追加テスト（または新規テストファイル）のみ。他の既存スクリプトは変更しない
- 環境操作は uv のみ

## 1.6 優先順位

| 要求ID | MoSCoW |
|---|---|
| FR-001〜FR-005, NFR-001〜002 | Must |
| 他スクリプト（render.py / render_keypoints.py 等）への展開 | Won't（本案件のスコープ外） |
| ネスト・リスト構文を含む正式YAML対応 | Won't（フラット構造のみ） |

**MVP の範囲**: FR-001〜FR-004（設定YAMLだけで実行できる + CLI 上書き）。FR-005 のエラー網羅はその上に積む。

## CLI 仕様（要求レベル）

```bash
# 設定YAMLのみで実行
uv run --project phase4 python render_fps_video.py --config run_fps.yaml

# 一部だけCLIで上書き
uv run --project phase4 python render_fps_video.py --config run_fps.yaml --output another.mp4
```

設定YAMLの例（`run_fps.yaml`）:

```yaml
# session001 用の定型設定
ply_path: /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply
npz_path: phase4/data/session001_f145749_world300.npz
toml: phase4/data/Calib_FPSCamera.toml
camera: FPSCamera
fps: 30
output: phase4/data/test_fps.mp4
```

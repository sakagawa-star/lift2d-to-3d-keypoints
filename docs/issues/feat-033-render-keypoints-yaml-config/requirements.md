# feat-033 要求仕様書: render_keypoints.py の YAML 設定ファイル読み込み

作成日: 2026-08-13
基準: `docs/REQUIREMENTS_STANDARD.md`
入力資料: feat-029 要求仕様書（方式=YAML、優先順位=CLI が YAML を上書き、パーサー=簡易フラットパーサー〔新規ライブラリなし〕は 2026-08-09 ヒアリングで確定済み。本案件はその方式を `render_keypoints.py` に展開する）

## 1. プロジェクト概要

- **何を作るのか**: `phase4/render_keypoints.py` に `--config <YAMLパス>` オプションを追加し、CLI オプション（位置引数を含む）の内容を YAML ファイルから読み込めるようにする
- **なぜ作るのか**: 同スクリプトは位置引数3つ（PLY・TOML・キーポイント）と `--camera` が毎回必要でコマンドが長い。定型部分をファイル化して入力を短くする（feat-029 と同じ動機）
- **誰が使うのか**: 本プロジェクトの研究者
- **どこで使うのか**: phase4 venv（torch/gsplat/CUDA 必須）。開発・動作確認はローカル機（gtune2、RTX 5060 Ti）で行う。本書および設計書のコマンド例に付す `TORCH_CUDA_ARCH_LIST="9.0+PTX"` はローカル機向けの値であり、他環境（A100 サーバー: sm_80）ではその環境に応じた値を使う（環境変数の要否・値は本案件のスコープ外。`docs/TECH_STACK.md` 参照）

## 2. 用語定義

feat-029 と同一の用語を使う。機能設計書・コード内でも同じ用語を使う。

| 用語 | 定義 |
|---|---|
| 設定YAML | `--config` で渡すフラット構造（`key: value` のみ、ネスト・リスト構文なし）の YAML ファイル。`#` 始まりの行と空行は無視する |
| 簡易フラットパーサー | `render_fps_video.py` の `load_yaml_flat`（feat-029 実装）と同仕様のパーサー。行を最初の `:` で分割し、キーと値を文字列として読む |
| 設定キー | 設定YAMLに書けるキー。CLI オプション名のハイフンをアンダースコアに置換した名前（FR-002 の表で全列挙） |
| 明示指定 | コマンドラインでオプション・位置引数を実際に与えること（デフォルト値の採用は含まない） |

## 3. 機能要求一覧

### FR-001: `--config` オプションと設定YAMLの読み込み

- **機能名**: 設定YAMLの読み込み
- **概要**: `--config <パス>` で設定YAMLを読み込み、CLI オプションの値として使えるようにする
- **入力**: 設定YAMLのパス
- **出力**: 読み込まれた設定値（以降の処理は feat-032 時点の実装と同一）
- **詳細**:
  - 設定YAMLは簡易フラットパーサーで読む（新規ライブラリは追加しない）
  - `--config` を指定しない場合、動作は feat-032 時点の実装と完全に同一とする（後方互換。ただし FR-004 のとおり、必須項目欠落時のエラーメッセージ文言のみ変わる）
  - 指定パスが存在しない・読めない場合はエラーメッセージ（パス表示）を出して終了コード 2 で終了する
- **受け入れ基準**: 動画モードの全必須項目を設定YAMLに書き、`render_keypoints.py --config run.yaml` だけで実行できる（レンダリング部はスタブ化したユニットテストで確認）。`--config` なしの既存テストが全件パスする
- **優先度**: Must

### FR-002: 設定キーと型変換・検証

- **機能名**: 設定キーの型変換・検証
- **概要**: 設定キーは以下に限定し、値には CLI と同一の型変換を適用する
- **入力**: 簡易パーサーが返す文字列値
- **出力**: 型変換済みの設定値
- **詳細**: 設定キーの全列挙（これ以外のキーはエラー。FR-005）:

| 設定キー | 対応するCLI | 型・書式 |
|---|---|---|
| `ply_path` | 位置引数1 | 文字列（パス） |
| `toml_path` | 位置引数2 | 文字列（パス） |
| `keypoints_path` | 位置引数3 | 文字列（パス） |
| `camera` | `--camera` | 文字列 |
| `near_plane` | `--near-plane` | float |
| `output_dir` | `--output-dir` | 文字列（パス） |
| `background` | `--background` | スペース区切りの float 3個（例: `background: 0 0 0`） |
| `no_occlusion` | `--no-occlusion` | `true` または `false`（小文字のみ） |
| `occlusion_margin` | `--occlusion-margin` | float |
| `start_frame` | `--start-frame` | int |
| `end_frame` | `--end-frame` | int |
| `mp4` | `--mp4` | `true` または `false`（小文字のみ） |
| `mp4_fps` | `--mp4-fps` | float |
| `no_png` | `--no-png` | `true` または `false`（小文字のみ） |
| `no_keypoints` | `--no-keypoints` | `true` または `false`（小文字のみ） |
| `distort` | `--distort` | `true` または `false`（小文字のみ） |

  - 型変換失敗時は設定キー名を含むエラーメッセージで終了コード 2 とする
  - CLI 側に値域検証が無い項目（near_plane 等）には YAML 側でも値域検証を追加しない（CLI と同一の変換のみ）
- **受け入れ基準**: 各キーが対応する CLI 指定と同じ効果を持つことをユニットテストで確認できる。`near_plane: abc` のような変換不能値がキー名を含むエラーになる
- **優先度**: Must

### FR-003: 優先順位（CLI が YAML を上書き）

- **機能名**: CLI と設定YAMLの統合
- **概要**: 同じ項目が CLI と設定YAMLの両方にある場合、CLI の明示指定を優先する
- **入力**: CLI 引数と設定YAML
- **出力**: 統合された設定値
- **詳細**:
  - 優先順位: **CLI 明示指定 > 設定YAML > 組み込みデフォルト**
  - 位置引数はコマンドラインに書いた個数分だけ先頭から上書きする（1個だけ書いた場合は `ply_path` として解釈される。argparse の位置引数の仕様どおり。feat-029 と同一）
  - フラグ（`no_occlusion` / `mp4` / `no_png` / `no_keypoints` / `distort`）の CLI からの上書きは true 方向のみ可能（CLI に「false にするオプション」が存在しないため）。設定YAMLの `true` を false に戻すには設定YAMLを編集する。この非対称性を `--help` に明記する
- **受け入れ基準**: YAML に `near_plane: 0.3`、CLI に `--near-plane 0.5` を与えると 0.5 が使われる。YAML のみなら 0.3、両方なしなら組み込みデフォルト 0.1
- **優先度**: Must

### FR-004: 必須項目の統合後検証

- **機能名**: 必須項目の統合後検証
- **概要**: 必須項目（`ply_path`, `toml_path`, `camera`）は「CLI か設定YAMLのどちらかで与えられていればよい」とし、統合後に欠落を検証する
- **入力**: 統合された設定値
- **出力**: エラーメッセージまたは処理続行
- **詳細**:
  - 統合後にいずれかが欠けていれば、欠けている項目名を列挙したエラーメッセージを出して終了コード 2 で終了する（このため `ply_path` / `toml_path` は必須位置引数から省略可能な位置引数に、`--camera` は required=True から統合後検証に変わる。**欠落時のエラーメッセージ文言は argparse 標準の required エラーから「必須項目が未指定です」形式に変わる**が、終了コード 2 は維持する）
  - `keypoints_path` は必須項目に含めない（`--no-keypoints` との組み合わせで要否が変わるため、既存の組み合わせ検証に従う）
  - 既存の組み合わせ検証（`--no-keypoints` 時の禁止オプション、`keypoints_path` の要否、`--distort` の制約、`--no-png` の `--mp4` 併用必須）は、統合後の値に対して従来どおり適用する（設定YAML由来の値も「与えられた」として扱う）
- **受け入れ基準**: camera を CLI にも YAML にも書かない場合に「camera」を含むエラーで終了コード 2 になる。YAML で `no_keypoints: true` と `keypoints_path` を両方書いた場合に既存の組み合わせエラーになる
- **優先度**: Must

### FR-005: 設定YAMLのエラー検出

- **機能名**: 設定YAMLの誤り検出
- **概要**: 設定YAMLの誤りを明確なエラーで検出する
- **入力**: 設定YAML
- **出力**: エラーメッセージ（終了コード 2）
- **詳細**: 以下をすべてエラーとする
  1. FR-002 の表にないキー（タイポ検出。有効なキー一覧を表示する）
  2. `config` キー（設定YAMLから別の設定YAMLを参照する連鎖は不可）
  3. bool キーの `true` / `false` 以外の値（大文字・混在も不可）
  4. `background` の要素数が3でない・float に変換できない
  5. 数値キー（near_plane, occlusion_margin, start_frame, end_frame, mp4_fps）の型変換失敗
- **受け入れ基準**: 上記5種の誤りがそれぞれエラーメッセージ（キー名を含む）で検出される
- **優先度**: Must

## 4. 非機能要求

- **後方互換**: `--config` を使わない既存のコマンドラインの挙動を変えない（既存テスト全件成功で検証。唯一の例外は FR-004 に明記した必須項目欠落時のエラー文言）
- **パフォーマンス**: 設定YAML読み込みの追加時間は無視できる（数十行のテキスト読み込み）。レンダリング部の性能は変更しない
- **対応環境**: feat-032 と同一（追加の環境要件なし）。ユニットテストは root venv の `uv run pytest` で実行できること（既存方式）
- **セキュリティ**: 該当なし（ローカルファイル処理のみ）

## 5. 制約条件

- **新規ライブラリを追加しない**（PyYAML 不採用。feat-029 ヒアリングで確定済みの方針を踏襲）
- 汎用部品（`load_yaml_flat`, `_yaml_bool`, `parse_config_yaml`）は `render_fps_video.py` の feat-029 実装を import して再利用する（同一パッケージ内・トップレベル import は軽量〔torch 非依存〕であることを確認済み。重複実装を追加しない）
- 実装（ステップ6）での変更対象は `phase4/render_keypoints.py`、`phase4/render_fps_video.py`（`parse_config_yaml` の再利用のための最小変更のみ。挙動変更なし）、新規テストファイル `tests/test_feat033_config_yaml.py`、テスト結果 `tests/results/feat-033_test_result.txt` のみ。これに加えて、完了処理（ステップ8、Claude Code 本体が実施）で `docs/BACKLOG.md` / `docs/CHANGELOG.md` / `CLAUDE.md` / `README.md` を更新する（設計書8章と対応）
- 環境操作は uv のみ

## 6. 優先順位

| 要求ID | MoSCoW |
|---|---|
| FR-001〜FR-005 | Must |
| 他スクリプト（render.py 等）への展開 | Won't（本案件のスコープ外） |
| ネスト・リスト構文を含む正式YAML対応 | Won't（フラット構造のみ） |

**MVP の範囲**: FR-001〜FR-005 のすべて（すべて Must であり、本案件の完了条件に含める。段階リリースはしない）。

## CLI 仕様（要求レベル）

```bash
# 設定YAMLのみで実行（動画モード）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py --config run_keypoints.yaml

# 一部だけCLIで上書き
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py --config run_keypoints.yaml --mp4-fps 25
```

設定YAMLの例（`run_keypoints.yaml`）:

```yaml
# session001 用の定型設定
ply_path: /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply
toml_path: phase4/data/Blender/handai-hosp1_20251024.toml
keypoints_path: phase4/data/session001_f145749_world300_filtered.npz
camera: int_cam01_img
near_plane: 0.5
output_dir: phase4/data/keypoints_check
mp4: true
no_png: true
```

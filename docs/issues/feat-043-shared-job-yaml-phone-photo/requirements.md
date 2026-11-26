# feat-043 要求仕様書: スマートフォン静止画の前処理と推定で 1 つの設定 YAML を共用する

作成日: 2026-11-25
基準: `docs/REQUIREMENTS_STANDARD.md`
入力資料: 本案件の README.md「合意済みの方式」（2026-11-25 ユーザーと合意）、feat-042 の要求仕様書（`--config` の方式）

## 1. プロジェクト概要

- **何を作るのか**:
  - `phase4/prepare_phone_photo.py`（前処理。feat-041）に `--config <YAMLパス>` を追加する
  - 前処理と `phase4/estimate_intrinsics_loop.py`（推定。feat-041、feat-042 で `--config` 対応済み）の CLI オプション名と設定キーを統一する
  - その結果、1 つの設定 YAML を両方のツールに渡せるようにする
- **なぜ作るのか**: 現状は、前処理と推定で同じ値（カメラ名、前処理後画像のディレクトリ、初期 TOML、前処理後の 2D 点 CSV）を 2 回書く必要がある。うち 3 つは前処理の出力パスを推定の入力として書き写している。同じファイルに違う名前（前処理 `--out-toml` と推定 `--toml` など）が付いているためで、作業が煩雑で分かりにくい（ユーザー指摘 2026-11-25）
- **誰が使うのか**: 本プロジェクトの研究者
- **どこで使うのか**: phase4 の uv 環境（推定は torch / gsplat / CUDA が必要）。開発と動作確認はローカル機（gtune2、RTX 5060 Ti）

## 2. 用語定義

| 用語 | 定義 |
|---|---|
| 前処理 | `phase4/prepare_phone_photo.py` |
| 推定 | `phase4/estimate_intrinsics_loop.py` |
| 設定YAML | `--config` で渡すフラット構造（`key: value` のみ）の YAML ファイル。読み方は feat-042 と同じ（`render_fps_video.py` の `load_yaml_flat`） |
| 共用 YAML | 前処理と推定の両方に渡す 1 つの設定YAML。両ツールの設定キーを合わせて書く |
| 相手のキー | 共用 YAML に書かれた、もう一方のツールだけが使う設定キー（FR-001 の表で「無視」の欄） |

## 3. 機能要求一覧

### FR-001: 名前の統一

> **注記（feat-044、2026-11-26）**: 下の表の `--expect-model`・`--expect-focal-mm`（`expect_model`・`expect_focal_mm`）は feat-044 で削除された（3 ツールとも未知のキーとして終了コード 2）。前処理には `--photo-info`（キー `photo_info`）が必須で加わり、書き出しツール `extract_photo_info.py` が同じ YAML を共用する。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

- **機能名**: CLI オプション名・設定キーの統一
- **概要**: 同じファイル・同じ値には、CLI と設定YAML、前処理と推定の両方で同じ名前を付ける。CLI オプション名と設定キーの違いは、先頭の `--` の有無と `-` / `_` の違いだけとする
- **詳細**: 両ツールの CLI オプションと設定キーを次の表のとおりとする（`--config` 自体は設定キーにならない）

| 中身 | CLI | 設定キー | 前処理 | 推定 |
|---|---|---|---|---|
| カメラ名 | `--camera` | `camera` | 必須 | 必須 |
| 原本 JPEG | `--image` | `image` | 必須（読む） | 相手のキー |
| 原本座標の 2D 点 CSV | `--points-2d-original` | `points_2d_original` | 必須（読む） | 相手のキー |
| 期待する機種名（Exif Model） | `--expect-model` | `expect_model` | 任意 | 相手のキー |
| 期待する実焦点距離 [mm] | `--expect-focal-mm` | `expect_focal_mm` | 任意 | 相手のキー |
| 前処理後画像のディレクトリ | `--images-dir` | `images_dir` | 必須（書く） | 必須（読む） |
| 初期 TOML | `--init-toml` | `init_toml` | 必須（書く） | 必須（読む） |
| 前処理後の 2D 点 CSV | `--points-2d` | `points_2d` | 必須（書く） | 必須（読む） |
| 3D 基準点 CSV | `--points-3d` | `points_3d` | 相手のキー | 必須（読む） |
| 3DGS PLY | `--ply` | `ply` | 相手のキー | 必須（読む） |
| 推定の出力先（新規ディレクトリ） | `--out-dir` | `out_dir` | 相手のキー | 必須（書く） |
| 推定の中間ファイル置き場（新規ディレクトリ） | `--tmp-dir` | `tmp_dir` | 相手のキー | 任意 |
| 前処理の出力の上書き許可 | `--overwrite` | `overwrite` | 任意（フラグ） | 相手のキー |

  - 名前が変わるオプション（旧名は互換用に残さない。両ツールとも 2026-11-25 に作ったもので、利用者は本プロジェクトの研究者だけ）:

    | ツール | 旧 | 新 |
    |---|---|---|
    | 前処理 | `--out-toml` | `--init-toml` |
    | 前処理 | `--points-2d-out` | `--points-2d` |
    | 前処理 | `--points-2d-in` | `--points-2d-original` |
    | 推定 | `--toml`（設定キー `toml`） | `--init-toml`（設定キー `init_toml`） |

  - 推定の `out_dir`・`tmp_dir` は、推定の開始時に存在しない新しいディレクトリでなければならない（bug-006）。そのため、前処理の出力（`images_dir`、`init_toml`、`points_2d`）を `out_dir`・`tmp_dir` の配下に置いてはならない（前処理が先にそのディレクトリを作ってしまい、推定が終了コード 1 になる。codex-07 指摘2）
  - 前処理後画像のファイル名は `{images_dir}/{camera}.png` とする。これは `refine_extrinsics.py`（feat-026）がこの名前で画像を読む既存の仕様で、本案件で新たに決めるものではない。本案件では、これ以外にディレクトリとファイル名を組み合わせる暗黙の規則を追加しない
- **受け入れ基準**: 両ツールの `--help` に表の CLI オプションだけが出る（推定は表の推定の欄が「相手のキー」以外のもの、前処理は前処理の欄が「相手のキー」以外のもの）。旧名を指定すると終了コード 2 になる
- **優先度**: Must

### FR-002: 前処理の `--config`

- **機能名**: 前処理の設定YAMLの読み込み
- **概要**: 前処理に `--config <パス>` を追加し、FR-001 の設定キーを設定YAMLから読めるようにする
- **詳細**: 方式は feat-042 と同じ
  - 優先順位: CLI の明示指定 > 設定YAML > 既定値
  - 必須項目は CLI か設定YAMLのどちらかにあればよい
  - `--config` の省略形は不可
  - 不正な UTF-8 は終了コード 2
  - 文字列値の NUL 文字は拒否する
  - bool（`overwrite`）は小文字の `true` / `false` のみ。CLI からは true 方向にだけ上書きできる
  - `expect_focal_mm` は CLI と同じ `float` 変換
- **受け入れ基準**: 前処理の必須項目をすべて設定YAMLに書けば、`--config` だけで前処理が完了する（Exif の読み込みをスタブ化したユニットテストで確認）
- **優先度**: Must

### FR-003: 共用 YAML（相手のキーの無視）

- **機能名**: 共用 YAML の受け入れ
- **概要**: 前処理と推定は、それぞれ相手のキーを黙って無視する。これにより、両ツールのキーをすべて書いた 1 つの共用 YAML を、両方に渡せる
- **詳細**:
  - 相手のキーは、値の型変換・検査も行わずに無視する（例: 推定に渡した共用 YAML の `expect_focal_mm: abc` はエラーにならない。前処理に渡すとエラーになる）
  - 両ツールのどちらにも無いキーは、従来どおり未知のキーとしてエラー（終了コード 2）
  - 相手のキーは CLI では受け付けない（CLI で相手のオプションを指定すると、argparse の未知の引数として終了コード 2）。相手のオプション名が自分のオプション名の省略形と見なされないよう、両ツールとも省略形を受け付けない（§4 の後方互換の例外）
- **受け入れ基準**: FR-001 の表の全キーを書いた共用 YAML を、前処理と推定の両方に `--config` で渡せる。それぞれのツールでは、自分のキーの値だけが使われる
- **優先度**: Must

### FR-004: 前処理のカメラ名の制限

- **機能名**: 前処理のカメラ名の検査
- **概要**: 前処理の `--camera` も、推定（bug-006）と同じ条件で検査する。前処理後画像のファイル名（`{images_dir}/{camera}.png`）に使うため、パス区切りを含む名前で `images_dir` の外に書かないようにする
- **詳細**: `""`・`"."`・`".."`、パス区切り（`/`、`\`）を含む名前、`Path(camera).name != camera` の名前は、何も書かずに終了コード 1（推定と同じメッセージ）
- **受け入れ基準**: `--camera sub/cam` で終了コード 1、どの出力も作られない
- **優先度**: Must

### FR-005: 推定の名前の変更

- **機能名**: 推定の初期 TOML のオプション名の変更
- **概要**: 推定の `--toml`（設定キー `toml`）を `--init-toml`（`init_toml`）に変える。推定がそれ以外に持つ CLI オプションと設定キーは変えない
- **詳細**:
  - 推定の内部で最終精緻化の `refine_extrinsics.py` を呼ぶときの引数（`refine_extrinsics.py` 自身の `--toml` などの CLI オプション）は変えない
  - ループレポートのヘッダの入力パスの表示名は `toml` から `init_toml` に変わる
- **受け入れ基準**: `--init-toml` で従来の `--toml` と同じ処理になる（既存テストの名前を置き換えて全件成功）
- **優先度**: Must

## 4. 非機能要求

- **後方互換**: 両ツールの、FR-001 の表で名前が変わるオプション・設定キーは旧名で使えなくなる（意図的。FR-001）。それ以外の CLI オプション、終了コード、出力ファイルの内容は変えない
  - 例外: 推定のループレポートのヘッダの表示名（FR-005）
  - 例外: 前処理と推定は、オプション名の省略形をすべて受け付けない（argparse の `allow_abbrev=False`。完全な名前だけを使う）。改名により、従来の省略形が別のオプションとして解釈される危険があるため。例: 前処理では曖昧だった `--o` が `--overwrite` に、旧 `--points-2d-out` の省略形 `--points-2d-o` が `--points-2d-original` に解釈される。推定では前処理の `--image`（原本 JPEG）が `--images-dir` の省略形として受け付けられ、画像ディレクトリとして使われる。省略形は、設定YAMLを読む前に終了コード 2 になる（codex-09、実装時の発見 2026-11-25）
  - 例外: 前処理の 3 つの出力パス（`--images-dir`、`--init-toml`、`--points-2d`）に空文字を指定すると、CLI・設定YAMLのどちらでも終了コード 2 になる（従来は空の `--images-dir` で実行ディレクトリに画像を書けた。空の出力パスによる意図しない場所への書き込みと途中失敗を防ぐため。codex-07 指摘1）
  - 例外: 前処理の必須項目欠落時のエラー文言が「必須項目が未指定です（CLIまたは--configで指定）: …」に変わる。`unrecognized arguments` が同時にある場合はそちらが先に報告される（feat-042 と同じ。終了コード 2 は同じ）
- **出力の同一性**: 同じ入力で実行したとき、前処理の出力（前処理後画像、初期 TOML、前処理後の 2D 点 CSV）と推定の結果は、本案件の前と同じになる
- **他のツールへの影響なし**: `refine_extrinsics.py`・`adjust_extrinsics_multiview.py` の CLI オプション・設定キーと動作は変えない
- **パフォーマンス・対応環境・セキュリティ**: 本案件の前と同じ

## 5. 制約条件

- 新規ライブラリを追加しない
- feat-042 の共通関数（`refine_extrinsics.py` の `parse_args_with_config`・`_yaml_str` など）を再利用する。共通関数を変える場合は、`refine_extrinsics.py`・`adjust_extrinsics_multiview.py` の動作を変えない形にする
- 実装での変更対象は機能設計書 §2 のファイルに限る
- 環境操作は uv だけで行う
- bug-007（推定の最終精緻化の相対パス）は本案件の範囲外。修正までは、プロジェクトルートで実行するか、すべてのパスを絶対パスで指定する（注記: bug-007 で修正済み。子プロセスは親のカレントディレクトリを継承するため、どのディレクトリからでも相対パスで実行できる。2026-11-26）

## 6. 優先順位

| 要求ID | MoSCoW |
|---|---|
| FR-001〜FR-005 | Must |
| 前処理と推定を 1 コマンドで続けて実行する入口 | Won't（ユーザー決定 2026-11-25。実行は 2 コマンド） |
| `refine_extrinsics.py` に画像パスを直接渡すオプションを足して `{images_dir}/{camera}.png` の規則を無くす | Won't（範囲が広がるため） |
| 旧名の互換用エイリアス | Won't（FR-001） |

## 7. 使い方（要求レベル）

```bash
# 共用 YAML で前処理 → 推定（プロジェクトルートで実行）
uv run --project phase4 python phase4/prepare_phone_photo.py --config job_s25.yaml
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py --config job_s25.yaml
```

注記（feat-044、2026-11-26）: 現行版では `expect_model`・`expect_focal_mm` は削除され、前処理には `photo_info`（写真情報ファイル）が必須になった。この例のままでは終了コード 2 になる。現行の例は README の `estimate_intrinsics_loop.py ほか` の節を参照

共用 YAML の例（`job_s25.yaml`）:

```yaml
camera: s25_20261124_152436
image: phase4/data/10F/s25_original/20261124_152436.jpg
points_2d_original: phase0/data/points_2d_10F.csv
expect_model: Galaxy S25
expect_focal_mm: 5.4
images_dir: phase4/data/10F/s25_images
init_toml: phase4/data/10F/s25_20261124_152436_init.toml
points_2d: phase4/data/10F/points_2d_s25_20261124_152436.csv
points_3d: phase0/data/kijunten_locations_10F.csv
ply: phase4/data/10F/point_cloud.ply
out_dir: phase4/data/10F/s25_result
```

前処理の出力（`images_dir`・`init_toml`・`points_2d`）は、`out_dir`（と `tmp_dir`）の外に置く（FR-001）。

## 8. 変更履歴

- 2026-11-25: 初版作成
- 2026-11-25: 実装時の発見（推定が `--image` を `--images-dir` の省略形として受け付ける）により、推定も省略形を受け付けないように変更

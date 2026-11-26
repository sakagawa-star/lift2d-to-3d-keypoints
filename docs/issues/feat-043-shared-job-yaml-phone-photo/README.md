# feat-043: スマートフォン静止画の前処理と推定で 1 つの設定 YAML を共用する

## ステータス

Closed（2026-11-25 起票、2026-11-26 完了。Codex レビュー codex-01〜11（依頼文に「致命的な点のみ」の前提）で収束、人レビュー通過。実装は Sonnet サブエージェント委任、全体 742 passed / 1 skipped。手動テスト合格（共用 YAML 1 つで前処理→推定を完走、median 0.671px・合格。旧名・相手のオプションは終了コード 2））

## 概要

`phase4/prepare_phone_photo.py`（前処理。feat-041）を設定 YAML（`--config`）に対応させ、`phase4/estimate_intrinsics_loop.py`（推定。feat-041、feat-042 で `--config` 対応済み）と **1 つの設定 YAML を共用**できるようにする。同じファイル・同じ値には CLI・YAML・両ツールで同じ名前を付け、共用 YAML に各値を 1 回だけ明示的に書けば両方のツールが動くようにする。

## 背景（2026-11-25 ユーザー指摘）

- 現状は、前処理と推定で同じ値を 2 回書く必要がある。カメラ名、前処理後画像のディレクトリ、初期 TOML、前処理後の 2D 点 CSV の 4 つで、うち 3 つは前処理の出力パスを推定の入力として書き写している。「繰り返し作業が多くて煩雑。作業が洗練されていない」

  | 前処理 `prepare_phone_photo.py` | 推定 `estimate_intrinsics_loop.py` |
  |---|---|
  | `--camera` | `--camera`（同じ値） |
  | `--images-dir`（出力先） | `--images-dir`（入力。同じ値） |
  | `--out-toml`（出力） | `--toml`（入力。同じ値） |
  | `--points-2d-out`（出力） | `--points-2d`（入力。同じ値） |
  | `--image`、`--points-2d-in`、`--expect-model`、`--expect-focal-mm`、`--overwrite` | `--ply`、`--points-3d`、`--out-dir`、`--tmp-dir` |

- 重複の原因はキーの名前ではない。中間ファイルのパスを人が決めて、両方に書いていることが原因（キー名をどちらに合わせても書き写しは残る）

## 合意済みの方式（2026-11-25）

経緯: 最初に「前処理ディレクトリ `prepared_dir` の中に決まったファイル名で中間ファイルを置く」案を出したが、ユーザーに「分かりづらい。禁止」と却下された。「CLI と config は同じ名前を使え」とも指示された。そこで次の方式に改めた。

- **同じファイル・同じ値には、CLI と YAML、前処理と推定の両方で同じ名前を付ける**。共用 YAML には各パスを 1 回だけ明示的に書く（暗黙の命名規則は追加しない）
- 1 つの設定 YAML を前処理と推定の両方に `--config` で渡す。実行は従来どおり 2 コマンド
- 共用 YAML の中の相手のツール専用のキーは黙って無視する。どちらにも無いキーは未知のキーとしてエラー（feat-042 と同じ）
- 前処理の `--camera` にも、推定（bug-006）と同じ「パス区切りを含む名前の拒否」を入れる
- 旧名は互換用に残さない（どちらも 2026-11-25 に作ったツール）

名前の統一表（CLI と YAML キーの違いは `-` と `_` だけ）:

| 中身 | CLI | YAML キー | 前処理 | 推定 |
|---|---|---|---|---|
| カメラ名 | `--camera` | `camera` | 使う | 使う |
| 原本 JPEG | `--image` | `image` | 読む | 無視 |
| 原本座標の 2D 点 CSV | `--points-2d-original` | `points_2d_original` | 読む | 無視 |
| 期待する機種名・実焦点距離 | `--expect-model`、`--expect-focal-mm` | `expect_model`、`expect_focal_mm` | 使う | 無視 |
| 前処理後画像のディレクトリ | `--images-dir` | `images_dir` | 書く | 読む |
| 初期 TOML | `--init-toml` | `init_toml` | 書く | 読む |
| 前処理後の 2D 点 CSV | `--points-2d` | `points_2d` | 書く | 読む |
| 3D 基準点 CSV | `--points-3d` | `points_3d` | 無視 | 読む |
| 3DGS PLY | `--ply` | `ply` | 無視 | 読む |
| 推定の出力先（新規ディレクトリ） | `--out-dir` | `out_dir` | 無視 | 書く |
| 推定の中間ファイル置き場 | `--tmp-dir` | `tmp_dir` | 無視 | 使う |
| 前処理の出力の上書き許可 | `--overwrite` | `overwrite` | 使う | 無視 |

名前が変わるオプション: 前処理 `--out-toml` → `--init-toml`、`--points-2d-out` → `--points-2d`、`--points-2d-in` → `--points-2d-original`。推定 `--toml`（YAML `toml`）→ `--init-toml`（`init_toml`）。

既存の約束事として残すもの: 前処理後画像のファイル名は `{images_dir}/{camera}.png`。`refine_extrinsics.py`（feat-026）がこの名前で読む既存の仕様で、本案件で新たに決めるものではない（ユーザーに説明済み）。

注記（feat-044、2026-11-26）: 現行版では `expect_model`・`expect_focal_mm` は削除され、前処理には `photo_info`（写真情報ファイル）が必須になった。この例のままでは終了コード 2 になる。現行の例は README の `estimate_intrinsics_loop.py ほか` の節を参照

共用 YAML の例:

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

## 関連

- feat-041（前処理・推定の初版）、feat-042（推定の `--config`）、bug-006（出力先の新規作成）、bug-007（推定の最終精緻化の相対パス）

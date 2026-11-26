# feat-044: 前処理の Exif 依存の整理（写真情報ファイルの分離と --expect-model / --expect-focal-mm の削除）

## ステータス

Closed（2026-11-26 完了）。初版（expect 系の削除のみ）は Codex レビュー codex-01〜04 で収束したが、人レビューで範囲拡大の指示（写真情報ファイルの分離）があり requirements.md / design.md を書き直した。書き直し版は Codex レビュー codex-05〜10（全文ゲート 1 回、依頼文に「致命的な点のみ」の前提）で全指摘解消し、人レビューを通過。実装は Sonnet サブエージェント委任（想定外事象なし）。自動テスト 776 passed / 1 skipped（`tests/results/feat-044_test_result.txt`）、実データ確認は `verification.md`。手動テスト合格（2026-11-26、`phase4/data/10F/job_feat044.yaml` で書き出し→前処理→推定を完走し median 0.671px・合格、loop.toml・final.toml は feat-043 とバイト一致）

## 概要

`phase4/prepare_phone_photo.py`（前処理。feat-041）の Exif への依存を整理する。

- 初期焦点距離の計算に使う 35mm 換算焦点距離を、Exif から直接読むのをやめ、**写真情報ファイル**（`focal_length_35mm: 23` の 1 行）から読む。写真情報ファイルは新しいスクリプト `phase4/extract_photo_info.py` が Exif から書き出す。Exif の無い画像では人が書く
- 任意オプション `--expect-model` と `--expect-focal-mm`（設定キー `expect_model` / `expect_focal_mm`）と、それに伴う Exif の検査（`Model` の一致、`FocalLength` の ±0.05mm 一致）を削除する

## 背景（2026-11-26 ユーザー指摘）

- README の共用設定YAMLの例（feat-043）に載っている `expect_model: Galaxy S25` / `expect_focal_mm: 5.4` について、ユーザーから「必要なさそう」「意味のない設定項目を残す理由が分からない」と指摘された
- 2 つは任意の安全確認（別の機種・別のレンズで撮った写真の取り違えの検出）で、省略しても処理結果は変わらない
- ユーザー判断（2026-11-26）: README の例から消すだけでなく、**ツールからオプションごと削除する**
- 判断の基準（ユーザー、2026-11-26）: 「YAML ファイルは人が手動で書く。人が手で書く必要がある設定は残す。Exif から値を目で見て手で書くことは YAML に必要が無い項目。理由は間違うから。Exif の値で使う必要があるものは YAML では扱わない」
  - `expect_model` / `expect_focal_mm` は、Exif の `Model` / `FocalLength` を人が見て書き写す項目なので、この基準で削除する
  - Exif の値で処理に必要なもの（初期焦点距離の計算に使う `FocalLengthIn35mmFilm`、検査に使う `Orientation`・`DigitalZoomRatio`）は、従来どおりツールが Exif から直接読む（YAML・CLI には出さない）（このうち `FocalLengthIn35mmFilm` は、下の「範囲の拡大」で写真情報ファイルに分けた）
  - 前処理・推定の他の設定キー（カメラ名、各ファイルのパス、`overwrite`）は人が決める値で、Exif の書き写しではないため残す

## 範囲の拡大（2026-11-26 人レビューでのユーザー指示）

- 「Exif から読むべき値は、YAML ではなく別の設定ファイルに分けるべき。理由は Exif が無い画像ファイルの時に、今回のツールが使えなくなるから」
- ヒアリング結果（2026-11-26）:
  - 別ファイル（写真情報ファイル）に入れる値: 35mm 換算焦点距離だけ（向き・デジタルズームは Exif に値があるときだけ検査）
  - 作り方: Exif から自動で書き出す。Exif が無い画像のときだけ人が書く
  - 案件の扱い: feat-044 に統合
- 合意した方式: 書き出し専用の新スクリプト `phase4/extract_photo_info.py`（`--image`・`--photo-info`・`--config`。上書きはしない。出力先が既に存在すれば終了コード 1。2026-11-26 のレビュー codex-06 を受けてユーザー判断で `--overwrite` を削除。`overwrite` は前処理だけのキー）を作り、前処理に必須の `--photo-info` を足す。共用設定YAMLは書き出し・前処理・推定の 3 つで共用する

## 関連

- feat-041（前処理の初版。FR-001 の入力と検査項目）、feat-043（共用 YAML。推定の `IGNORED_KEYS` にこの 2 キーを含む）

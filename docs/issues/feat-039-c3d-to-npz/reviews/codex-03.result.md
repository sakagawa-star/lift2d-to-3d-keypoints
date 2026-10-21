# Codex レビュー結果 codex-03

- 日付: 2026-10-21
- 対象: `docs/issues/feat-039-c3d-to-npz/requirements.md`, `docs/issues/feat-039-c3d-to-npz/design.md`
- ストリーム: `rev-lift2d-feat-039`
- フェーズ: 全文ゲート（`/new` 実施後の新しい会話）
- 指摘数: 高 2 / 中 0 / 低 0
- トークン実測: total 302,200（input 299,891 うち cached 244,864 / output 2,309 うち reasoning 854）
- rollout: `~/.codex/sessions/2026/10/21/rollout-2026-10-21T09-47-10-01a15bed-2572-7c01-b2e9-ae960eefa5b5.jsonl`

## 依頼文

docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/feat-039-c3d-to-npz/requirements.md docs/issues/feat-039-c3d-to-npz/design.md 。本ドキュメントは対話ベースのレビューを経ている。今回は最終ゲートとして、文書全体の整合性（セクション間の矛盾、参照切れ、抜け）を含めて全文を確認して。

## 回答

[AGENTS.md適用]

対象2文書を全文確認し、既存コード・依存ライブラリ・参照先とも照合しました。最終ゲートで修正が必要な問題は2件です。

1. 重要度：高 — 固定の一時ファイル名により既存データを破壊し得る
   - 対象：design.md:132
   - `<出力>.tmp.npz` が既存ファイルなら無条件に上書きする。また、このパスが既存の出力ファイルへのシンボリックリンクなら、検証前に出力本体を書き換えるため、FR-005の安全性を満たさない
   - 修正案：出力と同じディレクトリに一意な一時ファイルを排他的に作成し、そのファイルだけを検証・置換・削除する設計に変更する。既存の一時パスやリンク先を変更しないテストも追加する

2. 重要度：高 — 実データ検証手順に存在しないファイルへの参照がある
   - 対象：design.md:279
   - `phase4/data/run_keypoints_tokyo1.yaml` は存在しない。PLY・TOML・カメラの取得元が参照切れとなり、記載された検証手順をそのまま実行できない
   - 修正案：実在する `phase4/data/tokyo1/run_keypoints_tokyo1.yaml` に参照を修正する。指定された3キーは、このファイルに存在する

## 対応

1. design.md 4.4 を `tempfile.mkstemp`（出力と同じディレクトリ、排他作成）＋ `os.fdopen` へのファイルオブジェクト書き込みに変更。依存関係表に標準ライブラリを追記。requirements.md FR-005 に一時ファイルの作成条件を追記。T-18・T-25・T-26 の期待値を `*.tmp.npz` が残らない形に修正し、T-27（既存の同名ファイル・シンボリックリンクを壊さない）を追加
2. YAML がユーザーにより `phase4/data/tokyo1/` へ移動されていたことを確認。移動に左右されないよう、design.md 6章の実データ確認 4 に PLY・TOML・カメラの値を絶対パスで直接記載（取得元 YAML の現在のパスも併記）

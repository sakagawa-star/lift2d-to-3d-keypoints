# codex-07 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-043（GPT-6.2-Sol medium）
- フェーズ: 全文ゲート 2 回目（`/new` で会話をクリアしてから依頼。前提条件「致命的な点のみ」）
- 指摘数: 高 1 / 中 2 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":522430,"cached_input_tokens":450176,"cache_write_input_tokens":0,"output_tokens":4280,"reasoning_output_tokens":1231,"total_tokens":526710}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T17-17-34-01a211c8-1177-7e40-ae7d-e99f790a0a73.jsonl

## 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| 1（高） | 後方互換の要件（列挙した例外以外は維持）と設計が両立しない: 空の --images-dir を CLI で明示しても終了コード 2 になる（従来は実行ディレクトリに出力できた）。推定の --i（--images-dir の省略形）が --init-toml 追加で曖昧になる | requirements.md §4 の後方互換の例外に 2 つを追加。推定では --images-dir（--im 以上）を使うよう案内。design.md §4.2 にも --i の注記 |
| 2（中） | 前処理の出力を推定の out_dir 配下に置くと、前処理が先にディレクトリを作り、推定が出力先既存で終了コード 1 になる | FR-001 と §7 の例に「前処理の出力を out_dir・tmp_dir の配下に置いてはならない」と明記 |
| 3（中） | 実データ検証の手順が既存の設定YAML（job_feat043.yaml）を上書きし得る | §6.2 の事前確認に設定YAML自体の不在確認を追加 |

# codex-13 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-042（GPT-6.2-Sol medium）
- フェーズ: 全文ゲート 3 回目の解消確認（codex-12 と同じ会話）
- 指摘数: 高 0 / 中 2（未解消 1、新規 1） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1260987,"cached_input_tokens":1151104,"cache_write_input_tokens":0,"output_tokens":8630,"reasoning_output_tokens":2284,"total_tokens":1269617}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T16-03-35-01a21184-5894-7283-9b0d-234958cc0a68.jsonl

## 解消確認

| codex-12 の指摘 | 判定 |
|---|---|
| 1 引数エラーの優先順位 | 未解消（中）: FR-001 は変更点を文言だけに限定したまま。FR-004 の受け入れ基準に先行する引数エラーの条件が無い。例外 1 の「別の引数エラー」は範囲が広い |
| 2 NUL 文字 | 解消 |
| 3 テスト 16 | 解消 |

新規（中）: テスト 10b は NUL を含む 2 つの値を別ケースにすること

## 対応

- FR-001 から後方互換の例外を参照する形に変更。例外 1 を「未知のオプション（unrecognized arguments）」に限定。FR-004 の受け入れ基準を「他の引数エラーが無く統合パースまで進む場合」に限定
- テスト 10b を (a) points_3d、(b) cameras の独立ケースに分けた

# codex-28 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート 3 回目の解消確認（同じ会話。`/new` は実施せず）
- 指摘数: 高 0 / 中 1（新規） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1676858,"cached_input_tokens":1555584,"cache_write_input_tokens":0,"output_tokens":7654,"reasoning_output_tokens":1704,"total_tokens":1684512}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-40-58-01a21138-b4ab-7df3-bc27-3521131a87e4.jsonl

## 解消確認

codex-26 指摘1〜3、codex-27 新規1・2 はすべて解消（Codex の簡易確認: SIGTERM 終了で GNU time が Exit status: 0 を出しても EXIT_CODE=143 が表示された）。

## 新規指摘

- **中 — Bash の errexit が有効だと、失敗時の EXIT_CODE が表示されない**: `コマンド; echo` は bash -e で終了コード 3 のとき echo に到達しない。exit_code=0 で初期化し、コマンドの末尾に `|| exit_code=$?`、直後に `printf 'EXIT_CODE=%s\n' "$exit_code"` で表示する形にすること

## 対応

指摘どおりの形に変更（design.md §9 手順 6、investigation.md §2.5、criteria.md）。

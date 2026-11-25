# codex-21 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 解消確認（codex-20 と同じ会話。`/new` は実施せず）
- 指摘数: 高 0 / 中 1（新規） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":2392054,"cached_input_tokens":2234112,"cache_write_input_tokens":0,"output_tokens":11835,"reasoning_output_tokens":2266,"total_tokens":2403889}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-25-55-01a2112a-eb49-7c70-a929-798e4dfa18fa.jsonl

## 判定

- codex-20 の同時実行の排他を本イテレーションの修正範囲から外す判断: **妥当**（BUGFIX_STANDARD.md §2.3）

## 新規指摘

- **中 — 採用案の説明が同時実行の制約と矛盾**: §2.3 の採用案は「新旧の混在も起きない」と無条件に書いているが、同じ節の追記では同時実行で混在し得るとしている。採用案の説明を「逐次実行では前回の出力との混在を防ぐ」に限定し、requirements.md §5 と design.md §4.2.1 に「同じ --out-dir・カメラ名で実行する場合は、前の実行が終了してから起動する」と明記する（文書の修正のみ）

## 対応

investigation.md §2.3 の採用案の説明を限定。requirements.md §5 に「同時実行」の制約を追加。design.md §4.2.1 に同じ注記を追加。

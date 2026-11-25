# codex-24 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート 2 回目の解消確認（codex-23 と同じ会話。`/new` は実施せず）
- 指摘数: 高 0 / 中 1（未解消） / 低 0（指摘1 は別案件化が妥当、起票と回避策の補足を要求）
- トークン実測（累積）: "total_token_usage":{"input_tokens":1471098,"cached_input_tokens":1328768,"cache_write_input_tokens":0,"output_tokens":6696,"reasoning_output_tokens":1159,"total_tokens":1477794}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-35-32-01a21133-bba5-74f1-815b-dbd1791f8ba5.jsonl

## 解消確認

| codex-23 の指摘 | 判定 |
|---|---|
| 1（高）一時ファイルによる入力上書き | 別案件化は妥当。起票が未確認、回避策に不足（既存の .tmp が入力を指すシンボリックリンクの場合。入力と分離した新規の出力ディレクトリを使うことを明記すべき） |
| 2（中）評価画像の書き込み失敗 | 正式判定について解消（criteria 項目 6） |
| 3（中）TOML の有無の矛盾 | 解消 |
| 4（中）m1b の根拠 | 未解消: investigation.md §2.1 の「追加測定の結果」に「歪み・主点の誤差とは合わず、局所的な面の位置ずれと合う」が残る |

## 対応

- 指摘1: bug-006（docs/issues/bug-006-intrinsics-loop-refine-tmp-input-collision/README.md）を起票し、docs/BACKLOG.md に追加。investigation.md §2.3 に案件 ID、シンボリックリンクの場合、回避策（入力と分離した新規の出力ディレクトリ）を追記
- 指摘4: investigation.md §2.1 の該当箇所を観測事実までに限定し、原因は §2.2 を参照する形に変更

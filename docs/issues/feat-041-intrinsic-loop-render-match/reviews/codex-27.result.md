# codex-27 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート 3 回目の解消確認（codex-26 と同じ会話。`/new` は実施せず）
- 指摘数: 高 1 / 中 2（未解消 1、新規 1） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1289126,"cached_input_tokens":1178880,"cache_write_input_tokens":0,"output_tokens":6123,"reasoning_output_tokens":1573,"total_tokens":1295249}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-40-58-01a21138-b4ab-7df3-bc27-3521131a87e4.jsonl

## 解消確認

- codex-26 指摘1・2: 解消（記録の仕組みについては下記の新規指摘）
- codex-26 指摘3: 未解消（中）。design.md §4.2.7 の箇条書きが「評価の統計値と判定」を無条件に再掲。精緻化と評価の所要時間も、各処理を実行した場合に記録すると要求・設計に明記すること

## 新規指摘

1. **高 — 追加したログ出力が入力ファイルを上書きし得る**: time -o と tee は固定パスに書き込む。そのパスが入力 TOML・CSV を指す既存のシンボリックリンクなら、入力が切り詰められる
2. **中 — シグナル終了を終了コード 0 として記録し得る**: GNU time -v は子プロセスが SIGTERM で終了すると「Command terminated by signal 15」と「Exit status: 0」を同時に出す。Exit status 欄だけでは異常終了を判別できない

## 対応

- 新規1・2: ログ記録用に追加した `time -o` と `tee`（固定パスへの書き込み）を取りやめた。記録の仕組みを足すたびに例外が見つかるため、最も単純な方法に戻した: コマンドの直後に `echo "EXIT_CODE=$?"` を実行し、その値を正式な終了コードとする。所要時間は /usr/bin/time -v が標準エラー出力に出す Elapsed を使う。`Command terminated by signal` の行があれば異常終了（不合格）。これらの行はコマンドの出力から実験ログに書き写す（design.md §9 手順 6、investigation.md §2.5、criteria.md）
- 未解消3: design.md §4.2.7 の箇条書きを「refine_extrinsics.py の所要時間（実行した場合）、評価の所要時間（実行した場合）」に変え、評価の統計値と判定の無条件の再掲を削除。requirements.md FR-007 の所要時間も「それぞれを実行した場合に記録」に変更

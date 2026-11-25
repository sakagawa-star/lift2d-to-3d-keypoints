# codex-19 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート指摘の解消確認（codex-18 と同じ会話。`/new` は実施せず）
- 指摘数: 高 0 / 中 2（未解消 1、新規 1） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1613065,"cached_input_tokens":1480320,"cache_write_input_tokens":0,"output_tokens":9215,"reasoning_output_tokens":1633,"total_tokens":1622280}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-25-55-01a2112a-eb49-7c70-a929-798e4dfa18fa.jsonl

## 解消確認

| codex-18 の指摘 | 判定 |
|---|---|
| 未解消6 正式実行と手動テストの担当 | 解消 |
| 新規1（高）入力 TOML の削除 | 解消（衝突検査とテスト 20b） |
| 新規2（中）削除の途中失敗 | 未解消（中）: 入力の読み込み・検証が削除後なので、入力パスの指定ミスでも前回の成果物を失う。テスト 20c は削除途中の OSError を検証しない。§4.2.7 のレポート規則に削除前の拒否・削除失敗が反映されていない |

## 新規指摘

- **中 — リンク先が存在しないシンボリックリンクが出力先の検査をすり抜ける**: Path.exists() はリンク先が無いと偽になり、種類検査と削除の対象から外れる。その後レポート等を書くと、リンクをたどって --out-dir 外にファイルを作り得る

## 対応（行き詰まり検出による方針の取り直し）

削除案から新しい指摘が 2 回続いた（codex-18、codex-19）ため、CLAUDE.md の行き詰まり検出に従い、「--overwrite 時に前回出力を削除する」案を取り下げた。代替案（削除＋検査追加／退避／--overwrite 廃止）を比較し、**estimate_intrinsics_loop.py から --overwrite を廃止**する案を採用した。出力 8 ファイルのいずれかが存在すれば（os.path.lexists。リンク先の無いシンボリックリンクを含む）処理開始前に終了コード 1 とする。再実行は新しい --out-dir か手動削除で行う。

- requirements.md NFR-004、design.md §4.2.1（CLI・擬似コード）、§4.2.7、テスト 7c の記述、テスト 20 の置き換え（--overwrite が引数エラーになること、リンク先の無いシンボリックリンクで拒否されること）、テスト 20b・20c の削除
- investigation.md §2.3 に「削除案の取り下げ」と比較表、§2.4 リスク2 を更新

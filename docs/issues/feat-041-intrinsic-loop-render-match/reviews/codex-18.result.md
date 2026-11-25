# codex-18 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md、experiments/m1_p95_tail/experiment_log.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート指摘（codex-17）の解消確認（同じ会話。`/new` は実施せず）
- 指摘数: 高 1 / 中 2（未解消 1、新規 1） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1203072,"cached_input_tokens":1085184,"cache_write_input_tokens":0,"output_tokens":7562,"reasoning_output_tokens":1301,"total_tokens":1210634}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-25-55-01a2112a-eb49-7c70-a929-798e4dfa18fa.jsonl

## 解消確認

| codex-17 の指摘 | 判定 |
|---|---|
| 1 テスト 19 | 解消 |
| 2 新旧の混在 | 解消（削除処理が新たに生む問題は下記の新規指摘） |
| 3 境界値の検証 | 解消 |
| 4 m1b の分類 | 解消 |
| 5 CONV_PX の選定理由 | 解消 |
| 6 正式実行と手動テストの担当 | 未解消（中）: investigation.md が手順 6 の実行を「手動テスト」と呼び、ステップ7 でユーザーが確認する内容と完了条件が無い |

## 新規指摘

1. **高 — `--overwrite` が入力 TOML を削除し得る**: 前回の {camera}_loop.toml を --toml に指定して同じ出力先を使うと、読み込み前に入力を削除する。NFR-004 の入力保全と矛盾。削除・書き込みの前に入出力のパス衝突を検査し、該当時はファイルに触れず終了コード 1。別表記で同じパスを指すケースも検証する
2. **中 — 旧出力の削除が途中で失敗した場合の扱いが未定義**: 出力予定パスの一つがディレクトリなら、一部を削除した後に unlink() が失敗し、前回の成果物が一部だけ残る。削除前の検査、削除失敗時の終了コード・エラー表示・残った成果物の扱いを定め、テスト 20 に途中失敗のケースを追加する

## 対応

- 新規1: design.md §4.2.1 に (a) 入出力の衝突検査（Path.resolve() で比較）を追加。requirements.md NFR-004 に追記。テスト 20b（別表記の --toml）を追加
- 新規2: (b) 種類の検査（存在する 8 パスがすべて通常ファイル）を削除前に行い、該当時は何も削除せず終了コード 1。(c) 削除中の OSError は終了コード 1、削除済み・残りの一覧を表示。テスト 20c（出力パスがディレクトリ）を追加
- 未解消6: investigation.md §2.5 で手順 6 の実行を「ステップ6 の実データ検証」と明記し、ステップ7 でユーザーが確認する項目（eval.txt、checker/blend 画像、final.toml の 3DGS 取り込み）と完了条件を追加。design.md §9・criteria.md も同じ表現にそろえた

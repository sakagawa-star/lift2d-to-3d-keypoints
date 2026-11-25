# codex-07 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-042（GPT-6.2-Sol medium）
- フェーズ: 全文ゲート（`/new` で会話をクリアしてから依頼）
- 指摘数: 高 1 / 中 4 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":694271,"cached_input_tokens":575104,"cache_write_input_tokens":0,"output_tokens":6344,"reasoning_output_tokens":2076,"total_tokens":700615}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-53-20-01a2117a-f667-73f2-b1cf-8dc46bf9f8a4.jsonl

## 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| 1（高） | 多視点調整は FR-004 の必須 5 項目だけでは main が --init-toml 未指定で終了し、FR-001「必須項目を YAML に書けば開始できる」を満たさない。13b の成功側は全キーのため検出できない | FR-001 受け入れ基準と design.md §4.6 に「adjust は必須 5 項目 + init_toml が必要」と明記。13b に最小構成（必須 5 + init_toml）のケースを追加 |
| 2（中） | FR-004 の YAML 由来の既存 out_dir の拒否をテストがカバーしていない | テスト 17（YAML の out_dir に既存ディレクトリ → 戻り値 1、中身不変、run_loop_iteration 非呼び出し）を追加 |
| 3（中） | テスト 16 はオプション名の有無しか見ず、引数値の不変性を確認できない | 期待するコマンド（feat-041 §4.2.6 の形）を組み立て、記録した引数列全体と cwd を完全一致で照合する形に変更 |
| 4（中） | テスト 12 は config=None が増えるので Namespace 全体の比較が失敗する | 比較対象を旧パーサーの dest（config 以外）に限定し、args.config is None を別に確認と明記 |
| 5（中） | requirements.md §7 の --cameras cam03 の上書き例は、例の TOML にそのカメラが無く入力エラーになる | --seed-base 6000 に変更 |

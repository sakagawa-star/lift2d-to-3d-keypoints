# codex-02 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-042（GPT-6.2-Sol medium）
- フェーズ: 反復（codex-01 の解消確認）
- 指摘数: 高 1 / 中 1（いずれも未解消の残り） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":573595,"cached_input_tokens":496128,"cache_write_input_tokens":0,"output_tokens":7356,"reasoning_output_tokens":1112,"total_tokens":580951}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-43-31-01a21171-f871-7160-8e7f-75fb0c8a56c1.jsonl

## 解消確認

| codex-01 の指摘 | 判定 |
|---|---|
| 1（高）相対パスのずれ | 解消（bug-007 への切り出しは妥当） |
| 2（高）--c の互換性 | 例外化は妥当だが設計は要修正（中）: feat-029 / feat-033 の本パーサーは allow_abbrev=False で、--conf を受け付ける今回の設計と挙動が異なる。現設計は --c camA を拒否する前に camA を設定ファイルとして開こうとする。曖昧な指定はファイル読み込み前に拒否し、テスト 11c でローダーが呼ばれないことを確認すること |
| 3（中）不正な UTF-8 | 解消 |
| 4（中）テスト 13 | 13b は解消。13a は要修正（高）: load_ply の RuntimeError は InputError に変換されレポートを書かずに終わるので、seed_base をレポートで確認する期待値が成立しない。load_ply を成功させ、validate_camera を通し、process_camera のスタブで記録して MatcherEnvError でレポート出力経路に入れること。CLI と YAML の比較では出力先を分けること |

## 対応

- 指摘2: pre_parser を allow_abbrev=False（--config の完全一致のみ）にし、本パーサーは従来どおり省略形を許可（既存 3 本の省略形を壊さないため feat-029/033 には合わせない）。--c は本パーサーの曖昧エラー（ローダーは呼ばれない）、--conf は args.config != pre_args.config で検出して終了コード 2。requirements.md（FR-001、後方互換の例外 2）、design.md §4.1、テスト 11c（ローダー非呼び出しの確認）を更新
- 指摘4: テスト 13a を、load_ply 成功・validate_camera 通過・process_camera スタブで記録して MatcherEnvError、の経路に変更。CLI と YAML で出力・中間パスを分ける

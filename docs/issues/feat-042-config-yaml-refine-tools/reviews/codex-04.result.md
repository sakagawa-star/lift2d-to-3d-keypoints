# codex-04 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-042（GPT-6.2-Sol medium）
- フェーズ: 反復（codex-03 の解消確認）
- 指摘数: 高 0 / 中 1（新規） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1214573,"cached_input_tokens":1101568,"cache_write_input_tokens":0,"output_tokens":12523,"reasoning_output_tokens":1860,"total_tokens":1227096}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-43-31-01a21171-f871-7160-8e7f-75fb0c8a56c1.jsonl

## 解消確認

- --c と --config の省略形: 解消（--conf a --config b は順序を逆にしても同じパスでも読み込み前に終了コード 2、--config a --c camA も読み込み前に拒否、--ca・--config=PATH・CLI 上書きは期待どおり。テスト 11c に再現ケースあり）

## 新規指摘

- 中: --conf a.yaml --help は最初の parse_args がヘルプを表示して終了コード 0 で終わり、省略形検査に到達しない（FR-001 と不一致）。ヘルプ時の例外を明記するか、トークン検査を最初の parse_args より前に置くこと

## 対応

トークン検査を最初の parse_args より前に移した（design.md §4.1 の擬似コードと手順の説明）。--conf a.yaml --help はヘルプ表示より前に終了コード 2。--help 単独は従来どおり終了コード 0。テスト 11c に (g) --conf {YAML} --help を追加。

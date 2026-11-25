# codex-01 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-042（GPT-6.2-Sol medium）
- フェーズ: 反復（初回）
- 指摘数: 高 2 / 中 2 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":262333,"cached_input_tokens":210688,"cache_write_input_tokens":0,"output_tokens":3892,"reasoning_output_tokens":597,"total_tokens":266225}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-43-31-01a21171-f871-7160-8e7f-75fb0c8a56c1.jsonl

## 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| 1（高） | estimate_intrinsics_loop.py の最終精緻化は cwd=ROOT で子プロセスを起動し相対パスをそのまま渡すため、リポジトリ外から相対パスで実行すると子でパスがずれる。子に渡すパスを絶対化すべき | 本案件の前から CLI でも起きる問題で --config とは関係しないため範囲外とし、bug-007 として起票（docs/issues/bug-007-intrinsics-loop-refine-relative-paths/、BACKLOG）。requirements.md §5.1 に記載 |
| 2（高） | --config 追加で従来の --c（--camera / --cameras の省略形）が使えなくなり、後方互換要件に反する | feat-029 / feat-033 と同じ挙動として、後方互換の例外 2 に明記（requirements.md 非機能要求）。pre_parser は既定（省略形許可）のままにして、--conf 等の省略形で pre_parser と本パーサーの解釈を一致させる理由を design.md §4.1 に記載。テスト 11c を追加 |
| 3（中） | UTF-8 として読めない設定YAMLで UnicodeDecodeError が伝播し終了コード 2 にならない | UTF-8 を入力仕様に明記（FR-001）。parse_args_with_config で UnicodeDecodeError を捕まえて parser.error（design.md §4.1・§4.5）。テスト 11b を追加 |
| 4（中） | テスト 13 は設定YAMLだけで処理を開始できることを検証できない（adjust は --init-toml 未指定エラーで両経路が一致し得る） | テスト 13 を 13a（refine: TOML・CSV・PLY の読み込みまで進み、渡されたパスとレポートを確認）と 13b（adjust: build_camera_dict まで進み、toml・init_toml・init_cameras を確認。init_toml なしは別ケース）に置き換え |

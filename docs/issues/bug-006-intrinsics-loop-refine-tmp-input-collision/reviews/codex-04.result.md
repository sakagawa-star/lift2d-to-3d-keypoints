# codex-04 レビュー結果

- 日付: 2026-11-25
- 対象: bug-006 investigation.md、feat-041 requirements.md・design.md
- ストリーム: rev-lift2d-bug-006（GPT-6.1-Sol medium）
- フェーズ: 全文ゲートの解消確認（codex-03 と同じ会話）
- 指摘数: 高 1 / 中 1（新規） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1031312,"cached_input_tokens":929920,"cache_write_input_tokens":0,"output_tokens":5678,"reasoning_output_tokens":2343,"total_tokens":1036990}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-18-51-01a2115b-6298-73c3-bce9-3161385857cf.jsonl

## 解消確認

- codex-03（親シンボリックリンク）: 解消（mkdir(exist_ok=False) が拒否する。テスト 25 も対応）

## 新規指摘

1. **高 — カメラ名によって出力が新規ディレクトリの外へ出る**: _out_paths はカメラ名をそのまま連結する。--camera /work/victim ではレポートが /work/victim_loop_report.txt になり、TOML 読み込みエラーでもレポートを書くので、GPU 処理前に入力を上書きし得る。従来の 8 パス検査では拒否されていた。カメラ名を単一のファイル名成分に制限する
2. **中 — 中間ディレクトリ作成失敗時に出力先が空とは限らない**: --out-dir /work/new --tmp-dir /work/new/sub/.. では sub を作った後に FileExistsError となり、後始末の rmdir も失敗して予定のエラー表示・return 1 に到達しない

## 対応

1. カメラ名の制限（""・"."・".."、Path(camera).name != camera、"/" や "\\" を含む → 終了コード 1、--out-dir 作成前）を追加。requirements.md NFR-004、design.md §4.2.1、テスト 26
2. 後始末の rmdir の OSError を捕まえ、元のエラーを表示して終了コード 1。「作成したディレクトリが残っています」も表示。design.md §4.2.1、テスト 27

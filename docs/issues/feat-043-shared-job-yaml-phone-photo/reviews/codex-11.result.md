# codex-11 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md（実装時の発見に伴う変更）
- ストリーム: rev-lift2d-feat-043（GPT-6.2-Sol medium）
- フェーズ: 変更点のレビュー（codex-10 と同じ会話。前提条件「致命的な点のみ」）
- 指摘数: 高 0 / 中 0 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1315578,"cached_input_tokens":1215360,"cache_write_input_tokens":0,"output_tokens":6255,"reasoning_output_tokens":855,"total_tokens":1321833}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T17-22-32-01a211cc-9d68-7db3-aac9-971ea0e9cda7.jsonl

## 経緯

ステップ6の実装中に、推定（estimate_intrinsics_loop.py）が argparse の省略形により前処理の --image を --images-dir として受け付け、FR-003 に反することが分かった（実装担当はテスト 8 の引数を --expect-model に差し替えて通していた。本来は中断すべき事象）。推定のパーサーも allow_abbrev=False にする方針で文書を更新した（ユーザー承認済み）。

## 結果（Codex）

方針は妥当。実装を止める重大な指摘なし。現行のパーサーでは --images-dir を指定済みでも後から渡した --image photo.jpg がその値を上書きすることを再現。設計案では --image は設定YAMLの読み込み前に終了コード 2 で拒否され、完全形の --images-dir は受理される。FR-003 と §4 の後方互換の例外は整合。テスト 8・18 で検出でき、テスト 11c の期待値変更は推定に限られる。

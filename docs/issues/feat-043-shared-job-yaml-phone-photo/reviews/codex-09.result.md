# codex-09 レビュー結果

- 日付: 2026-11-25
- 対象: requirements.md、design.md
- ストリーム: rev-lift2d-feat-043（GPT-6.2-Sol medium）
- フェーズ: 全文ゲート 3 回目（`/new` で会話をクリアしてから依頼。前提条件「致命的な点のみ」。ユーザー判断で継続）
- 指摘数: 高 1 / 中 0 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":704053,"cached_input_tokens":624768,"cache_write_input_tokens":0,"output_tokens":3909,"reasoning_output_tokens":792,"total_tokens":707962}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T17-22-32-01a211cc-9d68-7db3-aac9-971ea0e9cda7.jsonl

## 指摘

- 高: 前処理のオプション改名により、省略形が別の意味で受理され、既存の成果物を上書きし得る。現行で曖昧（終了コード 2）の --o が改名後は --overwrite として受理される。旧 --points-2d-out の省略形 --points-2d-o が改名後は --points-2d-original と解釈され、出力先のつもりの CSV が入力に使われる。テスト 7 は完全形だけを対象にしており検出できない

## 対応

前処理のパーサーを allow_abbrev=False にし、オプション名の省略形をすべて受け付けないようにした（個別の省略形を拒否する方式より単純で、改名による解釈の変化を根本から防げる）。requirements.md §4 の後方互換の例外を「前処理は省略形をすべて受け付けない」に置き換え。design.md §4.2 に記載。テスト 7 に (f) --o、(g) --points-2d-o、(h) --cam を追加（既存出力が変わらないことも確認）。

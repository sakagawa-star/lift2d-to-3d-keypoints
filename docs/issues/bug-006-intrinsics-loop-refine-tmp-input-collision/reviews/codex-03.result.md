# codex-03 レビュー結果

- 日付: 2026-11-25
- 対象: bug-006 investigation.md、feat-041 requirements.md・design.md
- ストリーム: rev-lift2d-bug-006（GPT-6.1-Sol medium）
- フェーズ: 全文ゲート（`/new` で会話をクリアしてから依頼）
- 指摘数: 高 1 / 中 0 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":486619,"cached_input_tokens":406528,"cache_write_input_tokens":0,"output_tokens":3419,"reasoning_output_tokens":1721,"total_tokens":490038}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-18-51-01a2115b-6298-73c3-bce9-3161385857cf.jsonl

## 指摘

### 1. 高 — 親シンボリックリンクを経由すると、修正後も既存の中間ディレクトリを再利用でき、入力の上書きが残る

- 配置: /work/new 未作成、/work/link -> new/../existing、/work/existing/sub/iter00_render.png -> 入力画像。--out-dir /work/new --tmp-dir /work/link/sub
- 内容: 引数に .. は無い。検査時は new が無いので lexists("/work/link/sub") は偽で検査を通過。出力先の作成で new ができるとリンクが解決可能になり、mkdir(exist_ok=True) が既存の existing/sub を再利用し、入力画像が上書きされる。別プロセスとの競合は不要
- 修正案: ディレクトリ作成前に親成分の解決不能なシンボリックリンクを拒否し、中間ディレクトリは exist_ok=False で新規作成する。回帰テストを追加する

## 対応（行き詰まり検出による方針の取り直し）

「書き込む前にパスを検査する」方式から抜け道が 2 回続いた（codex-01 の ..、codex-03 の親シンボリックリンク）ため、CLAUDE.md の行き詰まり検出に従い、この方式を取り下げた。--out-dir と --tmp-dir を mkdir(parents=True, exist_ok=False) で新しく作る方式に切り替えた。存在の判定を OS が作成時に行うので、パスの書き方や途中のリンク解決によらない。新しく作ったディレクトリの中身はこのプロセスが書いたものだけになる。--tmp-dir の作成に失敗したときは、直前に作った空の --out-dir を rmdir で消す。.. の拒否と出力 8 パスの個別検査は不要になった。investigation.md §1.2〜1.5、feat-041 requirements.md NFR-004、design.md §4.2.1・§4.2.7・§5・§8（テスト 23〜25 を置き換え。25 は本指摘の配置）・§9 を更新。

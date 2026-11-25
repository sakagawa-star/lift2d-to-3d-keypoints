# codex-01 レビュー結果

- 日付: 2026-11-25
- 対象: bug-006 investigation.md、feat-041 requirements.md（NFR-004）・design.md（§4.2.1、§4.2.7、§5、§8、§9）
- ストリーム: rev-lift2d-bug-006（GPT-6.1-Sol medium）
- フェーズ: 反復（初回）
- 指摘数: 高 1 / 中 0 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":222616,"cached_input_tokens":181248,"cache_write_input_tokens":0,"output_tokens":1897,"reasoning_output_tokens":494,"total_tokens":224513}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T15-16-29-01a21159-38a9-7f90-ab41-62edc16e4eca.jsonl

## 指摘

### 1. 高 — 未作成の親と .. を含むパスで事前検査を回避でき、入力破壊が残る

- 対象: investigation.md、design.md §4.2.1
- 内容: --tmp-dir /work/new/../existing（new 未作成、existing 既存）では os.path.lexists が False になり、その後の mkdir(parents=True, exist_ok=True) が new を作って既存の existing を再利用する。--out-dir でも同じ指定で 9 パスの検査を回避できる。別プロセスとの競合は不要
- 修正案: --out-dir と --tmp-dir の両方で、.. 成分を含む指定を拒否するなど、検査から書き込みまで参照先が変わらない規則を定義し、終了コード 1・ディレクトリ作成も入力変更も起きない回帰テストを追加する

## 対応

--out-dir・--tmp-dir のパス成分に .. を含む場合は、出力済み検査より先に終了コード 1 で拒否する規則を追加（investigation.md §1.3 方針3・修正コード、requirements.md NFR-004、design.md §4.2.1・CLI）。正規化（abspath）で検査する案は、途中の成分が既存シンボリックリンクだと OS の解決と食い違うため不採用と明記。テスト 25 を追加。

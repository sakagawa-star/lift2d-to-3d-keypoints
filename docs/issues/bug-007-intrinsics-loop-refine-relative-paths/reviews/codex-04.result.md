# codex-04（bug-007 修正計画 全文ゲート 1 回目）

- 日付: 2026-11-26
- 対象: investigation.md、feat-041 design.md §4.2.6、feat-042 design.md §6 テスト16（関連: bug-007 README、estimate_intrinsics_loop.py、refine_extrinsics.py、テスト）
- ストリーム: rev-lift2d-bug-007
- フェーズ: 全文ゲート（`/new` で会話をクリアしてから依頼。依頼文に「致命的な点のみ指摘」の前提）
- 指摘数: 高 2 / 中 0 / 低 0
- トークン: total_tokens 523,036（input 518,916 うち cached 445,312、output 4,120、reasoning 727）
- rollout: ~/.codex/sessions/2026/11/26/rollout-2026-11-26T08-47-00-01a2151a-fe38-7dc3-bd58-7651454fa1a8.jsonl

## 総評

cwd を外す修正方針は feat-041 設計書 §4.2.6 と feat-042 設計書 §6 テスト16で一致。差し戻し理由は自動テストの合格条件と手動テストの実行手順。

## 指摘

### 指摘1（高）テスト28は修正後も記載どおりの合格条件を満たせない

`--out-dir out --tmp-dir wtmp` を指定すると、子プロセスの `--tmp-dir` は `work/wtmp/refine` を指す。項目3はこれも `work/out` 配下にあることを要求している。最終 TOML とレポートは `work/out` 配下、子の `--tmp-dir` は `work/wtmp/refine` と、検証先を明記する。

### 指摘2（高）手動テストは記載どおりに実行すると起動前に失敗する

§1.5 は `phase4/` から再現コマンドを実行するよう指示しているが、そのコマンドの `--directory phase4` は、すでに `phase4/` にいると存在しない `phase4/phase4/` を指す（同じディレクトリ指定で uv が終了コード 2 を返すことを確認）。「リポジトリルートから、`--out-dir` だけを変更して再現コマンドを実行する」と明記する。

既存の再現ログを照合済み。修正後の pytest と GPU を使う手動テストは未実施。

## 対応

- 指摘1: 1.5 テスト28の確認項目3を、`--out-toml`・`--out-report` は `work/out` 配下、`--tmp-dir` は `work/wtmp/refine` と分けて明記
- 指摘2: 1.5 手動テストを「リポジトリのルートで、`--out-dir` だけ変えて実行」と明記し、理由（`uv run --directory phase4` がカレントディレクトリを変える）を追記。1.1 再現手順 2 の文言も同じ趣旨に修正

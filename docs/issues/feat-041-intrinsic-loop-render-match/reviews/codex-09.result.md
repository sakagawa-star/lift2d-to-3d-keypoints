# codex-09 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション1）、requirements.md、design.md（イテレーション1の変更反映）
- ストリーム: rev-lift2d-feat-041
- フェーズ: 反復（不具合修正基準での初回レビュー。`/new` は実施せず、codex-08 の会話を継続）
- 指摘数: 高 0 / 中 1 / 低 0
- トークン実測（累積 total_token_usage）: input 1,253,871（cached 1,062,656）、output 5,308（reasoning 1,570）、total 1,259,179
- rollout: ~/.codex/sessions/2026/11/25/rollout-2026-11-25T10-03-41-01a2103a-d925-7871-b771-49698b6670a2.jsonl

## 総評（Codex）

マッチャー切り替え、信頼度選別、OOM 処理、ログ、再検証手順は、investigation.md の変更案に沿って requirements.md / design.md に反映されている。

## 指摘

### 1. 中 — i=0 で収束すると、修正の要である LoFTR が実行されない

- 対象: design.md:239、requirements.md:106、investigation.md:179
- 内容: 収束条件が従来のままである。初期値が真値に近く、i=0 の MASt3R の対応が同一座標に量子化される場合は、i=0 でそのまま収束して終了する。そのため LoFTR による内部パラメータ更新に進まず、今回特定した見かけの収束が残る。最終精緻化は内部パラメータを固定するので、この抜けを補えない。CPU の合成例では、実際には最大約 1.02px のずれがある同一座標の対応 527 点を与えると、Δ_intr ≈ 5.1×10⁻¹³px、status=3 となり、i=0 で終了した
- 修正案: 収束判定を `i >= LOFTR_FROM_ITER and delta < CONV_PX` とし、少なくとも 1 回は LoFTR の更新後に収束を判定する。FR-005、設計の全体フロー・終了条件、investigation.md を揃え、i=0 の Δ がゼロでも i=1 に進む回帰テストを追加する
- 対応: 反映済み。`is_converged(i, delta)` を新設し、requirements FR-005 受け入れ基準 1、design §3.1 / §4.2.1 / §4.2.5 / §6.2 / §8 テスト 18、investigation.md §1.3「収束判定の変更」を更新した

GPU・実データの再実行は行っていない（Codex 記載）。

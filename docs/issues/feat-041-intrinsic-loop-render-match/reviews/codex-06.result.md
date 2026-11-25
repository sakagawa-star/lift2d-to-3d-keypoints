# codex-06 レビュー結果

- 日付: 2026-11-25
- 対象: `docs/issues/feat-041-intrinsic-loop-render-match/requirements.md`、`docs/issues/feat-041-intrinsic-loop-render-match/design.md`（背景として README.md）
- ストリーム: `rev-lift2d-feat-041`
- フェーズ: 全文ゲート（2回目。`/new` で会話をクリアしてから依頼）
- 指摘数: 高 0 / 中 1 / 低 0
- トークン実測（total_token_usage 累積）: input 667,440（cached 589,824）/ output 2,885（reasoning 1,114）/ total 670,325
- rollout: `~/.codex/sessions/2026/11/25/rollout-2026-11-25T10-00-35-01a21038-006f-7081-8723-38451efb456a.jsonl`

## 指摘

### 1. 重要度：中 — RANSAC の inlier 添字を一次元化せず、最小二乗の残差が全点間比較になる

対象: design.md §4.2.3（`X = X_w[inl], u = uq[inl]`）、§4.2.4（残差の式）

`solvePnPRansac` の `inl` は (N,1)。§4.2.3 の `X = X_w[inl], u = uq[inl]` では、u が (N,1,2) となる。これを §4.2.4 の `(proj.reshape(-1,2) - u)` に渡すと、ブロードキャストにより残差が (N,N,2) になり、対応する点同士の再投影誤差を最小化できない。300 点でこの形状を再現確認した。

修正案: 既存の `refine_extrinsics.run_iteration` と同様に、抽出前に `inl = inl.flatten()` を入れる。併せて `joint_refine` の入力を X: (N,3)、u: (N,2) と明記し、残差が 2N 要素になることを検証する。

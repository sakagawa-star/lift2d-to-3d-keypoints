**結果**

致命的な残存指摘はありません。前回の中3件は解消されています。

- **中1: 分散計算の有効画素**  
  解消済み。[stage4_6_pnp_iteration.py:50](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:50) で `valid_mask` を受け取り、[stage4_6_pnp_iteration.py:103](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:103) で `finite depth && alpha > threshold && z range` を有効画素として渡しているため、pipeline Stage 4 の定義と整合しています。

- **中2: solvePnP 精密化の ok 無視**  
  解消済み。[stage4_6_pnp_iteration.py:146](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:146) の精密化後、[stage4_6_pnp_iteration.py:149](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:149) で失敗時に明示的に failure return しています。

- **中3: 退化診断の扱い**  
  方針として解消済み。[stage4_6_pnp_iteration.py:155](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:155) に「本実験では診断値の記録のみ」と明記され、[experiment_log.md:114](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/experiment_log.md:114) にも同じ方針が記載されています。pipeline 本実装との差分が実験スコープとして明示されたので、スクリプトのバグとは見なしません。

**重要度分類**

- 高: なし
- 中: なし
- 低: なし

実行はしていません。今回はコードと実験ログの整合レビューのみです。
---

## Claude Code の記録（2026-07-31）

Stage 4〜6 実装のレビュー収束（codex-13〜14、高・中ゼロ）。実行へ進む。

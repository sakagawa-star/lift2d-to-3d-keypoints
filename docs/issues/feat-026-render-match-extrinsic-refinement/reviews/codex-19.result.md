再レビュー結果: **高/中/低いずれも指摘なし**。前回の中2件は解消されています。

- `n_used` ガード: [coarse_align_rescue.py:143](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:143) で Kabsch 後の使用点数を受け取り、[coarse_align_rescue.py:144](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:144) で `MIN_RAW_MATCHES` 未満なら粗整列不成立として中断しています。少数点の不安定な `M` を適用する問題は閉じています。

- 最終インライア集合での再推定: [coarse_align_rescue.py:84](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:84) 以降で、最終 `use` が3点以上なら [coarse_align_rescue.py:86](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:86) で `M` を再推定してから返しています。

- Stage 1.5 の補正式: [pipeline.md:140](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:140) 以降に、カメラ中心固定の意図と `C^(0) = -(R^(0))^T t^(0)`、`R^(0) ← M R^(0)`、`t^(0) ← -R^(0) C^(0)` が明記されています。実装の [coarse_align_rescue.py:154](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:154) から [coarse_align_rescue.py:156](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:156) と整合しています。

実行はしていません。コードとドキュメントの整合確認のみです。
---

## Claude Code の記録（2026-07-31）

粗整列レスキュー実装のレビュー収束（codex-18〜19、高・中ゼロ）。実験実行へ進む。

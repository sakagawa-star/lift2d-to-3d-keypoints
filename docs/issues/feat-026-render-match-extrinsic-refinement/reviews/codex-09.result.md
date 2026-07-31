再レビュー結果です。

**高**
なし。

**中**
- 前回の Stage 2 実装不整合は、Stage 2 節と実験ログでは解消されていますが、`pipeline.md` の後段に同じ古い記述が残っています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:385) は A-1 で `render_image(distort=True, return_depth=True)` を使うと書いており、[pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:409) も Stage 2 を「そのまま流用（実装済み）」としています。これは実行不能と判明した経路なので、将来の実験手順・実装スコープを誤誘導します。  
  修正提案: どちらも Stage 2 節と同じ「色: `render_image(distort=True)`、深度+α: `render_depth_alpha_distorted` の2パス」に差し替えてください。

**低**
なし。

前回指摘の解消状況としては、有効画素ゼロ時の `valid.any()` 分岐は [stage2_smoke_test.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage2_smoke_test.py:140) で入っており解消。ED/NaN セマンティクスの注記もスクリプトと Stage 2 節に反映済みです。
---

## Claude Code の対応方針（2026-07-31）

- 中1（pipeline.md 後段の旧記述残存）: §5 実験A-1 と実装スコープ表の2箇所を「2パス方式（色: render_image(distort=True)、深度+α: render_depth_alpha_distorted）」に差し替え

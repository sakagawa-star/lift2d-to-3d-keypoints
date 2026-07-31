レビュー結果です。コード変更はしていません。

**高**
なし。方式Aの中核は正しいです。gsplat 1.5.3 は `RGB+ED` で depth を色チャンネルに追加して rasterize 後に `alpha` で割る実装なので、`z_i` を3ch色として流して `accum / alpha` する現在の実装は、有効α画素では `D=Σ(w·z)/Σw` と同じです。`backgrounds` も渡していないため、深度への背景混入もありません。`viewmat` も `X_C = R X_W + t` として扱えており、`means @ R.T + t` は正しいです。

**中**
- αが十分な画素がゼロのとき、失敗を報告する前に統計計算で落ちます。  
  [stage2_smoke_test.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage2_smoke_test.py:137) の `valid = (alpha1 > 0.5) & isfinite(depth1)` が空でも、直後に `median` / `percentile` を計算しています。これは「座標系が壊れて何も見えない」「near_plane が強すぎる」など、Stage 2 smoke が検出すべき失敗ケースで結果ファイルを残せない可能性があります。  
  修正提案: `if not valid.any():` で分岐し、深度統計は `N/A`、判定は明示的に失敗として `stage2_result.txt` と `query_alpha.npy` / `query_depth.npy` を保存してください。

- `pipeline.md` の Stage 2 実装記述が、今回の方式Aと食い違っています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:154) と [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:174) はまだ「`RGB+ED` を1回」「`render_image(distort=True, return_depth=True)` をそのまま流用」と読めますが、実機確認済みの前提ではその経路は停止します。式は合っていますが、実装定義が古いです。  
  修正提案: Stage 2 の式は維持しつつ、gsplat 1.5.3 の3DGUT/eval3d制約により、実装は「RGBレンダ1回 + `render_depth_alpha_distorted` による深度/αレンダ1回」と明記してください。実験ログの [experiment_log.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/experiment_log.md:38) も同じ更新が必要です。

**低**
- `alpha <= 1e-6` を NaN にする点は、厳密には gsplat の `ED` 出力そのものとは違います。gsplat 側は `alpha.clamp(...)` で割るため、完全背景は通常 0 深度相当になります。一方、このスクリプトの NaN 化は Stage 4 の `alpha` / finite gate 前提ではむしろ安全です。  
  修正提案: コメントを「EDと同一セマンティクス」は `alpha > 1e-6` の有効画素に限る、と弱めてください。背景を NaN にするのは妥当です。
---

## Claude Code の対応方針（2026-07-31）

- 中1（有効画素ゼロ時のクラッシュ）: `valid.any()` で分岐し、深度統計 N/A・疎通判定「失敗」を明示して結果ファイルを保存するよう修正
- 中2（pipeline.md / experiment_log.md の実装記述が旧方式のまま）: pipeline.md Stage 2 の実装注記を「3DGUT レンダ2回（色 + z_iを色として流す深度）」に訂正し、RGB+ED 併用が実行不能と実機判明した経緯を記録。experiment_log.md のセットアップ表と実行記録にも同内容を反映
- 低1（EDセマンティクスの注記）: 「α > 1e-6 の有効画素に限り ED と同一。完全背景は gsplat では0深度相当、本実装では NaN（Stage 4 ゲート前提では安全側）」にコメントを弱めた

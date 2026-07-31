再レビュー結果: **高はなし**。ただし、Stage 4〜6 の定義整合として無視しづらい **中** が3件あります。

**中**

1. [stage4_6_pnp_iteration.py:50](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:50) の `depth_variance_map` が `finite(depth)` のみで局所分散を計算しており、[pipeline.md:209](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:209) の「窓内の有効画素、つまり gate(iv) を満たす画素のみ」と一致していません。  
   低α画素や z range 外の深度が分散に混ざるため、gate(ii) が pipeline 定義と異なる判定になります。  
   修正案: `depth_variance_map(depth, valid_mask)` にして、`valid_mask = isfinite(depth) & (alpha > GATE_ALPHA) & z範囲内` だけで `cnt/s1/s2` を計算してください。

2. [stage4_6_pnp_iteration.py:142](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:142) の `solvePnP` 精密化結果の `ok` を無視しています。  
   既存の `estimate_extrinsic` 系と同型にするなら、精密化失敗時は失敗として扱うべきです。現状だと失敗しても `rvec/tvec` をそのまま使って RMSE・姿勢誤差・保存結果を出します。  
   修正案: `if not ok: result.txt に failure を書いて return` を追加。

3. [stage4_6_pnp_iteration.py:146](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage4_6_pnp_iteration.py:146) で退化指標は計算していますが、pipeline Stage 6 の「閾値未満なら fail」としては使っていません。  
   `experiment_log.md` では「退化診断: 通過」を予測していますが、スクリプト上は通過/失敗を判定できません。  
   修正案: 実験用に `vol_ratio`, `depth_range`, `area_ratio` の閾値を定数化し、未満なら failure にする。測定のみが意図なら pipeline/log 側で「この実験では診断値の記録のみ」と明記してください。

確認済みで問題なし:
- `keep` と `inl` の二重添字は整合しています。
- `knnMatch` 由来の `u_r/u_q` 方向を前提にした 3D化、PnP 入力、oracle purity の参照方向は破綻していません。
- Stage 5 の `undistortPoints(P=K) -> K^-1 ray -> z scale -> W系` は pipeline の式と整合しています。
- SIFT のため gate(i) を省略する判断は、実験ログの前提と矛盾していません。
---

## Claude Code の対応方針（2026-07-31）

- 中1（分散計算の有効画素）: depth_variance_map に valid_mask 引数を追加し、ゲート(iv)相当（有限・α・z範囲）を満たす画素のみで分散計算するよう修正（pipeline.md Stage 4 の定義に整合）
- 中2（精密化の ok 無視）: solvePnP 精密化失敗時は failure を記録して終了するよう修正
- 中3（退化診断の扱い）: 本実験は「診断値の記録のみ」と方針決定（閾値はこの実験群の実測から較正予定のため、先に仮置きすると循環になる）。スクリプトのコメントと experiment_log.md の予測欄に明記

再レビューしました。前回指摘の2件は解消されています。

- `gate_depth_quality` の分散閾値: `tau_d ** 2` まで明記されました。
- ブートストラップの試行失敗 / 確定失敗: 分離されました。

**高**
なし。

**中**
1. **`gate_depth_quality` の `pixel_valid` が feat-026 実装と一致していない**
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:110) は「feat-026 実装と同一」としつつ、`pixel_valid = isfinite(depth) & (alpha > 1e-6)` としています。実際の feat-026 側は `alpha > GATE_ALPHA` と `GATE_Z_RANGE` も含めて `pixel_valid` を作り、そこから `var_map` と `tau_d` を計算しています（[refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:524)）。この差で局所分散と中央値深度の基準が変わり、ブートストラップPnPの入力点が汚れます。
   修正提案: `pixel_valid = isfinite(depth) & (alpha > GATE_ALPHA) & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1])` に修正してください。`pixel_valid` が0件の場合は全点 reject も明記すると安全です。

**低**
なし。
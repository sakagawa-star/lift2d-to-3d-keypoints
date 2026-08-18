再レビューしました。前回指摘の2件は解消されています。

- `--toml` / `--init-toml` のマージ規則: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:57) で明確化済み。
- LoFTR confidence filter の所在: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:120) で `collect_anchor_points` 側責務として明確化済み。

**高**
なし。

**中**
1. **`gate_depth_quality` の局所分散閾値が次元不一致のまま**
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:110) は「局所分散 ≤ `GATE_VAR_REL`」と書いていますが、`GATE_VAR_REL=0.02` は相対係数であり、分散値と直接比較する値ではありません。feat-026 の実装は `tau_d = GATE_VAR_REL * median(depth[pixel_valid])`、判定は `var_map < tau_d ** 2` です（[refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:524)）。
   修正提案: `gate_depth_quality` の判定式を明記してください。例: `pixel_valid` 作成、`var_map = depth_variance_map(...)`、`tau_d = GATE_VAR_REL * median(valid_depth)`、`cond_var = var_map[iy, ix] < tau_d ** 2`。

2. **ブートストラップ失敗リスト投入のタイミングがまだ曖昧**
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:112) は「不足なら失敗リストに移す」と書きつつ、同じ文で「別ペアが残っていれば次周で試す」としています。実装者が先に fail 確定すると、別の較正済みカメラとの有効ペアを試さずに未較正カメラを落とせます。
   修正提案: 「試行失敗」と「確定失敗」を分けて定義してください。`u` ごとに試行済み `(c,u)` ペアと理由を保持し、S×U の未試行採用ペアがなくなった時点でのみ確定failにする、と明記するのが安全です。

**低**
なし。
**高**

なし。

**中**

1. [pipeline_loop.py:207](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/pipeline_loop.py:207) の収束判定用の並進更新量が、`pose_error()` 経由で **カメラ中心差**になっています。一方、[pipeline.md:292](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:292) は `||t^(k+1)-t^(k)||` と書いています。  
   カメラ中心差の方が座標系としては自然ですが、現状は Stage 7 の定義式と実装が一致していません。  
   修正案: 実装方針をカメラ中心差で通すなら、pipeline.md の Stage 7 式を `C=-R^T t`、`Δτ=||C^(k+1)-C^(k)||` に直す。厳密に pipeline.md に合わせるなら `np.linalg.norm(tvec_new - tvec_cur)` に変更する。

2. [pipeline_loop.py:140](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/pipeline_loop.py:140) の退化診断が `λ3/λ1` と `depth_range` だけで、stage4_6 で記録していた **凸包面積比** が落ちています。pipeline Stage 6 の退化診断3条件、および Stage 4〜6 実験結果との比較に必要な値なので、収束域マップ用の履歴として欠落です。  
   修正案: `hull = cv2.convexHull(u_q_keep[inl].astype(np.float32))` と `area_ratio = contourArea / (width*height)` を追加し、`diag` と history に保存する。

**低**

なし。

確認済み:
- `SCHEDULE` と `sched(k)` は現行の `k=0` / `k>=1` 方針と整合しています。
- `--start-from-stage4-6` は `k_start=1` なので、Stage 7 の引き締め設定を使います。
- `run_iteration` の Stage 3/4/5/6 処理はレビュー済み実装から大きく劣化していません。
- oracle 採点は `purity` などの診断値にしか使われておらず、ゲート・PnP・収束判定には混ざっていません。
- `python3 -m py_compile` は通りました。
---

## Claude Code の対応方針（2026-07-31）

- 中1（収束判定のΔτ定義の不一致）: 実装（カメラ中心差）に合わせて pipeline.md Stage 7 の式を Δτ = ||C^(k+1) − C^(k)||, C = −R^T t に修正し、理由（t は回転と結合した量のため、物理的に意味のあるカメラ移動量で判定）を注記
- 中2（凸包面積比の欠落）: diag に area_ratio を追加（stage4_6 と同じ計算。収束域マップの履歴・閾値較正の材料として保存）

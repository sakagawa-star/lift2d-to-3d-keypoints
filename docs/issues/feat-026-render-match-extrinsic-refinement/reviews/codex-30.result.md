**高:** なし。

**中:** `cv2` の RANSAC seed が `m3_run.py` と一致していません。  
`m3_diag_quasi.py` は固定で `SEED_CV = 42` を使っていますが、M3 本実験は `SEED_CV_BASE + ci * 10 + si` なので、P0 なら `cam05520129=110`, `cam41520557=140` です。準誤マッチは「RANSAC 外れ」と定義されているため、seed 差分は `f_sp`/`f_per` の測定対象そのものを変え得ます。  
該当: [m3_diag_quasi.py:54](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_diag_quasi.py:54), [m3_diag_quasi.py:186](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_diag_quasi.py:186), [m3_run.py:50](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_run.py:50), [m3_run.py:326](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_run.py:326)

修正提案: `m3_run.py` の `CAMERAS`/`SEED_CV_BASE` と同じ式で、対象・対照それぞれの P0 seed を設定してください。少なくとも出力にも実 seed をカメラ別に記録するべきです。

**低:** なし。

それ以外の `f_sp`、`f_per`、`|d_peak|`、重畳可視化、座標系、1px ビンの扱いは D1 定義と整合しています。Stage 2〜6 の処理本体も、分類目的で `solvePnP` 精密化を省いている点を除けば、主要条件は `m3_run.py` と一致しています。
---

## Claude Code の対応方針（2026-07-31）

- 対象: m3_diag_quasi.py 実行前レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892 / 初回（診断スクリプト）
- 中1（RANSAC seed が本実験と不一致）: m3_run.py から CAMERAS / SEED_CV_BASE を import し、P0 系列と同一式（cam05520129=110、cam41520557=140）で設定。実 seed をカメラ別に結果へ記録

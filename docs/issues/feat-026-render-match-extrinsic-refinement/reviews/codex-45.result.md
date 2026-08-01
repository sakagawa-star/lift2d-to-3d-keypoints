**Findings**

- **中**: [m4_stage0.py:112](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_stage0.py:112) は `len(names) < 6` だけを拒否しており、7点以上が混入した場合もそのまま PnP に使います。criteria §5-1 は「6点PnP」として lock 済みなので、余分な点が入ると Stage 0 の初期ポーズが criteria と別条件になります。現CSVは6点でしたが、スクリプト側で固定条件を守れていません。  
  **修正提案**: `len(names) != 6` を明示的失敗にし、結果ファイルにも「手動点数が criteria と不一致」と記録して終了する。

**確認済み**

PnP 経路自体は `solvePnPRansac(iterations=1000, reprojectionError=8.0, SOLVEPNP_ITERATIVE) → inlierで solvePnP(ITERATIVE)` で、`phase0/estimate_camera_params.py` の K既知・6点以上経路と一致しています。Stage 0 の記録項目（重畳PNG、有効画素率、深度分布/z干渉、SIFT数）も criteria §8-1 を満たしています。

構文確認は `UV_CACHE_DIR=/tmp/uv-cache uv run --project phase4 python -m py_compile ...` で通っています。
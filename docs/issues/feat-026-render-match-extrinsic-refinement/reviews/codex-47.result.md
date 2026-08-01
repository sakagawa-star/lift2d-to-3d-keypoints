**Findings**

- **中**: §6.3 の実測記録が criteria の「全反復・段ごとの通過数」と一致していません。criteria は「生マッチ→ratio→変位ゲート→深度ゲート→RANSAC インライア」を全反復で記録する要求ですが、[m4_run.py:403](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:403)〜[406](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:406) は k≥1 の60反復だけを集計し、k=0 の3反復を除外しています。また [m4_run.py:170](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:170)〜[176](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:176) は raw/ratio/変位/深度/RANSAC の逐次通過数ではなく、独立条件の集計に寄っています。Go/No-Go ロジック自体は歪みませんが、No-Go 時の失敗モード切り分けや (b-1)〜(b-3) の実測が criteria より狭くなります。  
  **修正提案**: 各 `diag` に `n_raw`（または knn候補数）, `n_ratio=len(good)`, `n_disp`, `n_depth_gate`, `N`, `inliers` を同じ分母・順序で記録し、結果 txt では k=0 と k≥1 を分けるか、全63反復の範囲として集計してください。パイプラインの `keep` 条件は変えず、診断集計だけを追加するのが安全です。

**確認済み**

`run_iteration_real_m4` のパイプライン本体は M3 の `run_iteration_real` と同じ処理・定数・順序に見えます。サンプリング手順、RANSAC seed の与え方、pooled 合意、二峰仲裁条件、§6.4 前提条件、§7 判定式も criteria と一致しています。

構文確認は `UV_CACHE_DIR=/tmp/uv-cache uv run --project phase4 python -m py_compile ...` で通っています。実レンダ実行はしていません。
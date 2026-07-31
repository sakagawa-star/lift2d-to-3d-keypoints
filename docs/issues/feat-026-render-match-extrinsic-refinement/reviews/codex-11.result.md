レビュー結果です。致命度のある指摘だけ挙げます。

**高**
- 正解変位場が「厳密計算」になっていません。  
  [stage3_perturb_match.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage3_perturb_match.py:85) で SIFT のサブピクセル座標 `u_r` を `astype(int)` して深度参照しており、実質 floor サンプリングです。一方、逆投影の ray は [stage3_perturb_match.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage3_perturb_match.py:91) の元のサブピクセル座標から作っています。深度だけ隣接ピクセルの値になるため、特にSIFTが乗りやすいエッジ・シルエット・深度不連続で oracle がずれ、正しいマッチを誤って不正解にできます。Stage 3 の主指標を壊し得ます。  
  修正提案: `u_r` の深度は bilinear 補間で取得してください。2x2近傍に NaN がある、αが低い、深度分散が大きい場合は oracle 無効として除外するのが妥当です。少なくとも floor ではなく、補間不能点を明示的に invalid 扱いにしてください。

**中**
- マッチング失敗ケースで結果を残せずクラッシュします。  
  [stage3_perturb_match.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage3_perturb_match.py:147) の `imread` 失敗、`detectAndCompute` の descriptor `None`、`knnMatch` の近傍数不足、ratio後マッチ0件で落ち得ます。特に `good == []` だと [stage3_perturb_match.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/stage3_perturb_match.py:156) が `(0,)` 配列になり、`oracle_displacement` 内の `u_r[:, 1]` で失敗します。SIFTが十分取れるかを測る実験なので、失敗はクラッシュではなく測定結果として残すべきです。  
  修正提案: 入力画像・descriptor・`len(des_q) >= 2`・`len(des_r) > 0` をチェックし、`knn` は `len(pair) == 2` のものだけ ratio test してください。マッチ0件時は `(0,2)` 配列で保存し、`stage3_result.txt` に「マッチ0」と書いて正常終了してください。

**低**
なし。

`perturb_pose` の `C=-R^T t`、`t'=-R'C'`、`R'=R_delta @ R` は意図どおりの左摂動として整合しています。`oracle_displacement` の `undistortPoints → K^-1 → dスケール → R^T(X-t) → projectPoints` も、深度サンプリングを除けば pipeline Stage 5 と一致しています。`knnMatch(des_r, des_q)` に対する `queryIdx=レンダ`, `trainIdx=クエリ` の扱いも正しいです。
---

## Claude Code の対応方針（2026-07-31）

- 高1（深度のfloor参照でoracleがずれる）: `sample_depth_bilinear` を追加し bilinear 補間に変更。2x2近傍の画像外・NaN含み・深度広がりが中心深度の5%超（不連続=シルエット）は oracle 無効として除外
- 中1（マッチング失敗ケースのクラッシュ）: imread None は明示エラー、descriptor None/不足のガード、knnMatch は len(pair)==2 のみ ratio test、マッチ0件でも (0,2) 配列で正常続行し結果を記録するよう修正。採点の NaN 処理も isfinite ベースに明瞭化

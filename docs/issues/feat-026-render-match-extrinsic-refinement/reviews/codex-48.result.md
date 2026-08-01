**Findings**

- **中**: 成立性が不成立のとき、§6.3 の診断集計を出さずに早期 return します。[m4_run.py:328](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:328)〜[346](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:346) で No-Go を書いて終了するため、No-Go 時に criteria §7 の「失敗モードを (b-1)〜(b-4) の実測から特定」が結果 txt に残りません。診断集計は [m4_run.py:411](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:411) 以降なので、3チェーン成立時にしか出ません。  
  **修正提案**: §6.3 集計を関数化し、`est_ok` 不成立時も取得済み `diags` の範囲で出力してから No-Go で終了してください。未実行分は「未実行」と明記すればよいです。

- **中**: 深度系除外の逐次分類順が、α/z 不良を分散除外に吸い込む可能性があります。[m4_run.py:163](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:163)〜[166](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:166) の `pixel_valid` は α と z を含むため、α/z が悪い画素では `cond_var` も落ちやすいです。その後 [m4_run.py:180](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:180)〜[183](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_run.py:183) で `excl_var` を `excl_alpha` / `excl_z` より先に分類しているため、(b-2) の失敗原因内訳が歪みます。  
  **修正提案**: 除外内訳は `cond_disp` 後に `depth_ok → alpha → z → var` の順で分類してください。`keep` 自体は現在の AND 条件のままで変更不要です。

**前回指摘の解消状況**

n_raw、逐次通過数、k=0/k≥1 の分離集計は成功時パスでは解消しています。`keep` の計算と Go/No-Go 判定式は criteria と一致しています。

構文確認は通っています。実レンダ実行はしていません。
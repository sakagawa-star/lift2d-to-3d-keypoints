**高**

1. `MAX_NFEV=200` のままだと、今回の修正が既知の実測条件で成立しません。  
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:75) では `MAX_NFEV = 200`、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:167) では `max_nfev=MAX_NFEV, x_scale='jac'` ですが、同じ段落の実測根拠は `x_scale='jac'` でも `nfev=270` で収束です。つまり設計どおり実装すると 200 回で打ち切られ、[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:85) の FR-005「`status >= 1`」を満たせない可能性が高いです。

   修正提案: `MAX_NFEV` を少なくとも実測収束回数 270 を上回る値、例えば `500` または `1000` に更新し、§2.5 に「`x_scale='jac'` 前提でも実測 `nfev=270` のため、収束余裕を持って `MAX_NFEV` を設定する」と明記してください。もし `MAX_NFEV=200` を維持するなら、`max_nfev=200` でも `status>=1` になる実測を取り直して、その結果を根拠として置く必要があります。

**中・低**

該当なし。

`x_scale='jac'` を必須化する判断自体は、未知数の単位スケール不均衡に対する対策として妥当です。ただし現状は評価回数上限だけが実測根拠と食い違っており、ここを直さないと FR-005 の受け入れ基準に届かない設計になります。
前回指摘の高1件は解消されています。

[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:75) で `MAX_NFEV=1000` に更新され、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:168) でも `x_scale='jac'` 前提の実測 `nfev=270` に対して十分な余裕を持つ根拠が明記されています。これにより [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:85) の FR-005「`status >= 1`」受け入れ基準との食い違いは解消されています。

**高・中・低**

該当なし。

`x_scale='jac'` 必須化、`MAX_NFEV=1000`、FR-005 の収束条件の間に、現時点で致命的な矛盾は見当たりません。
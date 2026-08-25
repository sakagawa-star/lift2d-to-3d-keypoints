前回指摘の高1件は解消されています。

[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:80) は `w_k=1` 固定・交会角重み無効化・`TRI_MIN_ANGLE_DEG` ゲート維持に更新済みで、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:167) の主経路仕様と一致しています。ADR も [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:275) で `w_k=1` 採用、関数・単体テスト温存に差し替わっています。

**高・中・低**

該当なし。

§4 の交会角重みテストは、`crossing_angle_weight` 関数を将来再有効化用に温存する方針と整合しています。requirements/design 間で、採用値 `HUBER_PX=1.0`、`W_CROSS=2.0`、`w_k=1` に関する致命的な矛盾は見当たりません。
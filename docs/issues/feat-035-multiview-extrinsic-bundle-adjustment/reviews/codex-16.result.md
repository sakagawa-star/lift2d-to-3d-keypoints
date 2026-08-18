再レビューしました。前回指摘2件は解消されています。

- Stage B の S/U 定義とスキップ条件は、マージ後の `None` 判定に修正済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:107)、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:117)
- p0_bootstrap 実験手順で `--init-cameras cam05520125 cam05520126` の明示指定が必須化済み: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:17)

**高**
なし。

**中**
なし。

**低**
なし。

同種の Stage B スキップ事故を防ぐために必要な requirements/design/criteria 側の記述は揃っています。致命的に修正すべき問題は見当たりません。
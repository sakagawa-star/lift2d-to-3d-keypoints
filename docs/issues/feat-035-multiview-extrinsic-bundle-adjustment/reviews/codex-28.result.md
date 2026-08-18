再レビューしました。前回指摘4件は解消済みです。

確認箇所:
- Stage F のエラー条件は、全ペア不採用または連結性不成立で終了コード1に統一済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:148), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:151)
- `--init-cameras` 省略条件は §2.0 / CLI / テスト設計に反映済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:60), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:192), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:241)
- Step 3F criteria に連結性 Go 条件と直前予測枠が追加済み: [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:65)
- ADR のアンカー不足カメラは信頼初期ポーズ表現に修正済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:268)

**指摘事項**

高: なし  
中: なし  
低: なし

現時点で、requirements / design / roadmap 間に実装を止めるレベルの矛盾や、実験プロトコル違反につながる未定義事項は見つかりませんでした。
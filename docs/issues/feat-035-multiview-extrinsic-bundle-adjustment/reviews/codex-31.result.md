再レビューしました。前回 codex-30 の中2件はいずれも解消済みです。

確認箇所:
- `sched` が流用元一覧に追加済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:25)
- `sched` の import 方針も明記済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:87)
- 定数一覧の前書きが FR-008 対象/固定流用値を区別する表現に修正済み: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:62)
- FR-003 のゲート定数に `ANCHOR_DISP_TAU_PX=10.0px` が追加済み: [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:61)

**指摘事項**

高: なし  
中: なし  
低: なし

技術的根拠、requirements/design 間の整合、FR-008 対象外の扱いはいずれも問題ありません。
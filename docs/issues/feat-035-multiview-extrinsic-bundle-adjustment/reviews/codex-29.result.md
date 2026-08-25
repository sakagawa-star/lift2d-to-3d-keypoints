レビューしました。criteria lock を妨げる高・中の問題は見つかりませんでした。

確認できた点:
- 測定実験と Go/No-Go 判定の分離が明記されています: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_epi_filter/criteria.md:7)
- Q1〜Q3 は Stage F / roadmap Step 3F と整合しています: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_epi_filter/criteria.md:13)
- 直前予測枠に採用ペア数・連結成否・Go/No-Go 予測が含まれています: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_epi_filter/criteria.md:24)
- Go/No-Go 条件と No-Go 時の3選択肢が事前定義されています: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_epi_filter/criteria.md:34)
- EPI_TOL_PX の採用値決定を Step 7 に送る扱いも明確です: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_epi_filter/criteria.md:47)

**指摘事項**

高: なし  
中: なし  
低: なし

この内容で criteria lock に進めてよい状態です。
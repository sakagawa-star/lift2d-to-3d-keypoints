レビューしました。高はありませんが、**中 2件**あります。

**高**
なし。

**中**

1. **目視確認が進行条件にどう効くかが未定義**

[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:30) は「1台以上成功なら Step 4」としていますが、[criteria.md:31](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:31) の目視確認が NG だった場合に止めるのか進むのかが書かれていません。roadmap では目視問題時に進行停止です（[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:48)）。

修正提案: §4 に「Step 4 へ進む条件は、6台中1台以上成功、かつ成功カメラの比較画像が固定済み目視判定項目を満たすこと。目視NGの場合は進行停止し調査に戻る」と明記してください。

2. **直前予測の枠が Q1/Q2 の主要測定値をカバーしていない**

Q1 はゲート通過対応点数・PnP inlier 数・inlier RMSE を測ります（[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:11)）。Q2 は全特徴点中央値と上段除外中央値を測ります（[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:12)）。しかし予測枠は inlier 数範囲と上段除外中央値だけで、ゲート通過数・inlier RMSE・全特徴点中央値の予測がありません（[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:23)）。CLAUDE.md の予測→実行→照合に対して、照合対象が欠けます。

修正提案: §3 に以下を追加してください。

- ゲート通過対応点数の範囲予測
- inlier RMSE の範囲予測 [px]
- ずれ中央値の「全特徴点」と「上段除外」の両方の範囲予測

**低**
なし。
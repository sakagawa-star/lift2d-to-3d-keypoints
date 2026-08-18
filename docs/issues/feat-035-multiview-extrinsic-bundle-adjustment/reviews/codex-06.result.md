先行研究反映分を中心に再レビューしました。MASt3R相互最近傍の明記自体には、致命的な矛盾は見つけていません。

**高**
なし。

**中**
1. **FR-008 の実験項目が requirements と design で一致していない**
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:204) では `THETA_SIGMA_DEG`、交会角重み無効化 `w_k=1`、アンカーMASt3R比較が FR-008 の実験候補に追加されています。一方、[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:88) の FR-008 は旧来の項目リストのままで、交会角重みやアンカーMASt3R比較が要求側に出ていません。
   修正提案: requirements の FR-008 概要・出力・受け入れ基準にも、`THETA_SIGMA_DEG` と `w_k=1` ベースライン、任意追加のアンカーMASt3R比較を明記してください。

2. **FR-005 は交会角重み必須、FR-008 は無効化採用もあり得る書き方になっている**
   [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:64) はクロス項に交会角ガウス重みを「適用する」としています。しかし [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:204) は `w_k=1` の無効化を走査範囲に含め、ホールドアウト中央値最小の設定を採用する流れです。無効化が最良なら、FR-005 と採用結果が矛盾します。
   修正提案: `w_k=1` は「比較用ベースラインであり採用候補ではない」と明記するか、FR-005 側を「交会角重みは既定で適用し、FR-008で無効化を含めて採否を決める」に変更してください。

**低**
なし。
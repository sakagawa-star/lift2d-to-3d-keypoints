**中**

1. `EPI_TOL_PX` / `N_PAIR_MIN` の採用判定が、比較不能なホールドアウト中央値に依存しています。  
   [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/criteria.md:35) で、`EPI_TOL_PX`・`N_PAIR_MIN`・`HOLDOUT_RATIO` はホールドアウト集合が変わるため同一集合比較ではない、と正しく認識しています。一方で [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/criteria.md:46) の採用規則は `HOLDOUT_RATIO` だけを例外にし、`EPI_TOL_PX` / `N_PAIR_MIN` には中央値最小ルールをそのまま適用しています。これは、難しい対応点やペアを落とした設定が見かけ上よく見えるため、採用値決定を歪めます。

   修正提案: `EPI_TOL_PX` / `N_PAIR_MIN` についても事前に coverage-aware な採用規則を定義してください。例えば「連結性成立に加え、採用ペア数・ホールドアウト点数がベースライン以上、または事前定義した下限以上の候補だけで中央値最小を採用」とする。もしくは `HOLDOUT_RATIO` と同様に「感度記録のみで既定値維持」と明記してください。どちらにする場合も [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:238) の採用規則と矛盾しないよう、design §3 側も合わせて更新するのが安全です。

**高・低**

該当なし。

design §3 の必須項目、直前予測枠、任意項目の採否は概ね満たしています。上記の採用規則だけ、criteria lock 前に閉じないと事後解釈の余地が残ります。
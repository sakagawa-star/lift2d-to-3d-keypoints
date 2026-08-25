前回指摘の中1件は解消されています。

[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/criteria.md:48) で `EPI_TOL_PX` / `N_PAIR_MIN` の採用候補が「連結性成立、採用ペア数がベースライン以上、ホールドアウト点数がベースライン以上」に制限され、難しい点・ペアを落としただけの設定が中央値最小で採用される問題は塞がれています。さらに [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:238) 側も同じ coverage-aware 規則に更新されており、criteria/design 間の矛盾もありません。

**高・中・低**

該当なし。

design §3 の必須事項、HOLDOUT_RATIO の比較不能扱い、任意項目の採否、CLAUDE.md の事前定義プロトコルとの整合はいずれも問題ありません。
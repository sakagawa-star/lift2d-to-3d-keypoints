レビューしました。構造としては概ね整っていますが、CLAUDE.md の実験プロトコル上、**中 1件**あります。

**高**
なし。

**中**

1. **直前予測の枠が Go/No-Go 判定の主要出力をカバーしていない**

[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_pair_connectivity/criteria.md:13) の Q3 は、Q2 が No の場合に `N_PAIR_MIN'` を決める重要な測定項目です。さらに [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_pair_connectivity/criteria.md:32) では、その存在有無が Go/No-Go に直結します。

一方、直前予測の枠は [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_pair_connectivity/criteria.md:24)〜[27](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_pair_connectivity/criteria.md:27) で、`N_PAIR_MIN'` の予測、Go/No-Go 予測、OOM/error 件数の予測がありません。CLAUDE.md は実行直前に入出力予測を experiment_log に記録する運用を求めています（[CLAUDE.md](/home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:256)）。

修正提案: §3 に以下を追加してください。

- P4: Q2 が No の場合の暫定閾値 `N_PAIR_MIN'` の予測値（候補: 45,40,...,5, または連結不能）
- P5: Go/No-Go 予測
- P6: OOM/error になるペア数と該当ペアの予測

**低**
なし。
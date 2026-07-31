**高**

1. D2 の RMSE/Jaccard 測定が再現可能な形に固定されていません。  
[experiment_log.md:221](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/experiment_log.md:221) の「各アトラクタ中心で1反復のみ」は、RANSAC seed、k 条件、どの2つを「アトラクタ中心」とするかが未定義です。さらに Jaccard も「マッチ座標の近傍対応」とだけあり、近傍半径・座標系・対応の一意化規則がありません。  
このままだと `RMSE差` と `Jaccard` が実装者の裁量で変わり、D2 の原因解釈を後から動かせます。

修正提案: D2 に以下を固定してください。

- アトラクタA/Bの定義: 例 `A = 554 P0/P1 pooled中心`, `B = 554 P2中心` など
- 1反復条件: `k=1`, gate 10px, RANSAC 2px, seed 固定
- 可能なら seed 1本ではなく固定 seed list で RMSE/Jaccard の中央値を使う
- Jaccard: query座標かrender座標か、近傍半径 px、相互最近傍か片方向か、式 `|A∩B|/|A∪B|` を明記

**中**

1. M3-2 の Go 条件で、他4カメラの第2クラスタを禁止していません。  
[criteria_m3_2.md:16](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_2.md:16) では第2クラスタ占有率 `≥0.2` を二峰分類しますが、[criteria_m3_2.md:28](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_2.md:28) の他4カメラ受理条件には `第2クラスタ < 0.2` が入っていません。  
つまり、他4カメラで `pooled f_c≥0.7` かつ `第2クラスタ≥0.2` の強い副モードが出ても Go になり得ます。

修正提案: 条件2に「かつ第2クラスタ占有率 < 0.2」を追加してください。M3-1 で二峰を No-Go にした基準と揃います。

**低:** なし。

確認点として、M3-2 の `pooled f_c≥0.7` は、3チェーン中1チェーン20サンプルが完全に別アトラクタへ定着するケースを `40/60=0.667 < 0.7` で必ず落とす、という主張は正しいです。`m3_1_history.npz` に各系列20サンプルが保存されているため、M3-2 を再サンプリングなしの再分析にする点も妥当です。
---

## Claude Code の対応方針（2026-07-31）

- 対象: criteria_m3_2.md + D2 計画（experiment_log.md）初回レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 高1（D2 測定が未固定）: アトラクタ定義（A=554 P0/P1 pooled 中心、B=P2 中心）、実行条件（k=1、seed リスト {7000..7009} の10通り、A/B 同一 seed）、RMSE は中央値同士の差、Jaccard は「u_q 距離≤2px の相互最近傍対応、|A∩B|/|A∪B|、10 seed の中央値」と明記
- 中1（M3-2 条件2に第2クラスタ禁止なし）: 条件2に「かつ第2クラスタ占有率 < 0.2」を追加

再レビュー結果です。

前回の高指摘だった「§5.3 が数値未確定で lock できない」問題は解消されています。criteria の p95 表は [m3_0a_results_summary.txt](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_results_summary.txt:8) の実測値と一致しています。

**高**

- なし。

**中**

- [m3_0a_resample_p95.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_resample_p95.py:41) の `REQUIRED_CAMS` が旧判定対象3カメラのままです。criteria は [全5カメラ判定](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:27)に改訂済みなので、再現実行時に旧低信頼2カメラが欠けてもスクリプトが成功扱いになります。今回の出力には5カメラ揃っていますが、lock の根拠スクリプトとしては不整合です。  
  修正提案: `REQUIRED_CAMS` を5カメラ全てに更新し、M3-0a を再実行して同じ summary を得たことを `experiment_log.md` に追記する。

- [experiment_log.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/experiment_log.md:28) の「クリックミスではなく」「Config_scene の K・歪みはこれらの手動点から推定されたものなので」は、現ログだけだと少し断定が強いです。実測から確実に言えるのは、`intrinsics_all` 条件で見えた高外れ値率がデータ品質指標として不適切で、Config_scene 条件では大幅に解消した、というところまでです。  
  修正提案: 根拠資料を追加できないなら、「M1 の高外れ値率は intrinsics_all 条件下の系統的再投影誤差を大きく含んでおり、手動点そのものの品質指標として使うのは誤りだった」程度に弱める。

**低**

- なし。

層別廃止と Go=5カメラ中3以上への変更は、M3 本実験前の M3-0a 結果だけに基づく改訂なので、事後的な合否操作とは見ません。上の2点を直せば lock 文書として通せます。
---

## Claude Code の対応方針（2026-07-31）

- 対象: 数値確定版 criteria.md + experiment_log.md レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 中1（REQUIRED_CAMS が旧3カメラのまま）: 5カメラに更新して M3-0a を再実行。summary は初回と完全一致（diff 差分ゼロ）を確認し experiment_log.md に追記
- 中2（解釈の断定が強い）: experiment_log.md の解釈を「intrinsics_all 条件下の系統的再投影誤差を大きく含み、手動点そのものの品質指標として使うのは誤りだった」に弱め、Config_scene 生成過程由来の説明は「状況証拠で未確認」と明記

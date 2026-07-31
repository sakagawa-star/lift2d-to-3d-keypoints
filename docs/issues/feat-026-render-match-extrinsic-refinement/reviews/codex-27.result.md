再レビュー結果です。

前回指摘のうち、`REQUIRED_CAMS` 5カメラ化と再実行一致の記録は解消されています。`experiment_log.md` 側の断定も適切に弱まっています。

**高**

- なし。

**中**

- [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:30) 側にはまだ「M1 の高い外れ値率は内部パラメータ不一致による系統誤差が主因」という断定が残っています。同じ趣旨が [§5.3](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:86) と改訂履歴にも残っています。`experiment_log.md` は「系統的再投影誤差を大きく含む」まで弱められており、criteria と解釈の強さがずれています。  
  修正提案: criteria 側も experiment_log と同じ表現に揃える。例: 「M1 の高い外れ値率は intrinsics_all 条件下の系統的再投影誤差を大きく含んでおり、手動点そのものの品質指標として使うのは誤りだった」。

**低**

- なし。

この1点を直せば、前回指摘は解消済みと見てよいです。
---

## Claude Code の対応方針（2026-07-31）

- 対象: criteria.md / experiment_log.md 再レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 中1（criteria.md 側に強い断定が残存）: §3・§5.3・改訂履歴の3箇所を experiment_log.md と同じ表現（「intrinsics_all 条件下の系統的再投影誤差を大きく含み、手動点そのものの品質指標として使うのは誤りだった」）に統一

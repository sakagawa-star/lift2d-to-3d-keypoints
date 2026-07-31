結論: まだ lock には早いです。方式の方向性は M3 の原因調査と整合していますが、判定を動かし得る穴が残っています。

**高**

1. 既存成功系列の正確度劣化を検出できない  
[criteria_m3_1.md:33](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:33) では救済系列だけを §5.3 p95 で基準ポーズと比較しています。一方、M3 成立済み12系列は [criteria_m3_1.md:34](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:34) で「M3 解から 2cm/0.3°以内」だけです。  
この条件だと、既存成功カメラの P0 が §5.3 p95 の外へ drift しても、P0/P1/P2 が一緒に動けば同一解条件も通り、M3-1 が Go になり得ます。サンプリングで測れるのは精度であって正確度ではない、という `experiment_log.md` の限界明示とも噛み合っていません。

修正提案: 判定対象5カメラすべてについて、少なくとも P0 最終解は §5.3 p95 以内を必須にする。より明確には、15系列すべての最終解を各カメラの基準ポーズと §5.3 p95 以内で照合し、その上で同一解 1cm/0.1° を見る。

2. `N_s=20` と `f_c≥0.7` が実測導出になっていない  
[criteria_m3_1.md:11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:11), [criteria_m3_1.md:18](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:18) の主要数値は固定されていますが、`N_s=20` の根拠がなく、`f_c≥0.7` も「大半」「保守側」という定性的説明に留まっています。これは今回の確認点 (1) に対して未充足です。

修正提案: lock 前に M3-1a 相当のキャリブレーション、または既存 M3 履歴からの明示的な導出を追加する。最低限、`N_s=20` は推定誤差・再現性・計算量から、`f_c≥0.7` は「14/20 未満を不合意とする」統計的/実測的理由から固定してください。

**中**

1. `r_pos=2cm` の根拠説明に数値矛盾がある  
[criteria_m3_1.md:17](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:17) は `r_pos=2.0cm` が §5.3 p95 `1.84〜2.55cm` 以下だと書いていますが、cam41520554 の p95 は 1.84cm なので事実として 2.0cm は超えています。また、M3 の「1歩の更新量」から、20サンプルのクラスタ半径を直接導いたとは言い切れません。

修正提案: 全カメラで p95 以下を主張するなら `r_pos≤1.84cm` にするか、カメラ別 `r_pos=min(2.0cm, p95_pos_cam)` にする。2cm を使うなら「一部 p95 を超えるが、最終解の p95 判定で正確度を別途担保する」と明記する。

2. 多峰検出が Go/No-Go に効かない  
[criteria_m3_1.md:20](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:20) で第2クラスタ占有率 `≥0.2` を二峰と分類しますが、[criteria_m3_1.md:31](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_1.md:31) 以降の Go 条件には入っていません。`f_c=0.70`、第2クラスタ `0.30` のような明確な二峰でも受理され得ます。

修正提案: 判定系列で第2クラスタ占有率 `≥0.2` なら M3-1 は No-Go、または少なくとも F2 分岐扱いにする、と固定してください。多峰を「診断値」だけに留めると、偽陽性チェックとして弱いです。

**低**

なし。
---

## Claude Code の対応方針（2026-07-31）

- 対象: criteria_m3_1.md 初回レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 高1（既存成功系列の正確度劣化を検出できない）: 判定条件2として「全15系列の最終解を基準ポーズと §5.3 p95 で照合」を必須化
- 高2（N_s=20・f_c≥0.7 が実測導出でない）: N_s は標準誤差（σ/√n ≤ 3.5mm = 同一解閾値の1/3）・f_c 分解能・計算量から、f_c は「1歩2cm超の観測頻度 1/49（M3 の k≥1 全49反復中1件）の15倍マージン、14/20 未満は不合意」として明示
- 中1（r_pos=2cm と 554 の p95=1.84cm の矛盾）: 事実を明記し、「クラスタ半径は近傍の定義であり正確度は p95 照合で別途担保」と整理（codex 提案の後者案）
- 中2（多峰が Go/No-Go に効かない）: 「判定15系列に第2クラスタ占有率 ≥0.2 が現れないこと」を Go 条件5（単峰性）に組み込み。現れたら F2 分岐として No-Go

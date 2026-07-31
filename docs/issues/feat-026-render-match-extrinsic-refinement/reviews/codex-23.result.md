再レビュー結果です。

前回指摘4件のうち、3件は解消されています。§5.2 の P0/P1/P2 全成立条件、§5.6/§6 の SIFT baseline 限定、§5.4 の独立検証から手動点再投影整合への格下げはいずれも前回の問題を潰せています。実品質を測れるという過大主張も残っていません。

**高**

- [criteria.md §5.3](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:74) の一致性閾値が、まだ数値として lock されていません。Config_scene の K・歪みで再計測する方針は正しいですが、現状は「M3-0a で後から p95 を埋めて再lock」と書かれているだけです。この文書単体では M3 Go/No-Go の判定基準が未完成です。  
  修正提案: M3 本実験前に M3-0a を実行し、判定対象3カメラの `位置差 p95 / 角度差 p95` を §5.3 に表で固定する。結果ファイル、改修スクリプト、seed=42、試行数1000も記録し、改訂履歴に「数値確定」として再lock する。そこまで完了して初めて criteria lock と扱うべきです。

**中**

- なし。

**低**

- なし。

結論として、前回の設計上の抜け穴はほぼ解消済みです。ただし §5.3 の実数値が未確定なので、現時点の criteria.md は「lock 前の手順書」であって、まだ lock 文書としては使えません。
---

## Claude Code の対応方針（2026-07-31）

- 対象: criteria.md 再レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892 / 再（codex-22 の続き）
- 前回指摘4件中3件解消を確認。残る高1件（§5.3 の実数値が未確定のままでは lock 文書として未完成）に対応:
  - M3-0a スクリプト `experiments/m3_realquery/m3_0a_resample_p95.py` を作成（M1 スクリプトの内部パラメータ入力を Config_scene.toml に差し替え、n0=10・1000試行・seed=42。他ロジックは M1 と同一）
  - スクリプトを Codex レビュー（codex-24）→ 収束後に実行 → p95 を §5.3 に表で固定し、結果ファイル・スクリプト・seed・試行数を記録して「数値確定」として再lock する

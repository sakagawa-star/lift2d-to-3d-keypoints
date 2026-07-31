再レビュー結果です。スクリプト本体は実行していません。

前回指摘の中2件は解消されています。

**高**

- なし。

**中**

- なし。

**低**

- なし。

確認内容:

- `cv2.setRNGSeed(SEED)` が [main 冒頭](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_resample_p95.py:93)で設定され、summary に OpenCV/NumPy バージョンも記録されるようになっています。
- 判定対象3カメラは `REQUIRED_CAMS` として固定され、対応点不足・基準ポーズ推定失敗・全試行失敗・結果欠落で `RuntimeError` になります。欠けたまま完走する問題は潰れています。

現状、実行前レビューとして致命的な問題は見当たりません。
---

## Claude Code の記録（2026-07-31）

- 対象: m3_0a_resample_p95.py 再レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892 / 再（codex-24 の続き）
- 高・中・低ゼロで収束。M3-0a を実行した
- 実測結果: 全5カメラ p95 = 位置 1.84〜2.55cm / 角度 0.35〜0.46°（M1 参考値の約1/4）。予測（±50%以内）と大きく乖離。照合の結果、M1 の高外れ値率（44〜47%）は intrinsics_all と Config_scene の内部パラメータ不一致による系統誤差が主因と判明（experiments/m3_realquery/experiment_log.md に記録）
- criteria.md を更新: §5.3 数値確定、§3 層別廃止（判定対象5カメラ）、§6 Go=3/5、§5.4 名簿差し替え、§5.2 根拠整理 → codex-26 で再レビュー

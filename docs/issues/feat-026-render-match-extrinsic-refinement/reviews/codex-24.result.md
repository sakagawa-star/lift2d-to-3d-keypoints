実装レビュー結果です。スクリプト本体は実行していません。

結論として、criteria §5.3 の主要条件、つまり Config_scene の K・歪み、`n0=10`、1000試行、`seed=42`、M1 と同じ `estimate_extrinsic` と `pose_diff` の流用は概ね一致しています。M1 との差分も、判定ロジック上は意図範囲に収まっています。

**高**

- なし。

**中**

- [m3_0a_resample_p95.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_resample_p95.py:91) は NumPy の抽出乱数だけを `seed=42` にしていますが、実際の PnP 経路は [cv2.solvePnPRansac](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:335) を使います。OpenCV 側の RNG を固定していないため、「seed固定の決定的計算」としては弱いです。  
  修正提案: `cv2.setRNGSeed(SEED)` を PnP 実行前に明示的に呼ぶ。可能なら summary に OpenCV version も記録する。

- [m3_0a_resample_p95.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_0a_resample_p95.py:110) は対応点不足、基準ポーズ推定失敗、全試行失敗でも `continue` して最終的に結果ファイルを書けます。閾値確定用スクリプトとしては、判定対象3カメラの p95 が欠けたまま成功扱いになる余地があります。  
  修正提案: 判定対象 `cam05520126`, `cam05520129`, `cam41520556` を必須カメラとして定義し、いずれかで対応点不足・全点PnP失敗・全試行失敗が起きたら `RuntimeError` で停止する。参考2カメラは警告記録でも可。

**低**

- なし。

上の2点を直せば、実行前レビューとしては問題ありません。THRESHOLDS 出力列を削った差分は p95 確定値には影響しないので、致命扱いしません。
---

## Claude Code の対応方針（2026-07-31）

- 対象: m3_0a_resample_p95.py 実行前レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892 / 初回（スクリプト）
- 中1（OpenCV RNG 未固定）: main() 冒頭で `cv2.setRNGSeed(SEED)` を追加。summary に OpenCV / NumPy のバージョンを記録
- 中2（判定対象カメラ欠落でも完走）: REQUIRED_CAMS（判定対象3カメラ）を定義し、対応点不足・基準ポーズ失敗・全試行失敗・結果欠落のいずれでも RuntimeError で停止するよう修正

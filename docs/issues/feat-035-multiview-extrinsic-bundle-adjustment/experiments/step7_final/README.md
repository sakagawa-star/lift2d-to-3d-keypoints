# feat-035 Step 7 反映後の再検証記録（最終採用値）

実施日: 2026-08-25 / ステータス: ユーザー最終手動テスト待ち

## 最終採用値（p1_param_sweep + 改訂1。2026-08-25 ユーザー承認済み）

- HUBER_PX=2.0（既定値維持。当初 1.0 採用は W_CROSS=2.0 との相互作用で BA 停滞のため差し戻し）
- W_CROSS=2.0（W_ANCHOR=1.0）
- 交会角重み無効化（w_k=1。crossing_angle_weight 関数・テストは温存）
- EPI_TOL_PX=3.0 / N_PAIR_MIN=50 / HOLDOUT_RATIO=0.2 は既定値維持
- 経緯の全量: `../p1_param_sweep/experiment_log.md`（走査・想定外事象・分解診断・改訂決定）

## 反映後の再検証（roadmap Step 7）

1. `uv run pytest -v`: **513 passed / 1 skipped**（既存スキップ）。`tests/results/feat-035_test_result.txt`
2. end-to-end 実行（inuyama 8台、--overwrite）: 正常終了
   - Stage D: **status=2（収束）**、nfev=519、コスト 67904→54606
   - **結合検証成立**: ホールドアウト中央値 **0.8684px** / RMSE 1.1314px（分解診断の実測と完全一致。走査の単独最良値 0.8774px を下回る全測定中の最良値）
   - フォールバック発動 0/8（最大悪化 cam41520554 の +0.12px < 0.3px）
   - 初期→最終ポーズ変化: 位置 0.01〜0.15cm / 回転 0.008〜0.065°
3. 出力TOML（multiview_all8.toml）の `render_keypoints.py --near-plane 0.25 --no-keypoints --distort` 読込レンダ: 8台全て成功

## レンダ vs 実写ずれ（Step 3M と同一測定法での参考値）

構造エッジの <5px モード中央値は全台 1〜2px 水準（Step 3M / Step 6 と同等。金網部の見かけの大ずれは既報の測定誤マッチ。p0_manual_all8/experiment_log.md §2 参照）。

| カメラ | 採択点 | 全中央値 | <5px割合 | <5px内中央値 |
|---|---|---|---|---|
| cam05520125 | 338 | 1.4px | 73.1% | 1.00px |
| cam05520126 | 350 | 16.0px | 29.7% | 1.00px |
| cam05520128 | 312 | 2.8px | 58.7% | 2.00px |
| cam05520129 | 360 | 5.4px | 21.7% | 1.41px |
| cam41520554 | 334 | 20.0px | 20.4% | 1.00px |
| cam41520556 | 303 | 1.4px | 55.4% | 1.00px |
| cam41520557 | 366 | 9.2px | 47.8% | 1.00px |
| cam41520558 | 314 | 1.4px | 79.0% | 1.00px |

## 成果物

- `renders/still_<cam>.png` / `renders/overlay_<cam>.png`: 最終TOMLによるレンダと重ね合わせ（8台）
- 本体出力: `phase0/data/inuyama/multiview_all8.toml` / `multiview_all8_report.txt`（git管理外）

## 次のアクション

- ユーザー最終手動テスト（roadmap Step 7 末尾）: overlay 8枚 + 最終レポートの確認。合格後にのみ Step 8（完了処理）へ進む

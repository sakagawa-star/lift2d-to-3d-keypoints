# bug-003: estimate_camera_params.py 通常モードの入力・収束ガード不足

## ステータス

Closed（2026-07-27 発番、同日修正完了。手動テストで改修前後の結果一致と点数不足時のエラー表示を確認済み）

## 概要

`phase0/estimate_camera_params.py` のコードレビュー（Claude Code、REVIEW_CRITERIA.md 準拠）で検出された重要度「高・中」の指摘4件を修正する。

- **H-1（高）**: 通常モード（K未知）に最小点数チェックがなく、基準点が少ないと `least_squares(method='lm')` の `ValueError`（残差数 < 変数数）または `cv2.solvePnP` の `cv2.error` でトレースバック終了する
- **M-1（中）**: 定数 `MIN_POINTS_NO_DIST`（= 6）が未使用（H-1 のチェック実装漏れの兆候）
- **M-2（中）**: 全 `least_squares` 呼び出しで収束判定（`result.status`）を確認していない
- **M-3（中）**: config.yaml / 内部パラメータTOML の必須キー欠落時に生の `KeyError` でトレースバック終了する

## 再現手順

### H-1
1. 基準点が5点以下しかマッチしない `points_2d.csv` を用意する（例: 対象カメラの行を4点分だけ残す）
2. `uv run python estimate_camera_params.py config.yaml` を phase0/ で実行する
3. 日本語エラーメッセージではなく `ValueError`（点数4〜5）または `cv2.error`（点数0〜3）のトレースバックで終了する

### M-3
1. `target_camera` 行を削除した config.yaml を用意する
2. 同スクリプトを実行すると `KeyError: 'target_camera'` のトレースバックで終了する

## 関連ドキュメント

- 修正計画: `investigation.md`
- レビュー結果: `reviews/`

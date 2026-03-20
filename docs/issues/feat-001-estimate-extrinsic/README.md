# feat-001: 内部パラメータ既知での外部パラメータ推定（Stage 1）

## Status: Closed

## 概要

K既知のカメラに対して外部パラメータ（R, t）のみを推定するミニマムツール `estimate_extrinsic.py` を新規作成する。

## 背景

現在の `estimate_camera_params.py` はカメラの内部パラメータ（K, 歪み係数）が未知であることを前提としている。チェスボードやChArUcoで内部パラメータを推定済みのカメラに対して、外部パラメータ（R, t）のみを求める機能がない。

K既知であれば推定変数が6個（R, tのみ）に減り、精度・安定性が大幅に向上する。

## 主な仕様

- Calib_scene.toml 形式のファイルから内部パラメータ（K, 歪み係数）を読み込む
- `cv2.solvePnPRansac` → `cv2.solvePnP(ITERATIVE)` の二段構えで外部パラメータを推定
- 基準点4〜5点: RANSACなし、3点以下: エラー終了
- 出力: Calib_scene.toml 形式（標準出力）
- 再投影誤差（RMSE）で評価

## テスト用データ

- 内部パラメータTOML: `phase0/data/ufukui/05520125_intrinsics.toml`（カメラ名: `cam_05520125`）

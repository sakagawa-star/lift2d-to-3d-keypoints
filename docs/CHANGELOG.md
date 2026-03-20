# CHANGELOG

## リリース履歴

### 2026-03-21

- **feat-004**: gsplatバッチレンダリングパイプライン（`phase4/render.py`）
  - PLYファイルとカメラポーズJSONを受け取り、gsplat（CUDA）でフレームごとにPNG画像を出力
  - Blender→OpenCV座標系変換、PLY属性の活性化関数変換（sigmoid, exp, normalize）
  - `--rotate-z90` オプション: Blenderアドオン旧バージョンのバグによるZ軸90度ずれを補正
  - `--background` オプション: 背景色指定

### 2026-03-20

- **feat-001**: 内部パラメータ既知での外部パラメータ推定ツール `estimate_extrinsic.py` を新規作成（Stage 1）
  - Calib_scene.toml形式のTOMLファイルから内部パラメータを読み込み、R, tのみを推定
  - solvePnPRansac + solvePnP(ITERATIVE) の二段構えで外れ値検出・精密化
  - 依存関係に `tomli` を追加

- **feat-002**: `estimate_camera_params.py` に広角レンズ用8係数歪みモデルを追加
  - `--wide` オプションで8係数（k1, k2, p1, p2, k3, k4, k5, k6）の歪みモデルを使用可能に
  - TOML/CSV出力の8係数対応、solvePnP初期値推定のエラーハンドリング追加

- **feat-003**: `estimate_extrinsic.py` を `estimate_camera_params.py` に統合（Stage 2）
  - `--intrinsic-toml` オプションでK既知モード（R, tのみ推定）を追加
  - 重複関数を `common.py` に切り出し
  - `estimate_extrinsic.py` を削除


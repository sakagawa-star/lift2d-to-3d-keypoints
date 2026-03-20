# feat-001 要求仕様書: 内部パラメータ既知での外部パラメータ推定（Stage 1）

## 目的

カメラの内部パラメータ（K行列、歪み係数）が既知の場合に、外部パラメータ（回転R、並進t）のみを推定するスタンドアロンツール `estimate_extrinsic.py` を新規作成する。

## 背景

- 現在の `estimate_camera_params.py` は内部パラメータ未知を前提とし、全パラメータを同時推定する
- チェスボードやChArUcoで内部パラメータが推定済みのカメラでは、K既知として外部パラメータのみを推定する方が精度・安定性が高い（推定変数が6個に減るため）

## 機能要件

### FR-1: 内部パラメータの読み込み

- Calib_scene.toml 形式のファイルから内部パラメータを読み込む
- TOMLファイルのパスはコマンドライン引数で指定する
- TOMLファイル内に複数カメラのパラメータが記述されている場合、config.yaml の `target_camera` に一致するセクションのみを使用する
- 読み込む項目: `matrix`（K行列: fx, fy, cx, cy）、`distortions`（歪み係数: k1, k2, p1, p2、オプションでk3）、`size`（画像サイズ）
- TOML読み込みには `tomli` ライブラリを使用する

### FR-2: 外部パラメータの推定

- 推定手法は基準点数に応じて切り替える:
  - **6点以上**: `cv2.solvePnPRansac` で外れ値を検出・除外した後、inlier点のみで `cv2.solvePnP(flags=SOLVEPNP_ITERATIVE, useExtrinsicGuess=True)` で精密化
  - **4〜5点**: RANSACなし。`cv2.solvePnP(flags=SOLVEPNP_ITERATIVE)` のみ実行
  - **3点以下**: エラー終了（エラーメッセージを出力して終了コード1）
- エラー条件:
  - `solvePnPRansac` または `solvePnP` が失敗（`success=False`）した場合はエラー終了する
  - RANSAC後のinlier数が4点未満の場合はエラー終了する（外れ値が多すぎて推定不能）

### FR-3: 出力

- Calib_scene.toml 形式で標準出力に出力する
- 出力内容: 入力の内部パラメータ（K, 歪み係数）＋ 推定した外部パラメータ（rotation, translation）
- 出力形式は既存の `estimate_camera_params.py` と同じフォーマットに従う
- 数値はPythonデフォルト精度で出力する（既存の `estimate_camera_params.py` と同じ方式）
- camera_params.csv 形式の出力は本ツールでは行わない

### FR-4: 検証情報の表示

- 再投影誤差（RMSE）を全基準点（inlier・outlier問わず）で計算・表示する
- 各基準点ごとの再投影誤差を表示する（outlier点にはマークを付ける）
- RANSAC使用時: inlier/outlier の分類結果を表示する（どの基準点がoutlierかを名前付きで表示）

### FR-5: コマンドラインインターフェース

```
python estimate_extrinsic.py <config.yaml> <intrinsic.toml>
```

- 第1引数: config.yaml（既存フォーマット。`target_camera`, `points_3d`, `points_2d` を使用）
- 第2引数: 内部パラメータTOMLファイル
- config.yaml の `image_width`, `image_height` はTOMLの `size` から取得するため不要（指定されていても無視）
- config.yaml の `camera_params` は本ツールでは使用しない

## 非機能要件

### NF-1: 依存関係

- `tomli` を pyproject.toml の dependencies に追加する
- Python 3.11以降では標準ライブラリの `tomllib` が使えるが、Python 3.10対応のため `tomli` を使用する

### NF-2: 既存コードへの影響

- `estimate_camera_params.py` は一切変更しない
- `phase0_verification.py` は一切変更しない

## 入力データ仕様

### config.yaml（既存フォーマット）

```yaml
target_camera: cam_05520125
points_3d: kijunten_locations.csv
points_2d: points_2d.csv
```

### 内部パラメータTOMLファイル（Calib_scene.toml形式）

```toml
[cam_05520125]
name = "cam_05520125"
size = [1920.0, 1080.0]
matrix = [[1177.3399, 0.0000, 956.7042], [0.0000, 1177.8163, 495.7718], [0.0000, 0.0000, 1.0000]]
distortions = [-0.0536, 0.0983, -0.0054, -0.0027]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false

[metadata]
adjusted = false
error = 0.3356
```

- `metadata` セクションは無視する
- `rotation`, `translation`, `fisheye` は読み込まない（本ツールではrotation/translationを推定する）

### 3D基準点CSV、2D画像座標CSV

既存の `estimate_camera_params.py` と同じフォーマット（CLAUDE.md参照）。

## 出力データ仕様

### 標準出力（Calib_scene.toml形式）

```toml
[cam_05520125]
name = "cam_05520125"
size = [1920.0, 1080.0]
matrix = [[1177.3399, 0.0, 956.7042], [0.0, 1177.8163, 495.7718], [0.0, 0.0, 1.0]]
distortions = [-0.0536, 0.0983, -0.0054, -0.0027]
rotation = [<推定値>, <推定値>, <推定値>]
translation = [<推定値>, <推定値>, <推定値>]
fisheye = false
```

## テスト

### テストデータ

- 内部パラメータTOML: `phase0/data/ufukui/05520125_intrinsics.toml`（カメラ名: `cam_05520125`）
- config.yaml、3D基準点CSV、2D画像座標CSVはテスト時に別途用意する必要がある

### テスト項目

1. TOMLファイルから正しく内部パラメータを読み込めること
2. 複数カメラのTOMLから `target_camera` に一致するカメラのみ抽出できること
3. `metadata` セクションが無視されること
4. 6点以上の場合にRANSAC + ITERATIVE精密化が実行されること
5. 4〜5点の場合にITERATIVEのみが実行されること
6. 3点以下の場合にエラー終了すること
7. Calib_scene.toml 形式で正しく出力されること
8. 再投影誤差が表示されること

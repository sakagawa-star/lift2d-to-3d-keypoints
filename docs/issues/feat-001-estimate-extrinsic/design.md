# feat-001 機能設計書: 内部パラメータ既知での外部パラメータ推定（Stage 1）

## 概要

`phase0/estimate_extrinsic.py` を新規作成する。既存スクリプトの構造（データ読み込み→推定→結果出力）を踏襲しつつ、推定ロジックを「K既知でR,tのみ推定」に特化させる。

## ファイル構成

### 新規作成

- `phase0/estimate_extrinsic.py` — メインスクリプト

### 変更

- `pyproject.toml` — `tomli` を dependencies に追加

### 変更しないファイル

- `phase0/estimate_camera_params.py`
- `phase0/phase0_verification.py`
- `phase0/convert_toml_to_csv.py`

## モジュール設計

`estimate_extrinsic.py` は以下の関数で構成する。既存スクリプトと同様にシングルファイル構成とする（共通モジュール化はStage 2で行う）。

### データ読み込み関数

#### `load_yaml_simple(yaml_path: str) -> dict`
- 既存スクリプトと同じ実装（簡易YAMLパーサー）

#### `load_intrinsic_toml(toml_path: str, camera_name: str) -> dict`
- `tomli` でTOMLファイルを読み込む
- `camera_name` に一致するセクションを検索し、内部パラメータを返す
- 一致するセクションがない場合はエラー終了
- 戻り値:
  ```python
  {
      'K': np.ndarray,        # 3x3 カメラ行列
      'dist': np.ndarray,     # 歪み係数 (4要素 or 5要素)
      'image_width': int,     # 画像幅
      'image_height': int,    # 画像高さ
  }
  ```
- TOMLセクション内のキーマッピング:
  - `matrix` → K行列（`[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]` 形式のネスト配列）
  - `distortions` → 歪み係数配列
  - `size` → `[width, height]`（`tomli` は float としてパースするため、`int()` に変換して返す）
- `metadata` セクションはスキップする（カメラ名と一致しないため自然にスキップされる）

#### `load_points_3d(csv_path: str) -> dict`
- 既存スクリプトと同じ実装

#### `load_points_2d(csv_path: str, camera_name: str) -> dict`
- 既存スクリプトと同じ実装

#### `match_points(points_3d: dict, points_2d: dict) -> tuple`
- 既存スクリプトと同じ実装

### 推定関数

#### `estimate_extrinsic(points_3d: np.ndarray, points_2d: np.ndarray, K: np.ndarray, dist: np.ndarray, point_names: list[str]) -> dict`

外部パラメータ推定のメインロジック。

**引数:**
- `points_3d`: マッチ済み3D点 (N, 3)
- `points_2d`: マッチ済み2D点 (N, 2)
- `K`: カメラ行列 (3, 3)
- `dist`: 歪み係数
- `point_names`: 基準点名リスト

**戻り値:**
```python
{
    'rvec': np.ndarray,          # 回転ベクトル (3,)
    'tvec': np.ndarray,          # 並進ベクトル (3,)
    'inliers': list[str],        # inlier基準点名
    'outliers': list[str],       # outlier基準点名
    'used_ransac': bool,         # RANSACを使用したか
}
```

**アルゴリズム:**

```
num_points = len(points_3d)

if num_points <= 3:
    エラー終了（"基準点が3点以下です。最低4点必要です。"）

if num_points >= 6:
    # Step 1: RANSAC で外れ値検出
    success, rvec, tvec, inliers_idx = cv2.solvePnPRansac(
        points_3d, points_2d, K, dist,
        iterationsCount=1000,
        reprojectionError=8.0,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not success or inliers_idx is None:
        エラー終了（"solvePnPRansac が失敗しました"）

    # inlier/outlier を分類
    inlier_mask を inliers_idx から生成
    inlier_3d, inlier_2d = inlierのみ抽出

    if len(inlier_3d) < 4:
        エラー終了（"inlier数が4点未満です（{len(inlier_3d)}点）。外れ値が多すぎます。"）

    # Step 2: inlier点で ITERATIVE 精密化
    success, rvec, tvec = cv2.solvePnP(
        inlier_3d, inlier_2d, K, dist,
        rvec=rvec, tvec=tvec,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not success:
        エラー終了（"solvePnP(ITERATIVE) 精密化が失敗しました"）

else:  # 4〜5点
    # RANSACなし、SQPNP を使用（SOLVEPNP_ITERATIVE は最低6点必要なため）
    success, rvec, tvec = cv2.solvePnP(
        points_3d, points_2d, K, dist,
        flags=cv2.SOLVEPNP_SQPNP
    )

    if not success:
        エラー終了（"solvePnP が失敗しました"）

    全点をinlierとする
```

### 評価関数

#### `compute_reprojection_errors(points_3d: np.ndarray, points_2d: np.ndarray, K: np.ndarray, dist: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> tuple[float, np.ndarray]`

- `cv2.projectPoints` で再投影し、各点の誤差とRMSEを計算
- RANSAC使用時もinlier・outlier問わず**全点**で計算する（outlier点の誤差も確認可能にするため）
- 戻り値: `(rmse, per_point_errors)`
  - `rmse`: float（全点のRMSE）
  - `per_point_errors`: np.ndarray（各点のL2誤差）

### 出力関数

#### `print_toml_output(camera_name: str, image_width: int, image_height: int, K: np.ndarray, dist: np.ndarray, rvec: np.ndarray, tvec: np.ndarray)`

- Calib_scene.toml 形式で標準出力に出力
- 数値はPythonデフォルト精度で出力する（既存の `estimate_camera_params.py` と同じ方式）
- 内部パラメータは入力TOMLからの「パススルー」だが、numpy配列経由で出力するため数値フォーマットが入力と異なる場合がある（例: 入力 `0.0000` → 出力 `0.0`）。これは許容する
- camera_params.csv 形式の出力は行わない

### メイン関数

#### `run_estimation(config_path: str, toml_path: str)`

処理フロー:
1. `load_yaml_simple` で config.yaml を読み込み → `target_camera`, `points_3d`, `points_2d` を取得
2. CSVファイルのパスは config.yaml の親ディレクトリからの相対パスとして解決する（既存の `estimate_camera_params.py` と同じ方式: `config_dir / config['points_3d']`）
3. `load_intrinsic_toml` で TOML を読み込み → K, dist, image_size を取得（TOMLパスはコマンドライン引数で渡されたパスをそのまま使用）
4. `load_points_3d`, `load_points_2d` でCSVを読み込み
5. `match_points` で3D-2D点をマッチング
6. 情報表示（カメラ名、基準点数、使用する内部パラメータ等）
7. `estimate_extrinsic` でR, t推定
8. RANSAC結果表示（inlier/outlier）
9. `compute_reprojection_errors` で再投影誤差を計算・表示（全点対象）
10. `print_toml_output` でTOML形式の結果を出力

#### `main()`

- `argparse` でコマンドライン引数をパース
  - 第1引数 `config`: config.yaml のパス
  - 第2引数 `intrinsic_toml`: 内部パラメータTOMLファイルのパス
- ファイル存在確認
- `run_estimation` を呼び出し

## TOML読み込みの実装詳細

`tomli` は TOML を Python の dict にパースする。Calib_scene.toml の `matrix` フィールドはネスト配列 `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]` のため、`tomli` はこれを `list[list[float]]` としてパースする。

```python
import tomli

with open(toml_path, 'rb') as f:
    data = tomli.load(f)

# data['cam_05520125']['matrix'] → [[1177.3399, 0.0, 956.7042], [0.0, 1177.8163, 495.7718], [0.0, 0.0, 1.0]]
# → np.array(data['cam_05520125']['matrix']) で 3x3 の ndarray になる
```

既存の `convert_toml_to_csv.py` の `parse_toml_simple` は正規表現ベースの簡易パーサーで、数値をフラットに抽出するため matrix が9要素の1次元配列になる。本ツールでは `tomli` を使うため、この問題は発生しない。

## コマンドライン仕様

```
usage: estimate_extrinsic.py [-h] config intrinsic_toml

内部パラメータ既知での外部パラメータ推定

positional arguments:
  config          設定ファイル (config.yaml)
  intrinsic_toml  内部パラメータファイル (Calib_scene.toml形式)

使用例:
    python estimate_extrinsic.py data/config.yaml data/ufukui/05520125_intrinsics.toml
```

## 標準出力の構成

```
============================================================
外部パラメータ推定 (K既知): cam_05520125
============================================================
設定ファイル: data/config.yaml
内部パラメータ: data/ufukui/05520125_intrinsics.toml
3D座標: data/kijunten_locations.csv
2D座標: data/points_2d.csv
画像サイズ: 1920 x 1080

[内部パラメータ (TOML読み込み)]
fx: 1177.34
fy: 1177.82
cx: 956.70
cy: 495.77
歪み係数: [-0.0536, 0.0983, -0.0054, -0.0027]

基準点数: XX点
  - 基準_01
  - 基準_02
  ...

============================================================
推定結果
============================================================

[推定手法]
solvePnPRansac → solvePnP(ITERATIVE) 精密化

[RANSAC結果]
inlier: XX点 / 全XX点
  inlier:  基準_01, 基準_02, ...
  outlier: 基準_XX

[外部パラメータ]
rvec: [X.XXXXXX, X.XXXXXX, X.XXXXXX]
tvec: [X.XXXXXX, X.XXXXXX, X.XXXXXX]

[再投影誤差]
RMSE: X.XX pixels
評価: ✓ 優秀
（閾値: < 5px 優秀, < 10px 良好, < 20px 許容範囲, >= 20px 要確認。既存 estimate_camera_params.py と同じ）

[各点の再投影誤差]
  基準_01: X.XX px ✓
  基準_02: X.XX px ✓
  基準_XX: X.XX px ✗ [outlier]
  ...

============================================================
Calib_scene.toml 形式
============================================================

[cam_05520125]
name = "cam_05520125"
size = [1920.0, 1080.0]
matrix = [[1177.3399, 0.0, 956.7042], [0.0, 1177.8163, 495.7718], [0.0, 0.0, 1.0]]
distortions = [-0.0536, 0.0983, -0.0054, -0.0027]
rotation = [X.XXXXXX, X.XXXXXX, X.XXXXXX]
translation = [X.XXXXXX, X.XXXXXX, X.XXXXXX]
fisheye = false
```

## テスト設計

### テストファイル

`tests/test_estimate_extrinsic.py`

### テスト方針

合成データ（既知のK, R, tから `cv2.projectPoints` で2D点を生成）を使ってテストする。これにより実データのCSVファイルに依存しないテストが可能。

### テストケース

#### T-1: TOML読み込み

- 正常なTOMLから K, dist, size を正しく読み込めること
- 複数カメラのTOMLから指定カメラのみ抽出できること
- 存在しないカメラ名の場合にエラーが発生すること
- 歪み係数が5要素（k3あり）の場合も読み込めること

#### T-2: 推定（6点以上、RANSAC使用）

- 合成データ（ノイズなし）でR, tが正しく推定されること（再投影誤差 < 1.0 px）
- 1点を外れ値として大きくずらした場合、その点がoutlierとして検出されること

#### T-3: 推定（4〜5点、RANSACなし）

- 4点の合成データでR, tが推定できること
- 5点の合成データでR, tが推定できること

#### T-4: 推定（3点以下、エラー）

- 3点の場合にエラー終了すること（SystemExit or 例外）
- 0点（マッチなし）の場合にエラー終了すること

#### T-5: 再投影誤差計算

- ノイズなし合成データで再投影誤差が十分小さいこと（< 1.0 px）

#### T-6: TOML出力フォーマット

- 出力が Calib_scene.toml 形式として正しいこと（`tomli` でパース可能であること）

### 合成データ生成方法

```python
# 既知のパラメータ
K = np.array([[1177.34, 0, 956.70], [0, 1177.82, 495.77], [0, 0, 1]])
dist = np.array([-0.0536, 0.0983, -0.0054, -0.0027])
rvec_true = np.array([0.1, -0.2, 0.3])
tvec_true = np.array([1.0, -0.5, 5.0])

# 3D基準点（適当な座標）
points_3d = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.5, 0.5, 0.5],
    [0.0, 0.0, 1.0],
    ...
], dtype=np.float64)

# 2D投影点を計算
points_2d, _ = cv2.projectPoints(points_3d, rvec_true, tvec_true, K, dist)
points_2d = points_2d.reshape(-1, 2)
```

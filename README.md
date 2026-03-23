# lift2d-to-3d-keypoints

2D-3D点対応によるカメラパラメータ推定ツール

## 主な機能

- 2D-3D点対応によるカメラ内部・外部パラメータ推定
- 歪み係数の推定（4係数 / 5係数 / 8係数広角対応）
- 内部パラメータ既知（K既知）での外部パラメータのみの推定
- 複数カメラの一括推定（K既知モード）
- 三角測量による外部パラメータ検証
- Ground Truthとの比較検証
- Calib_scene.toml / camera_params.csv 形式での結果出力

## Setup

```
uv sync
```

## Run

すべてのスクリプトは `phase0/` ディレクトリで実行する。

```bash
cd phase0

# 標準（4係数歪みモデル）
uv run python estimate_camera_params.py data/config.yaml

# 主点固定（cx, cyを画像中心に固定）
uv run python estimate_camera_params.py data/config.yaml --fix-center

# 主点固定 + k3（5係数）
uv run python estimate_camera_params.py data/config.yaml --fix-center --k3

# 広角レンズ（8係数歪みモデル）
uv run python estimate_camera_params.py data/config.yaml --wide

# 広角 + 主点固定
uv run python estimate_camera_params.py data/config.yaml --wide --fix-center

# K既知モード（内部パラメータTOMLからR, tのみ推定）
uv run python estimate_camera_params.py data/config_lab2.yaml --intrinsic-toml data/ufukui/cam05520125_intrinsics.toml

# 複数カメラ一括推定（K既知モード、結果をTOMLファイルに出力）
uv run python estimate_camera_params.py data/config_lab2.yaml \
  --intrinsic-toml data/ufukui/intrinsics_all.toml \
  --output data/ufukui/extrinsic_all.toml

# 三角測量による外部パラメータ検証
uv run python verify_triangulation.py data/config_lab2.yaml data/ufukui/extrinsic_all.toml

# 推定結果の検証（Ground Truth比較）
uv run python phase0_verification.py data/config.yaml

# TOML→CSV変換
uv run python convert_toml_to_csv.py
```

### 推定モード

| オプション | 歪み係数 | 主点 |
|---|---|---|
| （なし） | k1, k2, p1, p2 | 推定 |
| `--fix-center` | k1, k2, p1, p2 | 画像中心に固定 |
| `--fix-center --k3` | k1, k2, p1, p2, k3 | 画像中心に固定 |
| `--wide` | k1, k2, p1, p2, k3, k4, k5, k6 | 推定 |
| `--wide --fix-center` | k1, k2, p1, p2, k3, k4, k5, k6 | 画像中心に固定 |
| `--intrinsic-toml` | TOML読み込み（K既知、R,tのみ推定） | TOML読み込み |

### レンズと推奨オプション

| レンズ | 画角 | 推奨オプション |
|---|---|---|
| 標準レンズ | ~60° | （オプションなし） |
| 広角レンズ | 60-100° | `--wide` |
| 魚眼レンズ | >120° | 別途fisheyeモデルが必要 |

## 3D基準点CSVの生成

Blender上で `phase0/blender/mk_points_3d.py` を実行する。Blenderファイル内の「基準点」コレクションから `基準_NNN` にマッチするオブジェクトの座標を抽出し、`kijunten_locations.csv` をBlenderファイルと同じディレクトリに出力する。

## データフォーマット

### ファイル構成

```
phase0/data/
├── config_*.yaml          # 設定ファイル（カメラ別）
├── kijunten_locations*.csv # 3D座標（全カメラ共通）
├── points_2d*.csv          # 2D座標（全カメラ、縦持ち）
├── camera_params.csv       # カメラパラメータ（検証用）
└── ufukui/                 # 内部パラメータTOML（K既知モード用）
    └── *_intrinsics.toml
```

### kijunten_locations.csv

```csv
ObjectName,X,Y,Z
基準_01,-0.0199,-0.2968,-0.1913
基準_02,...
```

### points_2d.csv（行の順番自由）

```csv
ObjectName,camera_name,X,Y
基準_01,cam01,780,913
基準_02,cam01,1877,483
基準_01,cam02,523,845
...
```

### config.yaml

```yaml
target_camera: cam01
camera_params: camera_params.csv
points_3d: kijunten_locations.csv
points_2d: points_2d.csv
image_width: 960
image_height: 540
```

- `camera_params` は検証スクリプト（`phase0_verification.py`）のみ使用
- `image_width`, `image_height` のデフォルトは 960x540
- K既知モード（`--intrinsic-toml`）使用時は画像サイズをTOMLから取得

### 内部パラメータTOML（Calib_scene.toml形式）

```toml
[cam05520125]
name = "cam05520125"
size = [1920.0, 1080.0]
matrix = [[1177.3399, 0.0, 956.7042], [0.0, 1177.8163, 495.7718], [0.0, 0.0, 1.0]]
distortions = [-0.0536, 0.0983, -0.0054, -0.0027]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false
```

## テスト

```bash
uv run pytest -v
```

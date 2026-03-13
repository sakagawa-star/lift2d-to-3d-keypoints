# lift2d-to-3d-keypoints

2D-3D点対応によるカメラパラメータ推定ツール

## Setup
```
uv sync
```

## Run
```
cd phase0

# 通常（主点も推定）
uv run python estimate_camera_params.py data/config.yaml

# 主点固定（cx, cyを画像中心に固定）
uv run python estimate_camera_params.py data/config.yaml --fix-center

# 主点固定 + k3
uv run python estimate_camera_params.py data/config.yaml --fix-center --k3
```

## 3D基準点CSVの生成

Blender上で `phase0/blender/mk_points_3d.py` を実行する。Blenderファイル内の「基準点」コレクションから `基準_NNN` にマッチするオブジェクトの座標を抽出し、`kijunten_locations.csv` をBlenderファイルと同じディレクトリに出力する。

## データフォーマット

### ファイル構成

```
phase0/data/
├── config.yaml            # 設定ファイル
├── kijunten_locations.csv # 3D座標（全カメラ共通）
├── points_2d.csv          # 2D座標（全カメラ、縦持ち）
└── camera_params.csv      # カメラパラメータ（全カメラ）
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

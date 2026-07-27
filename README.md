# lift2d-to-3d-keypoints

2D-3D点対応によるカメラパラメータ推定ツール + 推定結果を用いた 3DGS（gsplat）レンダリング検証

## 主な機能

**phase0: カメラパラメータ推定**

- 2D-3D点対応によるカメラ内部・外部パラメータ推定
- 歪み係数の推定（4係数 / 5係数 / 8係数広角対応、接線歪みゼロ固定）
- 内部パラメータ既知（K既知）での外部パラメータのみの推定
- 複数カメラの一括推定（K既知モード）
- 三角測量による外部パラメータ検証
- Ground Truthとの比較検証
- Calib_scene.toml / camera_params.csv 形式での結果出力

**phase4: gsplatレンダリング**

- 3DGS PLY + カメラポーズJSON のバッチレンダリング（連番PNG/MP4）
- 3DGSレンダリングへの人体キーポイント重ね描き（オクルージョン考慮、C3D入力）
- キャリブ推定結果のGT比較用静止画レンダリング（歪みモデル対応、`--distort`）
- Blenderからのカメラポーズ書き出し（通常カメラ / FPS頭部追従カメラ）
- C3D前処理（NPZ→C3D変換、時間方向平滑化）

## Setup

```bash
# phase0（プロジェクトルート）
uv sync

# phase4（独立した uv 環境）
uv sync --project phase4
```

## Run

phase0 のスクリプトは `phase0/` ディレクトリで実行する。

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

# 接線歪みゼロ固定（p1=p2=0、放射歪みのみ推定。--fix-center/--k3 と併用可、--wide とは併用不可）
uv run python estimate_camera_params.py data/config.yaml --fix-center --zero-tangent

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

# 2D座標を静止画上にプロットして可視化
# --camera: 対象カメラ名（config の target_camera が複数指定の場合は必須）
# --label:  基準点の番号（ObjectNameの数字部分）をラベル表示
uv run python visualize_points_2d.py data/config.yaml --label

# TOML→CSV変換（入力TOML・出力CSVの2引数が必須）
uv run python convert_toml_to_csv.py data/Calib_scene.toml data/camera_params.csv
```

### 推定モード

| オプション | 歪み係数 | 主点 |
|---|---|---|
| （なし） | k1, k2, p1, p2 | 推定 |
| `--fix-center` | k1, k2, p1, p2 | 画像中心に固定 |
| `--fix-center --k3` | k1, k2, p1, p2, k3 | 画像中心に固定 |
| `--wide` | k1, k2, p1, p2, k3, k4, k5, k6 | 推定 |
| `--wide --fix-center` | k1, k2, p1, p2, k3, k4, k5, k6 | 画像中心に固定 |
| `--zero-tangent` | k1, k2（p1, p2 は0固定。`--k3` 併用で k3 も推定、`--fix-center` 併用可、`--wide` とは併用不可） | オプションに従う |
| `--intrinsic-toml` | TOML読み込み（K既知、R,tのみ推定） | TOML読み込み |

### パラメータ範囲チェック（K未知モード）

推定完了後、再投影誤差の直後に `[パラメータ範囲チェック]` セクションが表示され、物理的に妥当な範囲を外れたパラメータに警告が出る（表示のみで終了コード・TOML/CSV 出力は不変。K既知モードは対象外）。

| 項目 | 正常範囲（S = max(画像幅, 画像高さ)） | 適用条件 |
|---|---|---|
| fx, fy | 0.3×S 〜 3.0×S | 常時 |
| fx/fy 比 | 0.9 〜 1.1 | 常時 |
| cx, cy | 画像中心 ± 10% | 主点推定時のみ（`--fix-center` なし） |
| \|k1\|, \|k2\|, \|k3\| | 1.0 以下 | 歪み推定時、`--wide` 以外（k3 は `--k3` 時のみ） |
| \|p1\|, \|p2\| | 0.01 以下 | 歪み推定時、`--zero-tangent` 以外（`--wide` でもチェック） |

警告が出た場合は点対応の誤り・点配置の偏り・パラメータ相殺（過学習）の可能性がある。`--zero-tangent` や `--fix-center` の使用を検討する。

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

## phase4: gsplatレンダリング

phase0 とは独立した uv 環境（`phase4/pyproject.toml`）。スクリプトは `phase4/` ディレクトリで実行する。

### 実行環境の注意

- **CUDA GPU 必須**（render.py / render_keypoints.py）。gsplat の CUDA 拡張が初回実行時に JIT コンパイルされるため、環境変数 **`TORCH_CUDA_ARCH_LIST="9.0+PTX"` を必ず付ける**（理由の詳細は CLAUDE.md 参照）
- `camera_pose.py` / `fps_camera_pose.py` は Blender 内スクリプト（`blender -b ... --python ...` で実行。fps_camera_pose.py は Blender 4.5.5）
- `npz_to_c3d.py` / `filter_c3d.py` は Blender・GPU 不要
- データファイル（PLY・ポーズJSON・C3D・.blend）は `phase4/data/` に置く（git管理外）

### スクリプト一覧

| スクリプト | 役割 |
|---|---|
| `camera_pose.py` | Blenderカメラのポーズ（c2w）をJSONに書き出し |
| `fps_camera_pose.py` | FPS頭部追従カメラのポーズ書き出し（ヘッドレスでも向きを計算） |
| `npz_to_c3d.py` | リフトアップ済み3DキーポイントNPZ → C3D 変換（Blender取り込み対応） |
| `filter_c3d.py` | C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相） |
| `render.py` | PLY + ポーズJSON のバッチレンダリング（連番PNG/MP4） |
| `render_keypoints.py` | キャリブTOMLカメラでの3DGSレンダリング + キーポイント重ね描き / 静止画モード |

### camera_pose.py（カメラポーズ書き出し）

```bash
blender -b data/FPS-camera.blend --python camera_pose.py -- --camera FPSCamera
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--camera` | （必須） | カメラオブジェクト名 |
| `--output` | `data/<カメラ名>_poses.json` | 出力JSONパス |

### fps_camera_pose.py（FPS頭部追従カメラのポーズ書き出し）

アーマチュア＋アンカー＋子カメラ構成の .blend 用。`frame_change_post` ハンドラが `-b`（ヘッドレス）で発火しない問題に対応するため、Frankfurt平面ベースの姿勢計算を内蔵している。

```bash
/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b data/Blender/session001_world_22pt.blend \
    --python fps_camera_pose.py -- --camera Cam_FPS --output data/Cam_FPS_poses.json
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--camera` | （必須） | カメラオブジェクト名 |
| `--armature` | `E00000` | アーマチュア名 |
| `--anchor` | `Cam_Anchor` | アンカーEmpty名 |
| `--output` | `data/<カメラ名>_poses.json` | 出力JSONパス |

### npz_to_c3d.py（NPZ→C3D変換）

world座標系 (X,Y,Z)[m] の NPZ を、Blender io_anim_c3d で正立取り込みできる C3D（mm / +Z / +Y 規約）に変換する。

```bash
uv run python npz_to_c3d.py data/session001_world_22pt.npz
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--output` | `<入力>.c3d` | 出力C3Dパス |
| `--fps` | `30.0` | フレームレート（C3D point rate） |

### filter_c3d.py（C3D時間方向平滑化）

リフトアップ推定由来のジッターを Butterworth 2次 filtfilt（ゼロ位相）で除去する。入力は本プロジェクト規約のC3D（`npz_to_c3d.py` 出力）限定。

```bash
uv run python filter_c3d.py data/session001_world_22pt.c3d
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--output` | `<入力>_filtered.c3d` | 出力C3Dパス |
| `--cutoff` | `6.0` | カットオフ周波数[Hz]。下げるほど滑らか |
| `--rate` | なし | サンプリング周波数の補完（C3Dのpoint rate欠損時のみ使用可） |
| `--max-gap` | `10` | 線形補間する欠損ギャップ長の上限[フレーム]。超過はセグメント分割 |

### render.py（バッチレンダリング）

```bash
# dry-run（画像保存なしで動作確認・速度計測）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render.py data/project.ply data/FPSCamera_poses.json --dry-run

# 連番PNG + MP4
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render.py data/project.ply data/FPSCamera_poses.json --mp4
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--output-dir` | `./data/images` | 出力ディレクトリ |
| `--background` | `0 0 0` | 背景色 RGB（0-1、3値） |
| `--rotate-z90` | OFF | ワールドZ軸まわりに90°回転して描画 |
| `--start-frame` / `--end-frame` | なし | フレーム範囲（両端含む） |
| `--dry-run` | OFF | 画像を保存せず速度計測のみ |
| `--mp4` | OFF | MP4も出力 |
| `--mp4-fps` | `30` | MP4フレームレート（整数） |

### render_keypoints.py（キーポイント重ね描き / 静止画モード）

2つのモードがある。

**動画モード**（`c3d_path` を渡す）: キャリブTOMLのカメラで3DGSをレンダリングし、C3Dの人体キーポイント（Halpe26 + Spine/Thorax の既知28マーカー、欠損許容）をオクルージョン考慮で全フレーム重ね描きする。出力は連番PNG（`frame_<C3Dフレーム番号:06d>.png`）と `--mp4` 指定時のMP4。

```bash
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml data/Blender/keypoints.c3d \
    --camera cam41520554 --near-plane 0.5 --output-dir /tmp/keypoints --mp4
```

**静止画モード**（`--no-keypoints`。`c3d_path` は省略必須）: 3DGS背景のみの `still_<カメラ名>.png` を1枚出力する（再実行時は上書き）。`--distort` を付けるとTOMLの歪み係数（長さ4/5/8対応）で歪みモデルレンダリングになり、`estimate_camera_params.py` の推定結果をGT実写と視覚比較する用途に使う。

```bash
# ピンホール静止画
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    --camera cam41520554 --near-plane 0.5 --no-keypoints --output-dir /tmp/calib_check

# 歪みモデル静止画（GT比較用）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    --camera cam41520554 --near-plane 0.5 --no-keypoints --distort --output-dir /tmp/calib_check
```

| オプション | 既定値 | 説明 | 使えるモード |
|---|---|---|---|
| `--camera` | （必須） | TOML内の対象カメラ名 | 両方 |
| `--near-plane` | `0.1` | nearクリップ距離[m]。`0.01` だとカメラ至近のfloaterで黒い靄になるため `0.5` 推奨 | 両方 |
| `--output-dir` | `./data/keypoints_<カメラ名>` | 出力ディレクトリ | 両方 |
| `--background` | `0 0 0` | 背景色 RGB（0-1、3値） | 両方 |
| `--no-keypoints` | OFF | 静止画モード（`c3d_path` 省略必須） | — |
| `--distort` | OFF | TOMLの歪み係数で歪みレンダリング（gsplat 3DGUT経路） | 静止画のみ |
| `--no-occlusion` | OFF | 深度によるキーポイント隠蔽を無効化（比較用） | 動画のみ |
| `--occlusion-margin` | `0.05` | オクルージョン判定の深度マージン[m] | 動画のみ |
| `--start-frame` / `--end-frame` | なし | C3Dフレーム番号の範囲（両端含む）。1フレームだけ出すなら両方に同じ値 | 動画のみ |
| `--mp4` | OFF | MP4も出力（fps既定はC3D rate） | 動画のみ |
| `--mp4-fps` | C3D rate | MP4フレームレート（小数可） | 動画のみ |
| `--no-png` | OFF | 連番PNG保存をスキップしMP4のみ出力（`--mp4` 併用必須。数万フレームで大幅高速化） | 動画のみ |

静止画モードで動画専用オプションを指定するとエラー（終了コード2）になる。

## テスト

```bash
uv run pytest -v
```

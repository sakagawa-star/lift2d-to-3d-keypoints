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

- `fisheye = true` のセクションは未対応（エラー終了する。魚眼モデルは別途対応が必要）
- `--output` に `--intrinsic-toml` と同じパスは指定できない（入力TOMLの上書き防止のためエラー終了する）

## phase4: gsplatレンダリング

phase0 とは独立した uv 環境（`phase4/pyproject.toml`）。スクリプトは `phase4/` ディレクトリで実行する。

### 実行環境の注意

- **CUDA GPU 必須**（render.py / render_keypoints.py / render_fps_video.py）。gsplat の CUDA 拡張が初回実行時に JIT コンパイルされるため、環境変数 **`TORCH_CUDA_ARCH_LIST="9.0+PTX"` を必ず付ける**（理由の詳細は CLAUDE.md 参照）
- `render_fps_video.py` の MP4 モードは **ffmpeg / ffprobe（libx264 エンコーダが有効なビルド）も必須**（開始前に検査され、なければエラー終了）
- `camera_pose.py` / `fps_camera_pose.py` は Blender 内スクリプト（`blender -b ... --python ...` で実行。fps_camera_pose.py は Blender 4.5.5）
- `npz_to_c3d.py` / `filter_c3d.py` / `filter_npz.py` は Blender・GPU 不要（render_fps_video.py の `--dump-poses` モードも GPU 不要）
- データファイル（PLY・ポーズJSON・C3D・.blend）は `phase4/data/` に置く（git管理外）

### スクリプト一覧

| スクリプト | 役割 |
|---|---|
| `camera_pose.py` | Blenderカメラのポーズ（c2w）をJSONに書き出し |
| `fps_camera_pose.py` | FPS頭部追従カメラのポーズ書き出し（ヘッドレスでも向きを計算） |
| `npz_to_c3d.py` | リフトアップ済み3DキーポイントNPZ → C3D 変換（Blender取り込み対応） |
| `filter_c3d.py` | C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相） |
| `filter_npz.py` | NPZキーポイントの時間方向平滑化（NPZ→NPZ、C3D・Blender不要） |
| `render.py` | PLY + ポーズJSON のバッチレンダリング（連番PNG/MP4） |
| `render_keypoints.py` | キャリブTOMLカメラでの3DGSレンダリング + キーポイント重ね描き / 静止画モード |
| `refine_extrinsics.py` | 手動点（一意6点以上）+ LoFTR 自動マッチングによる外部パラメータ精緻化（K既知） |
| `render_fps_video.py` | NPZ直読みの一人称視点（FPS）動画一括生成（Blender・C3D 不要、再開可能、YAML設定対応、`--gpus` でチャンク並列レンダリング） |

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

### filter_npz.py（NPZ時間方向平滑化）

リフトアップ済み3DキーポイントNPZ（`npz_to_c3d.py` 入力と同一フォーマット）を C3D 変換なしで直接平滑化し、新しいNPZに書き出す。フィルタは `filter_c3d.py` と同一（Butterworth 2次 filtfilt・ゼロ位相）。NaN（人不在区間）は出力でも NaN のまま維持し、必須3キー以外の追加キーは無加工でコピーする。

```bash
uv run python filter_npz.py data/session001_world_22pt.npz --fps 30
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--fps` | （必須） | サンプリング周波数[Hz]（NPZに記録がないため必須） |
| `--output` | `<入力>_filtered.npz` | 出力NPZパス |
| `--cutoff` | `6.0` | カットオフ周波数[Hz]。下げるほど滑らか |
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

### refine_extrinsics.py（外部パラメータ精緻化。feat-026）

手動プロット点（カメラごとに一意な 2D-3D 対応 **6点以上**。多点 CSV はそのまま全点使用）を初期値に、3DGS レンダと LoFTR 自動マッチングの反復でカメラ外部パラメータ（R, t）を精緻化する。内部パラメータは入力 TOML の値を使う（K既知）。

```bash
# プロジェクトルートで実行（phase4 環境 + matcher_lab 環境を subprocess 連携）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/refine_extrinsics.py \
  --toml <入力TOML(Calib_scene形式)> --ply <3DGS PLY> --images-dir <実写画像ディレクトリ> \
  --points-3d <3D基準点CSV> --points-2d <2D点CSV> \
  --out-toml <出力TOML> --out-report <診断レポートtxt> \
  [--cameras cam1 cam2 ...] [--overwrite] [--seed-base 5000] [--tmp-dir <中間ファイル置き場>]
```

- 実写画像は `{images-dir}/{カメラ名}.png`（1920x1080 のみ対応）。対象カメラ省略時は 2D点 CSV に行がある全カメラを逐次処理
- 出力 TOML は入力の全カメラを保持し、受理カメラのみ rotation/translation を置換（入力ファイルは変更しない。既存出力の上書きは `--overwrite` 必須）
- 受理判定はサンプリング型（3チェーン×20サンプルの合意 f_c≥0.7。二峰時は手動点再投影で仲裁）。結果・失敗段・診断値はレポート参照
- 事前準備: `uv sync --project matcher_lab` と LoFTR 重みのローカル配置（初回のみ `matcher_lab/loftr_smoke.py` の実行で自動取得）。実行時はオフラインで動作
- 処理時間の目安: 1カメラ 3〜6分（RTX 5060 Ti 実測）

### render_fps_video.py（NPZ直読みFPS動画一括生成。feat-027/029/030/031）

リフトアップ済み3DキーポイントNPZ（`npz_to_c3d.py` 入力と同一フォーマット。`filter_npz.py` で平滑化済みを想定）から、頭部7点（LEye/REye/LEar/REar/Nose/Head/Neck）でFPSカメラポーズを計算し、3DGS（PLY）をレンダリングして1本のMP4を生成する。Blender・C3D 工程は不要。頭部7点が `joint_names` にない NPZ はエラー終了する。

- カメラ位置=両目中点、向き=一次視線（`fps_camera_pose.py` と同一の数式。旧Blenderパイプラインとの等価性検証済み）
- 内部パラメータは Calib_scene.toml 型 TOML から `--camera` で選択（ピンホール。歪み係数は無視。width/height は偶数必須）
- 頭部7点に NaN があるフレームと縮退フレームは黒画面で出力し、タイムライン（動画時刻=実時刻）を維持する
- チャンク（既定10000フレーム）単位の区間MP4で生成して最後に連結。中断後は同じコマンドの再実行で完成済みチャンクをスキップして再開（破損チャンクは ffprobe 検査で検出して作り直し。パラメータ変更時はエラー、`--overwrite` で作り直し）
- `--gpus` でチャンクを複数ワーカープロセスに動的分配して並列レンダリング（feat-030）。カンマ区切りリストの要素数=ワーカー数、各要素=そのワーカーのGPU ID（同一IDの繰り返しで同一GPUに複数ワーカー可）。未指定なら従来どおり直列。直列⇔並列をまたぐ再開可。ワーカー失敗時は即時中止（完成済みチャンクは保持され再実行で再開）
- MP4 生成成功時に `<MP4名>_info.txt`（例: `test_fps.mp4` → `test_fps_info.txt`）を自動保存（feat-031）。記載: 動画ファイル名・総所要時間・描画フレーム平均・総フレーム数・フレームレート・NaN黒フレーム数/番号・縮退黒フレーム数/番号・解像度。黒フレーム番号は frame_id 基準の連続区間圧縮（0件は `なし`）

```bash
# MP4 生成
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_fps_video.py data/<PLY> data/<NPZ> \
    --toml data/Calib_FPSCamera.toml --camera FPSCamera --fps 30

# 並列レンダリング（feat-030。例: GPU 0 に2ワーカー。単一GPUで実測 約1.47倍）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_fps_video.py data/<PLY> data/<NPZ> \
    --toml data/Calib_FPSCamera.toml --camera FPSCamera --fps 30 --gpus 0,0

# 設定YAMLで実行（feat-029。CLI 指定が YAML を上書き）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_fps_video.py --config data/run_fps.yaml

# ポーズのみJSONダンプ（GPU不要）/ デバッグ静止画（それぞれ排他モード）
uv run python render_fps_video.py data/<PLY> data/<NPZ> --toml <TOML> --camera <名前> --fps 30 --dump-poses poses.json
```

| オプション | 既定値 | 説明 |
|---|---|---|
| `--toml` | （必須※） | Calib_scene.toml 型 TOML パス |
| `--camera` | （必須※) | TOML 内のカメラ名 |
| `--fps` | （必須※） | フレームレート（NPZに記録がないため必須） |
| `--output` | `<NPZ名>_fps.mp4` | 最終MP4パス（NPZと同ディレクトリ） |
| `--gpu` | `0` | 直列モードで使用するGPU ID（単一。`--gpus` と併用不可） |
| `--gpus` | なし | 並列モード。ワーカーのGPU IDリスト（カンマ区切り、要素数=ワーカー数。例 `0,0`=GPU 0 に2ワーカー、`0,1,2`=3GPUに1ワーカーずつ。MP4モード専用、`--gpu` と併用不可） |
| `--chunk-size` | `10000` | チャンクのフレーム数（再開の単位） |
| `--crf` / `--preset` | `18` / `medium` | libx264 エンコード設定 |
| `--overwrite` | OFF | チャンクディレクトリを削除して作り直す |
| `--keep-chunks` | OFF | 連結後もチャンクディレクトリを残す |
| `--still-range START END` | なし | デバッグ静止画モード（排他。frame_id 基準・両端含む。`--still-dir` 必須） |
| `--dump-poses PATH` | なし | ポーズダンプモード（排他。c2w を JSON 出力、GPU不要） |
| `--config PATH` | なし | 設定YAML（フラット `key: value`。キーは CLI 名のハイフン→アンダースコア。※必須項目は YAML でも指定可） |

設定YAMLの例:

```yaml
ply_path: data/point_cloud.ply
npz_path: data/session001_filtered.npz
toml: data/Calib_FPSCamera.toml
camera: FPSCamera
fps: 30
output: data/session001_fps.mp4
# gpus: 0,0    # 並列レンダリング（feat-030。gpu キーとは併用不可）
```

## テスト

```bash
uv run pytest -v
```

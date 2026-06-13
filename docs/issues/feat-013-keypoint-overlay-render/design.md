# 機能設計書: feat-013 3DGSレンダリング画像への3Dキーポイント重ね描き

## 1. 新規ファイル

| ファイル | 内容 |
|---|---|
| `phase4/render_keypoints.py` | 本機能のメインスクリプト |
| `tests/test_feat013_render_keypoints.py` | ユニットテスト（bpy/CUDA非依存の純粋ロジック） |

`phase4/render.py` は変更しない（安定性維持）。再利用可能な純粋関数（`load_ply`, `print_ply_summary`）は `from render import ...` で読み込む。

## 2. 依存関係

`phase4/pyproject.toml` に **以下3つを追加**する（いずれも現在 phase4 環境に未インストール）。

- `opencv-python`（`cv2`）: `projectPoints`, `Rodrigues`, 円・線描画、PNG保存（`imwrite`）。phase0 では使用済みだが phase4 環境には無い
- `tomli`: TOML読み込み。**phase4 は Python 3.10.19 のため標準ライブラリ `tomllib`（3.11+）が使えず、`tomli` も未インストール**。phase0 と同じく `import tomli` を使う（確認済み: `import tomllib` も `import tomli` も現状 ModuleNotFoundError）
- `c3d`（PyPI、py-c3d）: C3D読み込み。`io_anim_c3d` 全体は bpy 依存のため使わず、その基盤である PyPI `c3d` パッケージを直接使う

### C3D読み込みAPIの確定方針
検証では `io_anim_c3d/c3d`（bpy非依存を確認済み。`from c3d.reader import Reader`）を使った。これは PyPI `c3d`（py-c3d）の派生であり、本機能では **PyPI `c3d` を採用**する。想定API（py-c3d 0.5系）:

- `reader = c3d.Reader(open(path, "rb"))`
- `reader.read_frames()` → `(frame_no, points, analog)` を yield。`points` は shape `(N, 5)` = `[x, y, z, residual, camera]`
- `reader.point_labels` → ラベルリスト、`reader.point_rate` → float
- `reader.first_frame` → 開始フレーム番号（プロパティ）

**実装開始時の唯一の確認作業**: `uv add c3d` 後、上記4点（特に `first_frame` がプロパティかメソッドか、`read_frames` の yield 形）をインストール版で2行のREPL確認する。万一API差異があれば、検証済みで bpy 非依存の `io_anim_c3d/c3d` サブパッケージを `phase4/` に vendoring する（フォールバック）。

## 3. データフロー

```
TOML ──parse──► カメラ辞書（K, rvec, tvec, size）
                   │
                   ├─► viewmat = [[R,t],[0,0,0,1]]  ─┐
                   ├─► K（ピンホール・歪みなし）     ─┼─► gsplat(RGB+ED) ─► 背景RGB + 深度マップ + alpha ─┐
PLY ──load_ply──► gaussians ─────────────────────────┘                                                  │
                                                                          （深度で前後判定）├─► オクルージョン考慮で重ね描き ─► PNG / MP4
C3D ──read──► フレームごと69マーカー                                                                     │
                   │                                                                                     │
                   └─► Halpe26抽出 ─► (pz,px,py)×0.001 ─► cv2.projectPoints(K) ─► 2D + カメラ深度 ──────┘
```

カメラは固定。フレームごとに変わるのはC3Dキーポイントのみ。レンダリング画像はカメラが固定なので**1回だけレンダリングして使い回す**（背景はフレーム間で不変）。各フレームではその使い回し画像のコピーにキーポイントを描画する。これにより gsplat 呼び出しは全フレームで1回に削減でき、長尺データで大幅に高速化する。

> 注: 3DGS自体は静的シーン（点群）であり時間変化しない。したがって固定カメラなら全フレーム同一の背景画像になる。これは本機能の重要な最適化前提。

## 4. モジュール構成

```python
"""3DGSレンダリング画像への3Dキーポイント重ね描きスクリプト"""

import argparse, os, sys, shutil, subprocess, time
import numpy as np
import cv2          # ルートvenv(pytest環境)にもあるので top-level でよい
import tomli        # 同上

# 重い/環境限定の依存は関数内 import に隔離する（camera_pose.py の bpy と同じパターン）:
#   - c3d            … ルートvenv(pytest)に無くphase4 venvのみ → load_c3d_frames 内で import
#   - render(torch/gsplat) … 同上 → render_background / main 内で import
# これにより純粋ロジック（座標変換・投影・描画）をルートvenvの pytest でテストできる。
# 実行（main）は phase4 venv で `uv run python render_keypoints.py ...` する。

# --- 定数 ---

# Halpe26 キーポイント名（C3D label と一致）
HALPE26_NAMES = [
    "Hip", "RHip", "RKnee", "RAnkle", "RBigToe", "RSmallToe", "RHeel",
    "LHip", "LKnee", "LAnkle", "LBigToe", "LSmallToe", "LHeel",
    "Neck", "Head", "Nose", "LEye", "REye", "LEar", "REar",
    "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow", "LWrist",
]

# ボーン端点名 → HALPE26_NAMES内インデックスの辞書（描画・ボーン部分隠蔽で使用）
NAME_TO_IDX = {name: i for i, name in enumerate(HALPE26_NAMES)}

# Halpe26 スケルトン（ボーン）定義: (始点名, 終点名, 部位)
# 部位: "R"=右半身, "L"=左半身, "C"=体幹/顔
HALPE26_SKELETON = [
    ("Head", "Nose", "C"), ("Nose", "Neck", "C"), ("Neck", "Hip", "C"),
    ("Nose", "REye", "R"), ("REye", "REar", "R"),
    ("Nose", "LEye", "L"), ("LEye", "LEar", "L"),
    ("Neck", "RShoulder", "R"), ("RShoulder", "RElbow", "R"), ("RElbow", "RWrist", "R"),
    ("Neck", "LShoulder", "L"), ("LShoulder", "LElbow", "L"), ("LElbow", "LWrist", "L"),
    ("Hip", "RHip", "R"), ("RHip", "RKnee", "R"), ("RKnee", "RAnkle", "R"),
    ("Hip", "LHip", "L"), ("LHip", "LKnee", "L"), ("LKnee", "LAnkle", "L"),
    ("RAnkle", "RHeel", "R"), ("RAnkle", "RBigToe", "R"), ("RBigToe", "RSmallToe", "R"),
    ("LAnkle", "LHeel", "L"), ("LAnkle", "LBigToe", "L"), ("LBigToe", "LSmallToe", "L"),
]

# 色 (BGR, OpenCV)。左右は被験者本人の解剖学的左右（マーカー名 R*/L* に従う。カメラ視点ではない）
COLOR_RIGHT = (0, 0, 255)     # 赤（被験者の右半身: RShoulder 等）
COLOR_LEFT = (255, 0, 0)      # 青（被験者の左半身: LShoulder 等）
COLOR_CENTER = (0, 255, 0)    # 緑（体幹・顔の中心線）
POINT_COLOR = (0, 255, 255)   # 黄（キーポイント円）
POINT_RADIUS = 4
LINE_THICKNESS = 2

# C3D の無効サンプル判定: points[:, 3]（residual）が負なら無効
C3D_INVALID_RESIDUAL = 0.0    # residual < 0 を無効とする閾値

# オクルージョン判定
OCCLUSION_MARGIN = 0.05    # [m] 3DGS深度ノイズ＋マーカーが体表より内側に埋まる分を吸収。これ以上奥なら隠す
ALPHA_THRESH = 0.5         # 累積不透明度がこれ未満の画素は「3DGSなし」扱い（隠さない）
BONE_SAMPLES = 24          # ボーン部分隠蔽の線分サンプル数
```

## 5. 関数仕様

### 5.1 `load_cameras_toml(toml_path: str) -> dict[str, dict]`
- `tomli.load` で読み込み、`matrix` キーを持つテーブルのみカメラとして返す
- 各カメラ値を `{"K": np.ndarray(3,3), "D": np.ndarray(5,), "rvec": np.ndarray(3,), "tvec": np.ndarray(3,), "width": int, "height": int}` に整形
- bpy/CUDA非依存 → テスト対象

### 5.2 `select_camera(cameras: dict, name: str) -> dict`
- `name` が無ければ `ValueError`（利用可能カメラ名一覧をメッセージに含める）→ main で捕捉し stderr出力・終了コード1
- テスト対象

### 5.3 `camera_to_viewmat(cam: dict) -> np.ndarray`
- `R, _ = cv2.Rodrigues(cam["rvec"])`、`viewmat = [[R, t],[0,0,0,1]]`（4x4 float32）を返す
- OpenCV world-to-camera をそのまま gsplat の viewmat とする
- テスト対象（既知のrvec/tvecに対する数値検証）

### 5.4 `compute_keypoint_depth(kpts_calib: np.ndarray, cam: dict) -> np.ndarray`
- 各キーポイントの**カメラからの深度**を返す。`R = cv2.Rodrigues(cam["rvec"])` として `(R @ X + t)` のZ成分。入力 `(...,3)` → 出力 `(...,)`
- この深度を、レンダリング深度マップ（5.10）と比較してオクルージョン判定（5.12）に使う
- bpy/CUDA非依存 → テスト対象
- **（注）旧 `distortion_to_gsplat` は廃止**。歪みなし（ピンホール統一）に変更したため不要。歪みを gsplat の UT 経路に乗せると投影の近似で黒い靄・品質劣化が出る、というのが手動テストで判明したため

### 5.5 `load_c3d_frames(c3d_path: str) -> tuple[list[str], np.ndarray, np.ndarray, float, int]`
- C3Dを読み込み、`(labels, data, residual, point_rate, first_frame)` を返す
  - `labels`: マーカー名リスト（strip済み）
  - `data`: shape `(n_frames, n_markers, 3)` の生座標（mm）
  - `residual`: shape `(n_frames, n_markers)`。py-c3d の `points[:, 3]`。**`residual < 0` は無効サンプル**（カメラ未観測。座標にはNaNや前値・0が入りうるため、座標値ではなく residual で無効判定する）
  - `point_rate`: float、`first_frame`: int（C3Dの開始フレーム番号）
- `c3d.Reader.read_frames()`（`(frame_no, points(N,5), analog)` を yield）を回して `points[:, :3]` と `points[:, 3]` を蓄積する薄いラッパ
- テストは小さな合成C3Dを書き出して読む or 実データに対する `skipif`（9.1参照）

### 5.6 `c3d_to_calib(points_mm: np.ndarray) -> np.ndarray`
- 入力 `(..., 3)` の `(px,py,pz)` mm → `(pz, px, py) * 0.001`
- `np.stack([pz, px, py], axis=-1) * 0.001`
- **本機能の中核**。bpy/CUDA非依存 → 重点テスト対象

### 5.7 `extract_halpe26(labels, data, residual) -> tuple[np.ndarray, np.ndarray]`
- `labels` から HALPE26_NAMES の各名のインデックスを引き、対応する `data[:, idx, :]` を Halpe26順に並べた `(n_frames, 26, 3)` を返す
- 各点の有効フラグ `valid` `(n_frames, 26)` も返す。**有効 = `residual >= 0` かつ 座標にNaNを含まない**。「全成分0」は判定基準にしない（原点近傍の正当な点を誤って無効化するため）
- C3DにHalpe26名が欠ける場合は `ValueError`（不足マーカー名を表示）
- テスト対象

### 5.8 `project_keypoints(kpts_calib: np.ndarray, cam: dict) -> np.ndarray`
- 1フレーム分 `(26,3)` を `cv2.projectPoints(kpts_calib, cam["rvec"], cam["tvec"], cam["K"], distCoeffs=None)` で `(26,2)` に投影して返す（**歪みなし＝ピンホール**。背景の3DGSがピンホールのため整合させる）
- NaN座標は `np.nan_to_num` でサニタイズしてから渡す（無効点は描画側で `valid`/可視性によりスキップ）
- テスト対象（実データ依存・skipif。歪みなしの既知値）

### 5.9 `draw_overlay(image, pts2d, valid, visible) -> np.ndarray`
- `image`（H,W,3 uint8 BGR）のコピーに、Halpe26 の点とボーンを**オクルージョン考慮で**描画して返す
- **点**: `valid[i] and visible[i]` の点のみ円を描画（手前の3DGSに隠れる点＝`visible=False` は描かない）
- **ボーン（部分隠蔽）**: 両端が `valid` の辺について、両端キーポイントの**3D位置**（calib座標）を N 分割（既定 `BONE_SAMPLES=24`）で線形補間し、各サンプルを投影・深度算出して 5.12 の可視判定を適用、**可視サンプルが連続する区間だけ線分を描く**。これにより障害物を横切るボーンが途中で切れる
  - 実装簡略化のため、ボーン描画は「サンプル列のうち可視な隣接ペアを短い線分で繋ぐ」方式（点描に近いが太線で連続的に見える）
- 部位色（R/L/C）は維持
- テスト対象（小画像で描画が落ちないこと、`visible` と部分描画の制御）

> **draw_overlay の確定シグネチャ**: `draw_overlay(image, kpts_calib, pts2d, valid, kp_visible, cam, depth_map, alpha_map, margin, occlusion=True) -> image`
> - `kp_visible`: 5.12 で計算した各キーポイントの可視性 `(26,)`。**点の描画**に使う（`occlusion=True` 時は `valid[i] and kp_visible[i]`、`occlusion=False` 時は `valid[i]` のみ）
> - **ボーン描画は draw_overlay 内でサンプル単位に再判定する**（点の可視性とボーンサンプルの可視性は別物）。手順: 各辺 `(start, end)` の両端 `ia=NAME_TO_IDX[start]`, `ib=NAME_TO_IDX[end]` の3D座標 `kpts_calib[ia], kpts_calib[ib]` を `BONE_SAMPLES` 個に線形補間 → 5.8相当で投影＋5.4相当で深度算出 → `compute_visibility`(5.12) をサンプル列に適用（**補間サンプルにはC3D由来の `valid` が無いので `valid=np.ones(BONE_SAMPLES, bool)` を渡す**。辺は両端valid時のみ描画するので前提として両端有効）→ **可視な隣接サンプルのみ短い線分で結ぶ**
> - **`occlusion=False`（`--no-occlusion`）時**: 点は `valid` のみで描画（`kp_visible` を無視＝全て手前扱い）、ボーンもサンプル可視判定をスキップして両端valid辺を全描画する（旧2D重ね描き）。`depth_map`/`alpha_map` は使わない
> - 投影・深度・可視判定の各純粋関数（5.4/5.8/5.12）を呼ぶ薄い描画層に保つ

### 5.10 `render_background(gaussians, cam) -> tuple[np.ndarray, np.ndarray, np.ndarray]`
- gsplat で1回レンダリングし、`(bgr, depth_map, alpha_map)` を返す（固定カメラのため使い回す）
  - `bgr`: (H,W,3) uint8。gsplat出力はRGBなので `[..., ::-1]` でBGR化
  - `depth_map`: (H,W) float32。各画素の**期待深度**（カメラからの距離）。オクルージョン判定に使う
  - `alpha_map`: (H,W) float32。各画素の累積不透明度。低alpha=3DGSが無い/薄い画素の判定に使う（隠すものが無い→キーポイントは可視）
- **ピンホール古典経路（`render.py` の `render_frame` と同一。歪みなし）**:
  - `camera_model="pinhole"`・**歪み係数なし**・`with_ut=False`・`packed=True`（いずれもデフォルト。UT歪みは靄・品質劣化のため不使用）
  - `render_mode="RGB+ED"` を指定 → 戻り値 `rendered` は (1,H,W,4)。`[...,:3]`=RGB、`[...,3]`=expected depth。`alphas` は (1,H,W,1)
  - `near_plane=0.01, far_plane=1e10`、`sh_degree` は load_ply 由来
- `viewmats` は 5.3 の `camera_to_viewmat`（OpenCV w2c）をそのまま渡す
- 取り出し: `depth_map = rendered[0,:,:,3]`、`alpha_map = alphas[0,:,:,0]`、`rgb = rendered[0,:,:,:3].clamp(0,1)`
- **現実装からの変更点**: 戻り値を `(bgr, depth_map, alpha_map)` の3つ化、`with_ut=True`→削除（デフォルトFalse）、`radial/tangential_coeffs` 削除、`render_mode="RGB+ED"` 追加、`packed=False`→削除（デフォルトTrue）
- CUDA依存 → 手動テスト

### 5.11 `main()`
- 引数解析 → PLYロード → TOMLロード・カメラ選択 → C3Dロード・Halpe26抽出・calib変換 → **背景1回レンダリング（bgr + depth_map + alpha_map）** → フレームループ → MP4/PNG出力
- フレームループ（各フレーム i）: `project_keypoints`（歪みなし2D）→ `compute_keypoint_depth`（カメラ深度）→ `compute_visibility`（点の可視性）→ `draw_overlay`（点は可視のみ、ボーンは部分隠蔽）→ 出力。`--no-occlusion` 時は `compute_visibility` をスキップし `draw_overlay(..., occlusion=False)` を渡して点・ボーンとも全描画（従来の2D重ね描き）
- MP4/ffmpeg・フレーム範囲・dry-run のロジックは `render.py` の `main()` を踏襲（一部コードは重複するが render.py 非改変を優先）
- **BGR統一**: 背景はBGRで保持。PNG保存は `cv2.imwrite(path, bgr_img)`（OpenCVはBGR想定）。MP4は ffmpeg 入力を **`-pix_fmt bgr24`**（render.py の `rgb24` から変更。さもないと色が反転する）にし、`bgr_img.tobytes()` を流す
- **フレーム番号の定義**: `--start-frame`/`--end-frame` は **C3D絶対フレーム番号**（`first_frame` 起点、両端含む）。各フレームの絶対番号 = `first_frame + i`（i は0始まりインデックス）。出力PNG名は `frame_{絶対番号:06d}.png`（render.py 踏襲）。範囲が空なら終了コード1（8章）
- **`--mp4-fps`**: `type=float`, `default=None`。未指定時は `round(point_rate)` を採用（C3D読込後に決定するため argparse default では表現できず main 内で補完）
- **`--no-keypoints`**: 背景のみ出力（デバッグ用）。固定カメラ＝全フレーム同一背景のため、この場合は**警告を表示**する（PNG: 全フレーム同一画像をN枚出力／MP4: 静止映像になる旨）。キーポイント描画をスキップするだけで出力フレーム数は通常と同じ。この場合は **C3D読込・Halpe26抽出・投影もスキップ**してよい（背景確認用途のため。C3DにHalpe26名が無くても背景は出せる）。出力フレーム数はC3Dのフレーム数を用いるため、フレーム数取得のための軽量なC3D読込（`first_frame`/フレーム数のみ）は行う
- **`--no-occlusion`**: オクルージョン判定を無効化し、全キーポイントを常に手前に描く（旧2D重ね描き）。デバッグ・比較用

### 5.12 `compute_visibility(pts2d, depth_cam, valid, depth_map, alpha_map, margin) -> np.ndarray`
- 各点（キーポイント or ボーンサンプル点）の**可視性** `(n,) bool` を返す。判定（点 i）:
  1. `valid[i]` が False → `False`
  2. 投影位置が画像外（後述の境界条件）→ `True`（隠す3DGSが画面内に無い。枠外はオクルージョン判定不能なので常に可視とし、描画はOpenCVのクリッピングに委ねる）
  3. `alpha_map[row, col] < ALPHA_THRESH`（3DGSが無い/薄い画素）→ `True`
  4. `depth_cam[i] > depth_map[row, col] + margin`（キーポイントが3DGSより奥）→ `False`（隠れる）
  5. それ以外 → `True`
- **判定順序の根拠（順序を変えないこと）**: gsplat の expected depth は `depth_acc / alpha.clamp(min=1e-10)` で正規化される（rendering.py で確認）。**低alpha画素では深度が不安定（0付近〜巨大値）で信頼できない**ため、alpha判定（ステップ3）を深度判定（ステップ4）より**必ず先**に置き、低alpha画素を深度比較に到達させない
- **画素インデックス規約**: gsplat出力は `[H, W]` = `[row=v(縦), col=u(横)]`。投影座標 `(u, v)` を `col = int(round(u))`, `row = int(round(v))` で丸めて `depth_map[row, col]` / `alpha_map[row, col]` を参照する（u と v の取り違えに注意）
- **画像外（ステップ2）の境界**: `0 <= col < W` かつ `0 <= row < H` を満たさなければ画像外（`<` 比較。W/H ちょうど・負値は外）。配列参照前に必ず境界判定して IndexError を防ぐ
- `margin` は `--occlusion-margin`（既定 `OCCLUSION_MARGIN`）。`depth_map`/`alpha_map` は numpy 配列で渡す
- bpy/CUDA非依存 → テスト対象（合成の深度マップ・alphaマップで5分岐＋境界・丸めを検証）

## 6. CLI 仕様

```
uv run python render_keypoints.py <ply> <toml> <c3d> --camera <名前> [options]
```

| 引数 | 必須 | デフォルト | 説明 |
|---|---|---|---|
| `ply_path` | Yes | - | 3DGS PLYパス |
| `toml_path` | Yes | - | キャリブTOMLパス |
| `c3d_path` | Yes | - | キーポイントC3Dパス |
| `--camera` | Yes | - | TOML内の対象カメラ名 |
| `--output-dir` | No | `./data/images` | 出力ディレクトリ |
| `--mp4` | No | False | MP4出力 |
| `--mp4-fps` | No | None→`round(point_rate)` | MP4フレームレート（`type=float`、未指定時C3Dのpoint_rateを四捨五入） |
| `--dry-run` | No | False | 保存せず動作確認・速度計測 |
| `--start-frame` | No | None | 処理開始フレーム（C3D絶対フレーム番号、`first_frame`起点） |
| `--end-frame` | No | None | 処理終了フレーム（C3D絶対フレーム番号、含む） |
| `--background` | No | `0 0 0` | 背景色RGB[0-1]（render.py踏襲） |
| `--no-keypoints` | No | False | キーポイント描画なし（背景のみ。デバッグ用。全フレーム同一背景になる旨を警告表示） |
| `--no-occlusion` | No | False | オクルージョン無効（全点を手前に描く＝旧2D重ね描き。比較用） |
| `--occlusion-margin` | No | `OCCLUSION_MARGIN`(0.05) | 深度マージン[m]（argparse default は定数 `OCCLUSION_MARGIN` を参照し値の二重管理を避ける。3DGSよりこの値以上奥なら隠す） |

実行例:
```bash
cd phase4
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply \
    data/Blender/Config_scene.toml \
    data/Blender/20260606-osaka-hosp1_0-301_filt_butterworth_LSTM.c3d \
    --camera cam41520554 --mp4
```

## 7. アルゴリズム詳細（座標変換）

### 7.1 C3D → キャリブ座標
```
p = (px, py, pz)  [mm]            # C3D生座標
X_calib = (pz, px, py) * 0.001    [m]
```
根拠: Pose2Sim は triangulation 結果（キャリブ座標、Z-up）を `zup_to_yup`（新=(Y,Z,X)）で変換しTRC/C3D出力する。その逆は「新の各成分を元へ戻す」= 元X=新Z(pz), 元Y=新X(px), 元Z=新Y(py)。実データ5カメラ再投影で検証済み。

### 7.2 カメラ（gsplat viewmat）
```
R = Rodrigues(rotation)           # 3x3, world-to-camera
viewmat = [[R, t],
           [0, 0, 0, 1]]          # 4x4, OpenCV world-to-camera をそのまま使用
```

### 7.3 投影（キーポイント）とオクルージョン
```
pts2d, _ = cv2.projectPoints(X_calib, rotation, translation, K, distCoeffs=None)  # 歪みなし
depth_cam = (R @ X_calib.T + t.reshape(3,1)).T[:, 2]                              # カメラ深度
# 可視判定は概念的に「その画素に3DGSが無い/薄い」または「キーポイントが3DGSより手前」
# ※これは概念図。確定仕様（判定順序=alpha先・深度後、画素インデックス row=int(round(v))/col=int(round(u))、
#   境界条件）は 5.12 を参照。実装は 5.12 の compute_visibility を使うこと
```
背景3DGSもピンホール（歪みなし）でレンダリングするため、キーポイント投影と同じ画素空間に乗る。`render_mode="RGB+ED"` の深度マップと各キーポイントのカメラ深度を比較して前後関係（オクルージョン）を反映する（具体ロジックは 5.12）。

### 7.4 ボーンの部分隠蔽
- ボーン両端の3D座標 `Xa, Xb`（calib）を `t∈[0,1]` で線形補間し `BONE_SAMPLES` 個サンプル → 各サンプルを投影・深度算出 → 可視判定
- 可視な隣接サンプル同士を短い線分で結ぶ（不可視サンプルを跨ぐ箇所は描かない）→ 障害物を横切るボーンが途中で切れる

## 8. エラー処理

| 状況 | 動作 |
|---|---|
| `--camera` がTOMLに無い | カメラ名一覧をstderr表示、終了コード1 |
| C3DにHalpe26名が不足 | 不足名をstderr表示、終了コード1 |
| フレーム範囲が空 | メッセージ表示、終了コード1（render.py踏襲） |
| 欠損キーポイント（residual<0/NaN） | 当該点・接続ボーンを描画スキップ（クラッシュしない） |
| 手前の3DGSに隠れるキーポイント | オクルージョン判定で描画スキップ（クラッシュしない） |
| ffmpeg不在（MP4時） | エラー表示、終了コード1（render.py踏襲） |

## 9. テスト設計

純粋ロジック（`c3d_to_calib`, `camera_to_viewmat`, `compute_keypoint_depth`, `compute_visibility`, `extract_halpe26`, `draw_overlay`）は**合成データのみ**で完結させる。実データ（TOML/C3D/PLY）依存テストは `@pytest.mark.skipif(not Path(...).exists())` でファイル非存在時にスキップする（これらは gitignore 対象で他環境に無いため）。

**既存テスト（`tests/test_feat013_render_keypoints.py`）の更新方針**（現行ファイルは前イテレーション版。実装時に下記で全面改訂する）:
- **削除**: `distortion_to_gsplat` の import と `TestDistortionToGsplat` クラス（関数廃止のため。残すと import エラーで collect 失敗する）
- **更新**: `project_keypoints` のテストは歪みなし（`distCoeffs=None`）前提に修正。`draw_overlay` のテストは新シグネチャ（`kp_visible` 等、5.9）に合わせ、`visible=False` の点が描かれないことを検証
- **追加**: 下記の `compute_keypoint_depth` / `compute_visibility` のテスト

- `c3d_to_calib`: 既知の `(px,py,pz)` に対し `(pz,px,py)*0.001` を返す（スカラ・配列・複数フレーム形状）
- `camera_to_viewmat`: 既知 rvec/tvec に対し `Rodrigues` 整合・4x4・最下行 `[0,0,0,1]`
- `compute_keypoint_depth`: 既知 rvec/tvec/点に対し `(R·X+t)` のZ成分が一致（恒等回転で `t_z + X_z` 等）
- `compute_visibility`: 合成 depth_map/alpha_map で5分岐を検証（invalid→False、枠外→True、低alpha→True、奥→False、手前→True）
- `extract_halpe26`: 合成labels/data/residualで26点をHalpe26順に抽出、`residual<0`→valid=False、NaN座標→valid=False、Halpe26名不足→ValueError
- `draw_overlay`: 小画像で例外なく描画、`valid`/`visible` がFalseの点・ボーンが描かれない（オクルージョンで隠れる点が消えること）
- `load_cameras_toml` / `select_camera`（**実データ依存・skipif**）: 実データTOMLで5カメラ取得、存在しない名でValueError
- `project_keypoints`（**実データ依存・skipif**）: 実データTOMLの1カメラ＋実C3Dフレーム1で、26点中大半が画像枠内（座標系確定の回帰テスト。歪みなし）。**歪みなし投影は歪みありと枠内点数が変わりうるため、実機で歪みなしの枠内点数を1回確認し閾値（>=20等）を妥当に再設定する**

### 9.2 手動テスト（実機 GPU、ユーザー実施）
- `TORCH_CUDA_ARCH_LIST="9.0+PTX"` 付きで実データをレンダリングし、人体キーポイントが3DGS点群上の妥当な位置・姿勢に重なることを目視確認
- **3DGSが歪み・靄なくピンホールで鮮明に描画されること**（GT＝Blenderカメラビューと構図が一致）
- **オクルージョンの確認**: カメラ手前の障害物に人体が差し掛かるフレームで、隠れるべきキーポイント・ボーンが正しく消えること。`--no-occlusion` との比較で隠蔽の効果を確認
- `--dry-run` で速度計測、`--mp4` でMP4生成確認

## 10. CLAUDE.md への影響（完了時に更新）

- ディレクトリ構成に `phase4/render_keypoints.py` を追記
- phase4 セクションに本スクリプトの実行例・ワークフロー（元データ直読み、Blender非経由）を追記
- phase4 の依存追加（opencv-python, tomli, c3d）を反映

## 11. 実装時の確認事項（リスク）

1. **gsplat 深度マップの仕様**（5.10）: `render_mode="RGB+ED"` の戻り値形状（`rendered` が (1,H,W,4)、末尾が expected depth であること）と、`alphas` の形状 (1,H,W,1) を実機で1回確認する。深度の単位がカメラからの距離（Z成分系）であることも、既知キーポイント深度と1点照合する
2. **オクルージョン閾値の調整**（5.12）: `OCCLUSION_MARGIN`(0.05m) と `ALPHA_THRESH`(0.5) は経験値。3DGSの深度は半透明境界でぼやけるため、隠れすぎ/隠れなさすぎを手動テストで見て調整する。`--occlusion-margin` で実機調整可能にする
3. **ボーン部分隠蔽の品質**（5.9/7.4）: `BONE_SAMPLES`(24) のサンプル数で、障害物境界のボーン切断が滑らかか確認。粗ければ増やす
4. **C3D読み込みAPIの版差**（2章）: 確認済み（py-c3d 0.6.0 で `first_frame` プロパティ・`read_frames` の (frame_no, (N,5), analog) を実機確認済み）
5. **背景レンダリングの速度**: ピンホール古典経路（UT不使用）で `--dry-run` 実測。背景は1回のみ（固定カメラ）なので長尺でもオーバーヘッドは小さい

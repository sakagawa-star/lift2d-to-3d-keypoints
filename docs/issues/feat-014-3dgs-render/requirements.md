# 要求仕様書: feat-014 ピンホール3DGSレンダリング（PNG出力）

## 1. 背景・目的

feat-013 を分割し、その第1ステップとして「指定カメラ視点で3DGS PLY をピンホールカメラでレンダリングしてPNG1枚を出力する」最小機能を確実に作る。出力PNGをグランドトゥルース（Blender+KIRIで同カメラ視点をレンダリングした画像 `0001_color.png`）と並べ、3DGSレンダリングの正しさ（カメラ位置・向き・構図・floater除去）を目視検証できるようにする。

`phase4/render_keypoints.py` を3DGSレンダリングのみに作り直す。キーポイント重ね描き・オクルージョンは後続（feat-013 再開）で段階的に追加する。

## 2. 入力

実データは `phase4/data/Blender/` に存在する。

- **PLY**: LCC Studio製3DGS（`point_cloud.ply`）。`render.py` の `load_ply()` で読める形式（`x,y,z`, `scale_*`, `rot_*`, `opacity`, `f_dc_*`, `f_rest_*`）
- **TOML**: Pose2Simキャリブ（`Config_scene.toml`）。複数カメラのテーブル。各カメラに `matrix`(K), `distortions`, `rotation`(Rodrigues), `translation`, `size`。R, t は OpenCV規約の world-to-camera
- **--camera**: 対象カメラ名（例 `cam41520554`）

C3D・人体キーポイントは本機能では使わない。

## 3. 機能要件

### 3.1 カメラ選択
- `--camera <名前>`（必須）で TOML 内のカメラを1台選択する
- 指定名が存在しない場合、エラーメッセージと利用可能カメラ名一覧を表示し、終了コード1で終了する

### 3.2 レンダリング
- gsplat で **ピンホールカメラ（歪みなし）** レンダリングする
- viewmat は TOML から直接構成: `[[R, t],[0,0,0,1]]`（R=Rodrigues(rotation), t=translation）。Blender経由の変換は使わない
- `camera_model="pinhole"`・歪み係数なし・`with_ut=False`・`packed=True`（= `render.py` の `render_frame` と同一の古典経路）
- `--near-plane`（既定 0.1）で near クリップ距離を指定する。`near_plane=0.01` だとカメラ至近の floater で黒い靄になるため、floater を除去できる値を指定可能にする
- 主点（cx, cy）が画像中心からずれていても TOML の K をそのまま使い反映する
- 解像度は TOML の `size`（実データ 1920×1080）

### 3.3 出力
- **PNG 1枚**を出力する（`cv2.imwrite`。gsplat出力RGBをBGRに変換して保存）
- 出力パスは `--output`（既定 `./data/render_<カメラ名>.png`）
- 背景色は `--background`（RGB[0-1]、既定 黒）

## 4. 品質基準

- 出力PNGが GT（`0001_color.png`）と並べて**同じ病室として一致**すること（カメラ位置・向き・構図）。実機での目視確認
- `--near-plane` を適切な値（0.1〜0.5）にすると、`near_plane=0.01` で出ていた**黒い靄が消えて病室が鮮明になる**こと
- 存在しないカメラ名でエラー終了すること
- 純粋ロジック（`load_cameras_toml`, `select_camera`, `camera_to_viewmat`）の bpy/CUDA 非依存ユニットテストが `uv run pytest -v` で成功すること
- gsplatレンダリング部分は実機（GPU + `TORCH_CUDA_ARCH_LIST="9.0+PTX"`）での手動テストで確認する

## 5. スコープ外（後続ステップ・feat-013 で扱う）

- 人体キーポイント（C3D）の読み込み・投影・重ね描き
- オクルージョン（深度マップによる前後判定）
- 深度マップの出力（オクルージョン用）
- MP4出力・連番PNG・フレーム範囲（本機能は固定カメラで1枚のみ。点群は静的）
- レンズ歪みの再現（ピンホール統一）
- near_plane の最適値の自動決定（`--near-plane` で手動指定。GT比較で調整）

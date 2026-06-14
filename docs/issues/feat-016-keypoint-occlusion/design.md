# 機能設計書: feat-016 キーポイントのオクルージョン（深度による前後判定）

本設計書は CLAUDE.md の機能追加フローに従い、実装前にレビューを受けるためのものである。コード例は「意図の伝達」が目的であり、そのままコピーして使うものではない。

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 C3D先頭フレーム読み込み | 5.5 `load_c3d_first_frame`, 5.6 `c3d_to_calib`, 5.7 `extract_halpe26` |
| FR-002 投影・カメラ深度算出 | 5.8 `project_keypoints`, 5.4 `compute_keypoint_depth` |
| FR-003 深度マップ・αマップ取得 | 5.3 `render_image`（`return_depth` 拡張） |
| FR-004 オクルージョン判定 | 5.9 `compute_visibility` |
| FR-005 重ね描き | 5.10 `draw_overlay` |
| FR-006 PNG出力 | 5.11 `main` |
| FR-007 `--no-occlusion` | 5.10 `draw_overlay`, 5.11 `main` |
| FR-008 `--occlusion-margin` | 5.9, 5.11, 6 CLI |

## 2. システム構成

### 2.1 変更ファイル
| ファイル | 変更種別 | 内容 |
|---|---|---|
| `phase4/render_keypoints.py` | 拡張 | feat-015 完成版に C3D投影系・オクルージョン判定・重ね描きを追加。`render_image` に `return_depth` 引数を追加 |
| `phase4/pyproject.toml` | （確認のみ） | `c3d>=0.6.0` は**既に追加済み**。変更不要 |
| `docs/TECH_STACK.md` | 追記 | `c3d` の用途・選定理由・バージョンを追記（未記載の場合） |
| `tests/test_feat016_keypoints.py` | 新規 | 本機能の純粋ロジックのユニットテスト |
| `tests/test_feat015_render.py` | 更新 | `main` 系3テストを新CLI（`c3d_path` 必須化）・新 `render_image` スタブ署名に合わせて更新（後述 8章） |

`phase4/render.py` は変更しない（`load_ply`/`print_ply_summary` を `from render import` で再利用）。

### 2.2 モジュール内の関数構成（`render_keypoints.py`）

```
既存（feat-015、変更なし or 軽微）:
  load_cameras_toml / select_camera / camera_to_viewmat   … そのまま
  render_image                                            … return_depth 引数を追加（後方互換）
  _build_parser / main                                    … CLI拡張・処理追加

新規（feat-013 design.md から復元 ＋ 新設）:
  [定数]  HALPE26_NAMES / NAME_TO_IDX / HALPE26_SKELETON / 色・半径・線幅 / 閾値
  load_c3d_first_frame   … C3D先頭フレーム読み込み（新設。c3d を関数内 import）
  c3d_to_calib           … C3D生座標(mm) → キャリブ座標(m)（復元）
  extract_halpe26        … ラベルからHalpe26を抽出・有効フラグ（復元）
  project_keypoints      … 歪みなし投影（復元）
  compute_keypoint_depth … カメラ深度（復元）
  compute_visibility     … オクルージョン判定（復元）
  draw_overlay           … 点・ボーンのオクルージョン考慮描画（復元）
```

### 2.3 依存隔離（CUDA非依存テストのため）
- `cv2`, `numpy`, `tomli` は top-level import（ルートvenv の pytest 環境にもある）
- `c3d` は `load_c3d_first_frame` 内で関数内 import（ルートvenv に無い／phase4のみ）
- `torch`/`gsplat`/`render`（`load_ply` 経由 torch）は `render_image`/`main` 内で関数内 import
- → 座標変換・投影・可視判定・描画の純粋関数はルートvenv の pytest でテストできる

## 3. 技術スタック

- 言語: Python 3.10（phase4 環境）
- ライブラリ（phase4）: gsplat 1.5.3（3DGSレンダリング・深度）、opencv-python（投影・描画・PNG）、tomli（TOML）、numpy、**c3d>=0.6.0（C3D読み込み、追加済み）**
- パッケージ管理: uv（`phase4/pyproject.toml`）
- 選定理由（c3d）: bpy 非依存で C3D を直読みできる PyPI パッケージ（py-c3d）。`io_anim_c3d` 全体は bpy 依存のため使わない

## 4. データフロー

```
TOML ──load_cameras_toml──► カメラ辞書（K, rvec, tvec, size）
                                │
                                ├─► viewmat = [[R,t],[0,0,0,1]] ─┐
                                ├─► K（ピンホール・歪みなし）    ─┼─► render_image(return_depth=True)
PLY ──load_ply──► gaussians ────┘                                │      └─► 背景BGR + 深度マップ + αマップ ─┐
                                                                  │                                          │
C3D ──load_c3d_first_frame──► 先頭フレーム(labels,data,residual)  │      （深度・αで前後判定）               │
       └─► extract_halpe26 ─► (26,3)mm + valid(26,)               │       compute_visibility ─► 可視(26,)    │
              └─► c3d_to_calib ─► (26,3)m ─► project_keypoints ─► 2D(26,2) ─┘                                │
                                          └─► compute_keypoint_depth ─► カメラ深度(26,) ─────────────────────┤
                                                                                                             ▼
                                                                              draw_overlay ─► 重ね描きBGR ─► PNG1枚
```

- カメラ・フレームともに固定（先頭フレーム1枚）。3DGSレンダリングは1回のみ。

## 5. 関数仕様

> 復元元: 下記で「（復元）」と記す関数の詳細仕様・定数定義は、中止案件 feat-013 の `docs/issues/feat-013-keypoint-overlay-render/design.md`（5章・4章のコードスニペット）に完全な形で残っている。git の旧コミットは現存しないため、復元元はこの design.md とする。本案件はそれを**先頭フレーム1枚のスコープに合わせて取り込む**。

### 5.1 `load_cameras_toml` / 5.2 `select_camera` / 5.3a `camera_to_viewmat`
feat-015 の既存実装をそのまま使う（変更なし）。

### 5.3 `render_image(gaussians, cam, near_plane, background=(0,0,0), return_depth=False)` 【拡張】
- **後方互換が最重要**: 既定 `return_depth=False` のときは feat-015 と完全に同一挙動（戻り値は BGR `(H,W,3) uint8` のみ、`render_mode="RGB"`）。feat-015 のレンダリングテストは GPU 手動テスト扱いだが、`main` 経由のスタブ署名互換は 8章で担保する
- `return_depth=True` のとき:
  - `render_mode="RGB+ED"` を指定 → `rendered` が `(1,H,W,4)`（`[...,:3]`=RGB、`[...,3]`=expected depth）、`alphas` は `(1,H,W,1)`
  - 背景合成は従来同様 `rgb = rendered[...,:3]` に対して `(rgb + (1-alphas)*bg).clamp(0,1)` → BGR化
  - 戻り値 `(bgr, depth_map, alpha_map)`:
    - `depth_map`: `rendered.squeeze(0)[..., 3].cpu().numpy()` → `(H,W) float32`
    - `alpha_map`: `np.squeeze(alphas.squeeze(0).cpu().numpy())` → `(H,W) float32`。**αの shape 揺れ（`(1,H,W,1)`/`(1,H,W)`）を `np.squeeze` で正規化**
- `[...,:3]` のスライスは RGB（4ch）でも RGB-only（3ch）でも先頭3chを取るので、共通コードにできる
- 深度はカメラ光軸方向のz深度（gsplat の expected depth = `Σ(α·z)/Σα`）。キーポイントのカメラ深度と同じ単位
- CUDA依存 → 手動テスト（深度・α取得の正しさは実機で確認）

### 5.4 `compute_keypoint_depth(kpts_calib: np.ndarray, cam: dict) -> np.ndarray` （復元）
- 各キーポイントのカメラ深度を返す。`R = cv2.Rodrigues(cam["rvec"])[0]` として `(R @ X + t)` のZ成分。入力 `(...,3)` → 出力 `(...,)`
- bpy/CUDA非依存 → テスト対象

### 5.5 `load_c3d_first_frame(c3d_path: str) -> tuple[list[str], np.ndarray, np.ndarray]` 【新設】
- C3Dを開き、**先頭フレームのみ**読んで `(labels, data, residual)` を返す:
  - `labels`: マーカー名リスト（`reader.point_labels`、strip済み）
  - `data`: shape `(n_markers, 3)` の先頭フレーム生座標（mm）
  - `residual`: shape `(n_markers,)`。py-c3d の `points[:, 3]`。`residual < 0` は無効サンプル
- 実装方針: `reader = c3d.Reader(open(path, "rb"))`、`for frame_no, points, analog in reader.read_frames(): return ...`（**最初の1フレームで break/return**。ジェネレータを全消費しない）
- `c3d` は関数内 import（phase4のみ）
- フレームが0件のC3D → `ValueError`（メッセージにパス）。8章
- API版差は確認済み（py-c3d 0.6.0: `point_labels` プロパティ、`read_frames` が `(frame_no, points(N,5), analog)` を yield）。実装時に2行REPLで再確認する

### 5.6 `c3d_to_calib(points_mm: np.ndarray) -> np.ndarray` （復元・中核）
- 入力 `(...,3)` の `(px,py,pz)` mm → `np.stack([pz, px, py], axis=-1) * 0.001`（m）
- 根拠: Pose2Sim の `zup_to_yup`（新=(Y,Z,X)）の逆変換。元X=新Z(pz), 元Y=新X(px), 元Z=新Y(py)。実データで検証済み
- bpy/CUDA非依存 → 重点テスト対象

### 5.7 `extract_halpe26(labels, data, residual) -> tuple[np.ndarray, np.ndarray]` （復元）
- `labels` から `HALPE26_NAMES` の各名のインデックスを引き、`data[idx]` を Halpe26順に並べた `(26,3)` を返す
- 有効フラグ `valid` `(26,)` も返す。**有効 = `residual[idx] >= 0` かつ 座標にNaNを含まない**。「全成分0」は判定基準にしない
- C3DにHalpe26名が欠ける場合は不足名を表示して `ValueError`
- 先頭フレーム1枚向けに `(26,3)`/`(26,)`（feat-013 の `(n_frames,26,3)` から1フレーム版に簡約）
- テスト対象

### 5.8 `project_keypoints(kpts_calib: np.ndarray, cam: dict) -> np.ndarray` （復元）
- `(26,3)` を `cv2.projectPoints(kpts_calib, cam["rvec"], cam["tvec"], cam["K"], distCoeffs=None)` で `(26,2)` に投影（**歪みなし＝ピンホール**。背景3DGSと整合）
- NaN座標は `np.nan_to_num` でサニタイズしてから渡す（無効点は描画側で `valid`/可視性によりスキップ）
- テスト対象（実データ依存・skipif）

### 5.9 `compute_visibility(pts2d, depth_cam, valid, depth_map, alpha_map, margin, near_plane) -> np.ndarray` （復元＋背面ガード追加）
- 各点（キーポイント or ボーンサンプル点）の可視性 `(n,) bool` を返す。判定（点 i）:
  1. `valid[i]` が False → `False`
  2. **`depth_cam[i]` が非finite（NaN/inf）、または `depth_cam[i] <= near_plane`（カメラ背面・至近）→ `False`**（カメラ後方の点が `cv2.projectPoints` で前方に折り返って投影され誤描画されるのを防ぐ）
  3. 投影位置が画像外（`0<=col<W` かつ `0<=row<H` を満たさない）→ `True`（隠す3DGSが画面内に無い。描画はOpenCVクリッピングに委ねる）
  4. `alpha_map[row, col] < ALPHA_THRESH`（3DGSが無い/薄い画素）→ `True`
  5. `depth_cam[i] > depth_map[row, col] + margin`（キーポイントが3DGSより奥）→ `False`（隠れる）
  6. それ以外 → `True`
- **判定順序の根拠（順序を変えないこと）**: ①valid → ②深度有効性（finite かつ > near_plane）→ ③画像外 → ④α → ⑤深度比較。背面ガード（②）は投影座標を信頼する前に置く（背面点の投影座標は無意味なため画像外判定より先）。gsplat の expected depth は低α画素で不安定（0付近〜巨大値）なので、**α判定（④）を深度比較（⑤）より必ず先**に置き、低α画素を深度比較に到達させない
- `near_plane` は `main` から `--near-plane`（既定 0.1）の値を渡す。レンダリングの near クリップと整合させ、レンダリングで除外された至近の点をキーポイント側でも不可視にする
- **画素インデックス規約**: gsplat出力は `[row=v(縦), col=u(横)]`。投影座標 `(u,v)` を `col=int(round(u))`, `row=int(round(v))` で丸めて参照（u/v 取り違え注意）。配列参照前に必ず境界判定して IndexError を防ぐ
- `margin` は `--occlusion-margin`（既定 `OCCLUSION_MARGIN`=0.05）。`ALPHA_THRESH`=0.5 は内部定数（CLI非公開）
- bpy/CUDA非依存 → テスト対象（5分岐＋境界・丸め）

### 5.10 `draw_overlay(image, kpts_calib, pts2d, valid, kp_visible, cam, depth_map, alpha_map, margin, near_plane, occlusion=True) -> np.ndarray` （復元）
- `image`（H,W,3 uint8 BGR）のコピーに、Halpe26 の点とボーンをオクルージョン考慮で描画して返す
- **点**: `occlusion=True` 時は `valid[i] and kp_visible[i]` の点のみ円（黄、半径 `POINT_RADIUS`）。`occlusion=False` 時は `valid[i]` のみ
- **ボーン（部分隠蔽）**: 両端が `valid` の辺について、両端キーポイントの**3D座標**（calib）を `BONE_SAMPLES`(24) 個に線形補間 → 5.8相当で投影＋5.4相当で深度算出 → 5.9 `compute_visibility(..., near_plane)` をサンプル列に適用（補間サンプルは C3D由来 valid が無いので `valid=np.ones(BONE_SAMPLES, bool)`。背面ガードは `compute_visibility` 内の深度判定に委ねる）→ **可視な隣接サンプルのみ短い線分で結ぶ**。これにより障害物を横切るボーンが途中で切れる
  - `occlusion=False` 時はサンプル可視判定をスキップし、両端valid辺を全描画（旧2D重ね描き）。`depth_map`/`alpha_map` は使わない（`None` 許容）
- 部位色（R=赤/L=青/C=緑）を維持。左右は被験者の解剖学的左右
- テスト対象（小画像で例外なく描画、`valid`/`kp_visible` False の点・ボーンが描かれない）

### 5.11 `main()` / `_build_parser()`
- **通常時（オクルージョンあり）**: 引数解析 → カメラ選択（torch ロード前に早期検証）→ C3Dロード・Halpe26抽出・calib変換 → PLYロード → `render_image(return_depth=True)`（背景BGR+深度+α）→ `project_keypoints`（歪みなし2D）→ `compute_keypoint_depth`（カメラ深度）→ `compute_visibility(..., near_plane)`（点の可視性）→ `draw_overlay(..., depth_map, alpha_map, margin, near_plane, occlusion=True)` → PNG保存
- **`--no-occlusion` 時**: `render_image(return_depth=False)` を呼び（**深度・αを生成しない**＝深度レンダリング経路を通さない）、背景BGRのみ取得 → `compute_keypoint_depth`/`compute_visibility` は呼ばず → `draw_overlay(..., depth_map=None, alpha_map=None, margin, near_plane, occlusion=False)`（`draw_overlay` は `occlusion=False` 時 `depth_map`/`alpha_map` を参照しないので `None` 可）
- PNG保存は feat-015 と同一（`cv2.imwrite` の False/`cv2.error` 両系統を扱う、空dirnameガード）
- 出力先 `--output` 省略時は `./data/keypoints_<カメラ名>.png`

## 6. CLI 仕様

```
uv run python render_keypoints.py <ply> <toml> <c3d> --camera <名前> [options]
```

| 引数 | 必須 | デフォルト | 説明 |
|---|---|---|---|
| `ply_path` | Yes | - | 3DGS PLYパス |
| `toml_path` | Yes | - | キャリブTOMLパス |
| `c3d_path` | Yes | - | キーポイントC3Dパス（**新規・必須位置引数**） |
| `--camera` | Yes | - | TOML内の対象カメラ名 |
| `--near-plane` | No | 0.1 | near クリップ距離[m]（floater除去） |
| `--output` | No | `./data/keypoints_<カメラ名>.png` | 出力PNGパス |
| `--background` | No | `0 0 0` | 背景色RGB[0-1] |
| `--no-occlusion` | No | False | オクルージョン無効（全点を手前に描く＝旧2D重ね描き。比較用） |
| `--occlusion-margin` | No | `OCCLUSION_MARGIN`(0.05) | 深度マージン[m]（argparse default は定数を参照し二重管理を避ける） |

実行例:
```bash
cd phase4
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    data/Blender/<keypoints>.c3d \
    --camera cam41520554 --near-plane 0.5 --output /tmp/keypoints.png
```

## 7. 定数（`HALPE26_NAMES`/`HALPE26_SKELETON`/色）

feat-013 design.md 4章のスニペットをそのまま用いる（26名・スケルトン辺・色定義・閾値）。要点:
- `HALPE26_NAMES`: 26点（Hip, RHip, ... LWrist）
- `NAME_TO_IDX = {name: i}`
- `HALPE26_SKELETON`: `(始点名, 終点名, 部位 "R"/"L"/"C")` の辺リスト
- 色（BGR）: `COLOR_RIGHT=(0,0,255)` 赤 / `COLOR_LEFT=(255,0,0)` 青 / `COLOR_CENTER=(0,255,0)` 緑 / `POINT_COLOR=(0,255,255)` 黄
- `POINT_RADIUS=4`, `LINE_THICKNESS=2`, `BONE_SAMPLES=24`, `OCCLUSION_MARGIN=0.05`, `ALPHA_THRESH=0.5`

## 8. テスト設計

### 8.1 新規 `tests/test_feat016_keypoints.py`（CUDA非依存）
純粋ロジックを合成データで検証する。`c3d`/`torch`/`gsplat` は関数内 import なので import 不要（`load_c3d_first_frame` のテストのみ実データ skipif または合成C3D書き出し）。

- `c3d_to_calib`: 既知 `(px,py,pz)` → `(pz,px,py)*0.001`（スカラ・配列・複数点形状）
- `compute_keypoint_depth`: 既知 rvec/tvec/点で `(R·X+t)` のZ成分一致（恒等回転で `t_z + X_z`）
- `extract_halpe26`: 合成 labels/data/residual で26点をHalpe26順抽出、`residual<0`→valid False、NaN座標→valid False、Halpe26名不足→ValueError
- `compute_visibility`: 合成 depth_map/alpha_map で分岐（invalid→False、**深度≤near_plane→False、深度NaN/inf→False**、枠外→True、低α→True、奥→False、手前→True）＋境界（`col==W`/`row==H`/負）＋丸め＋判定順序（背面ガードが画像外より先、α判定が深度比較より先）
- `draw_overlay`: 小画像で例外なく描画、`occlusion=True` で `kp_visible=False` の点・ボーンが描かれない、`occlusion=False` で `valid` の点が描かれる
- `project_keypoints`（**実データ依存・skipif**）: 実TOML1カメラ＋実C3D先頭フレームで、26点中大半（>=20 目安、実機確認で再設定）が画像枠内
- 結果を `tests/results/feat-016_test_result.txt` に保存

### 8.2 既存 `tests/test_feat015_render.py` の更新（後方互換の担保）
`c3d_path` 必須化と `render_image(return_depth=True)` 呼び出しに伴い、`main` を呼ぶ3テストを更新する（純粋ロジックテスト＝`camera_to_viewmat`/`load_cameras_toml`/`select_camera` は無変更）:
- `test_imwrite_false_returns_1` / `test_imwrite_cv2error_returns_1`: argv に `c3d_path`（合成C3D or skipif）を追加。`_install_render_stub` の `render_image` スタブ署名を `(..., return_depth=False)` 受け取りに変更し、`return_depth=True` 時は `(bgr, depth_map, alpha_map)` を返すスタブにする。C3Dロード・抽出・投影もスタブ/合成で通す
- `test_missing_camera_returns_1`: カメラ選択は torch ロード前なので、`c3d_path` を argv に足すだけで通る（C3Dロード前に return 1 する設計を維持）
- **設計判断**: feat-015 main テストの「13件不変」は厳密には成立しない（必須位置引数追加のため）。純粋ロジックテストは不変に保ち、main 系3件のみ最小修正する。`render_image` の `return_depth` 既定 False による**関数レベルの後方互換**は維持する

### 8.3 手動テスト（実機GPU、ユーザー実施）
- `TORCH_CUDA_ARCH_LIST="9.0+PTX"` で実データをレンダリングし、人体キーポイントが3DGS点群上の妥当な位置・姿勢に乗ること
- **オクルージョン確認**: カメラ手前の障害物に人体が差し掛かる箇所で、隠れるべき点・ボーンが消えること。`--no-occlusion` との比較で効果を確認
- **閾値調整**: `near_plane`（floater除去。0.01→0.5）、`ALPHA_THRESH`(0.5)、`OCCLUSION_MARGIN`(0.05) の妥当性、点半径・線太さ・色の視認性を確認・調整
- 深度・αの単位/shape/値域を1点照合（既知キーポイント深度 vs depth_map）

## 9. エラー処理

| 状況 | 動作 |
|---|---|
| `--camera` がTOMLに無い | カメラ名一覧をstderr表示、終了コード1（torch ロード前に検出） |
| C3DにHalpe26名が不足 | 不足名をstderr表示、終了コード1 |
| C3Dにフレームが0件 | メッセージ表示、`ValueError`／終了コード1 |
| 欠損キーポイント（residual<0/NaN） | 当該点・接続ボーンを描画スキップ（クラッシュしない） |
| 手前の3DGSに隠れるキーポイント | オクルージョン判定で描画スキップ（クラッシュしない） |
| PNG保存失敗 | `cv2.imwrite` False/`cv2.error` 両系統を扱い、stderr表示・終了コード1 |

## 10. 境界条件

- 全キーポイントが無効（valid 全 False）→ 点・ボーンとも描画されず背景のみのPNG（クラッシュしない）
- 全キーポイントが画像外 → `compute_visibility` で全て可視扱い、`draw_overlay` は OpenCV クリッピングで何も描かない
- αマップが全て低α（3DGSが薄い）→ 全点可視（隠すものが無い）

## 11. 設計判断の記録（ADR簡易版）

- **採用: 先頭フレーム1枚 / PNG1枚に限定**。却下: 連番PNG・MP4・フレーム範囲・dry-run・`--no-keypoints`。理由: feat-013 が全部入りで頓挫。中核（投影＋オクルージョン）に絞り、複数フレーム化は次案件に分割
- **採用: 背景判定をαマップ（accumulation）で行う**。却下: 深度==0 を背景条件にする。理由: `RGB+ED` の expected depth はα加重期待値で、背景画素でも小さな値が出うるため誤判定する。αで「3DGSが無い/薄い」を判定するのが頑健
- **採用: `render_image` に `return_depth` 引数を追加（既定 False）**。却下: 別関数 `render_background` を新設。理由: feat-015 の `render_image` と処理がほぼ同一で、引数追加の方が重複が少なく後方互換も取りやすい
- **採用: ピンホール統一（歪みなし）**。却下: gsplat UT経路で歪み再現。理由: feat-013/015 で黒い靄・品質劣化
- **採用: `ALPHA_THRESH`(0.5) は内部定数（CLI非公開）**。却下: CLIオプション化。理由: 調整頻度が低くCLIを増やしたくない。`--occlusion-margin` のみ公開

## 12. CLAUDE.md への影響（完了時に更新）
- phase4 セクションの `render_keypoints.py` 実行例にキーポイント重ね描き・オクルージョンを追記
- ディレクトリ構成の `render_keypoints.py` 説明から「feat-016で…予定」を削除し実態に更新
- 完了済み案件リストに feat-016 を追加

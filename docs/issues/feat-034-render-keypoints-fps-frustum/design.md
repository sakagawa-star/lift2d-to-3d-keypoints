# feat-034 機能設計書: render_keypoints.py FPSカメラ視錐台ワイヤフレームの重ね描き

作成日: 2026-08-13
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 視錐台ワイヤフレーム描画 | 4.2, 4.4, 4.5 |
| FR-002 FPSポーズ計算（正本再利用） | 4.1, 4.3 |
| FR-003 FPSカメラの K・解像度取得 | 4.2, 4.6 |
| FR-004 錐台奥行きの指定 | 4.2, 4.6 |
| FR-005 オクルージョンの反映 | 4.4 |
| FR-006 設定YAML対応 | 4.7 |
| FR-007 引数・入力の検証 | 4.6 |
| FR-008 実行サマリの表示 | 4.5, 5 |
| FR-009 既定動作の不変 | 4.5, 4.6, 7(D-3) |

## 2. システム構成

### モジュール構成

変更は `phase4/render_keypoints.py` 1ファイルのみ。`render_fps_video.py` は import 元として使うだけで変更しない。

| ファイル | 変更内容 |
|---|---|
| `phase4/render_keypoints.py` | 定数3個・関数3個の追加、既存 import 文への追記、`_build_parser` / `CONFIG_CONVERTERS` / `main` への追記、モジュール docstring 更新 |
| `tests/test_feat034_fps_frustum.py` | 新規テスト |

### 依存関係

```
render_keypoints.py
  └─ render_fps_video.py（既存 import に追加: compute_fps_poses, HEAD_JOINT_NAMES）
       ※ render_fps_video.py のトップレベル import は軽量（torch 非依存）であることを
         feat-033 で確認済み。循環 import なし（render_fps_video.load_intrinsics は
         関数内 import で render_keypoints を参照するため、トップレベルの循環は生じない）
```

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / OpenCV / tomli / gsplat 経路）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 座標系と前提（FR-002）

- キーポイント入力はフレームループと同じ経路（`extract_keypoints` → `c3d_to_calib`）でキャリブ座標 [m] に変換して使う。キャリブ座標は NPZ の world 座標と恒等（feat-018/032 で確認済み）なので、world 座標を前提とする `compute_fps_poses()` にそのまま渡せる
- `compute_fps_poses(x3d_world, head_idx)` の引数仕様: `x3d_world` は (F, J, 3) float64、`head_idx` は頭部7点名→J軸インデックスの辞書。本機能では J=7 の専用配列を組み、`head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}` を渡す
- 戻り値 `c2w_b` は Blenderカメラ規約（カメラは −Z を向く、並進 = eyes_mid）の (F,4,4)。無効フレームは全要素 NaN、`valid` (F,) bool が併せて返る。`invalid_reasons` は使用しない（FR-008 のとおり件数のみ表示するため）

### 4.2 新規関数の設計

#### 定数（モジュールレベル、既存の色定数群の直後に追加）

```python
# 意図伝達用（そのままコピーしない）
FRUSTUM_COLOR = (255, 0, 255)   # マゼンタ (BGR)。スケルトンの赤/青/緑/黄と区別する
FRUSTUM_DEPTH_DEFAULT = 0.5     # 錐台奥行きの既定値 [m]
# 視錐台の8線分。頂点インデックス 0=視点(apex), 1..4=遠端4隅
FRUSTUM_EDGES = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
```

#### `collect_head_points(labels, frames_data) -> np.ndarray`

全フレームの頭部7点をキャリブ座標で集める。

- **入力**: `labels`（マーカー名リスト）、`frames_data`（フレーム範囲フィルタ**後**の list[dict]。各要素は `frame_no` / `data` / `residual` を持つ既存構造）
- **出力**: (F, 7, 3) float64、キャリブ座標 [m]。第2軸の順序は `HEAD_JOINT_NAMES` と同一
- **処理ロジック**:
  1. フレームごとに `extract_keypoints(labels, fr["data"], fr["residual"])` → `c3d_to_calib()` を適用する（フレームループと同一経路。二重呼び出しになるが 28点の辞書引きのみで無視できる軽さ。7(D-2) 参照）
  2. `HEAD_JOINT_NAMES` の各マーカーについて、`valid[NAME_TO_IDX[name]]` が False の点は座標を `np.nan` に置き換える（無効判定を `compute_fps_poses()` に一元化するため。7(D-4)）
- **境界条件**: F=0 は呼び出し前に既存のフレーム範囲チェックで弾かれているため考慮不要。頭部7点が `labels` に無いケースは 4.6 の検証で事前に弾く

#### `compute_frustum_vertices(c2w_b, K_fps, width, height, depth) -> np.ndarray`

FPSカメラの c2w_b から視錐台の5頂点（キャリブ座標）を全フレーム一括で計算する。

- **入力**:
  - `c2w_b`: (F,4,4) float64。`compute_fps_poses()` の戻り値（無効フレームは NaN）
  - `K_fps`: (3,3) float64。FPSカメラの内部パラメータ
  - `width`, `height`: int。FPSカメラの解像度
  - `depth`: float。有限かつ > 0（呼び出し前に 4.6 の検証3で保証済み）。錐台奥行き [m]
- **出力**: (F,5,3) float64。インデックス0=視点（eyes_mid）、1..4=遠端4隅。無効フレームは NaN（c2w_b の NaN が伝播する）
- **処理ロジック**（全フレーム一括のベクトル化。擬似コード、そのままコピーしない）:

```python
corners_px = [(0, 0), (width, 0), (width, height), (0, height)]  # 左上から時計回り
dirs_cv = [((u - cx) / fx, (v - cy) / fy, 1.0) for (u, v) in corners_px]  # (4,3)
convert = np.diag([1.0, -1.0, -1.0, 1.0])   # Blender→OpenCV。c2w_blender_to_viewmats と同一式
with np.errstate(invalid="ignore"):
    c2w_cv = c2w_b @ convert                 # (F,4,4)
    R_cv = c2w_cv[:, :3, :3]
    apex = c2w_cv[:, :3, 3]                  # = eyes_mid (F,3)
    corners = apex[:, None, :] + depth * np.einsum("fij,cj->fci", R_cv, dirs_cv)  # (F,4,3)
    return np.concatenate([apex[:, None, :], corners], axis=1)  # (F,5,3)
```

- 方向ベクトルの z 成分（OpenCVカメラ座標）を 1 に正規化してから `depth` 倍するため、遠端4隅は「光軸方向距離 = depth の平面」上に乗る（斜距離ではない）
- 歪み係数は使わない（ピンホール。FR-003）

#### `draw_frustum(image, verts_calib, cam, depth_map, alpha_map, margin, near_plane, occlusion=True) -> np.ndarray`

視錐台の8線分を観察カメラに投影してオクルージョン考慮で描画する。

- **入力**:
  - `image`: (H,W,3) uint8 BGR。背景画像（変更しない。コピーに描画）
  - `verts_calib`: (5,3) float64。`compute_frustum_vertices` の1フレーム分。**有限値のみ**（無効フレームは呼び出し側でスキップする）
  - `cam`: 観察カメラの辞書（既存 `select_camera` の戻り値）
  - `depth_map`, `alpha_map`, `margin`, `near_plane`, `occlusion`: 既存 `draw_overlay` と同じ意味。`occlusion=False` のとき `depth_map` / `alpha_map` は None 可
- **出力**: 描画済み画像（入力のコピー）
- **処理ロジック**: 既存 `draw_overlay` のボーン描画と同一方式。`FRUSTUM_EDGES` の各線分について:
  - `occlusion=False`: 両端を `project_keypoints` で投影し `cv2.line` で直接描く
  - `occlusion=True`: 3D線分を `BONE_SAMPLES`（=24）点に線形補間 → `project_keypoints` / `compute_keypoint_depth` / `compute_visibility`（valid は全 True）→ 可視な隣接サンプル間のみ `cv2.line` で描く
  - 色 `FRUSTUM_COLOR`、太さ `LINE_THICKNESS`。`compute_visibility` の判定順序・`ALPHA_THRESH` は既存のまま流用する
- **既存コードとの関係**: `draw_overlay` は変更しない（7(D-3)）。サンプリング描画の約10行は `draw_frustum` 内に持つ（共通化リファクタはしない）

### 4.3 main の処理フロー変更（FR-001/002）

`main()` への追記箇所を処理順に列挙する（番号は挿入位置の説明であり実装の行番号ではない）:

1. **引数検証**（4.6。既存の `--no-keypoints` 組み合わせ検証ブロックに追記＋直後に新規ブロック）
2. **`--frustum-depth` の既定値解決**（既存の `occlusion_margin` 既定値解決の直後）: `args.frustum_depth` が None なら `FRUSTUM_DEPTH_DEFAULT` を代入する
3. **FPSカメラ選択**（既存の観察カメラ `select_camera` の直後）: `args.fps_frustum` のとき実行する。検索先辞書は `args.fps_toml` が None なら既存の `cameras` 辞書、指定時は `load_cameras_toml(args.fps_toml)` で別途読み込んだ辞書とする（イテレーション1 で追加）。`select_camera(検索先辞書, args.fps_camera)` の `ValueError` は既存と同様に stderr 表示して return 1

```python
# 意図伝達用の擬似コード（そのままコピーしない）
fps_cam = None
if args.fps_frustum:
    if args.fps_toml is None:
        fps_cameras = cameras                       # 従来どおり toml_path の辞書
    else:
        try:
            fps_cameras = load_cameras_toml(args.fps_toml)
        except FileNotFoundError:
            print(f"エラー: --fps-toml のファイルが見つかりません: {args.fps_toml}", file=sys.stderr)
            return 1
        except tomli.TOMLDecodeError as e:
            print(f"エラー: --fps-toml をTOMLとして解釈できません: {args.fps_toml}: {e}", file=sys.stderr)
            return 1
    try:
        fps_cam = select_camera(fps_cameras, args.fps_camera)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
```
4. **頭部マーカー存在チェック**（既存の `present` 計算の直後、**`args.fps_frustum` が True の場合のみ**実行）: 4.6 の検証6。False の場合はチェック自体を行わない（既存の欠損許容の挙動を変えない）
5. **視錐台の事前計算**（フレーム範囲フィルタの**後**、`from render import load_ply` の**前**。フィルタ後の `frames_data` とインデックスを揃えるため）:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
frustum_verts = None
frustum_valid = None
if args.fps_frustum:
    print(f"FPS視錐台: カメラ {fps_cam['name']} ({fps_cam['width']}x{fps_cam['height']}), "
          f"奥行き {args.frustum_depth} m")
    head_arr = collect_head_points(labels, frames_data)        # (F,7,3)
    head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}
    c2w_b, frustum_valid, _ = compute_fps_poses(head_arr, head_idx)
    frustum_verts = compute_frustum_vertices(
        c2w_b, fps_cam["K"], fps_cam["width"], fps_cam["height"], args.frustum_depth)
```

6. **フレームループ内の描画**（既存 `draw_overlay` 呼び出しの直前。描画順=視錐台が下、スケルトンが上）:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
base = bgr_bg
if frustum_verts is not None and frustum_valid[i]:
    base = draw_frustum(bgr_bg, frustum_verts[i], cam, depth_map, alpha_map,
                        args.occlusion_margin, args.near_plane, occlusion=occlusion)
overlay = draw_overlay(base, ...)   # 引数は既存のまま（image のみ base に変える）
```

7. **サマリ表示**（フレームループの `finally` 後、`rc != 0` チェックを通過した後の完了表示の直前）: 4.5 参照

### 4.4 オクルージョンの扱い（FR-005）

- `occlusion`（= `not args.no_occlusion`）をスケルトンと共有する。視錐台専用の切り替えは設けない
- `occlusion=True` の経路では、背景レンダリングで取得済みの `depth_map` / `alpha_map` をそのまま使う（追加のレンダリングなし）
- 視点（両目中点）が人物頭部の 3DGS 内にあるため、稜線の根元区間は `compute_visibility` の深度比較で不可視になり描かれない。これは仕様どおりの挙動（requirements FR-005）

### 4.5 出力・表示（FR-001/008/009）

- 出力ファイル（連番PNG / MP4）の形式・命名は一切変更しない。視錐台は既存フレーム画像への上書き描画のみ
- 開始時表示（`--fps-frustum` 時のみ1行）: `FPS視錐台: カメラ <名前> (<W>x<H>), 奥行き <depth> m`
- 終了時表示（`--fps-frustum` 時のみ、完了表示の直前）:
  - 全フレーム描画時: `FPS視錐台: N/N フレームに描画`
  - 非描画あり: `FPS視錐台: X/N フレームに描画（頭部キーポイント欠損・縮退により Y フレームは非描画）`
  - X = `int(np.count_nonzero(frustum_valid))`、N = `len(frames_data)`、Y = N − X
- `--fps-frustum` なしの場合、実行時の標準出力・出力ファイルとも従来と完全同一（追加表示なし）。ただし argparse が生成する usage 行・`--help` 表示・設定YAMLの未知キーエラーのキー一覧に新オプション（`--fps-frustum` / `--fps-camera` / `--fps-toml` / `--frustum-depth`）が追加されることは許容する（requirements FR-009 の対象外事項）

### 4.6 CLI・引数検証（FR-003/004/007/009）

#### `_build_parser()` への追加（既存 `--config` の直前に3本追加）

| オプション | 型・アクション | デフォルト | help 要旨 |
|---|---|---|---|
| `--fps-frustum` | `store_true` | False | FPSカメラ（頭部キーポイント由来）の視錐台ワイヤフレームをマゼンタで重ね描き（`--fps-camera` 必須） |
| `--fps-camera` | str | None | 視錐台のFOVに使うTOML内カメラ名（`--fps-frustum` 専用） |
| `--fps-toml` | str | None | `--fps-camera` を検索するTOMLファイル（`--fps-frustum` 専用、省略時: 位置引数 toml_path と同じファイル）（イテレーション1 で追加） |
| `--frustum-depth` | float | None | 視錐台の奥行き[m]（`--fps-frustum` 専用、default: 0.5） |

`--frustum-depth` / `--fps-camera` のデフォルトを None にするのは「`--fps-frustum` なしでの指定」を検出するため（既存 `--occlusion-margin` と同じパターン）。既定値 0.5 は main で解決する（4.3-2）。

#### 検証（すべて重い処理＝PLY/torch ロードの前。requirements FR-007 の番号に対応）

| # | 条件 | 検出方法・処理 | 終了コード |
|---|---|---|---|
| 4 | `--no-keypoints` と `--fps-frustum`/`--fps-camera`/`--fps-toml`/`--frustum-depth` の併用 | 既存の `forbidden` リストに4タプル追加: `("--fps-frustum", args.fps_frustum)`, `("--fps-camera", args.fps_camera is not None)`, `("--fps-toml", args.fps_toml is not None)`, `("--frustum-depth", args.frustum_depth is not None)` → 既存の `parser.error` | 2 |
| 1 | `--fps-frustum` あり・`--fps-camera` なし | `parser.error("--fps-frustum 指定時は --fps-camera を指定してください")` | 2 |
| 3 | `--frustum-depth` が 0 以下または非有限（nan / inf / -inf） | 判定は「`args.frustum_depth is not None` かつ（`not np.isfinite(args.frustum_depth)` または `args.frustum_depth <= 0`）」（NumPy は import 済みのため新規 import 不要）→ `parser.error("--frustum-depth は 0 より大きい有限の値を指定してください")` | 2 |
| 2 | `--fps-frustum` なしで `--fps-camera` / `--fps-toml` / `--frustum-depth` 指定 | `parser.error("<オプション名> は --fps-frustum と併用してください")` | 2 |
| 5 | `--fps-camera` が検索先 TOML（`--fps-toml` 指定時はそのファイル、省略時は `toml_path`）に存在しない | `select_camera` の `ValueError` を捕捉し stderr 表示（既存の観察カメラと同じ形式） | 1 |
| 5b | `--fps-toml` のファイルが存在しない・TOMLとして解釈できない | 4.3-3 の擬似コードのとおり `FileNotFoundError` / `tomli.TOMLDecodeError` を捕捉し、パスを含むメッセージを stderr 表示 | 1 |
| 6 | **`args.fps_frustum` が True の場合のみ**: 頭部7点のいずれかが `labels` に無い | `missing_head = [n for n in HEAD_JOINT_NAMES if n not in labels]` が空でなければ `頭部マーカーが入力にありません（--fps-frustum に必須）: <名前列挙>` を stderr 表示。False の場合はチェックしない（既存の欠損許容を維持） | 1 |

- 検証 1〜3 は既存の `--no-keypoints` 組み合わせ検証ブロックの直後に新規ブロックとして置く（4 は既存ブロック内）。検証 2 の判定は「`args.fps_frustum` が False かつ（`args.fps_camera is not None` または `args.fps_toml is not None` または `args.frustum_depth is not None`）」
- 検証 6 は「マーカー構成に無い」場合のみ。構成にはあるがフレーム単位で NaN・residual<0 の欠損は無効フレームとして継続する（FR-002）

### 4.7 設定YAML（FR-006）

`CONFIG_CONVERTERS` に4キーを追加する（feat-033 の仕組みに乗るため、これ以外の変更は不要）:

```python
"fps_frustum": _yaml_bool, "fps_camera": str, "fps_toml": str, "frustum_depth": float,
```

- 優先順位（CLI 明示指定 > 設定YAML > 既定値）、フラグ `fps_frustum` の true 方向のみ上書きの非対称性、型変換エラーの扱いは feat-033 と同一（追加実装なし。既存機構が処理する）
- 検証（4.6）は統合後の値に対して行うため、YAML 由来の指定も CLI 指定と同じ扱いになる

### 4.8 エラーハンドリング一覧

| エラー | 検出 | 処理 |
|---|---|---|
| 引数の組み合わせ誤り（4.6 の 1〜4） | main 冒頭の検証 | `parser.error` → 終了コード 2 |
| `--fps-toml` のファイル不存在・パース不能 | `load_cameras_toml` の FileNotFoundError / tomli.TOMLDecodeError | stderr 表示 → return 1（4.3-3） |
| FPSカメラ名が検索先 TOML に無い | `select_camera` の ValueError | stderr 表示 → return 1 |
| 頭部マーカーが構成に無い（`args.fps_frustum` が True の場合のみ検査） | labels との突き合わせ | stderr 表示 → return 1 |
| フレーム単位の頭部点欠損・縮退・回転検証失敗 | `compute_fps_poses` の valid | 該当フレームの視錐台のみ非描画で継続。終了時にサマリ表示 |
| 全フレーム無効 | 同上 | エラーにしない（`FPS視錐台: 0/N フレームに描画（...）` と表示して正常終了） |

### 4.9 境界条件

- **フレーム数1**: 通常どおり動作（事前計算は F=1 のベクトル化呼び出し）
- **視錐台が画面外**: `project_keypoints` は画面外座標を返し、`compute_visibility` は画面外サンプルを可視扱い、`cv2.line` のクリッピングに委ねる（既存ボーンと同じ。追加処理なし）
- **視点が観察カメラの背後**: `compute_visibility` の背面ガード（`depth_cam <= near_plane` → 不可視）が効く（既存ロジックのまま）。`occlusion=False` 時は既存ボーン同様に背面点も投影・描画される（既存の `--no-occlusion` の既知の性質に従う。専用対策はしない）
- **`--frustum-depth` が極端に大きい（例: 100）**: エラーにしない（遠端が画面外に出るだけで、稜線のクリッピングで破綻しない）

## 5. ログ・デバッグ設計

- 追加する標準出力は 4.5 の開始時1行・終了時1行のみ。フレームごとのログは追加しない（既存の進捗表示を変えない）
- ログレベル機構は導入しない（既存スクリプトと同じ print ベース）

## 6. テスト設計

`tests/test_feat034_fps_frustum.py` に以下を実装する。gsplat/torch を import しない範囲（幾何・描画・CLI検証・YAML）でテストする（既存 feat-032/033 テストと同じ方針。fixture・一時ファイルの作り方は `tests/test_feat032_npz_input.py` / `tests/test_feat033_config_yaml.py` の流儀に従う）。

| # | テスト | 期待値 |
|---|---|---|
| T-01 | `compute_frustum_vertices`: 恒等ポーズ（c2w_b=I）・fx=fy=100, cx=W/2, cy=H/2, W=200, H=100, depth=0.5 | apex=(0,0,0)。ピクセル(0,0)の隅 = 0.5·(−1.0, +0.5, −1.0)（OpenCV方向 ((0−100)/100, (0−50)/100, 1) の Blender→world 変換 (x,−y,−1) 倍） |
| T-02 | 同: apex が c2w_b の並進と一致 | 並進を (1,2,3) にして apex=(1,2,3) |
| T-03 | 同: 遠端4隅の「光軸方向距離」がすべて depth | 各隅を viewmat（`c2w_blender_to_viewmats`）でカメラ座標に戻し z=depth |
| T-04 | 同: 無効フレーム（c2w_b 全NaN） | 出力 (5,3) 全NaN |
| T-05 | 同: depth を2倍 | 遠端4隅の apex からの変位が2倍 |
| T-06 | CLI検証1: `--fps-frustum` のみ（`--fps-camera` なし） | SystemExit(2)、stderr に `--fps-camera` |
| T-07 | CLI検証2: `--fps-camera` のみ / `--frustum-depth` のみ | SystemExit(2)、stderr に `--fps-frustum` |
| T-08 | CLI検証3: `--frustum-depth 0` / 負値 / `nan` / `inf` / `-inf` | SystemExit(2) |
| T-09 | CLI検証4: `--no-keypoints --fps-frustum ...` | SystemExit(2)、既存形式のメッセージ |
| T-10 | CLI検証5: TOML に無い `--fps-camera` | main の戻り値 1、stderr にカメラ名一覧 |
| T-11 | CLI検証6: `--fps-frustum --fps-camera <cam>` 指定時に頭部マーカー（例: Nose）が無いキーポイント入力 | main の戻り値 1、stderr に欠損マーカー名 |
| T-11b | 同じ入力（Nose 無し）を `--fps-frustum` **なし**で実行 | 頭部マーカーエラーにならない（既存の欠損許容どおり描画スキップ扱いで処理が先へ進む） |
| T-12 | YAML: `fps_frustum: true` / `fps_camera` / `fps_toml` / `frustum_depth` の4キーが反映される | `parse_args_with_config` の Namespace で確認 |
| T-13 | YAML: CLI `--frustum-depth 1.0` が YAML `frustum_depth: 0.3` を上書き | args.frustum_depth == 1.0 |
| T-14 | `collect_head_points`: valid=False の頭部点が NaN 化され、`compute_fps_poses` で該当フレームが invalid になる | frustum_valid が False |
| T-15 | `draw_frustum` occlusion=False: 合成画像にマゼンタ画素が増える | FRUSTUM_COLOR の画素数 > 0 |
| T-16 | `draw_frustum` occlusion=True: 全サンプルが深度マップより奥（隠れる）→ 描画なし / 全サンプル手前 → 描画あり | マゼンタ画素数 0 / > 0 |
| T-17 | 既定動作: `--fps-frustum` なしの Namespace（fps_frustum=False, fps_camera=None, frustum_depth=None）| デフォルト値の確認 |
| T-18 | 回帰: 既存テスト全件 | `uv run pytest -v` 全件パス（feat-033 時点 425 passed / 1 skipped + 本案件追加分） |
| T-19 | FPSカメラの K・解像度が実際に使われる: K・解像度が異なる camA / camB を持つ TOML を用意し、`--camera camA --fps-camera camB` で main を実行（`compute_frustum_vertices` を monkeypatch で捕捉し、PLY ロード部はスタブ化して直後に中断） | `compute_frustum_vertices` に渡る K・width・height が camB のものと一致し、開始サマリに camB の名前・解像度が表示される |
| T-20 | `--fps-toml` の別ファイル取得（イテレーション1）: 観察カメラ TOML と別の TOML（K・解像度が異なるカメラ camC 入り）を用意し、`--fps-toml <別TOML> --fps-camera camC` で main を実行（T-19 と同方式の monkeypatch 捕捉） | `compute_frustum_vertices` に渡る K・width・height が camC のものと一致する |
| T-21 | CLI検証2（イテレーション1）: `--fps-toml` を `--fps-frustum` なしで指定 | SystemExit(2)、stderr に `--fps-frustum` |
| T-22 | CLI検証5b（イテレーション1）: `--fps-toml` に不存在パス / TOMLとして解釈できないファイル | いずれも main の戻り値 1、stderr にパス表示 |
| T-23 | YAML経由の `fps_toml`（イテレーション1）: 設定YAMLに `fps_toml: <別TOML>` と `fps_camera: camC` を書いて main を実行（T-20 と同方式の monkeypatch 捕捉） | `compute_frustum_vertices` に渡る K・width・height が別TOML側 camC のものと一致する |

- テスト結果は `tests/results/feat-034_test_result.txt` に `-v` 出力を保存する

### 実データでの動作確認（実装ステップ内で実施）

phase4/ ディレクトリで、feat-032/033 の動作確認と同じ session001 実データを使う:

```bash
# 動作確認用（フレーム範囲を絞って MP4 出力。パスは実データ環境に合わせる）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py --config data/run_keypoints.yaml \
    --fps-frustum --fps-camera <TOML内カメラ名> \
    --start-frame <小範囲> --end-frame <小範囲> --mp4 --output-dir data/feat034_check
```

確認項目:
1. 正常終了（終了コード 0）し、開始時・終了時サマリが表示される
2. 出力PNGにマゼンタ（BGR 255,0,255 近傍）の画素が存在する（頭部前方に錐台が見える）
3. `--fps-frustum` なしで同条件実行した出力と、なし側がバイト同一（既定動作の不変）

## 7. 設計判断（ADR 簡易版）

- **D-1 ポーズ計算は import 再利用**: `compute_fps_poses` / `HEAD_JOINT_NAMES` を `render_fps_video` から import する。複製実装は正本の二重管理になるため却下（feat-032/033 で確立したパターン。render_fps_video のトップレベルは torch 非依存で import 可）
- **D-2 事前一括計算**: FPSポーズと視錐台頂点はフレームループ前に全フレーム一括（ベクトル化）で計算する。ループ内で1フレームずつ `compute_fps_poses` を呼ぶ案は、同関数がベクトル化前提で書かれており呼び出しオーバーヘッドが無駄なため却下。`extract_keypoints` が事前計算とループで二重に呼ばれるが、28点の辞書引きのみで実測影響なし（共通化はループ構造の変更を伴い変更範囲が拡大するため却下）
- **D-3 `draw_overlay` は変更せず新関数 `draw_frustum` を追加**: ボーン描画と共通のサンプリング約10行を関数内に持つ。共通ヘルパー切り出し（draw_overlay のリファクタ）は既存機能の回帰リスクがあり、重複が小さいため却下
- **D-4 無効判定は NaN 化で `compute_fps_poses` に一元化**: フレーム単位の欠損（valid=False）を NaN に変換して渡し、無効判定（NaN・縮退・回転検証）を正本に委ねる。呼び出し側での独自判定は判定基準の二重管理になるため却下
- **D-5 奥行きは固定既定値 0.5m ＋ `--frustum-depth`**: K からは開き角しか決まらず奥行きは可視化上の任意選択（壁打ちで案a採用済み）。人体スケール連動はフレーム間で伸縮し視認性を損なうため却下
- **D-6 頭部マーカーの構成欠損は早期エラー**: 7点のいずれかがマーカー構成に無い場合、全フレームで視錐台が描けず `--fps-frustum` の指定が無意味になるため、重い処理前に終了コード 1 で弾く。警告して続行する案は「オプションを指定したのに何も描かれない」結果になり誤解を招くため却下。フレーム単位の欠損（構成にはある）は非描画継続
- **D-7 オクルージョンは `--no-occlusion` に連動**: 視錐台専用の切り替えオプションは CLI 肥大のため設けない（壁打ちで「オクルージョンあり」確定。比較用の無効化は既存フラグで足りる）
- **D-8 描画順は視錐台が下・スケルトンが上**: 主対象（人体）の視認性を優先する。2Dで重なる画素のみの差であり、どちらでも機能要件は満たすが実装判断ゼロの原則で固定する
- **D-9 色・太さは定数固定**: `FRUSTUM_COLOR`（マゼンタ）・`LINE_THICKNESS` はオプション化しない（要求にないため。変更したくなったら後続案件）
- **D-10（イテレーション1 で改訂） FPSカメラの K は既定で `toml_path`、`--fps-toml` で別 TOML を指定可**: 初版は「観察カメラと FPS カメラは同一キャリブファイルに入っている」前提で別TOML指定を設けない決定としたが、2026-08-13 の手動テストでヒアリング漏れ（別ファイルのケースがある）と指摘され、`--fps-toml`（省略時は `toml_path` にフォールバック＝後方互換）を追加した。`render_fps_video.load_intrinsics` は解像度偶数チェック（libx264 制約）を含み本機能には不要なため引き続き使わず、`load_cameras_toml` + `select_camera` で取る（省略時はロード済みの `cameras` 辞書を再利用し、同一ファイルの二重読み込みをしない）

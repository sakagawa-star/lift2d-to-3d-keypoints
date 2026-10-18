# feat-037 機能設計書: render_keypoints.py 複数人の3Dキーポイント重ね描き

作成日: 2026-10-18
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-037-render-keypoints-multi-person/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 複数ファイルの受け付け | 4.1, 4.3 |
| FR-002 複数人モードは C3D のみ | 4.2 |
| FR-003 対象フレーム（和集合） | 4.3, 4.4 |
| FR-004 point rate の一致検査 | 4.3 |
| FR-005 人物ごとのマーカー構成 | 4.3 |
| FR-006 人物ごとのオクルージョン | 4.6 |
| FR-007 奥の人物から描く | 4.5, 4.6 |
| FR-008 人物ラベル | 4.5, 4.6 |
| FR-009 FPS視錐台の対象人物 | 4.2, 4.4 |
| FR-010 設定YAML | 4.7 |
| FR-011 引数の検証 | 4.2, 4.3, 4.8 |
| FR-012 実行時の表示 | 4.3, 5 |
| FR-013 1人モードの後方互換 | 4.3, 4.6, 6, 7(D-1) |

## 2. システム構成

### 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `phase4/render_keypoints.py` | 定数（ラベル描画用）・関数4個の追加、`_build_parser` / `CONFIG_CONVERTERS` / `main` の変更、モジュール docstring への追記 |
| `tests/test_feat037_multi_person.py` | 新規テスト |
| `tests/test_feat033_config_yaml.py` | `keypoints_path` の型変更に伴う期待値の修正2か所（6 章） |

`render_fps_video.py` / `npz_to_c3d.py` / `render.py` は変更しない。

### 依存関係

既存と同じ（`render_keypoints.py` → `render_fps_video.py`〔YAML 機構・`compute_fps_poses`〕、関数内 import で `render` / `npz_to_c3d` / `c3d`）。新しい import は無い。

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / OpenCV / tomli / c3d / gsplat 経路。バージョンは `docs/TECH_STACK.md` のとおり）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 CLI の変更（FR-001, FR-009）

`_build_parser()` の変更:

1. 位置引数 `keypoints_path` を `nargs="?"` から `nargs="*"` に変える。`default=None` はそのまま。help を次の文言にする:
   `人体キーポイントファイルパス（1個以上。1ファイル=1人。2個以上の複数人指定は C3D のみ。1個のときは C3D または NPZ〔拡張子 .npz で NPZ と判別〕。Halpe26 + Spine/Thorax の既知28マーカー、欠損許容、全フレームを使用。--no-keypoints 時は省略）（--config でも指定可。設定YAMLでは ; 区切り）`
2. `--fps-person` を `--frustum-depth` の直後に追加する: `type=int, default=None`、help `視錐台を描く人物番号（1始まり、キーポイントファイルの指定順。--fps-frustum 専用、default: 1）`

`nargs="*"` の挙動（phase4 venv の Python 3.10.19 で 2026-10-18 に確認済み）:

| 入力 | `args.keypoints_path` |
|---|---|
| `a.ply b.toml c.c3d` | `["c.c3d"]` |
| `a.ply b.toml c.c3d d.c3d --camera x` | `["c.c3d", "d.c3d"]` |
| `a.ply b.toml`（キーポイントなし） | `[]`（default が None でも空リストになる） |
| 位置引数なし＋設定YAMLで `set_defaults(keypoints_path=[...])` | 設定YAMLのリスト |
| 設定YAMLのリストがあり、CLI で `c.c3d` | `["c.c3d"]`（丸ごと置き換え） |
| `a.ply b.toml c.c3d --camera x d.c3d` | argparse エラー（終了コード 2、`unrecognized arguments: d.c3d`） |

このため `main` 以降では `args.keypoints_path` を常に `list[str]` として扱い、「キーポイントなし」は `None` ではなく**空リスト**で判定する。

### 4.2 引数検証の変更（FR-002, FR-009, FR-011）

`main()` 冒頭の既存検証ブロックを次のように変える。すべて PLY / torch ロードの前、カメラ選択（`load_cameras_toml`）の前に行う。

| # | 変更箇所 | 変更後 | 終了コード |
|---|---|---|---|
| 1 | `if args.no_keypoints:` 内の `if args.keypoints_path is not None:` | `if args.keypoints_path:`（1個以上で真）。メッセージは既存のまま | 2 |
| 2 | 既存 `forbidden` リスト | 末尾に `("--fps-person", args.fps_person is not None)` を追加 | 2 |
| 3 | `else:` 内の `if args.keypoints_path is None:` | `if not args.keypoints_path:`（空リストで真）。メッセージは既存のまま | 2 |
| 4 | 既存の「`--fps-frustum` なしで fps 系オプション」ループ | 列挙に `("--fps-person", args.fps_person is not None)` を `--frustum-depth` の後に追加 | 2 |
| 5 | 新規（既存の `--frustum-depth` 値域検証の直後） | 下記擬似コードの NPZ 混在検査 | 2 |
| 6 | 新規（5 の直後） | 下記擬似コードの `--fps-person` 値域検査 | 2 |

```python
# 意図伝達用の擬似コード（そのままコピーしない）
n_persons = len(args.keypoints_path)
multi = n_persons >= 2
if multi:
    npz_paths = [p for p in args.keypoints_path if p.lower().endswith(".npz")]
    if npz_paths:
        parser.error("複数人（キーポイントファイル2個以上）の指定は C3D のみ対応です"
                     f"（NPZ: {', '.join(npz_paths)}）")
if args.fps_person is not None and not (1 <= args.fps_person <= n_persons):
    parser.error(f"--fps-person は 1〜{n_persons} の整数を指定してください"
                 f"（指定値: {args.fps_person}）")
```

- 6 の検査は、静止画モード（`--no-keypoints`）では 2 で、`--fps-frustum` なしでは 4 で先に終了しているため、到達するのは `--fps-frustum` ありの動画モードのみ。`n_persons` はここでは 1 以上
- 既定値の解決: 既存の `if args.frustum_depth is None:` の直後に `if args.fps_person is None: args.fps_person = 1` を追加する

### 4.3 キーポイント読み込みと人物レコード（FR-001, FR-003〜FR-005, FR-012, FR-013）

#### 人物レコード（`dict`）

| キー | 型 | 内容 |
|---|---|---|
| `index` | int | 人物番号（1始まり） |
| `path` | str | ファイルパス |
| `labels` | list[str] | マーカー名リスト（ローダの戻り値） |
| `frames` | list[dict \| None] | 対象フレーム列（4.4 の `frame_nos`）と同じ長さ・同じ並び。欠席は None。要素の dict はローダの frames_data 要素そのもの（`frame_no` / `data` / `residual`） |
| `present` | set[str] | 既知28マーカーのうち `labels` にあるもの |
| `skeleton` | list[tuple[str,str,str]] | `build_skeleton(present)` |

#### 1人モード（`multi` が False）の読み込み

既存コード（`is_npz` 判定〜「`{fmt}: {total_frames} フレーム, rate=...`」の表示まで）を、`args.keypoints_path` を `args.keypoints_path[0]` に置き換えるだけで残す。表示・エラー・判定の順序は既存のまま変えない（FR-013）。その後、人物レコードを1個作る:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
frame_nos = [fr["frame_no"] for fr in frames_data]      # ファイル内の並び順のまま（並べ替えない）
persons = [{"index": 1, "path": path, "labels": labels, "frames": list(frames_data),
            "present": present, "skeleton": skeleton}]
```

既存の頭部マーカー存在チェック（`--fps-frustum` 時）も既存の位置・メッセージのまま残す。

#### 複数人モード（`multi` が True）の読み込み

処理順（各段で失敗したらその場で終了する）:

1. 人物 k = 1..N の順に、ファイルごとに次を行う
   1. `print(f"P{k}: キーポイント読み込み中（C3D, 全フレーム）: {path}")`
   2. `load_c3d_all_frames(path)`。`ValueError` / `FileNotFoundError` は既存どおり `print(e, file=sys.stderr)` して return 1
   3. フレーム番号の重複検査: frames_data を先頭から走査し、既出のフレーム番号が現れたら `エラー: フレーム番号が重複しています: P{k}: {path}: frame {番号}` を stderr に出して return 1
   4. `present` を計算。空なら `C3Dに既知マーカーが1つもありません（P{k}: {path}, labels: {', '.join(labels)}）` を stderr に出して return 1
   5. 表示: `P{k}: キーポイント: {len(present)}/{len(KEYPOINT_NAMES)} マーカーを描画対象とします`、欠損マーカーがあれば `P{k}: 欠損マーカー（描画スキップ）: {名前列挙}`、最後に `P{k}: C3D: {len(frames_data)} フレーム, rate={point_rate} Hz`
   6. `skeleton = build_skeleton(present)`、frame_no → frames_data 要素 の辞書 `by_frame` を作って保持する
2. `--fps-frustum` 時の頭部マーカー検査: 人物 `args.fps_person` の `labels` に対して、既存と同じ `missing_head` 判定を行う。欠けていれば `頭部マーカーが入力にありません（--fps-frustum に必須）: P{k}: {path}: {名前列挙}` を stderr に出して return 1（k = `args.fps_person`）
3. point rate 一致検査: 全人物の `point_rate` を `==` で比較し、1つでも違えば次を stderr に出して return 1

   ```
   エラー: キーポイントファイルの point rate が一致しません
     P1: a.c3d rate=30.0 Hz
     P2: b.c3d rate=60.0 Hz
   ```

   一致すれば `point_rate` を共通値とする（以降の MP4 fps 既定値の決定は既存コードのまま）
4. 対象フレーム（和集合）: `frame_nos = sorted(set().union(*(by_frame.keys() for 各人物)))`。各人物の `frames = [by_frame.get(f) for f in frame_nos]`
5. 表示: `人物数: {N}, 対象フレーム（和集合）: {len(frame_nos)} フレーム, rate={point_rate} Hz`

- 1〜5 を、既存の1人モードの読み込み処理（`is_npz` 判定〜rate 表示）の**代わりに**実行する（`if multi: ... else: <既存>` の分岐）
- 既存の `total_frames = len(frames_data)` は、両モードとも `total_frames = len(frame_nos)` に置き換える

### 4.4 フレーム範囲フィルタと視錐台の事前計算（FR-003, FR-009）

#### フレーム範囲フィルタ

既存のフィルタ（`frames_data` を絞る処理）を、`frame_nos` と全人物の `frames` を同じ添字で絞る処理に置き換える。メッセージは既存のまま。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if args.start_frame is not None or args.end_frame is not None:
    start = ...; end = ...                          # 既存と同じ
    keep = [i for i, f in enumerate(frame_nos) if start <= f <= end]
    if not keep:
        <既存のエラー表示> ; return 1
    frame_nos = [frame_nos[i] for i in keep]
    for p in persons:
        p["frames"] = [p["frames"][i] for i in keep]
    print(f"フレーム範囲: {total_frames} フレーム中 {len(frame_nos)} フレームを対象")
```

以降、既存コードで `frames_data` / `len(frames_data)` を参照していた箇所（視錐台事前計算、フレームループ、視錐台サマリ、完了表示）は `frame_nos` / 人物レコードを参照するように置き換える。

#### 視錐台の事前計算

既存の `collect_head_points(labels, frames_data)` の呼び出しを、対象人物 `fp = persons[args.fps_person - 1]` について次のように変える。`collect_head_points` 自体は変更しない。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
fps_frames = [fr if fr is not None else make_absent_frame(f, len(fp["labels"]))
              for f, fr in zip(frame_nos, fp["frames"])]
head_arr = collect_head_points(fp["labels"], fps_frames)       # (F,7,3)
```

- 欠席フレームは全マーカーが無効（NaN・residual = −1）のフレームとして渡すため、`compute_fps_poses` が無効フレームと判定し、視錐台は描かれない（FR-009）。サマリの非描画数に含まれる
- 開始時の表示（`FPS視錐台: カメラ ...`）・終了時のサマリの文言は既存のまま変えない。終了時サマリの N は `len(frame_nos)`

### 4.5 新規関数（FR-007, FR-008）

#### 定数（既存の `FRUSTUM_THICKNESS` の直後に追加）

```python
# 人物ラベル（feat-037。複数人モードのみ描画）
LABEL_FONT = cv2.FONT_HERSHEY_SIMPLEX
LABEL_FONT_SCALE = 1.0
LABEL_THICKNESS = 2             # 文字の線の太さ [px]
LABEL_OUTLINE_THICKNESS = 6     # 黒縁取りの線の太さ [px]（文字より先に描く）
LABEL_COLOR = (255, 255, 255)   # 白 (BGR)
LABEL_OUTLINE_COLOR = (0, 0, 0) # 黒 (BGR)
LABEL_OFFSET = 20               # 基準点から文字のベースラインまでの上方向の距離 [px]
```

#### `make_absent_frame(frame_no: int, n_markers: int) -> dict`

欠席フレームを表す frames_data 要素を返す。

- 戻り値: `{"frame_no": frame_no, "data": np.full((n_markers, 3), np.nan), "residual": np.full(n_markers, -1.0)}`（float64）

#### `compute_person_depth(depth_cam: np.ndarray, valid: np.ndarray) -> float | None`

人物深度（FR-007）を返す。

- 入力: `depth_cam` (28,) float64（`compute_keypoint_depth` の戻り値）、`valid` (28,) bool
- 出力: `valid` が True の点の `depth_cam` の算術平均（float）。valid が1個も無い、または平均が非有限なら None
- near_plane・3DGS による隠蔽は考慮しない（有効点はすべて平均に含める）

#### `compute_label_anchor(pts2d, valid, depth_cam, near_plane) -> tuple[int, int] | None`

人物ラベルの基準点（FR-008）を返す。

- 入力: `pts2d` (28,2) float64、`valid` (28,) bool、`depth_cam` (28,) float64、`near_plane` float
- 処理:
  1. 候補 = `valid[i]` が True かつ `depth_cam[i]` が有限かつ `depth_cam[i] > near_plane` の点
  2. `Head`（`NAME_TO_IDX["Head"]`）が候補なら、その `pts2d` を返す
  3. そうでなければ、候補のうち `pts2d[i, 1]`（v 座標）が最小の点を返す。同値は添字の小さい方
  4. 候補が無ければ None
- 戻り値は `(int(round(u)), int(round(v)))`

#### `draw_person_labels(image, items) -> np.ndarray`

人物ラベルをまとめて描く。

- 入力: `image` (H,W,3) uint8 BGR、`items`: `list[tuple[str, tuple[int, int]]]`（ラベル文字列と基準点）
- 出力: 描画済みの画像。`items` が空なら `image` をそのまま返す（コピーしない）。空でなければ `image.copy()` に描いて返す
- 1項目ごとの処理:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
(tw, th), _ = cv2.getTextSize(text, LABEL_FONT, LABEL_FONT_SCALE, LABEL_THICKNESS)
org = (anchor[0] - tw // 2, anchor[1] - LABEL_OFFSET)    # 左下（ベースライン）
cv2.putText(img, text, org, LABEL_FONT, LABEL_FONT_SCALE, LABEL_OUTLINE_COLOR,
            LABEL_OUTLINE_THICKNESS, cv2.LINE_AA)
cv2.putText(img, text, org, LABEL_FONT, LABEL_FONT_SCALE, LABEL_COLOR,
            LABEL_THICKNESS, cv2.LINE_AA)
```

- 文字が画面外にはみ出す場合は OpenCV のクリッピングに任せる（位置の補正はしない）

### 4.6 フレームループ（FR-006〜FR-008, FR-013）

既存の `for i, fr in enumerate(frames_data):` を `for i, frame_no in enumerate(frame_nos):` に変え、描画部分を次のように置き換える。PNG 保存・MP4 書き込み・進捗表示は既存のまま（`fr['frame_no']` を `frame_no` に置き換えるだけ）。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
base = bgr_bg
if frustum_verts is not None and frustum_valid[i]:
    base = draw_frustum(...)                                   # 既存のまま（視錐台が一番下）

entries = []
for p in persons:
    fr = p["frames"][i]
    if fr is None:
        continue                                               # 欠席
    kpts_mm, valid = extract_keypoints(p["labels"], fr["data"], fr["residual"])
    kpts_calib = c3d_to_calib(kpts_mm)
    pts2d = project_keypoints(kpts_calib, cam)
    depth_cam = compute_keypoint_depth(kpts_calib, cam)
    if occlusion:
        kp_visible = compute_visibility(pts2d, depth_cam, valid, depth_map, alpha_map,
                                        args.occlusion_margin, args.near_plane)
    else:
        kp_visible = np.ones(len(KEYPOINT_NAMES), dtype=bool)
    entries.append({"person": p, "kpts_calib": kpts_calib, "pts2d": pts2d,
                    "valid": valid, "kp_visible": kp_visible, "depth_cam": depth_cam})

if multi:
    drawable = []
    for e in entries:
        d = compute_person_depth(e["depth_cam"], e["valid"])
        if d is not None:
            drawable.append((d, e))
    drawable.sort(key=lambda t: (-t[0], t[1]["person"]["index"]))   # 奥から。同値は番号順
    ordered = [e for _, e in drawable]
else:
    ordered = entries                                          # 1人モードは常に描画処理に渡す

overlay = base
for e in ordered:
    overlay = draw_overlay(overlay, e["kpts_calib"], e["pts2d"], e["valid"], e["kp_visible"],
                           cam, e["person"]["skeleton"], depth_map, alpha_map,
                           args.occlusion_margin, args.near_plane, occlusion=occlusion)

if multi:
    items = []
    for e in ordered:
        anchor = compute_label_anchor(e["pts2d"], e["valid"], e["depth_cam"], args.near_plane)
        if anchor is not None:
            items.append((f"P{e['person']['index']}", anchor))
    overlay = draw_person_labels(overlay, items)
```

- `draw_overlay` / `compute_visibility` / `extract_keypoints` / `draw_frustum` は変更しない
- 1人モードでは、既存と同じ関数を同じ引数で1回ずつ呼ぶ（`compute_keypoint_depth` は既存では `occlusion` 時のみ呼んでいたが、本変更で常に呼ぶ。戻り値は `occlusion=False` 時は描画に使わないため出力は変わらない）。したがって出力画像は既存と同一になる（FR-013）
- 1人モードでは人物は欠席しない（`frames` は frames_data そのもの）ため、`entries` は常に1要素
- ラベルは全人物のスケルトンを描いた後にまとめて描く（最前面）。3DGS 隠蔽・`occlusion` の値に関係なく描く

### 4.7 設定YAML（FR-010）

1. 変換関数 `_yaml_paths(value: str) -> list[str]` を `_yaml_floats3` の直後に追加する:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def _yaml_paths(value: str) -> list[str]:
    """設定YAMLの keypoints_path 値（; 区切りのパス1個以上）を変換する（feat-037）。"""
    parts = [p.strip() for p in value.split(";")]
    if any(p == "" for p in parts):
        raise argparse.ArgumentTypeError(
            f"; 区切りのパスに空の要素があります（パスは1個以上必要）: {value!r}")
    return parts
```

2. `CONFIG_CONVERTERS` の `"keypoints_path": str` を `"keypoints_path": _yaml_paths` に変え、`"fps_person": int` を `"frustum_depth": float` の後に追加する
3. エラーは既存の `parse_config_yaml` が `設定YAML キー 'keypoints_path': <メッセージ>` として終了コード 2 にする（追加実装なし）

| 設定YAMLの値 | 変換結果 |
|---|---|
| `data/a.c3d` | `["data/a.c3d"]` |
| `dir/a b.npz`（空白入り1パス） | `["dir/a b.npz"]` |
| `data/a.c3d; data/b.c3d` | `["data/a.c3d", "data/b.c3d"]` |
| `data/a.c3d;`・`a.c3d;;b.c3d`・空値 | エラー（終了コード 2） |

- 空値（`keypoints_path:`）は従来 `""` を1パスとして受け付け、読み込み時に失敗していた。本変更で設定YAMLの段階のエラー（終了コード 2）になる（挙動の変化だが、従来も正常には動かない入力のため FR-013 の後方互換の対象外とする）

### 4.8 エラーハンドリング一覧

| エラー | 検出 | 処理 | 終了コード |
|---|---|---|---|
| `--no-keypoints` とキーポイントファイルの併用 | 4.2-1 | `parser.error`（既存メッセージ） | 2 |
| キーポイントファイル0個（動画モード） | 4.2-3 | `parser.error`（既存メッセージ） | 2 |
| `--no-keypoints` と `--fps-person` | 4.2-2 | `parser.error`（既存形式） | 2 |
| `--fps-frustum` なしの `--fps-person` | 4.2-4 | `parser.error`（既存形式） | 2 |
| `--fps-person` が範囲外 | 4.2-6 | `parser.error` | 2 |
| 複数人モードで NPZ を含む | 4.2-5 | `parser.error` | 2 |
| 設定YAML `keypoints_path` に空の要素 | 4.7 | `parse_config_yaml` の既存処理 | 2 |
| ファイル不存在・C3D として読めない | 4.3 複数人-1-2 | 既存どおり stderr 表示 | 1 |
| ファイル内のフレーム番号重複（複数人モード） | 4.3 複数人-1-3 | stderr 表示 | 1 |
| 既知マーカー0個のファイル | 4.3 複数人-1-4（1人モードは既存） | stderr 表示 | 1 |
| 対象人物に頭部マーカーが無い（`--fps-frustum`） | 4.3 複数人-2（1人モードは既存） | stderr 表示 | 1 |
| point rate 不一致 | 4.3 複数人-3 | stderr 表示 | 1 |
| フレーム範囲内に対象フレームなし | 4.4 | 既存メッセージ | 1 |

ファイル不存在のとき、`c3d` ライブラリの `open` が送出する `FileNotFoundError` は既存の except 節で捕捉される（既存の1人モードと同じ）。

### 4.9 境界条件

- **キーポイントファイル1個**: 1人モード（既存動作）。`--fps-person 1` の明示指定は許容
- **全員欠席のフレーム**: 和集合の定義上は生じない（どれかのファイルにあるフレームだけが対象）。ただし全員が有効点ゼロのフレームはあり得る。その場合は背景（と視錐台）だけの画像を出力する
- **人物が観察カメラの背後にいる**: 人物深度が負になり、並べ替えでは最も手前扱いになる。点・ボーンは既存の背面ガード（`compute_visibility`）で描かれない（`occlusion` 時）。ラベルは基準点の候補が無いため描かれない
- **ラベル同士・ラベルとスケルトンの重なり**: 補正しない（後に描いたものが上）
- **同じパスを2回指定**: 別の人物として扱う。2人が完全に重なって描かれる（ラベル `P1` / `P2` も重なる）

## 5. ログ・デバッグ設計

- print ベース（既存と同じ。ログレベル機構は導入しない）
- 複数人モードで追加する標準出力は 4.3 の人物ごとの行と全体の1行のみ。フレームごとの進捗表示は既存のまま
- 1人モードの標準出力は変えない

## 6. テスト設計

`tests/test_feat037_multi_person.py` を新規作成する。gsplat / torch を import しない範囲でテストする。main を通すテストは、既存 `tests/test_feat017_all_frames.py` / `tests/test_feat034_fps_frustum.py` と同じ方式（`render` モジュール・`render_image`・`load_c3d_all_frames` のスタブ化、合成TOML）で行う。`load_c3d_all_frames` のスタブはパスごとに違うデータ（ラベル・フレーム番号・rate）を返せるようにする。

| # | テスト | 期待値 |
|---|---|---|
| T-01 | パーサー: 位置引数に C3D 2個 | `args.keypoints_path == ["a.c3d", "b.c3d"]` |
| T-02 | パーサー: 位置引数に1個 | `["a.c3d"]` |
| T-03 | パーサー: `--no-keypoints` でキーポイント省略 | `[]` |
| T-04 | 検証: `--no-keypoints` とキーポイント1個 | SystemExit(2) |
| T-05 | 検証: 動画モードでキーポイント0個 | SystemExit(2)、stderr に `keypoints_path` |
| T-06 | 検証: `a.c3d b.npz`（大文字 `B.NPZ` も） | SystemExit(2)、stderr に NPZ のパス |
| T-07 | 検証: `--fps-person` を `--fps-frustum` なしで指定 | SystemExit(2)、stderr に `--fps-frustum` |
| T-08 | 検証: `--fps-person 0` / `3`（2人時） | SystemExit(2) |
| T-09 | 検証: `--no-keypoints --fps-person 1` | SystemExit(2) |
| T-10 | main: 2ファイルの point rate が 30 と 60 | 戻り値 1、stderr に両方の rate |
| T-11 | main: 2人目のファイルに既知マーカーが無い | 戻り値 1、stderr に `P2` |
| T-12 | main: 複数人モードでファイル内フレーム番号重複 | 戻り値 1、stderr に `重複` |
| T-13 | main: P1=フレーム1〜3、P2=フレーム3〜5 | 出力PNG が frame_000001〜000005 の5枚 |
| T-14 | main: T-13 で `--start-frame 4 --end-frame 9` | 2枚（4, 5） |
| T-15 | `compute_person_depth`: valid 点のみ平均 / valid なし → None / inf を含む → None | 値・None |
| T-16 | `compute_label_anchor`: Head 有効 → Head の位置 / Head 無効 → v 最小の点 / Head が near_plane 以下 → v 最小の別の点 / 候補なし → None | 各位置・None |
| T-17 | 描画順: 2人の人物深度を 2m と 4m にして main を実行し、`draw_overlay` を monkeypatch で呼び出し順を記録（元の関数に委譲） | 4m の人物が先、2m の人物が後。距離を入れ替えると順序も入れ替わる |
| T-18 | 描画順: 人物深度が同じ2人 | 人物番号順 |
| T-19 | 欠席・有効点ゼロの人物は `draw_overlay` が呼ばれない | 呼び出し回数 |
| T-20 | ラベル: 2人モードで `draw_person_labels` に渡る items | `[("P2", ...), ("P1", ...)]`（描画順と同じ並び） |
| T-21 | ラベル: 1人モードでは `draw_person_labels` が呼ばれない | 呼び出し0回 |
| T-22 | ラベル: `occlusion` 時に頭部が3DGSに隠れる（合成深度マップで Head を不可視にする）フレームでも items に入る | items にその人物がある |
| T-23 | `draw_person_labels`: items 空なら入力と同一オブジェクト / 1項目で白画素が増える | `is` 判定 / 画素数 > 0 |
| T-24 | 視錐台: `--fps-frustum --fps-person 2` で `collect_head_points` に渡る labels が P2 のもの | monkeypatch で捕捉 |
| T-25 | 視錐台: 対象人物が欠席のフレームは `frustum_valid` が False | サマリの非描画数 |
| T-26 | 視錐台: `--fps-person 2` で P2 だけに頭部マーカーが無い | 戻り値 1、stderr に `P2`。同じ入力で `--fps-person 1`（P1 は頭部あり）は通る |
| T-27 | 設定YAML: `keypoints_path: a.c3d; b.c3d` | `["a.c3d", "b.c3d"]` |
| T-28 | 設定YAML: `keypoints_path: dir/a b.npz`（空白入り） | `["dir/a b.npz"]` |
| T-29 | 設定YAML: `a.c3d;` / `a.c3d;;b.c3d` / 空値 | SystemExit(2)、stderr に `keypoints_path` |
| T-30 | 設定YAML: `fps_person: 2` が反映される | `args.fps_person == 2` |
| T-31 | 設定YAMLの2パスを CLI の1パスで置き換え | `["c.c3d"]` |
| T-32 | 1人モードの後方互換: 同一入力で、変更後の main の出力PNGと、1人分を直接 `draw_overlay` で描いた画像が一致 | `np.array_equal` |
| T-33 | 回帰: 既存テスト全件 | `uv run pytest -v` 全件パス（変更前 518 passed / 1 skipped + 本案件追加分） |

### 既存テストの修正（この2か所に限る）

`tests/test_feat033_config_yaml.py`:

- `test_backward_compat_no_config` の `assert args.keypoints_path == "dummy.npz"` → `assert args.keypoints_path == ["dummy.npz"]`
- `test_positional_partial_override` の `assert args.keypoints_path == "dummy.npz"  # YAML由来のまま` → `assert args.keypoints_path == ["dummy.npz"]  # YAML由来のまま`

これ以外の既存テストが失敗した場合は、実装を中断して報告する（CLAUDE.md「実装の実行方法」の想定外事象の扱い）。

テスト結果は `tests/results/feat-037_test_result.txt` に `-v` 出力を保存する。

### 実データでの動作確認（実装ステップ内で実施）

プロジェクトルートで実行する。PLY・TOML は `phase4/data/run_keypoints.yaml` と同じもの、C3D は `phase4/data/session001_f145749_world300.c3d` を使う（カメラ名・パスはサブエージェントが `phase4/data/run_keypoints.yaml` と実データを確認して決め、報告に記載する）。

1. **1人モードのバイト同一性**: 変更前の `render_keypoints.py`（`git show HEAD:phase4/render_keypoints.py` をスクラッチディレクトリに保存し、`phase4/` を `PYTHONPATH` に通して実行）と変更後で、同じ C3D 1個・`--start-frame` / `--end-frame` で3フレームに絞って PNG を出力し、全 PNG がバイト同一であること
2. **複数人モード**: 同じ C3D を2回指定して同じ3フレームを出力し、終了コード 0・人物ごとの読み込み表示・`P1` / `P2` ラベル（白文字）が出力 PNG に描かれていることを目視確認する（同じデータのため2人は重なる）
3. 異なる人物の C3D での確認は手動テスト（ステップ7）でユーザーが行う

## 7. 設計判断（ADR 簡易版）

- **D-1 1人モードは既存の読み込み処理をそのまま残す**: 1人モードと複数人モードで読み込みを共通化（和集合化・並べ替え・重複検査を常に行う）する案は、ファイル内のフレーム順・表示・NPZ 経路が変わり得て後方互換（FR-013）の保証が難しくなるため却下。分岐で既存経路を温存する
- **D-2 位置引数を `nargs="*"` に拡張**: ユーザー選択（新オプション追加案は却下）。`nargs="+"` にしない理由は静止画モード（キーポイント省略）を許すため
- **D-3 設定YAMLは `;` 区切り**: 当初案の空白区切りは、運用中の `phase4/data/run_keypoints.yaml` の空白入りパスを壊すため却下。カンマ区切りも検討したが、`;` の方がファイル名に現れにくいため `;` を採用
- **D-4 人物深度は有効点の単純平均**: 体幹点（Hip 等）だけの平均にする案は、欠損構成（22点 C3D など）で点が無い場合の分岐が増えるため却下。ヒアリングで「平均距離で並べる簡易的な前後関係」と合意済み
- **D-5 人物間の関節単位の隠蔽はしない**: ヒアリングで「奥の人物から描く」を選択（関節・ボーン単位は難易度が高いため不採用）
- **D-6 ラベルの基準点は Head、無ければ画像上で最も上の点**: 頭部の上に置くというヒアリング結果に従う。Head が欠損する構成（22点 C3D 等）でも頭部付近に出すため、画像上で最も上の有効点に切り替える
- **D-7 ラベルは最前面・常に表示**: ヒアリングで「常に表示」を選択。スケルトンより後に描く
- **D-8 欠席フレームは無効フレームとして視錐台計算に渡す**: `collect_head_points` / `compute_fps_poses` を変えずに済み、無効判定を正本に一元化できる（feat-034 D-4 と同じ考え方）
- **D-9 point rate は完全一致で比較**: 許容誤差を設ける案は、どこまでを同じとみなすかの根拠が無いため却下。同じ収録系から作られた C3D は同じ float 値になる前提
- **D-10 複数人モードの NPZ は不可**: ヒアリングで「C3D のみ」を選択

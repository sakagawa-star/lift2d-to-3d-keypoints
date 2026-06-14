# feat-017 機能設計書: render_keypoints.py 全フレーム対応（連番PNG + MP4）

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|--------|----------------|
| FR-001 全フレーム読み込み | 1.4.1 `load_c3d_all_frames` |
| FR-002 フレーム範囲指定 | 1.4.2 フレーム範囲フィルタ |
| FR-003 背景レンダリング1回計算 | 1.4.3 背景の事前計算 |
| FR-004 各フレームのオーバーレイ描画 | 1.4.4 フレームループ |
| FR-005 連番PNG出力 | 1.4.5 出力（PNG） |
| FR-006 MP4出力 | 1.4.6 出力（MP4） |

## 1.2 システム構成

- 対象ファイル: `phase4/render_keypoints.py`（既存を拡張、新規ファイルなし）
- 変更内容:
  - **新規関数** `load_c3d_all_frames(c3d_path) -> (labels, frames_data, point_rate)`
  - **新規関数** `get_mp4_writer(...)`（ffmpeg起動。render.py のロジックを移植）※ 1.4.6 参照
  - **既存関数の再利用（変更なし）**: `extract_halpe26`, `c3d_to_calib`, `project_keypoints`,
    `compute_keypoint_depth`, `compute_visibility`, `draw_overlay`, `render_image`,
    `load_cameras_toml`, `select_camera`
  - **`load_c3d_first_frame` は削除**（`load_c3d_all_frames` に統合。先頭フレーム取得は
    全フレーム版の先頭要素で代替可能）
  - **`main` の書き換え**: 単一フレーム処理 → 範囲フィルタ＋フレームループ
  - **`_build_parser` の変更**: `--output` 廃止、`--output-dir`/`--start-frame`/`--end-frame`/
    `--mp4`/`--mp4-fps` 追加
- 依存方向: `render_keypoints.py` → `render.py`（`load_ply`, `print_ply_summary`）。循環なし。

## 1.3 技術スタック

- Python 3.10、phase4 の uv 環境
- 既存ライブラリのみ: numpy, opencv-python, tomli, c3d(py-c3d), torch, gsplat
- MP4化: システムの ffmpeg（render.py と同一の subprocess パイプ方式。NVENC→libx264 フォールバック）
- 新規ライブラリ追加なし（TECH_STACK.md 更新不要）

## 1.4 各機能の詳細設計

### 1.4.1 `load_c3d_all_frames`（FR-001）

#### データフロー
- 入力: `c3d_path: str`
- 出力: `tuple[list[str], list[dict], float]`
  - `labels`: マーカー名リスト（strip済み）
  - `frames_data`: フレームごとの dict のリスト。各要素:
    - `{"frame_no": int, "data": np.ndarray(n_markers,3) float64 mm, "residual": np.ndarray(n_markers,) float64}`
  - `point_rate`: float（C3D rate [Hz]。取得不能時は `0.0`。**丸めず小数のまま保持**）

#### 処理ロジック（擬似コード）
```
def load_c3d_all_frames(c3d_path):
    import c3d
    with open(c3d_path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [l.strip() for l in reader.point_labels]
        # point_rate は None・欠損・非数値があり得るため安全に正規化（例外を投げない）
        raw_rate = getattr(reader, "point_rate", None)
        try:
            point_rate = float(raw_rate)
        except (TypeError, ValueError):
            point_rate = 0.0
        frames_data = []
        for frame_no, points, _analog in reader.read_frames():
            frames_data.append({
                "frame_no": int(frame_no),
                "data": np.asarray(points[:, :3], dtype=np.float64),
                "residual": np.asarray(points[:, 3], dtype=np.float64),
            })
    if not frames_data:
        raise ValueError(f"C3Dにフレームがありません: {c3d_path}")
    return labels, frames_data, point_rate
```
- `read_frames()` の `frame_no` をそのまま C3Dフレーム番号として採用する（FR-002 の範囲指定と一致）。
- 既存 `load_c3d_first_frame` の「先頭1フレームで break」ロジックは廃止し、全フレームを保持する。

#### エラーハンドリング
- フレーム0件 → `ValueError`（メッセージ「C3Dにフレームがありません」）→ main で捕捉し終了コード1。
- `reader.point_rate` が None・取得不能・非数値 → 上記 try/except で `point_rate=0.0` に正規化し、
  例外は投げない。fps のフォールバック（30）は **main 側に一本化**する（1.4.6）。
  本関数は `point_rate`（0.0 含む）をそのまま返す。

#### 境界条件
- 1フレームのみのC3D → frames_data 長さ1。先頭フレーム出力が feat-016 と一致すること（FR-004）。

### 1.4.2 フレーム範囲フィルタ（FR-002）

#### 処理ロジック
- `render.py` と同一方式:
```
if start_frame is not None or end_frame is not None:
    start = start_frame if start_frame is not None else -inf
    end   = end_frame   if end_frame   is not None else +inf
    frames_data = [fr for fr in frames_data if start <= fr["frame_no"] <= end]
    if not frames_data:
        print(エラー, file=stderr); return 1
```
- 両端含む。両方 None なら全フレーム。

#### 境界条件
- 範囲が全フレームより外 → 0件 → 終了コード1。

### 1.4.3 背景の事前計算（FR-003）

#### 処理ロジック
- フレームループの**前**で1回だけ実行する:
```
occlusion = not args.no_occlusion
if occlusion:
    bgr_bg, depth_map, alpha_map = render_image(gaussians, cam, near_plane, bg, return_depth=True)
else:
    bgr_bg = render_image(gaussians, cam, near_plane, bg)
    depth_map = alpha_map = None
```
- `bgr_bg` は読み取り専用の背景。各フレームの `draw_overlay` は内部で `image.copy()` するため
  背景を破壊しない（既存実装で確認済み）。
- 投影に必要な `cam` はフレーム間で不変。`compute_visibility` も `depth_map`/`alpha_map` 共有。

#### 設計判断
- **採用**: 背景・深度・αを1回計算してループ外で保持。
- **却下**: フレームごとに `render_image` を呼ぶ案。理由: カメラ固定で背景は不変。gsplat 呼び出しは
  最も重い処理であり、N回呼ぶのは N倍の無駄。

### 1.4.4 フレームループ（FR-004）

#### 処理ロジック
```
for i, fr in enumerate(frames_data):
    kpts_mm, valid = extract_halpe26(labels, fr["data"], fr["residual"])
    kpts_calib = c3d_to_calib(kpts_mm)
    pts2d = project_keypoints(kpts_calib, cam)
    if occlusion:
        depth_cam = compute_keypoint_depth(kpts_calib, cam)
        kp_visible = compute_visibility(pts2d, depth_cam, valid, depth_map, alpha_map,
                                        occlusion_margin, near_plane)
    else:
        kp_visible = np.ones(26, dtype=bool)
    overlay = draw_overlay(bgr_bg, kpts_calib, pts2d, valid, kp_visible, cam,
                           depth_map, alpha_map, occlusion_margin, near_plane, occlusion=occlusion)
    # 出力（1.4.5 / 1.4.6）
```
- `extract_halpe26` は欠損ラベルがあると `ValueError`。ラベル集合はフレーム間で不変のため、
  **ループ前に一度だけ** Halpe26 ラベル存在チェックを行い、ループ内の例外は想定しない
  （valid フラグはフレームごとに変わるが、ラベルの有無はC3D全体で固定）。
  - 実装: ループ前に `extract_halpe26(labels, frames_data[0]["data"], frames_data[0]["residual"])`
    を試行し、`ValueError`（マーカー不足）なら終了コード1。

#### エラーハンドリング
- Halpe26マーカー不足 → ループ前チェックで検出 → stderr＋終了コード1。
- 投影・描画の例外は想定しない（feat-016 で検証済みの確定ロジック）。

#### 境界条件
- あるフレームで全点 valid=False（全欠損）→ 点・ボーンなしの背景のみPNGが出力される（正常）。

### 1.4.5 出力（PNG）（FR-005）

#### ファイル・ディレクトリ設計
- 出力ディレクトリ: `--output-dir`（省略時 `./data/keypoints_<カメラ名>/`）
- ファイル名: `frame_<frame_no:06d>.png`（C3Dフレーム番号でゼロ埋め6桁）
- ループ前に `os.makedirs(output_dir, exist_ok=True)`。
- 保存: `cv2.imwrite(path, overlay)`。失敗時（False返却 or cv2.error）はそのフレームで
  stderr にメッセージを出し終了コード1（render_keypoints 既存の2系統エラー処理を踏襲）。
  - **ただし ffmpeg 起動済み（`--mp4`）の場合、即 return せず 1.4.6 の `try/finally` を経由して
    ffmpeg プロセスをクリーンアップしてから終了コード1で終了する**（プロセスリーク防止）。

### 1.4.6 出力（MP4）（FR-006）

#### 処理ロジック
- `--mp4` 指定時、連番PNG出力と**並行して**MP4にも書き込む（両方出力。requirements FR-006）。
- **起動順序**: ffmpeg の存在確認・起動は**フレームループの前**に行う（render.py と同じ）。
  そのため ffmpeg 不在時はPNGも1枚も出力されずに終了する（requirements FR-006 受け入れ基準と一致）。
- **クリーンアップ（プロセスリーク防止）**: `--mp4` 時はフレームループ全体を `try/finally` で囲む。
  `finally` で ffmpeg の `stdin` が未クローズなら `stdin.close()`、`wait()` し、必要に応じて
  `stderr` を回収する。PNG保存失敗・`stdin.write` 失敗・想定外例外のいずれの経路でも
  ffmpeg プロセスを残さない。正常終了時は `finally` 前に `stdin.close(); wait()` 済みでよい。
- ffmpeg 起動は `render.py` の方式を移植:
  - `shutil.which("ffmpeg")` で存在確認。なければ stderr＋終了コード1（ループ前なのでPNGも未出力）。
  - `ffmpeg -encoders` で `h264_nvenc` の有無を確認し NVENC→libx264 フォールバック。
  - 入力は rawvideo rgb24、解像度は `cam["width"]x cam["height"]`、`-r` に fps。
  - `subprocess.Popen(..., stdin=PIPE, stderr=PIPE)` を起動し、ループ内で
    **RGB変換した** フレームを `stdin.write(rgb.tobytes())`。
    - 注意: `overlay` は BGR（OpenCV）。ffmpeg には rgb24 を渡すため `overlay[:, :, ::-1]` で
      RGB化してから書き込む。
    - **ffmpeg途中終了対策**: `stdin.write` を `try/except (BrokenPipeError, OSError)` で囲み、
      捕捉したら ffmpeg の `stderr` を読み取って stderr に出力し、終了コード1で終了する
      （ffmpeg が rawvideo を受理できず途中で死ぬとパイプが切れ、無対策だと traceback で落ちる）。
  - ループ後 `stdin.close(); wait()`。returncode≠0 なら stderr に ffmpeg stderr を出し終了コード1。
- fps 決定ロジック（**小数を丸めず保持**。`--mp4-fps` と内部値はともに float）:
```
if args.mp4_fps is not None:
    fps = float(args.mp4_fps)          # 例: 59.94 を保持
elif point_rate and point_rate > 0:
    fps = point_rate                   # C3D rate を丸めず保持（29.97 等）
else:
    fps = 30.0
    print("警告: C3D rate を取得できないため MP4 fps=30 を使用", file=stderr)
```
- ffmpeg へは `-r` に `repr(fps)`（または `f"{fps}"`）の文字列を渡す。ffmpeg は小数fpsを受理する。
  解像度ガード（render.py の `-s WxH`）は `cam["width"]x cam["height"]` を使う。
- MP4パス: `<output-dir>/output.mp4`。

#### 設計判断
- **採用**: PNGとMP4を同時出力（ユーザー選択「連番PNG + MP4」）。render.py は排他だが本案件は両出力。
- **採用**: fps 既定は C3D rate（実時間に一致）。`--mp4-fps` で上書き可。**fps は float で小数を保持**
  （C3D rate が 29.97/59.94 等の非整数でも動画の実時間がずれないようにする。render.py は int だが
  本案件は実時間一致を要件とするため float にする）。
- **却下**: render.py のように `--mp4` でPNGを出さない案。理由: ユーザー要求が両出力。
- **却下**: fps を `int(round())` する案。理由: 非整数 rate で動画時間がずれ、時系列検証が破綻する。

## 1.5 状態遷移

該当なし（バッチ処理、GUI/ステートフルなし）。

## 1.6 ファイル・ディレクトリ設計

| 項目 | 値 |
|------|----|
| 連番PNGディレクトリ | `--output-dir`（既定 `./data/keypoints_<カメラ名>/`） |
| PNGファイル名 | `frame_<frame_no:06d>.png` |
| MP4ファイル | `<output-dir>/output.mp4` |

## 1.7 インターフェース定義

### 新規/変更関数シグネチャ
```python
def load_c3d_all_frames(c3d_path: str) -> tuple[list[str], list[dict], float]: ...
# get_mp4_writer は実装簡略のため main 内インラインでも可（render.py 準拠）。
# 関数化する場合:
def start_ffmpeg(output_dir: str, width: int, height: int, fps: float) -> subprocess.Popen: ...
```

### CLI（`_build_parser`）
| 引数 | 型 | 既定 | 説明 |
|------|----|------|------|
| `ply_path` | str（位置） | - | 3DGS PLY |
| `toml_path` | str（位置） | - | キャリブTOML |
| `c3d_path` | str（位置） | - | C3D（Halpe26、全フレーム） |
| `--camera` | str | 必須 | 対象カメラ名 |
| `--near-plane` | float | 0.1 | nearクリップ[m] |
| `--output-dir` | str | None→`./data/keypoints_<カメラ名>/` | 連番PNG出力先 |
| `--background` | float×3 | 0 0 0 | 背景色RGB[0-1] |
| `--no-occlusion` | flag | False | オクルージョン無効 |
| `--occlusion-margin` | float | 0.05 | 深度マージン[m] |
| `--start-frame` | int | None | 開始C3Dフレーム番号（含む） |
| `--end-frame` | int | None | 終了C3Dフレーム番号（含む） |
| `--mp4` | flag | False | MP4も出力 |
| `--mp4-fps` | float | None→C3D rate | MP4フレームレート（小数可、例 59.94） |

- **廃止**: `--output`（単一PNG）。

## 1.8 ログ・デバッグ設計

- INFO相当（print）:
  - 対象カメラ・解像度
  - C3D読み込み（総フレーム数、C3D rate）
  - 範囲フィルタ後のフレーム数（`総数 中 N フレームを対象`）
  - 背景レンダリング完了（occlusion ON/OFF）
  - 各フレーム `[i/N] frame <frame_no> -> <path>`（および `-> mp4`）
  - MP4エンコーダ名、MP4保存パス
  - 完了時刻と秒/フレーム
- WARNING相当（stderr）: C3D rate 取得不能時の「MP4 fps=30 を使用」警告。
- ERROR相当（stderr）: カメラ名不正、C3D空、Halpe26不足、範囲0件、ffmpeg不在、
  PNG保存失敗、ffmpeg途中終了（パイプ切断、stderr回収）、ffmpeg終了コード≠0。いずれも終了コード1。

## 1.9 コード例の注意

本設計書のコードスニペットは意図伝達用であり、そのままコピーして使うものではない
（変数名・エラーメッセージは既存コードのスタイルに合わせて実装する）。

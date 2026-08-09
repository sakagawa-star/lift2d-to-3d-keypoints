# feat-027 機能設計書: NPZ直読みによる一人称視点動画一括生成（Blender廃止）

作成日: 2026-08-07
基準: `docs/DESIGN_STANDARD.md`
対応要求: `requirements.md`（feat-027）

---

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 NPZ読み込みと検証 | §4.1 |
| FR-002 内部パラメータ読み込み | §4.2 |
| FR-003 カメラポーズ計算 | §4.3 |
| FR-004 フレーム有効性判定 | §4.3 |
| FR-005 チャンク分割・区間MP4 | §4.4, §4.5, §4.6 |
| FR-006（欠番） | 対象外（複数GPU並列は後続案件に分割。2026-08-07 人レビュー決定） |
| FR-007 再開 | §4.7 |
| FR-008 連結・最終MP4 | §4.8 |
| FR-009 デバッグ静止画モード | §4.9 |
| FR-010 ポーズダンプモード | §4.10 |
| FR-011 実行ログ・サマリ | §1.8 |
| FR-012 等価性検証（案A） | §5 |
| NFR-001〜005 | §1.3, §4.6, §4.7, §1.6 |

## 1.2 システム構成

新規ファイルは `phase4/render_fps_video.py` の1本のみ。既存スクリプトは変更しない。

```
phase4/render_fps_video.py   # 新規（本設計の対象）
  ├─ import phase4/npz_to_c3d.py      : load_npz()               # トップレベル import 可（numpy のみ依存）
  ├─ import phase4/render_keypoints.py: load_cameras_toml(), select_camera()   # 関数内 import（tomli/cv2 依存のため）
  └─ import phase4/render.py          : load_ply(), render_frame()             # 関数内 import（torch/gsplat 依存のため）
```

- モジュールトップレベルの import は標準ライブラリ（argparse, json, os, shutil, subprocess, sys, time）と numpy のみとする。torch / gsplat / render / render_keypoints は使用する関数の内側で import する（制約条件「重い依存の import 方針」）
- `npz_to_c3d` / `render_keypoints` / `render` は同一ディレクトリ（`phase4/`）のためスクリプト実行時は素の `import` で解決する。テストからは `sys.path.insert` で解決する（既存テストと同じ方式）

### プロセス構成（MP4モード時）

```
メインプロセス（単一プロセス・単一GPU。ADR-5）
  ├─ NPZ読込・ポーズ全フレーム計算・チャンク計画・マニフェスト管理
  ├─ torch.cuda.set_device(--gpu) → PLY を1回ロード（有効フレームを初めて描画する時）
  ├─ チャンク番号順に直列処理: チャンクごとに ffmpeg 子プロセスを起動し rawvideo をパイプ書き込み
  └─ 全チャンク完成後に ffmpeg concat で連結
```

複数GPU並列は後続案件に分割（2026-08-07 人レビュー決定）。ただしチャンクの独立性（各チャンクが自己完結の区間MP4になり、完成判定・再開がチャンク単位で閉じる構造）は本設計で維持し、後続案件がチャンクの分配機構を足すだけで並列化できるようにする。

## 1.3 技術スタック

- 言語: Python 3.10（phase4 uv 環境。`uv run --project phase4`）
- ライブラリ（すべて phase4 既存依存。新規追加なし）:
  - numpy: ポーズ計算・配列処理
  - torch + gsplat: レンダリング（`render.py` 経由）
  - tomli: TOML読込（`render_keypoints.py` 経由）
  - Pillow: 静止画モードのPNG保存
- 外部バイナリ: ffmpeg（libx264 エンコード・concat 連結）。NVENC は使用しない（ADR-2）
- `docs/TECH_STACK.md` の更新: 不要（新規ライブラリなし）

## 4. 各機能の詳細設計

### 4.1 NPZ読み込みと検証（FR-001）

**データフロー**

- 入力: NPZパス（str）
- 出力: `x3d_world (F,J,3) float64`（world座標[m]）、`frame_ids (F,) int64`（昇順連番）、`joint_names list[str]`
- `npz_to_c3d.load_npz()` をそのまま呼ぶ。同関数の ValueError（必須キー欠落・形状不一致・非連番）は main で捕捉し「エラー: {メッセージ}」を stderr に出して戻り値1

**頭部7点検証**

```
HEAD_JOINT_NAMES = ["LEye", "REye", "LEar", "REar", "Nose", "Head", "Neck"]  # モジュール定数

def resolve_head_indices(joint_names: list[str]) -> dict[str, int]:
    # 戻り値: {"LEye": idx, ...}（7要素）
    # 欠落があれば ValueError("NPZのjoint_namesに頭部関節がありません: LEar, Nose"（欠落名を列挙）)
```

追加キー（`pnp_ok`, `coord_system` を含む規約外キー）は `load_npz` が返さないため自然に無視される。

**F=0 の検証**: `load_npz()` は F=0 を拒否しないため、main が `load_npz` 直後に F == 0 を検査し、「エラー: NPZ にフレームがありません（F=0）: {path}」を stderr に出して終了コード1で終了する（FR-001。マニフェスト・ログの `frame_ids[0]` / `frame_ids[-1]` 参照より前に弾く）。

**境界条件**: F=1 でも動作する（`load_npz` の連番検証は F>=2 のみ）。J=7（頭部のみ）でも動作する。F=0 は上記のとおりエラー終了する。

### 4.2 カメラ内部パラメータの読み込み（FR-002）

```
def load_intrinsics(toml_path: str, camera_name: str) -> dict:
    # 関数内 import: from render_keypoints import load_cameras_toml, select_camera
    # 戻り値: {"fx","fy","cx","cy": float, "width","height": int}
```

- `load_cameras_toml(toml_path)` → `select_camera(cameras, camera_name)`（カメラ名不一致時の一覧つき ValueError は既存実装のものを使う）
- K 行列から fx=K[0,0], fy=K[1,1], cx=K[0,2], cy=K[1,2] を取り出す。D・rvec・tvec は使用しない
- width % 2 != 0 または height % 2 != 0 なら ValueError(f"解像度が偶数ではありません（libx264 yuv420p 制約）: {width}x{height}")

**境界条件**: TOML にカメラテーブルが1つもない場合は `select_camera` の ValueError（利用可能カメラ一覧が空）で終了する。

### 4.3 カメラポーズ計算とフレーム有効性判定（FR-003, FR-004）

**データフロー**

- 入力: `x3d_world (F,J,3) float64`、頭部インデックス dict
- 出力:
  - `c2w_b (F,4,4) float64`: Blenderカメラ規約の c2w。無効フレームは全要素 NaN
  - `valid (F,) bool`: 有効フレームマスク
  - `invalid_reasons list[tuple[int, str]]`: 縮退・回転検証失敗フレームの (配列インデックス, 理由文字列)。NaN由来は含めない（FR-004: NaN は警告なし）

**定数**（`fps_camera_pose.py` と同値）

```
EPS = 1e-8        # 正規化前ベクトル長の縮退判定
ORTHO_TOL = 1e-6  # 行列式・直交性の許容誤差
```

**処理ロジック**（全フレーム一括のベクトル化 numpy。`np.errstate(invalid="ignore", divide="ignore")` で警告抑制）

```
def compute_fps_poses(x3d_world, head_idx) -> tuple[c2w_b, valid, invalid_reasons]:
    LEye..Neck = x3d_world[:, head_idx[名前], :]     # 各 (F,3)
    eyes_mid = (LEye + REye) / 2                      # カメラ位置
    ear_mid  = (LEar + REar) / 2

    finite = 頭部7点すべての座標が np.isfinite      # (F,) bool
    valid = finite.copy()
    # 以降の縮退判定は finite なフレームに対してのみ理由を記録する

    p_raw  = Nose - ear_mid ;  |p_raw|  < EPS → 無効(理由 "Nose-ear_mid 縮退")
    r0_raw = LEye - REye    ;  |r0_raw| < EPS → 無効(理由 "LEye-REye 縮退")
    p  = p_raw / |p_raw| ; r0 = r0_raw / |r0_raw|
    f_raw = p - r0 * dot(p, r0) ; |f_raw| < EPS → 無効(理由 "f(p⊥r0) 縮退")
    f = f_raw / |f_raw|
    u = normalize(cross(f, r0))
    hu_raw = Head - Neck ; |hu_raw| < EPS → 無効(理由 "Head-Neck 縮退")
    head_up = hu_raw / |hu_raw|
    u = where(dot(u, head_up) < 0, -u, u)            # 符号確定（dot==0 は反転しない）
    b = -f
    r = normalize(cross(u, b))
    R = stack列(r, u, b)                              # (F,3,3)。列 = (右, 上, 後方)

    # 回転行列検証（有効フレームのみ）
    非有限含む / |det(R)-1| > ORTHO_TOL / max|R@R^T - I| > ORTHO_TOL
      → 無効(理由 "回転行列検証失敗(det=..., max_dev=...)")

    c2w_b[valid] = [[R, eyes_mid], [0,0,0,1]] ; c2w_b[~valid] = NaN
```

- 1フレームで複数の縮退が同時成立する場合、理由は上記の判定順で最初の1つを記録する
- ループなし（F=100万でも数秒オーダー。NFR-002）

**OpenCV変換と viewmat**

```
def c2w_blender_to_viewmats(c2w_b: (F,4,4) float64) -> (F,4,4) float32:
    # render.py blender_to_opencv_c2w と同一式: c2w_cv = c2w_b @ diag(1,-1,-1,1)
    # R_cv = R_b @ diag(1,-1,-1), t は不変
    # viewmat = inv(c2w_cv) を解析的に: [[R_cv^T, -R_cv^T @ t], [0,0,0,1]]
    # 無効フレームは NaN のまま（使用しない）
```

**エラーハンドリング**: この関数はエラー終了しない（無効フレームは黒として続行。FR-004）。呼び出し側（main）が `invalid_reasons` を「警告: frame_id {fid}: {理由}」形式で stderr に1件ずつ出力する。

**境界条件**: 全フレーム無効（valid が全て False）でも続行し、全黒の動画を生成する（警告サマリで判別可能）。

### 4.4 チャンク計画（FR-005）

```
def plan_chunks(num_frames: int, chunk_size: int, frame_ids) -> list[dict]:
    # 戻り値: [{"index": int, "i0": int, "i1": int, "fid0": int, "fid1": int}, ...]
    # i0/i1 は配列インデックス（i1 は排他）、fid0/fid1 は frame_id（fid1 は包含）
    # チャンク k: i0 = k*chunk_size, i1 = min((k+1)*chunk_size, num_frames)
```

- チャンク数 = ceil(F / chunk_size)。最終チャンクのみ端数
- ファイル名: `chunk_{index:05d}_{fid0:08d}_{fid1:08d}.mp4`（書き込み中は末尾 `.tmp`）
- **境界条件**: F <= chunk_size → 1チャンク。chunk_size は argparse の型検証で 1 以上を強制（0以下は `parser.error`）

### 4.5 チャンク直列レンダリング（FR-005）

MP4 モードの本体処理。メインプロセス内でチャンク番号順に直列実行する。

**処理ロジック**（`render_chunk(chunk, viewmats, valid, config, gaussians) -> tuple[int, float]`。戻り値 = (描画フレーム数, 所要秒)）

PLY（gaussians）の所有は main 側とする: main がチャンクループ内で必要時に1回だけロードし、以後すべての `render_chunk` 呼び出しに**同じ参照**を渡す。`render_chunk` は gaussians をロードしない（全フレーム黒のチャンクでは None のまま渡され、参照されない）。これにより「PLY はGPUに1回だけロードし全チャンクで使い回す」（FR-005）が成立する。

```
main 側（チャンクループの前）:
    関数内 import: torch, render
    torch.cuda.set_device(gpu)          # --gpu で指定した1枚
    gaussians = None                     # PLY は必要になるまでロードしない
    black = bytes(height * width * 3)    # 黒フレームの rawvideo データ（1回だけ生成）

main 側（チャンクごと。スキップ対象を除き番号順）:
    if chunk範囲の valid に True があり gaussians is None:
        gaussians = render.load_ply(ply_path)        # current device にロードされる（main が保持）
    n_rendered, sec = render_chunk(chunk, viewmats, valid, config, gaussians)

render_chunk 内:
    t0 = time.time()
    ffmpeg 起動（§4.6。出力は <chunk名>.tmp）
    for i in chunk範囲:
        if valid[i]:
            camera = {width, height, fx, fy, cx, cy,
                      "viewmat": torch.tensor(viewmats[i], dtype=float32, device="cuda"),
                      "background": 黒テンソル (1,3)}
            img = render.render_frame(gaussians, camera)   # (H,W,3) [0,1]
            buf = (img.cpu().numpy() * 255).astype(uint8).tobytes()
        else:
            buf = black
        ffmpeg.stdin.write(buf)   # BrokenPipeError → ffmpeg stderr を読んで失敗扱い
    ffmpeg.stdin.close(); ffmpeg.wait()
    returncode != 0 → 失敗扱い（stderr 添付）
    durable_replace(tmp, 最終ファイル名)                   # 完成の確定（耐久リネーム）
```

`durable_replace(tmp, dst)` は `fsync_file(tmp)` → `os.replace(tmp, dst)` → `fsync_dir(os.path.dirname(os.path.abspath(dst)))` を**この順で**行う共通ヘルパーで、（dirname は abspath 経由で取る。`--output out.mp4` のような basename のみの有効入力で `os.path.dirname` が空文字になり fsync が壊れるのを防ぐ）確定書き出しのすべて（区間MP4・manifest.json・最終MP4・ポーズダンプJSON）で使う。`fsync_file(path)` は `os.open(path, os.O_RDONLY)` → `os.fsync(fd)` → `os.close(fd)`。`fsync_dir(path)` はディレクトリを `os.open(path, os.O_RDONLY)` で開いて同様に fsync する（rename エントリの永続化。POSIX の標準手順）。これにより「確定ファイル名が存在する ＝ 中身がディスクに書き切られている」が電源断でも成立する。

- レンダリングは `torch.no_grad()` 内で行う
- 背景は黒 (0,0,0) 固定（CLI 非公開。`render_frame` の background に黒テンソルを渡す）
- **失敗時の全体挙動**: チャンクが失敗（ffmpeg エラー・CUDA エラー・書き込み失敗）したら、当該チャンクの `.tmp` を削除（best-effort）し、後続チャンクを処理せず「エラー: チャンク {index:05d} が失敗しました: {理由}」で終了コード1。完成済みチャンクは残る（FR-007 で再開）
- **KeyboardInterrupt**: ffmpeg 子プロセスを terminate し、終了コード 130。`.tmp` が残り得るが、次回実行時に同名 `.tmp` へ上書きされるため再開に影響しない

### 4.6 ffmpeg 起動（FR-005）

```
def start_ffmpeg_chunk(out_tmp_path, width, height, fps, crf, preset) -> subprocess.Popen:
    ffmpeg_bin = shutil.which("ffmpeg")   # main 起動時にも存在チェック済み（二重防御）
    cmd = [ffmpeg_bin, "-y",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
           "-r", str(fps), "-i", "pipe:0",
           "-c:v", "libx264", "-crf", str(crf), "-preset", preset,
           "-pix_fmt", "yuv420p", "-loglevel", "error",
           "-f", "mp4", out_tmp_path]
    return subprocess.Popen(cmd, stdin=PIPE, stderr=PIPE)
```

- エンコーダは libx264 固定（ADR-2）。`render.py` の NVENC 自動検出は踏襲しない
- 出力先の拡張子は `.tmp` で ffmpeg が拡張子からコンテナを判別できないため、出力側の `-f mp4` を必ず指定する（上記スニペットの最後の2引数。テスト T14 でスモーク検証する）

### 4.7 マニフェストと再開（FR-007）

**チャンクディレクトリ**: `<output>.chunks/`（例: `data/session001_fps.mp4.chunks/`）
**マニフェスト**: `<chunk_dir>/manifest.json`

```json
{"schema": 1,
 "npz_path": "<abspath>", "ply_path": "<abspath>", "toml_path": "<abspath>",
 "camera": "cam...", "fps": 30.0, "width": 1920, "height": 1080,
 "chunk_size": 10000, "crf": 18, "preset": "medium",
 "num_frames": 194279, "fid_first": 145749, "fid_last": 340027}
```

**処理手順（main、MP4モード）**

1. `--overwrite` 指定時: chunk_dir が存在すれば `shutil.rmtree` で削除
2. chunk_dir が存在しない → `os.makedirs` で作成し、直後に `fsync_dir(os.path.dirname(os.path.abspath(chunk_dir)))` でディレクトリエントリ自体を永続化する（abspath 経由は §4.5 の durable_replace と同じ理由）。続けてマニフェストを `write_manifest`（内部で `durable_replace` を使用）で耐久書き出しする（再開判定が manifest に依存するため、電源断で manifest だけ欠損すると完成済みチャンクが再開不能になるのを防ぐ）
3. chunk_dir が存在する → `load_manifest` で読む:
   - manifest.json がない（`FileNotFoundError`）→ エラー終了「チャンクディレクトリが不完全です（manifest.json なし）。--overwrite で作り直してください: {path}」
   - manifest.json が JSON として読めない（`json.JSONDecodeError` を捕捉し `ValueError` に変換）→ エラー終了「manifest.json が壊れています。--overwrite で作り直してください: {path}」
   - manifest.json を読み、現在のパラメータと全キー比較。不一致キーがあれば「パラメータが変更されています（不一致: chunk_size, crf）。--overwrite で作り直してください」で終了コード1
   - 一致 → 再開モード。最終ファイル名が存在するチャンクごとに `verify_chunk_mp4(path, expected_frames)`（§4.7.1）を実行し、合格したものだけをスキップ対象にする。不合格は「警告: チャンク {index:05d} が壊れています（{理由}）。作り直します」を stderr に出してファイルを削除し、再レンダリング対象に戻す
4. スキップ対象はチャンクループで処理せず、ログに「チャンク {index} スキップ（完成済み）」を出す

### 4.7.1 チャンク整合検証（verify_chunk_mp4）

存在＝完成とは扱わない（電源断・OSクラッシュでは fsync 導入後も、旧バージョン生成物や手動コピー等で壊れたファイルが置かれる可能性が残るため、読む側でも検証する）。

```
def verify_chunk_mp4(path: str, expected_frames: int) -> tuple[bool, str]:
    # ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames \
    #         -of default=nokey=1:noprint_wrappers=1 <path>
    # を subprocess.run で実行する。
    # - returncode != 0 → (False, "コンテナ読み取り不能")
    # - 出力を int にパースできない → (False, "フレーム数取得不能")
    # - int値 != expected_frames → (False, f"フレーム数不一致（実際{n} 期待{expected_frames}）")
    # - 一致 → (True, "")
```

- nb_frames は MP4 コンテナのメタデータ（moov 内のサンプル数）から得られるため**全フレームのデコードは発生しない**（1ファイルあたりミリ秒〜数十ミリ秒）
- 途中で切れたファイルは、「コンテナ読み取り不能」（ffprobe が非ゼロ終了）または「フレーム数取得不能」（ffprobe が終了コード0で正常終了するが nb_frames を出力しない）の**いずれか**として検出される。どちらの理由になるかは ffprobe のバージョン・切断位置に依存するが、どちらの経路でも不合格になるため検出漏れはない（2026-08-08 実機検証: git master ビルドの ffprobe は切り詰めファイルに対し終了コード0・nb_frames 空出力だった。moov 欠損で非ゼロ終了するビルド・切断位置も存在する）
- ffprobe バイナリは MP4 モードの開始前検査（§4.11）で `shutil.which("ffprobe")` により確認する

パス比較は `os.path.abspath` 同士の文字列一致。fps/crf 等は値の一致（float は `==`。CLI からの入力同士なので丸め誤差は生じない）。

### 4.8 連結（FR-008）

全チャンク完成（スキップ含む）後に main が実行する。

```
def concat_chunks(chunk_dir, chunk_names: list[str], output_path) -> None:
    # concat_list.txt を chunk_dir に書く。各行: file '<チャンクの絶対パス>'
    # ffmpeg -y -f concat -safe 0 -i concat_list.txt -c copy -loglevel error -f mp4 <output_tmp>
    #   （output_tmp は `.tmp` 拡張子でコンテナを判別できないため、出力側の -f mp4 が必須。§4.6 と同じ）
    # returncode != 0 → RuntimeError(stderr)
    # durable_replace(output_tmp, output_path)（§4.5 の共通ヘルパー）
```

- `chunk_names` は plan_chunks の順（index 順）で渡す（glob しない）
- output_tmp は `<output_path>.tmp`
- 全チャンクが同一エンコードパラメータのため `-c copy` で連結可能
- 連結成功後: `--keep-chunks` でなければ `shutil.rmtree(chunk_dir)` し、ログに削除を明示
- 総フレーム数の機械検証（ffprobe）はプログラムには組み込まない（100万フレームの全デコードに数分かかるため）。手動テスト手順として ffprobe での確認を要求する（`ffprobe -count_frames -show_entries stream=nb_read_frames <output>`）

**境界条件**: チャンクが1個の場合も concat 経路で統一する（分岐を作らない）。

### 4.9 デバッグ静止画モード（FR-009）

- 発動条件: `--still-range START END` 指定（両端含む、frame_id 基準）
- 検証: START > END → エラー。START または END が `[frame_ids[0], frame_ids[-1]]` の範囲外 → エラー（NPZ の範囲を表示）。`--still-dir` 未指定 → エラー
- インデックス変換: `i = fid - int(frame_ids[0])`（frame_ids は昇順連番のため）
- 処理: `--gpu` で指定した GPU を `torch.cuda.set_device` し、範囲内フレームを順に描画（有効フレーム）または黒画像生成（無効フレーム）して `<still-dir>/frame_{fid:08d}.png` に Pillow で保存する。範囲内に有効フレームが1つもなければ PLY をロードしない
- still-dir は `os.makedirs(exist_ok=True)` で作成する
- チャンク・マニフェスト・ffmpeg は一切使わない

### 4.10 ポーズダンプモード（FR-010）

- 発動条件: `--dump-poses PATH` 指定
- 出力JSONスキーマ:

```json
{"npz_path": "<abspath>", "num_frames": 194279,
 "frames": [{"frame_id": 145749, "valid": true,  "c2w": [[...4要素...]×4行]},
            {"frame_id": 145750, "valid": false, "c2w": null}, ...]}
```

- c2w は Blenderカメラ規約・float64 のリスト表現（FR-012 で旧パイプラインJSONの `c2w` と直接比較するため規約を揃える）
- 耐久書き出し（tmp → `durable_replace`。§4.5 の共通ヘルパー）
- GPU・PLY・TOML 解像度検証はこのモードでも実行するか → **TOML は読む**（引数必須のため整合検証も同一に走る）が、**GPU チェック・ffmpeg チェックは行わない**（NFR-001: GPU 不要）
- 巨大NPZでは出力が数百MBになるため、開始時に「ポーズダンプ: {F} フレームを {path} に書き出します」を表示する

### 4.11 CLI・引数検証（全FR共通の入口）

argparse（`allow_abbrev=False`）。検証順序（重い処理より先に安価な検証を全て行う）:

1. argparse 型レベル: `--fps` は float かつ > 0（カスタム型）、`--chunk-size` は int かつ >= 1、`--crf` は int かつ 0〜51、`--gpu` は int かつ >= 0（カスタム型）
2. 排他制約: `--still-range` と `--dump-poses` の同時指定 → `parser.error`。排他モード時に `--output` / `--overwrite` / `--keep-chunks` の指定 → `parser.error`。`--still-range` 時に `--still-dir` なし → `parser.error`
3. ファイル存在: ply_path / npz_path / --toml が存在しない → エラー終了（パス表示）
4. `--output` の拡張子が `.mp4` でない → エラー終了（MP4モードのみ）
5. MP4モードのみ: `shutil.which("ffmpeg")` と `shutil.which("ffprobe")` の検査に加え、`ffmpeg_has_libx264(ffmpeg_bin)` で libx264 エンコーダの使用可否を検査する（`ffmpeg -hide_banner -encoders` の出力に `libx264` が含まれるかで判定。`render.py` の NVENC 検出と同方式）。不可なら「エラー: ffmpeg に libx264 エンコーダがありません」で終了コード1（静止画モードはいずれも不要）。MP4モード・静止画モード: torch import 後 `torch.cuda.device_count()` で `--gpu` の範囲検査
6. NPZ ロード・頭部7点検証（§4.1）、TOML ロード・偶数検証（§4.2）

`main(argv=None) -> int` 形式とし、`sys.exit(main())` で終了する（既存スクリプトの慣例に合わせ、テストから呼べるようにする）。

## 1.5 状態遷移

GUI・常駐処理はないため対象外（バッチ処理。モードは CLI 引数で起動時に確定し、実行中に遷移しない）。

## 1.6 ファイル・ディレクトリ設計

| ファイル | パス | 命名・形式 |
|---|---|---|
| 最終MP4 | `--output`（デフォルト: NPZと同ディレクトリの `<NPZ名>_fps.mp4`） | H.264/yuv420p, CRF=`--crf`, preset=`--preset`, fps=`--fps` |
| チャンクディレクトリ | `<output>.chunks/` | 連結成功後に削除（`--keep-chunks` で保持） |
| 区間MP4 | `<chunk_dir>/chunk_{index:05d}_{fid0:08d}_{fid1:08d}.mp4` | 書き込み中は `.tmp` 付き |
| マニフェスト | `<chunk_dir>/manifest.json` | §4.7 のスキーマ |
| 連結リスト | `<chunk_dir>/concat_list.txt` | ffmpeg concat demuxer 形式 |
| 静止画 | `<still-dir>/frame_{frame_id:08d}.png` | RGB PNG |
| ポーズダンプ | `--dump-poses` のパス | §4.10 のJSONスキーマ |

## 1.7 インターフェース定義

公開関数（テスト対象。すべて `render_fps_video.py` 内）:

```python
HEAD_JOINT_NAMES: list[str]                     # 頭部7点の関節名（順序固定）
EPS: float = 1e-8
ORTHO_TOL: float = 1e-6

def resolve_head_indices(joint_names: list[str]) -> dict[str, int]
def load_intrinsics(toml_path: str, camera_name: str) -> dict           # 関数内で render_keypoints を import
def compute_fps_poses(x3d_world: np.ndarray, head_idx: dict[str, int]
                      ) -> tuple[np.ndarray, np.ndarray, list[tuple[int, str]]]
    # 戻り値: (c2w_b (F,4,4) float64, valid (F,) bool, invalid_reasons)
def c2w_blender_to_viewmats(c2w_b: np.ndarray) -> np.ndarray            # (F,4,4) float32
def plan_chunks(num_frames: int, chunk_size: int, frame_ids: np.ndarray) -> list[dict]
def chunk_filename(chunk: dict) -> str
def build_manifest(args, num_frames: int, fid_first: int, fid_last: int,
                   width: int, height: int) -> dict
def manifest_mismatch_keys(saved: dict, current: dict) -> list[str]
def start_ffmpeg_chunk(out_tmp_path: str, width: int, height: int,
                       fps: float, crf: int, preset: str) -> subprocess.Popen
def format_render_average(n_rendered: int, render_seconds: float) -> str
def fsync_file(path: str) -> None
def fsync_dir(path: str) -> None
def durable_replace(tmp_path: str, dst_path: str) -> None   # fsync_file → os.replace → fsync_dir
def write_manifest(chunk_dir: str, manifest: dict) -> None   # durable_replace を使用
def load_manifest(chunk_dir: str) -> dict                    # 不在→FileNotFoundError, 破損→ValueError
def ffmpeg_has_libx264(ffmpeg_bin: str) -> bool              # -encoders 出力に libx264 が含まれるか
def verify_chunk_mp4(path: str, expected_frames: int) -> tuple[bool, str]
def concat_chunks(chunk_dir: str, chunk_names: list[str], output_path: str) -> None
def render_chunk(chunk: dict, viewmats, valid, config: dict, gaussians) -> tuple[int, float]
def main(argv=None) -> int
```

- 呼び出し方向: `main` → 各関数（一方向。循環依存なし）

## 1.8 ログ・デバッグ設計

logging モジュールは使わず print とする（phase4 既存スクリプトの慣例）。通常情報は stdout、警告・エラーは stderr。メッセージは日本語。

| 時点 | 出力例 |
|---|---|
| 開始 | `NPZ読み込み: {path}` / `  フレーム数 F={F}, 関節数 J={J}, frame_ids {fid0}〜{fid1}` |
| 判定 | `有効フレーム: {n_valid} / {F}（NaN欠損: {n_nan}, 縮退: {n_degen}）` |
| 縮退警告 | `警告: frame_id {fid}: {理由}`（stderr、縮退・回転検証失敗の1件ごと） |
| 計画 | `チャンク: {n}個（chunk_size={s}）, 使用GPU: {id}, 解像度 {W}x{H}, fps={fps}, crf={crf}, preset={preset}` / `出力: {output}` |
| 再開 | `チャンク {index:05d} スキップ（完成済み）` |
| 破損検出 | `警告: チャンク {index:05d} が壊れています（{理由}）。作り直します`（stderr） |
| チャンク完了 | `チャンク {index:05d} (fid {fid0}-{fid1}) 完了: 描画 {n_rendered}/{n} フレーム, {sec:.1f}秒` |
| 失敗 | `エラー: チャンク {index:05d} が失敗しました: {理由}`（stderr） |
| 連結 | `連結: {output}（{F} フレーム）` / `チャンクディレクトリ削除: {chunk_dir}` |
| 終了 | `完了: 総所要 {sec:.1f}秒, 描画フレーム平均 {format_render_average(n_rendered, render_sec)}`。`format_render_average` は n_rendered == 0 なら `"N/A（描画0フレーム）"`、それ以外は `f"{render_sec / n_rendered:.3f}秒/フレーム"` を返す（ゼロ除算防止。全フレーム黒でも完走する） |

## 5. 等価性検証（FR-012、案A）の設計

実施は実装完了後・手動テスト前。CLAUDE.md「実験・検証の進め方」に従い、着手前に criteria 文書を作成して Codex レビューで lock する。本節は実験の設計方針のみ定める（数値予測は実施直前に experiment_log.md へ記録する）。

- 場所: `docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/`（criteria.md / experiment_log.md）
- 使用データ: 300フレームの短尺データ（feat-018/019 で使用した `session001_f145749_world300.npz` 系列と対応する `.blend`）。旧パイプラインを回すため Blender が必要（ローカル機で実施）
- 手順の骨子:
  1. 旧経路: 対応 `.blend` に対し `blender -b ... --python fps_camera_pose.py -- --camera Cam_FPS` でポーズJSONを取得
  2. 新経路: 同一NPZに対し `render_fps_video.py --dump-poses` でポーズJSONを取得
  3. 照合スクリプト（experiments 配下の使い捨てスクリプト）で全フレームの c2w を比較: 位置差 = ‖t_old − t_new‖、回転角度差 = arccos((trace(R_old^T R_new) − 1)/2)
- 合格基準（requirements FR-012 で確定済み）: 位置差 max < 1mm かつ 回転角度差 max < 0.1°
- 比較上の注意: 旧経路の c2w は Blenderカメラ規約なので、新経路のダンプ（同じくBlenderカメラ規約）とそのまま比較する。フレーム対応は C3D フレーム 1..F ↔ NPZ インデックス 0..F−1（`npz_to_c3d.py` の対応ログ仕様）

## 6. 設計判断の記録（ADR）

| ID | 採用 | 却下と理由 |
|---|---|---|
| ADR-1 | チャンクはフレーム数固定の機械分割 | 有効区間単位の分割: 区間長がまちまちで並列効率が落ち、再開判定も複雑化（2026-08-07 ヒアリングで固定長を確定） |
| ADR-2 | エンコーダは libx264 固定 | NVENC 自動検出（render.py 方式）: A100 は NVENC 非搭載であり、マシンによってコーデックパラメータが変わると `-c copy` 連結や再開時の一貫性が壊れる |
| ADR-3 | c2w は Blenderカメラ規約を正本とし、OpenCV変換は `render.py` の式を踏襲 | 最初から OpenCV 規約で組み立てる案: 旧パイプライン（Blender規約JSON）との等価性検証で直接比較できなくなる |
| ADR-4 | チャンク完成判定 = アトミックリネーム（tmp → fsync → `os.replace` → ディレクトリ fsync）+ マニフェストのパラメータ一致 + 再開時の ffprobe メタデータ検証（§4.7.1。デコードなし） | (1) ファイル存在のみの判定（初版設計）: 電源断・OSクラッシュで「確定名だけ存在して中身欠損」を見逃す（2026-08-07 人レビュー指摘で変更）。(2) 全フレームデコードによる検証: 100万フレーム規模では遅すぎる。メタデータ検証（moov 読み取りのみ）で切り詰め破損は確実に検出できる |
| ADR-5 | 単一GPU・単一プロセスの直列チャンクループ（2026-08-07 人レビューで並列実装を後続案件に分割） | multiprocessing ワーカー方式（初版設計）: ローカル機はGPU1枚で複数GPU固有の経路を検証できず、未検証コードを本番（A100）に持ち込むリスクがあるため本案件から除外。チャンクの独立性（自己完結の区間MP4+チャンク単位の完成判定）は維持し、後続案件が分配機構の追加で並列化できるようにする |
| ADR-6 | ポーズは全フレーム事前計算（チャンクループの前） | チャンクループ内での逐次計算: メモリは全載せで十分小さく（NFR-002）、ダンプモード・等価性検証と計算経路を共有できる事前計算が単純 |
| ADR-7 | 黒フレームは固定バイト列を直接 ffmpeg へ | 黒を gsplat でレンダリング: GPU が無駄（hearing-notes.md「黒区間はGPU不要」） |
| ADR-8 | 動画 I/O は ffmpeg パイプ（既存踏襲） | imageio / OpenCV VideoWriter: 新規依存の追加や細かいエンコード制御の喪失を避ける |
| ADR-9 | 縮退フレームは黒＋警告で続行 | エラー終了（旧実装）: 5〜10時間バッチが1フレームで停止する（2026-08-07 ヒアリングで確定） |

## 7. テスト設計

- テストファイル: `tests/test_feat027_render_fps_video.py`
- 実行: `uv run --project phase4 --with pytest pytest tests/test_feat027_render_fps_video.py -v`（feat-028 と同方式。scipy ではなく torch 系が root 環境にないため、GPU・torch 不要の純ロジックに限定しつつ、tomli 依存テストは `pytest.importorskip("tomli")` でガードする）
- 全体回帰: `uv run pytest -v`（root 環境。torch/tomli 非依存のテストはそのまま走り、依存テストは skip される設計とする）
- テスト結果保存: `tests/results/feat-027_test_result.txt`

| # | 対象 | 内容 |
|---|---|---|
| T1 | compute_fps_poses | 合成データ（既知配置: 両目をX軸上、鼻を+Y方向等）で c2w の位置=両目中点・各軸方向が期待値と一致 |
| T2 | compute_fps_poses | head_up 符号反転ケース（上下逆の頭部配置）で u が反転される |
| T3 | compute_fps_poses | 縮退4種（Nose=耳中点 / LEye=REye / p∥r0 / Head=Neck）で該当フレームのみ invalid になり理由文字列が対応する |
| T4 | compute_fps_poses | NaN フレーム（全点・部分）が invalid になり invalid_reasons に**含まれない**（警告なし仕様） |
| T5 | compute_fps_poses | ベクトル化実装 vs テスト内のフレーム単位スカラー参照実装（mathutils を使わない素朴な numpy 移植）でランダムデータ一致（atol=1e-12） |
| T6 | c2w_blender_to_viewmats | 逆行列性は float64 で検証: viewmat を float64 に昇格して viewmat @ (c2w_b @ diag(1,-1,-1,1)) ≈ I（有効フレーム全件、atol=1e-5。入力 viewmat が float32 のため丸めを許容）。戻り値の dtype == float32 は別アサーションで検証 |
| T7 | plan_chunks / chunk_filename | F=1 / F=chunk_size / F=chunk_size+1 の分割数・境界・fid 対応・ファイル名形式 |
| T8 | manifest | build → mismatch なし。chunk_size / crf を変えると当該キーが検出される。`write_manifest` → `load_manifest` の往復で内容一致。壊れたJSONファイルで `load_manifest` が「manifest.json が壊れています」を含む ValueError を送出する |
| T9 | resolve_head_indices | 7点解決成功、欠落時に欠落名を含む ValueError |
| T10 | CLI | `--fps` 欠落 / fps<=0 / crf 範囲外 / `--gpu` 負値 / 排他違反 / `--still-range` に `--still-dir` なし → SystemExit(2)（parser.error） |
| T11 | load_intrinsics | 合成TOML（tmp_path）から K・解像度取得。奇数解像度で ValueError。（`pytest.importorskip("tomli")`） |
| T12 | dump-poses | 小合成NPZ（tmp_path）+ 合成TOML で main() を実行し、JSON の frames 数・valid・c2w 形状を検証（GPU 不要） |
| T13 | F=0 検証 | F=0 の合成NPZで main()（dump-poses モード）が戻り値1・エラーメッセージを返す |
| T14 | start_ffmpeg_chunk | `.mp4.tmp` パスへ黒フレーム数枚を書き込み、returncode 0・非空ファイル生成を検証（`-f mp4` 指定のスモークテスト。ffmpeg が不在または libx264 エンコーダが使用不可の環境では skip） |
| T15 | format_render_average | n_rendered=0 で `"N/A（描画0フレーム）"`、n_rendered>0 で `"{:.3f}秒/フレーム"` 形式を返す |
| T16 | concat_chunks | T14 の方式で生成した区間MP4を2本連結し、`<output>.mp4.tmp` 経由で最終MP4が生成される（returncode 0・非空・`os.replace` 後に最終パスが存在）ことを検証（連結側 `-f mp4` のスモークテスト。ffmpeg が不在または libx264 エンコーダが使用不可の環境では skip） |
| T17 | verify_chunk_mp4 | T14 で生成した正常チャンクは (True, "")。同ファイルを途中で切り詰めたコピーは False で、理由は「コンテナ読み取り不能」または「フレーム数取得不能」のいずれか（§4.7.1 のとおり ffprobe のバージョン・切断位置に依存するため両方を許容する）。正常ファイルに期待フレーム数+1 を渡すと (False, フレーム数不一致) （ffmpeg/ffprobe が不在または libx264 エンコーダが使用不可の環境では skip） |
| T18 | durable_replace | monkeypatch で `fsync_file` / `os.replace` / `fsync_dir` の呼び出しを記録し、この順で1回ずつ呼ばれること・確定パスにファイルが存在することを検証（ffmpeg 不要。全確定書き出し経路の順序保証をこの1テストで担保する）。dst が basename のみ（ディレクトリ部なし。tmp_path に chdir して実行）でも `fsync_dir` に空文字が渡らず成功することを検証 |
| T19 | ffmpeg_has_libx264 | monkeypatch で `subprocess.run` の出力を差し替え、`-encoders` 出力に `libx264` を含む場合 True・含まない場合 False を返すことを検証（実 ffmpeg 不要） |

GPU を要するレンダリング経路（FR-005 / FR-007 / FR-008 / FR-009 の実描画）は自動テスト対象外とし、手動テスト（実データ・ローカルGPU → A100）で確認する。

## 8. 実装メモ（Sonnet サブエージェント向け注意）

- 環境操作は uv のみ。`pip install` / `python -m venv` / `.venv` 直接操作は禁止（settings.json の deny ルールでブロックされる）
- 既存スクリプト（`render.py` / `render_keypoints.py` / `npz_to_c3d.py` / `fps_camera_pose.py` / `filter_npz.py`）は一切変更しない
- 実行確認コマンド例（ローカル機は `TORCH_CUDA_ARCH_LIST="9.0+PTX"` が必要。CLAUDE.md 参照）:

```bash
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python render_fps_video.py \
  data/<PLY> data/<NPZ> --toml data/<TOML> --camera <カメラ名> --fps 30 --gpu 0
```

- 本設計書のコードスニペットは意図の伝達が目的であり、そのままコピーして使うものではない

# 機能設計書: feat-045 スマートフォン静止画のカメラパラメータ推定ウィザード

## 1. 対応要求マッピング

| 要求 | 設計 |
|---|---|
| FR-001 原本と作業フォルダ（ジョブフォルダ）の指定 | §4.3 手順 1〜3、§4.4 |
| FR-002 範囲画像の書き出し | §4.3 手順 4、§4.5 |
| FR-003 2D・3D 基準点の確認 | §4.3 手順 5、§4.6 |
| FR-004 入力のコピーと共用設定YAML | §4.2、§4.3 手順 6〜7 |
| FR-005 3 ツールの実行 | §4.3 手順 8、§4.7 |
| FR-006 中断と終了コード | §4.8 |

## 2. システム構成

### 2.1 変更対象

| ファイル | 変更 |
|---|---|
| `phase4/estimate_wizard.py` | 新規 |
| `tests/test_feat045_estimate_wizard.py` | 新規 |

既存のコード（3 ツールを含む）は変更しない。

### 2.2 依存関係

- `prepare_phone_photo` の `compute_crop`・`convert_point`・`OUT_W`・`OUT_H`・`MIN_POINTS`（打てる範囲と点数の判定。前処理と同じ計算）、`read_exif`・`validate_exif`（原本の Exif 検査。前処理と同じ判定）
- `extract_photo_info.read_focal_35mm`（35mm 換算焦点距離の有無。書き出しと同じ判定）
- `refine_extrinsics.load_points_2d_csv`・`load_points_3d_csv`・`InputError`（CSV の読み込みと重複・必須列・非有限値の検査）
- 3 ツールは関数を import するだけで、本体（`main`）は子プロセスとして実行する（`sys.executable` で、ウィザードと同じ Python）
- Pillow はウィザード自身では import しない。`read_exif`・`read_focal_35mm` は関数の中で Pillow を遅延 import する（phase4 環境にはある）。ルート環境のテストではこの 2 つを `monkeypatch` で差し替える（`tests/test_feat044_photo_info.py` と同じ方式）

## 3. 技術スタック

追加なし（Python 3.10、OpenCV、NumPy、標準ライブラリの `shutil`・`subprocess`・`argparse`）。

## 4. 詳細設計

### 4.1 定数

```python
PHASE4_DIR = Path(__file__).resolve().parent
JOBS_DIR = PHASE4_DIR / "data" / "EIL"   # 作業フォルダの親（feat-045 イテレーション1）
JOB_NAME_RE = re.compile(r"[A-Za-z0-9_-]+")
TOOLS = ["extract_photo_info.py", "prepare_phone_photo.py", "estimate_intrinsics_loop.py"]
DARKEN = 0.5            # 範囲外の明るさの倍率
LINE_COLOR = (0, 0, 255)  # BGR 赤
```

### 4.2 ジョブフォルダの構成（`job_paths(job: Path, original_suffix: str) -> dict[str, Path]`）

`job` は絶対パス（`(JOBS_DIR / 作業フォルダ名).resolve()`）。名前はすべて固定で、共用設定YAMLに絶対パスで書き出す（3 ツールが名前を推測することはない）。

| キー | パス | 作るもの |
|---|---|---|
| `original` | `job / ("original" + original_suffix)` | ウィザード（原本のコピー。拡張子は原本のまま） |
| `crop_area` | `job / "crop_area.png"` | ウィザード（範囲画像） |
| `points_check` | `job / "points_check.png"` | ウィザード（確認画像） |
| `points_2d_original` | `job / "points_2d_original.csv"` | ウィザード（2D 点 CSV のコピー） |
| `points_3d` | `job / "points_3d.csv"` | ウィザード（3D 基準点 CSV のコピー） |
| `ply` | `job / "point_cloud.ply"` | ウィザード（PLY のコピー） |
| `job_yaml` | `job / "job.yaml"` | ウィザード（共用設定YAML） |
| `photo_info` | `job / "photo_info.yaml"` | 書き出し |
| `images_dir` | `job / "images"` | 前処理（中に `{カメラ名}.png`） |
| `init_toml` | `job / "init.toml"` | 前処理 |
| `points_2d` | `job / "points_2d.csv"` | 前処理 |
| `out_dir` | `job / "out"` | 推定（存在しない新しいパスとして推定が作る） |

- 範囲画像・確認画像は PNG（劣化しない形式。FR-002 受け入れ基準 2 を画素値で確かめるため）
- 推定の `tmp_dir` は書かない（推定の既定の動き）
- 写真情報ファイルと前処理の出力は `out_dir` の外にある（feat-044 の配置の制約を満たす）

共用設定YAML（`format_job_yaml(camera: str, p: dict[str, Path]) -> str`）の中身（この順・この形）:

```yaml
# 共用設定YAML（estimate_wizard.py が作成。feat-045）
camera: {camera}
image: {p["original"]}
photo_info: {p["photo_info"]}
points_2d_original: {p["points_2d_original"]}
images_dir: {p["images_dir"]}
init_toml: {p["init_toml"]}
points_2d: {p["points_2d"]}
points_3d: {p["points_3d"]}
ply: {p["ply"]}
out_dir: {p["out_dir"]}
```

### 4.3 処理手順（`main(argv: list[str] | None = None, input_fn=input) -> int`）

`input_fn` はテストで差し替えるための引数。

0. `argparse.ArgumentParser(description=..., allow_abbrev=False).parse_args(argv)`。引数は受け付けない（`--help` のみ。余計な引数は argparse により終了コード 2）
1. **原本**: `ask(input_fn, "原本の画像のパス")`。`Path(値).expanduser().resolve()` を `original_src` とする。`check_original(original_src)`（§4.4）が誤りの文言のリストを返したら、すべて表示して聞き直す。成功したら戻り値の `img`・`w`・`h`・`y0`・`crop_h`・`rng`（打てる範囲 `(x_min, x_max, y_min, y_max)`）を保持し、「原本サイズ {w}x{h}、打てる範囲 x={x_min}〜{x_max}、y={y_min}〜{y_max}」を表示
2. **作業フォルダ**（パスではなく名前だけを受け付ける。investigation.md イテレーション1）: 先に次を表示する（`{JOBS_DIR}` は実際の絶対パス）:

   ```
   推定用の作業フォルダを {JOBS_DIR} の下に新しく作ります。
   このフォルダに、原本・2D/3D 基準点・PLY のコピー、確認用の画像、設定ファイル（job.yaml）、推定結果をすべて保存します（PLY のコピーを含むため 100MB 以上になります）。
   ```

   `name = ask(input_fn, "作業フォルダの名前（英数字・_・- だけ。例 20261126-01）")`。`JOB_NAME_RE.fullmatch(name)` でなければ「エラー: 作業フォルダの名前は英数字・_・- だけで書いてください: {name}」、`os.path.lexists(JOBS_DIR / name)` なら「エラー: その名前の作業フォルダは既にあります: {JOBS_DIR / name}」を表示して聞き直す。`job = (JOBS_DIR / name).resolve()`。続けて `job_place_error(job)`（§4.4）が文言を返したら `エラー: {文言}` を表示して 1 を返す（設置の問題で、名前を変えても直らないため聞き直さない。作業フォルダは作っていない）
3. **カメラ名**: 先に `この写真のカメラ名を決めます。初期 TOML・2D 点 CSV（camera_name 列）・推定結果で、この写真を指す名前として使います。英数字・_・- だけが使えます（先頭の - と metadata は不可）。` を表示する。`default = original_src.stem if camera_name_error(original_src.stem) is None else None`。`ask(input_fn, "カメラ名", default=default)`。`camera_name_error(name)`（§4.4）が文言を返したら表示して聞き直す
   - ここで `job.mkdir(parents=True, exist_ok=False)`。続けて `p = job_paths(job, original_src.suffix)`、`shutil.copyfile(original_src, p["original"])`。「作業フォルダを作りました: {job}」を表示（`JOBS_DIR` が無ければ `mkdir(parents=True)` で作られる）
4. **範囲画像**: `cv2.imwrite(str(p["crop_area"]), draw_crop_area(img, rng))`（戻り値が False なら `OSError` を送出）。次を表示する:

   ```
   範囲画像: {p["crop_area"]}
   2D 点は原本の x={x_min}〜{x_max}、y={y_min}〜{y_max} の範囲（範囲画像の明るい部分）に打ってください。範囲外の点は前処理で除かれます。
   2D 点 CSV の書式（UTF-8）:
     ObjectName,camera_name,X,Y
     基準_001,{camera},188,1485
   camera_name 列には {camera} と書きます。X, Y は原本の画素座標です。
   2D 点 CSV ができたら、そのパスを入力してください。
   ```

   （例の行の座標は書式を示すための固定値）
5. **2D・3D 基準点の確認**（ループ。`prev_2d = None`、`prev_3d = None`）:
   1. `src_2d = ask(..., "2D 点 CSV のパス", default=prev_2d)`、`src_3d = ask(..., "3D 基準点 CSV のパス", default=prev_3d)`（いずれも `expanduser().resolve()`）。`prev_2d, prev_3d = src_2d, src_3d`
   2. `p2 = load_points_2d_csv(src_2d)`、`p3 = load_points_3d_csv(src_3d)`。`InputError`、`OSError`（ファイルが無い等）、`UnicodeDecodeError`（UTF-8 として読めない。表示は「エラー: CSV を UTF-8 として読めません（UTF-8 で保存し直してください）: {パス}」）なら理由を表示して 5.1 へ
   3. `cam_pts = {name: xy for (cam, name), xy in p2.items() if cam == camera}`。空なら「2D 点 CSV に camera_name={camera} の行がありません」を表示して 5.1 へ
   3b. `bad = sorted(n for n in cam_pts if any(c in n for c in (",", '"', "\n", "\r")))`。あれば「エラー: 名前に , " 改行は使えません（前処理が CSV を引用符なしで書くため）: …」を表示して 5.1 へ
   4. `both, only_2d, only_3d = check_names(cam_pts, p3)`（§4.6）。3 つを表示する（各行「両方にある名前（n 個）: 名前, …」の形。0 個なら「なし」）。`only_2d` があれば「エラー: 3D 基準点 CSV に無い名前が 2D 点 CSV にあります」を表示して 5.1 へ
   5. `outside = sorted(n for n, (x, y) in cam_pts.items() if not in_range(float(x), float(y), y0, w))`（§4.6）。`outside` があれば「範囲外のため前処理で除かれる点: …」を表示。`len(cam_pts) - len(outside) < MIN_POINTS` なら「エラー: 範囲内の点が {n} 点です。前処理には {MIN_POINTS} 点以上必要です」を表示して 5.1 へ
   6. `check_img, skipped = draw_points(draw_crop_area(img, rng), cam_pts)`（§4.6）。`skipped` があれば「警告: 画像の外の点は描いていません: …」を表示。`cv2.imwrite(str(p["points_check"]), check_img)`（False なら `OSError`）。「確認画像: {パス}」を表示
   7. `ask_yes_no(input_fn, "点の位置と名前は正しいですか")` が True ならループを抜ける。False なら 5.1 へ
6. **PLY**: `src_ply = ask(..., "3DGS の PLY ファイルのパス")`（`expanduser().resolve()`）。`src_ply.is_file()` でなければ「ファイルがありません」を表示して聞き直す
7. **コピーと共用設定YAML**: `shutil.copyfile` で `src_2d → p["points_2d_original"]`、`src_3d → p["points_3d"]`、`src_ply → p["ply"]`。`p["job_yaml"].write_text(format_job_yaml(camera, p), encoding="utf-8")`。「共用設定YAML: {パス}」を表示
8. **実行**: `manual_commands(p["job_yaml"])`（§4.7）を「手で実行する場合のコマンド:」に続けて表示し、`ask_yes_no(input_fn, "書き出し → 前処理 → 推定 を実行しますか")`。False なら「実行しませんでした。上のコマンドで実行できます。」を表示して 0 を返す。True なら `return run_tools(p["job_yaml"])`

### 4.4 入力の補助関数

```python
def ask(input_fn, label: str, default: str | Path | None = None) -> str:
    """label を表示して 1 行読む。前後の空白を除き、空なら default（None なら聞き直す）を返す。"""
    # 表示: default があれば f"{label} [{default}]: "、無ければ f"{label}: "

def ask_yes_no(input_fn, label: str) -> bool:
    """f"{label} [y/n]: " で聞く。y / yes → True、n / no → False（大文字小文字を区別しない）。それ以外は聞き直す。"""

def job_place_error(job: Path) -> str | None:
    """解決後の作業フォルダの絶対パスの検査（investigation.md イテレーション1、codex-07）。job は resolve 済み。
    str(job) に改行文字（\n \r）を含む、または str(job) != str(job).strip() なら
    「作業フォルダのパスに改行または前後の空白が含まれます（共用設定YAMLに書けないため）: {job!r}」。それ以外は None。
    シンボリックリンクの検査はしない（phase4/data と phase4/data/EIL は通常のディレクトリである前提。ユーザー判断 2026-11-26、codex-08）"""

CAMERA_NAME_RE = re.compile(r"[A-Za-z0-9_-]+")

def camera_name_error(name: str) -> str | None:
    """次の 3 つを満たせば None。違反なら理由の文言:
    - CAMERA_NAME_RE.fullmatch(name)（違反:「カメラ名は英数字・_・- だけで書いてください（初期 TOML のテーブル名・CSV・ファイル名にそのまま使うため）: {name}」）
    - not name.startswith("-")（違反:「カメラ名は - で始められません（推定が --cameras の値として渡すため）: {name}」）
    - name != "metadata"（違反:「カメラ名に metadata は使えません（最終精緻化が TOML の [metadata] を特別に扱うため）」）"""
    # 英数字・_・- だけなら TOML の裸のキーとして読め（前処理は [カメラ名] と書く）、
    # パス区切り・"."・".."・空白・改行も含まれないので、前処理・推定のカメラ名の規則も満たす

def in_range(x: float, y: float, y0: int, w: int) -> bool:
    """前処理と同じ判定: convert_point(x, y, y0, OUT_W / w) の結果 (u, v) が
    0 <= u <= OUT_W - 1 かつ 0 <= v <= OUT_H - 1 なら True。"""
    # prepare_phone_photo._run 手順 6 の判定式と同じ（scale = OUT_W / width）

def valid_range(w: int, h: int, y0: int) -> tuple[int, int, int, int]:
    """打てる範囲の整数座標 (x_min, x_max, y_min, y_max)。
    x は range(w) の各整数のうち in_range(x, y_c, y0, w) が True のものの最小・最大
    （y_c = y0 + 1。x の判定は y に依存しない）、y は range(h) の各整数のうち
    in_range(x_c, y, y0, w) が True のものの最小・最大（x_c = w // 2）。"""
    # 4000x3000 なら (1, 3998, 376, 2623)。全数を調べるので in_range とちょうど一致する

def check_original(src: Path) -> tuple[list[str], dict | None]:
    """原本の判定（人が点を打つ前に行う）。誤りがあれば (文言のリスト, None)、無ければ ([], 情報)。
    情報は {"img": 画像, "w": 幅, "h": 高さ, "y0": y0, "crop_h": crop_h, "rng": valid_range(w, h, y0)}。"""
    # 1. img = cv2.imread(str(src), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)（前処理と同じ読み方）。
    #    None なら ["画像を開けません: {src}"] を返す
    # 2. h, w = img.shape[:2]。compute_crop(w, h) の ValueError は [str(e)] を返す
    # 3. exif = read_exif(str(src))（OSError は ["Exif を読めません: {src}（{e}）"]）。
    #    exif["Size"] != (w, h) なら ["画像サイズが Exif 側の読み取り値と一致しません …"]（前処理と同じ検査）。
    #    errors = validate_exif(exif, w, h)
    # 4. f35 = read_focal_35mm(str(src))（OSError は同上）。f35 is None or f35 <= 0 なら次の文言を errors に足す:
    #    「Exif に 35mm 換算焦点距離がありません。Exif の無い画像はウィザードでは扱えません。
    #     写真情報ファイルを手で書き、前処理・推定を手で実行してください（README の estimate_intrinsics_loop.py ほかの節）」
    # 5. errors があれば (errors, None)。無ければ ([], 情報)
```

### 4.5 範囲画像（`draw_crop_area(img: np.ndarray, rng: tuple[int, int, int, int]) -> np.ndarray`）

入力 `img` は変更せず、コピーに描く。`h, w = img.shape[:2]`、`x_min, x_max, y_min, y_max = rng`（打てる範囲。§4.4 `valid_range`）。

1. `out = (img * DARKEN).astype(np.uint8)`（全体を暗くする）。`out[y_min:y_max + 1, x_min:x_max + 1] = img[y_min:y_max + 1, x_min:x_max + 1]`（範囲の内側を原本の値に戻す）
2. 線: `t = max(2, round(w / 800))`（4000 幅で 5）。`cv2.line(out, (0, y_min), (w - 1, y_min), LINE_COLOR, t)`、`cv2.line(out, (0, y_max), (w - 1, y_max), LINE_COLOR, t)`（左右の端は各 1 列なので線は引かない）
3. 文字: `font = cv2.FONT_HERSHEY_SIMPLEX`、`scale = w / 1000`、`th_ = max(1, round(w / 1000))`、`gap = 2 * t`、x 位置は `20`
   - 上端 `f"y={y_min}"`: 文字の高さを `tht`（`cv2.getTextSize`）として、基線を `y_min - gap` に置く。`y_min - gap - tht < 0`（上に入らない）なら基線を `y_min + gap + tht` に置く
   - 下端 `f"y={y_max}"`: 基線を `y_max + gap + tht` に置く。`y_max + gap + tht > h - 1` なら基線を `y_max - gap` に置く
   - `cv2.putText(out, 文字, (20, 基線), font, scale, LINE_COLOR, th_, cv2.LINE_AA)`

範囲内の画素は、線と文字が重なる部分を除いて原本と同じ値になる。

### 4.6 名前の照合と確認画像

```python
def check_names(cam_pts: dict[str, object], p3: dict[str, object]) -> tuple[list[str], list[str], list[str]]:
    """(両方にある名前, 2D にだけある名前, 3D にだけある名前)。それぞれ sorted。"""

def point_label(name: str) -> str:
    """ObjectName の最後の数字列（re.findall(r"\d+", name) の最後）。数字が無ければ name 全体。"""
    # phase0/visualize_points_2d.py の extract_point_number と同じ

def draw_points(img: np.ndarray, pts: dict[str, object]) -> tuple[np.ndarray, list[str]]:
    """img のコピーに点とラベルを描く。戻り値は (画像, 画像の外のため描かなかった名前の sorted リスト)。"""
```

描き方は `phase0/visualize_points_2d.py` の `draw_points_on_image`・`draw_label` と同じ形で、画像の幅に合わせて大きさを `s = max(1.0, w / 1920)` 倍する（1920 幅では元と同じ大きさ。4000 幅の原本で見える大きさにするため）:

- 位置 `ix, iy = int(round(x)), int(round(y))`。`ix < 0 or ix >= w or iy < 0 or iy >= h` なら描かずに名前を記録
- `cv2.circle(img, (ix, iy), round(8 * s), (0, 0, 255), max(1, round(2 * s)))`、`cv2.circle(img, (ix, iy), max(1, round(1 * s)), (0, 255, 255), -1)`
- ラベル `point_label(name)`: `font = cv2.FONT_HERSHEY_SIMPLEX`、`scale = 0.6 * s`、`thickness = max(1, round(s))`、オフセット `off = round(10 * s)`、余白 `margin = round(3 * s)`。位置の決め方（右上 → 右端なら左へ → 上端なら下へ → 全方向のクランプ）は `draw_label` と同じで、10・3 を `off`・`margin` に置き換える
- 縁取り `cv2.putText(..., (0, 0, 0), thickness + 2, cv2.LINE_AA)` → 本体 `cv2.putText(..., (0, 255, 0), thickness, cv2.LINE_AA)`

範囲外（暗い部分）の点も、画像の中であれば描く（範囲外に打ったことが分かるように）。

### 4.7 3 ツールの実行

```python
def manual_commands(job_yaml: Path) -> list[str]:
    """手で実行する場合の 3 コマンド（絶対パス。シェルにそのまま貼れるよう shlex.join で引用する）。"""
    # args(tool) = ["uv", "run", "--project", str(PHASE4_DIR), "python", str(PHASE4_DIR / tool), "--config", str(job_yaml)]
    # [shlex.join(args("extract_photo_info.py")),
    #  shlex.join(args("prepare_phone_photo.py")),
    #  'TORCH_CUDA_ARCH_LIST="9.0+PTX" ' + shlex.join(args("estimate_intrinsics_loop.py"))]

def recovery_commands(job_yaml: Path) -> list[str]:
    """失敗後に続きを実行するための 3 コマンド。manual_commands との違い:
    - 前処理に "--overwrite" を足す（途中まで書いた出力があっても上書きして続けられる。対象はジョブフォルダ内の前処理の出力だけ）
    - 推定に "--out-dir", str(新しい出力ディレクトリ) を足す（CLI が設定YAMLより優先）。新しい出力ディレクトリは
      job_yaml.parent / f"out_{n}"（n = 2, 3, … のうち os.path.lexists が False になる最小の n）
    書き出しは変えない。ただし、書き出しが失敗したのに写真情報ファイル（job_yaml.parent / "photo_info.yaml"）が
    存在する場合（書き出しが公開後の一時ファイル削除で失敗した場合など）は、書き出しのコマンドを表示せず、代わりに
    「写真情報ファイルは既にあります: {パス}。中身（focal_length_35mm）を確認し、正しければ次の前処理から続けてください。
     作り直す場合は、そのファイルが写真情報ファイルであることを確かめてから削除し、書き出しから実行してください」
    を表示する（run_tools が i=0 で失敗したときに、この存在確認を行う）。"""

def run_tools(job_yaml: Path, runner=subprocess.run) -> int:
    """3 ツールを順に実行する。runner はテストで差し替える。"""
    # for i, tool in enumerate(TOOLS):
    #     print(f"=== {tool} を実行します ===")
    #     rc = runner([sys.executable, str(PHASE4_DIR / tool), "--config", str(job_yaml)]).returncode
    #     if rc != 0:
    #         print(f"エラー: {tool} が終了コード {rc} で終わりました。ここで止めます。", file=sys.stderr)
    #         残り（i 番目以降）のコマンドを recovery_commands(job_yaml)[i:] で表示（「原因を直したら、次のコマンドで続きを実行できます:」）
    #         return 1
    # print(f"完了しました。推定の出力: {job_yaml.parent / 'out'}")
    # return 0
```

- `runner` には `capture_output` を渡さない（ツールの出力をそのまま画面に流す）。`cwd` も渡さない（親のカレントディレクトリを継承。パスはすべて絶対パスなので影響しない）
- 環境変数はそのまま子プロセスに引き継がれる（`TORCH_CUDA_ARCH_LIST` は人がウィザード起動時に付ける）
- 失敗したときに表示する続きのコマンドは `recovery_commands`（前処理に `--overwrite`、推定に新しい `--out-dir`）。ウィザードはジョブフォルダ内のファイルを消さない。`job.yaml` の `out_dir` は書き換えないので、再実行後の推定の出力は表示した `--out-dir` の先にある

### 4.8 中断・エラーと終了コード

- `main` は手順 1〜8 を `try` で囲み、`KeyboardInterrupt`・`EOFError` で「中断しました。」（作業フォルダ作成後なら「作業フォルダ: {job}（そのまま残しています）」も）を表示して 1 を返す。共用設定YAMLを書き終える前（手順 7 の YAML 書き込みが完了する前）の中断では、続けて「新しい作業フォルダ名で最初からやり直してください。」を表示する。書き終えた後で 3 ツールを起動する前（手順 8 の実行確認での中断）では、続けて「次のコマンドで続けられます:」と `manual_commands(p["job_yaml"])` の 3 行を表示する（YAML 書き込み完了をフラグで記録して判定する）
- `run_tools` は各ツールの実行（`runner(...)`）を `try` で囲み、`KeyboardInterrupt` を捕まえたら「中断しました（{tool} の実行中）。」と、失敗時と同じ続きのコマンド（`recovery_commands(job_yaml)[i:]`。i=0 で写真情報ファイルがある場合の扱いも失敗時と同じ）を表示して 1 を返す
- `OSError`（コピー・書き込み・フォルダ作成の失敗）は `エラー: {e}` を表示して 1 を返す（作業フォルダ作成後ならそのパスも表示）
- 終了コード: 0=成功・実行しないを選んだ、1=ツールの失敗・ファイル操作の失敗・中断、2=引数エラー
- `if __name__ == "__main__": sys.exit(main())`

### 4.9 境界条件

| 条件 | 動き |
|---|---|
| 16:9 で撮った原本（4000x2252） | y0=1。打てる範囲の上端 y_min は `valid_range` で求めた値（y=2）。上端の文字は線の下に描く（§4.5） |
| 2D 点 CSV に他のカメラの行がある | 照合・確認画像には使わず、コピーにはそのまま含める（前処理はカメラ名の行だけ使う） |
| 2D 点が範囲外（暗い部分） | 確認画像には描き、名前を表示する。範囲内が 6 点未満ならウィザードが誤りとして CSV のパスを聞き直す（前処理に進まない） |
| 原本のファイル名に空白がある | カメラ名の既定値が規則違反になることがある。その場合は聞き直す |

## 5. ログ・デバッグ設計

標準出力に各手順の結果（パス、範囲、照合結果）を表示する。エラーは標準エラー出力に `エラー: ` で始めて表示する。ログファイルは作らない。

## 6. テスト設計

### 6.1 新規 `tests/test_feat045_estimate_wizard.py`

ルート環境で実行する（`sys.path` に `phase4` を足す。既存の `tests/test_feat044_photo_info.py` と同じ方式）。GPU・実データ不要。`input_fn` は答えのリストを順に返す関数、`runner` は記録して指定の終了コードを返す偽物にする。画像は `np.zeros` 等で作った小さな画像を `cv2.imwrite` で一時ディレクトリに書く（幅 64、高さ 48 なら `compute_crop` は y0=6、crop_h=36）。

| No | 対象 | 内容 | 期待 |
|---|---|---|---|
| 1 | `draw_crop_area` | 全画素 200 の 64x48 画像、rng=(0, 63, 6, 41)（64x48 の `valid_range`） | 行 0・行 47 の x=0 の画素が 100、行 20 の x=0 の画素が 200（文字は x=20 以降、線は行 5〜7・40〜42 なので x=0 の行 0・20・47 にはかからない）。入力画像は変更されていない |
| 2 | `draw_crop_area` | 4000x3000 の全画素 200 画像、rng=(1, 3998, 376, 2623) | 行 0 と行 2999 の x=0・x=3999 が 100、行 1500 の x=0・x=3999 が 100 で x=1〜3998 が 200、行 376 と行 2623 の x=1000 が赤（B=0, G=0, R=255）（文字は x=20 から約 500 画素の幅なので x=1000 にはかからない） |
| 3 | `point_label` | `基準_001`、`P12a3`、`abc` | `001`、`3`、`abc` |
| 4 | `check_names` | 2D {A,B}、3D {B,C} | (["B"], ["A"], ["C"]) |
| 5 | `draw_points` | 1920x1080 の黒画像に点 (100,100)・(5000,5) | 戻り値の名前リストが 2 点目だけ。(100,100) の画素が黄（B=0, G=255, R=255）、(108,100) 付近に赤 |
| 6 | `camera_name_error` | `cam1`、`s25_2026-11`、`a/b`、`..`、`cam 1`、`写真`、`cam.1`、`a\nb`、``、`-cam`、`metadata` | 最初の 2 つだけ None |
| 7 | `ask` | 答え `""` → default あり / 答え `""`,`x` → default なし | default を返す / `x` を返す（2 回聞く） |
| 8 | `ask_yes_no` | 答え `maybe`,`Y` | True（2 回聞く） |
| 9 | `job_paths`・`format_job_yaml` | job=/tmp/j、suffix `.jpg` | §4.2 の表どおりのパス。YAML の各行が §4.2 の形で、`load_yaml_flat` で読んだキーが 10 個 |
| 10 | 共用設定YAMLの受け付け | No.9 の YAML を一時ディレクトリに書き、3 ツール（`extract_photo_info`・`prepare_phone_photo`・`estimate_intrinsics_loop`）それぞれについて `refine_extrinsics.parse_args_with_config(["--config", yaml], ツール.build_arg_parser(), ツール.CONFIG_CONVERTERS, ツール.REQUIRED_OPTIONS, ツール.IGNORED_KEYS)` を呼ぶ | 3 ツールとも例外・`SystemExit` なし（引数の解釈だけ。ツール本体は実行しない） |
| 11 | `manual_commands` | job_yaml=/tmp/j/job.yaml | 3 行。3 行目だけ `TORCH_CUDA_ARCH_LIST="9.0+PTX"` で始まり、各行が `--config /tmp/j/job.yaml` で終わる |
| 12 | `run_tools` | runner が全部 0 | 3 回呼ばれ、各引数が `[sys.executable, PHASE4_DIR/ツール, "--config", yaml]`（TOOLS の順）。戻り値 0 |
| 13 | `run_tools` | runner が 2 回目に 1 | 2 回だけ呼ばれ、戻り値 1。標準エラーに `prepare_phone_photo.py` と `終了コード 1`。表示された続きのコマンドが 2 行（前処理に `--overwrite`、推定に `--out-dir <job>/out_2`） |
| 13b | `recovery_commands` | ジョブフォルダに `out` と `out_2` が既にある | 推定のコマンドの `--out-dir` が `<job>/out_3`。書き出しのコマンドは `manual_commands` と同じ |
| 13c | `run_tools` | runner が 1 回目に 1、ジョブフォルダに `photo_info.yaml` がある | 書き出しのコマンドは表示されず「写真情報ファイルは既にあります」と前処理・推定の続きのコマンドが表示される。戻り値 1 |
| 14 | `main` 通し（実行しない） | 64x48 の原本、2D CSV（カメラ `cam` の A〜F の 6 点。座標はすべて範囲 x=0〜63・y=6〜41 の内側。ほかに別カメラの行）、3D CSV（A〜G）、PLY（任意のバイト列）を用意。`read_exif` は `{"Orientation": 1, "DigitalZoomRatio": None, "Size": (64, 48)}`、`read_focal_35mm` は 23 を返すよう `monkeypatch`（No.15〜19・24・25・27・28 も同じ）。作業フォルダの親は `monkeypatch.setattr(wizard, "JOBS_DIR", tmp_path / "EIL")` で一時ディレクトリに差し替える（`main` を通すテストすべて）。答え: 原本, `job1`（作業フォルダ名）, `cam`, 2D, 3D, `y`, PLY, `n` | 戻り値 0。`tmp_path / "EIL" / "job1"` に §4.2 のウィザードが作る 7 ファイルがあり、コピー 4 つは元とバイト一致。`crop_area.png`・`points_check.png` が読める |
| 15 | `main` 通し（実行する） | No.14 と同じで最後の答えが `y`、`run_tools` を偽物に差し替え（`monkeypatch`） | 偽物が `job.yaml` のパスで 1 回呼ばれ、その戻り値（0 と 1 の 2 通り）が `main` の戻り値になる |
| 16 | 聞き直し（原本・作業フォルダ名・カメラ名） | `JOBS_DIR` に `old` フォルダを先に作る。答え: 存在しないパス, 原本, `old`（既存の名前）, `job1`, `a/b`, `cam`, …（以下 No.14 と同じ） | 戻り値 0（それぞれ聞き直して進む） |
| 17 | 聞き直し（照合） | 1 回目: 2D に 3D に無い名前 D がある CSV → 2 回目: 正しい CSV（3D は既定値のまま `""`）→ `n` → 3 回目: `""`,`""`（既定値のまま）→ `y` | 戻り値 0。標準エラーに「3D 基準点 CSV に無い名前」。2D 点 CSV の重複（同じ camera_name・ObjectName が 2 行）でも同じく聞き直す（別ケース） |
| 18 | カメラの行が無い | 2D CSV に `cam` の行が無い → 正しい CSV | 「camera_name=cam の行がありません」を表示して聞き直し、戻り値 0 |
| 19 | 中断 | 答えの途中（ジョブ作成後）で `EOFError` | 戻り値 1。ジョブフォルダは残る。「中断しました」 |
| 20 | 引数 | `main(["--foo"])` | `SystemExit` の code が 2 |
| 21 | `valid_range`・`in_range` | 4000x3000（y0=375）と 4000x2252（y0=1） | 4000x3000 は (1, 3998, 376, 2623)。両方で、x_min・x_max・y_min・y_max の点は `in_range` が True、x_min−1・x_max+1・y_min−1・y_max+1 の点（他方の座標は範囲の中央）は False。`in_range` の結果は `prepare_phone_photo.convert_point` と前処理の判定式（0 ≤ u ≤ OUT_W−1、0 ≤ v ≤ OUT_H−1）で計算した結果と同じ |
| 22 | `check_original` | 64x48 の画像で、`read_exif`・`read_focal_35mm` を差し替え: (a) Orientation 6、(b) DigitalZoomRatio 2.0、(c) 焦点距離 None、(d) 焦点距離 0、(e) Size が (32, 24)、(f) 正常、(g) 開けないファイル（テキスト） | (a)〜(e)・(g) は誤りの文言が 1 つ以上で情報は None。(c) の文言に「Exif の無い画像はウィザードでは扱えません」。(f) は誤りなしで rng=(0, 63, 6, 41) |
| 23 | `manual_commands` の引用 | job_yaml=`/tmp/camera jobs/job/job.yaml`（空白を含む） | 1・2 行目は `shlex.split` した最後の要素が job_yaml の文字列、3 行目は先頭の `TORCH_CUDA_ARCH_LIST="9.0+PTX" ` を除いて同じ |
| 24 | 範囲内の点が足りない | No.14 の 2D CSV で F の座標を y=0（範囲外）にしたもの → 正しい CSV | 「範囲外のため前処理で除かれる点: F」と「範囲内の点が 5 点です」を表示して 2 つの CSV のパスを聞き直し、戻り値 0 |
| 27 | 作業フォルダ名の聞き直し | 作業フォルダ名に `a/b`・`../x`・`a b`・`old`（既存）・`20261126-01` の順に答える（以下 No.14 と同じ） | 最初の 4 つで理由を表示して聞き直し、`JOBS_DIR / "20261126-01"` が作られる。`tmp_path / "EIL"` 以外の場所（`tmp_path` 直下、カレントディレクトリ）にフォルダが作られていない。戻り値 0 |
| 29b | 作成前に止まる | `JOBS_DIR` を `tmp_path / "a\nb" / "EIL"` に差し替え、答えは No.14 と同じ | 戻り値 1、標準エラーに「改行または前後の空白」、`tmp_path / "a\nb" / "EIL" / "job1"` が作られていない |
| 31 | CSV の文字コード | 1 回目の 2D 点 CSV を CP932 で書いた `基準_001` を含む CSV → 2 回目に UTF-8 の正しい CSV | 「UTF-8 として読めません」を表示して 2 つの CSV のパスを聞き直し、戻り値 0 |
| 32 | 実行中の中断 | `run_tools` に、2 回目の呼び出しで `KeyboardInterrupt` を送出する runner を渡す | 戻り値 1、「中断しました（prepare_phone_photo.py の実行中）」と、前処理（`--overwrite` 付き）・推定（`--out-dir <job>/out_2`）の続きのコマンドが表示される |
| 33 | YAML 前の中断 | No.19 と同じ（作業フォルダ作成後、YAML 作成前に `EOFError`） | 「新しい作業フォルダ名で最初からやり直してください」が表示される |
| 34 | 実行確認での中断 | No.14 の答えの最後（実行確認）で `EOFError` | 戻り値 1。「次のコマンドで続けられます」と `manual_commands` の 3 行が表示される。「最初からやり直してください」は表示されない |
| 29 | `job_place_error` | (a) `tmp_path / "EIL" / "j"`、(b) `tmp_path / "a\nb" / "j"`、(c) 末尾に空白のあるパス | (a) は None、(b)(c) は「改行または前後の空白」を含む文言 |
| 28 | 説明の表示 | No.14 と同じ | 標準出力で、作業フォルダ名の質問より前に「推定用の作業フォルダを」と `JOBS_DIR` の文字列、カメラ名の質問より前に「この写真のカメラ名を決めます」が出る。PLY の質問文が「3DGS の PLY ファイルのパス」 |
| 25 | 原本の聞き直し | 1 回目の原本で `read_focal_35mm` が None → 2 回目は 23（差し替えた関数が呼び出し回数で返り値を変える） | 範囲画像を書く前に原本を聞き直し、戻り値 0 |

### 6.2 実行と保存

`uv run pytest -v` の全件を実行し、出力を `tests/results/feat-045_test_result.txt` に保存する。

### 6.3 実データでの確認（実装担当がステップ6で実行。GPU を使う）

プロジェクトルートで、答えを標準入力から流してウィザードを実行する。作業フォルダ名は `feat045-check`（作業フォルダ `phase4/data/EIL/feat045-check` は存在しないこと。既存のファイルは上書き・削除しない）。以下の確認の `job/…` は `phase4/data/EIL/feat045-check/…` と読む。

```bash
printf '%s\n' \
  phase4/data/10F/s25_original/20261124_152436.jpg \
  feat045-check \
  s25_20261124_152436 \
  phase0/data/points_2d_10F.csv \
  phase0/data/kijunten_locations_10F.csv \
  y \
  phase4/data/10F/point_cloud.ply \
  y \
| TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_wizard.py
echo "exit=$?"
```

確認すること（結果は `docs/issues/feat-045-estimate-wizard/verification.md` に記録する。実行コマンド・終了コード・`cmp` の結果・画像の確認結果を含める）:

1. 終了コード 0
2. `cmp` で、`job/original.jpg`・`job/points_2d_original.csv`・`job/points_3d.csv`・`job/point_cloud.ply` がそれぞれ元のファイルと一致
3. `cmp` で、`job/images/s25_20261124_152436.png`・`job/init.toml`・`job/points_2d.csv` が `phase4/data/10F/feat044_images/s25_20261124_152436.png`・`phase4/data/10F/feat044_init.toml`・`phase4/data/10F/feat044_points_2d.csv` と一致
4. `cmp` で、`job/out/s25_20261124_152436_loop.toml`・`job/out/s25_20261124_152436_final.toml` が `phase4/data/10F/feat044_out/` の同名ファイルと一致

比較元（すべて git 管理外。2026-11-26 に gtune2〔RTX 5060 Ti〕で feat-044 の手動テストとして既存 3 ツールを手で実行した出力）の SHA-256:

| ファイル | SHA-256 |
|---|---|
| `phase4/data/10F/feat044_images/s25_20261124_152436.png` | `29ac31d24ef5a37b6e5764f5631f1c04e408fdec96abc8f773bb7d7c897ed87a` |
| `phase4/data/10F/feat044_init.toml` | `d3495780624562243e359a178e57bde621fb949c2d9c7ebc631613f15a9db4c5` |
| `phase4/data/10F/feat044_points_2d.csv` | `0bc127e8c8bc0c276b21fb605ded1e3531c18a11961a66c6ef8780f97e6a783c` |
| `phase4/data/10F/feat044_out/s25_20261124_152436_loop.toml` | `9cead24f521524f7129ffefb6b802be0811e5f90bea457a5b509c2dce89ce446` |
| `phase4/data/10F/feat044_out/s25_20261124_152436_final.toml` | `4f0769dda97a3d6219c01a50e34f524e0601115ed290739c880d7f3940c0cdf3` |

比較元が無い、またはハッシュが合わない環境では、ウィザードを実行する前に、同じ環境・同じ入力で既存 3 ツールを手で実行して比較元を作る:

1. `mkdir phase4/data/10F/feat045_base`（新規。既にあれば中止して報告）
2. 次の内容の YAML を `phase4/data/10F/feat045_base/base.yaml` に書く（`base/` 配下は 3 ツールが作る）
3. 3 ツールに `--config phase4/data/10F/feat045_base/base.yaml` で渡して順に実行する（コマンドは README の `estimate_intrinsics_loop.py ほか` の節のとおり）
4. その後のウィザードの実行は上と同じ（作業フォルダ名 `feat045-check`。作業フォルダは `phase4/data/EIL/feat045-check`）
5. 確認 3・4 の比較元は次の対応で置き換える: `feat044_images/s25_20261124_152436.png` → `feat045_base/base/images/s25_20261124_152436.png`、`feat044_init.toml` → `feat045_base/base/init.toml`、`feat044_points_2d.csv` → `feat045_base/base/points_2d.csv`、`feat044_out/{名前}` → `feat045_base/base/out/{名前}`（いずれも `phase4/data/10F/` 配下）

base.yaml の内容:

```yaml
camera: s25_20261124_152436
image: phase4/data/10F/s25_original/20261124_152436.jpg
photo_info: phase4/data/10F/feat045_base/base/photo_info.yaml
points_2d_original: phase0/data/points_2d_10F.csv
images_dir: phase4/data/10F/feat045_base/base/images
init_toml: phase4/data/10F/feat045_base/base/init.toml
points_2d: phase4/data/10F/feat045_base/base/points_2d.csv
points_3d: phase0/data/kijunten_locations_10F.csv
ply: phase4/data/10F/point_cloud.ply
out_dir: phase4/data/10F/feat045_base/base/out
```
5. `job/crop_area.png` を Read ツールで見て、y=376・y=2623 に赤線と文字があり、範囲の外（上下の帯と左右の端の各 1 列）が暗いこと。`job/points_check.png` を見て、6 点に赤い円と `001`〜`006` のラベルがあること（画像の確認結果を verification.md に文章で記録）

手順 3・4 が一致しない場合は想定外として中断し報告する。

## 7. 設計判断（ADR 簡易版）

| 判断 | 理由 |
|---|---|
| ウィザードが固定の名前でジョブフォルダ内に配置し、共用設定YAMLに絶対パスで全部書く | ファイル名・ディレクトリ名を決める手間を省くのが目的（ユーザー要求）。名前は YAML にすべて明示されるので、3 ツールが暗黙の名前を推測する仕組みは増えない。絶対パスにするのは、YAML を後でどのカレントディレクトリから使っても同じ意味になるため |
| 入力のコピー先の名前を固定にする（原本は `original{拡張子}`、PLY は `point_cloud.ply`） | 元のファイル名のままだと、ウィザードが作るファイル（`crop_area.png` 等）と名前がぶつかる場合がある |
| PLY もジョブフォルダにコピーする（約 105MB） | ユーザー要求（入力を良い感じの場所にコピーして使う）。ジョブフォルダ 1 つで入力がそろう |
| 範囲画像・確認画像を PNG にする | 劣化しない形式で、範囲内の画素が原本と同じであることを確かめられる |
| 3 ツールは子プロセスで実行し、ウィザードは再開しない | 既存ツールの動き・終了コード・出力済み検査をそのまま使える。止まったら共用設定YAMLで手で続けられる |
| 原本の判定（Exif 検査・35mm 換算焦点距離の有無・打てる範囲）を点打ちの前に行い、既存ツールの関数をそのまま使う | 人が GIMP で点を打った後に書き出し・前処理で拒否されると、点打ちの作業が無駄になる（codex-01）。判定を既存の関数に任せることで、ツールとウィザードの判定が食い違わない |
| Exif の無い画像はウィザードの対象外 | ウィザードは書き出しで写真情報ファイルを作る前提。Exif の無い画像は feat-044 の手順（写真情報ファイルを手で書く）で手動実行できる。ウィザードに手書きの経路を足すのは要求外 |
| カメラ名を英数字・`_`・`-` に限る | 前処理は初期 TOML に `[カメラ名]` と書くため、空白・日本語・`.` を含む名前は TOML として読めない、または別の名前になる（codex-01） |
| 2D 点を打つ作業はウィザードの外 | ユーザー判断（GUI は作業量が多すぎる。今の GIMP のまま） |
| CSV の読み込みに `refine_extrinsics.py` の関数を使う | 重複・必須列・非有限値の検査が既にあり、推定と同じ規則で検査できる |

## 8. 完了時の更新対象（ステップ8で Claude Code 本体が行う）

- `README.md`: phase4 のスクリプト一覧の表に `estimate_wizard.py` の行を足す（`extract_photo_info.py` の行の前）。`estimate_intrinsics_loop.py ほか` の節の中に、ウィザードの使い方（起動コマンド、対話の流れ、ジョブフォルダの構成、止まったときの続け方）を書く
- `CLAUDE.md`: ディレクトリ構成に `estimate_wizard.py` の行を足す（`extract_photo_info.py` の前）
- `docs/BACKLOG.md`（Closed）、`docs/CHANGELOG.md`、本案件 README のステータス

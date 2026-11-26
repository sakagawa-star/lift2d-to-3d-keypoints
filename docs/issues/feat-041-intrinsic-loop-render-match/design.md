# feat-041 機能設計書: レンダ＋マッチング反復による内部パラメータ（K・歪み）と外部パラメータの同時推定

本書のコード例は意図を伝えるためのもので、そのままコピーして使うものではない。用語は requirements.md §2 に従う。

## 0. 対応要求マッピング

| 要求 ID | 設計セクション |
|---|---|
| FR-001 | §4.1 |
| FR-002 | §4.2.2 |
| FR-003 | §4.2.3 |
| FR-004 | §4.2.4 |
| FR-005 | §4.2.5 |
| FR-006 | §4.2.6 |
| FR-007 | §4.2.7、§7 |
| FR-008 | §4.3 |
| NFR-001 | §4.2.7（所要時間の記録） |
| NFR-002 | §2 |
| NFR-003 | §4.2.3（乱数 seed） |
| NFR-004 | §4.1.3、§4.2.1（上書き確認） |
| NFR-005 | §4.2.3（既存 CLI をそのまま使う） |

## 1. システム構成

### 1.1 モジュール構成

| ファイル | 種別 | 担当 |
|---|---|---|
| `phase4/prepare_phone_photo.py` | 新規 | FR-001。写真の前処理、Exif 検査、初期 TOML 作成、2D 点の座標変換 |
| `phase4/estimate_intrinsics_loop.py` | 新規 | FR-002〜FR-007。ループ本体、`refine_extrinsics.py` の呼び出し、評価の呼び出し、レポート |
| `phase4/evaluate_overlay.py` | 新規 | FR-008。重ね合わせ評価（関数 + 単体 CLI） |
| `tests/test_feat041_intrinsics_loop.py` | 新規 | 単体テスト（GPU・実データ不要の部分） |
| `phase4/refine_extrinsics.py` | 既存・変更しない | 関数・定数を import して使う。最終精緻化は subprocess で呼ぶ |
| `phase4/render_keypoints.py` | 既存・変更しない | `load_cameras_toml`, `select_camera`, `render_image` を import |
| `phase4/render.py` | 既存・変更しない | `load_ply` を関数内で遅延 import |
| `matcher_lab/mast3r_cli.py` | 既存・変更しない | 反復 i = 0 のマッチ。subprocess で呼ぶ |
| `matcher_lab/loftr_cli.py` | 既存・変更しない | 反復 i ≥ 1 のマッチ。`refine_extrinsics.call_matcher` 経由の subprocess で呼ぶ |

### 1.2 依存関係（呼び出し方向。循環なし）

```
prepare_phone_photo.py ──import──> refine_extrinsics.load_points_2d_csv
estimate_intrinsics_loop.py ──import──> refine_extrinsics（関数・定数）
                            ──import──> render_keypoints（load_cameras_toml, select_camera, render_image）
                            ──import──> evaluate_overlay（evaluate_and_write）
                            ──遅延import──> render.load_ply
                            ──subprocess──> matcher_lab/mast3r_cli.py（matcher_lab 環境。i = 0）
                            ──import──> refine_extrinsics.call_matcher ──subprocess──> matcher_lab/loftr_cli.py（matcher_lab 環境。i ≥ 1）
                            ──subprocess──> phase4/refine_extrinsics.py（phase4 環境、sys.executable）
evaluate_overlay.py ──import──> refine_extrinsics（render_depth_alpha_distorted, NEAR_PLANE, GATE_ALPHA, GATE_Z_RANGE）
                    ──import──> render_keypoints（load_cameras_toml, select_camera, render_image）
                    ──遅延import──> render.load_ply（CLI 実行時のみ）
```

`refine_extrinsics.py` は `render_keypoints` を top-level で import するが、torch / gsplat は関数内で遅延 import している。したがって、上記 3 本も torch / gsplat / Pillow を関数内でだけ import し、ルート環境の pytest から import できるようにする。

### 1.3 ディレクトリ構成（実データ検証時の例）

```
phase4/data/10F/
├── point_cloud.ply
├── s25_original/20261124_152436.jpg        # 原本（入力。変更しない）
├── s25_images/s25_20261124_152436.png      # FR-001 出力（前処理後画像）
├── s25_20261124_152436_init.toml           # FR-001 出力（初期 TOML）
├── points_2d_s25_20261124_152436.csv       # FR-001 出力（前処理後座標の 2D 点）
└── feat041_out/                            # estimate_intrinsics_loop.py の --out-dir
    ├── s25_20261124_152436_loop.toml
    ├── s25_20261124_152436_final.toml
    ├── s25_20261124_152436_loop_report.txt
    ├── s25_20261124_152436_refine_report.txt
    ├── s25_20261124_152436_eval.txt
    ├── s25_20261124_152436_eval_blend.png
    ├── s25_20261124_152436_eval_checker.png
    └── s25_20261124_152436_eval_flowmag.png
```

## 2. 技術スタック

- Python 3.10、phase4 の uv 環境（`uv run --project phase4`）。実行時は `TORCH_CUDA_ARCH_LIST="9.0+PTX"` を設定する（CLAUDE.md 記載の既知制約）
- 使用ライブラリ（すべて既存の依存。新規追加なし。`docs/TECH_STACK.md` の更新は不要）: numpy、opencv-python 4.13（`cv2.DISOpticalFlow` を含む）、scipy 1.15（`scipy.optimize.least_squares`）、Pillow 12（Exif 読み取り）、tomli、torch / gsplat（レンダ。既存関数経由）
- MASt3R は matcher_lab 環境の既存 CLI `matcher_lab/mast3r_cli.py` を subprocess で使う。LoFTR は同じ環境の既存 CLI `matcher_lab/loftr_cli.py` を `refine_extrinsics.call_matcher` 経由で使う

## 3. 定数

### 3.1 `estimate_intrinsics_loop.py`

| 定数 | 値 | 意味 |
|---|---|---|
| `IMG_W`, `IMG_H` | 1920, 1080 | 前処理後画像のサイズ |
| `TAU_PX` | `{0: 250.0, 1: 30.0}` | ゲートの変位しきい値[px]。キーは反復番号 i（i ≥ 1 はすべてキー 1 の値） |
| `RANSAC_PX` | `{0: 8.0, 1: 3.0}` | 外れ値除去 solvePnPRansac の再投影しきい値[px]（i ≥ 1 はキー 1 の値） |
| `PNP_RANSAC_ITER` | 1000 | solvePnPRansac の反復回数 |
| `N_MIN_INTR` | 300 | 外れ値除去後に必要な最小対応点数 |
| `LSQ_LOSS` | `"soft_l1"` | 同時最小二乗の損失関数 |
| `LSQ_F_SCALE` | 1.0 | 損失関数のスケール[px] |
| `LSQ_MAX_NFEV` | 200 | 同時最小二乗の最大関数評価回数 |
| `CONV_PX` | 2.0 | 収束しきい値。i ≥ `LOFTR_FROM_ITER` の反復で Δ_intr < 2.0px なら収束（イテレーション2 で 0.5 → 2.0。10F の 1 回の実行で観測した LoFTR 反復の Δ_intr の揺れ（i ≥ 4 で 1.040〜1.872px）の上限 1.872px を上回る値として選んだ） |
| `MAX_ITER` | 8 | 反復回数の上限（i = 0〜7） |
| `F_RANGE` | (0.3×1920, 3.0×1920) = (576.0, 5760.0) | f の許容範囲[px] |
| `CX_TOL`, `CY_TOL` | 192.0, 108.0 | 主点の画像中心からの許容ずれ[px]（画像幅・高さの 10%） |
| `K_ABS_MAX` | 1.0 | \|k1\|, \|k2\| の上限 |
| `GRID_CELLS` | (4, 4) | 格子被覆の分割数（横 4 × 縦 4） |
| `CELL_MIN_PTS` | 10 | 1セルを「被覆あり」とする最小点数 |
| `COVER_WARN` | 12 | 被覆ありセルがこれ未満なら警告（処理は続ける） |
| `SEED_BASE` | 7000 | 反復 i の solvePnPRansac 前に `cv2.setRNGSeed(SEED_BASE + i)` |
| `DELTA_GRID_N` | 5 | Δ_intr の格子点数（5x5） |
| `UNDIST_CRITERIA` | `(cv2.TERM_CRITERIA_COUNT \| cv2.TERM_CRITERIA_EPS, 100, 1e-10)` | 深度リフトの逆歪み `cv2.undistortPointsIter` の終了条件（codex-03 指摘2: 既定の `cv2.undistortPoints` は k1=-0.3 で約 4.75px の誤差が残る） |
| `LIFT_ROUNDTRIP_PX` | 0.01 | 深度リフトの逆歪み検査のしきい値[px] |
| `REFINE_UNDIST_GRID` | (33, 19) | 最終精緻化前の逆歪み精度検査の格子点数（横 33 × 縦 19） |
| `REFINE_UNDIST_MAX_PX` | 0.1 | 最終精緻化前の逆歪み精度検査のしきい値[px] |
| `LOFTR_FROM_ITER` | 1 | この反復番号以降は LoFTR でマッチを求める（それより前は MASt3R）。investigation.md イテレーション1 |

ゲートの定数は `refine_extrinsics.py` のものを import して使う: `NEAR_PLANE`(0.5)、`GATE_ALPHA`(0.5)、`GATE_Z_RANGE`((0.5, 10.0))、`GATE_VAR_WINDOW`(5)、`GATE_VAR_REL`(0.02)。LoFTR のマッチの選別には `LOFTR_CONF_TH`(0.2) を import して使う。

### 3.2 `evaluate_overlay.py`

| 定数 | 値 | 意味 |
|---|---|---|
| `CLAHE_CLIP`, `CLAHE_TILE` | 2.0, (8, 8) | グレースケール画像のコントラスト正規化 |
| `DIS_PRESET` | `cv2.DISOPTICAL_FLOW_PRESET_MEDIUM` | DIS のプリセット |
| `FB_CONSIST_PX` | 1.0 | 順逆フロー整合のしきい値[px] |
| `TEXTURE_GRAD_MIN` | 20.0 | 写真のテクスチャ判定（CLAHE 後グレー画像の Sobel 勾配の大きさ）の下限 |
| `VALID_RATIO_MIN` | 0.10 | 集計対象画素の割合がこれ未満なら「判定不能」 |
| `PASS_MEDIAN_PX` | 1.0 | 合格ライン（中央値）。95 パーセンタイル値は記録のみで判定に使わない（イテレーション2 で `PASS_P95_PX` を削除） |
| `CHECKER_TILE` | 120 | 市松模様のタイルの一辺[px] |
| `FLOWMAG_CLIP_PX` | 5.0 | ヒートマップの上限[px] |

### 3.3 `prepare_phone_photo.py`

| 定数 | 値 | 意味 |
|---|---|---|
| `OUT_W`, `OUT_H` | 1920, 1080 | 出力サイズ |
| `DIAG_35MM` | 43.27 | 35mm 判の対角長[mm] |
| `FOCAL_MM_TOL` | 0.05 | 期待実焦点距離との許容差[mm] |
| `MIN_POINTS` | 6 | 変換後に必要な一意 ObjectName 数 |

## 4. 各機能の詳細設計

### 4.1 FR-001 `prepare_phone_photo.py`

#### 4.1.1 CLI

> **注記（feat-044、2026-11-26）**: `--expect-model`・`--expect-focal-mm` は feat-044 で削除され（指定すると終了コード 2）、写真情報ファイルを指定する必須の `--photo-info` が追加された。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

> **注記（feat-043、2026-11-26）**: 本節のオプション名は feat-043 で変更された。`--out-toml` → `--init-toml`、`--points-2d-out` → `--points-2d`、`--points-2d-in` → `--points-2d-original`（旧名は使えない）。`--config` が追加され、オプションの省略形は受け付けなくなった（`allow_abbrev=False`）。`--camera` はパス区切りを含む名前を拒否する。現在の仕様は `docs/issues/feat-043-shared-job-yaml-phone-photo/` を参照。本節は feat-041 時点の記録として残す。

```
uv run --project phase4 python phase4/prepare_phone_photo.py \
    --image <原本JPEG> --camera <カメラ名> \
    --images-dir <前処理後画像の出力ディレクトリ> --out-toml <初期TOML> \
    --points-2d-in <原本座標の2D点CSV> --points-2d-out <前処理後座標の2D点CSV> \
    [--expect-model "Galaxy S25"] [--expect-focal-mm 5.4] [--overwrite]
```

すべて必須（`--expect-model`, `--expect-focal-mm`, `--overwrite` を除く）。終了コード: 0=成功、1=入力エラー・検査不合格・出力済み、2=引数エラー（argparse）。

#### 4.1.2 データ

- Exif（Pillow の `Image.open(path).getexif()` と `get_ifd(0x8769)` を統合した辞書）から読む項目: `Model`（str）、`Orientation`（int、無ければ None）、`FocalLength`（float、無ければ None）、`FocalLengthIn35mmFilm`（int、無ければ None）、`DigitalZoomRatio`（float、無ければ None）。画像サイズ (W, H) は `Image.size`
- 2D 点 CSV は `refine_extrinsics.load_points_2d_csv` で読む（重複・欠損列・非有限値はその関数の例外 `InputError` で終了コード 1）

#### 4.1.3 処理手順

> **注記（feat-044、2026-11-26）**: feat-044 で検査項目と初期焦点距離の入力元が変わった。35mm 換算焦点距離は Exif ではなく写真情報ファイル（`--photo-info`）から読み、Exif の `FocalLengthIn35mmFilm`・`Model`・`FocalLength` は参照しない。`Orientation`・`DigitalZoomRatio` の検査は Exif に値がある場合だけ行う。写真情報ファイルが前処理の出力と同じファイルを指す場合は処理開始前に終了コード 1。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

1. `--images-dir/{camera}.png`、`--out-toml`、`--points-2d-out` のいずれかが存在し `--overwrite` が無ければ、エラー（終了コード 1）
2. Exif を読み、`validate_exif` で検査する。エラーが1件以上あれば全件を表示して終了コード 1。検査項目（すべて独立に評価して列挙する）:
   - `Orientation` が None でも 1 でもない
   - W ≤ H
   - `FocalLengthIn35mmFilm` が None または 0 以下
   - `DigitalZoomRatio` が None でも 1.0 でもない
   - `--expect-model` 指定時、`Model`（前後の空白を除く）が一致しない
   - `--expect-focal-mm` 指定時、`FocalLength` が None、または差の絶対値が `FOCAL_MM_TOL` を超える
3. 切り抜き範囲を `compute_crop(W, H)` で求める: `W * 9 % 16 != 0` ならエラー、`Hc = W * 9 // 16`、`H < Hc` ならエラー、`y0 = (H - Hc) // 2`。戻り値 `(y0, Hc)`
4. 画像を `cv2.imread(path, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)` で読み（None ならエラー）、`img[y0:y0+Hc, :]` を `cv2.resize(..., (1920, 1080), interpolation=cv2.INTER_AREA)` で縮小する
5. 初期焦点距離 `f = f35 * hypot(W, H) / 43.27 * (1920 / W)`（`initial_focal_px`）
6. 2D 点の変換（`convert_point`）。縮小率 `s = 1920 / W`。画素中心規約:
   - `x = (X + 0.5) * s - 0.5`
   - `y = (Y - y0 + 0.5) * s - 0.5`
   - `0 <= x <= 1919` かつ `0 <= y <= 1079` の点だけ残す。それ以外は除外して ObjectName を表示する
   - 対象は camera_name が `--camera` の行だけ。残った一意 ObjectName が 6 未満ならエラー（終了コード 1）
7. 出力（いずれも一時ファイル `{出力パス}.tmp` に書いてから `Path.replace` で出力パスに置き換える）:
   - `{images-dir}/{camera}.png`（ディレクトリが無ければ作る）。`cv2.imwrite` は拡張子で画像形式を決めるため、末尾が `.tmp` のパスには書けない。そこで `ok, buf = cv2.imencode(".png", img)` で PNG のバイト列を作り（`ok` が False ならエラー、終了コード 1）、`buf.tobytes()` を `{camera}.png.tmp` にバイナリで書いてから置き換える
   - 初期 TOML（下記書式）
   - 2D 点 CSV（ヘッダ `ObjectName,camera_name,X,Y`、座標は小数 3 桁に丸める、行順は入力 CSV の出現順）

初期 TOML の書式（`refine_extrinsics._format_toml_value` と同じ文字列化。数値は `str(float)`）:

```
[{camera}]
name = "{camera}"
size = [1920.0, 1080.0]
matrix = [[{f}, 0.0, 960.0], [0.0, {f}, 540.0], [0.0, 0.0, 1.0]]
distortions = [0.0, 0.0, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 0.0]
fisheye = false
```

主点 (960.0, 540.0) は `estimate_camera_params.py --fix-center` の `W/2, H/2` と同じ規約である。

#### 4.1.4 境界条件

- 原本がすでに 1920x1080 の場合: `Hc = 1080`、`y0 = 0`、縮小率 1.0 でそのまま出力する
- 原本が 16:9 より横長（H < Hc）: エラー
- 2D 点 CSV に当該カメラの行が 0 件: 「一意 ObjectName が 6 未満」としてエラー

### 4.2 FR-002〜FR-007 `estimate_intrinsics_loop.py`

#### 4.2.1 CLI と全体の流れ

> **注記（feat-043、2026-11-26）**: 本節の `--toml` は feat-043 で `--init-toml`（設定キー `init_toml`）に変更された（旧名は使えない）。オプションの省略形は受け付けなくなった（`allow_abbrev=False`）。最終精緻化で `refine_extrinsics.py` に渡す引数（`refine_extrinsics.py` 自身の `--toml`）は変わらない。現在の仕様は `docs/issues/feat-043-shared-job-yaml-phone-photo/` を参照。

```
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py \
    --toml <初期TOML> --camera <カメラ名> --ply <PLY> --images-dir <前処理後画像ディレクトリ> \
    --points-3d <3D基準点CSV> --points-2d <前処理後座標の2D点CSV> --out-dir <出力ディレクトリ（存在しない新しいパス）> \
    [--tmp-dir <中間ファイル置き場（存在しない新しいパス）>]
```

終了コード: 0=成功（最終精緻化で受理され、評価を実行した。評価の合否は問わない）、1=入力エラー・環境異常・出力済み、2=引数エラー、3=ループ失敗（点数不足、RANSAC 失敗、MASt3R の OOM が再試行後も発生、LoFTR の OOM、未収束、範囲外）、4=最終精緻化で不受理。

```
main:
  out = §1.3 の 8 ファイルのパス（--out-dir 配下）。内訳: {camera}_loop.toml、{camera}_final.toml、
        {camera}_loop_report.txt、{camera}_refine_report.txt と、
        evaluate_overlay.eval_output_paths(str(out_dir / f"{camera}_eval")) が返す 4 パス
  カメラ名の制限（bug-006 codex-04）: camera が "" / "." / ".." のいずれか、または Path(camera).name != camera、または "/" か "\\" を含む
      → エラー(1)。出力ファイル名にそのまま連結されるので、パス区切りを含む名前で --out-dir の外へ書かないため
  --out-dir を mkdir(parents=True, exist_ok=False) で新しく作る（bug-006）。
      FileExistsError → エラー(1)「出力ディレクトリが既に存在します（存在しない新しいパスを指定してください）」
      その他の OSError → エラー(1)。存在の判定は OS が作成時に行うので、パスの書き方（.. を含む等）や途中のシンボリックリンクの
      解決によらない。リンク先の無いシンボリックリンクも「存在する」として拒否される
      新しく作ったディレクトリの中身はこのプロセスとその子プロセスが書いたファイルだけになるので、出力 8 ファイルの個別検査は不要
      （最終精緻化の一時ファイル {camera}_final.toml.tmp も、前から置かれていたリンクや入力に当たらない）
      --overwrite オプションは持たない（イテレーション2。今回の実行が途中で終わったときに前回の _final.toml や評価ファイルが
      残って今回の結果と混在するのを防ぐため。再実行は新しい --out-dir を指定するか、利用者が前回のディレクトリを手で削除する）
      同じ --out-dir・カメラ名で実行する場合は、前の実行が終了してから起動する（同時実行の排他は行わない。requirements.md §5）
  --tmp-dir 指定時: mkdir(parents=True, exist_ok=False) で新しく作る（bug-006）。
      FileExistsError → 直前に作った空の --out-dir を rmdir で消してからエラー(1)「中間ファイル置き場が既に存在します（存在しない新しいパスを指定してください）」
      その他の OSError → 同じく --out-dir を rmdir で消してからエラー(1)
      後始末の rmdir が OSError で失敗しても（--tmp-dir の作成が親を途中まで作ってから失敗した場合など。例: --tmp-dir {out-dir}/sub/..）、
      元のエラーを表示して終了コード 1。「作成したディレクトリが残っています（手で削除してください）: {out_dir}」も表示する（codex-04）
  --tmp-dir 省略時: tmp_dir = TemporaryDirectory（正常終了時に削除）
  上記で拒否した場合は、レポートを含め何も書かない（作った --out-dir は消す）
  cams = load_cameras_toml(--toml); cam0 = select_camera(cams, --camera)   # 無ければエラー(1)
  p3d = load_points_3d_csv; p2d = load_points_2d_csv
  v = validate_camera(camera, cam0, --images-dir, p3d, p2d)   # refine_extrinsics の関数
  v["ok"] が False → エラー(1)（理由を表示）
  gaussians = load_ply(--ply)                                  # 失敗はエラー(1)
  pose0 = FR-002（§4.2.2）
  params = Params(f=cam0.K[0,0], cx=cam0.K[0,2], cy=cam0.K[1,2], k1=cam0.D[0], k2=cam0.D[1], rvec, tvec=pose0)
  for i in 0..MAX_ITER-1:
      it = run_loop_iteration(gaussians, params, v["image_path"], i, tmp_dir)   # §4.2.3, §4.2.4
      new = it["params"]
      delta = delta_intr(params, new)                                           # §4.2.5
      レポートに1行（§7）
      range_errors = check_ranges(new)
      if range_errors: レポートに記録して終了(3)
      params = new
      if is_converged(i, delta): converged = True; break      # i >= LOFTR_FROM_ITER and delta < CONV_PX
  if not converged:
      ループ結果 TOML を最後の反復の値（params）で書く（§4.2.6 と同じ書式・同じパス）
      レポートに「未収束（ループ結果 TOML は最後の反復の値）」と記録して終了(3)
      （反復途中の失敗・範囲外のときは TOML を書かない。各場面の扱いは下の「終了状態とループ結果 TOML の有無」の表）
  ループ結果 TOML を書く（§4.2.6）
  e = refine_undistort_error_px(K, D)（§4.2.6）。e > 0.1 → 終了(3)
  最終精緻化（§4.2.6）。不受理 → 終了(4)
  評価（§4.3 の evaluate_and_write を、最終 TOML・同じ gaussians で呼ぶ）
  レポートに評価結果を記録して終了(0)
```

**終了状態とループ結果 TOML の有無**（イテレーション2、codex-23 指摘3）。表の「書く」は書き込みに成功した場合。ループ結果 TOML・評価ファイル・レポートなどの書き込み自体が失敗した場合（ディスク容量不足など）は特別な扱いをせず（§4.2.7 の例外規定）、Python の例外で異常終了する（終了コード 1。初版の収束時と同じ。investigation.md イテレーション2「今回の修正に含めないこと」）:

| 終了の場面 | 終了コード | 終了状態（レポート） | `{camera}_loop.toml` |
|---|---|---|---|
| 反復途中の失敗（`LoopFailure`: 点数不足、RANSAC 失敗、OOM、非有限、OpenCV 例外） | 3 | ループ失敗（理由） | 書かない |
| 範囲チェックの違反 | 3 | 範囲外（違反内容） | 書かない |
| 上限到達による未収束 | 3 | 未収束（ループ結果 TOML は最後の反復の値） | 書く（最後の反復の値） |
| 収束後の逆歪み精度検査の不合格（§4.2.6） | 3 | ループ失敗（最終精緻化の前提違反: 逆歪み誤差 {e}px） | 書く（収束した値。TOML を書いた後に検査するため。初版から同じ） |
| 最終精緻化の不受理 | 4 | 精緻化不受理 | 書く |
| `refine_extrinsics.py` の異常終了（終了コード 0 以外） | 1 | refine_extrinsics.py が異常終了（終了コード {n}） | 書く |
| 出力済み検査の後の入力エラー・環境異常（`InputError`、`MatcherEnvError`。TOML・CSV・PLY の読み込み失敗、手動点 PnP 失敗、マッチャー CLI の異常終了など） | 1 | エラー（理由） | 反復の前なら書かない。反復の途中（マッチャー CLI の異常終了）でも書かない |
| 成功 | 0 | 成功 | 書く |

`run_loop_iteration` が失敗（点数不足、RANSAC 失敗、MASt3R OOM 2 回、LoFTR OOM、最小二乗の結果が非有限）を返したら、レポートに理由を記録して終了コード 3 で終わる。`MatcherEnvError`（`check_matcher_meta` の違反、MASt3R・LoFTR の 42 以外の非 0 終了）は終了コード 1。

#### 4.2.2 FR-002 初期外部パラメータ

`refine_extrinsics.pnp_manual(v["p3"], v["p2"], cam0["K"], cam0["D"])` を呼ぶ。None なら終了コード 1。戻り値の rvec, tvec を初期値にする。レポートに手動点数・inlier 数・`median_reproj_px` の値を記録する。

#### 4.2.3 FR-003 反復の1回分

```
run_loop_iteration(gaussians, params, query_png, i, tmp_dir) -> dict:
  cam = params.to_cam()      # {"K","D"=[k1,k2,0,0],"rvec","tvec","width":1920,"height":1080}
  bgr = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
  depth, alpha = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)
  render_png = tmp_dir / f"iter{i:02d}_render.png"; cv2.imwrite
  out_npz = tmp_dir / f"iter{i:02d}_matches.npz"
  if i < LOFTR_FROM_ITER:
      matcher_name = "MASt3R"
      u_q, u_r, meta = call_mast3r(query_png, render_png, out_npz)
  else:
      matcher_name = "LoFTR"
      try:
          u_q, u_r, conf, meta = call_matcher(query_png, render_png, out_npz)   # refine_extrinsics の関数
      except MatcherOOM:
          → 失敗「LoFTR OOM」
      sel = conf >= LOFTR_CONF_TH
      u_q, u_r = u_q[sel], u_r[sel]
  check_matcher_meta(meta)                 # refine_extrinsics の関数（範囲内率 1.0、round-trip ≤ 0.5px）。両マッチャー共通
  n_match = len(u_q)                       # LoFTR は信頼度で選んだ後の件数（refine_extrinsics.py の「選別後」と同じ意味）
  マッチ 0 件 → 失敗「マッチ0件」
  pixel_valid = isfinite(depth) & (alpha > GATE_ALPHA) & (depth > 0.5) & (depth < 10.0)
  var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)
  tau_d = GATE_VAR_REL * median(depth[pixel_valid])
  bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, TAU_PX[min(i,1)])
  keep = bd["keep"]; d = bd["d_bilin"]
  bd["N"] < N_MIN_INTR → 失敗「ゲート通過 N={N} < 300」（深度リフト・solvePnPRansac に進まない）
  ゲート通過点を同じ順序の配列に切り出す（以降はこの 3 配列だけを使い、keep は使わない）:
      idx = np.flatnonzero(keep)
      ur = u_r[idx]; uq = u_q[idx]; dk = d[idx]          # いずれも点数 n0 = len(idx)、同じ順序
  深度リフト（lift_to_world。refine_extrinsics.run_iteration の Stage 5 と同じ式で、逆歪みだけ高精度化）:
      u_pin = cv2.undistortPointsIter(ur, K, D, None, K, UNDIST_CRITERIA)        # 点数 n0
      n = inv(K) @ [u_pin, 1]                                                     # 点数 n0
      p = cv2.projectPoints(n, rvec=0, tvec=0, K, D)                              # 点数 n0
      ok_undist = |p - ur| <= LIFT_ROUNDTRIP_PX                                   # bool, 点数 n0
      X_cam = n * dk                                                              # 点数 n0（n は z=1 の光線）
      X_w_all = (X_cam - tvec) @ R          # R = Rodrigues(rvec)。点数 n0
      return X_w_all, ok_undist
  検査合格点だけを残す（3 配列に同じマスクを適用して、点数と順序をそろえる）:
      X_w = X_w_all[ok_undist]; uq = uq[ok_undist]                             # 点数 n1 = ok_undist.sum()
      diag["excl_undist"] = n0 - n1
      n1 < N_MIN_INTR → 失敗「逆歪み検査後 N={n1} < 300」
  cv2.setRNGSeed(SEED_BASE + i)
  ok, rv, tv, inl = cv2.solvePnPRansac(X_w, uq, K, D,
        iterationsCount=PNP_RANSAC_ITER, reprojectionError=RANSAC_PX[min(i,1)],
        flags=cv2.SOLVEPNP_ITERATIVE)
  not ok or inl is None → 失敗「solvePnPRansac 失敗」
  inl = inl.flatten()        # solvePnPRansac の inl は shape (N,1)。1 次元にしないと uq[inl] が (N,1,2) になる
  len(inl) < N_MIN_INTR → 失敗「外れ値除去後 {n} < 300」
  X = X_w[inl].reshape(-1, 3); u = uq[inl].reshape(-1, 2)     # X: (N,3) float64、u: (N,2) float64
  new = joint_refine(X, u, params の f,cx,cy,k1,k2 と rv,tv)      # §4.2.4
  rmse = sqrt(mean(|project(new, X) - u|^2))
  cover = 被覆ありセル数（§4.2.7）
  return {"ok": True, "params": new, "diag": {"matcher": matcher_name, ...}}
```

**OpenCV 例外の扱い**: `run_loop_iteration` の中の OpenCV 呼び出し（`cv2.undistortPointsIter`、`cv2.solvePnPRansac`、`joint_refine` 内の `cv2.projectPoints`）で `cv2.error` が発生した場合は、`LoopFailure(f"OpenCV 例外: {呼び出し名}: {例外メッセージの先頭 200 文字}")` に変換する（終了コード 3、レポートに理由を記録）。

`call_mast3r` の仕様:

```
cmd = ["uv","run","--project", MATCHER_LAB_DIR, "python", MATCHER_LAB_DIR/"mast3r_cli.py",
       query_png, render_png, "--out", out_npz, "--resolution", res]
res="full" で実行 → 終了コード 42 なら res="1280x720" で1回だけ再実行
2 回目も 42 → 失敗「MASt3R OOM」（終了コード 3）
0 と 42 以外 → MatcherEnvError（終了コード 1、stderr の末尾 2000 文字を表示）
NPZ の u_a を u_q（写真側）、u_b を u_r（レンダ側）として float64 で返す。meta は json.loads(str(npz["meta"]))
```

画像 A に写真、画像 B にレンダを渡す（feat-035 の CLI 仕様: u_a は画像 A の座標）。

`call_matcher`（`refine_extrinsics.py` の既存関数。i ≥ 1 で使う）の仕様: `loftr_cli.py` に写真（query）とレンダを渡し、戻り値は `(u_q, u_r, conf, meta)`。終了コード 42 は `MatcherOOM`、それ以外の非 0 終了は `MatcherEnvError` を送出する。`MatcherOOM` は再試行せず `LoopFailure("LoFTR OOM")` に変換する（`loftr_cli.py` に解像度を変えるオプションが無いため）。`MatcherEnvError` は MASt3R の場合と同じく終了コード 1。

#### 4.2.4 FR-004 同時最小二乗

変数ベクトル `x = [f, cx, cy, k1, k2, r1, r2, r3, t1, t2, t3]`（11 個）。入力は `X`: shape (N,3) float64（3DGS 座標系 [m]）、`u`: shape (N,2) float64（前処理後画像の画素座標）。`joint_refine` の冒頭で `X.shape == (N, 3)` かつ `u.shape == (N, 2)` を `assert` で確かめる。残差ベクトルは 2N 要素になる。

```
def residuals(x, X, u):
    f, cx, cy, k1, k2 = x[0:5]
    K = [[f,0,cx],[0,f,cy],[0,0,1]]; D = [k1,k2,0,0]
    proj, _ = cv2.projectPoints(X, x[5:8], x[8:11], K, D)
    return (proj.reshape(-1,2) - u).ravel()

res = scipy.optimize.least_squares(residuals, x0, args=(X, u), method="trf",
          loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=200)
```

- x0 は f, cx, cy, k1, k2 が現在の値、r, t が §4.2.3 の solvePnPRansac の戻り値（rv, tv）
- `res.x` に非有限値があれば失敗「最小二乗の結果が非有限」。`res.success` が False でも `res.x` を採用し、レポートに `res.status` を記録する

#### 4.2.5 FR-005 収束判定と範囲チェック

**Δ_intr の定義**（`delta_intr(old, new) -> float`）:

1. 格子点 `g`: x 方向 `linspace(0, 1919, 5)`、y 方向 `linspace(0, 1079, 5)` の 25 点
2. 旧パラメータの焦点距離・主点だけで正規化座標を作る（歪みの逆変換はしない）: `n = ((g_x - cx_old) / f_old, (g_y - cy_old) / f_old)`
3. 同じ `n` を旧・新の両パラメータで投影する: `p_old = cv2.projectPoints([n_x, n_y, 1], rvec=0, tvec=0, K_old, D_old)`、`p_new = cv2.projectPoints([n_x, n_y, 1], rvec=0, tvec=0, K_new, D_new)`
4. `Δ_intr = max |p_new - p_old|`（25 点のユークリッド距離の最大値）

同じ正規化座標を両方で投影して差を取るので、旧と新が同じパラメータなら歪みの有無によらず Δ_intr は 0 になる（`cv2.undistortPoints` の反復解法の誤差が入らない。codex-01 指摘3: 逆変換を使う旧定義では、k1=0.2, k2=0.1 の同一パラメータで Δ_intr ≈ 0.62px になり、収束しきい値 0.5px を満たせなかった）。外部パラメータは Δ_intr に含めない。

**範囲チェック**（`check_ranges(params) -> list[str]`）: 次の違反をすべて列挙する。空なら合格。

- `f < 576.0` または `f > 5760.0`
- `|cx - 960| > 192.0`
- `|cy - 540| > 108.0`
- `|k1| > 1.0`、`|k2| > 1.0`

**終了条件**: 各反復の後に範囲チェック → 違反があれば終了コード 3。違反が無く、`i >= LOFTR_FROM_ITER`（= 1）かつ `Δ_intr < 2.0`（`CONV_PX`）なら収束（`is_converged(i, delta) -> bool`）。i = 0 は MASt3R のマッチで、座標が 3.75px 刻みのため、Δ_intr が 0 でも収束とせずに i = 1（LoFTR）に進む（codex-09 指摘1。investigation.md イテレーション1）。i = 7 の反復を終えても収束しなければ終了コード 3（最後の反復の値でループ結果 TOML を書く。§4.2.1）。

#### 4.2.6 FR-006 ループ結果 TOML と最終精緻化

**最終精緻化前の逆歪み精度検査**（`refine_undistort_error_px(K, D) -> float`）: `refine_extrinsics.py` は深度リフトに既定の `cv2.undistortPoints(pts, K, D, P=K)` を使い、本案件では変更できない。歪みが大きいとこの逆変換に誤差が残るため、呼び出す前に収束後の K, D で次の値を計算する。格子点 `g` = x 方向 `linspace(0, 1919, 33)` × y 方向 `linspace(0, 1079, 19)` の 627 点、`u = cv2.undistortPoints(g, K, D, P=K)`、`n = inv(K) @ [u, 1]`、`p = cv2.projectPoints(n, rvec=0, tvec=0, K, D)`、誤差 = `max |p - g|`。誤差が `REFINE_UNDIST_MAX_PX`（0.1px）を超えたら、レポートに値を記録して終了コード 3（「最終精緻化の前提違反: 逆歪み誤差 {e}px」）。超えなければ値をレポートに記録して進む。0.1px は `refine_extrinsics.py` の外れ値しきい値（2px）に比べて無視できる大きさとして決めた。

ループ結果 TOML（`{camera}_loop.toml`）は §4.1.3 と同じ書式で、`matrix = [[f,0,cx],[0,f,cy],[0,0,1]]`、`distortions = [k1, k2, 0.0, 0.0]`、`rotation` / `translation` は最後の反復の値にする。

最終精緻化は次のコマンドを subprocess で実行する（cwd は指定せず呼び出し元のカレントディレクトリを継承、環境変数も継承。相対パスは呼び出し元と同じ基準で解釈される。bug-007）:

```
[sys.executable, ROOT/"phase4/refine_extrinsics.py",
 "--toml", loop_toml, "--ply", --ply, "--images-dir", --images-dir,
 "--points-3d", --points-3d, "--points-2d", --points-2d,
 "--out-toml", final_toml, "--out-report", refine_report,
 "--cameras", camera, "--tmp-dir", tmp_dir/"refine", "--overwrite"]
```

- `--overwrite` を常に付ける（出力済みの確認は §4.2.1 で済ませているため）
- 終了コードが 0 以外 → 終了コード 1（stderr の末尾 2000 文字を表示）
- 受理の判定: `refine_report` の行のうち `f"  {camera}: 受理("` で始まる行があれば受理。無ければ不受理 → 終了コード 4

#### 4.2.7 FR-007 レポート

`{camera}_loop_report.txt` に次を書く（書式は §7）。

- ヘッダ: 実行日時、入力パス全部、初期パラメータ（f, cx, cy, k1, k2）
- FR-002 の結果
- 反復ごとの1行: i、マッチャー名（MASt3R / LoFTR）、マッチ数、ゲート通過数 N、外れ値除去後の点数、RMSE[px]、f、cx、cy、k1、k2、Δ_intr[px]、被覆ありセル数/16、最小二乗の status、所要時間[s]。被覆ありセル数が 12 未満なら行末に「警告: 被覆不足」
- 被覆ありセル: 1920x1080 を横 4 × 縦 4 に等分し（1セル 480x270）、外れ値除去後の u（写真側）が 10 点以上入っているセルの数
- 終了状態: 「成功」「未収束（ループ結果 TOML は最後の反復の値）」「範囲外（違反内容）」「ループ失敗（理由）」「精緻化不受理」「refine_extrinsics.py が異常終了（終了コード {n}）」「エラー（理由）」のいずれか（終了コードとの対応は §4.2.1 の「終了状態とループ結果 TOML の有無」の表）。評価の統計値と判定は、評価を実行した場合（終了コード 0）だけ記録する
- `refine_extrinsics.py` の所要時間[s]（最終精緻化を実行した場合）、評価の所要時間[s]（評価を実行した場合）

レポートを書くのは、§4.2.1 の`--out-dir`・`--tmp-dir` の新規作成（§4.2.1。bug-006）に成功した後に終了する場合だけで、終了コード（0, 1, 3, 4）によらず終了の直前に書き出す。新規作成で拒否した場合と引数エラー（終了コード 2）の場合は、レポートを含め `--out-dir` 配下に一切書き込まず、理由を標準エラー出力にだけ出す（既存の出力を壊さないため。NFR-004）。例外として、ファイルの書き込み自体が OSError で失敗した場合（ディスク容量不足など）は、Python の例外で異常終了する（終了コード 1。§4.2.1 の表の注記。初版の収束時と同じ）。レポート以外（ループ結果 TOML、評価の 4 ファイルなど）の書き込みが失敗した場合は、レポートの書き込みに進まない（レポートは作成されない）。それまでに書かれた成果物（ループ結果 TOML、最終 TOML など）や途中まで書いたファイルは残り得る。レポート自体の書き込みが失敗した場合は、不完全なレポートが残り得る。どの場合も、同じ出力先での再実行は --out-dir が既に存在するので拒否される（codex-36 指摘4）。

### 4.3 FR-008 `evaluate_overlay.py`

#### 4.3.1 CLI

```
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/evaluate_overlay.py \
    --toml <TOML> --camera <カメラ名> --ply <PLY> --image <前処理後画像PNG> \
    --out-prefix <出力接頭辞> [--overwrite]
```

終了コード: 0=評価を実行した（判定が合格・不合格・判定不能のいずれでも 0）、1=入力エラー・出力済み（`eval_output_paths(--out-prefix)` の 4 パスのいずれかが存在し `--overwrite` が無い。この場合は何も書かない）、2=引数エラー。`--out-prefix` の親ディレクトリが無ければ作る。画像サイズが TOML の size と一致しない場合は終了コード 1。

#### 4.3.2 関数構成

- `eval_output_paths(out_prefix) -> dict[str, str]`: 4 出力のパスを返す。`{"txt": f"{out_prefix}.txt", "blend": f"{out_prefix}_blend.png", "checker": f"{out_prefix}_checker.png", "flowmag": f"{out_prefix}_flowmag.png"}`。出力済み検査（`evaluate_overlay.py` の CLI と §4.2.1）と書き込み（`evaluate_and_write`）の両方がこの関数でパスを得る（命名を 1 か所で定義するため。codex-03 指摘1）
- `evaluate_and_write(gaussians, cam, photo_bgr, out_prefix) -> dict`: レンダしてから `flow_stats` と `make_images` を呼び、`eval_output_paths(out_prefix)` の 4 パスに書き、統計値の辞書を返す
- `flow_stats(photo_bgr, render_bgr, render_valid) -> dict`: GPU 不要。テスト対象
- `judge(stats) -> str`: `"合格"` / `"不合格"` / `"判定不能"`
- `make_images(photo_bgr, render_bgr, valid, mag) -> dict[str, ndarray]`

#### 4.3.3 処理手順

```
render_bgr = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
depth, alpha = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)
render_valid = isfinite(depth) & (alpha > 0.5) & (depth > 0.5) & (depth < 10.0)

flow_stats:
  clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
  P = clahe.apply(gray(photo)); Rn = clahe.apply(gray(render))
  dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
  F_fw = dis.calc(P, Rn, None)          # 写真の画素 x → レンダの x + F_fw(x)
  F_bw = dis.calc(Rn, P, None)
  mx = xs + F_fw[...,0]; my = ys + F_fw[...,1]
  inside = (0 <= mx <= W-1) & (0 <= my <= H-1)
  F_bw_s = cv2.remap(F_bw, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
  consistent = |F_fw + F_bw_s| < 1.0
  gx = Sobel(P, CV_32F, 1, 0, ksize=3); gy = Sobel(P, CV_32F, 0, 1, ksize=3)
  texture = sqrt(gx^2 + gy^2) >= 20.0
  valid = inside & consistent & texture & render_valid
  mag = |F_fw|
  stats = {n_valid, valid_ratio = n_valid / (W*H), median, p95 (np.percentile(…, 95)), mean, max}
          （n_valid == 0 のとき median, p95, mean, max は NaN）
judge:
  valid_ratio < 0.10 → "判定不能"
  median <= 1.0 → "合格"          # p95 は判定に使わない（イテレーション2）
  それ以外 → "不合格"
```

`render_valid` は写真側の画素 x と同じ位置の値で判定する（重ね合わせの評価対象は数 px 以内のずれなので、同じ位置で代用する）。

#### 4.3.4 出力画像

- `{out_prefix}_blend.png`: `cv2.addWeighted(photo, 0.5, render, 0.5, 0)`
- `{out_prefix}_checker.png`: 120px 四方のタイルで、(タイル列 + タイル行) が偶数のタイルは写真、奇数のタイルはレンダ
- `{out_prefix}_flowmag.png`: `clip(mag, 0, 5.0) / 5.0 * 255` を uint8 にして `cv2.applyColorMap(…, cv2.COLORMAP_JET)`。`valid` が False の画素は (128, 128, 128)
- `{out_prefix}.txt`: 入力パス、n_valid、valid_ratio、median、p95、mean、max（小数 3 桁）、`valid_ratio_raw` と `median_raw`（丸めない値。Python の `repr(float)` で書く。合否の境界値を出力から確かめられるようにするため。イテレーション2、codex-17 指摘3）、合格ライン（書式: `合格ライン: median <= 1.000 px（valid_ratio >= 0.100）。p95 は記録のみ`）、判定。文字列の組み立ては関数 `format_eval_txt(stats, verdict, inputs_text) -> str` で行い、`evaluate_and_write` はその戻り値を書く。行の順序: 1 行目 `feat-041: 重ね合わせ評価`、入力、n_valid、valid_ratio、valid_ratio_raw、median、median_raw、p95、mean、max、合格ライン、判定

## 5. ファイル・ディレクトリ設計

- 入力: 原本 JPEG、PLY、3D 基準点 CSV（`kijunten_locations.csv` 形式）、2D 点 CSV（`points_2d.csv` 形式）
- 前処理後画像のファイル名は `{camera}.png`（`refine_extrinsics.py` の規約）
- 出力ファイル名は §1.3 のとおり。評価の 4 ファイルは `estimate_intrinsics_loop.py` が `out_prefix = {out-dir}/{camera}_eval` で `evaluate_and_write` を呼ぶことで、`{camera}_eval.txt`、`{camera}_eval_blend.png`、`{camera}_eval_checker.png`、`{camera}_eval_flowmag.png` になる
- 中間ファイル（`--tmp-dir`）: `iter{i:02d}_render.png`、`iter{i:02d}_matches.npz`（i = 0 は MASt3R の NPZ でキー `u_a`, `u_b`, `meta`、i ≥ 1 は LoFTR の NPZ でキー `u_q`, `u_r`, `conf`, `meta`）、`refine/`（`refine_extrinsics.py` の中間）。`--out-dir` と `--tmp-dir` には存在しない新しいパスを指定する（既に存在すれば終了コード 1。bug-006）。`--tmp-dir` 省略時はシステムの一時ディレクトリを使い、終了時（終了コードによらず）に削除する
- 設定ファイルは使わない（CLI 引数のみ）。※ feat-042 で `--config`（設定YAML）に対応した（`docs/issues/feat-042-config-yaml-refine-tools/`）

## 6. インターフェース定義

### 6.1 `prepare_phone_photo.py`

```python
def read_exif(path: str) -> dict: ...                       # Pillow を関数内 import
def validate_exif(exif: dict, width: int, height: int,
                  expect_model: str | None, expect_focal_mm: float | None) -> list[str]: ...
def compute_crop(width: int, height: int) -> tuple[int, int]: ...   # (y0, crop_h)。不正は ValueError
def initial_focal_px(f35: float, width: int, height: int, out_w: int = 1920) -> float: ...
def convert_point(x: float, y: float, y0: int, scale: float) -> tuple[float, float]: ...
def format_init_toml(camera: str, f: float) -> str: ...
def main(argv: list[str] | None = None) -> int: ...
```

### 6.2 `estimate_intrinsics_loop.py`

```python
@dataclass
class Params:
    f: float; cx: float; cy: float; k1: float; k2: float
    rvec: np.ndarray; tvec: np.ndarray            # shape (3,), float64
    def K(self) -> np.ndarray: ...
    def D(self) -> np.ndarray: ...                # [k1, k2, 0, 0]
    def to_cam(self) -> dict: ...                 # render_image / render_depth_alpha_distorted 用
    def to_vector(self) -> np.ndarray: ...        # 11 要素
    @staticmethod
    def from_vector(x: np.ndarray) -> "Params": ...

class LoopFailure(Exception): ...                 # 終了コード 3

def call_mast3r(query_png: Path, render_png: Path, out_npz: Path) -> tuple[np.ndarray, np.ndarray, dict]: ...
def lift_to_world(u_r: np.ndarray, d: np.ndarray, params: Params) -> tuple[np.ndarray, np.ndarray]: ...  # (X_w, 逆歪み検査の合格マスク)
def refine_undistort_error_px(K: np.ndarray, D: np.ndarray) -> float: ...
def joint_refine(X: np.ndarray, u: np.ndarray, x0: np.ndarray) -> tuple[Params, int]: ...  # (結果, status)
def delta_intr(old: Params, new: Params) -> float: ...
def is_converged(i: int, delta: float) -> bool: ...   # i >= LOFTR_FROM_ITER and delta < CONV_PX
def check_ranges(p: Params) -> list[str]: ...
def count_covered_cells(u: np.ndarray) -> int: ...
def run_loop_iteration(gaussians, params: Params, query_png: Path, i: int, tmp_dir: Path) -> dict: ...
def format_loop_toml(camera: str, p: Params) -> str: ...
def refine_accepted(report_text: str, camera: str) -> bool: ...
def main(argv: list[str] | None = None) -> int: ...
```

### 6.3 `evaluate_overlay.py`

```python
def flow_stats(photo_bgr: np.ndarray, render_bgr: np.ndarray, render_valid: np.ndarray) -> dict: ...
    # 戻り値は stats に加えて "valid"(H,W bool) と "mag"(H,W float32) を含む
def judge(stats: dict) -> str: ...
def make_images(photo_bgr, render_bgr, valid, mag) -> dict[str, np.ndarray]: ...  # キー: blend, checker, flowmag
def eval_output_paths(out_prefix: str) -> dict[str, str]: ...   # キー: txt, blend, checker, flowmag
def evaluate_and_write(gaussians, cam: dict, photo_bgr: np.ndarray, out_prefix: str) -> dict: ...
def format_eval_txt(stats: dict, verdict: str, inputs_text: str) -> str: ...   # {out_prefix}.txt の内容（§4.3.4）
def main(argv: list[str] | None = None) -> int: ...
```

## 7. ログ・デバッグ設計

- 標準出力に進捗を日本語で出す（例: `反復 i=2: レンダ中...`、`反復 i=2: LoFTR 実行中...`。マッチャー名は i = 0 で MASt3R、i ≥ 1 で LoFTR）。ログライブラリは使わない（既存スクリプトと同じ）
- エラーは標準エラー出力に `エラー: {内容}` で出す
- レポートの反復行の書式:

```
i=2 | LoFTR マッチ5321 N=4410 inl=3102 RMSE=0.912px | f=1283.41 cx=957.88 cy=543.02 k1=0.0123 k2=-0.0451 | Δ=0.731px | 被覆14/16 | lsq_status=1 | 41.2s
```

## 8. テスト設計（`tests/test_feat041_intrinsics_loop.py`、GPU・実データ不要）

| # | 対象 | 内容 | 期待 |
|---|---|---|---|
| 1 | `compute_crop` | (4000,3000)、(4000,2252)、(1920,1080)、(4000,2000)、(4001,3000) | (375,2250)、(1,2250)、(0,1080)、ValueError、ValueError |
| 2 | `initial_focal_px` | f35=23, 4000x3000 | 23×5000/43.27×0.48 と 1e-9 以内で一致 |
| 3 | `convert_point` | 原本の切り抜き範囲の四隅の画素中心 (−0.5 補正込み) | 前処理後の対応点に一致（式の逆変換との往復誤差 1e-9 以内） |
| 4 | `validate_exif` | 正常な辞書、各違反を1つずつ含む辞書 | 正常は空リスト、違反ごとに1件 |
| 5 | `format_init_toml` / `format_loop_toml` | 出力を `load_cameras_toml` で読み戻す | K、D、size、rvec、tvec が一致 |
| 6 | `joint_refine` | 既知パラメータ（f=1280, cx=955, cy=545, k1=0.05, k2=−0.1、任意の姿勢）で 3D 点 2000 点（奥行き 1.5〜5m、画像全体に分布）を投影し、seed 0 の正規ノイズ σ=0.3px を加え、初期値を f=1250、cx=960、cy=540、k1=k2=0、姿勢に 2cm/0.5° の誤差を入れる | f 相対誤差 ≤ 0.5%、\|cx 誤差\|, \|cy 誤差\| ≤ 3px、\|k1 誤差\|, \|k2 誤差\| ≤ 0.02 |
| 6b | `residuals` の形状 | X: (300,3)、u: (300,2) と、solvePnPRansac 形式の inl（shape (300,1)）から §4.2.3 の手順で X, u を作った場合 | どちらも残差ベクトルの要素数が 600（2N） |
| 7 | `delta_intr` | (a) 歪みなしの同じパラメータ、(b) k1=0.2, k2=0.1 の同じパラメータ（f=1280, cx=960, cy=540）、(c) 歪みなしで f だけ 1280→1281.28（0.1%）に変える | (a)(b) 1e-9 以内で 0、(c) 0.001 × √(960²+540²) = 1.1015px と 0.001px 以内で一致 |
| 7b | `run_loop_iteration` の失敗変換 | `gate_breakdown` の N が 0 / 3 / 299 になるよう細工した入力（レンダ・MASt3R はモックで置き換える） | いずれも `LoopFailure`（理由に「ゲート通過」を含む）。`cv2.error` は送出されない |
| 7c | 出力済み検査 | `--out-dir` に既存の `{camera}_loop_report.txt`（内容 "old"）を置いて `main` を呼ぶ（PLY 読み込み前に終わる） | 終了コード 1、既存ファイルの内容は "old" のまま |
| 7d | 前処理の PNG 出力 | 小さな画像（16x9 の倍数）で `prepare_phone_photo.main` を Exif 読み取りをモックして実行 | `{camera}.png` が `cv2.imread` で読めて 1920x1080、`{camera}.png.tmp` が残っていない |
| 8 | `check_ranges` | 各境界値の内側・外側 | 外側だけ違反として列挙 |
| 9 | `count_covered_cells` | 全セルに 10 点ずつ / 1 セルだけ 9 点 | 16 / 15 |
| 10 | `refine_accepted` | `  cam: 受理(単峰)`、`  cam: 受理(仲裁)`、`  cam: 不受理`、`  cam: 失敗(…)` を含む文字列 | True、True、False、False |
| 11 | `flow_stats` | seed 0 の乱数テクスチャ画像（1920x1080、ガウシアンぼかし σ=1.5）と、それを x 方向に 2px 平行移動した画像、render_valid は全 True | median 1.8〜2.2px |
| 12 | `flow_stats` | 同一画像の組 | median ≤ 0.1px |
| 13 | `judge` | valid_ratio 0.09 / (median 1.0, p95 3.0) / (1.01, 3.0) / (1.0, 50.0) / (median 1.0004, p95 3.0) / (median 0.9996, p95 3.0) / (valid_ratio 0.0999, median 0.5) / (valid_ratio 0.1, median 0.5) | 判定不能 / 合格 / 不合格 / 合格 / 不合格 / 合格 / 判定不能 / 合格（小数 3 桁で同じ表示になる境界の両側を含む） |
| 14 | `lift_to_world` | f=1280, cx=960, cy=540 で (a) 歪みなし、(b) k1=-0.3, k2=0、(c) k1=0.2, k2=0.1。姿勢ゼロと任意姿勢。3D 点 (1.6, 0.8, 2.0)m を含む画像内の点を投影した画素と、カメラ座標 z を深度として与える | 元の 3D 点と 1e-6m 以内で一致し、逆歪み検査で除外されない |
| 14b | `run_loop_iteration` の点数同期 | ゲート通過 301 点のうち 1 点だけが逆歪み検査に不合格になる入力（中心付近 300 点 + 逆歪み誤差の大きい端の 1 点。レンダ・MASt3R はモック、solvePnPRansac に渡る配列を記録する） | 例外なく進み、solvePnPRansac に渡る 3D 点と写真座標がともに 300 点で、各行が元の同じマッチ由来（順序一致） |
| 15 | `refine_undistort_error_px` | f=1280, cx=960, cy=540 で (a) 歪みなし、(b) k1=0.05, k2=-0.1、(c) k1=-0.3, k2=0 | (a) 1e-6px 以下、(b) 0.1px 以下（手元の確認値 0.0022px）、(c) 0.1px を超える |
| 16 | `run_loop_iteration` のマッチャー切り替え | レンダ・`call_mast3r`・`call_matcher` をモックで置き換え、i = 0、1、2 で呼ぶ。`call_matcher` のモックは信頼度 0.1 と 0.5 のマッチを混ぜて返す | i = 0 では `call_mast3r` だけ、i = 1、2 では `call_matcher` だけが呼ばれる。i ≥ 1 では `gate_breakdown` に渡る点数が信頼度 0.5 のマッチの件数と一致する。`diag["matcher"]` が i = 0 で "MASt3R"、i ≥ 1 で "LoFTR" |
| 17 | LoFTR の OOM | i = 1 で `call_matcher` のモックが `MatcherOOM` を送出する | `LoopFailure`（理由に「LoFTR OOM」を含む） |
| 18 | `is_converged` | (i, delta) = (0, 0.0)、(0, 1.99)、(1, 1.99)、(1, 2.0)、(2, 0.0) | False、False、True、False、True |
| 19 | 未収束時のループ結果 TOML | `estimate_intrinsics_loop` のモジュール属性を monkeypatch で置き換えて `main` を呼ぶ。置き換える属性: `load_cameras_toml`、`select_camera`、`load_points_3d_csv`、`load_points_2d_csv`、`validate_camera`（`{"ok": True, ...}`）、`pnp_manual`、`median_reproj_px`、`run_loop_iteration`。`run_loop_iteration` のモックは呼び出し回数を記録し、反復 i ごとに固有の値を返す: f = 1280 + 20 × (i mod 2) + 0.1 × i、cx = 960 + 0.01 × i、cy = 540 − 0.01 × i、k1 = 0.001 × i、k2 = −0.001 × i、rvec = (0.01 × i, 0.02, 0.03)、tvec = (0.1 × i, 0.2, 0.3)（毎回 Δ_intr ≥ 2.0 になる）。`sys.modules["render"]` に `load_ply` を持つダミーモジュールを入れる | 終了コード 3。`run_loop_iteration` がちょうど 8 回（i = 0〜7）呼ばれる。`{camera}_loop.toml` が存在し、`render_keypoints.load_cameras_toml` で読み戻した K（f, cx, cy）、D（k1, k2）、rvec、tvec が i = 7 の値と 1e-9 以内で一致する（i = 5 など他の反復の値とは一致しない）。`{camera}_final.toml` と評価の 4 ファイルは存在しない。レポートに「未収束（ループ結果 TOML は最後の反復の値）」がある |
| 20 | `--overwrite` の廃止とリンクの検出 | (a) `main` に `--overwrite` を付けて呼ぶ。(b) `--out-dir` に、リンク先が存在しないシンボリックリンク `{camera}_final.toml` を作って `main` を呼ぶ | (a) 終了コード 2（argparse の引数エラー）。(b) 終了コード 1、`--out-dir` 配下に新しいファイルが作られず、リンク先のパスにもファイルが作られない |
| 21 | `format_eval_txt` | stats（n_valid=207360、valid_ratio=0.1、median=1.0004、p95=4.0、mean=1.2、max=50.0）と判定 "不合格" で文字列を作る | `median: 1.000 px` と `median_raw: 1.0004`、`valid_ratio_raw: 0.1` の行があり、`_raw` の行の値を `float()` で読むと元の値と完全に一致する。合格ラインの行が `合格ライン: median <= 1.000 px（valid_ratio >= 0.100）。p95 は記録のみ` |
| 23 | `--out-dir` の新規作成（bug-006） | テスト 19 と同じモックで、`--out-dir` に (a) 既存の空ディレクトリ、(b) リンク先の無いシンボリックリンク、(c) 未作成の親と `..` を含み既存ディレクトリを指すパス（`{tmp_path}/new2/../outx`。`outx` は既存で、中に `{camera}_final.toml.tmp`（内容 "old"）を置く）を指定して `main` を呼ぶ | どれも終了コード 1。`run_loop_iteration` は呼ばれない。(a) は空のまま。(b) のリンクはそのまま。(c) の `{camera}_final.toml.tmp` は内容 "old" のまま |
| 24 | `--tmp-dir` の新規作成（bug-006） | テスト 19 と同じモックで、`--out-dir` は存在しないパス、`--tmp-dir` に (a) 既存の空ディレクトリ、(b) リンク先の無いシンボリックリンクを指定して `main` を呼ぶ。対照として (c) 存在しないパス | (a)(b) 終了コード 1、`run_loop_iteration` は呼ばれず、`--out-dir` は残っていない（作ってから消される）。(c) 従来どおり進む（テスト 19 と同じ終了コード 3） |
| 25 | 親成分のシンボリックリンク（bug-006、codex-03 の配置） | `{tmp_path}/existing/sub/iter00_render.png`（内容 "old"）を置き、`{tmp_path}/link -> new/../existing` のシンボリックリンクを作る（`new` は未作成）。`--out-dir {tmp_path}/new --tmp-dir {tmp_path}/link/sub` で `main` を呼ぶ（テスト 19 と同じモック） | 終了コード 1。`iter00_render.png` は内容 "old" のまま。`run_loop_iteration` は呼ばれない。`{tmp_path}/new` は残っていない |
| 26 | カメラ名の制限（bug-006） | テスト 19 と同じモックで、`--camera` に (a) `{tmp_path}/victim`（絶対パス。`{tmp_path}/victim_loop_report.txt` に内容 "old" を置く）、(b) `sub/cam`、(c) `..` を指定し、`--out-dir` は存在しないパスで `main` を呼ぶ | どれも終了コード 1。`--out-dir` は作られない。(a) の `victim_loop_report.txt` は内容 "old" のまま |
| 27 | `--tmp-dir` 作成の途中失敗（bug-006） | テスト 19 と同じモックで、`--out-dir {tmp_path}/new --tmp-dir {tmp_path}/new/sub/..` で `main` を呼ぶ | 終了コード 1（例外で落ちない）。標準エラー出力に「中間ファイル置き場が既に存在します」と「作成したディレクトリが残っています」の両方が出る。`run_loop_iteration` は呼ばれない |

テスト結果は `tests/results/feat-041_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する。

## 9. 実データでの動作確認

> **注記（feat-044、2026-11-26）**: 本節のコマンド例の `--expect-model` / `--expect-focal-mm` は feat-044 で削除された（指定すると終了コード 2）。前処理には `--photo-info`（写真情報ファイル）が必須になった。現在の仕様は `docs/issues/feat-044-remove-expect-exif-options/` と README を参照。本節は当時の記録として残す。

実装担当（CLAUDE.md ステップ6 の Sonnet サブエージェント）が実行する（requirements.md §7）。手順 6 の 1 回の実行を、イテレーション2 の正式な判定の結果とする（`experiments/e3_10F_it2/criteria.md` §2）。これはステップ6 の実データ検証である。ユーザーはステップ7（手動テスト）でその出力を確認する（確認項目と完了条件は investigation.md イテレーション2 §2.5）。

1. 前処理:
   ```
   uv run --project phase4 python phase4/prepare_phone_photo.py \
     --image phase4/data/10F/s25_original/20261124_152436.jpg --camera s25_20261124_152436 \
     --images-dir phase4/data/10F/s25_images --out-toml phase4/data/10F/s25_20261124_152436_init.toml \
     --points-2d-in phase0/data/points_2d_10F.csv --points-2d-out phase4/data/10F/points_2d_s25_20261124_152436.csv \
     --expect-model "Galaxy S25" --expect-focal-mm 5.4 --overwrite
   ```
   `s25_images/s25_20261124_152436.png` は疎通確認で作ったものを上書きする（同じ切り抜き・縮小のため画像内容は同じ）
2. 比較用の測定（判定には使わない）:
   ```
   TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/evaluate_overlay.py \
     --toml phase4/data/10F/s25_refined.toml --camera s25_20261124_152436 --ply phase4/data/10F/point_cloud.ply \
     --image phase4/data/10F/s25_images/s25_20261124_152436.png --out-prefix phase4/data/10F/baseline/s25_20261124_152436_eval
   ```
3. 本処理:
   ```
   TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py \
     --toml phase4/data/10F/s25_20261124_152436_init.toml --camera s25_20261124_152436 \
     --ply phase4/data/10F/point_cloud.ply --images-dir phase4/data/10F/s25_images \
     --points-3d phase0/data/kijunten_locations_10F.csv \
     --points-2d phase4/data/10F/points_2d_s25_20261124_152436.csv \
     --out-dir phase4/data/10F/feat041_out --tmp-dir phase4/data/10F/feat041_tmp
   ```
4. 合格ライン: `feat041_out/s25_20261124_152436_eval.txt` の判定が「合格」（中央値 ≤ 1.0px かつ 95 パーセンタイル値 ≤ 3.0px。初版の基準で、手順 3・5 の実行時点のもの。イテレーション2 以降の基準は手順 6）。CLAUDE.md の実験プロトコルに従い、2 と 3 の実行直前に予測値を、実行後に実測値を案件フォルダの `experiments/e1_10F/experiment_log.md` に記録する
5. investigation.md イテレーション1（i ≥ 1 を LoFTR に切り替え）の確認: 手順 1・2 は再実行せず前回の出力を使い、手順 3 を出力先だけ変えて実行する（前回の結果を比較のため残す）:
   ```
   TORCH_CUDA_ARCH_LIST="9.0+PTX" /usr/bin/time -v uv run --project phase4 python phase4/estimate_intrinsics_loop.py \
     --toml phase4/data/10F/s25_20261124_152436_init.toml --camera s25_20261124_152436 \
     --ply phase4/data/10F/point_cloud.ply --images-dir phase4/data/10F/s25_images \
     --points-3d phase0/data/kijunten_locations_10F.csv \
     --points-2d phase4/data/10F/points_2d_s25_20261124_152436.csv \
     --out-dir phase4/data/10F/feat041_out_it1 --tmp-dir phase4/data/10F/feat041_tmp_it1
   ```
   合格ラインは手順 4 と同じ（`feat041_out_it1/s25_20261124_152436_eval.txt` の判定が「合格」）。実行直前の予測値（反復数、各反復のマッチャー・マッチ数・N・RMSE の範囲、最終 f・k1・k2、評価の中央値・p95）と実行後の実測値、および前回の結果（median 1.171px、p95 13.125px）との比較表を `experiments/e2_10F_loftr/experiment_log.md` に記録する。あわせて `/usr/bin/time -v` の Elapsed (wall clock) time を記録し、NFR-001（全体 60 分以内）を満たすことを確認する
6. investigation.md イテレーション2（CONV_PX = 2.0、未収束時の TOML 出力、合否基準を中央値のみに変更）の確認: 手順 1・2 は再実行しない。手順 3 を出力先だけ変えて実行する:
   **事前確認（実行前に必ず行う。bug-006 の回避条件）**: (1) 出力先 `phase4/data/10F/feat041_out_it2` と中間ディレクトリ `phase4/data/10F/feat041_tmp_it2` がどちらも存在しないこと（それぞれ `test ! -e {パス} && test ! -L {パス}`。リンク先の無いシンボリックリンクも「存在する」とみなす。中間ディレクトリも確認するのは、`iter00_render.png` などへの書き込みが、残っていたリンク経由で入力を上書きし得るため。codex-31 指摘1）、(2) `--toml` などの入力ファイルがすべて `phase4/data/10F/feat041_out_it2` の外にあること。どちらかを満たさなければ正式な実行を開始せず、状況を報告して中断する。確認結果は実験ログに記録する。（bug-006 の修正後は、`--out-dir`・`--tmp-dir` が存在しないことをツール側でも検査する）
   ```
   exit_code=0
   TORCH_CUDA_ARCH_LIST="9.0+PTX" /usr/bin/time -v uv run --project phase4 python phase4/estimate_intrinsics_loop.py \
     --toml phase4/data/10F/s25_20261124_152436_init.toml --camera s25_20261124_152436 \
     --ply phase4/data/10F/point_cloud.ply --images-dir phase4/data/10F/s25_images \
     --points-3d phase0/data/kijunten_locations_10F.csv \
     --points-2d phase4/data/10F/points_2d_s25_20261124_152436.csv \
     --out-dir phase4/data/10F/feat041_out_it2 --tmp-dir phase4/data/10F/feat041_tmp_it2 \
     || exit_code=$?
   printf 'EXIT_CODE=%s\n' "$exit_code"
   ```
   パイプやログファイルへの書き込みは使わない（固定パスへの書き込みが入力を上書きし得るため。codex-27）。正式な終了コードは直後に表示される `EXIT_CODE=` の値とする（`|| exit_code=$?` の形にするのは、シェルの errexit が有効でも表示まで到達させるため。codex-28）（`/usr/bin/time -v` が出力する Exit status 欄は、シグナル終了時に 0 と表示されることがあるので使わない）。`Command terminated by signal` の行が出た場合は異常終了として扱う。所要時間は `/usr/bin/time -v` が標準エラー出力に出す Elapsed (wall clock) time を使う
   実行前に、判定基準 `experiments/e3_10F_it2/criteria.md` が Codex レビューで高・中ゼロに収束していること（criteria lock）を確認する。合格ラインは criteria.md §2 のすべて（終了コード 0・終了状態「成功」、valid_ratio ≥ 0.10、中央値 ≤ 1.0px、eval.txt の判定「合格」と合格ラインの記載、`/usr/bin/time -v` の Elapsed (wall clock) time で全体 60 分以内、最終 TOML と評価 4 ファイルが読めること。p95 は記録のみ）。実行直前の予測値と実行後の実測値（項目は `experiments/e3_10F_it2/criteria.md` §3 の一覧をそのまま使う: 収束する反復、最終 f・cx・cy・k1・k2、最終精緻化の受理の種類、評価の median・p95・valid_ratio、全体の所要時間）、参考実行（`feat041_out_it1_dump/conv2/`、median 0.671px、p95 4.085px）との比較表を `experiments/e3_10F_it2/experiment_log.md` に記録する。途中で失敗した場合は、到達しなかった項目を理由付きで「未実行」または「取得不能」と記録し、総合判定を「不合格」とする。総合判定が不合格なら、予測との乖離の有無にかかわらず、再試行やステップ7 に進まず investigation.md の次のイテレーションに戻る。総合判定が合格の場合だけステップ7 に進む（予測と実測が乖離した項目があれば、その原因を特定して実験ログに記録してから進む。原因を特定できない場合は、合格でもステップ7 への遷移を保留し、ユーザーに報告する）。実行後、criteria.md §2 の全 6 項目について、実測値または確認結果（終了コードは `EXIT_CODE=` の値、所要時間は Elapsed (wall clock) time、`valid_ratio_raw`・`median_raw`・`n_valid` は eval.txt、成果物の読み取り結果）と総合判定（合格／不合格）を `experiments/e3_10F_it2/experiment_log.md` に記録する。`EXIT_CODE=` の行、`/usr/bin/time -v` の出力全体、`Command terminated by signal` の行の有無は、コマンドの出力からそのまま実験ログに書き写す

## 10. 設計判断の記録（ADR）

| 判断 | 採用 | 却下案と理由 |
|---|---|---|
| 焦点距離 | fx = fy の 1 変数 | fx, fy の 2 変数: スマートフォンの画素は正方形で、1 枚の写真から 2 変数を分離すると不安定になる |
| ループのマッチャー | i = 0 は MASt3R（既存 `mast3r_cli.py`）、i ≥ 1 は LoFTR（既存 `loftr_cli.py`）。investigation.md イテレーション1、ユーザー決定 | 全反復 MASt3R: 長辺 512px で推論するため座標が 3.75px 刻みになり、1px 未満のずれを測れない（10F の実データで i ≥ 2 のマッチの 86〜89% が変位 0 になり、見かけの収束になった）。全反復 LoFTR: 視点・拡大率の差に弱く、内部パラメータが数 % ずれ歪みもゼロの i = 0 に不利。MASt3R のマッチの局所相関によるサブピクセル補正、MASt3R の切り出し原寸処理、密なオプティカルフロー、画像を直接比べる最適化: 検討したが、既存部品で実装でき、連続値が得られることを実データで確認済みの LoFTR を採用した |
| 最終精緻化 | `refine_extrinsics.py` を subprocess で呼ぶ | 関数 import で同一プロセス実行: `refine_extrinsics.main` が PLY を再読み込みし、中間ファイル・終了処理も独立しているため、プロセスを分けたほうが既存コードを変えずに済む |
| 損失関数 | soft_l1（trf） | lm（`estimate_camera_params.py` と同じ）: lm は頑健損失を使えず、RANSAC を通った MASt3R の誤マッチの影響をそのまま受ける |
| 収束の指標 | Δ_intr（格子点の移動量の最大値） | f・主点・k1・k2 ごとのしきい値: 単位が異なり、画像上の影響が比較できない。Δ_intr は 1 つの px 値で全部をまとめて評価できる |
| 収束しきい値 | CONV_PX = 2.0px（イテレーション2。10F の 1 回の実行で観測した揺れの上限 1.872px を上回る値） | 0.5px: LoFTR の反復ごとの揺れ（Δ_intr で 1.0〜1.9px）より小さく、10F の実データで 8 反復しても収束しなかった。直近数回の平均で判定する方式: 揺れをならせるが設計変更が大きい。反復回数の固定: 収束の判定がなくなる |
| 変位しきい値 | i=0 で 250px、i ≥ 1 で 30px | 固定: i=0 は初期の焦点距離誤差（数 %）で画像の端が数十 px ずれうるので広く取り、2 回目以降は内部パラメータが更新済みのため狭める |
| 評価のずれの測り方 | DIS オプティカルフロー（推定に使っていない方法） | MASt3R / LoFTR のマッチ残差: 推定に使った方法で採点すると、推定結果を自分自身で評価することになる |
| 合否基準 | 中央値 ≤ 1.0px のみ。p95 は記録のみ（イテレーション2、ユーザー決定） | 中央値 ≤ 1.0px かつ p95 ≤ 3.0px（初版）: 根拠がなかった。10F の測定で、p95 を押し上げるずれは最下段に集まり、240x180 のセルの中では向きがそろう（一貫性 C の中央値 0.81）が、主点から見た放射成分の符号はセルごとに正 4・負 4 で、隣のセルで反転した（原因は未特定。仮説は 3DGS の下側の撮影不足による局所的な再現誤差。カメラパラメータ・DIS・3DGS の各寄与は分離していない）。3DGS の再現が不確かな領域を除いて p95 を測る: 領域を決める客観的な基準が無い。新しい基準は、参考実行と測定の結果を見た後に決めた（investigation.md イテレーション2） |
| 評価の集計対象 | テクスチャあり・順逆整合・3DGS 有効の画素 | 全画素: 白い壁のようなテクスチャの無い領域では DIS のフローがほぼ 0 になり、ずれを過小評価する |
| 2D 点の座標変換 | 画素中心規約 `(X+0.5)s−0.5` | 疎通確認で使った `0.48X`: `cv2.resize` の画素対応と 0.26px ずれる |
| 既存コードの変更 | しない（import と subprocess のみ） | 共通部分を切り出すリファクタ: feat-026・feat-035 のテストと動作を変えるリスクがある |

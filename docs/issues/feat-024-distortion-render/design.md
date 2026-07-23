# feat-024 機能設計書: render_keypoints.py 歪みモデル対応レンダリング（GT比較用）

## 1.1 対応要求マッピング

| 要求ID | 対応セクション |
|---|---|
| FR-001 | 1.4.1（CLI）、1.4.3（静止画モード分岐） |
| FR-002 | 1.4.1 |
| FR-003 | 1.4.1 |
| FR-004 | 1.4.1、1.4.2（render_image 拡張） |
| FR-005 | 1.4.2（distortions_to_gsplat） |
| FR-006 | 1.4.2、1.4.3（既存経路の不変性） |

## 1.2 システム構成

変更対象は `phase4/render_keypoints.py` の1ファイルのみ（+ テスト・ドキュメント）。

```
phase4/render_keypoints.py           # 本体（CLI・render_image 拡張・静止画モード分岐・係数変換関数）
tests/test_feat024_distort_render.py # 新規テスト
docs/BACKLOG.md, docs/CHANGELOG.md, CLAUDE.md  # ドキュメント更新（完了ステップで）
```

依存関係は既存のまま（`render_keypoints.py` → `render.py` の `load_ply` 等）。
`render.py` は変更しない。

## 1.3 技術スタック

- Python 3.10 / uv（phase4 環境）、gsplat 1.5.3、torch、OpenCV、NumPy、tomli（すべて既存）
- gsplat 3DGUT 経路の使用条件（スパイク検証済み・gsplat 1.5.3 のソースで確認済み）:
  `with_ut=True, with_eval3d=True, packed=False`、
  `radial_coeffs`: torch.float32 CUDA `[1, 6]`、`tangential_coeffs`: torch.float32 CUDA `[1, 2]`

## 1.4 各機能の詳細設計

### 1.4.1 CLI（FR-001〜FR-004）

`_build_parser()` の変更:

```python
# c3d_path を省略可能な位置引数にする（既存3引数呼び出しは無変更で動作）
parser.add_argument("c3d_path", nargs="?", default=None,
                    help="人体キーポイントC3Dファイルパス（--no-keypoints 時は省略）")
# 追加フラグ
parser.add_argument("--no-keypoints", action="store_true",
                    help="キーポイント描画をスキップし3DGS背景のみの静止画1枚を出力（c3d_path 省略必須）")
parser.add_argument("--distort", action="store_true",
                    help="TOMLの歪み係数で歪みモデルレンダリング（--no-keypoints 専用）")
```

`--occlusion-margin` は「明示指定されたか」を検出できるよう argparse の default を
`None` に変更し、バリデーション後に既定値を補完する（help 文言は「default: 0.05」の
表記を維持）:

```python
parser.add_argument("--occlusion-margin", type=float, default=None,   # 既定 OCCLUSION_MARGIN は後段で補完
                    help=...)  # help の既定値表記は現行のまま
# main() のバリデーション後に:
if args.occlusion_margin is None:
    args.occlusion_margin = OCCLUSION_MARGIN
# これ以降の既存コード（フレームループ等）は args.occlusion_margin を従来どおり参照できる
```

`main()` のバリデーション（**既存の `--no-png` 単独チェックより前**、カメラ選択より前に
追加。順序が重要: `--no-keypoints --no-png` は FR-003 のメッセージで拒否されなければ
ならず、既存の「--no-png は --mp4 と併用」に先に捕まってはならない。
すべて `parser.error()` = 終了コード2）:

```python
if args.no_keypoints:
    if args.c3d_path is not None:
        parser.error("--no-keypoints 指定時は c3d_path を渡せません（静止画モードは C3D 不要）")
    # 静止画モードで無意味なオプションを個別に拒否（指定されたもののみ名前を出す）
    forbidden = [("--mp4", args.mp4), ("--mp4-fps", args.mp4_fps is not None),
                 ("--no-png", args.no_png),
                 ("--start-frame", args.start_frame is not None),
                 ("--end-frame", args.end_frame is not None),
                 ("--no-occlusion", args.no_occlusion),
                 ("--occlusion-margin", args.occlusion_margin is not None)]
    for name, given in forbidden:
        if given:
            parser.error(f"--no-keypoints 指定時は {name} は使用できません")
else:
    if args.c3d_path is None:
        parser.error("c3d_path を省略する場合は --no-keypoints を指定してください")
    if args.distort:
        parser.error("--distort は --no-keypoints と併用してください（動画モードは未対応）")
```

補足:
- `--occlusion-margin` の「指定されたか」は default=None 化により `is not None` で
  厳密判定する（既定値と同値の明示指定 `--occlusion-margin 0.05` も検出できる）
- `--distort` の判定は `--no-keypoints` 側の分岐では不要（静止画モードでは常に許可）

### 1.4.2 render_image 拡張と係数変換（FR-004, FR-005, FR-006）

**新規関数**（`render_image` の直前に配置）:

```python
def distortions_to_gsplat(D: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """TOML歪み係数（OpenCV並び、長さ4/5/8）を gsplat 用に詰め替える。

    Returns:
        (radial, tangential): radial は6要素 [k1,k2,k3,k4,k5,k6]、
        tangential は2要素 [p1,p2]（いずれも float64 numpy）
    Raises:
        ValueError: 長さが4/5/8以外
    """
    # len 4: [k1,k2,p1,p2]           → radial [k1,k2,0,0,0,0]
    # len 5: [k1,k2,p1,p2,k3]        → radial [k1,k2,k3,0,0,0]
    # len 8: [k1,k2,p1,p2,k3..k6]    → radial [k1,k2,k3,k4,k5,k6]
    # tangential は常に [p1,p2] = D[2:4]
```

**`render_image` のシグネチャ変更**（引数は**末尾に追加**。既存呼び出しは無変更で動作）:

```python
def render_image(gaussians, cam, near_plane, background=(0.0, 0.0, 0.0),
                 return_depth=False, distort=False):
```

- `distort=False`（既定）: 現行と**完全に同一**の rasterization 呼び出し
  （`camera_model="pinhole", with_ut=False, packed=True`。引数・順序も変えない）
- `distort=True`: `cam["D"]` を `distortions_to_gsplat` で変換し、
  ```python
  rasterization(..., camera_model="pinhole",
                with_ut=True, with_eval3d=True, packed=False,
                radial_coeffs=torch.tensor([radial], dtype=torch.float32, device=device),
                tangential_coeffs=torch.tensor([tangential], dtype=torch.float32, device=device),
                render_mode=render_mode, ...)
  ```
  それ以外の引数（means〜far_plane, sh_degree, render_mode）は distort=False 側と共通
- 分岐は rasterization 呼び出しの引数構成のみ。前後の viewmat/K 準備・背景合成・
  BGR変換・深度処理は共通コードのまま
- docstring の「歪みを gsplat の UT経路に乗せると黒い靄・品質劣化が出るため使わない」の
  記述は、feat-024 スパイクの結論（靄の真因は near_plane=0.01 の floater。
  `with_eval3d=True` + 適切な near_plane なら UT経路は実用可）に**書き換える**

### 1.4.3 静止画モード分岐（FR-001, FR-006）

`main()` のフロー変更。カメラ選択（既存 549〜556行）までは共通。その直後に分岐する:

```python
if args.no_keypoints:
    return _run_still_mode(args, cam)   # 静止画モード（新規関数）
# 以降、既存の動画モード処理（C3Dロード〜フレームループ）は一切変更しない
```

**新規関数 `_run_still_mode(args, cam) -> int`**:

0. `args.distort` の場合、`distortions_to_gsplat(cam["D"])` を try/except で**事前実行**し、
   ValueError なら stderr にメッセージを出して return 1（`load_ply` は CUDA テンソル化まで
   行う重い処理のため、その前に係数を検証する。変換結果はステップ4で再利用してよい）
1. `from render import load_ply, print_ply_summary` → PLY 読み込み・サマリ表示（既存と同形式）
2. `output_dir = args.output_dir if args.output_dir else f"./data/keypoints_{cam['name']}"`
   （既存の既定値ロジックと同一）
3. ログ表示: `背景レンダリング中（near_plane=..., distort=ON|OFF）...`
4. `bgr = render_image(gaussians, cam, args.near_plane, tuple(args.background), distort=args.distort)`
   （`return_depth` は使わない = 既定 False）
5. `os.makedirs(output_dir, exist_ok=True)`
6. `png_path = os.path.join(output_dir, f"still_{cam['name']}.png")` に `cv2.imwrite`。
   失敗（False返却 / cv2.error）は既存フレームループと同じメッセージ形式で stderr 出力し
   return 1。既存ファイルがあれば上書きする（連番 `frame_*.png` には触れない）
7. 保存パスと所要時間を表示して return 0

### エラーハンドリング

| エラー | 検出方法 | 処理 |
|---|---|---|
| FR-002/003/004 の不正な引数組み合わせ | `main()` 冒頭で判定 | `parser.error()`（終了コード2）。C3D/PLY/torch ロード前 |
| TOML `distortions` の長さが 4/5/8 以外 | `distortions_to_gsplat` | ValueError。静止画モードでは `_run_still_mode` の手順0（PLY ロード前）で `--distort` 指定時のみ事前検証して捕捉し、メッセージを stderr に出して return 1 |
| カメラ名不正 | 既存 `select_camera` | 既存のまま（利用可能名一覧を表示して return 1） |
| PNG 保存失敗 | `cv2.imwrite` False / cv2.error | stderr にパス付きエラー、return 1（既存フレームループと同形式） |
| gsplat UT経路の実行時エラー | 例外伝播 | 既存方針どおり上位へ伝播（トレースバック表示）。スパイクで正常動作は確認済み |

### 境界条件

- 動画モード（従来3引数）: 全オプション・全経路とも変更前と完全同一（FR-006）
- 静止画モード + `--distort` なし: ピンホール（`render_image` の distort=False 経路 = 現行と同一）
- `distortions` が全ゼロ + `--distort`: エラーにしない（UT経路 + ゼロ係数。スパイク条件(b)と同じで正常動作を確認済み）
- `--output-dir` 省略: 既存の既定値 `./data/keypoints_<カメラ名>` を使う

## 1.5 状態遷移

該当なし（CLI バッチ処理）。

## 1.6 ファイル・ディレクトリ設計

- 静止画出力: `<output-dir>/still_<カメラ名>.png`（8bit BGR PNG、サイズは TOML の size）
- その他の入出力パス規約は既存と同一。新規ファイルはテスト
  `tests/test_feat024_distort_render.py` のみ。テスト結果は
  `tests/results/feat-024_test_result.txt` に保存する

## 1.7 インターフェース定義

```python
# 変更（引数は末尾に追加。既存呼び出し互換）
def render_image(gaussians: dict, cam: dict, near_plane: float,
                 background=(0.0, 0.0, 0.0), return_depth: bool = False,
                 distort: bool = False)

# 追加
def distortions_to_gsplat(D: np.ndarray) -> tuple[np.ndarray, np.ndarray]
def _run_still_mode(args, cam: dict) -> int
```

- 呼び出し方向: main → _run_still_mode → render_image → distortions_to_gsplat（distort時のみ）
- 既存関数（load_c3d_all_frames, extract_keypoints, project_keypoints, draw_overlay 等）は
  シグネチャ・実装とも変更しない

## 1.8 ログ・デバッグ設計

print ベースの既存方針を踏襲。静止画モードのログは既存の背景レンダリングログに
`distort=ON|OFF` を加えた形式（1.4.3 参照）。

## 設計判断の記録（簡易ADR）

- **採用: 方式A（gsplat ネイティブ 3DGUT）** — スパイク（spike_plan.md / README「スパイク結果」）で
  (b)UT+ゼロ歪み≈(a)ピンホール（画素差0.81/255）、(c)実歪みで靄・劣化なしを確認。
  feat-013 の「UT=黒い靄」は near_plane=0.01 の floater との交絡だったと判断
- **却下: 方式B（ピンホール描画 + cv2.remap 後処理ワープ）** — ネイティブ対応がある以上、
  余白・解像度管理と深度整合の複雑さに見合わない
- **却下: 新規スクリプト（render_calib_check.py）新設** — feat-021/022 と同じ
  「render_keypoints.py にオプトインフラグを積む」拡張様式に統一（ユーザー決定）
- **採用: `--no-keypoints` ⇔ `c3d_path` 省略の双方向必須** — 「キーポイントなしで全フレーム
  ループ」はカメラ固定のため全フレーム同一画像となり無意味。静止画モードとして一本化する
- **採用: `--distort` は静止画モード専用** — 動画モードで歪みを有効にするには
  キーポイント投影（現在 distCoeffs=None）とオクルージョン深度の歪み整合が必要で
  スコープが膨らむ。GT比較という本案件の目的には静止画で十分。将来の別案件とする
- **却下: 静止画モードで `--no-occlusion` 等を警告して無視** — 出力に影響しない指定を
  黙認するとユーザーの誤解を生む。feat-022 の `--no-png` 単独拒否と同じく明示エラーとする

## テスト設計（tests/test_feat024_distort_render.py）

GPU 不要の純ロジックテストのみ（レンダリング品質はステップ6の手動テストで確認）。
既存 `tests/test_feat022_no_png.py`（パース/バリデーション + モック検証）のパターンを踏襲。

| ID | 内容 | 合格基準 |
|---|---|---|
| T-1 | `distortions_to_gsplat` の長さ4/5/8 の変換 | FR-005 の表のとおりの radial/tangential |
| T-2 | `distortions_to_gsplat` の不正長（0/3/6/9）| ValueError、メッセージに実際の長さを含む |
| T-3 | `c3d_path` 省略 + `--no-keypoints` なし | 終了コード2、メッセージ確認 |
| T-4 | `c3d_path` 指定 + `--no-keypoints` あり | 終了コード2、メッセージ確認 |
| T-5 | 静止画モード + 禁止7オプション各指定 | それぞれ終了コード2、オプション名入りメッセージ |
| T-6 | `--distort` + 動画モード（c3d_path あり） | 終了コード2、メッセージ確認 |
| T-7 | `render_image` を distort 引数なしで呼ぶ既存形式の互換 | rasterization に渡される引数が変更前と同一（with_ut=False, packed=True, 歪み係数なし）。**検証方法**: root pytest 環境に torch/gsplat が無いため、`sys.modules` にフェイクの `torch`/`gsplat` モジュール（rasterization は kwargs を記録するスタブ、torch.tensor 等は最小実装）を注入してから `render_image` を呼び、記録された kwargs を検証する（`render_image` は関数内 import のため注入が有効） |
| T-8 | `render_image(distort=True)` の rasterization 引数 | T-7 と同じフェイク注入方式で、with_ut=True, with_eval3d=True, packed=False と radial/tangential の値（distortions_to_gsplat の結果と一致）を確認 |
| T-9 | 静止画モードの出力ファイル名 | `_run_still_mode` をモック環境で実行し `still_<カメラ名>.png` が出力されること |
| T-10 | 既存テストの回帰なし | feat-015/016/017/021/022 の既存テストが無変更で全件パス |

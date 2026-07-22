# feat-023 機能設計書: estimate_camera_params.py 接線歪みゼロ固定オプション（--zero-tangent）

## 1.1 対応要求マッピング

| 要求ID | 対応セクション |
|---|---|
| FR-001 | 1.4.1（CLI）、1.4.2（投影関数）、1.4.3（推定分岐） |
| FR-002 | 1.4.2、1.4.3 |
| FR-003 | 1.4.1 |
| FR-004 | 1.4.4（出力） |
| FR-005 | 1.4.3（最小点数） |
| FR-006 | 1.4.3（ログ） |

## 1.2 システム構成

変更対象は `phase0/estimate_camera_params.py` の1ファイルのみ（+ テスト・ドキュメント）。

```
phase0/estimate_camera_params.py   # 本体（CLI追加、投影関数追加、推定分岐追加）
tests/test_feat023_zero_tangent.py # 新規テスト
docs/BACKLOG.md, docs/CHANGELOG.md, CLAUDE.md  # ドキュメント更新
```

依存関係は既存のまま: `estimate_camera_params.py` → `common.py`（load_yaml_simple, load_points_3d, load_points_2d, match_points）。`common.py` は変更しない。

## 1.3 技術スタック

- Python 3.10 / uv（phase0 環境）
- NumPy、SciPy `least_squares(method='lm')`、OpenCV `cv2.projectPoints` / `cv2.solvePnP`
- 新規ライブラリ追加なし。`cv2.projectPoints` に `dist = [k1, k2, 0, 0]`（または `[k1, k2, 0, 0, k3]`）を渡すことで p1 = p2 = 0 を実現する（distCoeffs の並びは OpenCV 仕様で k1, k2, p1, p2, k3）。

## 1.4 各機能の詳細設計

### 1.4.1 CLI（FR-001, FR-003）

`main()` の argparse に追加:

```python
parser.add_argument('--zero-tangent', action='store_true',
                    help='接線歪み係数 p1, p2 を 0 に固定する（放射歪みのみ推定）')
```

バリデーション（`main()` 内、`args = parser.parse_args()` の直後のバリデーション群に追加）:

1. `args.zero_tangent and args.wide` → `print("エラー: --zero-tangent と --wide は併用できません")` して `return 1`。判定は既存の `--intrinsic-toml` 存在確認より前に置く（重い処理の前に拒否）。
2. `args.zero_tangent and args.intrinsic_toml` → 既存の ignored リスト（`--k3`/`--wide`/`--fix-center` の警告）に `'--zero-tangent'` を追加して同じ警告文で無視する。

`run_estimation` のシグネチャ変更:

```python
def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool,
                   intrinsic_toml: str = None, output_path: str = None,
                   zero_tangent: bool = False):
```

`zero_tangent` は**既存引数の末尾**に追加する（既存の位置引数呼び出し・既存テストとの互換性維持のため。`intrinsic_toml` より前に挿入してはならない）。`main()` からは `zero_tangent=args.zero_tangent` とキーワード引数で渡す。K既知モード分岐（`if intrinsic_toml:`）は `zero_tangent` を参照しない（main で警告済み・無視）。

### 1.4.2 投影関数（FR-001, FR-002）

既存の投影関数群と同じパターンで4関数を追加する。関数名の `dist2`/`dist3` は「推定する歪み係数の個数」（k1,k2 / k1,k2,k3）を表す。

```python
def project_dist2_fix_center(params, points_3d, cx, cy):
    """放射2係数（k1,k2）、接線ゼロ、主点固定（10変数）"""
    fx, fy = params[0:2]
    k1, k2 = params[2:4]
    rvec = params[4:7]
    tvec = params[7:10]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, 0.0, 0.0])
    # 以下 cv2.projectPoints は既存関数と同一

def project_dist3_fix_center(params, points_3d, cx, cy):
    """放射3係数（k1,k2,k3）、接線ゼロ、主点固定（11変数）"""
    # params: fx, fy, k1, k2, k3, rvec(3), tvec(3)
    # dist = np.array([k1, k2, 0.0, 0.0, k3])

def project_dist2(params, points_3d):
    """放射2係数（k1,k2）、接線ゼロ（12変数）"""
    # params: fx, fy, cx, cy, k1, k2, rvec(3), tvec(3)
    # dist = np.array([k1, k2, 0.0, 0.0])

def project_dist3(params, points_3d):
    """放射3係数（k1,k2,k3）、接線ゼロ（13変数）"""
    # params: fx, fy, cx, cy, k1, k2, k3, rvec(3), tvec(3)
    # dist = np.array([k1, k2, 0.0, 0.0, k3])
```

（コード例は意図の伝達が目的。実装時は既存の `project_dist4_fix_center` 等の書式に合わせる。）

### 1.4.3 推定分岐・最小点数・ログ（FR-002, FR-005, FR-006）

**最小点数定数**（ファイル冒頭の定数群に追加。命名は既存に合わせる）:

```python
MIN_POINTS_DIST2 = 13            # 放射2係数・接線ゼロ: 12変数
MIN_POINTS_DIST2_FIXCENTER = 10  # 放射2係数・接線ゼロ+主点固定: 10変数
MIN_POINTS_DIST3 = 14            # 放射3係数・接線ゼロ: 13変数
MIN_POINTS_DIST3_FIXCENTER = 11  # 放射3係数・接線ゼロ+主点固定: 11変数
```

（既存の DIST4/DIST4_FIXCENTER と同じ規則: 主点固定は変数の数と同数、主点推定は変数の数+1。）

**min_points 決定**（`run_estimation` 内の既存 if 連鎖を拡張。zero_tangent は wide と排他なので wide 分岐の後に置く）:

```python
if use_wide:
    min_points = MIN_POINTS_DIST8_FIXCENTER if fix_center else MIN_POINTS_DIST8
elif zero_tangent and use_k3:
    min_points = MIN_POINTS_DIST3_FIXCENTER if fix_center else MIN_POINTS_DIST3
elif zero_tangent:
    min_points = MIN_POINTS_DIST2_FIXCENTER if fix_center else MIN_POINTS_DIST2
elif use_k3:
    ...（既存のまま）
```

**ヘッダー表示**（既存の「歪みモデル:」行）:

- `zero_tangent` かつ `use_k3` → `3係数（放射のみ）`
- `zero_tangent` かつ not `use_k3` → `2係数（放射のみ）`
- それ以外は既存のまま

**モード決定ログ**（既存の `mode_str` 組み立てに追加）:

- 例: `主点固定 + 接線歪みゼロ + k3なし（10変数）` / `主点推定 + 接線歪みゼロ + k3あり（13変数）`
- 分岐ごとの完全な文字列は下表のとおり:

| 組み合わせ | mode_str |
|---|---|
| `--zero-tangent --fix-center` | `主点固定 + 接線歪みゼロ + k3なし（10変数）` |
| `--zero-tangent --fix-center --k3` | `主点固定 + 接線歪みゼロ + k3あり（11変数）` |
| `--zero-tangent` | `主点推定 + 接線歪みゼロ + k3なし（12変数）` |
| `--zero-tangent --k3` | `主点推定 + 接線歪みゼロ + k3あり（13変数）` |

**最適化分岐**（`run_estimation` の fix_center / else ブロック内。既存分岐の先頭に zero_tangent 分岐を追加する。`estimate_distortion` が False の場合は既存の「歪みなし」分岐に落ちる=既存動作のまま）:

主点固定側（fix_center=True）に追加する2分岐:

```python
if use_wide and estimate_distortion:
    ...（既存）
elif estimate_distortion and zero_tangent and use_k3:
    # params_init = [fx, fy, 0,0,0(k1,k2,k3), rvec(3), tvec(3)]  … 11変数
    # residual = make_residual_fix_center(project_dist3_fix_center, cx_fixed, cy_fixed)
    # 展開: fx,fy=params[0:2]; k1,k2,k3=params[2:5]; p1=p2=0.0; rvec=params[5:8]; tvec=params[8:11]
elif estimate_distortion and zero_tangent:
    # params_init = [fx, fy, 0,0(k1,k2), rvec(3), tvec(3)]  … 10変数
    # 展開: fx,fy=params[0:2]; k1,k2=params[2:4]; p1=p2=k3=0.0; rvec=params[4:7]; tvec=params[7:10]
elif estimate_distortion and use_k3:
    ...（既存）
```

主点推定側（else）も同型で `project_dist3` / `project_dist2` を使い、cx, cy を params[2:4] に含める（13変数 / 12変数）。

いずれの分岐でも `p1_opt = p2_opt = 0.0`（zero_tangent かつ k3 なしのときは `k3_opt = 0.0` も）を代入して既存の結果表示・出力コードをそのまま通す。

### 1.4.4 出力（FR-004）

出力コードは変更しない。zero_tangent 分岐で `p1_opt = p2_opt = 0.0` を設定するため:

- 歪み係数表示: `p1: 0.000000` / `p2: 0.000000` と表示される（既存フォーマット）
- TOML: `distortions = [k1, k2, 0.0, 0.0]`（`--k3` 時は `[k1, k2, 0.0, 0.0, k3]`）— 配列長は既存と同じ 4 / 5
- CSV: 既存の列構成のまま p1, p2 列に 0.0

これにより `convert_toml_to_csv.py` および K既知モードの `--intrinsic-toml` 読み込みとの互換性を維持する。

### エラーハンドリング

| エラー | 検出方法 | 処理 |
|---|---|---|
| `--zero-tangent --wide` 併用 | `main()` でフラグ判定 | エラーメッセージ表示、`return 1`。データ読み込み前に判定 |
| `--zero-tangent --intrinsic-toml` 併用 | `main()` でフラグ判定 | 警告表示（既存 ignored リストに追加）、`--zero-tangent` を無視して続行 |
| solvePnP 初期値推定の失敗 | 既存処理のまま | 既存処理のまま（エラー表示、`return 1`） |
| least_squares の失敗 | 既存処理と同様（例外は上位へ伝播） | 既存処理と同様 |

### 境界条件

- 基準点数が最小点数未満: `estimate_distortion = False` となり既存の「歪みなし」分岐で推定する（`--zero-tangent` の有無で結果は同一）。出力も既存どおり4係数レイアウト（`--k3` 指定でも `estimate_distortion=False` なら4係数に落ちる既存挙動を維持。FR-004 の注記参照）。
- 基準点数が最小点数ちょうど: 歪み係数を推定する（FR-005 の表のとおり）。
- 基準点 0 点・4 点未満: 既存の solvePnP 失敗処理に従う（変更なし）。

## 1.5 状態遷移

該当なし（CLI バッチ処理）。

## 1.6 ファイル・ディレクトリ設計

入出力ファイルのパス規約・形式は既存と同一。新規ファイルはテスト `tests/test_feat023_zero_tangent.py` のみ。テスト結果は `tests/results/feat-023_test_result.txt` に保存する。

## 1.7 インターフェース定義

```python
# 変更（zero_tangent は末尾に追加。既存引数の順序は不変）
def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool,
                   intrinsic_toml: str = None, output_path: str = None,
                   zero_tangent: bool = False) -> int

# 追加（すべて module-level 関数、戻り値 np.ndarray shape=(N,2)）
def project_dist2_fix_center(params, points_3d, cx, cy)   # params: shape=(10,)
def project_dist3_fix_center(params, points_3d, cx, cy)   # params: shape=(11,)
def project_dist2(params, points_3d)                      # params: shape=(12,)
def project_dist3(params, points_3d)                      # params: shape=(13,)
```

`zero_tangent` にデフォルト値 `False` を持たせ、既存テスト（`run_estimation` を直接呼ぶもの）を壊さない。呼び出し方向は既存のまま（main → run_estimation → 投影関数）。

## 1.8 ログ・デバッグ設計

print ベースの既存方針を踏襲（ログレベル区分なし）。追加されるログは 1.4.3 のヘッダー表示・mode_str のみ。

## 設計判断の記録（簡易ADR）

- **採用**: p1, p2 を変数から除外した専用投影関数を追加する方式。既存コードの「モードごとに投影関数 + 分岐」というパターンに一致し、`--zero-tangent` なしの経路に一切触れないため回帰リスクがゼロ。
- **却下: dist 配列をマスクで動的構成する共通化リファクタ** — 既存 8 分岐すべてに手が入り回帰リスクが大きい。本プロジェクトの「小さく作る」方針に反する。
- **却下: bounds で |p1|,|p2| を制限（method='trf' 化）** — 全モードの最適化挙動が変わる。境界張り付き解も不自然なまま。
- **却下: 正則化項の追加** — λ 調整の恣意性が入り、RMSE の意味が変わる。
- **却下: `--wide` との併用サポート** — 8係数モデルの接線ゼロ版は現状ユースケースがなく、スコープ外。必要になったら別案件とする。

## テスト設計（tests/test_feat023_zero_tangent.py）

既存 `test_estimate_camera_params_wide.py` と同様に合成データ（真値 K, dist, R, t から `cv2.projectPoints` で 2D を生成）で検証する。

| ID | 内容 | 合格基準 |
|---|---|---|
| T-1 | `project_dist2` / `project_dist2_fix_center` / `project_dist3` / `project_dist3_fix_center` に真値パラメータを与えた投影が、`cv2.projectPoints`（dist=[k1,k2,0,0(,k3)]）の結果と一致 | RMSE < 1e-6 px |
| T-2 | 接線歪みゼロの合成データ（dist=[k1,k2,0,0]）に対し `--zero-tangent --fix-center` 相当の `run_estimation` が真値を復元 | fx, fy 相対誤差 < 1%、k1, k2 絶対誤差 < 0.01、出力 p1 = p2 = 0.0 |
| T-2b | 同合成データに対し `--zero-tangent` 単独（主点推定）相当の `run_estimation` が真値を復元 | fx, fy, cx, cy 相対誤差 < 1%、k1, k2 絶対誤差 < 0.01、出力 p1 = p2 = 0.0 |
| T-2c | 接線歪みゼロ+k3の合成データ（dist=[k1,k2,0,0,k3]）に対し `--zero-tangent --k3 --fix-center` 相当の `run_estimation` が真値を復元 | fx, fy 相対誤差 < 1%、k1, k2, k3 絶対誤差 < 0.01、出力 p1 = p2 = 0.0 |
| T-2d | 同k3合成データに対し `--zero-tangent --k3`（主点推定）相当の `run_estimation` が真値を復元 | fx, fy, cx, cy 相対誤差 < 1%、k1, k2, k3 絶対誤差 < 0.01、出力 p1 = p2 = 0.0 |
| T-3 | `--zero-tangent --wide` 併用で終了コード 1 とエラーメッセージ | メッセージ「--zero-tangent と --wide は併用できません」を含む |
| T-4 | `--zero-tangent --intrinsic-toml` 併用で警告表示され K既知モードが実行される | 警告文に `--zero-tangent` を含む |
| T-5 | 最小点数境界（主点固定）: zero_tangent で 10 点なら歪み推定、9 点なら 0 固定 | 標準出力の判定ログで確認 |
| T-5b | 最小点数境界（主点推定）: zero_tangent で 13 点なら歪み推定、12 点なら 0 固定 | 標準出力の判定ログで確認 |
| T-5c | 最小点数境界（主点固定+k3）: zero_tangent + use_k3 で 11 点なら歪み推定、10 点なら 0 固定 | 標準出力の判定ログで確認 |
| T-5d | 最小点数境界（主点推定+k3）: zero_tangent + use_k3 で 14 点なら歪み推定、13 点なら 0 固定 | 標準出力の判定ログで確認 |
| T-6 | `--zero-tangent` なしの既存経路の回帰なし | 既存テスト（extrinsic / wide）が全件パス |

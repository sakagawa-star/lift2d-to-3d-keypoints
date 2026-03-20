# feat-002 機能設計書: estimate_camera_params.py 広角レンズ対応

## 概要

`phase0/estimate_camera_params.py` に `--wide` オプションを追加し、広角レンズ用8係数歪みモデルを使用可能にする。`estimate_camera_params_wide.py` から8係数関連のコードを移植し、CSV出力は既存コードのスタイルに合わせて新規追加する。

## ファイル構成

### 変更

- `phase0/estimate_camera_params.py` — `--wide` オプションと8係数歪みモデルを追加

### 変更しないファイル

- `phase0/estimate_camera_params_wide.py`（参考元。変更しない）
- `phase0/estimate_extrinsic.py`
- `phase0/phase0_verification.py`
- `phase0/convert_toml_to_csv.py`

## 変更内容の詳細

### 1. 定数の追加

既存の定数の後に追加する:

```python
MIN_POINTS_DIST8 = 20            # 8係数: 18変数
MIN_POINTS_DIST8_FIXCENTER = 18  # 8係数+主点固定: 16変数
```

### 2. 投影関数の追加

既存の `project_dist5_fix_center` の後、既存の `project_dist5` の後にそれぞれ追加する。

#### `project_dist8_fix_center(params, points_3d, cx, cy)`

```python
def project_dist8_fix_center(params, points_3d, cx, cy):
    """8係数歪み（広角用）、主点固定（16変数）"""
    fx, fy = params[0:2]
    k1, k2, p1, p2, k3, k4, k5, k6 = params[2:10]
    rvec = params[10:13]
    tvec = params[13:16]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2, k3, k4, k5, k6])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)
```

#### `project_dist8(params, points_3d)`

```python
def project_dist8(params, points_3d):
    """8係数歪み（広角用）（18変数）"""
    fx, fy, cx, cy = params[0:4]
    k1, k2, p1, p2, k3, k4, k5, k6 = params[4:12]
    rvec = params[12:15]
    tvec = params[15:18]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2, k3, k4, k5, k6])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)
```

### 3. `run_estimation` 関数のシグネチャ変更

```python
# 変更前
def run_estimation(config_path: str, use_k3: bool, fix_center: bool):

# 変更後
def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool):
```

### 4. 必要点数の決定ロジック変更

既存の `min_points` 決定ロジックに `use_wide` の分岐を追加する:

```python
if use_wide:
    min_points = MIN_POINTS_DIST8_FIXCENTER if fix_center else MIN_POINTS_DIST8
elif use_k3:
    # 既存のまま
```

### 5. 初期値推定のエラーハンドリング追加と戻り値の変更

`run_estimation` 関数に戻り値を追加する。正常終了時は `return 0`、エラー時は `return 1`。

```python
# 変更前
_, rvec_init, tvec_init = cv2.solvePnP(
    points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
)

# 変更後
success, rvec_init, tvec_init = cv2.solvePnP(
    points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
)
if not success:
    print("エラー: solvePnP による初期値推定が失敗しました")
    return 1
```

関数の末尾（TOML/CSV出力の後）に `return 0` を追加する。

### 6. 最適化分岐の追加

主点固定モード・主点推定モードの両方で、既存の `if estimate_distortion and use_k3:` 分岐の**前**に `use_wide` の分岐を追加する。

**フォールバック動作**: `use_wide=True` かつ `estimate_distortion=False`（点数不足）の場合、`use_wide` の分岐にマッチせず、既存の歪みなし分岐（`else`）がそのまま適用される。追加の処理は不要。

**変数初期化**: 既存の歪みなし分岐・4係数分岐・5係数分岐では `k4_opt, k5_opt, k6_opt` が定義されないが、後続の歪み係数表示・TOML出力・CSV出力では `use_wide and estimate_distortion` の条件でガードされるため参照されない。よって既存分岐への `k4_opt = k5_opt = k6_opt = 0.0` の追加は不要。

#### 主点固定モードでの追加（`if fix_center:` ブロック内の先頭）

```python
if use_wide and estimate_distortion:
    # 8係数歪み、主点固定（16変数）
    params_init = np.array([
        fx_init, fy_init,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2, k3, k4, k5, k6 (OpenCV順)
        rvec_init[0], rvec_init[1], rvec_init[2],
        tvec_init[0], tvec_init[1], tvec_init[2]
    ])

    residual_func = make_residual_fix_center(project_dist8_fix_center, cx_fixed, cy_fixed)
    result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)

    params_opt = result.x
    fx_opt, fy_opt = params_opt[0:2]
    cx_opt, cy_opt = cx_fixed, cy_fixed
    k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = params_opt[2], params_opt[3], params_opt[4], params_opt[5], params_opt[6]
    k4_opt, k5_opt, k6_opt = params_opt[7], params_opt[8], params_opt[9]
    rvec_opt = params_opt[10:13]
    tvec_opt = params_opt[13:16]

    projected_opt = project_dist8_fix_center(params_opt, points_3d, cx_fixed, cy_fixed)

elif estimate_distortion and use_k3:
    # 既存コードそのまま
```

#### 主点推定モードでの追加（`else:` ブロック内の先頭）

```python
if use_wide and estimate_distortion:
    # 8係数歪み（18変数）
    params_init = np.array([
        fx_init, fy_init, cx_init, cy_init,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2, k3, k4, k5, k6 (OpenCV順)
        rvec_init[0], rvec_init[1], rvec_init[2],
        tvec_init[0], tvec_init[1], tvec_init[2]
    ])

    residual_func = make_residual(project_dist8)
    result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)

    params_opt = result.x
    fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
    k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = params_opt[4], params_opt[5], params_opt[6], params_opt[7], params_opt[8]
    k4_opt, k5_opt, k6_opt = params_opt[9], params_opt[10], params_opt[11]
    rvec_opt = params_opt[12:15]
    tvec_opt = params_opt[15:18]

    projected_opt = project_dist8(params_opt, points_3d)

elif estimate_distortion and use_k3:
    # 既存コードそのまま
```

### 7. モード決定ログの追加

既存のモードログに `use_wide` の分岐を追加する:

```python
if use_wide:
    mode_str = ("主点固定" if fix_center else "主点推定") + " + 8係数広角"
    if fix_center:
        mode_str += "（16変数）"
    else:
        mode_str += "（18変数）"
elif use_k3:
    # 既存のまま
```

### 8. 歪み係数表示の変更

既存の歪み係数表示ブロックに `use_wide` の分岐を追加する:

```python
if estimate_distortion:
    print(f"k1: {k1_opt:.6f}")
    print(f"k2: {k2_opt:.6f}")
    print(f"p1: {p1_opt:.6f}")
    print(f"p2: {p2_opt:.6f}")
    if use_k3 or use_wide:
        print(f"k3: {k3_opt:.6f}")
    if use_wide:
        print(f"k4: {k4_opt:.6f}")
        print(f"k5: {k5_opt:.6f}")
        print(f"k6: {k6_opt:.6f}")
```

### 9. TOML出力の変更

既存の `dist_str` 生成ロジックに `use_wide` の分岐を追加する:

```python
if use_wide and estimate_distortion:
    print("注意: distortions が8係数（k1,k2,p1,p2,k3,k4,k5,k6）です。")
    print("      convert_toml_to_csv.py は先頭4係数のみ読み込むため、k3-k6は無視されます。")
    dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}, {k3_opt}, {k4_opt}, {k5_opt}, {k6_opt}]"
elif use_k3 and estimate_distortion:
    dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}, {k3_opt}]"
else:
    dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}]"
```

### 10. CSV出力の変更

既存のCSV出力ロジックに `use_wide` の分岐を追加する:

```python
if use_wide and estimate_distortion:
    print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6,r1,r2,r3,t1,t2,t3")
    print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},"
          f"{k1_opt},{k2_opt},{p1_opt},{p2_opt},{k3_opt},{k4_opt},{k5_opt},{k6_opt},"
          f"{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")
elif use_k3 and estimate_distortion:
    # 既存のまま
```

### 11. `main()` 関数の変更

```python
# --wide 引数の追加
parser.add_argument('--wide', action='store_true',
                    help='広角レンズ用8係数モデルを使用')

# --wide が指定されたら --k3 は無視
use_k3 = args.k3 and not args.wide

# run_estimation の戻り値を反映（変更前は戻り値を無視していた）
result = run_estimation(args.config, use_k3, args.wide, args.fix_center)
if result:
    return result
return 0
```

### 12. docstring の更新

スクリプト冒頭のdocstringを以下のように更新する:

```python
"""
カメラ パラメータ推定スクリプト（主点固定・広角レンズ対応版）

オプション:
    --k3          : k3（6次放射歪み）も推定する
    --wide        : 広角レンズ用8係数モデルを使用
    --fix-center  : 主点(cx, cy)を画像中心に固定する

使い方:
    python estimate_camera_params.py config.yaml                    # 標準（4係数）
    python estimate_camera_params.py config.yaml --fix-center       # 主点固定
    python estimate_camera_params.py config.yaml --fix-center --k3  # 主点固定 + k3
    python estimate_camera_params.py config.yaml --wide             # 広角（8係数）
    python estimate_camera_params.py config.yaml --wide --fix-center # 広角 + 主点固定

レンズと推奨オプション:
    標準レンズ（~60°）   : （オプションなし）
    広角レンズ（60-100°）: --wide
    魚眼レンズ（>120°）  : 別途fisheyeモデルが必要
"""
```

**注**: 歪み係数は既存コードのスタイルに合わせて個別変数（`k1_opt, k2_opt, ...`）で管理する。参考実装の `dist_coeffs` リスト方式は採用しない。

## 変更しない箇所（リグレッション防止）

以下は一切変更しない:

- `load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` 関数
- 既存の投影関数（`project_no_dist`, `project_dist4`, `project_dist5` とそれぞれの `_fix_center` 版）
- 既存の残差関数（`make_residual`, `make_residual_fix_center`）
- 再投影誤差の評価閾値（< 5px 優秀, < 10px 良好, < 20px 許容範囲, >= 20px 要確認）。`--wide` 使用時も同じ閾値を使用する
- 各点の再投影誤差の閾値（< 10px で ✓、>= 10px で ✗ の2段階）。参考実装の3段階評価（✓/△/✗）は採用しない

## スコープ外の既知事項

- マッチング点数が0点の場合のエラーハンドリングは既存コードにも存在しない問題であり、本案件のスコープ外とする

## テスト設計

### テストファイル

`tests/test_estimate_camera_params_wide.py`

### テスト方針

合成データ（既知のK, 歪み係数, R, tから `cv2.projectPoints` で2D点を生成）を使ってテストする。

### テストケース

#### T-1: 8係数推定（主点推定）

- 合成データ（8係数歪み、20点以上）で全パラメータが推定できること
- 再投影誤差が十分小さいこと（< 1.0 px）

#### T-2: 8係数推定（主点固定）

- 合成データ（8係数歪み、18点以上）で `--fix-center` と同等の動作をすること
- 再投影誤差が十分小さいこと（< 1.0 px）

#### T-3: 8係数で点数不足時のフォールバック

- 点数が18点（主点固定）/ 20点（主点推定）未満の場合、歪み係数が0に固定されること

#### T-4: `--wide --k3` の排他制御

- `use_wide=True, use_k3=True` を渡した場合、main()で `use_k3` が `False` に上書きされること

#### T-5: リグレッションテスト（既存動作）

- `use_wide=False` の場合に既存の `run_estimation` と同じ結果が得られること
- 4係数モード、5係数モード、歪みなしモードそれぞれで再投影誤差が < 1.0 px であること

#### T-6: solvePnP 初期値推定失敗

- `solvePnP` が失敗した場合にエラー終了すること（戻り値1）

#### T-7: TOML/CSV出力フォーマット

- 8係数モードのTOML出力で `distortions` が8要素であること
- 8係数モードのCSV出力ヘッダーに `k3,k4,k5,k6` が含まれること

### 合成データ生成方法

```python
# 8係数歪みの既知パラメータ
K = np.array([[1177.34, 0, 956.70], [0, 1177.82, 495.77], [0, 0, 1]])
dist_8 = np.array([-0.05, 0.09, -0.005, -0.003, 0.01, 0.02, -0.01, 0.005])
rvec_true = np.array([0.1, -0.2, 0.3])
tvec_true = np.array([1.0, -0.5, 5.0])

# 3D基準点（20点以上、カメラ前方に分散配置）
points_3d = np.array([...], dtype=np.float64)

# 2D投影点
points_2d, _ = cv2.projectPoints(points_3d, rvec_true, tvec_true, K, dist_8)
points_2d = points_2d.reshape(-1, 2)
```

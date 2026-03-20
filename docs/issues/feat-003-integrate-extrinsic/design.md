# feat-003 機能設計書: estimate_extrinsic.py を estimate_camera_params.py に統合（Stage 2）

## 概要

`estimate_extrinsic.py` の K既知推定機能を `estimate_camera_params.py` に統合する。重複関数を `common.py` に切り出し、統合後に `estimate_extrinsic.py` を削除する。

## ファイル構成

### 新規作成

- `phase0/common.py` — 共通関数モジュール

### 変更

- `phase0/estimate_camera_params.py` — `--intrinsic-toml` オプションとK既知モードを追加、共通関数をインポートに変更
- `phase0/estimate_extrinsic.py` — 共通関数をインポートに変更（削除前の最終状態）

### 削除

- `phase0/estimate_extrinsic.py` — 統合完了後に削除
- `tests/test_estimate_extrinsic.py` — K既知モードテストに置き換え

### ドキュメント更新

- `docs/TECH_STACK.md` — `tomli` ライブラリと `common.py`, `estimate_camera_params.py` の使用箇所を追加

### 変更しないファイル

- `phase0/phase0_verification.py`
- `phase0/convert_toml_to_csv.py`

## 変更内容の詳細

### 1. 共通モジュール `phase0/common.py` の作成

以下の関数を `estimate_camera_params.py` から切り出す（実装は既存コードと同一）:

```python
"""共通関数モジュール"""

import csv

import numpy as np


def load_yaml_simple(yaml_path: str) -> dict:
    """簡易YAMLパーサー"""
    config = {}
    with open(yaml_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                key, value = line.split(':', 1)
                config[key.strip()] = value.strip()
    return config


def load_points_3d(csv_path: str) -> dict:
    """3D座標を読み込む"""
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row['ObjectName']
            points[name] = [float(row['X']), float(row['Y']), float(row['Z'])]
    return points


def load_points_2d(csv_path: str, camera_name: str) -> dict:
    """2D座標を読み込む"""
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['camera_name'] == camera_name:
                name = row['ObjectName']
                points[name] = [float(row['X']), float(row['Y'])]
    return points


def match_points(points_3d: dict, points_2d: dict):
    """3Dと2Dの点をマッチング"""
    matched_3d = []
    matched_2d = []
    names = []

    for name in points_3d:
        if name in points_2d:
            matched_3d.append(points_3d[name])
            matched_2d.append(points_2d[name])
            names.append(name)

    return (
        np.array(matched_3d, dtype=np.float64),
        np.array(matched_2d, dtype=np.float64),
        names
    )
```

### 2. `estimate_camera_params.py` の変更

#### 2-1. インポートの変更

重複関数を削除し、`common.py` からインポートする:

```python
import tomli
from common import load_yaml_simple, load_points_3d, load_points_2d, match_points
```

`load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` の関数定義を削除する。

#### 2-2. K既知モード用関数の追加

`estimate_extrinsic.py` から以下の関数を移植する:

- `load_intrinsic_toml(toml_path: str, camera_name: str) -> dict` — そのまま移植
- `estimate_extrinsic(points_3d, points_2d, K, dist, point_names) -> dict` — 移植時に `sys.exit(1)` を `raise SystemExit(1)` に置き換えない。代わりにエラー時は `print` + `return None` に変更する（後述）
- `compute_reprojection_errors(points_3d, points_2d, K, dist, rvec, tvec) -> tuple` — そのまま移植

**エラー処理の方針**: 移植元 `estimate_extrinsic.py` では `sys.exit(1)` でエラー終了しているが、統合先 `estimate_camera_params.py` では `return 1` パターンを使用している（feat-002で追加）。統合時はコードスタイルを統一し、以下のように変更する:

- `estimate_extrinsic()` 関数: エラー時に `sys.exit(1)` の代わりにエラーメッセージを `print` して `return None` を返す
- `_run_extrinsic_estimation()` 関数: `estimate_extrinsic()` が `None` を返した場合に `return 1` する。正常終了時は `return 0`
- `run_estimation()` 関数: `_run_extrinsic_estimation()` の戻り値（0 or 1）をそのまま `return` する

これにより `sys.exit(1)` がコードベースから排除され、テストが容易になる。

**4〜5点時の推定手法**: `estimate_extrinsic()` 内で4〜5点の場合は `cv2.solvePnP(flags=cv2.SOLVEPNP_SQPNP)` を使用する（`SOLVEPNP_ITERATIVE` は最低6点必要なため）。ロードマップには `ITERATIVE` と記載されているが、実装時に `SQPNP` に変更済み（feat-001で対応）。

#### 2-3. `run_estimation` 関数の変更

シグネチャを変更し、`intrinsic_toml` パラメータを追加する:

```python
def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool, intrinsic_toml: str = None):
```

関数の先頭（データ読み込みの前）に K既知モードの分岐を追加する:

```python
if intrinsic_toml:
    return _run_extrinsic_estimation(config_path, intrinsic_toml)
```

#### 2-4. K既知モードの処理関数 `_run_extrinsic_estimation`

`estimate_extrinsic.py` の `run_estimation` を `_run_extrinsic_estimation` として移植する。処理フローは `estimate_extrinsic.py` と同一:

1. `load_yaml_simple` で config.yaml を読み込み
2. CSVファイルのパスは config.yaml の親ディレクトリからの相対パスとして解決
3. `load_intrinsic_toml` で TOML を読み込み → K, dist, image_size を取得
4. `load_points_3d`, `load_points_2d` でCSVを読み込み
5. `match_points` で3D-2D点をマッチング
6. 情報表示（カメラ名、内部パラメータ、基準点数等。`estimate_extrinsic.py` の表示をそのまま移植）
7. `estimate_extrinsic` でR, t推定。戻り値が `None` の場合は `return 1`（エラー終了）
8. RANSAC結果表示（inlier/outlier の分類。`estimate_extrinsic.py` の表示をそのまま移植）
9. `compute_reprojection_errors` で再投影誤差を計算・表示（全点対象）
10. TOML形式の結果を出力（`_print_toml_output` 関数を使用）
11. `return 0`（正常終了）

#### 2-5. `main()` 関数の変更

```python
# --intrinsic-toml 引数の追加
parser.add_argument('--intrinsic-toml', default=None,
                    help='内部パラメータTOMLファイル（指定時はK既知モード）')

# --intrinsic-toml 指定時の警告
if args.intrinsic_toml:
    ignored = []
    if args.k3:
        ignored.append('--k3')
    if args.wide:
        ignored.append('--wide')
    if args.fix_center:
        ignored.append('--fix-center')
    if ignored:
        print(f"警告: --intrinsic-toml 指定時は {', '.join(ignored)} は無視されます")

# TOMLファイルの存在確認
if args.intrinsic_toml and not Path(args.intrinsic_toml).exists():
    print(f"エラー: TOMLファイルが見つかりません: {args.intrinsic_toml}")
    return 1

use_k3 = args.k3 and not args.wide

result = run_estimation(args.config, use_k3, args.wide, args.fix_center,
                        intrinsic_toml=args.intrinsic_toml)
if result:
    return result
return 0
```

#### 2-6. docstring の更新

```python
"""
カメラ パラメータ推定スクリプト（主点固定・広角レンズ・K既知対応版）

オプション:
    --k3              : k3（6次放射歪み）も推定する
    --wide            : 広角レンズ用8係数モデルを使用
    --fix-center      : 主点(cx, cy)を画像中心に固定する
    --intrinsic-toml  : 内部パラメータTOMLファイル（K既知モード）

使い方:
    python estimate_camera_params.py config.yaml                    # 標準（4係数）
    python estimate_camera_params.py config.yaml --fix-center       # 主点固定
    python estimate_camera_params.py config.yaml --fix-center --k3  # 主点固定 + k3
    python estimate_camera_params.py config.yaml --wide             # 広角（8係数）
    python estimate_camera_params.py config.yaml --wide --fix-center # 広角 + 主点固定
    python estimate_camera_params.py config.yaml --intrinsic-toml intrinsic.toml  # K既知

レンズと推奨オプション:
    標準レンズ（~60°）   : （オプションなし）
    広角レンズ（60-100°）: --wide
    魚眼レンズ（>120°）  : 別途fisheyeモデルが必要
"""
```

### 3. `estimate_extrinsic.py` の変更と削除

実装順序:
1. `common.py` を作成する
2. `estimate_camera_params.py` の共通関数を削除し `common.py` からインポートに変更する。K既知モード関数を追加する
3. `estimate_camera_params.py` の全テスト（既存 + K既知モード）を実行し通過を確認する
4. `estimate_extrinsic.py` を削除する
5. `tests/test_estimate_extrinsic.py` を削除する

### 4. TOML出力関数 `_print_toml_output`

`estimate_extrinsic.py` の `print_toml_output` を `_run_extrinsic_estimation` 内のローカル出力として移植する。既存の K未知モードのTOML出力と出力位置を揃えるため、`run_estimation` 内の同じ位置で出力する。

```python
def _print_toml_output(camera_name, image_width, image_height, K, dist, rvec, tvec):
    """K既知モード用のTOML出力"""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    dist_list = dist.tolist()
    dist_str = f"[{', '.join(str(v) for v in dist_list)}]"

    print(f"""
[{camera_name}]
name = "{camera_name}"
size = [{image_width}.0, {image_height}.0]
matrix = [[{fx}, 0.0, {cx}], [0.0, {fy}, {cy}], [0.0, 0.0, 1.0]]
distortions = {dist_str}
rotation = [{rvec[0]}, {rvec[1]}, {rvec[2]}]
translation = [{tvec[0]}, {tvec[1]}, {tvec[2]}]
fisheye = false
""")
```

## 変更しない箇所（リグレッション防止）

- 既存の投影関数・残差関数
- K未知モードの推定ロジック全体
- 再投影誤差の評価閾値
- TOML/CSV出力フォーマット（K未知モード）

## テスト設計

### テストファイル

`tests/test_estimate_camera_params_extrinsic.py`

### テストケース

#### T-1: K既知モード（6点以上、RANSAC使用）

- 合成データ（ノイズなし、8点）でR, tが正しく推定されること（再投影誤差 < 1.0 px）
- 1点を外れ値として大きくずらした場合、outlierとして検出されること

#### T-2: K既知モード（4〜5点、RANSACなし）

- 4点の合成データでR, tが推定できること（再投影誤差 < 1.0 px）

#### T-3: K既知モード（3点以下、エラー）

- 3点の場合にエラー終了すること

#### T-4: TOML読み込み

- 正常なTOMLからK, dist, sizeを正しく読み込めること
- 存在しないカメラ名の場合にエラーが発生すること

#### T-5: オプション排他の警告

- `--intrinsic-toml` と `--k3` の同時指定で警告メッセージが表示されること

#### T-6: リグレッションテスト

- `--intrinsic-toml` 未指定時に既存の4係数推定が正常動作すること（再投影誤差 < 1.0 px）

#### T-7: 共通モジュール

- `common.py` の関数が正しくインポート・動作すること

#### T-8: TOML出力フォーマット

- K既知モードの出力がCalib_scene.toml形式として正しいこと

### 合成データ生成方法

feat-001のテスト（`test_estimate_extrinsic.py`）と同じ方法を使用する。

# feat-008: estimate_camera_params.py 複数カメラ一括推定 — 機能設計書

## 変更対象ファイル

- `phase0/estimate_camera_params.py` — 既存ファイルの修正

## 変更箇所

### 1. `target_camera` のパース（カンマ区切り対応）

`_run_extrinsic_estimation` の冒頭で `target_camera` をパースする。
空文字列と重複をフィルタする。

```python
target_cameras_str = config['target_camera']
target_cameras = list(dict.fromkeys(
    c.strip() for c in target_cameras_str.split(',') if c.strip()
))
```

### 2. `--intrinsic-toml` のディレクトリ対応

`load_intrinsic_toml` を拡張し、ディレクトリが渡された場合は `*_intrinsics.toml` を全て読み込む。

```python
def load_intrinsic_toml(toml_path: str, camera_name: str) -> dict:
    """TOMLファイルまたはディレクトリから内部パラメータを読み込む"""
    path = Path(toml_path)

    if path.is_dir():
        # ディレクトリ内の *_intrinsics.toml を全て探索
        toml_files = sorted(path.glob('*_intrinsics.toml'))
        if not toml_files:
            print(f"エラー: ディレクトリ内に *_intrinsics.toml が見つかりません: {toml_path}")
            return None
        for toml_file in toml_files:
            with open(toml_file, 'rb') as f:
                data = tomli.load(f)
            if camera_name in data:
                cam_data = data[camera_name]
                K = np.array(cam_data['matrix'], dtype=np.float64)
                dist = np.array(cam_data['distortions'], dtype=np.float64)
                size = cam_data['size']
                return {
                    'K': K, 'dist': dist,
                    'image_width': int(size[0]),
                    'image_height': int(size[1]),
                }
        # 見つからなかった場合
        print(f"エラー: カメラ '{camera_name}' がディレクトリ内のTOMLに見つかりません: {toml_path}")
        return None
    else:
        # 既存のファイル指定ロジック（変更なし）
        with open(toml_path, 'rb') as f:
            data = tomli.load(f)

        if camera_name not in data:
            print(f"エラー: カメラ '{camera_name}' がTOMLファイルに見つかりません: {toml_path}")
            print(f"  利用可能なセクション: {[k for k in data.keys() if k != 'metadata']}")
            return None

        cam_data = data[camera_name]
        K = np.array(cam_data['matrix'], dtype=np.float64)
        dist = np.array(cam_data['distortions'], dtype=np.float64)
        size = cam_data['size']
        return {
            'K': K, 'dist': dist,
            'image_width': int(size[0]),
            'image_height': int(size[1]),
        }
```

### 3. `--output` オプション追加

`argparse` に `--output` を追加。

```python
parser.add_argument('--output', default=None,
                    help='推定結果の出力先TOMLファイル（K既知モードのみ）')
```

### 4. `main()` の変更

`--output` を `run_estimation` に渡す。通常モードで複数カメラ指定時のエラー処理を追加。
既存の `--intrinsic-toml` 存在確認メッセージをディレクトリ対応に修正する。

```python
def main():
    # ... argparse 部分は既存 + --output 追加 ...

    # --intrinsic-toml の存在確認（既存コードの修正）
    # 修正前: "エラー: TOMLファイルが見つかりません"
    # 修正後: ファイルとディレクトリの両方に対応したメッセージ
    if args.intrinsic_toml:
        if not Path(args.intrinsic_toml).exists():
            print(f"エラー: TOMLファイルまたはディレクトリが見つかりません: {args.intrinsic_toml}")
            return 1

    # --output が --intrinsic-toml なしで指定された場合の警告
    if args.output and not args.intrinsic_toml:
        print("警告: --output は --intrinsic-toml と併用時のみ有効です。無視します。")

    # 通常モードで複数カメラ指定のエラーチェック
    # 通常モード（K未知）では target_camera のカンマ分割は行わない。
    # main() でガードするため、通常モード側の run_estimation には
    # 常に単一カメラ名の target_camera が渡される。
    if not args.intrinsic_toml:
        config = load_yaml_simple(args.config)
        cameras = [c.strip() for c in config['target_camera'].split(',') if c.strip()]
        if len(cameras) > 1:
            print("エラー: 通常モード（K未知）では複数カメラの指定はサポートされていません。")
            print("  複数カメラの一括推定には --intrinsic-toml を使用してください。")
            return 1

    result = run_estimation(args.config, use_k3, args.wide, args.fix_center,
                            intrinsic_toml=args.intrinsic_toml,
                            output_path=args.output if args.intrinsic_toml else None)
    return result
```

### 5. `run_estimation` のシグネチャ変更

`output_path` 引数を追加し、`_run_extrinsic_estimation` に渡す。

```python
def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool,
                   intrinsic_toml: str = None, output_path: str = None):
    """メイン処理"""
    if intrinsic_toml:
        return _run_extrinsic_estimation(config_path, intrinsic_toml, output_path)
    # ... 通常モード（変更なし）...
```

### 6. `_run_extrinsic_estimation` の複数カメラ対応

現行の `_run_extrinsic_estimation(config_path, toml_path)` を修正し、`target_cameras` をループで処理する。

返り値: 1台以上成功した場合は 0、全カメラ失敗した場合は 1。

```python
def _run_extrinsic_estimation(config_path: str, toml_path: str, output_path: str = None):
    config = load_yaml_simple(config_path)
    config_dir = Path(config_path).parent

    target_cameras_str = config['target_camera']
    target_cameras = list(dict.fromkeys(
        c.strip() for c in target_cameras_str.split(',') if c.strip()
    ))

    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']
    points_3d_dict = load_points_3d(points_3d_path)

    results = {}  # カメラ名 → 推定結果（TOML出力用）

    for target_camera in target_cameras:
        # 内部パラメータ読み込み
        intrinsic = load_intrinsic_toml(toml_path, target_camera)
        if intrinsic is None:
            print(f"警告: {target_camera} の内部パラメータが見つかりません。スキップします。")
            continue

        K = intrinsic['K']
        dist = intrinsic['dist']
        img_width = intrinsic['image_width']
        img_height = intrinsic['image_height']

        # 2D座標読み込み（カメラ別）
        points_2d_dict = load_points_2d(points_2d_path, target_camera)
        points_3d_matched, points_2d_matched, point_names = match_points(points_3d_dict, points_2d_dict)

        num_points = len(point_names)
        if num_points == 0:
            print(f"警告: {target_camera} の2D観測データが見つかりません。スキップします。")
            continue

        # ヘッダー表示（カメラごと、既存フォーマット）
        print("=" * 60)
        print(f"外部パラメータ推定 (K既知): {target_camera}")
        print("=" * 60)
        print(f"設定ファイル: {config_path}")
        print(f"内部パラメータ: {toml_path}")
        print(f"3D座標: {points_3d_path}")
        print(f"2D座標: {points_2d_path}")
        print(f"画像サイズ: {img_width} x {img_height}")

        print(f"\n[内部パラメータ (TOML読み込み)]")
        print(f"fx: {K[0, 0]:.2f}")
        print(f"fy: {K[1, 1]:.2f}")
        print(f"cx: {K[0, 2]:.2f}")
        print(f"cy: {K[1, 2]:.2f}")
        print(f"歪み係数: {dist.tolist()}")

        print(f"\n基準点数: {num_points}点")
        for name in point_names:
            print(f"  - {name}")

        # 推定
        print("\n" + "=" * 60)
        print("推定結果")
        print("=" * 60)

        result = estimate_extrinsic(points_3d_matched, points_2d_matched, K, dist, point_names)
        if result is None:
            print(f"警告: {target_camera} の推定に失敗しました。スキップします。")
            continue

        rvec = result['rvec']
        tvec = result['tvec']

        # 推定手法の表示（既存フォーマット）
        if result['used_ransac']:
            print(f"\n[推定手法]")
            print("solvePnPRansac → solvePnP(ITERATIVE) 精密化")
            print(f"\n[RANSAC結果]")
            print(f"inlier: {len(result['inliers'])}点 / 全{num_points}点")
            print(f"  inlier:  {', '.join(result['inliers'])}")
            if result['outliers']:
                print(f"  outlier: {', '.join(result['outliers'])}")
            else:
                print("  outlier: なし")
        else:
            print(f"\n[推定手法]")
            print("solvePnP(SQPNP)（基準点が4〜5点のためRANSACなし）")

        # 外部パラメータ表示
        print(f"\n[外部パラメータ]")
        print(f"rvec: [{rvec[0]:.6f}, {rvec[1]:.6f}, {rvec[2]:.6f}]")
        print(f"tvec: [{tvec[0]:.6f}, {tvec[1]:.6f}, {tvec[2]:.6f}]")

        # 再投影誤差
        rmse, per_point_errors = compute_reprojection_errors(
            points_3d_matched, points_2d_matched, K, dist, rvec, tvec
        )
        print(f"\n[再投影誤差]")
        print(f"RMSE: {rmse:.2f} pixels")
        if rmse < 5:
            print("評価: ✓ 優秀")
        elif rmse < 10:
            print("評価: ✓ 良好")
        elif rmse < 20:
            print("評価: △ 許容範囲")
        else:
            print("評価: ✗ 要確認（点のずれ or 推定失敗）")

        outlier_set = set(result['outliers'])
        print("\n[各点の再投影誤差]")
        for i, name in enumerate(point_names):
            err = per_point_errors[i]
            if name in outlier_set:
                status = "✗ [outlier]"
            elif err < 10:
                status = "✓"
            else:
                status = "✗"
            print(f"  {name}: {err:.2f} px {status}")

        # TOML形式表示（既存の _print_toml_output を使用）
        print("\n" + "=" * 60)
        print("Calib_scene.toml 形式")
        print("=" * 60)
        _print_toml_output(target_camera, img_width, img_height, K, dist, rvec, tvec)

        # TOML出力用に保存（numpy配列のまま保持し、数値精度を統一する）
        results[target_camera] = {
            'name': target_camera,
            'size': [float(img_width), float(img_height)],
            'K': K,        # numpy ndarray
            'dist': dist,  # numpy ndarray
            'rvec': rvec,  # numpy ndarray
            'tvec': tvec,  # numpy ndarray
        }

    # ファイル出力
    if output_path:
        if results:
            _write_toml_output(output_path, results)
        else:
            print("\nエラー: 全カメラの推定に失敗しました。ファイルは出力しません。")

    return 0 if results else 1
```

### 7. TOML文字列生成の共通関数化とファイル出力

既存の `_print_toml_output` をリファクタし、TOML文字列の生成と出力先を分離する。
これにより標準出力とファイル出力で同一のフォーマット（数値精度を含む）を保証する。

K, dist, rvec, tvec は全て numpy 配列のまま f-string に埋め込むため、
`numpy.float64.__str__()` のデフォルト精度で出力される。標準出力もファイル出力も
同じ `_format_toml_section` を経由するので数値精度は完全に一致する。

```python
def _format_toml_section(camera_name: str, image_width: int, image_height: int,
                         K: np.ndarray, dist: np.ndarray,
                         rvec: np.ndarray, tvec: np.ndarray) -> str:
    """1カメラ分のTOMLセクション文字列を生成する"""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    dist_list = dist.tolist()
    dist_str = f"[{', '.join(str(v) for v in dist_list)}]"

    return (
        f'[{camera_name}]\n'
        f'name = "{camera_name}"\n'
        f'size = [{image_width}.0, {image_height}.0]\n'
        f'matrix = [[{fx}, 0.0, {cx}], [0.0, {fy}, {cy}], [0.0, 0.0, 1.0]]\n'
        f'distortions = {dist_str}\n'
        f'rotation = [{rvec[0]}, {rvec[1]}, {rvec[2]}]\n'
        f'translation = [{tvec[0]}, {tvec[1]}, {tvec[2]}]\n'
        f'fisheye = false\n'
    )


def _print_toml_output(camera_name: str, image_width: int, image_height: int,
                        K: np.ndarray, dist: np.ndarray,
                        rvec: np.ndarray, tvec: np.ndarray):
    """標準出力にTOMLセクションを表示する（既存関数のリファクタ）"""
    print()
    print(_format_toml_section(camera_name, image_width, image_height,
                               K, dist, rvec, tvec))


def _write_toml_output(output_path: str, results: dict):
    """推定結果をTOMLファイルに書き出す。
    results: {camera_name: {'name', 'size'([float,float]),
              'K'(ndarray), 'dist'(ndarray),
              'rvec'(ndarray), 'tvec'(ndarray)}, ...}
    """
    with open(output_path, 'w', encoding='utf-8') as f:
        for camera_name, data in results.items():
            section = _format_toml_section(
                camera_name,
                int(data['size'][0]), int(data['size'][1]),
                data['K'], data['dist'], data['rvec'], data['tvec']
            )
            f.write(section)
            f.write('\n')
    print(f"\n推定結果を保存しました: {output_path}")
```

## 既存コードへの影響

- `_run_extrinsic_estimation` の内部構造を変更する（ループ化）
- `_run_extrinsic_estimation` のシグネチャに `output_path` を追加
- `run_estimation` のシグネチャに `output_path` を追加
- `main()` に `--output` 引数の処理と通常モードの複数カメラエラーチェックを追加
- `load_intrinsic_toml` にディレクトリ対応を追加する
- 既存の `_print_toml_output` を `_format_toml_section` + `_print_toml_output` にリファクタする
- 1台指定時の動作は変わらない（ループが1回だけ回る）
- 通常モード（K未知）は一切変更しない

## 依存関係

新規パッケージの追加は不要。

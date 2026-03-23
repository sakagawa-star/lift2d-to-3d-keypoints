# feat-009: 三角測量による外部パラメータ検証 — 機能設計書

## モジュール構成

新規スクリプト `phase0/verify_triangulation.py` を作成する。

## コマンドライン仕様

```bash
uv run python verify_triangulation.py <config.yaml> <extrinsic.toml>
```

- `config.yaml`: 既存の設定ファイル（`points_3d`, `points_2d` のパスを参照）
- `extrinsic.toml`: feat-008 の `--output` で出力された全カメラの推定結果TOML

引数パースには `argparse` を使用する（既存の `estimate_camera_params.py` と統一）。

### config.yaml から使用するフィールド

- `points_3d`: 3D基準点CSVのパス（config.yaml の親ディレクトリからの相対パスとして解決する）
- `points_2d`: 2D観測CSVのパス（同上）

`target_camera` は使用しない（対象カメラは extrinsic.toml のセクション名で決まる）。

### main 関数

```python
def main():
    parser = argparse.ArgumentParser(description='三角測量による外部パラメータ検証')
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    parser.add_argument('extrinsic_toml', help='推定結果TOMLファイル')
    args = parser.parse_args()

    config = load_yaml_simple(args.config)
    config_dir = Path(args.config).parent

    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']

    # 全カメラのパラメータを読み込む
    cameras = load_all_cameras(args.extrinsic_toml)
    if len(cameras) < 2:
        print(f"エラー: カメラが2台未満です（{len(cameras)}台）。三角測量検証には2台以上必要です。")
        return 1

    # データ読み込み
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_all = load_points_2d_all(points_2d_path)

    # カメラペア生成 → 三角測量 → 結果表示
    # ... （以降のロジック）
    return 0


if __name__ == '__main__':
    exit(main())
```

## データフロー

```
1. config.yaml を読み込む → points_3d, points_2d のパスを取得
   パスは config.yaml の親ディレクトリからの相対パスとして解決する
2. extrinsic.toml を読み込む → 全カメラの K, dist, rvec, tvec を取得
   カメラが2台未満の場合はエラー終了
3. points_3d CSV を読み込む → 3D基準点座標（Ground Truth）
4. points_2d CSV を全カメラ分読み込む
5. カメラペアを生成（全組み合わせ）
6. 各ペアについて:
   a. 共有基準点を特定（両カメラで観測されている点）
   b. 共有基準点が2点未満ならスキップ
   c. 2D座標を歪み補正（cv2.undistortPoints、P指定なし → 正規化座標を返す）
   d. 射影行列 P = [R|t] を構築（Kなし）
   e. cv2.triangulatePoints で3D座標を復元
   f. 同次座標の w ≈ 0 の点を除外
   g. 復元3D座標と既知3D座標のユークリッド距離を計算
7. カメラペア別・カメラ別のサマリーを表示
   有効なペアが0件の場合はメッセージを表示
```

## 主要関数

### load_all_cameras(toml_path: str) -> dict

extrinsic.toml から全カメラのパラメータを読み込む。

```python
def load_all_cameras(toml_path: str) -> dict:
    """全カメラのパラメータを読み込む
    戻り値: {camera_name: {'K': ndarray, 'dist': ndarray, 'rvec': ndarray, 'tvec': ndarray}, ...}
    """
    with open(toml_path, 'rb') as f:
        data = tomli.load(f)

    cameras = {}
    for section_name, section in data.items():
        if not isinstance(section, dict) or 'matrix' not in section:
            continue  # metadata等のカメラ以外のセクションをスキップ

        # rotation/translation の要素数を検証
        rotation = section.get('rotation', [])
        translation = section.get('translation', [])
        distortions = section.get('distortions', [])

        if len(rotation) != 3:
            print(f"警告: {section_name} の rotation が3要素ではありません（{len(rotation)}要素）。スキップします。")
            continue
        if len(translation) != 3:
            print(f"警告: {section_name} の translation が3要素ではありません（{len(translation)}要素）。スキップします。")
            continue
        if len(distortions) < 4:
            print(f"警告: {section_name} の distortions が4要素未満です（{len(distortions)}要素）。スキップします。")
            continue

        cameras[section_name] = {
            'K': np.array(section['matrix'], dtype=np.float64),
            'dist': np.array(distortions, dtype=np.float64),
            'rvec': np.array(rotation, dtype=np.float64),
            'tvec': np.array(translation, dtype=np.float64),
        }
    return cameras
```

### load_points_2d_all(csv_path: str) -> dict

全カメラの2D座標を一括読み込み。スクリプト内に定義する（現時点での使用箇所が1つのため `common.py` には追加しない）。

```python
def load_points_2d_all(csv_path: str) -> dict[str, dict[str, list[float]]]:
    """全カメラの2D座標を読み込む
    戻り値: {camera_name: {ObjectName: [X, Y], ...}, ...}
    """
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            cam = row['camera_name']
            name = row['ObjectName']
            if cam not in points:
                points[cam] = {}
            points[cam][name] = [float(row['X']), float(row['Y'])]
    return points
```

### triangulate_points(cam1, cam2, pts1, pts2) -> ndarray

2台のカメラから三角測量で3D座標を復元する。

歪み補正は `cv2.undistortPoints` で P 指定なし（正規化座標を返す）方式を使用する。
射影行列は `[R|t]`（Kなし）を使用する。この方式は意図が明確で可読性が高い。

`cv2.undistortPoints` は入力形状 `(N, 1, 2)` を要求する。出力も `(N, 1, 2)`。
`cv2.triangulatePoints` は入力形状 `(2, N)` を要求する。
形状変換は `reshape(-1, 2).T` で `(N, 1, 2)` → `(N, 2)` → `(2, N)` とする。

```python
def triangulate_points(cam1: dict, cam2: dict,
                       pts1: np.ndarray, pts2: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """三角測量で3D座標を復元
    引数:
        cam1, cam2: {'K': ndarray, 'dist': ndarray, 'rvec': ndarray, 'tvec': ndarray}
        pts1, pts2: 2D座標 (N, 2)
    戻り値: (復元3D座標 (N, 3), 有効フラグ (N,))
    """
    K1, dist1 = cam1['K'], cam1['dist']
    K2, dist2 = cam2['K'], cam2['dist']

    R1, _ = cv2.Rodrigues(cam1['rvec'])
    R2, _ = cv2.Rodrigues(cam2['rvec'])

    # 射影行列（Kなし、正規化座標用）
    P1 = np.hstack([R1, cam1['tvec'].reshape(3, 1)])
    P2 = np.hstack([R2, cam2['tvec'].reshape(3, 1)])

    # 歪み補正 → 正規化座標（P指定なし）
    # 入力: (N, 1, 2)、出力: (N, 1, 2)
    pts1_undist = cv2.undistortPoints(
        pts1.reshape(-1, 1, 2).astype(np.float64), K1, dist1
    )
    pts2_undist = cv2.undistortPoints(
        pts2.reshape(-1, 1, 2).astype(np.float64), K2, dist2
    )

    # 三角測量
    # triangulatePoints の入力は (2, N) 形状
    # (N, 1, 2) → (N, 2) → (2, N)
    points_4d = cv2.triangulatePoints(
        P1, P2,
        pts1_undist.reshape(-1, 2).T,  # (2, N)
        pts2_undist.reshape(-1, 2).T   # (2, N)
    )

    # w ≈ 0 の点を検出（光線がほぼ平行な場合）
    w = points_4d[3]
    valid = np.abs(w) > 1e-6

    # 同次座標 → 3D座標
    points_3d = np.full((points_4d.shape[1], 3), np.nan)
    points_3d[valid] = (points_4d[:3, valid] / w[valid]).T

    return points_3d, valid
```

## カメラペア生成

```python
from itertools import combinations
camera_names = sorted(cameras.keys())
pairs = list(combinations(camera_names, 2))
```

## サマリー計算

カメラ別平均誤差は、そのカメラが含まれる全ペアの誤差を集約して計算する。
有効なペアが0件の場合は「有効なカメラペアがありませんでした」と表示する。

```python
from collections import defaultdict

camera_errors = defaultdict(list)
for (cam_a, cam_b), errors in pair_results.items():
    camera_errors[cam_a].extend(errors)
    camera_errors[cam_b].extend(errors)

if not camera_errors:
    print("有効なカメラペアがありませんでした。")
else:
    for cam_name in sorted(camera_errors.keys()):
        mean_err = np.mean(camera_errors[cam_name])
        print(f"  {cam_name}: {mean_err:.4f}m")
```

## 依存関係

- `common.py`: `load_yaml_simple`, `load_points_3d` を使用
- `argparse`: コマンドライン引数パース
- `pathlib.Path`: config.yaml からの相対パス解決
- `tomli`: TOML読み込み（既存の依存）
- `cv2`: `triangulatePoints`, `undistortPoints`, `Rodrigues`
- `numpy`: 行列演算
- `itertools`: カメラペア生成
- `csv`: 2D座標読み込み
- `collections.defaultdict`: サマリー集計

新規パッケージの追加は不要。

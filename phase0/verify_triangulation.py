"""
三角測量による外部パラメータ検証スクリプト

2台のカメラの2D観測から三角測量で3D座標を復元し、
既知の3D基準点座標との差を評価する。

使い方:
    python verify_triangulation.py config.yaml extrinsic.toml
"""

import argparse
import csv
from collections import defaultdict
from itertools import combinations
from pathlib import Path

import cv2
import numpy as np
import tomli

from common import load_yaml_simple, load_points_3d


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


def main():
    parser = argparse.ArgumentParser(description='三角測量による外部パラメータ検証')
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    parser.add_argument('extrinsic_toml', help='推定結果TOMLファイル')
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}")
        return 1

    if not Path(args.extrinsic_toml).exists():
        print(f"エラー: TOMLファイルが見つかりません: {args.extrinsic_toml}")
        return 1

    config = load_yaml_simple(args.config)
    config_dir = Path(args.config).parent

    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']

    # 全カメラのパラメータを読み込む
    cameras = load_all_cameras(args.extrinsic_toml)
    if len(cameras) < 2:
        print(f"エラー: カメラが2台未満です（{len(cameras)}台）。三角測量検証には2台以上必要です。")
        return 1

    print(f"読み込みカメラ数: {len(cameras)}台")
    for cam_name in sorted(cameras.keys()):
        print(f"  - {cam_name}")

    # データ読み込み
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_all = load_points_2d_all(points_2d_path)

    # カメラペア生成
    camera_names = sorted(cameras.keys())
    pairs = list(combinations(camera_names, 2))

    # 各ペアの結果を格納
    pair_results = {}  # (cam_a, cam_b) → [error1, error2, ...]

    for cam_a, cam_b in pairs:
        # 共有基準点を特定
        pts_a = points_2d_all.get(cam_a, {})
        pts_b = points_2d_all.get(cam_b, {})
        shared_names = sorted(set(pts_a.keys()) & set(pts_b.keys()) & set(points_3d_dict.keys()))

        if len(shared_names) < 2:
            continue

        # 共有点の2D座標と3D座標を収集
        pts1 = np.array([pts_a[name] for name in shared_names], dtype=np.float64)
        pts2 = np.array([pts_b[name] for name in shared_names], dtype=np.float64)
        gt_3d = np.array([points_3d_dict[name] for name in shared_names], dtype=np.float64)

        # 三角測量
        reconstructed_3d, valid = triangulate_points(cameras[cam_a], cameras[cam_b], pts1, pts2)

        # w ≈ 0 の点を警告
        invalid_count = np.sum(~valid)
        if invalid_count > 0:
            print(f"警告: {cam_a} - {cam_b} で {invalid_count} 点が光線平行のため除外されました")

        # ペアごとの結果表示
        print("=" * 60)
        print(f"三角測量検証: {cam_a} - {cam_b}")
        print("=" * 60)
        print(f"共有基準点: {len(shared_names)}点")

        print("\n[各点の3D復元誤差]")
        errors = []
        for i, name in enumerate(shared_names):
            if not valid[i]:
                print(f"  {name}: 除外（光線が平行）")
                continue
            gt = gt_3d[i]
            rec = reconstructed_3d[i]
            err = np.linalg.norm(gt - rec)
            errors.append(err)
            print(f"  {name}: 既知({gt[0]:.3f}, {gt[1]:.3f}, {gt[2]:.3f})"
                  f" vs 復元({rec[0]:.3f}, {rec[1]:.3f}, {rec[2]:.3f})"
                  f" → 誤差 {err:.3f}m")

        if errors:
            mean_err = np.mean(errors)
            max_err = np.max(errors)
            print(f"\n平均3D復元誤差: {mean_err:.3f}m")
            print(f"最大3D復元誤差: {max_err:.3f}m")
            pair_results[(cam_a, cam_b)] = errors
        else:
            print("\n有効な復元点がありませんでした。")

    # サマリー
    print("\n" + "=" * 60)
    print("サマリー")
    print("=" * 60)

    if not pair_results:
        print("有効なカメラペアがありませんでした。")
        return 0

    print("[カメラペア別 平均3D復元誤差]")
    for (cam_a, cam_b), errors in pair_results.items():
        mean_err = np.mean(errors)
        print(f"  {cam_a} - {cam_b}: {mean_err:.3f}m ({len(errors)}点)")

    # カメラ別平均誤差
    camera_errors = defaultdict(list)
    for (cam_a, cam_b), errors in pair_results.items():
        camera_errors[cam_a].extend(errors)
        camera_errors[cam_b].extend(errors)

    print("\n[カメラ別 平均3D復元誤差]（全ペアの平均）")
    for cam_name in sorted(camera_errors.keys()):
        mean_err = np.mean(camera_errors[cam_name])
        print(f"  {cam_name}: {mean_err:.3f}m")

    return 0


if __name__ == '__main__':
    exit(main())

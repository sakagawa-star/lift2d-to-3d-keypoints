"""
Phase0 検証スクリプト（改良版）
- config.yaml から設定を読み込み
- CSVファイルからカメラパラメータと基準点を読み込み

使い方:
    python phase0_verification.py config.yaml
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import cv2
from scipy.optimize import least_squares


def load_yaml_simple(yaml_path: str) -> dict:
    """
    簡易YAMLパーサー（標準ライブラリのみ使用）
    key: value 形式のみ対応
    """
    config = {}
    with open(yaml_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            # 空行・コメント行をスキップ
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                key, value = line.split(':', 1)
                config[key.strip()] = value.strip()
    return config


def load_camera_params(csv_path: str, camera_name: str) -> dict:
    """
    camera_params.csv から指定カメラのパラメータを読み込む
    """
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['camera_name'] == camera_name:
                return {
                    'width': int(row['width']),
                    'height': int(row['height']),
                    'K': np.array([
                        [float(row['fx']), 0, float(row['cx'])],
                        [0, float(row['fy']), float(row['cy'])],
                        [0, 0, 1]
                    ]),
                    'dist': np.array([
                        float(row['k1']), float(row['k2']),
                        float(row['p1']), float(row['p2'])
                    ]),
                    'rvec': np.array([
                        float(row['r1']), float(row['r2']), float(row['r3'])
                    ]),
                    'tvec': np.array([
                        float(row['t1']), float(row['t2']), float(row['t3'])
                    ])
                }
    raise ValueError(f"カメラ '{camera_name}' が見つかりません: {csv_path}")


def load_points_3d(csv_path: str) -> dict:
    """
    kijunten_locations.csv から3D座標を読み込む
    戻り値: {ObjectName: [X, Y, Z], ...}
    """
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row['ObjectName']
            points[name] = [
                float(row['X']), float(row['Y']), float(row['Z'])
            ]
    return points


def load_points_2d(csv_path: str, camera_name: str) -> dict:
    """
    points_2d.csv から指定カメラの2D座標を読み込む
    戻り値: {ObjectName: [X, Y], ...}
    """
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['camera_name'] == camera_name:
                name = row['ObjectName']
                points[name] = [float(row['X']), float(row['Y'])]
    return points


def match_points(points_3d: dict, points_2d: dict):
    """
    3Dと2Dの点をObjectNameでマッチングし、配列として返す
    """
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


def project_points_simple(params, points_3d):
    """
    レベル2用: K未知で投影
    params: [fx, fy, cx, cy, r1, r2, r3, t1, t2, t3]
    """
    fx, fy, cx, cy = params[0:4]
    rvec = params[4:7]
    tvec = params[7:10]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, None
    )
    return projected.reshape(-1, 2)


def residual(params, points_3d, points_2d):
    """最適化用の残差関数"""
    projected = project_points_simple(params, points_3d)
    return (projected - points_2d).flatten()


def run_verification(config_path: str):
    """メイン処理"""
    
    # 設定読み込み
    config = load_yaml_simple(config_path)
    config_dir = Path(config_path).parent
    
    target_camera = config['target_camera']
    camera_params_path = config_dir / config['camera_params']
    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']
    
    print("=" * 60)
    print(f"Phase0 検証: {target_camera}")
    print("=" * 60)
    print(f"設定ファイル: {config_path}")
    print(f"カメラパラメータ: {camera_params_path}")
    print(f"3D座標: {points_3d_path}")
    print(f"2D座標: {points_2d_path}")
    
    # データ読み込み
    cam_params = load_camera_params(camera_params_path, target_camera)
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_dict = load_points_2d(points_2d_path, target_camera)
    
    # マッチング
    points_3d, points_2d, point_names = match_points(points_3d_dict, points_2d_dict)
    
    print(f"\n基準点数: {len(point_names)}点")
    for name in point_names:
        print(f"  - {name}")
    
    # Ground Truth
    GT_K = cam_params['K']
    GT_dist = cam_params['dist']
    GT_rvec = cam_params['rvec']
    GT_tvec = cam_params['tvec']
    img_width = cam_params['width']
    img_height = cam_params['height']
    
    # ========================================
    # Ground Truthで再投影確認
    # ========================================
    print("\n[Ground Truthで再投影確認]")
    projected_gt, _ = cv2.projectPoints(
        points_3d, GT_rvec.reshape(3, 1), GT_tvec.reshape(3, 1), GT_K, GT_dist
    )
    projected_gt = projected_gt.reshape(-1, 2)
    
    for i, name in enumerate(point_names):
        err = np.linalg.norm(projected_gt[i] - points_2d[i])
        status = "✓" if err < 20 else "✗"
        print(f"  {name}: 誤差 {err:.1f} px {status}")
    
    reproj_gt = np.sqrt(np.mean(np.sum((projected_gt - points_2d) ** 2, axis=1)))
    print(f"  RMSE: {reproj_gt:.2f} px")
    
    # ========================================
    # レベル1: K既知でR,t推定
    # ========================================
    print("\n" + "=" * 60)
    print("レベル1: K既知でR,t推定 (solvePnP)")
    print("=" * 60)
    
    success, rvec_est, tvec_est = cv2.solvePnP(
        points_3d, points_2d, GT_K, GT_dist, flags=cv2.SOLVEPNP_ITERATIVE
    )
    
    if success:
        rvec_est = rvec_est.flatten()
        tvec_est = tvec_est.flatten()
        
        print("\n[推定結果]")
        print(f"rotation (Rodrigues): [{rvec_est[0]:.6f}, {rvec_est[1]:.6f}, {rvec_est[2]:.6f}]")
        print(f"translation: [{tvec_est[0]:.6f}, {tvec_est[1]:.6f}, {tvec_est[2]:.6f}]")
        
        print("\n[Ground Truth]")
        print(f"rotation (Rodrigues): [{GT_rvec[0]:.6f}, {GT_rvec[1]:.6f}, {GT_rvec[2]:.6f}]")
        print(f"translation: [{GT_tvec[0]:.6f}, {GT_tvec[1]:.6f}, {GT_tvec[2]:.6f}]")
        
        print("\n[誤差]")
        rvec_error = np.linalg.norm(rvec_est - GT_rvec)
        tvec_error = np.linalg.norm(tvec_est - GT_tvec)
        print(f"rotation誤差 (L2): {rvec_error:.6f} rad ({np.degrees(rvec_error):.2f}°)")
        print(f"translation誤差 (L2): {tvec_error:.6f} m ({tvec_error * 100:.2f} cm)")
        
        projected, _ = cv2.projectPoints(points_3d, rvec_est, tvec_est, GT_K, GT_dist)
        projected = projected.reshape(-1, 2)
        reproj_error = np.sqrt(np.mean(np.sum((projected - points_2d) ** 2, axis=1)))
        print(f"再投影誤差 (RMSE): {reproj_error:.2f} pixels")
    else:
        reproj_error = float('inf')
        print("solvePnP 失敗")
    
    # ========================================
    # レベル2: K未知で推定（歪みは0固定）
    # ========================================
    print("\n" + "=" * 60)
    print("レベル2: K未知で推定（歪みは0固定）")
    print("=" * 60)
    
    # 初期値（画像サイズから計算）
    fx_init = fy_init = 1500.0
    cx_init, cy_init = img_width / 2, img_height / 2
    
    K_init = np.array([[fx_init, 0, cx_init], [0, fy_init, cy_init], [0, 0, 1]])
    _, rvec_init, tvec_init = cv2.solvePnP(
        points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
    )
    rvec_init = rvec_init.flatten()
    tvec_init = tvec_init.flatten()
    
    params_init = np.array([
        fx_init, fy_init, cx_init, cy_init,
        rvec_init[0], rvec_init[1], rvec_init[2],
        tvec_init[0], tvec_init[1], tvec_init[2]
    ])
    
    print(f"\n[初期値]")
    print(f"fx, fy: {fx_init:.1f}, {fy_init:.1f}")
    print(f"cx, cy: {cx_init:.1f}, {cy_init:.1f}")
    
    # 最適化
    result = least_squares(
        residual, params_init, args=(points_3d, points_2d), method='lm', verbose=0
    )
    
    params_opt = result.x
    fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
    rvec_opt = params_opt[4:7]
    tvec_opt = params_opt[7:10]
    
    print("\n[推定結果]")
    print(f"fx: {fx_opt:.2f} (GT: {GT_K[0, 0]:.2f}, 誤差: {abs(fx_opt - GT_K[0, 0]):.2f}, {abs(fx_opt - GT_K[0, 0]) / GT_K[0, 0] * 100:.1f}%)")
    print(f"fy: {fy_opt:.2f} (GT: {GT_K[1, 1]:.2f}, 誤差: {abs(fy_opt - GT_K[1, 1]):.2f}, {abs(fy_opt - GT_K[1, 1]) / GT_K[1, 1] * 100:.1f}%)")
    print(f"cx: {cx_opt:.2f} (GT: {GT_K[0, 2]:.2f}, 誤差: {abs(cx_opt - GT_K[0, 2]):.2f})")
    print(f"cy: {cy_opt:.2f} (GT: {GT_K[1, 2]:.2f}, 誤差: {abs(cy_opt - GT_K[1, 2]):.2f})")
    
    print(f"\nrvec: [{rvec_opt[0]:.6f}, {rvec_opt[1]:.6f}, {rvec_opt[2]:.6f}]")
    print(f"tvec: [{tvec_opt[0]:.6f}, {tvec_opt[1]:.6f}, {tvec_opt[2]:.6f}]")
    
    print("\n[Ground Truth]")
    print(f"rvec: [{GT_rvec[0]:.6f}, {GT_rvec[1]:.6f}, {GT_rvec[2]:.6f}]")
    print(f"tvec: [{GT_tvec[0]:.6f}, {GT_tvec[1]:.6f}, {GT_tvec[2]:.6f}]")
    
    rvec_error_l2 = np.linalg.norm(rvec_opt - GT_rvec)
    tvec_error_l2 = np.linalg.norm(tvec_opt - GT_tvec)
    print(f"\nrotation誤差 (L2): {rvec_error_l2:.6f} rad ({np.degrees(rvec_error_l2):.2f}°)")
    print(f"translation誤差 (L2): {tvec_error_l2:.6f} m ({tvec_error_l2 * 100:.2f} cm)")
    
    projected_opt = project_points_simple(params_opt, points_3d)
    reproj_error_opt = np.sqrt(np.mean(np.sum((projected_opt - points_2d) ** 2, axis=1)))
    print(f"\n再投影誤差 (RMSE): {reproj_error_opt:.2f} pixels")
    
    print("\n[各点の再投影誤差]")
    for i, name in enumerate(point_names):
        err = np.linalg.norm(projected_opt[i] - points_2d[i])
        print(f"  {name}: {err:.2f} px")
    
    # ========================================
    # 結果サマリー
    # ========================================
    print("\n" + "=" * 60)
    print("結果サマリー")
    print("=" * 60)
    print(f"Ground Truth再投影: {reproj_gt:.2f} px")
    print(f"レベル1 (K既知): 再投影誤差 {reproj_error:.2f} px")
    print(f"レベル2 (K未知): 再投影誤差 {reproj_error_opt:.2f} px")
    print()
    print("[レベル2の内部パラメータ精度]")
    print(f"  fx誤差: {abs(fx_opt - GT_K[0, 0]) / GT_K[0, 0] * 100:.1f}%")
    print(f"  fy誤差: {abs(fy_opt - GT_K[1, 1]) / GT_K[1, 1] * 100:.1f}%")


def main():
    parser = argparse.ArgumentParser(
        description='Phase0 検証スクリプト'
    )
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    
    args = parser.parse_args()
    
    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}")
        return 1
    
    run_verification(args.config)
    return 0


if __name__ == '__main__':
    exit(main())

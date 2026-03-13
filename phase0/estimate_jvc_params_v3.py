"""
JVCカメラ パラメータ推定スクリプト（k3オプション対応版）

オプション:
    --k3    : k3（6次放射歪み）も推定する

使い方:
    python estimate_jvc_params_v3.py config_jvc.yaml          # k3なし（4係数）
    python estimate_jvc_params_v3.py config_jvc.yaml --k3     # k3あり（5係数）
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import cv2
from scipy.optimize import least_squares


# 歪み係数を推定する最小点数
MIN_POINTS_FOR_DISTORTION_4 = 15  # k1, k2, p1, p2（14変数）
MIN_POINTS_FOR_DISTORTION_5 = 16  # k1, k2, p1, p2, k3（15変数）


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


def project_points_no_distortion(params, points_3d):
    """歪みなしで投影（10変数）"""
    fx, fy, cx, cy = params[0:4]
    rvec = params[4:7]
    tvec = params[7:10]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, None
    )
    return projected.reshape(-1, 2)


def project_points_dist4(params, points_3d):
    """4係数歪みで投影（14変数）"""
    fx, fy, cx, cy = params[0:4]
    k1, k2, p1, p2 = params[4:8]
    rvec = params[8:11]
    tvec = params[11:14]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


def project_points_dist5(params, points_3d):
    """5係数歪みで投影（15変数）"""
    fx, fy, cx, cy = params[0:4]
    k1, k2, p1, p2, k3 = params[4:9]
    rvec = params[9:12]
    tvec = params[12:15]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2, k3])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


def residual_no_distortion(params, points_3d, points_2d):
    projected = project_points_no_distortion(params, points_3d)
    return (projected - points_2d).flatten()


def residual_dist4(params, points_3d, points_2d):
    projected = project_points_dist4(params, points_3d)
    return (projected - points_2d).flatten()


def residual_dist5(params, points_3d, points_2d):
    projected = project_points_dist5(params, points_3d)
    return (projected - points_2d).flatten()


def run_estimation(config_path: str, use_k3: bool):
    """メイン処理"""
    
    config = load_yaml_simple(config_path)
    config_dir = Path(config_path).parent
    
    target_camera = config['target_camera']
    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']
    
    img_width = int(config.get('image_width', 960))
    img_height = int(config.get('image_height', 540))
    
    print("=" * 60)
    print(f"カメラ パラメータ推定: {target_camera}")
    print("=" * 60)
    print(f"設定ファイル: {config_path}")
    print(f"3D座標: {points_3d_path}")
    print(f"2D座標: {points_2d_path}")
    print(f"画像サイズ: {img_width} x {img_height}")
    print(f"k3オプション: {'有効' if use_k3 else '無効'}")
    
    # データ読み込み
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_dict = load_points_2d(points_2d_path, target_camera)
    
    # マッチング
    points_3d, points_2d, point_names = match_points(points_3d_dict, points_2d_dict)
    num_points = len(point_names)
    
    print(f"\n基準点数: {num_points}点")
    for name in point_names:
        print(f"  - {name}")
    
    # 歪み係数を推定するかどうか判定
    min_points = MIN_POINTS_FOR_DISTORTION_5 if use_k3 else MIN_POINTS_FOR_DISTORTION_4
    estimate_distortion = num_points >= min_points
    
    if use_k3:
        if estimate_distortion:
            print(f"\n→ {num_points}点 >= {min_points}点: 歪み係数（k3含む5係数）を推定します")
        else:
            print(f"\n→ {num_points}点 < {min_points}点: 歪み係数は0に固定します")
            print(f"   （k3推定には{min_points}点以上必要）")
    else:
        if estimate_distortion:
            print(f"\n→ {num_points}点 >= {min_points}点: 歪み係数（4係数）を推定します")
        else:
            print(f"\n→ {num_points}点 < {min_points}点: 歪み係数は0に固定します")
    
    # ========================================
    # パラメータ推定
    # ========================================
    print("\n" + "=" * 60)
    print("パラメータ推定")
    print("=" * 60)
    
    # 初期値（共通部分）
    fx_init = fy_init = max(img_width, img_height)
    cx_init, cy_init = img_width / 2, img_height / 2
    
    K_init = np.array([[fx_init, 0, cx_init], [0, fy_init, cy_init], [0, 0, 1]])
    _, rvec_init, tvec_init = cv2.solvePnP(
        points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
    )
    rvec_init = rvec_init.flatten()
    tvec_init = tvec_init.flatten()
    
    print(f"\n[初期値]")
    print(f"fx, fy: {fx_init:.1f}, {fy_init:.1f}")
    print(f"cx, cy: {cx_init:.1f}, {cy_init:.1f}")
    
    if estimate_distortion and use_k3:
        # 5係数歪み（15変数）
        params_init = np.array([
            fx_init, fy_init, cx_init, cy_init,
            0.0, 0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2, k3
            rvec_init[0], rvec_init[1], rvec_init[2],
            tvec_init[0], tvec_init[1], tvec_init[2]
        ])
        
        result = least_squares(
            residual_dist5, params_init, 
            args=(points_3d, points_2d), method='lm', verbose=0
        )
        
        params_opt = result.x
        fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
        k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = params_opt[4:9]
        rvec_opt = params_opt[9:12]
        tvec_opt = params_opt[12:15]
        
        projected_opt = project_points_dist5(params_opt, points_3d)
        
    elif estimate_distortion and not use_k3:
        # 4係数歪み（14変数）
        params_init = np.array([
            fx_init, fy_init, cx_init, cy_init,
            0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2
            rvec_init[0], rvec_init[1], rvec_init[2],
            tvec_init[0], tvec_init[1], tvec_init[2]
        ])
        
        result = least_squares(
            residual_dist4, params_init, 
            args=(points_3d, points_2d), method='lm', verbose=0
        )
        
        params_opt = result.x
        fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
        k1_opt, k2_opt, p1_opt, p2_opt = params_opt[4:8]
        k3_opt = 0.0
        rvec_opt = params_opt[8:11]
        tvec_opt = params_opt[11:14]
        
        projected_opt = project_points_dist4(params_opt, points_3d)
        
    else:
        # 歪みなし（10変数）
        params_init = np.array([
            fx_init, fy_init, cx_init, cy_init,
            rvec_init[0], rvec_init[1], rvec_init[2],
            tvec_init[0], tvec_init[1], tvec_init[2]
        ])
        
        result = least_squares(
            residual_no_distortion, params_init, 
            args=(points_3d, points_2d), method='lm', verbose=0
        )
        
        params_opt = result.x
        fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
        k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = 0.0, 0.0, 0.0, 0.0, 0.0
        rvec_opt = params_opt[4:7]
        tvec_opt = params_opt[7:10]
        
        projected_opt = project_points_no_distortion(params_opt, points_3d)
    
    reproj_error = np.sqrt(np.mean(np.sum((projected_opt - points_2d) ** 2, axis=1)))
    
    # ========================================
    # 結果表示
    # ========================================
    print("\n" + "=" * 60)
    print("推定結果")
    print("=" * 60)
    
    print(f"\n[内部パラメータ]")
    print(f"fx: {fx_opt:.2f}")
    print(f"fy: {fy_opt:.2f}")
    print(f"cx: {cx_opt:.2f}")
    print(f"cy: {cy_opt:.2f}")
    
    print(f"\n[歪み係数]")
    if estimate_distortion:
        print(f"k1: {k1_opt:.6f}")
        print(f"k2: {k2_opt:.6f}")
        print(f"p1: {p1_opt:.6f}")
        print(f"p2: {p2_opt:.6f}")
        if use_k3:
            print(f"k3: {k3_opt:.6f}")
    else:
        print("(0に固定)")
    
    print(f"\n[外部パラメータ]")
    print(f"rvec: [{rvec_opt[0]:.6f}, {rvec_opt[1]:.6f}, {rvec_opt[2]:.6f}]")
    print(f"tvec: [{tvec_opt[0]:.6f}, {tvec_opt[1]:.6f}, {tvec_opt[2]:.6f}]")
    
    print(f"\n[再投影誤差]")
    print(f"RMSE: {reproj_error:.2f} pixels")
    
    if reproj_error < 5:
        print("評価: ✓ 優秀")
    elif reproj_error < 10:
        print("評価: ✓ 良好")
    elif reproj_error < 20:
        print("評価: △ 許容範囲")
    else:
        print("評価: ✗ 要確認（点のずれ or 推定失敗）")
    
    print("\n[各点の再投影誤差]")
    for i, name in enumerate(point_names):
        err = np.linalg.norm(projected_opt[i] - points_2d[i])
        status = "✓" if err < 10 else "✗"
        print(f"  {name}: {err:.2f} px {status}")
    
    # ========================================
    # Calib_scene.toml 形式で出力
    # ========================================
    print("\n" + "=" * 60)
    print("Calib_scene.toml 形式")
    print("=" * 60)
    
    if use_k3 and estimate_distortion:
        dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}, {k3_opt}]"
    else:
        dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}]"
    
    print(f"""
[{target_camera}]
name = "{target_camera}"
size = [{img_width}.0, {img_height}.0]
matrix = [[{fx_opt}, 0.0, {cx_opt}], [0.0, {fy_opt}, {cy_opt}], [0.0, 0.0, 1.0]]
distortions = {dist_str}
rotation = [{rvec_opt[0]}, {rvec_opt[1]}, {rvec_opt[2]}]
translation = [{tvec_opt[0]}, {tvec_opt[1]}, {tvec_opt[2]}]
fisheye = false
""")
    
    # ========================================
    # camera_params.csv 形式で出力
    # ========================================
    print("=" * 60)
    print("camera_params.csv 形式")
    print("=" * 60)
    
    if use_k3 and estimate_distortion:
        print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3,r1,r2,r3,t1,t2,t3")
        print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},{k1_opt},{k2_opt},{p1_opt},{p2_opt},{k3_opt},{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")
    else:
        print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,r1,r2,r3,t1,t2,t3")
        print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},{k1_opt},{k2_opt},{p1_opt},{p2_opt},{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")


def main():
    parser = argparse.ArgumentParser(
        description='カメラ パラメータ推定（k3オプション対応版）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
    python estimate_jvc_params_v3.py config.yaml          # k3なし（4係数）
    python estimate_jvc_params_v3.py config.yaml --k3     # k3あり（5係数）
        """
    )
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    parser.add_argument('--k3', action='store_true', 
                        help='k3（6次放射歪み係数）も推定する')
    
    args = parser.parse_args()
    
    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}")
        return 1
    
    run_estimation(args.config, args.k3)
    return 0


if __name__ == '__main__':
    exit(main())

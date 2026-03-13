"""
主点グリッドサーチによるカメラパラメータ推定

主点(cx, cy)の候補をグリッド状に探索し、
最も再投影誤差が小さい主点を採用する。

使い方:
    python estimate_camera_gridsearch.py config.yaml           # 通常モード
    python estimate_camera_gridsearch.py config.yaml --fine    # 高精度モード
    python estimate_camera_gridsearch.py config.yaml --grid-size 31 --range 0.2
"""

import argparse
import csv
import time
from pathlib import Path

import numpy as np
import cv2
from scipy.optimize import least_squares


def load_yaml_simple(yaml_path: str) -> dict:
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
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row['ObjectName']
            points[name] = [float(row['X']), float(row['Y']), float(row['Z'])]
    return points


def load_points_2d(csv_path: str, camera_name: str) -> dict:
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['camera_name'] == camera_name:
                name = row['ObjectName']
                points[name] = [float(row['X']), float(row['Y'])]
    return points


def match_points(points_3d: dict, points_2d: dict):
    matched_3d, matched_2d, names = [], [], []
    for name in points_3d:
        if name in points_2d:
            matched_3d.append(points_3d[name])
            matched_2d.append(points_2d[name])
            names.append(name)
    return np.array(matched_3d, dtype=np.float64), np.array(matched_2d, dtype=np.float64), names


def project_dist4_fix_center(params, points_3d, cx, cy):
    """主点固定モデル（12変数）"""
    fx, fy = params[0:2]
    k1, k2, p1, p2 = params[2:6]
    rvec = params[6:9]
    tvec = params[9:12]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2])
    projected, _ = cv2.projectPoints(points_3d, rvec.reshape(3,1), tvec.reshape(3,1), K, dist)
    return projected.reshape(-1, 2)


def residual_dist4_fix_center(params, points_3d, points_2d, cx, cy):
    projected = project_dist4_fix_center(params, points_3d, cx, cy)
    return (projected - points_2d).flatten()


def optimize_with_fixed_principal_point(points_3d, points_2d, cx, cy, img_width, img_height):
    """
    主点を固定して他のパラメータを最適化
    
    Returns:
        params: 最適化されたパラメータ
        rmse: 再投影誤差
        success: 最適化成功フラグ
    """
    # 初期値
    fx_init = fy_init = max(img_width, img_height)
    K_init = np.array([[fx_init, 0, cx], [0, fy_init, cy], [0, 0, 1]])
    
    try:
        _, rvec_init, tvec_init = cv2.solvePnP(
            points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
        )
        rvec_init = rvec_init.flatten()
        tvec_init = tvec_init.flatten()
    except:
        return None, float('inf'), False
    
    params_init = np.array([
        fx_init, fy_init,
        0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2
        rvec_init[0], rvec_init[1], rvec_init[2],
        tvec_init[0], tvec_init[1], tvec_init[2]
    ])
    
    try:
        result = least_squares(
            lambda p, p3d, p2d: residual_dist4_fix_center(p, p3d, p2d, cx, cy),
            params_init, args=(points_3d, points_2d), method='lm',
            max_nfev=1000  # 計算時間制限
        )
        
        residuals = result.fun
        rmse = np.sqrt(np.mean(residuals ** 2))
        
        return result.x, rmse, result.success
    except:
        return None, float('inf'), False


def grid_search(points_3d, points_2d, img_width, img_height, 
                cx_range, cy_range, grid_size, verbose=True):
    """
    主点のグリッドサーチ
    
    Returns:
        results: [(cx, cy, params, rmse), ...] のリスト
        best: 最良の結果
    """
    cx_list = np.linspace(cx_range[0], cx_range[1], grid_size)
    cy_list = np.linspace(cy_range[0], cy_range[1], grid_size)
    
    total = grid_size * grid_size
    results = []
    
    if verbose:
        print(f"\n探索中: {grid_size}x{grid_size} = {total}点")
        print(f"cx範囲: [{cx_range[0]:.1f}, {cx_range[1]:.1f}]")
        print(f"cy範囲: [{cy_range[0]:.1f}, {cy_range[1]:.1f}]")
    
    start_time = time.time()
    
    for i, cy in enumerate(cy_list):
        for j, cx in enumerate(cx_list):
            params, rmse, success = optimize_with_fixed_principal_point(
                points_3d, points_2d, cx, cy, img_width, img_height
            )
            
            if success and params is not None:
                results.append({
                    'cx': cx,
                    'cy': cy,
                    'params': params,
                    'rmse': rmse
                })
        
        # 進捗表示
        if verbose:
            progress = (i + 1) / grid_size * 100
            elapsed = time.time() - start_time
            eta = elapsed / (i + 1) * grid_size - elapsed
            print(f"\r  進捗: {progress:.0f}% ({i+1}/{grid_size}行), "
                  f"経過: {elapsed:.1f}秒, 残り: {eta:.1f}秒", end="")
    
    if verbose:
        print()  # 改行
    
    # 最良の結果を取得
    if results:
        best = min(results, key=lambda x: x['rmse'])
    else:
        best = None
    
    return results, best


def print_heatmap(results, cx_list, cy_list, best_cx, best_cy):
    """再投影誤差のヒートマップを表示"""
    
    # 結果を2D配列に変換
    grid_size_x = len(cx_list)
    grid_size_y = len(cy_list)
    heatmap = np.full((grid_size_y, grid_size_x), np.nan)
    
    for r in results:
        # 最も近いグリッド点を探す
        i = np.argmin(np.abs(cy_list - r['cy']))
        j = np.argmin(np.abs(cx_list - r['cx']))
        heatmap[i, j] = r['rmse']
    
    print("\n[再投影誤差ヒートマップ (px)]")
    print()
    
    # ヘッダー（cx値）- 間引いて表示
    step = max(1, grid_size_x // 7)
    print("      cx:", end="")
    for j in range(0, grid_size_x, step):
        print(f"{cx_list[j]:>7.0f}", end="")
    print()
    
    print("cy      ", end="")
    print("-" * (7 * ((grid_size_x + step - 1) // step)))
    
    # データ行 - 間引いて表示
    step_y = max(1, grid_size_y // 7)
    for i in range(0, grid_size_y, step_y):
        print(f"{cy_list[i]:>6.0f} |", end="")
        for j in range(0, grid_size_x, step):
            val = heatmap[i, j]
            if np.isnan(val):
                print("    -  ", end="")
            elif abs(cx_list[j] - best_cx) < 1 and abs(cy_list[i] - best_cy) < 1:
                print(f"{val:>6.1f}★", end="")
            else:
                print(f"{val:>7.1f}", end="")
        print()


def run_grid_search(config_path: str, grid_size: int, range_ratio: float, fine_mode: bool):
    """メイン処理"""
    
    config = load_yaml_simple(config_path)
    config_dir = Path(config_path).parent
    
    target_camera = config['target_camera']
    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']
    
    img_width = int(config.get('image_width', 960))
    img_height = int(config.get('image_height', 540))
    
    print("=" * 60)
    print(f"主点グリッドサーチ: {target_camera}")
    print("=" * 60)
    print(f"画像サイズ: {img_width} x {img_height}")
    print(f"グリッドサイズ: {grid_size} x {grid_size}")
    print(f"探索範囲: 中心 ± {range_ratio*100:.0f}%")
    print(f"高精度モード: {'有効' if fine_mode else '無効'}")
    
    # データ読み込み
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_dict = load_points_2d(points_2d_path, target_camera)
    points_3d, points_2d, point_names = match_points(points_3d_dict, points_2d_dict)
    
    print(f"基準点数: {len(point_names)}点")
    
    # ========================================
    # Stage 1: 粗いグリッドサーチ
    # ========================================
    print("\n" + "=" * 60)
    print("Stage 1: グリッドサーチ")
    print("=" * 60)
    
    cx_center = img_width / 2
    cy_center = img_height / 2
    cx_range = (cx_center - img_width * range_ratio, cx_center + img_width * range_ratio)
    cy_range = (cy_center - img_height * range_ratio, cy_center + img_height * range_ratio)
    
    start_time = time.time()
    
    results, best = grid_search(
        points_3d, points_2d, img_width, img_height,
        cx_range, cy_range, grid_size
    )
    
    stage1_time = time.time() - start_time
    
    if best is None:
        print("\nエラー: 最適化に失敗しました")
        return
    
    print(f"\nStage 1 完了: {stage1_time:.1f}秒")
    print(f"暫定最良: cx={best['cx']:.1f}, cy={best['cy']:.1f}, RMSE={best['rmse']:.2f} px")
    
    # ヒートマップ表示
    cx_list = np.linspace(cx_range[0], cx_range[1], grid_size)
    cy_list = np.linspace(cy_range[0], cy_range[1], grid_size)
    print_heatmap(results, cx_list, cy_list, best['cx'], best['cy'])
    
    # ========================================
    # Stage 2: 精密探索（オプション）
    # ========================================
    if fine_mode:
        print("\n" + "=" * 60)
        print("Stage 2: 精密探索")
        print("=" * 60)
        
        fine_range_ratio = 0.03  # ±3%
        fine_grid_size = 31
        
        cx_fine_range = (
            best['cx'] - img_width * fine_range_ratio,
            best['cx'] + img_width * fine_range_ratio
        )
        cy_fine_range = (
            best['cy'] - img_height * fine_range_ratio,
            best['cy'] + img_height * fine_range_ratio
        )
        
        start_time = time.time()
        
        results_fine, best_fine = grid_search(
            points_3d, points_2d, img_width, img_height,
            cx_fine_range, cy_fine_range, fine_grid_size
        )
        
        stage2_time = time.time() - start_time
        
        if best_fine is not None and best_fine['rmse'] < best['rmse']:
            best = best_fine
            results = results_fine
            print(f"\nStage 2 完了: {stage2_time:.1f}秒")
            print(f"改善: cx={best['cx']:.1f}, cy={best['cy']:.1f}, RMSE={best['rmse']:.2f} px")
            
            # 精密探索のヒートマップ
            cx_list_fine = np.linspace(cx_fine_range[0], cx_fine_range[1], fine_grid_size)
            cy_list_fine = np.linspace(cy_fine_range[0], cy_fine_range[1], fine_grid_size)
            print_heatmap(results_fine, cx_list_fine, cy_list_fine, best['cx'], best['cy'])
        else:
            print(f"\nStage 2 完了: 改善なし")
    
    # ========================================
    # 上位候補の表示
    # ========================================
    print("\n" + "=" * 60)
    print("探索結果")
    print("=" * 60)
    
    sorted_results = sorted(results, key=lambda x: x['rmse'])[:10]
    
    print("\n[上位10候補]")
    for i, r in enumerate(sorted_results):
        marker = "★" if i == 0 else " "
        print(f"  {i+1:2}. cx={r['cx']:>6.1f}, cy={r['cy']:>6.1f}: RMSE={r['rmse']:.2f} px {marker}")
    
    # ========================================
    # 最終結果
    # ========================================
    print("\n" + "=" * 60)
    print("最適パラメータ")
    print("=" * 60)
    
    params = best['params']
    fx_opt, fy_opt = params[0:2]
    k1_opt, k2_opt, p1_opt, p2_opt = params[2:6]
    rvec_opt = params[6:9]
    tvec_opt = params[9:12]
    cx_opt, cy_opt = best['cx'], best['cy']
    
    print(f"\n[内部パラメータ]")
    print(f"fx: {fx_opt:.2f}")
    print(f"fy: {fy_opt:.2f}")
    print(f"cx: {cx_opt:.2f} (グリッドサーチで決定)")
    print(f"cy: {cy_opt:.2f} (グリッドサーチで決定)")
    
    print(f"\n[歪み係数]")
    print(f"k1: {k1_opt:.6f}")
    print(f"k2: {k2_opt:.6f}")
    print(f"p1: {p1_opt:.6f}")
    print(f"p2: {p2_opt:.6f}")
    
    print(f"\n[外部パラメータ]")
    print(f"rvec: [{rvec_opt[0]:.6f}, {rvec_opt[1]:.6f}, {rvec_opt[2]:.6f}]")
    print(f"tvec: [{tvec_opt[0]:.6f}, {tvec_opt[1]:.6f}, {tvec_opt[2]:.6f}]")
    
    print(f"\n[再投影誤差]")
    print(f"RMSE: {best['rmse']:.2f} pixels")
    
    if best['rmse'] < 2:
        print("評価: ✓ 優秀")
    elif best['rmse'] < 5:
        print("評価: ✓ 良好")
    elif best['rmse'] < 10:
        print("評価: △ 許容範囲")
    else:
        print("評価: ✗ 要確認")
    
    # 各点の再投影誤差
    print("\n[各点の再投影誤差]")
    projected = project_dist4_fix_center(params, points_3d, cx_opt, cy_opt)
    for i, name in enumerate(point_names):
        err = np.linalg.norm(projected[i] - points_2d[i])
        status = "✓" if err < 5 else ("△" if err < 10 else "✗")
        print(f"  {name}: {err:.2f} px {status}")
    
    # ========================================
    # Calib_scene.toml形式
    # ========================================
    print("\n" + "=" * 60)
    print("Calib_scene.toml 形式")
    print("=" * 60)
    print(f"""
[{target_camera}]
name = "{target_camera}"
size = [{img_width}.0, {img_height}.0]
matrix = [[{fx_opt}, 0.0, {cx_opt}], [0.0, {fy_opt}, {cy_opt}], [0.0, 0.0, 1.0]]
distortions = [{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}]
rotation = [{rvec_opt[0]}, {rvec_opt[1]}, {rvec_opt[2]}]
translation = [{tvec_opt[0]}, {tvec_opt[1]}, {tvec_opt[2]}]
fisheye = false
""")
    
    # ========================================
    # 主点と画像中心の比較
    # ========================================
    print("=" * 60)
    print("診断情報")
    print("=" * 60)
    
    cx_diff = cx_opt - img_width / 2
    cy_diff = cy_opt - img_height / 2
    cx_diff_pct = cx_diff / img_width * 100
    cy_diff_pct = cy_diff / img_height * 100
    
    print(f"\n[主点と画像中心の差]")
    print(f"  Δcx: {cx_diff:+.1f} px ({cx_diff_pct:+.1f}%)")
    print(f"  Δcy: {cy_diff:+.1f} px ({cy_diff_pct:+.1f}%)")
    
    if abs(cx_diff_pct) < 5 and abs(cy_diff_pct) < 5:
        print("  → 主点は画像中心付近（正常）")
    elif abs(cx_diff_pct) < 10 and abs(cy_diff_pct) < 10:
        print("  → 主点はやや中心からずれている")
    else:
        print("  → 主点が中心から大きくずれている（要確認）")


def main():
    parser = argparse.ArgumentParser(
        description='主点グリッドサーチによるカメラパラメータ推定',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
    python estimate_camera_gridsearch.py config.yaml             # 通常モード
    python estimate_camera_gridsearch.py config.yaml --fine      # 高精度モード
    python estimate_camera_gridsearch.py config.yaml --grid-size 31 --range 0.2
        """
    )
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    parser.add_argument('--grid-size', type=int, default=21,
                        help='グリッドサイズ (default: 21)')
    parser.add_argument('--range', type=float, default=0.15, dest='range_ratio',
                        help='探索範囲（画像サイズに対する比率、default: 0.15）')
    parser.add_argument('--fine', action='store_true',
                        help='高精度モード（2段階探索）')
    
    args = parser.parse_args()
    
    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}")
        return 1
    
    run_grid_search(args.config, args.grid_size, args.range_ratio, args.fine)
    return 0


if __name__ == '__main__':
    exit(main())

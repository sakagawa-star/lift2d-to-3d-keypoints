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

import argparse
import sys
from pathlib import Path

import numpy as np
import cv2
import tomli
from scipy.optimize import least_squares

from common import load_yaml_simple, load_points_3d, load_points_2d, match_points


# 歪み係数を推定する最小点数
MIN_POINTS_NO_DIST = 6           # 歪みなし: 8-10変数
MIN_POINTS_DIST4 = 15            # 4係数: 12-14変数
MIN_POINTS_DIST5 = 16            # 5係数: 13-15変数
MIN_POINTS_DIST4_FIXCENTER = 12  # 4係数+主点固定: 12変数
MIN_POINTS_DIST5_FIXCENTER = 14  # 5係数+主点固定: 13変数
MIN_POINTS_DIST8 = 20            # 8係数: 18変数
MIN_POINTS_DIST8_FIXCENTER = 18  # 8係数+主点固定: 16変数


# ========================================
# 投影関数（主点固定版）
# ========================================

def project_no_dist_fix_center(params, points_3d, cx, cy):
    """歪みなし、主点固定（8変数）"""
    fx, fy = params[0:2]
    rvec = params[2:5]
    tvec = params[5:8]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, None
    )
    return projected.reshape(-1, 2)


def project_dist4_fix_center(params, points_3d, cx, cy):
    """4係数歪み、主点固定（12変数）"""
    fx, fy = params[0:2]
    k1, k2, p1, p2 = params[2:6]
    rvec = params[6:9]
    tvec = params[9:12]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


def project_dist5_fix_center(params, points_3d, cx, cy):
    """5係数歪み、主点固定（13変数）"""
    fx, fy = params[0:2]
    k1, k2, p1, p2, k3 = params[2:7]
    rvec = params[7:10]
    tvec = params[10:13]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2, k3])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    return projected.reshape(-1, 2)


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


# ========================================
# 投影関数（主点推定版）
# ========================================

def project_no_dist(params, points_3d):
    """歪みなし（10変数）"""
    fx, fy, cx, cy = params[0:4]
    rvec = params[4:7]
    tvec = params[7:10]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, None
    )
    return projected.reshape(-1, 2)


def project_dist4(params, points_3d):
    """4係数歪み（14変数）"""
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


def project_dist5(params, points_3d):
    """5係数歪み（15変数）"""
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


# ========================================
# 残差関数
# ========================================

def make_residual_fix_center(project_func, cx, cy):
    def residual(params, points_3d, points_2d):
        projected = project_func(params, points_3d, cx, cy)
        return (projected - points_2d).flatten()
    return residual


def make_residual(project_func):
    def residual(params, points_3d, points_2d):
        projected = project_func(params, points_3d)
        return (projected - points_2d).flatten()
    return residual


# ========================================
# K既知モード用関数
# ========================================

def load_intrinsic_toml(toml_path: str, camera_name: str) -> dict:
    """TOMLファイルから内部パラメータを読み込む"""
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
        'K': K,
        'dist': dist,
        'image_width': int(size[0]),
        'image_height': int(size[1]),
    }


def estimate_extrinsic(points_3d: np.ndarray, points_2d: np.ndarray,
                       K: np.ndarray, dist: np.ndarray,
                       point_names: list[str]) -> dict:
    """外部パラメータ推定のメインロジック（K既知モード）"""
    num_points = len(points_3d)

    if num_points <= 3:
        print(f"エラー: 基準点が{num_points}点しかありません。最低4点必要です。")
        return None

    if num_points >= 6:
        # Step 1: RANSAC で外れ値検出
        success, rvec, tvec, inliers_idx = cv2.solvePnPRansac(
            points_3d, points_2d, K, dist,
            iterationsCount=1000,
            reprojectionError=8.0,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success or inliers_idx is None:
            print("エラー: solvePnPRansac が失敗しました")
            return None

        # inlier/outlier を分類
        inlier_set = set(inliers_idx.flatten().tolist())
        inlier_names = [point_names[i] for i in range(num_points) if i in inlier_set]
        outlier_names = [point_names[i] for i in range(num_points) if i not in inlier_set]

        inlier_3d = points_3d[list(inlier_set)]
        inlier_2d = points_2d[list(inlier_set)]

        if len(inlier_3d) < 4:
            print(f"エラー: inlier数が4点未満です（{len(inlier_3d)}点）。外れ値が多すぎます。")
            return None

        # Step 2: inlier点で ITERATIVE 精密化
        success, rvec, tvec = cv2.solvePnP(
            inlier_3d, inlier_2d, K, dist,
            rvec=rvec, tvec=tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            print("エラー: solvePnP(ITERATIVE) 精密化が失敗しました")
            return None

        used_ransac = True

    else:  # 4〜5点
        # SOLVEPNP_ITERATIVE は最低6点必要なため、SOLVEPNP_SQPNP を使用（4点以上対応）
        success, rvec, tvec = cv2.solvePnP(
            points_3d, points_2d, K, dist,
            flags=cv2.SOLVEPNP_SQPNP
        )

        if not success:
            print("エラー: solvePnP が失敗しました")
            return None

        inlier_names = list(point_names)
        outlier_names = []
        used_ransac = False

    return {
        'rvec': rvec.flatten(),
        'tvec': tvec.flatten(),
        'inliers': inlier_names,
        'outliers': outlier_names,
        'used_ransac': used_ransac,
    }


def compute_reprojection_errors(points_3d: np.ndarray, points_2d: np.ndarray,
                                K: np.ndarray, dist: np.ndarray,
                                rvec: np.ndarray, tvec: np.ndarray) -> tuple[float, np.ndarray]:
    """再投影誤差を計算（全点対象）"""
    projected, _ = cv2.projectPoints(
        points_3d, rvec.reshape(3, 1), tvec.reshape(3, 1), K, dist
    )
    projected = projected.reshape(-1, 2)

    per_point_errors = np.linalg.norm(projected - points_2d, axis=1)
    rmse = np.sqrt(np.mean(per_point_errors ** 2))

    return rmse, per_point_errors


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
    """標準出力にTOMLセクションを表示する"""
    print()
    print(_format_toml_section(camera_name, image_width, image_height,
                               K, dist, rvec, tvec))


def _write_toml_output(output_path: str, results: dict):
    """推定結果をTOMLファイルに書き出す"""
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


def _run_extrinsic_estimation(config_path: str, toml_path: str, output_path: str = None):
    """K既知モードの処理（複数カメラ対応）"""

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

        # 外部パラメータ
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

        # TOML出力
        print("\n" + "=" * 60)
        print("Calib_scene.toml 形式")
        print("=" * 60)

        _print_toml_output(target_camera, img_width, img_height, K, dist, rvec, tvec)

        # TOML出力用に保存（numpy配列のまま保持し、数値精度を統一する）
        results[target_camera] = {
            'name': target_camera,
            'size': [float(img_width), float(img_height)],
            'K': K,
            'dist': dist,
            'rvec': rvec,
            'tvec': tvec,
        }

    # ファイル出力
    if output_path:
        if results:
            _write_toml_output(output_path, results)
        else:
            print("\nエラー: 全カメラの推定に失敗しました。ファイルは出力しません。")

    return 0 if results else 1


def run_estimation(config_path: str, use_k3: bool, use_wide: bool, fix_center: bool,
                   intrinsic_toml: str = None, output_path: str = None):
    """メイン処理"""

    # K既知モード
    if intrinsic_toml:
        return _run_extrinsic_estimation(config_path, intrinsic_toml, output_path)

    config = load_yaml_simple(config_path)
    config_dir = Path(config_path).parent
    
    target_camera = config['target_camera']
    points_3d_path = config_dir / config['points_3d']
    points_2d_path = config_dir / config['points_2d']
    
    img_width = int(config.get('image_width', 960))
    img_height = int(config.get('image_height', 540))
    
    # 主点（固定する場合）
    cx_fixed = img_width / 2
    cy_fixed = img_height / 2
    
    print("=" * 60)
    print(f"カメラ パラメータ推定: {target_camera}")
    print("=" * 60)
    print(f"設定ファイル: {config_path}")
    print(f"3D座標: {points_3d_path}")
    print(f"2D座標: {points_2d_path}")
    print(f"画像サイズ: {img_width} x {img_height}")
    print(f"歪みモデル: {'8係数（広角）' if use_wide else ('5係数' if use_k3 else '4係数')}")
    print(f"主点固定: {'有効 (cx={}, cy={})'.format(cx_fixed, cy_fixed) if fix_center else '無効'}")
    
    # データ読み込み
    points_3d_dict = load_points_3d(points_3d_path)
    points_2d_dict = load_points_2d(points_2d_path, target_camera)
    
    # マッチング
    points_3d, points_2d, point_names = match_points(points_3d_dict, points_2d_dict)
    num_points = len(point_names)
    
    print(f"\n基準点数: {num_points}点")
    for name in point_names:
        print(f"  - {name}")
    
    # 必要点数の決定
    if use_wide:
        min_points = MIN_POINTS_DIST8_FIXCENTER if fix_center else MIN_POINTS_DIST8
    elif use_k3:
        min_points = MIN_POINTS_DIST5_FIXCENTER if fix_center else MIN_POINTS_DIST5
    else:
        min_points = MIN_POINTS_DIST4_FIXCENTER if fix_center else MIN_POINTS_DIST4
    
    estimate_distortion = num_points >= min_points
    
    # モード決定のログ
    if use_wide:
        mode_str = ("主点固定" if fix_center else "主点推定") + " + 8係数広角"
        if fix_center:
            mode_str += "（16変数）"
        else:
            mode_str += "（18変数）"
    elif fix_center:
        mode_str = "主点固定"
        if use_k3:
            mode_str += " + k3あり（13変数）"
        else:
            mode_str += " + k3なし（12変数）"
    else:
        mode_str = "主点推定"
        if use_k3:
            mode_str += " + k3あり（15変数）"
        else:
            mode_str += " + k3なし（14変数）"
    
    if estimate_distortion:
        print(f"\n→ {num_points}点 >= {min_points}点: 歪み係数を推定します")
        print(f"   モード: {mode_str}")
    else:
        print(f"\n→ {num_points}点 < {min_points}点: 歪み係数は0に固定します")
    
    # ========================================
    # パラメータ推定
    # ========================================
    print("\n" + "=" * 60)
    print("パラメータ推定")
    print("=" * 60)
    
    # 初期値
    fx_init = fy_init = max(img_width, img_height)
    cx_init, cy_init = img_width / 2, img_height / 2
    
    K_init = np.array([[fx_init, 0, cx_init], [0, fy_init, cy_init], [0, 0, 1]])
    success, rvec_init, tvec_init = cv2.solvePnP(
        points_3d, points_2d, K_init, None, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        print("エラー: solvePnP による初期値推定が失敗しました")
        return 1
    rvec_init = rvec_init.flatten()
    tvec_init = tvec_init.flatten()
    
    print(f"\n[初期値]")
    print(f"fx, fy: {fx_init:.1f}, {fy_init:.1f}")
    if fix_center:
        print(f"cx, cy: {cx_fixed:.1f}, {cy_fixed:.1f} (固定)")
    else:
        print(f"cx, cy: {cx_init:.1f}, {cy_init:.1f}")
    
    # ========================================
    # 主点固定モード
    # ========================================
    if fix_center:
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
            # 5係数歪み、主点固定（13変数）
            params_init = np.array([
                fx_init, fy_init,
                0.0, 0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2, k3
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual_fix_center(project_dist5_fix_center, cx_fixed, cy_fixed)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt = params_opt[0:2]
            cx_opt, cy_opt = cx_fixed, cy_fixed
            k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = params_opt[2:7]
            rvec_opt = params_opt[7:10]
            tvec_opt = params_opt[10:13]
            
            projected_opt = project_dist5_fix_center(params_opt, points_3d, cx_fixed, cy_fixed)
            
        elif estimate_distortion and not use_k3:
            # 4係数歪み、主点固定（12変数）
            params_init = np.array([
                fx_init, fy_init,
                0.0, 0.0, 0.0, 0.0,  # k1, k2, p1, p2
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual_fix_center(project_dist4_fix_center, cx_fixed, cy_fixed)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt = params_opt[0:2]
            cx_opt, cy_opt = cx_fixed, cy_fixed
            k1_opt, k2_opt, p1_opt, p2_opt = params_opt[2:6]
            k3_opt = 0.0
            rvec_opt = params_opt[6:9]
            tvec_opt = params_opt[9:12]
            
            projected_opt = project_dist4_fix_center(params_opt, points_3d, cx_fixed, cy_fixed)
            
        else:
            # 歪みなし、主点固定（8変数）
            params_init = np.array([
                fx_init, fy_init,
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual_fix_center(project_no_dist_fix_center, cx_fixed, cy_fixed)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt = params_opt[0:2]
            cx_opt, cy_opt = cx_fixed, cy_fixed
            k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = 0.0, 0.0, 0.0, 0.0, 0.0
            rvec_opt = params_opt[2:5]
            tvec_opt = params_opt[5:8]
            
            projected_opt = project_no_dist_fix_center(params_opt, points_3d, cx_fixed, cy_fixed)
    
    # ========================================
    # 主点推定モード
    # ========================================
    else:
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
            # 5係数歪み（15変数）
            params_init = np.array([
                fx_init, fy_init, cx_init, cy_init,
                0.0, 0.0, 0.0, 0.0, 0.0,
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual(project_dist5)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
            k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = params_opt[4:9]
            rvec_opt = params_opt[9:12]
            tvec_opt = params_opt[12:15]
            
            projected_opt = project_dist5(params_opt, points_3d)
            
        elif estimate_distortion and not use_k3:
            # 4係数歪み（14変数）
            params_init = np.array([
                fx_init, fy_init, cx_init, cy_init,
                0.0, 0.0, 0.0, 0.0,
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual(project_dist4)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
            k1_opt, k2_opt, p1_opt, p2_opt = params_opt[4:8]
            k3_opt = 0.0
            rvec_opt = params_opt[8:11]
            tvec_opt = params_opt[11:14]
            
            projected_opt = project_dist4(params_opt, points_3d)
            
        else:
            # 歪みなし（10変数）
            params_init = np.array([
                fx_init, fy_init, cx_init, cy_init,
                rvec_init[0], rvec_init[1], rvec_init[2],
                tvec_init[0], tvec_init[1], tvec_init[2]
            ])
            
            residual_func = make_residual(project_no_dist)
            result = least_squares(residual_func, params_init, args=(points_3d, points_2d), method='lm', verbose=0)
            
            params_opt = result.x
            fx_opt, fy_opt, cx_opt, cy_opt = params_opt[0:4]
            k1_opt, k2_opt, p1_opt, p2_opt, k3_opt = 0.0, 0.0, 0.0, 0.0, 0.0
            rvec_opt = params_opt[4:7]
            tvec_opt = params_opt[7:10]
            
            projected_opt = project_no_dist(params_opt, points_3d)
    
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
    if fix_center:
        print(f"cx: {cx_opt:.2f} (固定)")
        print(f"cy: {cy_opt:.2f} (固定)")
    else:
        print(f"cx: {cx_opt:.2f}")
        print(f"cy: {cy_opt:.2f}")
    
    print(f"\n[歪み係数]")
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
    
    if use_wide and estimate_distortion:
        print("注意: distortions が8係数（k1,k2,p1,p2,k3,k4,k5,k6）です。")
        print("      convert_toml_to_csv.py は先頭4係数のみ読み込むため、k3-k6は無視されます。")
        dist_str = f"[{k1_opt}, {k2_opt}, {p1_opt}, {p2_opt}, {k3_opt}, {k4_opt}, {k5_opt}, {k6_opt}]"
    elif use_k3 and estimate_distortion:
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
    
    if use_wide and estimate_distortion:
        print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6,r1,r2,r3,t1,t2,t3")
        print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},"
              f"{k1_opt},{k2_opt},{p1_opt},{p2_opt},{k3_opt},{k4_opt},{k5_opt},{k6_opt},"
              f"{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")
    elif use_k3 and estimate_distortion:
        print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3,r1,r2,r3,t1,t2,t3")
        print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},{k1_opt},{k2_opt},{p1_opt},{p2_opt},{k3_opt},{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")
    else:
        print("camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,r1,r2,r3,t1,t2,t3")
        print(f"{target_camera},{img_width},{img_height},{fx_opt},{fy_opt},{cx_opt},{cy_opt},{k1_opt},{k2_opt},{p1_opt},{p2_opt},{rvec_opt[0]},{rvec_opt[1]},{rvec_opt[2]},{tvec_opt[0]},{tvec_opt[1]},{tvec_opt[2]}")

    return 0


def main():
    parser = argparse.ArgumentParser(
        description='カメラ パラメータ推定（主点固定・広角レンズ・K既知対応版）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
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
    )
    parser.add_argument('config', help='設定ファイル (config.yaml)')
    parser.add_argument('--k3', action='store_true',
                        help='k3（6次放射歪み係数）も推定する')
    parser.add_argument('--wide', action='store_true',
                        help='広角レンズ用8係数モデルを使用')
    parser.add_argument('--fix-center', action='store_true',
                        help='主点(cx, cy)を画像中心に固定する')
    parser.add_argument('--intrinsic-toml', default=None,
                        help='内部パラメータTOMLファイル（指定時はK既知モード）')
    parser.add_argument('--output', default=None,
                        help='推定結果の出力先TOMLファイル（K既知モードのみ）')

    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}")
        return 1

    # --intrinsic-toml 指定時の警告と存在確認
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

        if not Path(args.intrinsic_toml).exists():
            print(f"エラー: TOMLファイルが見つかりません: {args.intrinsic_toml}")
            return 1

    # --output が --intrinsic-toml なしで指定された場合の警告
    if args.output and not args.intrinsic_toml:
        print("警告: --output は --intrinsic-toml と併用時のみ有効です。無視します。")

    # --output の出力先ディレクトリの存在確認
    if args.output and args.intrinsic_toml:
        output_dir = Path(args.output).parent
        if not output_dir.exists():
            print(f"エラー: 出力先ディレクトリが存在しません: {output_dir}")
            return 1

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

    # --wide が指定されたら --k3 は無視
    use_k3 = args.k3 and not args.wide

    result = run_estimation(args.config, use_k3, args.wide, args.fix_center,
                            intrinsic_toml=args.intrinsic_toml,
                            output_path=args.output if args.intrinsic_toml else None)
    if result:
        return result
    return 0


if __name__ == '__main__':
    exit(main())

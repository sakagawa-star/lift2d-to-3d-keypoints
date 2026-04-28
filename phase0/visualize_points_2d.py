"""2D座標を静止画上にプロットして可視化するスクリプト

config.yaml に記述された image_<camera_name> キーから画像パスを取得し、
points_2d.csv の2D点を画像に重ねて描画した結果を保存する。
"""

import argparse
import sys
from pathlib import Path

import cv2

from common import load_points_2d, load_yaml_simple


def resolve_target_camera(config: dict, camera_arg: str | None) -> str:
    """対象カメラ名を決定する"""
    if camera_arg:
        return camera_arg.strip()

    target = config.get('target_camera', '').strip()
    if not target:
        print("エラー: config に target_camera が定義されていません", file=sys.stderr)
        sys.exit(1)

    if ',' in target:
        print(
            "エラー: target_camera が複数指定されています。"
            "--camera <camera_name> で対象カメラを1つ指定してください",
            file=sys.stderr,
        )
        sys.exit(1)

    return target


def resolve_image_path(config: dict, camera_name: str, config_dir: Path) -> Path:
    """対象カメラの画像パスを config から取得する（config_dir 基準で解決）"""
    key = f"image_{camera_name}"
    if key not in config:
        print(
            f"エラー: config にキー '{key}' がありません。"
            f"カメラ {camera_name} の画像パスを '{key}: <path>' で追記してください",
            file=sys.stderr,
        )
        sys.exit(1)

    image_path = config_dir / config[key]
    if not image_path.exists():
        print(f"エラー: 画像ファイルが見つかりません: {image_path}", file=sys.stderr)
        sys.exit(1)

    return image_path


def draw_points_on_image(
    image_path: Path,
    points: dict,
    output_path: Path,
    config: dict,
) -> tuple[int, int]:
    """画像に2D点を描画して保存する

    戻り値: (描画点数, 範囲外スキップ数)
    """
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"エラー: 画像の読み込みに失敗しました: {image_path}", file=sys.stderr)
        sys.exit(1)

    h, w = img.shape[:2]

    cfg_w = int(config['image_width']) if 'image_width' in config else None
    cfg_h = int(config['image_height']) if 'image_height' in config else None
    if cfg_w is not None and cfg_h is not None and (cfg_w != w or cfg_h != h):
        print(
            f"警告: 画像サイズ ({w}x{h}) が config の image_width/height "
            f"({cfg_w}x{cfg_h}) と一致しません",
            file=sys.stderr,
        )

    drawn = 0
    skipped = 0
    for name, (x, y) in points.items():
        ix, iy = int(round(x)), int(round(y))
        if ix < 0 or ix >= w or iy < 0 or iy >= h:
            print(
                f"警告: 点 {name} ({x}, {y}) は画像範囲外のためスキップ",
                file=sys.stderr,
            )
            skipped += 1
            continue
        cv2.circle(img, (ix, iy), radius=8, color=(0, 0, 255), thickness=2)
        cv2.circle(img, (ix, iy), radius=1, color=(0, 255, 255), thickness=-1)
        drawn += 1

    cv2.imwrite(str(output_path), img)
    return drawn, skipped


def main() -> int:
    parser = argparse.ArgumentParser(
        description="2D座標を静止画上にプロットして可視化"
    )
    parser.add_argument("config", help="設定YAMLファイル")
    parser.add_argument(
        "--camera",
        default=None,
        help="可視化対象のカメラ名（target_camera が複数指定の場合は必須）",
    )
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"エラー: 設定ファイルが見つかりません: {args.config}", file=sys.stderr)
        return 1

    config = load_yaml_simple(args.config)
    camera_name = resolve_target_camera(config, args.camera)
    config_dir = Path(args.config).resolve().parent

    if 'points_2d' not in config:
        print("エラー: config に points_2d が定義されていません", file=sys.stderr)
        return 1

    points_2d_path = config_dir / config['points_2d']
    points = load_points_2d(str(points_2d_path), camera_name)
    if not points:
        print(
            f"エラー: カメラ {camera_name} の2D点が {points_2d_path} に1件もありません",
            file=sys.stderr,
        )
        return 1

    image_path = resolve_image_path(config, camera_name, config_dir)
    output_path = image_path.with_name(
        f"{image_path.stem}_annotated{image_path.suffix}"
    )

    drawn, skipped = draw_points_on_image(image_path, points, output_path, config)

    print(f"カメラ: {camera_name}")
    print(f"入力画像: {image_path}")
    print(f"出力画像: {output_path}")
    print(f"描画点数: {drawn} / 範囲外スキップ: {skipped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

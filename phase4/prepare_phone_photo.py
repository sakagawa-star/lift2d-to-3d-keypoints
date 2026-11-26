"""feat-041: スマートフォン写真の前処理（FR-001）

原本JPEGを中央で16:9に切り抜いて1920x1080に縮小したPNGにし、写真情報ファイル（35mm換算焦点距離。
extract_photo_info.py が Exif から書き出すか手で書く）から初期内部パラメータ
TOML（Calib_scene.toml形式、1カメラ分）を作り、原本座標の2D点CSVを前処理後画像の座標に
変換する。

実行方法（プロジェクトルートで）:
    uv run --project phase4 python phase4/prepare_phone_photo.py \\
        --image <原本JPEG> --photo-info <写真情報ファイル> --camera <カメラ名> \\
        --images-dir <前処理後画像の出力ディレクトリ> --init-toml <初期TOML> \\
        --points-2d-original <原本座標の2D点CSV> --points-2d <前処理後座標の2D点CSV> \\
        [--overwrite]
    設定YAMLで指定する場合（推定 estimate_intrinsics_loop.py と共用の設定YAMLを渡してよい。feat-043）:
    uv run --project phase4 python phase4/prepare_phone_photo.py --config job.yaml

終了コード: 0=成功、1=入力エラー・検査不合格・出力済み、2=引数エラー（argparse）

設計根拠: docs/issues/feat-041-intrinsic-loop-render-match/design.md §4.1
"""
import argparse
import math
import os
import sys
from pathlib import Path

import cv2

from refine_extrinsics import (
    InputError,
    _format_toml_value,
    _yaml_str,
    load_points_2d_csv,
    parse_args_with_config,
)
from render_fps_video import _yaml_bool, load_yaml_flat

OUT_W, OUT_H = 1920, 1080
DIAG_35MM = 43.27
MIN_POINTS = 6

# Exif タグ番号
_TAG_ORIENTATION = 0x0112
_TAG_DIGITAL_ZOOM = 0xA404
_TAG_FOCAL_35MM = 0xA405
_IFD_EXIF = 0x8769


def read_exif(path: str) -> dict:
    """Exifから必要項目を読む。無い項目は None。"Size" に画像サイズ (W, H) を含む。"""
    from PIL import Image  # 遅延import（ルート環境はPillowなし）

    with Image.open(path) as im:
        exif = im.getexif()
        merged = dict(exif)
        try:
            merged.update(exif.get_ifd(_IFD_EXIF))
        except Exception:
            pass
        size = im.size

    def _get(tag, conv):
        v = merged.get(tag)
        if v is None:
            return None
        try:
            return conv(v)
        except (TypeError, ValueError):
            return None

    return {
        "Orientation": _get(_TAG_ORIENTATION, int),
        "DigitalZoomRatio": _get(_TAG_DIGITAL_ZOOM, float),
        "Size": (int(size[0]), int(size[1])),
    }


def validate_exif(exif: dict, width: int, height: int) -> list[str]:
    """Exif検査。違反をすべて独立に評価して列挙する。空リストなら合格。"""
    errors: list[str] = []
    ori = exif.get("Orientation")
    if ori is not None and ori != 1:
        errors.append(f"Orientation が 1 ではありません: {ori}")
    if width <= height:
        errors.append(f"縦長または正方形の画像です（幅{width} <= 高さ{height}）")
    dz = exif.get("DigitalZoomRatio")
    if dz is not None and dz != 1.0:
        errors.append(f"DigitalZoomRatio が 1.0 ではありません: {dz}")
    return errors


PHOTO_INFO_KEY = "focal_length_35mm"


def load_photo_info(path: str) -> float:
    """写真情報ファイルから 35mm 換算焦点距離を読む（feat-044）。不正は InputError。"""
    try:
        d = load_yaml_flat(path)
    except (OSError, UnicodeDecodeError) as e:
        raise InputError(f"写真情報ファイルを読み込めません: {path}（{e}）") from e
    unknown = sorted(k for k in d if k != PHOTO_INFO_KEY)
    if unknown:
        raise InputError(f"写真情報ファイルに未知のキーがあります: {', '.join(unknown)}"
                         f"（使えるキーは focal_length_35mm だけ）: {path}")
    if PHOTO_INFO_KEY not in d:
        raise InputError(f"写真情報ファイルに focal_length_35mm がありません: {path}")
    try:
        v = float(d[PHOTO_INFO_KEY])
    except ValueError:
        v = float("nan")
    if not math.isfinite(v) or v <= 0:
        raise InputError(f"写真情報ファイルの focal_length_35mm が正の数ではありません: "
                         f"{d[PHOTO_INFO_KEY]}: {path}")
    return v


def compute_crop(width: int, height: int) -> tuple[int, int]:
    """16:9 の切り抜き範囲 (y0, crop_h)。不正は ValueError。"""
    if width * 9 % 16 != 0:
        raise ValueError(f"幅 {width} が 16 の倍数ではないため 16:9 に切り抜けません")
    crop_h = width * 9 // 16
    if height < crop_h:
        raise ValueError(f"高さ {height} が 16:9 の切り抜き高さ {crop_h} 未満です")
    y0 = (height - crop_h) // 2
    return y0, crop_h


def initial_focal_px(f35: float, width: int, height: int, out_w: int = OUT_W) -> float:
    """初期焦点距離[px]（前処理後画像基準）"""
    return f35 * math.hypot(width, height) / DIAG_35MM * (out_w / width)


def convert_point(x: float, y: float, y0: int, scale: float) -> tuple[float, float]:
    """原本座標→前処理後座標（画素中心規約）"""
    return (x + 0.5) * scale - 0.5, (y - y0 + 0.5) * scale - 0.5


def format_init_toml(camera: str, f: float) -> str:
    """初期TOML文字列"""
    f = float(f)
    lines = [
        f"[{camera}]",
        f"name = {_format_toml_value(camera)}",
        f"size = {_format_toml_value([float(OUT_W), float(OUT_H)])}",
        "matrix = " + _format_toml_value(
            [[f, 0.0, 960.0], [0.0, f, 540.0], [0.0, 0.0, 1.0]]),
        "distortions = " + _format_toml_value([0.0, 0.0, 0.0, 0.0]),
        "rotation = " + _format_toml_value([0.0, 0.0, 0.0]),
        "translation = " + _format_toml_value([0.0, 0.0, 0.0]),
        "fisheye = false",
    ]
    return "\n".join(lines) + "\n"


def _write_atomic_bytes(path: Path, data: bytes) -> None:
    tmp = Path(str(path) + ".tmp")
    tmp.write_bytes(data)
    tmp.replace(path)


def _write_atomic_text(path: Path, text: str) -> None:
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="スマートフォン写真の前処理（feat-041 FR-001）",
                                allow_abbrev=False)
    p.add_argument("--image", default=None, help="原本JPEG（--config でも指定可）")
    p.add_argument("--photo-info", default=None,
                   help="写真情報ファイル（35mm換算焦点距離。extract_photo_info.py が書き出すか手で書く）"
                        "（--config でも指定可）")
    p.add_argument("--camera", default=None, help="カメラ名（--config でも指定可）")
    p.add_argument("--images-dir", default=None,
                   help="前処理後画像の出力ディレクトリ（画像は <images-dir>/<camera>.png）（--config でも指定可）")
    p.add_argument("--init-toml", default=None, help="初期TOMLの出力パス（--config でも指定可）")
    p.add_argument("--points-2d-original", default=None, help="原本座標の2D点CSV（--config でも指定可）")
    p.add_argument("--points-2d", default=None, help="前処理後座標の2D点CSVの出力パス（--config でも指定可）")
    p.add_argument("--overwrite", action="store_true", help="出力ファイルの上書きを許可する（--config でも指定可）")
    p.add_argument("--config", default=None,
                   help="設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。"
                        "優先順位は CLI > 設定YAML > 既定値。フラグ（--overwrite）は CLI から true 方向にだけ"
                        "上書きできる（設定YAMLの true を false に戻すには設定YAMLを編集する）。"
                        "推定（estimate_intrinsics_loop.py）と共用の設定YAMLを渡してよい（推定専用のキーは無視する）")
    return p


CONFIG_CONVERTERS = {
    "image": _yaml_str, "photo_info": _yaml_str, "camera": _yaml_str, "images_dir": _yaml_str, "init_toml": _yaml_str,
    "points_2d_original": _yaml_str, "points_2d": _yaml_str,
    "overwrite": _yaml_bool,
}
REQUIRED_OPTIONS = [("image", "--image"), ("photo_info", "--photo-info"), ("camera", "--camera"), ("images_dir", "--images-dir"),
                    ("init_toml", "--init-toml"), ("points_2d_original", "--points-2d-original"),
                    ("points_2d", "--points-2d")]
# 共用 YAML の相手（推定 estimate_intrinsics_loop.py）のキー（feat-043）
IGNORED_KEYS = frozenset({"points_3d", "ply", "out_dir", "tmp_dir"})


def _run(args) -> int:
    camera = args.camera
    if (camera in ("", ".", "..") or Path(camera).name != camera
            or "/" in camera or "\\" in camera):
        print(f"エラー: カメラ名にはパス区切りを含まない名前を指定してください: {camera}", file=sys.stderr)
        return 1
    out_png = Path(args.images_dir) / f"{camera}.png"
    out_toml = Path(args.init_toml)
    out_csv = Path(args.points_2d)

    # 0b. 写真情報ファイルと出力の衝突（feat-044）
    outs = [out_png, out_toml, out_csv]
    outs += [Path(str(p) + ".tmp") for p in outs]   # _write_atomic_* の一時ファイル
    pi = Path(args.photo_info)
    for o in outs:
        same = os.path.realpath(pi) == os.path.realpath(o)
        if not same and os.path.exists(pi) and os.path.exists(o):
            same = os.path.samefile(pi, o)          # ハードリンクによる別名
        if same:
            print(f"エラー: --photo-info が前処理の出力（または書き込み途中の一時ファイル）と同じファイルを指しています: {o}",
                  file=sys.stderr)
            return 1

    # 1. 出力済み検査
    if not args.overwrite:
        for pth in (out_png, out_toml, out_csv):
            if pth.exists():
                print(f"エラー: 出力ファイルが既に存在します（--overwrite で上書き可）: {pth}",
                      file=sys.stderr)
                return 1

    # 1b. 写真情報ファイル（feat-044）
    try:
        f35 = load_photo_info(args.photo_info)
    except InputError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    # 2. Exif検査
    exif = read_exif(args.image)
    width, height = exif["Size"]
    errors = validate_exif(exif, width, height)
    if errors:
        for e in errors:
            print(f"エラー: {e}", file=sys.stderr)
        return 1

    # 3. 切り抜き範囲
    try:
        y0, crop_h = compute_crop(width, height)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    # 4. 画像の切り抜き・縮小
    img = cv2.imread(args.image, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if img is None:
        print(f"エラー: 画像を読み込めません: {args.image}", file=sys.stderr)
        return 1
    if img.shape[1] != width or img.shape[0] != height:
        print(f"エラー: 画像サイズが Exif 側の読み取り値と一致しません"
              f"（cv2 {img.shape[1]}x{img.shape[0]}、Exif {width}x{height}）", file=sys.stderr)
        return 1
    cropped = img[y0:y0 + crop_h, :]
    resized = cv2.resize(cropped, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)

    # 5. 初期焦点距離
    f = initial_focal_px(f35, width, height, OUT_W)

    # 6. 2D点の変換
    try:
        pts_in = load_points_2d_csv(Path(args.points_2d_original))
    except InputError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1
    scale = OUT_W / width
    rows: list[tuple[str, float, float]] = []
    excluded: list[str] = []
    for (cam, name), xy in pts_in.items():
        if cam != camera:
            continue
        x, y = convert_point(float(xy[0]), float(xy[1]), y0, scale)
        if 0 <= x <= OUT_W - 1 and 0 <= y <= OUT_H - 1:
            rows.append((name, x, y))
        else:
            excluded.append(name)
    if excluded:
        print(f"切り抜き範囲外のため除外した2D点: {excluded}")
    if len({r[0] for r in rows}) < MIN_POINTS:
        print(f"エラー: 変換後の一意ObjectNameが {MIN_POINTS} 個未満です（{len(rows)}個）",
              file=sys.stderr)
        return 1

    # 7. 出力
    ok, buf = cv2.imencode(".png", resized)
    if not ok:
        print("エラー: PNGエンコードに失敗しました", file=sys.stderr)
        return 1
    out_png.parent.mkdir(parents=True, exist_ok=True)
    _write_atomic_bytes(out_png, buf.tobytes())
    out_toml.parent.mkdir(parents=True, exist_ok=True)
    _write_atomic_text(out_toml, format_init_toml(camera, f))
    csv_lines = ["ObjectName,camera_name,X,Y"]
    for name, x, y in rows:
        csv_lines.append(f"{name},{camera},{round(x, 3)},{round(y, 3)}")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    _write_atomic_text(out_csv, "\n".join(csv_lines) + "\n")

    print(f"原本サイズ: {width}x{height}  切り抜き: y0={y0}, 高さ={crop_h}")
    print(f"初期焦点距離 f = {f}")
    print(f"出力画像: {out_png}")
    print(f"初期TOML: {out_toml}")
    print(f"2D点CSV: {out_csv}（{len(rows)}点）")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)
    output_options = [("images_dir", "--images-dir"), ("init_toml", "--init-toml"), ("points_2d", "--points-2d")]
    empty = [label for dest, label in output_options if getattr(args, dest) == ""]
    if empty:  # 空のパスは書き込み先が実行ディレクトリになる・置換で途中失敗する
        parser.error(f"空の値は指定できません: {', '.join(empty)}")
    return _run(args)


if __name__ == "__main__":
    sys.exit(main())

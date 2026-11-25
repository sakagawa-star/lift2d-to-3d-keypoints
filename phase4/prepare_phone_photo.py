"""feat-041: スマートフォン写真の前処理（FR-001）

原本JPEGを中央で16:9に切り抜いて1920x1080に縮小したPNGにし、Exifから初期内部パラメータ
TOML（Calib_scene.toml形式、1カメラ分）を作り、原本座標の2D点CSVを前処理後画像の座標に
変換する。

実行方法（プロジェクトルートで）:
    uv run --project phase4 python phase4/prepare_phone_photo.py \\
        --image <原本JPEG> --camera <カメラ名> \\
        --images-dir <前処理後画像の出力ディレクトリ> --init-toml <初期TOML> \\
        --points-2d-original <原本座標の2D点CSV> --points-2d <前処理後座標の2D点CSV> \\
        [--expect-model "Galaxy S25"] [--expect-focal-mm 5.4] [--overwrite]
    設定YAMLで指定する場合（推定 estimate_intrinsics_loop.py と共用の設定YAMLを渡してよい。feat-043）:
    uv run --project phase4 python phase4/prepare_phone_photo.py --config job.yaml

終了コード: 0=成功、1=入力エラー・検査不合格・出力済み、2=引数エラー（argparse）

設計根拠: docs/issues/feat-041-intrinsic-loop-render-match/design.md §4.1
"""
import argparse
import math
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
from render_fps_video import _yaml_bool

OUT_W, OUT_H = 1920, 1080
DIAG_35MM = 43.27
FOCAL_MM_TOL = 0.05
MIN_POINTS = 6

# Exif タグ番号
_TAG_MODEL = 0x0110
_TAG_ORIENTATION = 0x0112
_TAG_FOCAL_LENGTH = 0x920A
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

    model = merged.get(_TAG_MODEL)
    if isinstance(model, bytes):
        model = model.decode("utf-8", errors="replace")
    return {
        "Model": model if isinstance(model, str) else None,
        "Orientation": _get(_TAG_ORIENTATION, int),
        "FocalLength": _get(_TAG_FOCAL_LENGTH, float),
        "FocalLengthIn35mmFilm": _get(_TAG_FOCAL_35MM, int),
        "DigitalZoomRatio": _get(_TAG_DIGITAL_ZOOM, float),
        "Size": (int(size[0]), int(size[1])),
    }


def validate_exif(exif: dict, width: int, height: int,
                  expect_model: str | None, expect_focal_mm: float | None) -> list[str]:
    """Exif検査。違反をすべて独立に評価して列挙する。空リストなら合格。"""
    errors: list[str] = []
    ori = exif.get("Orientation")
    if ori is not None and ori != 1:
        errors.append(f"Orientation が 1 ではありません: {ori}")
    if width <= height:
        errors.append(f"縦長または正方形の画像です（幅{width} <= 高さ{height}）")
    f35 = exif.get("FocalLengthIn35mmFilm")
    if f35 is None or f35 <= 0:
        errors.append(f"FocalLengthIn35mmFilm がありません（値: {f35}）")
    dz = exif.get("DigitalZoomRatio")
    if dz is not None and dz != 1.0:
        errors.append(f"DigitalZoomRatio が 1.0 ではありません: {dz}")
    if expect_model is not None:
        model = exif.get("Model")
        if model is None or model.strip() != expect_model.strip():
            errors.append(f"Model が期待値と一致しません（期待 {expect_model!r}、実際 {model!r}）")
    if expect_focal_mm is not None:
        fl = exif.get("FocalLength")
        if fl is None:
            errors.append("FocalLength がありません")
        elif abs(fl - expect_focal_mm) > FOCAL_MM_TOL:
            errors.append(
                f"FocalLength が期待値と異なります（期待 {expect_focal_mm}mm、実際 {fl}mm、"
                f"許容 {FOCAL_MM_TOL}mm）")
    return errors


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
    p.add_argument("--camera", default=None, help="カメラ名（--config でも指定可）")
    p.add_argument("--images-dir", default=None,
                   help="前処理後画像の出力ディレクトリ（画像は <images-dir>/<camera>.png）（--config でも指定可）")
    p.add_argument("--init-toml", default=None, help="初期TOMLの出力パス（--config でも指定可）")
    p.add_argument("--points-2d-original", default=None, help="原本座標の2D点CSV（--config でも指定可）")
    p.add_argument("--points-2d", default=None, help="前処理後座標の2D点CSVの出力パス（--config でも指定可）")
    p.add_argument("--expect-model", default=None, help="期待する機種名（Exif Model）（--config でも指定可）")
    p.add_argument("--expect-focal-mm", type=float, default=None,
                   help="期待する実焦点距離[mm]（--config でも指定可）")
    p.add_argument("--overwrite", action="store_true", help="出力ファイルの上書きを許可する（--config でも指定可）")
    p.add_argument("--config", default=None,
                   help="設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。"
                        "優先順位は CLI > 設定YAML > 既定値。フラグ（--overwrite）は CLI から true 方向にだけ"
                        "上書きできる（設定YAMLの true を false に戻すには設定YAMLを編集する）。"
                        "推定（estimate_intrinsics_loop.py）と共用の設定YAMLを渡してよい（推定専用のキーは無視する）")
    return p


CONFIG_CONVERTERS = {
    "image": _yaml_str, "camera": _yaml_str, "images_dir": _yaml_str, "init_toml": _yaml_str,
    "points_2d_original": _yaml_str, "points_2d": _yaml_str,
    "expect_model": _yaml_str, "expect_focal_mm": float, "overwrite": _yaml_bool,
}
REQUIRED_OPTIONS = [("image", "--image"), ("camera", "--camera"), ("images_dir", "--images-dir"),
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

    # 1. 出力済み検査
    if not args.overwrite:
        for pth in (out_png, out_toml, out_csv):
            if pth.exists():
                print(f"エラー: 出力ファイルが既に存在します（--overwrite で上書き可）: {pth}",
                      file=sys.stderr)
                return 1

    # 2. Exif検査
    exif = read_exif(args.image)
    width, height = exif["Size"]
    errors = validate_exif(exif, width, height, args.expect_model, args.expect_focal_mm)
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
    f = initial_focal_px(float(exif["FocalLengthIn35mmFilm"]), width, height, OUT_W)

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

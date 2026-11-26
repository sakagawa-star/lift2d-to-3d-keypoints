"""feat-044: 原本画像の Exif から写真情報ファイルを書き出す（FR-001）

原本画像の Exif から 35mm 換算焦点距離（FocalLengthIn35mmFilm）を読み、写真情報ファイル
（フラットな `key: value`。項目は focal_length_35mm の 1 つだけ）に書き出す。
写真情報ファイルは前処理 prepare_phone_photo.py の入力になる。Exif の無い画像では、
この書き出しを使わずに写真情報ファイルを手で書く。上書きはしない（出力先が既に存在すれば終了コード 1）。

実行方法（プロジェクトルートで）:
    uv run --project phase4 python phase4/extract_photo_info.py \\
        --image <原本画像> --photo-info <写真情報ファイルの出力パス。存在しない新しいパス>
    設定YAMLで指定する場合（前処理・推定と共用の設定YAMLを渡してよい。feat-043 / feat-044）:
    uv run --project phase4 python phase4/extract_photo_info.py --config job.yaml

終了コード: 0=成功、1=入力エラー・出力済み・Exif に値が無い・書き込み失敗、2=引数エラー（argparse）

設計根拠: docs/issues/feat-044-remove-expect-exif-options/design.md §4.1
"""
import argparse
import os
import sys
import tempfile
from pathlib import Path

from prepare_phone_photo import _IFD_EXIF, _TAG_FOCAL_35MM, PHOTO_INFO_KEY
from refine_extrinsics import _yaml_str, parse_args_with_config

CONFIG_CONVERTERS = {"image": _yaml_str, "photo_info": _yaml_str}
REQUIRED_OPTIONS = [("image", "--image"), ("photo_info", "--photo-info")]
# 共用 YAML の相手（前処理・推定）のキー（feat-044）
IGNORED_KEYS = frozenset({"camera", "images_dir", "init_toml", "points_2d_original", "points_2d",
                          "overwrite", "points_3d", "ply", "out_dir", "tmp_dir"})


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="原本画像の Exif から写真情報ファイルを書き出す（feat-044 FR-001）",
                                allow_abbrev=False)
    p.add_argument("--image", default=None, help="原本画像（--config でも指定可）")
    p.add_argument("--photo-info", default=None,
                   help="写真情報ファイルの出力パス。存在しない新しいパス（上書きしない）（--config でも指定可）")
    p.add_argument("--config", default=None,
                   help="設定YAML（フラット key: value）。キーはオプション名の -- を除き - を _ にしたもの。"
                        "優先順位は CLI > 設定YAML > 既定値。"
                        "前処理・推定と共用の設定YAMLを渡してよい（他のツール専用のキーは無視する）")
    return p


def read_focal_35mm(path: str) -> int | None:
    """Exif の FocalLengthIn35mmFilm を int で返す。無い・変換できない場合は None。"""
    from PIL import Image  # 遅延import（ルート環境はPillowなし）

    with Image.open(path) as im:
        exif = im.getexif()
        merged = dict(exif)
        try:
            merged.update(exif.get_ifd(_IFD_EXIF))
        except Exception:
            pass
    v = merged.get(_TAG_FOCAL_35MM)
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return None


def write_photo_info(out: Path, text: str) -> None:
    """全体を書き終えてから、出力先が存在しない場合だけ公開する。例外はそのまま送出する。"""
    out.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=out.parent, prefix=f".{out.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        os.link(tmp, out)  # out が既に存在すれば FileExistsError（確定時にも上書きしない）
    finally:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parse_args_with_config(argv, parser, CONFIG_CONVERTERS, REQUIRED_OPTIONS, IGNORED_KEYS)
    if args.photo_info == "":
        parser.error("空の値は指定できません: --photo-info")
    out = Path(args.photo_info)
    exists_msg = f"エラー: 出力先が既に存在します（上書きしません）。別の出力パスを指定してください: {out}"
    if os.path.lexists(out):
        print(exists_msg, file=sys.stderr)
        return 1
    try:
        f35 = read_focal_35mm(args.image)
    except OSError as e:
        print(f"エラー: 画像を開けません: {args.image}（{e}）", file=sys.stderr)
        return 1
    if f35 is None or f35 <= 0:
        print(f"エラー: 画像の Exif に 35mm 換算焦点距離（FocalLengthIn35mmFilm）がありません: {args.image}\n"
              f'写真情報ファイルを手で書いてください（書式: 1 行 "{PHOTO_INFO_KEY}: <35mm換算焦点距離[mm]>"）: {out}',
              file=sys.stderr)
        return 1
    text = ("# 写真情報ファイル（extract_photo_info.py が Exif から書き出し。feat-044）\n"
            f"{PHOTO_INFO_KEY}: {f35}\n")
    try:
        write_photo_info(out, text)
    except FileExistsError:
        print(exists_msg, file=sys.stderr)
        return 1
    except OSError as e:
        print(f"エラー: 写真情報ファイルを書けません: {out}（{e}）", file=sys.stderr)
        return 1
    print(f"35mm 換算焦点距離: {f35}")
    print(f"写真情報ファイル: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

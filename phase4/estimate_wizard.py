"""feat-045: スマートフォン静止画のカメラパラメータ推定ウィザード

原本 1 枚について、2D 点を打てる範囲の画像の書き出し、2D・3D 基準点の確認、入力のコピー、
共用設定YAMLの作成を対話で行い、既存の 3 ツール（書き出し extract_photo_info.py →
前処理 prepare_phone_photo.py → 推定 estimate_intrinsics_loop.py）を順に実行する。

実行方法（プロジェクトルートで。推定に GPU を使うため環境変数を付ける）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_wizard.py

終了コード: 0=成功（または実行前に n を選んだ）、1=ツールの失敗・ファイル操作の失敗・中断、
2=引数エラー（--help 以外の引数は受け付けない）

設計根拠: docs/issues/feat-045-estimate-wizard/design.md
"""
import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from extract_photo_info import read_focal_35mm
from prepare_phone_photo import (
    MIN_POINTS,
    OUT_H,
    OUT_W,
    compute_crop,
    convert_point,
    read_exif,
    validate_exif,
)
from refine_extrinsics import InputError, load_points_2d_csv, load_points_3d_csv

PHASE4_DIR = Path(__file__).resolve().parent
JOBS_DIR = PHASE4_DIR / "data" / "EIL"   # 作業フォルダの親（feat-045 イテレーション1）
JOB_NAME_RE = re.compile(r"[A-Za-z0-9_-]+")
TOOLS = ["extract_photo_info.py", "prepare_phone_photo.py", "estimate_intrinsics_loop.py"]
DARKEN = 0.5              # 範囲外の明るさの倍率
LINE_COLOR = (0, 0, 255)  # BGR 赤
CAMERA_NAME_RE = re.compile(r"[A-Za-z0-9_-]+")


# ---------------------------------------------------------------- ジョブフォルダの構成

def job_paths(job: Path, original_suffix: str) -> dict[str, Path]:
    """ジョブフォルダ内の固定の名前（job は絶対パス）。"""
    return {
        "original": job / ("original" + original_suffix),
        "crop_area": job / "crop_area.png",
        "points_check": job / "points_check.png",
        "points_2d_original": job / "points_2d_original.csv",
        "points_3d": job / "points_3d.csv",
        "ply": job / "point_cloud.ply",
        "job_yaml": job / "job.yaml",
        "photo_info": job / "photo_info.yaml",
        "images_dir": job / "images",
        "init_toml": job / "init.toml",
        "points_2d": job / "points_2d.csv",
        "out_dir": job / "out",
    }


def format_job_yaml(camera: str, p: dict[str, Path]) -> str:
    """共用設定YAML（3 ツールに --config で渡す）の中身。"""
    return (
        "# 共用設定YAML（estimate_wizard.py が作成。feat-045）\n"
        f"camera: {camera}\n"
        f"image: {p['original']}\n"
        f"photo_info: {p['photo_info']}\n"
        f"points_2d_original: {p['points_2d_original']}\n"
        f"images_dir: {p['images_dir']}\n"
        f"init_toml: {p['init_toml']}\n"
        f"points_2d: {p['points_2d']}\n"
        f"points_3d: {p['points_3d']}\n"
        f"ply: {p['ply']}\n"
        f"out_dir: {p['out_dir']}\n"
    )


# ---------------------------------------------------------------- 入力の補助関数

def ask(input_fn, label: str, default: "str | Path | None" = None) -> str:
    """label を表示して 1 行読む。前後の空白を除き、空なら default（None なら聞き直す）を返す。"""
    while True:
        prompt = f"{label} [{default}]: " if default is not None else f"{label}: "
        value = input_fn(prompt).strip()
        if value:
            return value
        if default is not None:
            return str(default)


def ask_yes_no(input_fn, label: str) -> bool:
    """y / yes → True、n / no → False（大文字小文字を区別しない）。それ以外は聞き直す。"""
    while True:
        value = input_fn(f"{label} [y/n]: ").strip().lower()
        if value in ("y", "yes"):
            return True
        if value in ("n", "no"):
            return False


def job_place_error(job: Path) -> "str | None":
    """解決後の作業フォルダの絶対パスの検査（改行・前後の空白。共用設定YAMLに書けないため）。"""
    s = str(job)
    if "\n" in s or "\r" in s or s != s.strip():
        return ("作業フォルダのパスに改行または前後の空白が含まれます"
                f"（共用設定YAMLに書けないため）: {job!r}")
    return None


def camera_name_error(name: str) -> "str | None":
    """カメラ名の規則を満たせば None。違反なら理由の文言。"""
    # 英数字・_・- だけなら TOML の裸のキーとして読め（前処理は [カメラ名] と書く）、
    # パス区切り・"."・".."・空白・改行も含まれないので、前処理・推定のカメラ名の規則も満たす
    if not CAMERA_NAME_RE.fullmatch(name):
        return (f"カメラ名は英数字・_・- だけで書いてください"
                f"（初期 TOML のテーブル名・CSV・ファイル名にそのまま使うため）: {name}")
    if name.startswith("-"):
        return f"カメラ名は - で始められません（推定が --cameras の値として渡すため）: {name}"
    if name == "metadata":
        return "カメラ名に metadata は使えません（最終精緻化が TOML の [metadata] を特別に扱うため）"
    return None


def in_range(x: float, y: float, y0: int, w: int) -> bool:
    """前処理と同じ判定: 変換後の座標が前処理後画像の範囲内なら True。"""
    u, v = convert_point(x, y, y0, OUT_W / w)
    return 0 <= u <= OUT_W - 1 and 0 <= v <= OUT_H - 1


def valid_range(w: int, h: int, y0: int) -> tuple[int, int, int, int]:
    """打てる範囲の整数座標 (x_min, x_max, y_min, y_max)。in_range と全数でちょうど一致する。"""
    y_c = y0 + 1
    xs = [x for x in range(w) if in_range(x, y_c, y0, w)]
    x_c = w // 2
    ys = [y for y in range(h) if in_range(x_c, y, y0, w)]
    return min(xs), max(xs), min(ys), max(ys)


def check_original(src: Path) -> tuple[list[str], "dict | None"]:
    """原本の判定（人が点を打つ前に行う）。誤りがあれば (文言のリスト, None)、無ければ ([], 情報)。"""
    img = cv2.imread(str(src), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if img is None:
        return [f"画像を開けません: {src}"], None
    h, w = img.shape[:2]
    try:
        y0, crop_h = compute_crop(w, h)
    except ValueError as e:
        return [str(e)], None
    try:
        exif = read_exif(str(src))
    except OSError as e:
        return [f"Exif を読めません: {src}（{e}）"], None
    errors: list[str] = []
    if tuple(exif["Size"]) != (w, h):
        errors.append(f"画像サイズが Exif 側の読み取り値と一致しません"
                      f"（cv2 {w}x{h}、Exif {exif['Size'][0]}x{exif['Size'][1]}）")
    errors += validate_exif(exif, w, h)
    try:
        f35 = read_focal_35mm(str(src))
    except OSError as e:
        return [f"Exif を読めません: {src}（{e}）"], None
    if f35 is None or f35 <= 0:
        errors.append("Exif に 35mm 換算焦点距離がありません。Exif の無い画像はウィザードでは扱えません。"
                      "写真情報ファイルを手で書き、前処理・推定を手で実行してください"
                      "（README の estimate_intrinsics_loop.py ほかの節）")
    if errors:
        return errors, None
    info = {"img": img, "w": w, "h": h, "y0": y0, "crop_h": crop_h,
            "rng": valid_range(w, h, y0)}
    return [], info


# ---------------------------------------------------------------- 範囲画像

def draw_crop_area(img: np.ndarray, rng: tuple[int, int, int, int]) -> np.ndarray:
    """打てる範囲の外を暗くし、上端・下端の行に線と y 座標の文字を描いたコピーを返す。"""
    h, w = img.shape[:2]
    x_min, x_max, y_min, y_max = rng
    out = (img * DARKEN).astype(np.uint8)
    out[y_min:y_max + 1, x_min:x_max + 1] = img[y_min:y_max + 1, x_min:x_max + 1]

    t = max(2, round(w / 800))
    cv2.line(out, (0, y_min), (w - 1, y_min), LINE_COLOR, t)
    cv2.line(out, (0, y_max), (w - 1, y_max), LINE_COLOR, t)

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = w / 1000
    th_ = max(1, round(w / 1000))
    gap = 2 * t

    text_top = f"y={y_min}"
    (_, tht), _ = cv2.getTextSize(text_top, font, scale, th_)
    base = y_min - gap
    if y_min - gap - tht < 0:
        base = y_min + gap + tht
    cv2.putText(out, text_top, (20, base), font, scale, LINE_COLOR, th_, cv2.LINE_AA)

    text_bottom = f"y={y_max}"
    (_, tht), _ = cv2.getTextSize(text_bottom, font, scale, th_)
    base = y_max + gap + tht
    if y_max + gap + tht > h - 1:
        base = y_max - gap
    cv2.putText(out, text_bottom, (20, base), font, scale, LINE_COLOR, th_, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------- 名前の照合と確認画像

def check_names(cam_pts: dict, p3: dict) -> tuple[list[str], list[str], list[str]]:
    """(両方にある名前, 2D にだけある名前, 3D にだけある名前)。それぞれ sorted。"""
    s2, s3 = set(cam_pts), set(p3)
    return sorted(s2 & s3), sorted(s2 - s3), sorted(s3 - s2)


def point_label(name: str) -> str:
    """ObjectName の最後の数字列。数字が無ければ name 全体。"""
    matches = re.findall(r"\d+", name)
    return matches[-1] if matches else name


def _draw_label(img: np.ndarray, label: str, ix: int, iy: int, w: int, h: int, s: float) -> None:
    """点の近傍にラベルを描く（phase0/visualize_points_2d.py の draw_label と同じ形）。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6 * s
    thickness = max(1, round(s))
    off = round(10 * s)
    margin = round(3 * s)
    (tw, th), baseline = cv2.getTextSize(label, font, scale, thickness)

    tx = ix + off
    ty = iy - off
    if tx + tw > w:
        tx = ix - off - tw
    if ty - th < 0:
        ty = iy + off + th
    tx = max(margin, min(tx, w - tw - margin))
    ty = max(th + margin, min(ty, h - baseline - margin))

    cv2.putText(img, label, (tx, ty), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, label, (tx, ty), font, scale, (0, 255, 0), thickness, cv2.LINE_AA)


def draw_points(img: np.ndarray, pts: dict) -> tuple[np.ndarray, list[str]]:
    """img のコピーに点とラベルを描く。戻り値は (画像, 画像の外のため描かなかった名前の sorted リスト)。"""
    out = img.copy()
    h, w = out.shape[:2]
    s = max(1.0, w / 1920)
    skipped: list[str] = []
    for name, (x, y) in pts.items():
        ix, iy = int(round(float(x))), int(round(float(y)))
        if ix < 0 or ix >= w or iy < 0 or iy >= h:
            skipped.append(name)
            continue
        cv2.circle(out, (ix, iy), round(8 * s), (0, 0, 255), max(1, round(2 * s)))
        cv2.circle(out, (ix, iy), max(1, round(1 * s)), (0, 255, 255), -1)
        _draw_label(out, point_label(name), ix, iy, w, h, s)
    return out, sorted(skipped)


# ---------------------------------------------------------------- 3 ツールの実行

def _tool_args(tool: str, job_yaml: Path) -> list[str]:
    return ["uv", "run", "--project", str(PHASE4_DIR), "python", str(PHASE4_DIR / tool),
            "--config", str(job_yaml)]


def manual_commands(job_yaml: Path) -> list[str]:
    """手で実行する場合の 3 コマンド（シェルにそのまま貼れるよう shlex.join で引用する）。"""
    return [shlex.join(_tool_args(TOOLS[0], job_yaml)),
            shlex.join(_tool_args(TOOLS[1], job_yaml)),
            'TORCH_CUDA_ARCH_LIST="9.0+PTX" ' + shlex.join(_tool_args(TOOLS[2], job_yaml))]


def recovery_commands(job_yaml: Path) -> list[str]:
    """失敗後に続きを実行するための 3 コマンド（前処理に --overwrite、推定に新しい --out-dir）。"""
    n = 2
    while os.path.lexists(job_yaml.parent / f"out_{n}"):
        n += 1
    new_out = job_yaml.parent / f"out_{n}"
    return [shlex.join(_tool_args(TOOLS[0], job_yaml)),
            shlex.join(_tool_args(TOOLS[1], job_yaml) + ["--overwrite"]),
            'TORCH_CUDA_ARCH_LIST="9.0+PTX" '
            + shlex.join(_tool_args(TOOLS[2], job_yaml) + ["--out-dir", str(new_out)])]


def _print_recovery(job_yaml: Path, i: int) -> None:
    """i 番目以降の続きのコマンドを表示する（i=0 で写真情報ファイルがある場合の扱いを含む）。"""
    cmds = recovery_commands(job_yaml)[i:]
    photo_info = job_yaml.parent / "photo_info.yaml"
    if i == 0 and photo_info.exists():
        print(f"写真情報ファイルは既にあります: {photo_info}。"
              "中身（focal_length_35mm）を確認し、正しければ次の前処理から続けてください。"
              "作り直す場合は、そのファイルが写真情報ファイルであることを確かめてから削除し、"
              "書き出しから実行してください")
        cmds = cmds[1:]
    print("原因を直したら、次のコマンドで続きを実行できます:")
    for c in cmds:
        print(c)


def run_tools(job_yaml: Path, runner=subprocess.run) -> int:
    """3 ツールを順に実行する。runner はテストで差し替える。"""
    for i, tool in enumerate(TOOLS):
        print(f"=== {tool} を実行します ===")
        try:
            rc = runner([sys.executable, str(PHASE4_DIR / tool), "--config", str(job_yaml)]).returncode
        except KeyboardInterrupt:
            print(f"\n中断しました（{tool} の実行中）。")
            _print_recovery(job_yaml, i)
            return 1
        if rc != 0:
            print(f"エラー: {tool} が終了コード {rc} で終わりました。ここで止めます。", file=sys.stderr)
            _print_recovery(job_yaml, i)
            return 1
    print(f"完了しました。推定の出力: {job_yaml.parent / 'out'}")
    return 0


# ---------------------------------------------------------------- メイン

def _write_image(path: Path, img: np.ndarray) -> None:
    if not cv2.imwrite(str(path), img):
        raise OSError(f"画像を書き込めません: {path}")


def _names(names: list[str]) -> str:
    return ", ".join(names) if names else "なし"


def _wizard(input_fn, state: dict) -> int:
    # 1. 原本
    while True:
        original_src = Path(ask(input_fn, "原本の画像のパス")).expanduser().resolve()
        errors, info = check_original(original_src)
        if errors:
            for e in errors:
                print(f"エラー: {e}", file=sys.stderr)
            continue
        break
    img, w, y0 = info["img"], info["w"], info["y0"]
    rng = info["rng"]
    x_min, x_max, y_min, y_max = rng
    print(f"原本サイズ {w}x{info['h']}、打てる範囲 x={x_min}〜{x_max}、y={y_min}〜{y_max}")

    # 2. 作業フォルダ（パスではなく名前だけを受け付ける）
    print(f"推定用の作業フォルダを {JOBS_DIR} の下に新しく作ります。")
    print("このフォルダに、原本・2D/3D 基準点・PLY のコピー、確認用の画像、設定ファイル（job.yaml）、"
          "推定結果をすべて保存します（PLY のコピーを含むため 100MB 以上になります）。")
    while True:
        name = ask(input_fn, "作業フォルダの名前（英数字・_・- だけ。例 20261126-01）")
        if not JOB_NAME_RE.fullmatch(name):
            print(f"エラー: 作業フォルダの名前は英数字・_・- だけで書いてください: {name}", file=sys.stderr)
            continue
        if os.path.lexists(JOBS_DIR / name):
            print(f"エラー: その名前の作業フォルダは既にあります: {JOBS_DIR / name}", file=sys.stderr)
            continue
        break
    job = (JOBS_DIR / name).resolve()
    msg = job_place_error(job)
    if msg is not None:
        print(f"エラー: {msg}", file=sys.stderr)
        return 1

    # 3. カメラ名
    print("この写真のカメラ名を決めます。初期 TOML・2D 点 CSV（camera_name 列）・推定結果で、"
          "この写真を指す名前として使います。英数字・_・- だけが使えます（先頭の - と metadata は不可）。")
    stem = original_src.stem
    default = stem if camera_name_error(stem) is None else None
    while True:
        camera = ask(input_fn, "カメラ名", default=default)
        msg = camera_name_error(camera)
        if msg is None:
            break
        print(f"エラー: {msg}", file=sys.stderr)
    job.mkdir(parents=True, exist_ok=False)
    state["job"] = job
    p = job_paths(job, original_src.suffix)
    shutil.copyfile(original_src, p["original"])
    print(f"作業フォルダを作りました: {job}")

    # 4. 範囲画像
    _write_image(p["crop_area"], draw_crop_area(img, rng))
    print(f"範囲画像: {p['crop_area']}")
    print(f"2D 点は原本の x={x_min}〜{x_max}、y={y_min}〜{y_max} の範囲（範囲画像の明るい部分）に"
          "打ってください。範囲外の点は前処理で除かれます。")
    print("2D 点 CSV の書式（UTF-8）:")
    print("  ObjectName,camera_name,X,Y")
    print(f"  基準_001,{camera},188,1485")
    print(f"camera_name 列には {camera} と書きます。X, Y は原本の画素座標です。")
    print("2D 点 CSV ができたら、そのパスを入力してください。")

    # 5. 2D・3D 基準点の確認
    prev_2d = prev_3d = None
    while True:
        src_2d = Path(ask(input_fn, "2D 点 CSV のパス", default=prev_2d)).expanduser().resolve()
        src_3d = Path(ask(input_fn, "3D 基準点 CSV のパス", default=prev_3d)).expanduser().resolve()
        prev_2d, prev_3d = src_2d, src_3d
        try:
            p2 = load_points_2d_csv(src_2d)
            p3 = load_points_3d_csv(src_3d)
        except UnicodeDecodeError:
            print("エラー: CSV を UTF-8 として読めません（UTF-8 で保存し直してください）: "
                  f"{src_2d} または {src_3d}", file=sys.stderr)
            continue
        except (InputError, OSError) as e:
            print(f"エラー: {e}", file=sys.stderr)
            continue
        cam_pts = {name: xy for (cam, name), xy in p2.items() if cam == camera}
        if not cam_pts:
            print(f"エラー: 2D 点 CSV に camera_name={camera} の行がありません", file=sys.stderr)
            continue
        bad = sorted(n for n in cam_pts if any(c in n for c in (",", '"', "\n", "\r")))
        if bad:
            print(f"エラー: 名前に , \" 改行は使えません（前処理が CSV を引用符なしで書くため）: {bad}",
                  file=sys.stderr)
            continue
        both, only_2d, only_3d = check_names(cam_pts, p3)
        print(f"両方にある名前（{len(both)} 個）: {_names(both)}")
        print(f"2D にだけある名前（{len(only_2d)} 個）: {_names(only_2d)}")
        print(f"3D にだけある名前（{len(only_3d)} 個）: {_names(only_3d)}")
        if only_2d:
            print("エラー: 3D 基準点 CSV に無い名前が 2D 点 CSV にあります", file=sys.stderr)
            continue
        outside = sorted(n for n, (x, y) in cam_pts.items() if not in_range(float(x), float(y), y0, w))
        if outside:
            print(f"範囲外のため前処理で除かれる点: {', '.join(outside)}")
        n_in = len(cam_pts) - len(outside)
        if n_in < MIN_POINTS:
            print(f"エラー: 範囲内の点が {n_in} 点です。前処理には {MIN_POINTS} 点以上必要です",
                  file=sys.stderr)
            continue
        check_img, skipped = draw_points(draw_crop_area(img, rng), cam_pts)
        if skipped:
            print(f"警告: 画像の外の点は描いていません: {', '.join(skipped)}", file=sys.stderr)
        _write_image(p["points_check"], check_img)
        print(f"確認画像: {p['points_check']}")
        if ask_yes_no(input_fn, "点の位置と名前は正しいですか"):
            break

    # 6. PLY
    while True:
        src_ply = Path(ask(input_fn, "3DGS の PLY ファイルのパス")).expanduser().resolve()
        if src_ply.is_file():
            break
        print(f"エラー: ファイルがありません: {src_ply}", file=sys.stderr)

    # 7. コピーと共用設定YAML
    shutil.copyfile(src_2d, p["points_2d_original"])
    shutil.copyfile(src_3d, p["points_3d"])
    shutil.copyfile(src_ply, p["ply"])
    p["job_yaml"].write_text(format_job_yaml(camera, p), encoding="utf-8")
    state["job_yaml"] = p["job_yaml"]
    state["yaml_done"] = True
    print(f"共用設定YAML: {p['job_yaml']}")

    # 8. 実行
    print("手で実行する場合のコマンド:")
    for c in manual_commands(p["job_yaml"]):
        print(c)
    if not ask_yes_no(input_fn, "書き出し → 前処理 → 推定 を実行しますか"):
        print("実行しませんでした。上のコマンドで実行できます。")
        return 0
    state["running"] = True
    return run_tools(p["job_yaml"])


def main(argv: "list[str] | None" = None, input_fn=input) -> int:
    parser = argparse.ArgumentParser(
        description="スマートフォン静止画のカメラパラメータ推定ウィザード（feat-045）。"
                    "引数は受け付けない（対話で答える）",
        allow_abbrev=False)
    parser.parse_args(argv)
    state: dict = {}
    try:
        return _wizard(input_fn, state)
    except (KeyboardInterrupt, EOFError):
        print("\n中断しました。")
        if "job" in state:
            print(f"作業フォルダ: {state['job']}（そのまま残しています）")
            if not state.get("yaml_done"):
                print("新しい作業フォルダ名で最初からやり直してください。")
            elif not state.get("running"):
                print("次のコマンドで続けられます:")
                for c in manual_commands(state["job_yaml"]):
                    print(c)
        return 1
    except OSError as e:
        print(f"エラー: {e}", file=sys.stderr)
        if "job" in state:
            print(f"作業フォルダ: {state['job']}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

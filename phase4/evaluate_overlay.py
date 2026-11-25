"""feat-041: 写真とレンダ画像の重ね合わせ評価（FR-008）

前処理後画像と、指定TOMLのパラメータでレンダした画像のずれを DIS オプティカルフローで
画素ごとに求め、統計値で合否を判定し、確認用画像（半透明合成・市松模様・ヒートマップ）を出力する。

実行方法（プロジェクトルートで）:
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/evaluate_overlay.py \\
        --toml <TOML> --camera <カメラ名> --ply <PLY> --image <前処理後画像PNG> \\
        --out-prefix <出力接頭辞> [--overwrite]

終了コード: 0=評価を実行した（合格・不合格・判定不能のいずれでも0）、
            1=入力エラー・出力済み、2=引数エラー

設計根拠: docs/issues/feat-041-intrinsic-loop-render-match/design.md §4.3
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from refine_extrinsics import GATE_ALPHA, GATE_Z_RANGE, NEAR_PLANE, render_depth_alpha_distorted
from render_keypoints import load_cameras_toml, render_image, select_camera

CLAHE_CLIP = 2.0
CLAHE_TILE = (8, 8)
DIS_PRESET = cv2.DISOPTICAL_FLOW_PRESET_MEDIUM
FB_CONSIST_PX = 1.0
TEXTURE_GRAD_MIN = 20.0
VALID_RATIO_MIN = 0.10
PASS_MEDIAN_PX = 1.0
CHECKER_TILE = 120
FLOWMAG_CLIP_PX = 5.0


def eval_output_paths(out_prefix: str) -> dict[str, str]:
    """評価の4出力のパス"""
    return {"txt": f"{out_prefix}.txt",
            "blend": f"{out_prefix}_blend.png",
            "checker": f"{out_prefix}_checker.png",
            "flowmag": f"{out_prefix}_flowmag.png"}


def _to_gray(img: np.ndarray) -> np.ndarray:
    if img.ndim == 2:
        return img
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def flow_stats(photo_bgr: np.ndarray, render_bgr: np.ndarray, render_valid: np.ndarray) -> dict:
    """DIS による写真→レンダのずれの統計。戻り値は stats に加えて "valid" と "mag" を含む。"""
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE)
    P = clahe.apply(_to_gray(photo_bgr))
    Rn = clahe.apply(_to_gray(render_bgr))
    H, W = P.shape

    dis = cv2.DISOpticalFlow_create(DIS_PRESET)
    F_fw = dis.calc(P, Rn, None)
    F_bw = dis.calc(Rn, P, None)

    xs, ys = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    mx = xs + F_fw[..., 0]
    my = ys + F_fw[..., 1]
    inside = (mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)
    F_bw_s = cv2.remap(F_bw, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    consistent = np.linalg.norm(F_fw + F_bw_s, axis=2) < FB_CONSIST_PX

    gx = cv2.Sobel(P, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(P, cv2.CV_32F, 0, 1, ksize=3)
    texture = np.sqrt(gx * gx + gy * gy) >= TEXTURE_GRAD_MIN

    valid = inside & consistent & texture & render_valid
    mag = np.linalg.norm(F_fw, axis=2).astype(np.float32)

    n_valid = int(valid.sum())
    stats = {"n_valid": n_valid, "valid_ratio": n_valid / float(W * H)}
    if n_valid == 0:
        stats.update({"median": float("nan"), "p95": float("nan"),
                      "mean": float("nan"), "max": float("nan")})
    else:
        m = mag[valid].astype(np.float64)
        stats.update({"median": float(np.median(m)),
                      "p95": float(np.percentile(m, 95)),
                      "mean": float(m.mean()), "max": float(m.max())})
    stats["valid"] = valid
    stats["mag"] = mag
    return stats


def judge(stats: dict) -> str:
    """"合格" / "不合格" / "判定不能" """
    if stats["valid_ratio"] < VALID_RATIO_MIN:
        return "判定不能"
    if stats["median"] <= PASS_MEDIAN_PX:
        return "合格"
    return "不合格"


def make_images(photo_bgr: np.ndarray, render_bgr: np.ndarray, valid: np.ndarray,
                mag: np.ndarray) -> dict[str, np.ndarray]:
    """確認用画像。キー: blend, checker, flowmag"""
    blend = cv2.addWeighted(photo_bgr, 0.5, render_bgr, 0.5, 0)

    H, W = photo_bgr.shape[:2]
    ty = (np.arange(H) // CHECKER_TILE)[:, None]
    tx = (np.arange(W) // CHECKER_TILE)[None, :]
    use_photo = ((ty + tx) % 2 == 0)[..., None]
    checker = np.where(use_photo, photo_bgr, render_bgr).astype(np.uint8)

    scaled = (np.clip(mag, 0, FLOWMAG_CLIP_PX) / FLOWMAG_CLIP_PX * 255.0).astype(np.uint8)
    flowmag = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    flowmag[~valid] = (128, 128, 128)
    return {"blend": blend, "checker": checker, "flowmag": flowmag}


def format_eval_txt(stats: dict, verdict: str, inputs_text: str) -> str:
    """{out_prefix}.txt の内容を組み立てる（設計 §4.3.4）。"""
    lines = [
        "feat-041: 重ね合わせ評価",
        f"入力: {inputs_text}" if inputs_text else "入力: (未指定)",
        f"n_valid: {stats['n_valid']}",
        f"valid_ratio: {stats['valid_ratio']:.3f}",
        f"valid_ratio_raw: {stats['valid_ratio']!r}",
        f"median: {stats['median']:.3f} px",
        f"median_raw: {stats['median']!r}",
        f"p95: {stats['p95']:.3f} px",
        f"mean: {stats['mean']:.3f} px",
        f"max: {stats['max']:.3f} px",
        f"合格ライン: median <= {PASS_MEDIAN_PX:.3f} px（valid_ratio >= {VALID_RATIO_MIN:.3f}）。p95 は記録のみ",
        f"判定: {verdict}",
    ]
    return "\n".join(lines) + "\n"


def evaluate_and_write(gaussians, cam: dict, photo_bgr: np.ndarray, out_prefix: str,
                       inputs_text: str = "") -> dict:
    """レンダ→評価→4ファイル書き出し。統計値の辞書（判定 "judge" つき）を返す。

    inputs_text: 評価txtの「入力パス」行に書く文字列（省略可）。"""
    render_bgr = render_image(gaussians, cam, near_plane=NEAR_PLANE, distort=True)
    depth, alpha = render_depth_alpha_distorted(gaussians, cam, NEAR_PLANE)
    render_valid = (np.isfinite(depth) & (alpha > GATE_ALPHA)
                    & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1]))

    st = flow_stats(photo_bgr, render_bgr, render_valid)
    verdict = judge(st)
    imgs = make_images(photo_bgr, render_bgr, st["valid"], st["mag"])

    paths = eval_output_paths(out_prefix)
    Path(out_prefix).parent.mkdir(parents=True, exist_ok=True)
    for key in ("blend", "checker", "flowmag"):
        cv2.imwrite(paths[key], imgs[key])

    Path(paths["txt"]).write_text(format_eval_txt(st, verdict, inputs_text), encoding="utf-8")

    out = {k: v for k, v in st.items() if k not in ("valid", "mag")}
    out["judge"] = verdict
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="写真とレンダ画像の重ね合わせ評価（feat-041 FR-008）")
    p.add_argument("--toml", required=True, help="評価対象のTOML")
    p.add_argument("--camera", required=True, help="カメラ名")
    p.add_argument("--ply", required=True, help="3DGS PLYファイル")
    p.add_argument("--image", required=True, help="前処理後画像PNG")
    p.add_argument("--out-prefix", required=True, help="出力ファイル名の接頭辞")
    p.add_argument("--overwrite", action="store_true", help="出力ファイルの上書きを許可する")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    paths = eval_output_paths(args.out_prefix)
    if not args.overwrite:
        for pth in paths.values():
            if Path(pth).exists():
                print(f"エラー: 出力ファイルが既に存在します（--overwrite で上書き可）: {pth}",
                      file=sys.stderr)
                return 1

    try:
        cams = load_cameras_toml(args.toml)
        cam = select_camera(cams, args.camera)
    except Exception as e:
        print(f"エラー: TOML またはカメラを読み込めません: {e}", file=sys.stderr)
        return 1

    photo = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if photo is None:
        print(f"エラー: 画像を読み込めません: {args.image}", file=sys.stderr)
        return 1
    h, w = photo.shape[:2]
    if (w, h) != (cam["width"], cam["height"]):
        print(f"エラー: 画像サイズ {w}x{h} が TOML の size {cam['width']}x{cam['height']} と"
              "一致しません", file=sys.stderr)
        return 1

    try:
        from render import load_ply  # 遅延import（CLI実行時のみ）
        gaussians = load_ply(args.ply)
    except Exception as e:
        print(f"エラー: PLYを読み込めません: {args.ply}（{e}）", file=sys.stderr)
        return 1

    inputs_text = f"toml={args.toml} camera={args.camera} ply={args.ply} image={args.image}"
    st = evaluate_and_write(gaussians, cam, photo, args.out_prefix, inputs_text)
    print(f"n_valid={st['n_valid']} valid_ratio={st['valid_ratio']:.3f} "
          f"median={st['median']:.3f}px p95={st['p95']:.3f}px 判定={st['judge']}")
    for pth in paths.values():
        print(f"出力: {pth}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

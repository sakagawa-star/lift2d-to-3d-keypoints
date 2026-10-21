"""C3D（1ファイル=1人の3Dキーポイント）→ NPZ 変換スクリプト（feat-039）。

render_fps_video.py の入力 NPZ（x3d_world / frame_ids / joint_names）を C3D から作る。
座標変換は render_keypoints.c3d_to_calib と同一。欠損点は NaN にする。

使い方（phase4 ディレクトリ）:
    uv run python c3d_to_npz.py input.c3d [--output out.npz]
"""

import argparse
import os
import sys
import tempfile

import numpy as np

from npz_to_c3d import load_npz
from render_fps_video import HEAD_JOINT_NAMES
from render_keypoints import c3d_to_calib, load_c3d_all_frames


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="C3D（1ファイル=1人の3Dキーポイント）→ NPZ 変換（render_fps_video.py 入力用）",
        allow_abbrev=False,
    )
    parser.add_argument("c3d_path", help="入力 C3D パス")
    parser.add_argument("--output", default=None,
                        help="出力 NPZ パス（省略時: 入力と同じ basename で拡張子 .npz）")
    return parser


def read_c3d(c3d_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str], float]:
    """C3D を読み込み (raw_mm, residual, frame_ids, labels, point_rate) を返す。

    Raises:
        FileNotFoundError: 入力ファイルが無い場合。
        ValueError: C3Dとして読めない・フレーム0件・マーカー0個・空/重複マーカー名・
            フレームが途中で欠けている場合。
    """
    try:
        labels, frames_data, point_rate = load_c3d_all_frames(c3d_path)
    except (FileNotFoundError, ValueError):
        raise
    except Exception as e:
        raise ValueError(f"C3Dとして読み込めません: {c3d_path}: {e}")

    if len(labels) == 0:
        raise ValueError(f"C3Dにマーカーがありません: {c3d_path}")
    for i, name in enumerate(labels):
        if name == "":
            raise ValueError(f"C3Dに空のマーカー名があります（{i}番目。0始まり）: {c3d_path}")
    seen: set[str] = set()
    dups: list[str] = []
    for name in labels:
        if name in seen and name not in dups:
            dups.append(name)
        seen.add(name)
    if dups:
        raise ValueError(f"C3Dのマーカー名が重複しています: {', '.join(dups)}: {c3d_path}")

    import c3d  # 関数内 import（load_c3d_all_frames と同じ流儀）

    with open(c3d_path, "rb") as f:
        declared = int(c3d.Reader(f).frame_count)
    if len(frames_data) != declared:
        raise ValueError(f"C3Dのフレームが途中で欠けています（宣言 {declared} フレーム、"
                         f"読み込み {len(frames_data)} フレーム）: {c3d_path}")

    raw_mm = np.stack([fr["data"] for fr in frames_data])
    residual = np.stack([fr["residual"] for fr in frames_data])
    frame_ids = np.array([fr["frame_no"] for fr in frames_data], dtype=np.int64)
    return raw_mm, residual, frame_ids, labels, point_rate


def convert_to_world(raw_mm: np.ndarray, residual: np.ndarray) -> tuple[np.ndarray, dict[str, int]]:
    """C3D raw 座標(mm)を world 座標(m)に変換し、欠損点を NaN にする。

    Returns:
        x3d_world: (F, J, 3) float64。
        counts: 欠損点の内訳（キー missing / residual / zero / nan / total）。
    """
    by_residual = ~(residual >= 0)                  # (a) 負または NaN
    by_nan = np.isnan(raw_mm).any(axis=-1)          # (b)
    by_zero = np.all(raw_mm == 0.0, axis=-1)        # (c) 厳密一致
    missing = by_residual | by_nan | by_zero
    x3d_world = c3d_to_calib(raw_mm)
    x3d_world[missing] = np.nan
    counts = {"missing": int(missing.sum()), "residual": int(by_residual.sum()),
              "zero": int(by_zero.sum()), "nan": int(by_nan.sum()), "total": int(missing.size)}
    return x3d_world, counts


def _verify_npz(path: str, x3d_world: np.ndarray, frame_ids: np.ndarray,
                joint_names: list[str]) -> None:
    """書き出した NPZ を読み戻し、書き出し前と一致するか検証する。

    Raises:
        ValueError: load_npz の検証違反。
        RuntimeError: 内容が一致しない場合。
    """
    x, f, n = load_npz(path)
    if x.shape != x3d_world.shape:
        raise RuntimeError("読み戻し検証失敗: x3d_world の形状 が一致しません")
    if not np.array_equal(np.isnan(x), np.isnan(x3d_world)):
        raise RuntimeError("読み戻し検証失敗: NaN の位置 が一致しません")
    if not np.array_equal(x[~np.isnan(x)], x3d_world[~np.isnan(x3d_world)]):
        raise RuntimeError("読み戻し検証失敗: x3d_world が一致しません")
    if not np.array_equal(f, frame_ids):
        raise RuntimeError("読み戻し検証失敗: frame_ids が一致しません")
    if n != list(joint_names):
        raise RuntimeError("読み戻し検証失敗: joint_names が一致しません")


def write_npz(out_path: str, x3d_world: np.ndarray, frame_ids: np.ndarray,
              joint_names: list[str]) -> None:
    """NPZ を一時ファイル経由で書き出し、読み戻し検証に成功したら出力パスに確定する。"""
    names_arr = np.array(joint_names, dtype=str)
    out_dir = os.path.dirname(os.path.abspath(out_path))
    fd, tmp_path = tempfile.mkstemp(dir=out_dir, prefix=os.path.basename(out_path) + ".",
                                    suffix=".tmp.npz")
    try:
        with os.fdopen(fd, "wb") as fh:
            np.savez(fh, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=names_arr)
        _verify_npz(tmp_path, x3d_world, frame_ids, joint_names)
        os.replace(tmp_path, out_path)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    out_path = args.output if args.output else os.path.splitext(args.c3d_path)[0] + ".npz"

    if os.path.splitext(out_path)[1].lower() != ".npz":
        print(f"エラー: 出力パスの拡張子が .npz ではありません: {out_path}", file=sys.stderr)
        return 1
    if os.path.abspath(out_path) == os.path.abspath(args.c3d_path):
        print(f"エラー: 出力パスが入力 C3D と同一です（上書き防止）: {out_path}", file=sys.stderr)
        return 1

    try:
        print(f"C3D読み込み: {args.c3d_path}")
        raw_mm, residual, frame_ids, labels, point_rate = read_c3d(args.c3d_path)
        F, J = raw_mm.shape[0], raw_mm.shape[1]
        print(f"  フレーム数 F={F}, マーカー数 J={J}, rate={point_rate} Hz")
        print(f"  frame_ids 範囲: {int(frame_ids[0])}〜{int(frame_ids[-1])}（C3D フレーム番号をそのまま使用）")

        x3d_world, counts = convert_to_world(raw_mm, residual)
        print(f"  欠損点（NaN にした点）: {counts['missing']}/{counts['total']} 点"
              f"（residual<0またはNaN: {counts['residual']}, 座標(0,0,0): {counts['zero']}, "
              f"座標NaN: {counts['nan']}。重複あり）")

        missing_head = [n for n in HEAD_JOINT_NAMES if n not in labels]
        if missing_head:
            print(f"警告: 頭部マーカーが C3D にありません: {', '.join(missing_head)}"
                  "（この NPZ は render_fps_video.py の入力に使えません）", file=sys.stderr)

        write_npz(out_path, x3d_world, frame_ids, labels)
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    print(f"完了: {out_path}（{F} フレーム, {J} マーカー）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

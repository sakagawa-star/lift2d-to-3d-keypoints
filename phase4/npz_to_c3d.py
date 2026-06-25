"""NPZ（リフトアップ済み3Dキーポイント）→ C3D 変換スクリプト。

2Dキーポイントを3Dにリフトアップした NPZ（`x3d_world`, world座標メートル）を、
Blender の C3D インポートアドオン `io_anim_c3d` で取り込める C3D ファイルに変換する。

座標規約は `render_keypoints.py` の `c3d_to_calib`（`(px,py,pz)mm → (pz,px,py)×0.001 m`）と
互換になるよう、world `(X,Y,Z)` m を C3D raw `(Y,Z,X)×1000` mm で書き出す。
これにより同じ C3D を Blender 取り込みと（将来的に）render_keypoints の双方で扱える。

C3D フレーム番号は 1..F の連番（py-c3d 0.6.0 のヘッダが 16bit で 65535 超の開始フレームを
正しく扱えないため。NPZ の絶対 `frame_ids` は C3D に保持せず、対応をログ出力する）。

実行（phase4 venv）:
  uv run --project phase4 python npz_to_c3d.py data/session001_f145749_world300.npz
  uv run --project phase4 python npz_to_c3d.py data/foo.npz --output data/bar.c3d --fps 30.0
"""

import argparse
import os
import sys

import numpy as np


# Blender io_anim_c3d を正立させる X/Y_SCREEN。world の鉛直上方向は実測で +Z（design.md / investigation.md）。
# io_anim_c3d は pose bone のローカル座標 + +Z向きボーンの rest 行列 B=[[1,0,0],[0,0,-1],[0,1,0]] で
# 描画するため、表示鉛直 = axis_y · p_raw。Y_SCREEN='+Y' で表示鉛直 = world鉛直 Z_w（恒等表示で正立）。
X_SCREEN = "+Z"
Y_SCREEN = "+Y"

# C3D point の単位（POINT:UNITS）。io_anim_c3d がメートル換算に使う。
POINT_UNITS = "mm"


def load_npz(npz_path: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """NPZ を読み込み・検証し、(x3d_world, frame_ids, joint_names) を返す。

    Args:
        npz_path: 入力 NPZ パス。

    Returns:
        x3d_world:   (F, J, 3) float64, world座標メートル。
        frame_ids:   (F,) int64, NPZ 絶対フレーム番号（昇順連番）。
        joint_names: 長さ J の関節名リスト。

    Raises:
        ValueError: 必須キー欠落・形状不一致・frame_ids が連番でない場合。
    """
    d = np.load(npz_path, allow_pickle=True)

    required = ("x3d_world", "frame_ids", "joint_names")
    missing = [k for k in required if k not in d.files]
    if missing:
        raise ValueError(
            f"NPZに必須キーがありません: {', '.join(missing)}（パス: {npz_path}）"
        )

    x3d_world = np.asarray(d["x3d_world"], dtype=np.float64)
    if x3d_world.ndim != 3 or x3d_world.shape[2] != 3:
        raise ValueError(
            f"x3d_world の形状が不正です: {x3d_world.shape}（期待: (F, J, 3)）"
        )
    F, J = x3d_world.shape[0], x3d_world.shape[1]

    frame_ids = np.asarray(d["frame_ids"]).reshape(-1).astype(np.int64)
    if len(frame_ids) != F:
        raise ValueError(
            f"frame_ids の長さ {len(frame_ids)} が x3d_world のフレーム数 {F} と一致しません"
        )
    if F >= 2 and not np.all(np.diff(frame_ids) == 1):
        raise ValueError(
            "frame_ids が昇順の連番（差分すべて1）ではありません。"
            "C3Dはフレーム番号が連続している前提です"
        )

    joint_names = [str(n) for n in np.asarray(d["joint_names"]).reshape(-1).tolist()]
    if len(joint_names) != J:
        raise ValueError(
            f"joint_names の長さ {len(joint_names)} が x3d_world の関節数 {J} と一致しません"
        )

    return x3d_world, frame_ids, joint_names


def world_to_c3d_raw(x3d_world: np.ndarray) -> np.ndarray:
    """world座標(m) → C3D raw座標(mm)。軸入替 (X,Y,Z) → (Y,Z,X) ×1000。

    `render_keypoints.c3d_to_calib`（raw (px,py,pz)mm → (pz,px,py)×0.001 m）の逆変換。
    `c3d_to_calib(world_to_c3d_raw(w)) == w` が成り立つ。NaN/Inf はそのまま伝播する
    （無効化は write_c3d の residual で行う）。

    Args:
        x3d_world: (..., 3) world座標メートル。

    Returns:
        (..., 3) C3D raw座標ミリメートル（float64）。
    """
    w = np.asarray(x3d_world, dtype=np.float64)
    X, Y, Z = w[..., 0], w[..., 1], w[..., 2]
    return np.stack([Y, Z, X], axis=-1) * 1000.0


def write_c3d(
    out_path: str,
    raw_mm: np.ndarray,
    joint_names: list[str],
    fps: float,
) -> None:
    """C3D raw座標を C3D ファイルへアトミックに書き出す。

    フレーム番号は 1..F 固定。各点の residual は座標が有限なら 0.0（有効）、
    NaN/Inf を含むなら -1.0（無効）。一時ファイルに書いて py-c3d で読み戻し検証
    （フレーム数・ラベル一致）に成功した場合のみ os.replace で最終パスへ確定する。

    Args:
        out_path:    出力 C3D パス。
        raw_mm:      (F, J, 3) C3D raw座標 mm。
        joint_names: 長さ J の関節名リスト。
        fps:         フレームレート [Hz]。

    Raises:
        ValueError:   フレーム0件・関節0件の場合。
        RuntimeError: 読み戻し検証に失敗した場合。
    """
    import c3d  # phase4 venv のみ。関数内 import

    raw_mm = np.asarray(raw_mm, dtype=np.float64)
    F, J = raw_mm.shape[0], raw_mm.shape[1]
    if F == 0:
        raise ValueError("書き出すフレームがありません（F == 0）")
    if J == 0:
        raise ValueError("関節がありません（J == 0）")

    writer = c3d.Writer(point_rate=fps, point_units=POINT_UNITS)
    writer.set_point_labels(joint_names)
    writer.set_screen_axis(X_SCREEN, Y_SCREEN)
    writer.set_start_frame(1)  # 1始まり固定（ADR-5）

    empty_analog = np.zeros((0, 1), dtype=np.float32)
    for f in range(F):
        finite = np.isfinite(raw_mm[f]).all(axis=1)  # (J,)
        points = np.zeros((J, 5), dtype=np.float32)
        points[:, 0:3] = np.where(finite[:, None], raw_mm[f], 0.0)  # 無効点座標は0
        points[:, 3] = np.where(finite, 0.0, -1.0)                  # residual
        points[:, 4] = 0.0                                          # camera mask
        writer.add_frames([(points, empty_analog)])

    # アトミック書き出し（ADR-6）: tmp に書く → 読み戻し検証 → os.replace
    tmp_path = out_path + ".tmp"
    try:
        with open(tmp_path, "wb") as h:
            writer.write(h)
        _verify_c3d(tmp_path, F, joint_names)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    os.replace(tmp_path, out_path)


def _verify_c3d(c3d_path: str, expected_frames: int, expected_labels: list[str]) -> None:
    """書き出した C3D を読み戻し、フレーム数とラベルが一致するか検証する。

    Raises:
        RuntimeError: フレーム数またはラベルが不一致の場合。
    """
    import c3d  # phase4 venv のみ。関数内 import

    with open(c3d_path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [label.strip() for label in reader.point_labels]
        n_frames = sum(1 for _ in reader.read_frames())

    if n_frames != expected_frames:
        raise RuntimeError(
            f"読み戻し検証失敗: フレーム数 {n_frames} が期待値 {expected_frames} と不一致"
        )
    if labels != list(expected_labels):
        raise RuntimeError(
            f"読み戻し検証失敗: ラベルが不一致\n  期待: {expected_labels}\n  実際: {labels}"
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="NPZ（リフトアップ済み3Dキーポイント）→ C3D 変換（io_anim_c3d 取り込み対応）",
        allow_abbrev=False,
    )
    parser.add_argument("npz_path", help="入力 NPZ パス")
    parser.add_argument("--output", default=None,
                        help="出力 C3D パス（省略時: 入力と同じ basename で拡張子 .c3d）")
    parser.add_argument("--fps", type=float, default=30.0,
                        help="フレームレート [Hz]（default: 30.0）")
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    out_path = args.output if args.output else os.path.splitext(args.npz_path)[0] + ".c3d"

    # 出力パス検証（重い NPZ ロード前に弾く）
    if os.path.splitext(out_path)[1].lower() != ".c3d":
        print(f"エラー: 出力パスの拡張子が .c3d ではありません: {out_path}", file=sys.stderr)
        return 1
    if os.path.abspath(out_path) == os.path.abspath(args.npz_path):
        print(f"エラー: 出力パスが入力 NPZ と同一です（上書き防止）: {out_path}",
              file=sys.stderr)
        return 1

    try:
        x3d_world, frame_ids, joint_names = load_npz(args.npz_path)
        F, J = x3d_world.shape[0], x3d_world.shape[1]
        print(f"NPZ読み込み: {args.npz_path}")
        print(f"  フレーム数 F={F}, 関節数 J={J}, fps={args.fps}")
        print(f"  NPZ frame_ids 範囲: {int(frame_ids[0])}〜{int(frame_ids[-1])}")

        raw_mm = world_to_c3d_raw(x3d_world)
        write_c3d(out_path, raw_mm, joint_names, args.fps)
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    # C3D フレーム番号（1..F）と NPZ 絶対フレーム番号の対応をログ出力（FR-003）
    print("C3D フレーム番号は 1..F の連番（NPZ 絶対 frame_ids は C3D に保持しない）")
    print(f"  C3D frame 1   <-> NPZ frame_id {int(frame_ids[0])}")
    print(f"  C3D frame {F} <-> NPZ frame_id {int(frame_ids[-1])}")
    print(f"完了: {out_path}（{F} フレーム, {J} 関節）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

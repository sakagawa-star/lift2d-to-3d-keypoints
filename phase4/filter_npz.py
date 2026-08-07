"""NPZキーポイントの時間方向平滑化スクリプト（feat-028）。

リフトアップ済み3DキーポイントNPZ（`npz_to_c3d.py` 入力と同一フォーマット）の
全関節軌跡を時間方向にButterworthローパスフィルタで平滑化し、新しいNPZファイルとして
書き出す。feat-027（NPZ直読みによる一人称視点動画一括生成）ではC3D・Blender工程を
廃止するため、feat-020 `filter_c3d.py`（C3D→C3D）による既存の平滑化が使えず、
NPZ段階で平滑化する。

無効サンプル（NaN/Inf）は、長さが --max-gap 以下のギャップのみ線形補間してフィルタ計算に
使い、超えるギャップではセグメント分割して独立にフィルタする。フィルタ適用に必要な
最小長（MIN_FILTER_SAMPLES）未満のセグメントは平滑化せずスキップする。
出力では無効サンプルを入力値のまま維持する（補間値を有効値として書き出さない）。

実行（phase4 venv。Blender・GPU不要）:
  uv run --project phase4 python filter_npz.py data/session001_world_22pt.npz --fps 30
  uv run --project phase4 python filter_npz.py data/session001_world_22pt.npz --fps 30 \
      --output data/out.npz --cutoff 4.0 --max-gap 5
"""

import argparse
import os
import sys

import numpy as np


MIN_FILTER_SAMPLES = 10       # filtfilt 既定 padlen(9) + 1
DEFAULT_CUTOFF = 6.0          # カットオフ周波数の既定値 [Hz]（feat-020 と同一）
DEFAULT_MAX_GAP = 10          # 最大補間ギャップ長の既定値 [フレーム]（feat-020 と同一）
REQUIRED_KEYS = ("x3d_world", "frame_ids", "joint_names")


def load_npz_keypoints(npz_path: str) -> tuple[
        np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray], np.dtype]:
    """NPZ を読み込み・規約検証し、(x3d, frame_ids, joint_names_arr, extras, x3d_dtype) を返す。

    Args:
        npz_path: 入力 NPZ パス。

    Returns:
        x3d:             (F, J, 3) float64（計算用に変換済み）、ワールド座標 [m]
        frame_ids:        入力の配列そのまま（dtype変換しない。書き出し用）
        joint_names_arr:  入力の配列そのまま（dtype変換しない。書き出し用）
        extras:           必須3キー以外の全キーの {キー名: 配列} 辞書（無加工）
        x3d_dtype:        入力 x3d_world の dtype（書き出し時の変換先）

    Raises:
        ValueError: 必須キー欠落・形状不一致・dtype不正・frame_idsが連番でない場合。
    """
    try:
        d = np.load(npz_path, allow_pickle=True)
    except (FileNotFoundError, OSError, ValueError):
        raise
    except Exception as e:
        # zipfile.BadZipFile は OSError を継承しないため ValueError に変換する
        raise ValueError(f"NPZファイルを読み込めません: {npz_path}（{e}）") from e

    missing = [k for k in REQUIRED_KEYS if k not in d.files]
    if missing:
        raise ValueError(
            f"NPZに必須キーがありません: {', '.join(missing)}（パス: {npz_path}）"
        )

    x3d_world = d["x3d_world"]
    if x3d_world.ndim != 3 or x3d_world.shape[2] != 3:
        raise ValueError(
            f"x3d_world の形状が不正です: {x3d_world.shape}（期待: (F, J, 3)）"
        )
    if not np.issubdtype(x3d_world.dtype, np.floating):
        raise ValueError(
            f"x3d_world の dtype が浮動小数点型ではありません: {x3d_world.dtype}"
        )
    F, J = x3d_world.shape[0], x3d_world.shape[1]
    if F == 0:
        raise ValueError("x3d_world のフレーム数が0です（F == 0）")
    if J == 0:
        raise ValueError("x3d_world の関節数が0です（J == 0）")
    x3d_dtype = x3d_world.dtype

    frame_ids = d["frame_ids"]
    if not np.issubdtype(frame_ids.dtype, np.integer):
        raise ValueError(
            f"frame_ids の dtype が整数型ではありません: {frame_ids.dtype}"
        )
    if frame_ids.ndim != 1 or frame_ids.shape != (F,):
        raise ValueError(
            f"frame_ids の形状が不正です: {frame_ids.shape}（期待: ({F},)）"
        )
    if F >= 2:
        ids64 = frame_ids.astype(np.int64)
        if not np.all(np.diff(ids64) == 1):
            raise ValueError(
                "frame_ids が昇順の連番（差分すべて1）ではありません"
            )

    joint_names_arr = d["joint_names"]
    if joint_names_arr.ndim != 1 or joint_names_arr.shape != (J,):
        raise ValueError(
            f"joint_names の形状が不正です: {joint_names_arr.shape}（期待: ({J},)）"
        )

    extras = {k: d[k] for k in d.files if k not in REQUIRED_KEYS}

    x3d = np.asarray(x3d_world, dtype=np.float64)

    return x3d, frame_ids, joint_names_arr, extras, x3d_dtype


def lowpass_filtfilt(series: np.ndarray, cutoff: float, fps: float) -> np.ndarray:
    """2次Butterworthローパスを filtfilt でゼロ位相適用する（実効4次）。

    Args:
        series: (N,) または (N, 3)。時間軸は axis=0。N >= MIN_FILTER_SAMPLES であること。
        cutoff: カットオフ周波数 [Hz]（0 < cutoff < fps/2 は呼び出し側で検証済み）
        fps:    サンプリング周波数 [Hz]

    Returns:
        同形状の平滑化済み配列。
    """
    from scipy.signal import butter, filtfilt  # phase4 venv のみ。関数内 import

    b, a = butter(N=2, Wn=cutoff / (fps / 2.0), btype="low")
    return filtfilt(b, a, series, axis=0)


def split_segments(valid: np.ndarray, max_gap: int) -> list[tuple[int, int]]:
    """有効フラグからセグメント [(lo, hi), ...]（両端含む・両端は有効）を返す。

    隣接する有効サンプル間の無効サンプル数が max_gap を超える箇所で区切る。
    先頭・末尾の無効区間はどのセグメントにも含まれない。有効0個なら空リスト。
    """
    idx = np.flatnonzero(valid)
    if len(idx) == 0:
        return []
    segments = []
    lo = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i - prev - 1 > max_gap:  # ギャップ（無効サンプル数）が上限超え → 区切る
            segments.append((int(lo), int(prev)))
            lo = i
        prev = i
    segments.append((int(lo), int(prev)))
    return segments


def filter_joint(
    x3d_j: np.ndarray, cutoff: float, fps: float, max_gap: int,
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """関節1本分 (F,3) をセグメント単位で平滑化する。

    Returns:
        (平滑化後座標 (F,3), スキップしたセグメント [(lo, hi), ...])。
        無効サンプル位置・セグメント外は入力値のまま。
    """
    valid = np.isfinite(x3d_j).all(axis=1)
    out = x3d_j.copy()
    skipped = []
    for lo, hi in split_segments(valid, max_gap):
        seg_len = hi - lo + 1
        if seg_len < MIN_FILTER_SAMPLES:
            skipped.append((lo, hi))
            continue
        seg_valid = valid[lo:hi + 1]
        seg = x3d_j[lo:hi + 1].copy()
        if not seg_valid.all():
            # ギャップ（max_gap 以下が保証済み）を成分ごとに線形補間で埋める
            t = np.arange(seg_len)
            for c in range(3):
                seg[~seg_valid, c] = np.interp(
                    t[~seg_valid], t[seg_valid], seg[seg_valid, c])
        filtered = lowpass_filtfilt(seg, cutoff, fps)
        # 有効位置のみ置き換え（補間値は出力に反映しない）
        rows = np.flatnonzero(seg_valid) + lo
        out[rows] = filtered[rows - lo]
    return out, skipped


def filter_all_joints(
    x3d: np.ndarray, cutoff: float, fps: float, max_gap: int,
) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    """(F, J, 3) 全関節を平滑化する。

    Returns:
        (平滑化後 (F,J,3), スキップした (関節添字, lo, hi) のリスト)。
    """
    out = x3d.copy()
    skipped_all = []
    for j in range(x3d.shape[1]):
        filtered, skipped = filter_joint(x3d[:, j, :], cutoff, fps, max_gap)
        out[:, j, :] = filtered
        skipped_all.extend((j, lo, hi) for lo, hi in skipped)
    return out, skipped_all


def write_npz(
    out_path: str, x3d_filtered: np.ndarray, x3d_dtype: np.dtype,
    invalid_mask_in: np.ndarray, frame_ids: np.ndarray,
    joint_names_arr: np.ndarray, extras: dict[str, np.ndarray],
) -> None:
    """平滑化後の座標をNPZへアトミックに書き出す。

    frame_ids・joint_names・extras は入力から読み込んだ配列をそのまま書く。
    一時ファイルに書いて読み戻し検証に成功した場合のみ os.replace で確定する。

    Args:
        out_path:         出力 NPZ パス。
        x3d_filtered:      (F,J,3) float64 平滑化後座標。
        x3d_dtype:         入力 x3d_world の dtype（書き出し時の変換先）。
        invalid_mask_in:   入力NPZ由来の無効マスク ~isfinite(入力x3d_world) (F,J,3) bool。
        frame_ids:         入力の frame_ids 配列そのまま。
        joint_names_arr:   入力の joint_names 配列そのまま。
        extras:            追加キーの {キー名: 配列} 辞書（無加工）。

    Raises:
        RuntimeError: 読み戻し検証に失敗した場合。
    """
    x3d_out = x3d_filtered.astype(x3d_dtype)

    tmp_path = out_path + ".tmp.npz"
    try:
        np.savez(
            tmp_path,
            x3d_world=x3d_out,
            frame_ids=frame_ids,
            joint_names=joint_names_arr,
            **extras,
        )
        _verify_npz(
            tmp_path, x3d_out.shape, x3d_dtype, invalid_mask_in,
            frame_ids, joint_names_arr, extras,
        )
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    os.replace(tmp_path, out_path)


def _verify_npz(
    npz_path: str, expected_shape: tuple, expected_dtype: np.dtype,
    invalid_mask_in: np.ndarray, frame_ids: np.ndarray,
    joint_names_arr: np.ndarray, extras: dict[str, np.ndarray],
) -> None:
    """書き出した NPZ を読み戻し、入力NPZ由来の値と一致するか検証する。

    Raises:
        RuntimeError: キー集合・形状・dtype・無効マスク・各配列のいずれかが不一致の場合。
    """
    d = np.load(npz_path, allow_pickle=True)

    expected_keys = set(REQUIRED_KEYS) | set(extras.keys())
    actual_keys = set(d.files)
    if actual_keys != expected_keys:
        raise RuntimeError(
            f"読み戻し検証失敗: キー集合が不一致\n"
            f"  期待: {sorted(expected_keys)}\n  実際: {sorted(actual_keys)}"
        )

    x3d_readback = d["x3d_world"]
    if x3d_readback.shape != expected_shape:
        raise RuntimeError(
            f"読み戻し検証失敗: x3d_world の形状が不一致 "
            f"期待={expected_shape} 実際={x3d_readback.shape}"
        )
    if x3d_readback.dtype != expected_dtype:
        raise RuntimeError(
            f"読み戻し検証失敗: x3d_world の dtype が不一致 "
            f"期待={expected_dtype} 実際={x3d_readback.dtype}"
        )

    invalid_mask_out = ~np.isfinite(x3d_readback)
    if not np.array_equal(invalid_mask_out, invalid_mask_in):
        raise RuntimeError(
            "読み戻し検証失敗: 無効サンプル（非有限値）の位置が入力と不一致"
        )

    if not np.array_equal(d["frame_ids"], frame_ids):
        raise RuntimeError("読み戻し検証失敗: frame_ids が入力と不一致")

    if not np.array_equal(d["joint_names"], joint_names_arr):
        raise RuntimeError("読み戻し検証失敗: joint_names が入力と不一致")

    for key, arr in extras.items():
        if not np.array_equal(d[key], arr):
            raise RuntimeError(f"読み戻し検証失敗: 追加キー {key} が入力と不一致")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="NPZキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相）",
        allow_abbrev=False,
    )
    parser.add_argument("npz_path", help="入力 NPZ パス（規約NPZ）")
    parser.add_argument("--fps", type=float, required=True,
                        help="サンプリング周波数 [Hz]（必須。NPZには記録されていない）")
    parser.add_argument("--output", default=None,
                        help="出力 NPZ パス（省略時: <入力>_filtered.npz）")
    parser.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF,
                        help=f"ローパスのカットオフ周波数 [Hz]（default: {DEFAULT_CUTOFF}）")
    parser.add_argument("--max-gap", type=int, default=DEFAULT_MAX_GAP,
                        help="線形補間で埋める無効サンプル連続数の上限 [フレーム]"
                             f"（default: {DEFAULT_MAX_GAP}）。超えるギャップはセグメント分割")
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    out_path = args.output if args.output else (
        os.path.splitext(args.npz_path)[0] + "_filtered.npz")

    # NPZ読み込み前の検証（重い処理の前に弾く）
    if os.path.splitext(out_path)[1].lower() != ".npz":
        print(f"エラー: 出力パスの拡張子が .npz ではありません: {out_path}", file=sys.stderr)
        return 1
    same_path = os.path.realpath(out_path) == os.path.realpath(args.npz_path)
    if not same_path and os.path.exists(out_path):
        try:
            same_path = os.path.samefile(args.npz_path, out_path)
        except OSError:
            same_path = False
    if same_path:
        print(f"エラー: 出力パスが入力 NPZ と同一です（上書き防止）: {out_path}",
              file=sys.stderr)
        return 1
    if args.fps <= 0:
        print(f"エラー: --fps は正の値を指定してください: {args.fps}", file=sys.stderr)
        return 1
    if args.cutoff <= 0:
        print(f"エラー: カットオフ周波数は正の値を指定してください: {args.cutoff}",
              file=sys.stderr)
        return 1
    if args.max_gap <= 0:
        print(f"エラー: --max-gap は正の整数を指定してください: {args.max_gap}",
              file=sys.stderr)
        return 1
    if args.cutoff >= args.fps / 2.0:
        print(f"エラー: カットオフ周波数（{args.cutoff} Hz）が Nyquist周波数"
              f"（{args.fps / 2.0} Hz）以上です", file=sys.stderr)
        return 1

    try:
        x3d, frame_ids, joint_names_arr, extras, x3d_dtype = load_npz_keypoints(
            args.npz_path)

        F, J = x3d.shape[0], x3d.shape[1]
        joint_names_list = [str(n) for n in joint_names_arr.reshape(-1)]
        extra_keys = ", ".join(extras.keys()) if extras else "なし"

        print(f"NPZ読み込み: {args.npz_path}")
        print(f"  フレーム数 F={F}, 関節数 J={J}, dtype={x3d_dtype}, 追加キー: {extra_keys}")
        print(f"フィルタ: Butterworth 2次 filtfilt（ゼロ位相）, カットオフ {args.cutoff} Hz, "
              f"fps {args.fps} Hz, 最大補間ギャップ {args.max_gap} フレーム")

        invalid_mask_in = ~np.isfinite(x3d)
        valid_fj = ~invalid_mask_in.all(axis=2)  # (F, J)
        invalid_counts = (~valid_fj).sum(axis=0)  # (J,)
        invalid_report = ", ".join(
            f"{joint_names_list[j]}={int(n)}" for j, n in enumerate(invalid_counts) if n > 0)
        print(f"  無効サンプル: {invalid_report if invalid_report else 'なし'}")

        filtered, skipped = filter_all_joints(x3d, args.cutoff, args.fps, args.max_gap)
        for j, lo, hi in skipped:
            print(f"警告: 関節 {joint_names_list[j]} のセグメント [{lo}..{hi}]"
                  f"（長さ {hi - lo + 1} < {MIN_FILTER_SAMPLES}）は平滑化をスキップしました")

        valid_all = ~invalid_mask_in.any(axis=2)  # (F, J) 座標3成分すべて有限
        if valid_all.any():
            disp = np.linalg.norm(filtered - x3d, axis=2)[valid_all] * 1000.0
            print(f"  平滑化変位 [mm]: 平均={disp.mean():.3f}, 最大={disp.max():.3f}")
        else:
            print("  平滑化変位 [mm]: 有効サンプルなし")

        pair_valid = valid_all[:-1] & valid_all[1:]  # (F-1, J)
        if pair_valid.any():
            before = np.linalg.norm(x3d[1:] - x3d[:-1], axis=2)[pair_valid] * 1000.0
            after = np.linalg.norm(filtered[1:] - filtered[:-1], axis=2)[pair_valid] * 1000.0
            print(f"  フレーム間変位 [mm]: 平滑化前 平均={before.mean():.3f}, "
                  f"平滑化後 平均={after.mean():.3f}")
        else:
            print("  フレーム間変位 [mm]: 有効サンプルなし")

        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        write_npz(out_path, filtered, x3d_dtype, invalid_mask_in,
                  frame_ids, joint_names_arr, extras)
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    print(f"完了: {out_path}（{F} フレーム, {J} 関節）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

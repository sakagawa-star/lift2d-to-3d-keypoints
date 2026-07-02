"""C3Dキーポイントの時間方向平滑化スクリプト（feat-020）。

リフトアップ推定由来の3DキーポイントC3D（`npz_to_c3d.py` 出力）はフレーム間ジッターを
含むため、全マーカー軌跡を2次Butterworthローパスフィルタの filtfilt（実効4次・ゼロ位相）
で平滑化し、新しいC3Dファイルとして書き出す。平滑化済みC3Dは Blender io_anim_c3d の
取り込み（→ fps_camera_pose.py）と render_keypoints.py の双方で利用できる。

入力は本プロジェクト規約のC3D（単位 mm、スクリーン軸 +Z/+Y。feat-018 で確定）に限定し、
規約外はエラー終了する（単位・軸の暗黙変更によるデータ破損を防ぐため）。

無効サンプル（residual < 0 / NaN）は、長さが --max-gap 以下のギャップのみ線形補間して
フィルタ計算に使い、超えるギャップではセグメント分割して独立にフィルタする。
出力では無効サンプルを無効のまま維持する（補間値を有効値として書き出さない）。

実行（phase4 venv。Blender・GPU不要）:
  uv run --project phase4 python filter_c3d.py data/keypoints.c3d
  uv run --project phase4 python filter_c3d.py data/keypoints.c3d --output data/out.c3d --cutoff 4.0
"""

import argparse
import os
import sys

import numpy as np


MIN_FILTER_SAMPLES = 10       # filtfilt 既定 padlen(9) + 1
DEFAULT_CUTOFF = 6.0          # カットオフ周波数の既定値 [Hz]
DEFAULT_MAX_GAP = 10          # 最大補間ギャップ長の既定値 [フレーム]
MAX_FIRST_FRAME = 65534       # py-c3d 16bit制約による開始フレーム上限

# 本プロジェクト規約のC3Dメタデータ（npz_to_c3d.py と同一。入力検証と出力書き込みに使う）
POINT_UNITS = "mm"
X_SCREEN = "+Z"
Y_SCREEN = "+Y"


def _param_str(reader, key: str):
    """py-c3d Reader からパラメータの文字列値を取り出す。欠落時は None。"""
    param = reader.get(key)
    if param is None:
        return None
    return param.string_value.strip()


def load_c3d(c3d_path: str) -> tuple[np.ndarray, np.ndarray, list[str], float, int]:
    """C3Dを読み込み、(points_mm, residuals, labels, point_rate, first_frame) を返す。

    本プロジェクト規約（単位 mm、スクリーン軸 +Z/+Y）の検証も行う。

    Returns:
        points_mm:   (F, J, 3) float64, C3D raw座標 mm
        residuals:   (F, J) float64（負値は無効サンプル）
        labels:      長さ J の str リスト（strip済み）
        point_rate:  float [Hz]。取得不能（None・非数値）なら 0.0
        first_frame: int（開始フレーム番号）

    Raises:
        ValueError: フレーム0件・単位/スクリーン軸が規約外の場合。
    """
    import c3d  # phase4 venv のみ。関数内 import

    with open(c3d_path, "rb") as f:
        reader = c3d.Reader(f)
        labels = [label.strip() for label in reader.point_labels]

        units = _param_str(reader, "POINT:UNITS")
        if units != POINT_UNITS:
            raise ValueError(
                f"入力C3Dの単位（POINT:UNITS）が {POINT_UNITS} ではありません: {units!r}。"
                f"本スクリプトの対象は本プロジェクト規約のC3D（npz_to_c3d.py 出力）に限定されます"
            )
        x_screen = _param_str(reader, "POINT:X_SCREEN")
        y_screen = _param_str(reader, "POINT:Y_SCREEN")
        if x_screen != X_SCREEN or y_screen != Y_SCREEN:
            raise ValueError(
                f"入力C3Dのスクリーン軸が規約（X_SCREEN={X_SCREEN}, Y_SCREEN={Y_SCREEN}）と"
                f"一致しません（欠落もエラー）: X_SCREEN={x_screen!r}, Y_SCREEN={y_screen!r}"
            )

        raw_rate = getattr(reader, "point_rate", None)
        try:
            point_rate = float(raw_rate)
        except (TypeError, ValueError):
            point_rate = 0.0
        first_frame = int(reader.first_frame)

        points_list = []
        residual_list = []
        for _frame_no, points, _analog in reader.read_frames():
            points_list.append(np.asarray(points[:, :3], dtype=np.float64))
            residual_list.append(np.asarray(points[:, 3], dtype=np.float64))

    if not points_list:
        raise ValueError(f"C3Dにフレームがありません: {c3d_path}")
    return (np.stack(points_list), np.stack(residual_list),
            labels, point_rate, first_frame)


def lowpass_filtfilt(series: np.ndarray, cutoff: float, rate: float) -> np.ndarray:
    """2次Butterworthローパスを filtfilt でゼロ位相適用する（実効4次）。

    Args:
        series: (N,) または (N, C)。時間軸は axis=0。N >= MIN_FILTER_SAMPLES であること。
        cutoff: カットオフ周波数 [Hz]（0 < cutoff < rate/2 は呼び出し側で検証済み）
        rate:   サンプリング周波数 [Hz]

    Returns:
        同形状の平滑化済み配列。
    """
    from scipy.signal import butter, filtfilt  # phase4 venv のみ。関数内 import

    b, a = butter(N=2, Wn=cutoff / (rate / 2.0), btype="low")
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


def filter_marker(
    points_mm: np.ndarray, residuals: np.ndarray,
    cutoff: float, rate: float, max_gap: int,
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """マーカー1本分 (F,3) をセグメント単位で平滑化する。

    Returns:
        (平滑化後座標 (F,3), スキップしたセグメント [(lo, hi), ...])。
        無効サンプル位置・セグメント外は入力値のまま。
    """
    valid = (residuals >= 0) & np.isfinite(points_mm).all(axis=1)
    out = points_mm.copy()
    skipped = []
    for lo, hi in split_segments(valid, max_gap):
        seg_len = hi - lo + 1
        if seg_len < MIN_FILTER_SAMPLES:
            skipped.append((lo, hi))
            continue
        seg_valid = valid[lo:hi + 1]
        seg = points_mm[lo:hi + 1].copy()
        if not seg_valid.all():
            # ギャップ（max_gap 以下が保証済み）を成分ごとに線形補間で埋める
            t = np.arange(seg_len)
            for c in range(3):
                seg[~seg_valid, c] = np.interp(
                    t[~seg_valid], t[seg_valid], seg[seg_valid, c])
        filtered = lowpass_filtfilt(seg, cutoff, rate)
        # 有効位置のみ置き換え（補間値は出力に反映しない）
        rows = np.flatnonzero(seg_valid) + lo
        out[rows] = filtered[rows - lo]
    return out, skipped


def filter_all_markers(
    points_mm: np.ndarray, residuals: np.ndarray,
    cutoff: float, rate: float, max_gap: int,
) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    """(F, J, 3) 全マーカーを平滑化する。

    Returns:
        (平滑化後 (F,J,3), スキップした (マーカー添字, lo, hi) のリスト)。
    """
    out = points_mm.copy()
    skipped_all = []
    for j in range(points_mm.shape[1]):
        filtered, skipped = filter_marker(
            points_mm[:, j, :], residuals[:, j], cutoff, rate, max_gap)
        out[:, j, :] = filtered
        skipped_all.extend((j, lo, hi) for lo, hi in skipped)
    return out, skipped_all


def write_c3d(
    out_path: str, points_mm: np.ndarray, residuals: np.ndarray,
    labels: list[str], rate: float, first_frame: int,
) -> None:
    """平滑化後の座標をC3Dへアトミックに書き出す。

    residual は入力から引き継ぐ（無効サンプルは座標 0.0・residual -1.0）。
    一時ファイルに書いて読み戻し検証に成功した場合のみ os.replace で確定する。

    Raises:
        ValueError:   フレーム0件・関節0件の場合。
        RuntimeError: 読み戻し検証に失敗した場合。
    """
    import c3d  # phase4 venv のみ。関数内 import

    points_mm = np.asarray(points_mm, dtype=np.float64)
    F, J = points_mm.shape[0], points_mm.shape[1]
    if F == 0:
        raise ValueError("書き出すフレームがありません（F == 0）")
    if J == 0:
        raise ValueError("関節がありません（J == 0）")

    writer = c3d.Writer(point_rate=rate, point_units=POINT_UNITS)
    writer.set_point_labels(labels)
    writer.set_screen_axis(X_SCREEN, Y_SCREEN)
    writer.set_start_frame(first_frame)

    empty_analog = np.zeros((0, 1), dtype=np.float32)
    for f in range(F):
        valid = (residuals[f] >= 0) & np.isfinite(points_mm[f]).all(axis=1)  # (J,)
        points = np.zeros((J, 5), dtype=np.float32)
        points[:, 0:3] = np.where(valid[:, None], points_mm[f], 0.0)
        points[:, 3] = np.where(valid, residuals[f], -1.0)
        points[:, 4] = 0.0  # camera mask
        writer.add_frames([(points, empty_analog)])

    # アトミック書き出し: tmp に書く → 読み戻し検証 → os.replace
    tmp_path = out_path + ".tmp"
    try:
        with open(tmp_path, "wb") as h:
            writer.write(h)
        _verify_c3d(tmp_path, F, labels)
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
        description="C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相）",
        allow_abbrev=False,
    )
    parser.add_argument("c3d_path", help="入力 C3D パス")
    parser.add_argument("--output", default=None,
                        help="出力 C3D パス（省略時: <入力>_filtered.c3d）")
    parser.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF,
                        help=f"ローパスのカットオフ周波数 [Hz]（default: {DEFAULT_CUTOFF}）")
    parser.add_argument("--rate", type=float, default=None,
                        help="C3Dの point rate が欠損している場合の補完値 [Hz]。"
                             "有効な point rate を持つC3Dへの指定はエラー")
    parser.add_argument("--max-gap", type=int, default=DEFAULT_MAX_GAP,
                        help="線形補間で埋める無効サンプル連続数の上限 [フレーム]"
                             f"（default: {DEFAULT_MAX_GAP}）。超えるギャップはセグメント分割")
    return parser


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)

    out_path = args.output if args.output else (
        os.path.splitext(args.c3d_path)[0] + "_filtered.c3d")

    # C3D読み込み前の検証（重い処理の前に弾く）
    if os.path.splitext(out_path)[1].lower() != ".c3d":
        print(f"エラー: 出力パスの拡張子が .c3d ではありません: {out_path}", file=sys.stderr)
        return 1
    same_path = os.path.realpath(out_path) == os.path.realpath(args.c3d_path)
    if not same_path and os.path.exists(out_path):
        try:
            same_path = os.path.samefile(args.c3d_path, out_path)
        except OSError:
            same_path = False
    if same_path:
        print(f"エラー: 出力パスが入力 C3D と同一です（上書き防止）: {out_path}",
              file=sys.stderr)
        return 1
    if args.cutoff <= 0:
        print(f"エラー: カットオフ周波数は正の値を指定してください: {args.cutoff}",
              file=sys.stderr)
        return 1
    if args.max_gap <= 0:
        print(f"エラー: --max-gap は正の整数を指定してください: {args.max_gap}",
              file=sys.stderr)
        return 1

    try:
        points_mm, residuals, labels, point_rate, first_frame = load_c3d(args.c3d_path)
        F, J = points_mm.shape[0], points_mm.shape[1]

        if not (1 <= first_frame <= MAX_FIRST_FRAME):
            print(f"エラー: 開始フレーム番号が対応範囲（1〜{MAX_FIRST_FRAME}）外です: "
                  f"{first_frame}（py-c3d 16bit制約）", file=sys.stderr)
            return 1

        # rate 決定: 有効な point rate があれば --rate は禁止。欠損時は --rate 必須
        if point_rate > 0:
            if args.rate is not None:
                print(f"エラー: C3Dに有効な point rate（{point_rate} Hz）があるため "
                      f"--rate（{args.rate} Hz）は指定できません", file=sys.stderr)
                return 1
            rate = point_rate
        else:
            if args.rate is None or args.rate <= 0:
                print("エラー: C3Dから point rate が取得できません。"
                      "--rate でサンプリング周波数 [Hz] を指定してください", file=sys.stderr)
                return 1
            rate = args.rate

        if args.cutoff >= rate / 2.0:
            print(f"エラー: カットオフ周波数（{args.cutoff} Hz）が Nyquist周波数"
                  f"（{rate / 2.0} Hz）以上です", file=sys.stderr)
            return 1

        print(f"C3D読み込み: {args.c3d_path}")
        print(f"  フレーム数 F={F}, 関節数 J={J}, point rate={rate} Hz, "
              f"開始フレーム={first_frame}")
        print(f"フィルタ: Butterworth 2次 filtfilt（ゼロ位相）, カットオフ {args.cutoff} Hz, "
              f"最大補間ギャップ {args.max_gap} フレーム")

        valid = (residuals >= 0) & np.isfinite(points_mm).all(axis=2)  # (F, J)
        invalid_counts = (~valid).sum(axis=0)  # (J,)
        invalid_report = ", ".join(
            f"{labels[j]}={int(n)}" for j, n in enumerate(invalid_counts) if n > 0)
        print(f"  無効サンプル: {invalid_report if invalid_report else 'なし'}")

        filtered, skipped = filter_all_markers(
            points_mm, residuals, args.cutoff, rate, args.max_gap)
        for j, lo, hi in skipped:
            print(f"警告: マーカー {labels[j]} のセグメント [{lo}..{hi}]"
                  f"（長さ {hi - lo + 1} < {MIN_FILTER_SAMPLES}）は平滑化をスキップしました")

        if valid.any():
            disp = np.linalg.norm(filtered - points_mm, axis=2)[valid]
            print(f"  平滑化変位 [mm]: 平均={disp.mean():.3f}, 最大={disp.max():.3f}")

        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        write_c3d(out_path, filtered, residuals, labels, rate, first_frame)
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    print(f"完了: {out_path}（{F} フレーム, {J} 関節）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

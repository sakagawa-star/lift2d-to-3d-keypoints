"""NPZ直読みによる一人称視点（FPS）動画一括生成（Blender廃止）

巨大NPZ（5〜10時間分のリフトアップ済み3Dキーポイント）を直読みし、頭部キーポイントから
一人称視点カメラポーズを numpy で計算して、3DGS（PLY）シーンを gsplat でレンダリングした
1本のMP4動画を生成するバッチプログラム（feat-027）。

現行の Blender 経由フロー（NPZ → C3D → Blender取り込み → fps_camera_pose.py →
render.py）を廃止し、手動作業ゼロで長時間データをバッチ生成する。

実行（phase4 venv、GPUが必要。TORCH_CUDA_ARCH_LIST はローカル機のみ必須。CLAUDE.md参照）:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python render_fps_video.py \\
      data/<PLY> data/<NPZ> --toml data/<TOML> --camera <カメラ名> --fps 30 --gpu 0

ポーズダンプモード（GPU不要）:
  uv run --project phase4 python render_fps_video.py <PLY> <NPZ> \\
      --toml <TOML> --camera <カメラ名> --fps 30 --dump-poses poses.json
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

from npz_to_c3d import load_npz  # トップレベル import 可（numpyのみ依存）


# === 定数 ===

# 頭部7点の関節名（順序固定。fps_camera_pose.py REQUIRED_BONES と同一）
HEAD_JOINT_NAMES = ["LEye", "REye", "LEar", "REar", "Nose", "Head", "Neck"]

EPS = 1e-8         # 正規化前ベクトル長の縮退判定しきい値
ORTHO_TOL = 1e-6   # 回転行列の行列式・直交性の許容誤差


# === FR-001: NPZ読み込みと検証（頭部関節解決） ===

def resolve_head_indices(joint_names: list[str]) -> dict[str, int]:
    """joint_names から頭部7点のインデックスを解決する。

    Args:
        joint_names: NPZ の joint_names（長さJ）。

    Returns:
        {"LEye": idx, "REye": idx, ..., "Neck": idx} の7要素辞書。

    Raises:
        ValueError: 頭部7点のうち1つでも joint_names に存在しない場合
            （欠落名を列挙したメッセージ）。
    """
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    missing = [name for name in HEAD_JOINT_NAMES if name not in name_to_idx]
    if missing:
        raise ValueError(
            f"NPZのjoint_namesに頭部関節がありません: {', '.join(missing)}"
        )
    return {name: name_to_idx[name] for name in HEAD_JOINT_NAMES}


# === FR-002: カメラ内部パラメータの読み込み ===

def load_intrinsics(toml_path: str, camera_name: str) -> dict:
    """Calib_scene.toml型TOMLからレンダリングカメラのKと解像度を読み込む。

    歪み係数D・外部パラメータ(rotation/translation)は読むが使用しない
    （ピンホールモデル。外部パラメータは頭部キーポイントから計算するため）。

    Args:
        toml_path: TOMLパス。
        camera_name: TOML内のカメラ名。

    Returns:
        {"fx","fy","cx","cy": float, "width","height": int}

    Raises:
        ValueError: カメラ名が存在しない（render_keypoints.select_camera 由来）、
            または解像度が奇数の場合。
    """
    from render_keypoints import load_cameras_toml, select_camera  # 関数内 import（tomli/cv2依存）

    cameras = load_cameras_toml(toml_path)
    cam = select_camera(cameras, camera_name)

    width, height = cam["width"], cam["height"]
    if width % 2 != 0 or height % 2 != 0:
        raise ValueError(
            f"解像度が偶数ではありません（libx264 yuv420p 制約）: {width}x{height}"
        )

    K = cam["K"]
    return {
        "fx": float(K[0, 0]), "fy": float(K[1, 1]),
        "cx": float(K[0, 2]), "cy": float(K[1, 2]),
        "width": int(width), "height": int(height),
    }


# === FR-003 / FR-004: カメラポーズ計算とフレーム有効性判定 ===

def compute_fps_poses(
    x3d_world: np.ndarray, head_idx: dict[str, int]
) -> tuple[np.ndarray, np.ndarray, list[tuple[int, str]]]:
    """全フレームの頭部7点からFPSカメラの c2w を numpy で計算する（ベクトル化）。

    計算式は fps_camera_pose.py の compute_anchor_rotation_euler() と同一（正本）。

    Args:
        x3d_world: (F, J, 3) float64, world座標[m]。
        head_idx: resolve_head_indices() の戻り値。

    Returns:
        c2w_b: (F, 4, 4) float64, Blenderカメラ規約のc2w。無効フレームは全要素NaN。
        valid: (F,) bool, 有効フレームマスク。
        invalid_reasons: (配列インデックス, 理由文字列) のリスト。
            縮退・回転検証失敗のみを含む（NaN由来は警告なし仕様のため含めない）。
    """
    x3d_world = np.asarray(x3d_world, dtype=np.float64)
    F = x3d_world.shape[0]

    LEye = x3d_world[:, head_idx["LEye"], :]
    REye = x3d_world[:, head_idx["REye"], :]
    LEar = x3d_world[:, head_idx["LEar"], :]
    REar = x3d_world[:, head_idx["REar"], :]
    Nose = x3d_world[:, head_idx["Nose"], :]
    Head = x3d_world[:, head_idx["Head"], :]
    Neck = x3d_world[:, head_idx["Neck"], :]

    eyes_mid = (LEye + REye) / 2.0
    ear_mid = (LEar + REar) / 2.0

    head_pts = np.stack([LEye, REye, LEar, REar, Nose, Head, Neck], axis=1)  # (F,7,3)
    finite = np.isfinite(head_pts).all(axis=(1, 2))  # (F,)

    invalid_reasons: list[tuple[int, str]] = []
    undetermined = finite.copy()  # まだ有効候補として残っているフレーム

    def _mark(cond: np.ndarray, reason: str) -> None:
        nonlocal undetermined
        hit = undetermined & cond
        for i in np.nonzero(hit)[0]:
            invalid_reasons.append((int(i), reason))
        undetermined = undetermined & ~hit

    with np.errstate(invalid="ignore", divide="ignore"):
        p_raw = Nose - ear_mid
        p_len = np.linalg.norm(p_raw, axis=1)
        _mark(p_len < EPS, "Nose-ear_mid 縮退")

        r0_raw = LEye - REye
        r0_len = np.linalg.norm(r0_raw, axis=1)
        _mark(r0_len < EPS, "LEye-REye 縮退")

        p_len_safe = np.where(p_len < EPS, 1.0, p_len)
        p = p_raw / p_len_safe[:, None]
        r0_len_safe = np.where(r0_len < EPS, 1.0, r0_len)
        r0 = r0_raw / r0_len_safe[:, None]

        dot_pr0 = np.sum(p * r0, axis=1)
        f_raw = p - r0 * dot_pr0[:, None]
        f_len = np.linalg.norm(f_raw, axis=1)
        _mark(f_len < EPS, "f(p⊥r0) 縮退")

        f_len_safe = np.where(f_len < EPS, 1.0, f_len)
        f = f_raw / f_len_safe[:, None]

        u_raw = np.cross(f, r0)
        u_len = np.linalg.norm(u_raw, axis=1)
        u_len_safe = np.where(u_len < EPS, 1.0, u_len)
        u = u_raw / u_len_safe[:, None]

        hu_raw = Head - Neck
        hu_len = np.linalg.norm(hu_raw, axis=1)
        _mark(hu_len < EPS, "Head-Neck 縮退")

        hu_len_safe = np.where(hu_len < EPS, 1.0, hu_len)
        head_up = hu_raw / hu_len_safe[:, None]

        dot_u_hu = np.sum(u * head_up, axis=1)
        flip = dot_u_hu < 0
        u = np.where(flip[:, None], -u, u)

        b = -f
        r_raw = np.cross(u, b)
        r_len = np.linalg.norm(r_raw, axis=1)
        r_len_safe = np.where(r_len < EPS, 1.0, r_len)
        r = r_raw / r_len_safe[:, None]

        # 回転行列。列 = (r, u, b)
        R = np.stack([r, u, b], axis=-1)  # (F,3,3)

        nonfinite_R = ~np.isfinite(R).all(axis=(1, 2))
        det = np.linalg.det(R)
        RRt = np.matmul(R, np.transpose(R, (0, 2, 1)))
        max_dev = np.max(np.abs(RRt - np.eye(3)[None, :, :]), axis=(1, 2))
        rot_bad = nonfinite_R | (np.abs(det - 1.0) > ORTHO_TOL) | (max_dev > ORTHO_TOL)

        hit = undetermined & rot_bad
        for i in np.nonzero(hit)[0]:
            invalid_reasons.append(
                (int(i), f"回転行列検証失敗(det={det[i]:.6g}, max_dev={max_dev[i]:.6g})")
            )
        undetermined = undetermined & ~hit

    valid = undetermined

    c2w_b = np.full((F, 4, 4), np.nan, dtype=np.float64)
    c2w_b[valid, :3, :3] = R[valid]
    c2w_b[valid, :3, 3] = eyes_mid[valid]
    c2w_b[valid, 3, :3] = 0.0
    c2w_b[valid, 3, 3] = 1.0

    return c2w_b, valid, invalid_reasons


def c2w_blender_to_viewmats(c2w_b: np.ndarray) -> np.ndarray:
    """Blenderカメラ規約のc2wをgsplat用viewmat（OpenCV規約）に変換する。

    render.py の blender_to_opencv_c2w と同一式:
    c2w_cv = c2w_b @ diag(1,-1,-1,1)。viewmat = inv(c2w_cv) を解析的に計算する。
    無効フレーム（c2w_bが非有限）は viewmat も全要素NaNとする。

    Args:
        c2w_b: (F, 4, 4) float64。

    Returns:
        (F, 4, 4) float32。
    """
    c2w_b = np.asarray(c2w_b, dtype=np.float64)
    F = c2w_b.shape[0]
    valid = np.isfinite(c2w_b).all(axis=(1, 2))

    with np.errstate(invalid="ignore"):
        convert = np.diag([1.0, -1.0, -1.0, 1.0])
        c2w_cv = c2w_b @ convert

        R_cv = c2w_cv[:, :3, :3]
        t_cv = c2w_cv[:, :3, 3]
        R_cv_T = np.transpose(R_cv, (0, 2, 1))

        viewmats = np.zeros((F, 4, 4), dtype=np.float64)
        viewmats[:, :3, :3] = R_cv_T
        viewmats[:, :3, 3] = -np.einsum("fij,fj->fi", R_cv_T, t_cv)
        viewmats[:, 3, 3] = 1.0

    viewmats[~valid] = np.nan
    return viewmats.astype(np.float32)


# === FR-005: チャンク計画 ===

def plan_chunks(num_frames: int, chunk_size: int, frame_ids: np.ndarray) -> list[dict]:
    """全フレームを固定フレーム数のチャンクに機械分割する。

    Args:
        num_frames: 総フレーム数。
        chunk_size: チャンクあたりのフレーム数。
        frame_ids: (num_frames,) 絶対フレーム番号（昇順連番）。

    Returns:
        [{"index", "i0", "i1", "fid0", "fid1"}, ...]（i0/i1は配列インデックス、i1は排他。
        fid0/fid1はframe_id、fid1は包含）。
    """
    frame_ids = np.asarray(frame_ids)
    n_chunks = (num_frames + chunk_size - 1) // chunk_size if num_frames > 0 else 0

    chunks = []
    for k in range(n_chunks):
        i0 = k * chunk_size
        i1 = min((k + 1) * chunk_size, num_frames)
        chunks.append({
            "index": k,
            "i0": i0, "i1": i1,
            "fid0": int(frame_ids[i0]),
            "fid1": int(frame_ids[i1 - 1]),
        })
    return chunks


def chunk_filename(chunk: dict) -> str:
    """チャンクの区間MP4ファイル名を返す。"""
    return f"chunk_{chunk['index']:05d}_{chunk['fid0']:08d}_{chunk['fid1']:08d}.mp4"


# === FR-007: マニフェスト ===

def build_manifest(
    args: argparse.Namespace, num_frames: int, fid_first: int, fid_last: int,
    width: int, height: int,
) -> dict:
    """再開判定用マニフェスト辞書を構築する。"""
    return {
        "schema": 1,
        "npz_path": os.path.abspath(args.npz_path),
        "ply_path": os.path.abspath(args.ply_path),
        "toml_path": os.path.abspath(args.toml),
        "camera": args.camera,
        "fps": float(args.fps),
        "width": int(width),
        "height": int(height),
        "chunk_size": int(args.chunk_size),
        "crf": int(args.crf),
        "preset": args.preset,
        "num_frames": int(num_frames),
        "fid_first": int(fid_first),
        "fid_last": int(fid_last),
    }


def manifest_mismatch_keys(saved: dict, current: dict) -> list[str]:
    """保存済みマニフェストと現在のパラメータで値が異なるキーを返す。"""
    keys = sorted(set(saved.keys()) | set(current.keys()))
    return [k for k in keys if saved.get(k) != current.get(k)]


# === 耐久書き出し（fsync + os.replace） ===

def fsync_file(path: str) -> None:
    """ファイル内容をディスクにfsyncする。"""
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def fsync_dir(path: str) -> None:
    """ディレクトリエントリ（rename等）をディスクにfsyncする。"""
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def durable_replace(tmp_path: str, dst_path: str) -> None:
    """fsync_file → os.replace → fsync_dir の順でアトミックに確定する。"""
    fsync_file(tmp_path)
    os.replace(tmp_path, dst_path)
    dir_path = os.path.dirname(os.path.abspath(dst_path))
    fsync_dir(dir_path)


def write_manifest(chunk_dir: str, manifest: dict) -> None:
    """マニフェストを耐久書き出しする。"""
    tmp_path = os.path.join(chunk_dir, "manifest.json.tmp")
    dst_path = os.path.join(chunk_dir, "manifest.json")
    with open(tmp_path, "w") as f:
        json.dump(manifest, f)
        f.flush()
    durable_replace(tmp_path, dst_path)


def load_manifest(chunk_dir: str) -> dict:
    """マニフェストを読み込む。

    Raises:
        FileNotFoundError: manifest.json が存在しない場合。
        ValueError: manifest.json が JSON として読めない場合。
    """
    path = os.path.join(chunk_dir, "manifest.json")
    with open(path) as f:
        text = f.read()
    try:
        return json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(
            f"manifest.json が壊れています。--overwrite で作り直してください: {chunk_dir}"
        ) from e


def ffmpeg_has_libx264(ffmpeg_bin: str) -> bool:
    """ffmpegバイナリがlibx264エンコーダを使用可能かを検査する（render.pyのNVENC検出と同方式）。"""
    result = subprocess.run(
        [ffmpeg_bin, "-hide_banner", "-encoders"], capture_output=True, text=True
    )
    return "libx264" in result.stdout


def verify_chunk_mp4(path: str, expected_frames: int) -> tuple[bool, str]:
    """区間MP4のコンテナメタデータを検査する（全フレームデコードなし）。

    Returns:
        (合格したか, 理由文字列（合格時は空文字列）)
    """
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=nb_frames",
         "-of", "default=nokey=1:noprint_wrappers=1", path],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        return False, "コンテナ読み取り不能"
    try:
        n = int(result.stdout.strip())
    except ValueError:
        return False, "フレーム数取得不能"
    if n != expected_frames:
        return False, f"フレーム数不一致（実際{n} 期待{expected_frames}）"
    return True, ""


# === FR-005: ffmpeg起動・チャンクレンダリング ===

def start_ffmpeg_chunk(
    out_tmp_path: str, width: int, height: int, fps: float, crf: int, preset: str
) -> subprocess.Popen:
    """rawvideo(rgb24)をパイプ入力しlibx264でエンコードするffmpegプロセスを起動する。"""
    ffmpeg_bin = shutil.which("ffmpeg")
    cmd = [
        ffmpeg_bin, "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
        "-r", str(fps), "-i", "pipe:0",
        "-c:v", "libx264", "-crf", str(crf), "-preset", preset,
        "-pix_fmt", "yuv420p", "-loglevel", "error",
        "-f", "mp4", out_tmp_path,
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def render_chunk(chunk: dict, viewmats, valid, config: dict, gaussians) -> tuple[int, float]:
    """1チャンク分をレンダリング（有効フレームのみ）・エンコードし区間MP4を確定する。

    黒フレームはGPUを使わず固定バイト列を直接エンコーダに渡す。
    gaussians は全フレーム黒のチャンクでは None のまま渡され、参照されない。

    Args:
        chunk: plan_chunks() の要素。
        viewmats: (F,4,4) float32（c2w_blender_to_viewmats の戻り値）。
        valid: (F,) bool。
        config: {"width","height","fx","fy","cx","cy","fps","crf","preset",
                 "chunk_dir","black_frame"}。
        gaussians: render.load_ply() の戻り値、または None。

    Returns:
        (描画フレーム数, 所要秒)

    Raises:
        RuntimeError: ffmpegへの書き込み失敗またはffmpegの異常終了。
    """
    import torch  # 関数内 import（torch/gsplat依存）
    from render import render_frame

    width, height = config["width"], config["height"]
    chunk_dir = config["chunk_dir"]
    final_path = os.path.join(chunk_dir, chunk_filename(chunk))
    tmp_path = final_path + ".tmp"

    proc = start_ffmpeg_chunk(
        tmp_path, width, height, config["fps"], config["crf"], config["preset"]
    )
    black = config["black_frame"]
    background = torch.zeros((1, 3), dtype=torch.float32, device="cuda")

    n_rendered = 0
    write_error = None
    t0 = time.time()
    try:
        for i in range(chunk["i0"], chunk["i1"]):
            if valid[i]:
                camera = {
                    "width": width, "height": height,
                    "fx": config["fx"], "fy": config["fy"],
                    "cx": config["cx"], "cy": config["cy"],
                    "viewmat": torch.tensor(viewmats[i], dtype=torch.float32, device="cuda"),
                    "background": background,
                }
                with torch.no_grad():
                    img = render_frame(gaussians, camera)
                buf = (img.cpu().numpy() * 255).astype(np.uint8).tobytes()
                n_rendered += 1
            else:
                buf = black
            try:
                proc.stdin.write(buf)
            except (BrokenPipeError, OSError) as e:
                write_error = str(e)
                break
    except KeyboardInterrupt:
        proc.terminate()
        proc.wait()
        raise

    if proc.stdin and not proc.stdin.closed:
        try:
            proc.stdin.close()
        except OSError:
            pass
    proc.wait()

    if write_error is not None or proc.returncode != 0:
        stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
        reason = write_error if write_error is not None else f"ffmpeg終了コード {proc.returncode}"
        raise RuntimeError(f"{reason}\n{stderr}")

    sec = time.time() - t0
    durable_replace(tmp_path, final_path)
    return n_rendered, sec


# === FR-008: 連結 ===

def concat_chunks(chunk_dir: str, chunk_names: list[str], output_path: str) -> None:
    """区間MP4群をffmpeg concat demuxerで再エンコードなしに連結する。

    Raises:
        RuntimeError: ffmpegが異常終了した場合。
    """
    concat_list_path = os.path.join(chunk_dir, "concat_list.txt")
    with open(concat_list_path, "w") as f:
        for name in chunk_names:
            abs_path = os.path.abspath(os.path.join(chunk_dir, name))
            f.write(f"file '{abs_path}'\n")

    output_tmp = output_path + ".tmp"
    ffmpeg_bin = shutil.which("ffmpeg")
    cmd = [
        ffmpeg_bin, "-y",
        "-f", "concat", "-safe", "0", "-i", concat_list_path,
        "-c", "copy", "-loglevel", "error", "-f", "mp4", output_tmp,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(result.stderr)
    durable_replace(output_tmp, output_path)


# === ログ ===

def format_render_average(n_rendered: int, render_seconds: float) -> str:
    """描画フレームの平均秒/フレームを表す文字列を返す（0フレーム時はゼロ除算しない）。"""
    if n_rendered == 0:
        return "N/A（描画0フレーム）"
    return f"{render_seconds / n_rendered:.3f}秒/フレーム"


# === CLI引数の型検証 ===

def _positive_float(value: str) -> float:
    f = float(value)
    if f <= 0:
        raise argparse.ArgumentTypeError(f"正の数である必要があります: {value}")
    return f


def _positive_int(value: str) -> int:
    i = int(value)
    if i < 1:
        raise argparse.ArgumentTypeError(f"1以上の整数である必要があります: {value}")
    return i


def _nonnegative_int(value: str) -> int:
    i = int(value)
    if i < 0:
        raise argparse.ArgumentTypeError(f"0以上の整数である必要があります: {value}")
    return i


def _crf_int(value: str) -> int:
    i = int(value)
    if not (0 <= i <= 51):
        raise argparse.ArgumentTypeError(f"0〜51である必要があります: {value}")
    return i


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="NPZ直読みによる一人称視点動画一括生成（Blender廃止）",
        allow_abbrev=False,
    )
    parser.add_argument("ply_path", help="3DGS PLYファイルパス")
    parser.add_argument("npz_path", help="入力NPZパス（平滑化済み想定）")
    parser.add_argument("--toml", required=True, help="Calib_scene.toml型TOMLパス")
    parser.add_argument("--camera", required=True, help="TOML内の対象カメラ名")
    parser.add_argument("--fps", required=True, type=_positive_float,
                        help="フレームレート（float。実データは30）")
    parser.add_argument("--output", default=None,
                        help="最終MP4パス（default: <NPZ名>_fps.mp4、NPZと同ディレクトリ）")
    parser.add_argument("--gpu", type=_nonnegative_int, default=0, help="使用GPU ID（default: 0）")
    parser.add_argument("--chunk-size", type=_positive_int, default=10000,
                        help="チャンクのフレーム数（default: 10000）")
    parser.add_argument("--crf", type=_crf_int, default=18,
                        help="libx264 CRF値（0〜51、default: 18）")
    parser.add_argument("--preset", default="medium", help="libx264 preset（default: medium）")
    parser.add_argument("--overwrite", action="store_true",
                        help="チャンクディレクトリを削除して作り直す")
    parser.add_argument("--keep-chunks", action="store_true",
                        help="連結後もチャンクディレクトリを残す")
    parser.add_argument("--still-range", type=int, nargs=2, default=None,
                        metavar=("START", "END"),
                        help="デバッグ静止画モード（排他。frame_id基準、両端含む）")
    parser.add_argument("--still-dir", default=None,
                        help="静止画モードのPNG出力ディレクトリ（--still-range時必須）")
    parser.add_argument("--dump-poses", default=None,
                        help="ポーズダンプモード（排他）の出力JSONパス")
    return parser


# === main ===

def _run_dump_poses(args: argparse.Namespace, x3d_world: np.ndarray,
                    frame_ids: np.ndarray, c2w_b: np.ndarray, valid: np.ndarray) -> int:
    """FR-010: ポーズダンプモード。レンダリングせずカメラポーズのみをJSONに書き出す。"""
    F = x3d_world.shape[0]
    print(f"ポーズダンプ: {F} フレームを {args.dump_poses} に書き出します")

    frames = []
    for i in range(F):
        if valid[i]:
            frames.append({
                "frame_id": int(frame_ids[i]), "valid": True, "c2w": c2w_b[i].tolist(),
            })
        else:
            frames.append({"frame_id": int(frame_ids[i]), "valid": False, "c2w": None})

    payload = {"npz_path": os.path.abspath(args.npz_path), "num_frames": F, "frames": frames}

    out_dir = os.path.dirname(os.path.abspath(args.dump_poses))
    os.makedirs(out_dir, exist_ok=True)
    tmp_path = args.dump_poses + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(payload, f)
        f.flush()
    durable_replace(tmp_path, args.dump_poses)

    print(f"完了: {args.dump_poses}（{F} フレーム）")
    return 0


def _run_still_mode(args: argparse.Namespace, frame_ids: np.ndarray, intrinsics: dict,
                    viewmats: np.ndarray, valid: np.ndarray) -> int:
    """FR-009: デバッグ静止画モード。指定範囲を連番PNGで出力する（MP4出力なし）。"""
    start, end = args.still_range
    if start > end:
        print(f"エラー: --still-range の開始が終了より後です: {start} > {end}", file=sys.stderr)
        return 1

    fid_min, fid_max = int(frame_ids[0]), int(frame_ids[-1])
    if start < fid_min or end > fid_max:
        print(
            f"エラー: --still-range が NPZ の frame_ids 範囲外です: "
            f"[{start}, {end}]（NPZ範囲: [{fid_min}, {fid_max}]）", file=sys.stderr
        )
        return 1

    os.makedirs(args.still_dir, exist_ok=True)

    i0 = start - fid_min
    i1 = end - fid_min + 1  # 排他的
    indices = list(range(i0, i1))

    import torch  # 関数内 import（torch/gsplat依存）
    from render import load_ply, render_frame
    from PIL import Image

    torch.cuda.set_device(args.gpu)

    width, height = intrinsics["width"], intrinsics["height"]
    black = np.zeros((height, width, 3), dtype=np.uint8)

    gaussians = None
    if any(valid[i] for i in indices):
        print(f"PLYファイル読み込み中: {args.ply_path}")
        gaussians = load_ply(args.ply_path)

    background = torch.zeros((1, 3), dtype=torch.float32, device="cuda")
    for i in indices:
        fid = int(frame_ids[i])
        out_path = os.path.join(args.still_dir, f"frame_{fid:08d}.png")
        if valid[i]:
            camera = {
                "width": width, "height": height,
                "fx": intrinsics["fx"], "fy": intrinsics["fy"],
                "cx": intrinsics["cx"], "cy": intrinsics["cy"],
                "viewmat": torch.tensor(viewmats[i], dtype=torch.float32, device="cuda"),
                "background": background,
            }
            with torch.no_grad():
                img = render_frame(gaussians, camera)
            arr = (img.cpu().numpy() * 255).astype(np.uint8)
        else:
            arr = black
        Image.fromarray(arr).save(out_path)
        print(f"  {out_path}")

    print(f"完了: {args.still_dir}（{len(indices)} フレーム）")
    return 0


def _run_mp4_mode(args: argparse.Namespace, output_path: str, frame_ids: np.ndarray,
                  intrinsics: dict, viewmats: np.ndarray, valid: np.ndarray) -> int:
    """FR-005/FR-007/FR-008: チャンク分割・区間MP4レンダリング・再開・連結の本体処理。"""
    F = len(frame_ids)
    width, height = intrinsics["width"], intrinsics["height"]

    chunks = plan_chunks(F, args.chunk_size, frame_ids)
    chunk_dir = output_path + ".chunks"

    print(
        f"チャンク: {len(chunks)}個（chunk_size={args.chunk_size}）, 使用GPU: {args.gpu}, "
        f"解像度 {width}x{height}, fps={args.fps}, crf={args.crf}, preset={args.preset}"
    )
    print(f"出力: {output_path}")

    current_manifest = build_manifest(
        args, F, int(frame_ids[0]), int(frame_ids[-1]), width, height
    )

    if args.overwrite and os.path.isdir(chunk_dir):
        shutil.rmtree(chunk_dir)

    skip_indices: set[int] = set()
    if not os.path.isdir(chunk_dir):
        os.makedirs(chunk_dir)
        fsync_dir(os.path.dirname(os.path.abspath(chunk_dir)))
        write_manifest(chunk_dir, current_manifest)
    else:
        try:
            saved_manifest = load_manifest(chunk_dir)
        except FileNotFoundError:
            print(
                f"エラー: チャンクディレクトリが不完全です（manifest.json なし）。"
                f"--overwrite で作り直してください: {chunk_dir}", file=sys.stderr
            )
            return 1
        except ValueError as e:
            print(f"エラー: {e}", file=sys.stderr)
            return 1

        mismatch = manifest_mismatch_keys(saved_manifest, current_manifest)
        if mismatch:
            print(
                f"エラー: パラメータが変更されています（不一致: {', '.join(mismatch)}）。"
                f"--overwrite で作り直してください", file=sys.stderr
            )
            return 1

        for chunk in chunks:
            final_path = os.path.join(chunk_dir, chunk_filename(chunk))
            if os.path.exists(final_path):
                expected = chunk["i1"] - chunk["i0"]
                ok, reason = verify_chunk_mp4(final_path, expected)
                if ok:
                    skip_indices.add(chunk["index"])
                else:
                    print(
                        f"警告: チャンク {chunk['index']:05d} が壊れています（{reason}）。作り直します",
                        file=sys.stderr
                    )
                    os.remove(final_path)

    import torch  # 関数内 import（torch/gsplat依存）
    torch.cuda.set_device(args.gpu)

    config = {
        "width": width, "height": height,
        "fx": intrinsics["fx"], "fy": intrinsics["fy"],
        "cx": intrinsics["cx"], "cy": intrinsics["cy"],
        "fps": args.fps, "crf": args.crf, "preset": args.preset,
        "chunk_dir": chunk_dir,
        "black_frame": bytes(height * width * 3),
    }

    gaussians = None
    total_rendered = 0
    total_render_sec = 0.0
    start_time = time.time()

    try:
        for chunk in chunks:
            if chunk["index"] in skip_indices:
                print(f"チャンク {chunk['index']:05d} スキップ（完成済み）")
                continue

            chunk_valid = valid[chunk["i0"]:chunk["i1"]]
            if bool(chunk_valid.any()) and gaussians is None:
                from render import load_ply
                print(f"PLYファイル読み込み中: {args.ply_path}")
                gaussians = load_ply(args.ply_path)

            try:
                n_rendered, sec = render_chunk(chunk, viewmats, valid, config, gaussians)
            except Exception as e:
                tmp_path = os.path.join(chunk_dir, chunk_filename(chunk) + ".tmp")
                if os.path.exists(tmp_path):
                    try:
                        os.remove(tmp_path)
                    except OSError:
                        pass
                print(
                    f"エラー: チャンク {chunk['index']:05d} が失敗しました: {e}", file=sys.stderr
                )
                return 1

            total_rendered += n_rendered
            total_render_sec += sec
            n = chunk["i1"] - chunk["i0"]
            print(
                f"チャンク {chunk['index']:05d} (fid {chunk['fid0']}-{chunk['fid1']}) 完了: "
                f"描画 {n_rendered}/{n} フレーム, {sec:.1f}秒"
            )
    except KeyboardInterrupt:
        print("\n中断されました", file=sys.stderr)
        return 130

    chunk_names = [chunk_filename(c) for c in chunks]
    print(f"連結: {output_path}（{F} フレーム）")
    try:
        concat_chunks(chunk_dir, chunk_names, output_path)
    except RuntimeError as e:
        print(f"エラー: 連結に失敗しました: {e}", file=sys.stderr)
        return 1

    if not args.keep_chunks:
        shutil.rmtree(chunk_dir)
        print(f"チャンクディレクトリ削除: {chunk_dir}")

    elapsed = time.time() - start_time
    print(
        f"完了: 総所要 {elapsed:.1f}秒, "
        f"描画フレーム平均 {format_render_average(total_rendered, total_render_sec)}"
    )
    print(f"最終MP4: {output_path}（{F} フレーム）")
    return 0


def main(argv=None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    # 排他制約（重い処理より前に検証）
    if args.still_range is not None and args.dump_poses is not None:
        parser.error("--still-range と --dump-poses は同時指定できません")

    exclusive_mode = args.still_range is not None or args.dump_poses is not None
    if exclusive_mode:
        for name, given in [("--output", args.output is not None),
                            ("--overwrite", args.overwrite),
                            ("--keep-chunks", args.keep_chunks)]:
            if given:
                parser.error(f"排他モード指定時は {name} は使用できません")

    if args.still_range is not None and args.still_dir is None:
        parser.error("--still-range 指定時は --still-dir が必須です")

    # ファイル存在チェック
    for path, label in [(args.ply_path, "PLYファイル"), (args.npz_path, "NPZファイル"),
                        (args.toml, "TOMLファイル")]:
        if not os.path.exists(path):
            print(f"エラー: {label}が見つかりません: {path}", file=sys.stderr)
            return 1

    still_mode = args.still_range is not None
    dump_mode = args.dump_poses is not None
    mp4_mode = not still_mode and not dump_mode

    output_path = args.output
    if mp4_mode:
        if output_path is None:
            output_path = os.path.join(
                os.path.dirname(os.path.abspath(args.npz_path)),
                os.path.splitext(os.path.basename(args.npz_path))[0] + "_fps.mp4",
            )
        if os.path.splitext(output_path)[1].lower() != ".mp4":
            print(f"エラー: --output の拡張子が .mp4 ではありません: {output_path}",
                 file=sys.stderr)
            return 1

    if mp4_mode:
        ffmpeg_bin = shutil.which("ffmpeg")
        if ffmpeg_bin is None:
            print("エラー: ffmpegが見つかりません", file=sys.stderr)
            return 1
        if shutil.which("ffprobe") is None:
            print("エラー: ffprobeが見つかりません", file=sys.stderr)
            return 1
        if not ffmpeg_has_libx264(ffmpeg_bin):
            print("エラー: ffmpeg に libx264 エンコーダがありません", file=sys.stderr)
            return 1

    if mp4_mode or still_mode:
        import torch  # 関数内 import（torch依存）
        n_gpu = torch.cuda.device_count()
        if args.gpu >= n_gpu:
            print(
                f"エラー: 指定GPU ID {args.gpu} が範囲外です（利用可能: 0〜{n_gpu - 1}）",
                file=sys.stderr
            )
            return 1

    # NPZ読み込み・頭部7点検証
    try:
        x3d_world, frame_ids, joint_names = load_npz(args.npz_path)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    F = x3d_world.shape[0]
    if F == 0:
        print(f"エラー: NPZ にフレームがありません（F=0）: {args.npz_path}", file=sys.stderr)
        return 1
    J = x3d_world.shape[1]

    try:
        head_idx = resolve_head_indices(joint_names)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    # TOML読み込み・偶数解像度検証
    try:
        intrinsics = load_intrinsics(args.toml, args.camera)
    except ValueError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    print(f"NPZ読み込み: {args.npz_path}")
    print(f"  フレーム数 F={F}, 関節数 J={J}, frame_ids {int(frame_ids[0])}〜{int(frame_ids[-1])}")

    # カメラポーズ計算（全フレーム事前計算。ADR-6）
    c2w_b, valid, invalid_reasons = compute_fps_poses(x3d_world, head_idx)
    viewmats = c2w_blender_to_viewmats(c2w_b)

    head_indices_list = list(head_idx.values())
    finite_mask = np.isfinite(x3d_world[:, head_indices_list, :]).all(axis=(1, 2))
    n_valid = int(np.count_nonzero(valid))
    n_nan = int(np.count_nonzero(~finite_mask))
    n_degen = F - n_valid - n_nan
    print(f"有効フレーム: {n_valid} / {F}（NaN欠損: {n_nan}, 縮退: {n_degen}）")

    for i, reason in invalid_reasons:
        print(f"警告: frame_id {int(frame_ids[i])}: {reason}", file=sys.stderr)

    if dump_mode:
        return _run_dump_poses(args, x3d_world, frame_ids, c2w_b, valid)

    if still_mode:
        return _run_still_mode(args, frame_ids, intrinsics, viewmats, valid)

    return _run_mp4_mode(args, output_path, frame_ids, intrinsics, viewmats, valid)


if __name__ == "__main__":
    sys.exit(main())

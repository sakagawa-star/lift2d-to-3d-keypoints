"""Blenderシーンから頭部追従カメラのポーズを書き出すスクリプト（Blender内実行用）

FPS風頭部追従カメラの向き（顔の一次視線方向）を各フレームで計算・適用してから
cam.matrix_world を読み、blender -b（ヘッドレス）でも頭部に追従した正しい c2w を
JSON に書き出す。向きを与える frame_change_post ハンドラは .blend に保存されず
バックグラウンド別プロセスでは発火しないため、同じ計算を本スクリプトに内蔵する。

実行例:
    blender -b data/Blender/2D-Lift.blend --python fps_camera_pose.py -- --camera Cam_FPS
"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

# KIRI Engine アドオンが3DGSオブジェクトに付与するモディファイヤ名（固定）
KIRI_MODIFIER_NAME = "KIRI_3DGS_Render_GN"

# デフォルトのアーマチュア名・アンカー名（対象 .blend の構成）
DEFAULT_ARMATURE_NAME = "session001_f145749_world300"
DEFAULT_ANCHOR_NAME = "Cam_Anchor"

# 姿勢計算に使うボーン名（Halpe/参考資料準拠、固定）
BONE_LEYE = "LEye"
BONE_REYE = "REye"
BONE_LEAR = "LEar"
BONE_REAR = "REar"
BONE_NOSE = "Nose"
BONE_HEAD = "Head"
BONE_NECK = "Neck"
REQUIRED_BONES = [BONE_LEYE, BONE_REYE, BONE_LEAR, BONE_REAR, BONE_NOSE, BONE_HEAD, BONE_NECK]

EPS = 1e-8          # 正規化前ベクトル長の縮退判定しきい値
ORTHO_TOL = 1e-6    # determinant/直交性・向き整合の許容誤差
ANG_TOL = 1e-4      # カメラのローカル回転ゼロ判定（rad）
POS_TOL = 1e-4      # カメラ位置＝両目中点の許容誤差（m）


def extract_script_args(argv: list[str]) -> list[str]:
    """コマンドライン引数から '--' 以降のスクリプト引数を取り出す。

    Blender は '--' 以降の引数を無視するため、そこをスクリプト用に使う。
    '--' がない場合は空リストを返す。
    """
    if "--" in argv:
        return argv[argv.index("--") + 1:]
    return []


def default_output_path(camera_name: str) -> str:
    """デフォルトの出力JSONパス（カレントディレクトリ基準）を返す。"""
    return os.path.join("data", f"{camera_name}_poses.json")


def parse_args(argv: list[str]) -> argparse.Namespace:
    """スクリプト引数を解析する。--output 省略時はデフォルトパスを補完する。"""
    parser = argparse.ArgumentParser(
        prog="fps_camera_pose.py",
        description="Blenderシーンから頭部追従カメラのポーズを書き出す",
    )
    parser.add_argument("--camera", required=True, help="カメラオブジェクト名")
    parser.add_argument("--armature", default=DEFAULT_ARMATURE_NAME, help="アーマチュア名")
    parser.add_argument("--anchor", default=DEFAULT_ANCHOR_NAME, help="アンカーEmpty名")
    parser.add_argument("--output", default=None,
                        help="出力JSONパス (default: data/<カメラ名>_poses.json)")
    args = parser.parse_args(argv)
    if args.output is None:
        args.output = default_output_path(args.camera)
    return args


def _fail(message: str) -> None:
    """エラーメッセージを stderr に出力して終了コード1で終了する。"""
    print(message, file=sys.stderr)
    sys.exit(1)


def _require_len(vec, name: str, frame: int) -> None:
    """正規化前ベクトルの長さが EPS 未満（縮退）なら異常終了する。"""
    if vec.length < EPS:
        _fail(f"エラー: フレーム {frame}: ベクトル {name} が縮退しています (length={vec.length})")


def validate_rotation(rot, frame: int) -> None:
    """回転行列の NaN/Inf・行列式・直交性を検証し、違反なら異常終了する。"""
    for row in rot:
        for v in row:
            if not math.isfinite(v):
                _fail(f"エラー: フレーム {frame}: 回転行列に NaN/Inf が含まれます")
    det = rot.determinant()
    if abs(det - 1.0) > ORTHO_TOL:
        _fail(f"エラー: フレーム {frame}: 回転行列の行列式が1ではありません (det={det})")
    # 直交性: rot @ rot^T が単位行列に一致するか
    ident = rot @ rot.transposed()
    max_dev = max(abs(ident[i][j] - (1.0 if i == j else 0.0))
                  for i in range(3) for j in range(3))
    if max_dev > ORTHO_TOL:
        _fail(f"エラー: フレーム {frame}: 回転行列が直交していません (max_dev={max_dev})")


def compute_anchor_rotation_euler(arm_eval, frame: int):
    """評価済みアーマチュアの頭部キーポイントから一次視線方向の回転(Euler)を計算する。

    要求仕様 FR-001 の式を正本とする。取得元は評価済み depsgraph 経由の arm_eval。
    """
    from mathutils import Matrix  # 関数内 import（pytest での bpy import 回避に合わせる）

    mw = arm_eval.matrix_world
    B = arm_eval.pose.bones
    LEye = mw @ B[BONE_LEYE].head
    REye = mw @ B[BONE_REYE].head
    LEar = mw @ B[BONE_LEAR].head
    REar = mw @ B[BONE_REAR].head
    Nose = mw @ B[BONE_NOSE].head
    Head = mw @ B[BONE_HEAD].head
    Neck = mw @ B[BONE_NECK].head
    ear_mid = (LEar + REar) * 0.5

    # FR-005(a): 各 normalize 前に元ベクトル長を検証（縮退ならエラー終了）
    p_raw = Nose - ear_mid
    _require_len(p_raw, "Nose-ear_mid", frame)
    r0_raw = LEye - REye
    _require_len(r0_raw, "LEye-REye", frame)
    p = p_raw.normalized()                   # 前方の手がかり（耳中点→鼻）
    r0 = r0_raw.normalized()                 # 両目軸（直交化の基準）
    f_raw = p - r0 * p.dot(r0)               # p∥r0 で縮退
    _require_len(f_raw, "f(p⊥r0)", frame)
    f = f_raw.normalized()                   # 前方＝一次視線（両目軸に直交）
    u = f.cross(r0).normalized()             # 上（暫定）
    hu_raw = Head - Neck
    _require_len(hu_raw, "Head-Neck", frame)
    head_up = hu_raw.normalized()            # 頭頂方向（符号確定の基準）
    if u.dot(head_up) < 0:
        u = -u
    b = -f                                   # カメラ ローカル +Z = 後方
    r = u.cross(b).normalized()              # r = u×b で右手系（det=+1）を保証
    rot = Matrix((r, u, b)).transposed()     # 列 = (右, 上, 後方)
    validate_rotation(rot, frame)            # FR-005(a): det/直交/NaN・Inf 検証
    return rot.to_euler()


def validate_camera_transform(cam, rot_euler, eyes_mid, frame: int) -> None:
    """適用後の cam.matrix_world が計算値と一致するか（位置・向き）を各フレームで検証する。"""
    mw = cam.matrix_world
    # 位置整合: カメラワールド位置が両目中点と一致するか
    dist = (mw.translation - eyes_mid).length
    if dist > POS_TOL:
        _fail(f"エラー: フレーム {frame}: カメラ位置が両目中点と一致しません (dist={dist} m)")
    # 向き整合: cam.matrix_world.to_3x3() が計算した回転と一致するか
    actual = mw.to_3x3()
    expected = rot_euler.to_matrix()
    max_dev = max(abs(actual[i][j] - expected[i][j]) for i in range(3) for j in range(3))
    if max_dev > ORTHO_TOL:
        _fail(f"エラー: フレーム {frame}: カメラの向きが計算値と一致しません (max_dev={max_dev})")


def export_camera_poses(camera_name: str, armature_name: str,
                        anchor_name: str, output_path: str) -> int:
    """頭部追従カメラのポーズを全フレーム分JSONに書き出し、フレーム数を返す。"""
    import bpy  # Blender内でのみ利用可能（pytest環境でのimportエラー回避のため関数内import）

    scene = bpy.context.scene
    view_layer = bpy.context.view_layer

    # 検索範囲は現在のシーン（bpy.data.objects だと他シーンのオブジェクトも拾い、
    # frame_set のデプスグラフ評価対象外の古いポーズを黙って書き出す恐れがある）
    cam = scene.objects.get(camera_name)
    cameras = [obj.name for obj in scene.objects if obj.type == 'CAMERA']
    if cam is None:
        print(f"エラー: オブジェクト '{camera_name}' がシーンに見つかりません", file=sys.stderr)
        print(f"シーン内のカメラ: {cameras}", file=sys.stderr)
        sys.exit(1)
    if cam.type != 'CAMERA':
        print(f"エラー: オブジェクト '{camera_name}' はカメラではありません (type={cam.type})", file=sys.stderr)
        print(f"シーン内のカメラ: {cameras}", file=sys.stderr)
        sys.exit(1)

    arm = scene.objects.get(armature_name)
    armatures = [obj.name for obj in scene.objects if obj.type == 'ARMATURE']
    if arm is None:
        print(f"エラー: アーマチュア '{armature_name}' がシーンに見つかりません", file=sys.stderr)
        print(f"シーン内のアーマチュア: {armatures}", file=sys.stderr)
        sys.exit(1)
    if arm.type != 'ARMATURE':
        print(f"エラー: オブジェクト '{armature_name}' はアーマチュアではありません (type={arm.type})", file=sys.stderr)
        print(f"シーン内のアーマチュア: {armatures}", file=sys.stderr)
        sys.exit(1)

    anchor = scene.objects.get(anchor_name)
    if anchor is None:
        _fail(f"エラー: アンカー '{anchor_name}' がシーンに見つかりません")
    if anchor.type != 'EMPTY':
        _fail(f"エラー: オブジェクト '{anchor_name}' は Empty ではありません (type={anchor.type})")

    # アンカー回転が cam.matrix_world に反映される親子関係が本スクリプトの前提
    if cam.parent is not anchor:
        parent_name = cam.parent.name if cam.parent else None
        _fail(f"エラー: カメラ '{camera_name}' の親がアンカー '{anchor_name}' ではありません (parent={parent_name})")

    # カメラのローカル回転が残ると計算した視線方向に混ざる（fail-fast チェック）
    if max(abs(a) for a in cam.rotation_euler) > ANG_TOL:
        rot_str = tuple(cam.rotation_euler)
        _fail(f"エラー: カメラ '{camera_name}' のローカル回転がゼロではありません (rotation_euler={rot_str})")

    # 必要ボーンの存在チェック
    missing = [name for name in REQUIRED_BONES if name not in arm.pose.bones]
    if missing:
        _fail(f"エラー: アーマチュア '{armature_name}' に必要なボーンがありません: {', '.join(missing)}")

    if scene.frame_start > scene.frame_end:
        _fail(f"エラー: フレーム範囲が不正です (frame_start={scene.frame_start} > frame_end={scene.frame_end})")

    # 3DGSオブジェクトのKIRIモディファイヤを一時無効化（元の表示状態を記録して復元する）
    kiri_modifiers = [obj.modifiers[KIRI_MODIFIER_NAME]
                      for obj in scene.objects if KIRI_MODIFIER_NAME in obj.modifiers]
    original_states = [mod.show_viewport for mod in kiri_modifiers]
    for mod in kiri_modifiers:
        mod.show_viewport = False

    try:
        camera_data = []
        for frame in range(scene.frame_start, scene.frame_end + 1):
            scene.frame_set(frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            arm_eval = arm.evaluated_get(depsgraph)
            rot_euler = compute_anchor_rotation_euler(arm_eval, frame)  # 縮退/非直交なら内部で exit(1)
            anchor.rotation_euler = rot_euler
            view_layer.update()  # アンカー回転を子カメラ matrix_world に反映
            # FR-005(b): 適用後のカメラ変換整合性を全フレーム検証（違反なら exit(1)）
            eyes_mid = (arm_eval.matrix_world @ arm_eval.pose.bones[BONE_LEYE].head
                        + arm_eval.matrix_world @ arm_eval.pose.bones[BONE_REYE].head) * 0.5
            validate_camera_transform(cam, rot_euler, eyes_mid, frame)
            render = scene.render
            focal_px = (cam.data.lens / cam.data.sensor_width) * render.resolution_x
            camera_data.append({
                'frame': frame,
                'c2w': np.array(cam.matrix_world).tolist(),
                'fx': focal_px,
                'fy': focal_px,
                'cx': render.resolution_x / 2,
                'cy': render.resolution_y / 2,
                'width': render.resolution_x,
                'height': render.resolution_y,
            })
    finally:
        # 例外発生時もモディファイヤの表示状態を元に戻す
        for mod, state in zip(kiri_modifiers, original_states):
            mod.show_viewport = state

    # 原子的書き出し: 一時ファイルに書いてから os.replace で確定（既存JSONを破壊しない）
    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=out_dir or ".", suffix=".json.tmp")
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(camera_data, f)
            f.flush()
        os.replace(tmp_path, output_path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    return len(camera_data)


def main() -> None:
    args = parse_args(extract_script_args(sys.argv))
    num_frames = export_camera_poses(args.camera, args.armature, args.anchor, args.output)
    print(f"カメラポーズ書き出し完了: {args.output} ({num_frames} フレーム)")


if __name__ == "__main__":
    main()

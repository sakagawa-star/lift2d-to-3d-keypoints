"""Blenderシーンからカメラポーズを書き出すスクリプト（Blender内実行用）

実行例:
    blender -b data/scene.blend --python camera_pose.py -- --camera cam41520554
"""
import argparse
import json
import os
import sys

import numpy as np

# KIRI Engine アドオンが3DGSオブジェクトに付与するモディファイヤ名（固定）
KIRI_MODIFIER_NAME = "KIRI_3DGS_Render_GN"


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
        prog="camera_pose.py",
        description="Blenderシーンからカメラポーズを書き出す",
    )
    parser.add_argument("--camera", required=True, help="カメラオブジェクト名")
    parser.add_argument("--output", default=None,
                        help="出力JSONパス (default: data/<カメラ名>_poses.json)")
    args = parser.parse_args(argv)
    if args.output is None:
        args.output = default_output_path(args.camera)
    return args


def export_camera_poses(camera_name: str, output_path: str) -> int:
    """カメラポーズを全フレーム分JSONに書き出し、フレーム数を返す。"""
    import bpy  # Blender内でのみ利用可能（pytest環境でのimportエラー回避のため関数内import）

    scene = bpy.context.scene
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

    if scene.frame_start > scene.frame_end:
        print(f"エラー: フレーム範囲が不正です (frame_start={scene.frame_start} > frame_end={scene.frame_end})",
              file=sys.stderr)
        sys.exit(1)

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

    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(camera_data, f)
    return len(camera_data)


def main() -> None:
    args = parse_args(extract_script_args(sys.argv))
    num_frames = export_camera_poses(args.camera, args.output)
    print(f"カメラポーズ書き出し完了: {args.output} ({num_frames} フレーム)")


if __name__ == "__main__":
    main()

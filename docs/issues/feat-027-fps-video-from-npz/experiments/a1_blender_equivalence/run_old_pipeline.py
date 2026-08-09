"""実験 a1 フェーズ1: 旧パイプライン（fps_camera_pose.py）を実行する（criteria 第3版）。

.blend を Blender 4.5.5 でフルロードし（4.0.2 ではバージョン不整合でクラッシュする。
experiment_log.md 診断参照）、シーンフレーム範囲を比較ウィンドウ 41〜340 に設定してから、
旧スクリプト fps_camera_pose.py の export_camera_poses() をそのまま呼んでポーズJSONを書き出す。
ポーズ計算・リグ検証（親子関係・ローカル回転ゼロ・ボーン存在・カメラ変換整合）は旧実装の
関数内でそのまま実行される。

実行（リポジトリルート）:
    /home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b \
        --python docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/run_old_pipeline.py
"""
import sys
from pathlib import Path

import bpy

# 固定パラメータ（criteria.md §2 で lock。変更禁止）
EXP_DIR = Path(__file__).resolve().parent
REPO_ROOT = EXP_DIR.parents[4]
BLEND_PATH = REPO_ROOT / "phase4" / "data" / "Blender" / "session001_world_22pt.blend"
FRAME_START = 41
FRAME_END = 340
CAMERA_NAME = "Cam_FPS"
ARMATURE_NAME = "E00000"
ANCHOR_NAME = "Cam_Anchor"
OUTPUT_PATH = EXP_DIR / "old_poses.json"


def main() -> None:
    if not BLEND_PATH.is_file():
        print(f"エラー: .blend が見つかりません: {BLEND_PATH}", file=sys.stderr)
        sys.exit(1)

    # .blend をフルロード（旧パイプラインと同じ土俵。シーン・リグ・制約をそのまま使う）
    bpy.ops.wm.open_mainfile(filepath=str(BLEND_PATH))

    # 比較ウィンドウ（頭部7点が全有効な最初の300フレーム連続区間。criteria.md §2）
    scene = bpy.context.scene
    scene.frame_start = FRAME_START
    scene.frame_end = FRAME_END

    # 旧スクリプトの関数をそのまま呼ぶ（フレームループ・リグ検証・JSON書き出しは旧実装の正本）
    sys.path.insert(0, str(REPO_ROOT / "phase4"))
    import fps_camera_pose

    num_frames = fps_camera_pose.export_camera_poses(
        CAMERA_NAME, ARMATURE_NAME, ANCHOR_NAME, str(OUTPUT_PATH)
    )
    print(f"旧経路ポーズ書き出し完了: {OUTPUT_PATH} ({num_frames} フレーム, "
          f"scene frame {FRAME_START}〜{FRAME_END})")


if __name__ == "__main__":
    main()

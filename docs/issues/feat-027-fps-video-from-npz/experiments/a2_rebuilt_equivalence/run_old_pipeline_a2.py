"""実験 a2 フェーズ1: 旧パイプラインを再構築データで実行する（criteria.md §2 で lock）。

平滑化済みNPZ由来の C3D を io_anim_c3d でインポートし、既存 .blend から append した
カメラリグ（Cam_Anchor / Cam_FPS）の ARMATURE 制約を新アーマチュアに付け替えたうえで、
旧スクリプト fps_camera_pose.py の export_camera_poses() をそのまま呼んでポーズJSONを書き出す。
リグ検証（親子関係・ローカル回転ゼロ・ボーン存在・毎フレームのカメラ位置=両目中点/向き整合）は
旧実装の関数内でそのまま実行され、リグ再構築ミスがあれば exit 1 で検出される。

実行（リポジトリルート）:
    /home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b \
        --python docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py
"""
import sys
from pathlib import Path

import bpy

# 固定パラメータ（criteria.md §2 で lock。変更禁止）
EXP_DIR = Path(__file__).resolve().parent
REPO_ROOT = EXP_DIR.parents[4]
C3D_PATH = EXP_DIR / "smoothed_world300.c3d"
RIG_BLEND_PATH = REPO_ROOT / "phase4" / "data" / "Blender" / "session001_world_22pt.blend"
RIG_OBJECTS = ["Cam_Anchor", "Cam_FPS"]
FRAME_START = 1
FRAME_END = 300
CAMERA_NAME = "Cam_FPS"
ANCHOR_NAME = "Cam_Anchor"
OUTPUT_PATH = EXP_DIR / "old_poses.json"


def _fail(message: str) -> None:
    print(message, file=sys.stderr)
    sys.exit(1)


def main() -> None:
    if not C3D_PATH.is_file():
        _fail(f"エラー: C3D がありません（フェーズ0b を先に実行）: {C3D_PATH}")
    if not RIG_BLEND_PATH.is_file():
        _fail(f"エラー: リグ取り出し元の .blend がありません: {RIG_BLEND_PATH}")

    # 1. シーン fps を C3D と同じ 30 に明示設定してからインポートする
    #    （io_anim_c3d のレート適応 adapt_frame_rate が scene fps / C3D fps でキー時刻を
    #    スケールするため、ヘッドレス既定の 24fps のままだと 30fps C3D のキーが 0.8 刻みに
    #    化ける。fps=30 に揃えればスケール比 1.0 で無害化される）
    scene = bpy.context.scene
    scene.render.fps = 30
    scene.render.fps_base = 1.0

    # 2. C3D をインポートし、新アーマチュアをインポート前後差分で特定する
    #    adapt_frame_rate プロパティが存在するバージョンでは False を明示する（二重防御）
    before = set(bpy.data.objects)
    import_kwargs = {"filepath": str(C3D_PATH)}
    op_props = bpy.ops.import_anim.c3d.get_rna_type().properties.keys()
    if "adapt_frame_rate" in op_props:
        import_kwargs["adapt_frame_rate"] = False
    result = bpy.ops.import_anim.c3d(**import_kwargs)
    if result != {"FINISHED"}:
        _fail(f"エラー: C3D インポートに失敗しました: {result}")
    new_armatures = [o for o in bpy.data.objects
                     if o not in before and o.type == "ARMATURE"]
    if len(new_armatures) != 1:
        _fail(f"エラー: インポートで生成されたアーマチュアが1体ではありません: "
              f"{[o.name for o in new_armatures]}")
    arm = new_armatures[0]
    print(f"C3D インポート完了: アーマチュア '{arm.name}' ({len(arm.pose.bones)} ボーン)")

    # 3. キーフレームを「Blender frame 1 = 先頭サンプル」に正規化する
    #    （importer バージョンにより先頭キーが frame 0 に入るため、一様シフトで揃える。
    #    シフト後にキー範囲が 1〜300 でなければレート適応等の混入としてエラー終了）
    action = arm.animation_data.action
    fr0, fr1 = action.frame_range
    shift = 1.0 - fr0
    if abs(shift - round(shift)) > 1e-6:
        _fail(f"エラー: 先頭キーが整数フレームに乗っていません（fpsスケーリング疑い）: "
              f"frame_range=({fr0}, {fr1})")
    if shift != 0.0:
        for fcurve in action.fcurves:
            for kp in fcurve.keyframe_points:
                kp.co.x += shift
                kp.handle_left.x += shift
                kp.handle_right.x += shift
            fcurve.update()
        print(f"キーフレームを {shift:+.0f} シフト（先頭を frame 1 に正規化）")
    fr0, fr1 = arm.animation_data.action.frame_range
    if not (abs(fr0 - FRAME_START) < 1e-6 and abs(fr1 - FRAME_END) < 1e-6):
        _fail(f"エラー: 正規化後のキー範囲が {FRAME_START}〜{FRAME_END} ではありません: "
              f"({fr0}, {fr1})")

    # 4. fail-fast 検証: ボーンのワールド位置を平滑化済みNPZと直接照合する
    #    （フレーム対応 Blender frame f ↔ NPZ index f-1 の最終保証。C3D の mm float32
    #    丸め誤差は 1e-7 m 級のため、許容 1e-5 m で off-by-one・スケールずれを確実に検出）
    import numpy as np
    d = np.load(str(EXP_DIR / "smoothed_world300.npz"), allow_pickle=True)
    x3d = np.asarray(d["x3d_world"], dtype=np.float64)
    names = [str(n) for n in np.asarray(d["joint_names"]).reshape(-1).tolist()]
    for frame in (1, 150, 300):
        scene.frame_set(frame)
        deps = bpy.context.evaluated_depsgraph_get()
        arm_eval = arm.evaluated_get(deps)
        for bone in ("LEye", "REye", "Nose"):
            v = arm_eval.matrix_world @ arm_eval.pose.bones[bone].head
            expected = x3d[frame - 1, names.index(bone)]
            diff = max(abs(v.x - expected[0]), abs(v.y - expected[1]),
                       abs(v.z - expected[2]))
            if diff > 1e-5:
                _fail(f"エラー: フレーム対応検証失敗: frame={frame} bone={bone} "
                      f"blender={(v.x, v.y, v.z)} npz={tuple(expected)} diff={diff:.3e}")
    print("フレーム対応検証 OK（frame 1/150/300 × LEye/REye/Nose、許容 1e-5 m）")

    # 5. カメラリグを append（依存で旧アーマチュア等が混入した場合は削除する）
    before_append = set(bpy.data.objects)
    result = bpy.ops.wm.append(
        directory=str(RIG_BLEND_PATH) + "/Object/",
        files=[{"name": name} for name in RIG_OBJECTS],
    )
    if result != {"FINISHED"}:
        _fail(f"エラー: リグの append に失敗しました: {result}")
    appended = [o for o in bpy.data.objects if o not in before_append]
    extras = [o for o in appended if o.name not in RIG_OBJECTS]
    for o in extras:
        print(f"append の依存で混入したオブジェクトを削除: {o.name}")
        bpy.data.objects.remove(o, do_unlink=True)
    missing = [n for n in RIG_OBJECTS if n not in bpy.context.scene.objects]
    if missing:
        _fail(f"エラー: append 後にリグがシーンにありません: {missing}")

    # 6. Cam_Anchor の ARMATURE 制約ターゲットを新アーマチュアで決定的に再構築する
    anchor = bpy.data.objects[ANCHOR_NAME]
    arm_constraints = [c for c in anchor.constraints if c.type == "ARMATURE"]
    if len(arm_constraints) != 1:
        _fail(f"エラー: Cam_Anchor の ARMATURE 制約が1個ではありません: {len(arm_constraints)}")
    con = arm_constraints[0]
    while con.targets:
        con.targets.remove(con.targets[0])
    for bone in ("LEye", "REye"):
        t = con.targets.new()
        t.target = arm
        t.subtarget = bone
        t.weight = 1.0
    print(f"ARMATURE 制約を再構築: target={arm.name}, bones=LEye/REye, weight=1.0/1.0")

    # 7. フレーム範囲を設定し、旧スクリプトの関数をそのまま呼ぶ
    scene.frame_start = FRAME_START
    scene.frame_end = FRAME_END

    sys.path.insert(0, str(REPO_ROOT / "phase4"))
    import fps_camera_pose

    num_frames = fps_camera_pose.export_camera_poses(
        CAMERA_NAME, arm.name, ANCHOR_NAME, str(OUTPUT_PATH)
    )
    print(f"旧経路ポーズ書き出し完了: {OUTPUT_PATH} ({num_frames} フレーム, "
          f"scene frame {FRAME_START}〜{FRAME_END})")

    # 8. 検証用にシーンを保存する（フェーズ1後の Blender 操作確認用。判定には使用しない。
    #    注意: カメラの向きは書き出し中の一時代入のため保存ファイルでは静的（最終フレームの
    #    向きのまま）。位置追従・リグ構成・キーフレーム 1〜300 は GUI で確認できる）
    blend_out = EXP_DIR / "a2_scene.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_out))
    print(f"検証用シーン保存: {blend_out}")


if __name__ == "__main__":
    main()

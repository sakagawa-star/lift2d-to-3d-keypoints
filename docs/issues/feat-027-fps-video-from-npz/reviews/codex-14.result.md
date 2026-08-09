<!-- メタ: 2026-08-08 / 対象: experiments/a1_blender_equivalence/criteria.md 第2版 + run_old_pipeline.py / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / criteria第2版レビュー -->

**高**
1. [criteria.md:25](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/criteria.md:25) が必須実行手順として `run_old_pipeline.py` を参照していますが、現時点の実験フォルダにはそのファイルが存在しません。append 方式の妥当性は「どのデータブロックを append し、scene/frame/output をどう固定し、旧 `fps_camera_pose` のどの関数を呼ぶか」に依存するため、criteria lock 前にスクリプト本体が固定されていないのは実行不能かつ事後変更の余地があります。  
   修正提案: `run_old_pipeline.py` を実験フォルダに作成し、criteria lock 対象に含めてください。最低限、固定 `.blend` パス、append 対象 `E00000` / `Cam_Anchor` / `Cam_FPS`、`scene.frame_start=41` / `frame_end=340`、`export_camera_poses(...)` 呼び出し、出力先、失敗時 exit 1 をコードで固定してください。

**中**
なし。

**低**
なし。

前回の「コマンドが完全でない / PLY 未固定」の指摘は解消されています。第2版の比較ウィンドウと閾値も、FR-012 の数値基準を弱めてはいません。append 方式自体は、旧 `fps_camera_pose.export_camera_poses` をそのまま呼ぶなら妥当ですが、その保証は `run_old_pipeline.py` を固定して初めて成立します。

---

**Claude Code の対応方針**（criteria第2版レビュー、指摘: 高1・中0・低0（append方式の妥当性・閾値不変は確認済み））: run_old_pipeline.py を実験フォルダに作成してコードで固定し、criteria の lock 対象に明記

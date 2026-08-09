<!-- メタ: 2026-08-08 / 対象: experiments/a1_blender_equivalence/criteria.md 第2版 + run_old_pipeline.py / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / criteria第2版 再レビュー -->

**高**
なし。

**中**
なし。

**低**
なし。

前回指摘は解消されています。[criteria.md:25](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/criteria.md:25) で `run_old_pipeline.py` が lock 対象として明記され、実ファイルも追加されています。スクリプト側でも固定パス・append 対象3オブジェクト・`FRAME_START=41` / `FRAME_END=340`・旧 `fps_camera_pose.export_camera_poses(...)` 呼び出しが [run_old_pipeline.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/run_old_pipeline.py:17) 以降で固定されています。

append 方式は、旧スクリプトのポーズ計算・リグ検証関数をそのまま呼ぶ構造なので、等価性検証の妥当性を損なう致命的な問題は見つかりませんでした。`py_compile` も通っています。

---

**Claude Code の対応方針**（criteria第2版 再レビュー、指摘: 高0・中0・低0）: 収束。第2版で lock し実験再開（フェーズ1'）

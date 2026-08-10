<!-- メタ: 2026-08-10 / 対象: experiments/a2_rebuilt_equivalence（criteria.md + run_old_pipeline_a2.py + compare_poses_a2.py） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 実験a2 criteria 再レビュー -->

**高**
なし。

**中**
なし。

**低**
なし。

前回指摘した C3D import の時刻対応問題は解消されています。[run_old_pipeline_a2.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:47) で fps を固定し、[adapt_frame_rate=False](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:55) を明示、さらに [キー範囲 1〜300 の正規化検証](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:85) と [NPZ 直接照合](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:97) が入っています。criteria 側も [P1b](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/criteria.md:36) として lock 内容に反映済みです。

確認として、lock 対象スクリプトの `py_compile` は通過し、Blender 4.5.5 の `bpy.ops.import_anim.c3d` に `adapt_frame_rate` プロパティが存在することも確認しました。実験フェーズ本体は実行していません。

---

**Claude Code の対応方針**（実験a2 criteria 再レビュー、指摘: 高0・中0・低0（py_compile 通過と 4.5.5 の adapt_frame_rate 存在も確認済み））: 収束。criteria lock。実験の実行はユーザーの実行前確認を待つ

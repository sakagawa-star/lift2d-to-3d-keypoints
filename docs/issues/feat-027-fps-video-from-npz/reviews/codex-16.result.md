<!-- メタ: 2026-08-08 / 対象: experiments/a1_blender_equivalence criteria第3版（Blender 4.5.5 直接フルロード） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / criteria第3版レビュー -->

**高**
1. 現行 criteria 第3版では「4.0.2 で生成した `old_poses.json` は無効、4.5.5 で再生成」としていますが、[experiment_log.md:75](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/experiment_log.md:75) 以降に、第2版の append 方式 `フェーズ1'` を前提にした `フェーズ3` の直前予測が現行セクションとして残っています。さらに [criteria.md:61](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/criteria.md:61) もまだ「append 方式」と書いており、第3版の直接フルロード方式と矛盾します。このままだと、無効化した旧 `old_poses.json` を使った照合に進めてしまう余地があります。  
   修正提案: 第2版由来のフェーズ3予測を「無効・破棄」と明示するか削除し、第3版用に `フェーズ1''`（4.5.5 直接フルロード）の直前予測・実測を新設してください。その後、4.5.5 で再生成した `old_poses.json` を前提にした新しいフェーズ3直前予測を記録してください。criteria §5 のフェーズ1表記も「直接フルロード方式」に直してください。

**中**
なし。

**低**
なし。

前回の `run_old_pipeline.py` 不在問題は解消されています。スクリプト自体は 4.5.5 フルロード、frame 41〜340 固定、旧 `export_camera_poses` 呼び出しに更新されており、静的には妥当です。問題はログと criteria の一部が第2版の append 実行状態をまだ現行扱いしている点です。

---

**Claude Code の対応方針**（criteria第3版レビュー、指摘: 高1・中0・低0）: §5 表記修正、第2版前提のログ記録を無効化・破棄注記、フェーズ1''/3セクション新設、無効 old_poses.json を削除

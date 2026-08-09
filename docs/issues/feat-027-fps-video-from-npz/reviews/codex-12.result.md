<!-- メタ: 2026-08-08 / 対象: experiments/a1_blender_equivalence/criteria.md / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 初回（実験a1 criteria lock用） -->

**高**
1. [criteria.md:20](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/criteria.md:20) の新経路コマンドが、そのまま実行できる形になっていません。文中では「phase4 ディレクトリで実行」としている一方、コマンドは `uv run --project phase4 python render_fps_video.py ...` になっており、`phase4/phase4` を project として解釈する形になります。また `<実在PLY>` が未固定です。`--dump-poses` でも PLY パスは存在検査対象なので、criteria lock 文書として実行者の選択余地が残ります。  
   修正提案: 実行ディレクトリを repo root か `phase4/` のどちらかに固定し、それに合わせて完全なコマンドにしてください。例: `phase4/` で実行するなら `uv run python render_fps_video.py data/project.ply data/session001_f145749_world300.npz ...` のように、PLY も実在パスで固定する。

**中**
なし。

**低**
なし。

数値判定そのものは requirements/design と一致しています。位置差 `max < 1mm`、回転角度差 `max < 0.1°`、300フレーム全件合格、1件でも超過なら不合格、という基準は事後解釈の余地なく定義されています。フレーム対応も `old frame f ↔ new frames[f-1]` と明記されています。

---

**Claude Code の対応方針**（初回（実験a1 criteria lock用）、指摘: 高1・中0・低0（数値判定・フレーム対応・事後解釈排除は問題なし））: §2 のコマンドをリポジトリルート実行の完全形に固定し、PLY を phase4/data/project.ply に固定

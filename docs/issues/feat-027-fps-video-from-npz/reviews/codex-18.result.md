<!-- メタ: 2026-08-10 / 対象: experiments/a2_rebuilt_equivalence（criteria.md + run_old_pipeline_a2.py + compare_poses_a2.py） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 実験a2 criteria 初回レビュー -->

**高**

1. 旧経路の C3D インポートが criteria のフレーム対応と一致していません。  
   [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/criteria.md:30) は旧 JSON の `frame` 1〜300 を新 JSON の `frames[f-1]` に対応させていますが、[run_old_pipeline_a2.py](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:44) は `bpy.ops.import_anim.c3d(filepath=...)` をデフォルト設定で呼んでいます。`io_anim_c3d` はデフォルトで `adapt_frame_rate=True` で、キー時刻を `scene.render.fps / C3D fps` でスケールします。こちらで確認した Blender 4.5.5 の `-b` 起動直後 fps は 24 なので、30fps C3D は 0, 0.8, 1.6... フレームに配置されます。さらに importer 実装上、先頭サンプルは Blender frame 1 ではなく frame 0 に入ります。  
   このままだと a2 は「新旧ポーズ差」を測っているように見えて、実際には旧経路側の時刻対応ずれを混ぜた比較になります。判定が偽陰性になり得るため、criteria lock 前に直すべきです。

   修正提案: `run_old_pipeline_a2.py` で C3D import 前に import 条件を明示し、criteria §2/§3 にも固定してください。現在の 1〜300 対応を維持するなら、少なくとも `adapt_frame_rate=False` を明示し、import 後に生成 action の全 keyframe x 座標を `+1` して「Blender frame 1 ↔ NPZ index 0」に揃える必要があります。あわせて `scene.render.fps = 30` も明示し、import 後に LEye/REye など数フレームを `smoothed_world300.npz` と照合する fail-fast 検証を入れると、off-by-one と fps スケールの再発を防げます。

**中**

なし。

**低**

なし。

---

**Claude Code の対応方針**（実験a2 criteria 初回レビュー、指摘: 高1・中0・低0）: io_anim_c3d のレート適応と先頭キー frame 0 の時刻対応ずれ対策: ラッパーに fps=30 明示・adapt_frame_rate=False・キー正規化と範囲検証・NPZ直接照合の fail-fast を追加し、criteria §2/§3 に lock 内容として反映

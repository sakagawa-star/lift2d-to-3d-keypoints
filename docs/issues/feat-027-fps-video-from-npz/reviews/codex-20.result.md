<!-- メタ: 2026-08-10 / 対象: experiments/a2_rebuilt_equivalence 第2版（a2_scene.blend 保存の追加） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 差分再レビュー -->

**高**
なし。

**中**
なし。

**低**
なし。

今回の差分は、[エクスポート成功後の `a2_scene.blend` 保存](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/run_old_pipeline_a2.py:156)として判定経路の後段に追加されており、`old_poses.json` 生成・フレーム対応検証・照合条件には影響していません。criteria でも [判定には使用しない成果物](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence/criteria.md:75)として明記されています。

前回の C3D フレーム対応修正も維持されています。lock 対象スクリプトの構文チェックは通過しました。

---

**Claude Code の対応方針**（差分再レビュー、指摘: 高0・中0・低0）: 収束。第2版で lock し、ユーザー承認済みのフェーズ0〜3 実行へ

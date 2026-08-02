**高**
- 試行回数管理にまだ穴があります。`phase2` が OOM で中断した場合、ログは `_flush()` されますが、`phase2_done` は成功後にしか立たないため、その後 `--phase 2` を `--rerun-defect` なしで再実行できます。partial な phase2 結果を見た後に再試行できるので、criteria §7 の「各フェーズ1回」に対してまだ閉じていません。  
  根拠: [criteria.md:106](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:106), [m3b_run.py:666](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/m3b_run.py:666), [m3b_run.py:734](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/m3b_run.py:734), [m3b_run.py:741](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/m3b_run.py:741), [m3b_run.py:748](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/m3b_run.py:748)。  
  修正提案: `phase2_attempted=True` を `phase2()` 呼び出し前に保存し、OOM 中断時も `phase2_aborted_oom=True` を state に保存してから終了する。`_guard_phase` は `*_done` ではなく `*_attempted` を見る形にする。できれば phase0/1 も同じ attempted/done/outcome 方式に統一する。

**中/低**
- 致命的な追加指摘なし。

前回指摘①の OOM 発生元分離、fallback 時の `check_meta`、phase2 OOM の捕捉・報告は概ね解消されています。`python3 -m py_compile` と各 uv 環境での `py_compile` も通りました。
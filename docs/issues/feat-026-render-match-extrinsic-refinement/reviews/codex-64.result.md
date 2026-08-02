**高**
- フェーズ3条件が `phase2_attempted and not phase2_go` で判定されており、criteria の「フェーズ2: LoFTR 未達」と同義になっていません。`phase2` が OOM で中断した場合も `phase2_attempted=True` かつ `phase2_go=False` になるため、後から `--phase 3` を実行すると MASt3R E2E に進めます。これは「LoFTR が M4 判定を完了して未達だった場合のみ phase3」という遷移表から外れ、OOM/未完了を No-Go 相当として扱ってしまいます。さらに `--phase 3` を phase2 未実行の状態で呼ぶと、LoFTR 採用済みでも `MASt3R 不適格` として No-Go を記録できる経路があります。  
  根拠: [criteria_m4_2.md:56](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria_m4_2.md:56), [m4_2_run.py:602](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_2_run.py:602), [m4_2_run.py:617](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_2_run.py:617)。  
  修正提案: `phase2_completed=True` を `run_e2e()` が正常に戻った後だけ保存し、phase3 条件は `(LoFTR 不適格 and mast3r_ok) or (phase2_completed and not phase2_go and mast3r_ok)` にする。`phase2_aborted_oom` や phase2 未実行時は phase3 へ進めず、明示的に中断または順序違反でエラーにする。

**中/低**
- 致命的な追加指摘なし。

補足: `py_compile` は `python3` と phase4 環境の両方で通りました。M4 の Stage 4/5/6、seed 系列、§6.2 受理、§6.4 前提条件の通常経路は、見た範囲では M4 本実験と整合しています。
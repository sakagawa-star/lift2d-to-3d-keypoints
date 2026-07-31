**高:** なし。

**中:** 補助可視化が失敗すると、仲裁結果の保存まで失敗します。  
判定自体は [d3_arbitrate.py:75](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py:75) から [d3_arbitrate.py:99](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py:99) で完了していますが、その後に偵察可視化として `load_ply` と `iterate_classify` を実行し、最後の [d3_arbitrate.py:133](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py:133) で初めて `d3_result.txt` を書いています。  
つまり、補助の PNG 生成が GPU/PLY/レンダ/RANSAC で失敗すると、D3 の本体である手動点仲裁結果も保存されません。可視化が判定値に混入してはいませんが、実行完了条件には混入しています。

修正提案: 判定結果を先に保存するか、可視化部分を `try/except` で囲んで、失敗時は「偵察可視化は未生成、判定には影響なし」と記録して続行してください。

**低:** なし。

A/B 定義、§5.4 と同一の再投影誤差、名簿中央値差 `>=1.0px`、差 `<1.0px` の保留は正しく実装されています。構文チェックも通っています。
---

## Claude Code の対応方針（2026-07-31）

- 対象: d3_arbitrate.py 実行前レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 中1（可視化の失敗が仲裁結果の保存を巻き込む）: 仲裁本体の結果を可視化の前に保存し、可視化を try/except で分離。失敗時は「偵察可視化は未生成、判定には影響なし」と記録して続行

**中**

[m4_1_sweep.py:226](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/m4_1_sweep.py:226>)  
フェーズ1の全12組について、criteria §4 は「各組・各チェーンの N と除外内訳（真NaN/広がり分離込み）を全記録」としていますが、現在の出力は各組の `N` だけです。除外内訳は選定された1組だけしか結果 txt に残りません。選定規則自体は `N≥30 → 変更度最小 → spread小` で正しく実装されていますが、失敗モードの実測記録が criteria と一致していません。

修正提案: フェーズ1のループ内で、各 `spread × var × chain` について少なくとも `n_disp`, `N`, `excl_depth_nan`, `excl_true_nan`, `excl_spread`, `excl_alpha`, `excl_z`, `excl_var` を `m4_1_sweep_result.txt` か別の `m4_1_sweep_table.csv` に全件出力してください。

**高**

該当なし。

**低**

該当なし。

確認済みの点:

- Stage 4 の `keep` 条件は `m4_run.run_iteration_real_m4` と同じで、差分は `rel_spread_th` / `GATE_VAR_REL` のパラメータ化に留まっています。
- フェーズ0は criteria の期待値表と `n_raw`, `matches`, `n_disp`, 各除外数, `N` を照合しています。
- 真NaN/広がり分離は `SPREAD_TH_OFF=1e9` により診断専用で実装され、`keep` 計算には影響していません。この深度生成では無効深度が NaN なので方式として妥当です。
- 構文チェックと import は通っています。
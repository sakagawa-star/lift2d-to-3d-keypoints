**高**

[criteria_m4_1.md:91](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria_m4_1.md:91>)  
品質非劣化の RMSE 判定母集団が閉じていません。「受理クラスタ側チェーンの中央値」は、pooled 判定ではクラスタとチェーンが別概念なので、実験後に「選ばれたクラスタのサンプル」「そのクラスタを多く含むチェーン」「全チェーン」など解釈を動かせます。これは §5.3-1 が Go/No-Go 条件なので致命的です。

修正提案: 判定式を固定してください。例: 「§5.2 で最終採用されたクラスタに属する k≥1 サンプルの RMSE 中央値 ≤ 2.0px。二峰仲裁時は仲裁で選ばれたクラスタのみ。単峰時は主クラスタのみ。クラスタ外サンプルはこの中央値には含めないが、別途記録する」のように、対象サンプルを一意にしてください。

**中**

[criteria_m4_1.md:52](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria_m4_1.md:52>)  
フェーズ0の再現検証が「m4_result.txt の各チェーンの N・除外内訳と完全一致」とされていますが、`m4_result.txt` には各チェーンの N はある一方、除外内訳は範囲・中央値のみで、各チェーンの完全一致対象が文書上固定されていません。フェーズ1のスイープ基盤が M4 と同一であることを保証するチェックなので、ここは曖昧にしない方がよいです。

修正提案: フェーズ0の一致対象を明記してください。`m4_history.npz` の具体キーを正とするか、criteria にチェーン A/B/C ごとの `n_disp`, `N`, `excl_nan_or_spread`, `excl_var`, `excl_alpha`, `excl_z` の期待値表を置き、「完全一致」の比較先を固定してください。

**低**

該当なし。

グリッド、選定規則、試行回数管理、M4 baseline No-Go との分離は概ね閉じています。上の2点を潰せば criteria lock 可能な水準です。
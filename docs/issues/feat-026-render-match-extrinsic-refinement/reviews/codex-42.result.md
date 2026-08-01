**Findings**

- **高**: [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:49) の `z∈(0.5,10)m` は「Stage 0 で干渉が観測されたら再lock」とされており、対象データを見た後に深度ゲートを変更できる抜け穴です。Stage 0 も [同ファイル:118](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:118) で実データを使うため、実験前固定の条件になっていません。  
  **修正提案**: M4 本判定では `z∈(0.5,10)m` を完全固定し、干渉しても合否救済に使わない。変更が必要なら M4-0 など別の事前実験として criteria lock し直し、変更後の結果を現行 M4 判定に混ぜない。

- **高**: 二峰時の受理条件が M3-2 から逸脱し、かつ数値的に閉じていません。[criteria.md:58](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:58) では `f_c ≥ 0.7` を受理条件とする一方、[criteria.md:77](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:77)〜[82](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:82) では `第2クラスタ ≥ 0.2` かつ手動点仲裁成立なら受理できます。M3-2 の lock 済み基準では `f_c < 0.7` は不受理で、二峰は分類対象です。さらに「二峰」と「多峰」の境界、仲裁対象にする峰、第三クラスタの扱いが固定されていません。  
  **修正提案**: 保守的には `f_c < 0.7` は M4 baseline No-Go とし、仲裁は別記録に落とす。仲裁で Go を許すなら、少なくとも「上位2クラスタのみを対象」「第3クラスタ占有率 < 0.2」「上位2クラスタ合計占有率の下限」などを実験前に数値固定する。

- **中**: [criteria.md:95](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:95)〜[98](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:98) の「M1 × M2/M3 × 受理条件の突き合わせ」が記録だけで、[criteria.md:111](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:111) の Go 条件に入っていません。roadmap M4 はこの突き合わせで「6点初期値+自動リファイン」の成立を判定するとしているため、現在の基準では初期値が既実証の収束域から外れても Go にできます。  
  **修正提案**: `6点PnP初期解→最終解` の差について、少なくとも M2 実測収束域の `位置 ≤ 50cm・角度 ≤ 10°` を M4 Go の前提条件に入れる。入れないなら、この M4 は roadmap の「総合判定」ではなく「診断値ベースの単発受理」に限定すると明記する。

低重要度で報告すべき致命点はありません。現状のまま lock するなら、特に上2件は実験後に合否を動かせる余地が残ります。
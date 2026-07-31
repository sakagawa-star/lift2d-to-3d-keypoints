レビュー結果です。致命的なものだけ挙げます。低重要度の指摘はありません。

**高**

- [criteria.md §5.2](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:68) の「P0・P1・P2 のうち成立した系列」で同一解判定する条件は危険です。P1/P2 が失敗しても、P0 だけで同一解が真になる余地があり、摂動検算が検算として機能しません。  
  修正提案: `P0/P1/P2 がすべて成立し、3系列すべての最大ペア間差が ≤1cm / ≤0.1°` を同一解条件にする。1つでも失敗したら §5.2 は不合格、P3/P4 は従来どおり参考扱い。

- [criteria.md §5.3](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:74) の一致性閾値は M1 n0=10 p95 から正しく転記されていますが、M1 は [intrinsics_all.toml 使用](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m1/report.md:10)で、M3 は [Config_scene.toml 使用](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:24)です。実ファイル上も焦点距離・主点が数十 px 規模で違います。例: cam41520556 は Config 側 f≈1216/1214, cx≈963 に対し、intrinsics_all 側 f≈1151/1146, cx≈936 です。これだと「同じ手動データの自己ばらつき」の物差しとして同一条件の実測値になっていません。  
  修正提案: M3 で使う Config_scene の K・歪みで M1 相当の再サンプリングを再実行し、その p95 を criteria に固定する。再実行しないなら、§5.3 は合否判定に使わず参考値に落とす。

- [criteria.md §5.6](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:106) と [§6](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:117) の関係が未固定です。SIFT 不成立後に LoFTR/MASt3R で成功した場合、それを M3 Go に数えるのか、SIFT baseline は No-Go として別実験扱いにするのかが読めません。これは実験後に合否を動かせます。  
  修正提案: `M3-SIFT baseline の Go/No-Go は SIFT 結果のみで判定する。matcher 切替は No-Go 後の別ブランチで、M3b criteria lock を作ってから評価する` と明記する。もし同じ M3 で数えるなら、matcher 名・版・入力・閾値・2/3 集計への含め方を事前固定する。

**中**

- [criteria.md §5.4](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria.md:89) は実品質を測れるとは主張していない点は良いです。ただし「独立検証データを確保できる」という表現は強すぎます。roadmap は Config_scene 生成に使っていない点、または追加手動点を独立候補にしていますが、criteria は M1 と同一の手動点を使い、しかも PLY/基準ポーズが同じ手動データ由来だと自認しています。  
  修正提案: Config_scene 生成に未使用の点 ID を明示できるなら、その点 ID と基準ポーズ再投影中央値を実験前に数値固定する。未使用性を証明できないなら、名称を「独立検証」ではなく「手動点再投影整合」に落とし、Go 条件に入れるかどうかを再判断する。

総評: 「実品質を測れる」という過大主張は概ね避けられています。一方で、同一解判定の抜け穴、intrinsics 不一致のままの M1 閾値流用、matcher 切替後の合否扱い未固定は、criteria lock としては実験前に潰すべきです。
---

## Claude Code の対応方針（2026-07-31）

- 対象: experiments/m3_realquery/criteria.md 初回レビュー / 初回
- 高1（同一解判定の抜け穴）: 「P0・P1・P2 の3系列すべて成立」を必須条件に修正。1系列でも失敗なら §5.2 不合格
- 高2（intrinsics 不一致のまま M1 閾値流用）: §5.3 の閾値を「Config_scene の K・歪みで n0=10 再サンプリングを再計測（M3-0a、seed=42 の決定的計算）した p95 で確定し再lock」方式に変更。M1 公表値は参考値に格下げ。手順 §7 に M3-0a を追加
- 高3（マッチャー切替後の合否扱い未固定）: M3 の Go/No-Go は SIFT baseline のみで判定と明記。切替は別ブランチ M3b とし、実施前に M3b criteria lock（マッチャー名・版・入力・閾値・集計を固定）を作成
- 中1（「独立検証データ確保」が過大）: §5.4 を「手動点再投影整合」に改名し Go 条件から除外。独立検証データは「確保できない」と確定（未使用点の証明不能・PLY 座標系も同一データ由来）。roadmap の分岐に従い M4/M7 は「診断値ベースの運用受理」に限定。検証専用の追加手動点による独立化はユーザー判断事項として残した

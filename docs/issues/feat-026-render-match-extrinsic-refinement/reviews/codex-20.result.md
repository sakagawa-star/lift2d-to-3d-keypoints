レビュー基準に沿って、致命度のある点だけに絞ります。低重要度の指摘はありません。

**高**
- M3 が「実品質との相関」を検証できる設計になっていない  
  [roadmap.md:40](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:40) で oracle なしの診断値を品質推定の代替として検証するとしていますが、[roadmap.md:44](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:44) の物差しは「20点方式の推定ポーズとの差・ホールドアウト再投影・目視」です。これは真値ではなく、既存手法への一致や自己整合性しか測れません。さらに M3 条件の入力には PLY・推定ポーズ・実写画像しかなく、ホールドアウト 2D/3D 点の I/O も定義されていません [roadmap.md:41](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:41)。  
  このままだと M3 で較正した受理条件を M4 に持ち込んでも、「正しいか」ではなく「既存推定や見た目に近いか」しか保証できません。  
  修正提案: M3 の目的を「実品質相関」から「既存20点基準との一致・自己整合性確認」に弱めるか、独立した検証データを追加してください。例: Config_scene 生成に使っていないホールドアウト対応点、別手段で得た高品質基準ポーズ、既知マーカー、または検証専用の追加手動点。独立基準がない場合、M4/M7 で「精度を評価できる」と書かず、「診断値ベースの運用受理に限る」と明記すべきです。

- M3 の Go/No-Go 条件が数値で固定されておらず、ロードマップ自身の原則に反している  
  全マイルストーンで「実行前に数値で定義する判定基準」を持つとしていますが [roadmap.md:14](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:14)、次の M3 は「数値は実験直前に確定」としており [roadmap.md:42](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:42)、実際に固定されているのは N ≥ 30 程度です [roadmap.md:43](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:43)。  
  特に「反復収束」「同一解収束」「20点方式との差」「ホールドアウト再投影」「レンダ重畳の目視」「SIFT 不成立」の閾値が未定義です。これでは実験後に都合よく合否を動かせます。  
  修正提案: M3 実行前に `M3 criteria lock` を追加し、少なくとも `N_in`、RMSE、退化診断閾値、収束閾値、摂動検算の摂動セットと同一解判定、ホールドアウト誤差の式と閾値、SIFT 不成立時の切替条件、記録先を固定してください。

**中**
- M4 の総合判定で、評価対象のパイプライン variant と対象環境での検証範囲が曖昧  
  M2 では Stage 1.5 粗整列ありで収束域が拡張した結果を含めています [roadmap.md:33](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:33)。一方、Stage 1.5 の正式化判断は M5 とされています [roadmap.md:57](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:57)。その前段の M4 で「6点初期値+自動リファイン」が成立するか総合判定するため [roadmap.md:52](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/roadmap.md:52)、Stage 1.5 を使うのか使わないのかが判定結果を左右します。  
  修正提案: M4 の実験条件に「baseline Stage 2〜7」「Stage 1.5 有効」のどちらを正式な Go/No-Go 判定対象にするかを明記してください。比較するなら両 variant を別々に記録し、M5 で Stage 1.5 を正式採用する条件も事前に定義してください。対象環境についても、1カメラだけで全体成立を結論するのか、限定パイロットとするのかを明確化する必要があります。
---

## Claude Code の対応方針（2026-07-31）

- 対象: roadmap.md v2 初回レビュー / session id: 019fb6b8-2e60-7230-a1f3-e1ea8b55656e / 初回
- 高1（M3 が実品質相関を検証できない）: M3 に「測れるものの限界」を明示し、問い(3)を「基準ポーズとの一致・自己整合性の代替指標」に弱めた。独立検証データ（ホールドアウト対応点・検証専用手動点）の確保可否を criteria lock 時に確定し、確保できない場合は M4/M7 の結論を「診断値ベースの運用受理に限る」と明記
- 高2（Go/No-Go 条件が数値で未固定）: M3-0「criteria lock」ステップを新設。criteria.md に固定する項目（N_in・Δτ・RMSE・退化診断・同一解判定・ホールドアウト誤差式・マッチャー切替条件・記録先）を列挙し、lock 後のなし崩し変更を禁止
- 中1（M4 の variant・範囲が曖昧）: 正式判定対象を baseline（Stage 1.5 なし）に固定、Stage 1.5 はレスキューとして別記録。正式採用条件を事前定義。1カメラの限定パイロットである旨を明示し、総合判定の主語を「パイロット範囲」に修正

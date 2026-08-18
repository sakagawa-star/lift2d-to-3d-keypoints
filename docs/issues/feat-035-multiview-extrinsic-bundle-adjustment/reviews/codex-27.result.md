再レビューしました。前回5件のうち、高2（Sampson式）、中1（EPI_TOL_PXのStep7追加）、中3の主要箇所（FR-005/design §2.4/手動例外）は解消されています。高1と中2は一部に矛盾が残っています。

**高**

1. Stage F の連結性エラー条件が design 内で矛盾している  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:148) では「全ポーズ確定カメラが単一連結成分に入らない場合は終了コード1」と明記されていますが、直後の境界条件 [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:151) が「エラーは全ペア不採用時のみ」となっており、連結性不成立をエラーにする修正と矛盾しています。

修正提案: line151 を「エラーは全ペア不採用時、またはフィルタ後採用ペアグラフで全ポーズ確定カメラが単一連結成分に入らない時」と書き換える。

**中**

1. `--init-cameras` 省略制限が §2.0 とテスト設計に反映されていない  
CLI詳細では省略を「全対象カメラ分のみを含む受理済みTOML」に限定していますが、共通マージ規則 [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:60) とテスト設計 [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:241) はまだ「省略時は init-toml 全カメラを信頼」の旧仕様に見えます。前回の中2は部分解消です。

修正提案: §2.0 とテスト設計にも、inuyama 主経路では明示必須、省略許可は「全対象カメラ分のみを含む受理済みTOML」に限る、と同じ条件を入れる。

2. Step 3F criteria の説明が測定のみで、Go/No-Go 条件を含んでいない  
[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:65) は `p0_epi_filter/criteria.md` を「測定ゴール」とだけ説明していますが、同Stepの完了条件 [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:67) は連結性のGo条件です。criteria にこのGo条件が明記されないと、CLAUDE.md の事前判定基準プロトコル上弱いです。

修正提案: line65 に「測定項目に加え、Go条件としてフィルタ後採用ペアグラフの全ポーズ確定カメラ単一連結を事前定義し、直前予測枠にも採用ペア数・連結成否を含める」と明記する。

3. ADRに Stage R 残骸が1箇所残っている  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:268) の「アンカー不足カメラ」がまだ「Stage R 後ポーズを最終値に」となっています。本文は信頼初期ポーズへ修正済みなので、ADRだけ不整合です。

修正提案: 「信頼初期ポーズ（Step 3M の feat-026 受理済みポーズ）を最終値に」へ置換する。

**低**

なし。
再レビューしました。前回の「事後解釈になる」指摘は、Step 3Rを測定-only→閾値確定→criteria再lock→固定閾値で再評価、に分けたことで解消しています。

**高**
- 状態分離は requirements 側では入っていますが、design 側の後段データフローがまだ `ポーズ確定` に統一されていません。[requirements.md:29](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:29)-[requirements.md:31](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:31) では `ポーズ確定 = 信頼初期 or 合意受理かつ品質受理` ですが、[design.md:139](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:139)、[design.md:149](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:149)、[design.md:173](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:173) はまだ `較正済みカメラ` / `Stage R 受理カメラ` 基準に読めます。これだと品質未受理の合意受理カメラが Stage A/BA/TOML に流れる余地が残ります。
  - 修正提案: design §2.4/§2.5/§2.6 をすべて `ポーズ確定カメラ` 基準に書き換える。特に `Stage R 受理済み` は `Stage R 合意受理かつ品質受理済み` に置換し、品質未受理カメラをアンカー取得・BA・評価・TOMLから除外すると明記する。

- `品質受理` が「本スクリプト外」とされている一方で、その結果を本体へ渡すI/Oが設計されていません。[design.md:129](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:129) は品質受理をStep 3R実験手順側に置いていますが、[design.md:177](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:177)-[design.md:185](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:185) のCLIにも [design.md:211](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:211)-[design.md:213](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:213) の `refine_cameras` 戻り値にも、品質受理済みカメラを渡す経路がありません。実装者が `ポーズ確定` を判定できません。
  - 修正提案: どちらかに固定する。スクリプト内で判定するなら、確定後の閾値を定数/CLIに持たせ、`refine_cameras` が `consensus_ok`・`misalignment_median_px`・`quality_ok`・`pose_confirmed` を返す設計にする。外部判定のままなら、`--pose-confirmed-cameras` や品質判定JSONなどの入力を追加し、未指定では Stage A 以降へ進めないと定義する。

**中**
- なし。

**低**
- なし。
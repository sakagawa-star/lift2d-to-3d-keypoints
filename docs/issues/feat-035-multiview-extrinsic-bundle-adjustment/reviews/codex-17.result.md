レビューしました。前回の `--init-cameras` 周りの2件は解消されたままですが、今回の Stage R 追加には致命的な不整合が残っています。

**高**

1. **Stage R で保証されたポーズだけを最終出力する経路になっていない**

FR-009 は Stage R 受理カメラに `≤5px` を課していますが、古い「ブートストラップポーズを最終値にする」記述が残っています。特に [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:143) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:173)、[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:93) は、アンカー不足またはブートストラップ成功カメラを Stage R 後ではなく粗い Stage B ポーズのまま最終出力できる読み方です。さらに [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:48) も Step 3 から Step 4 へ進むと書いており、Step 3R を飛ばす経路が残っています。

修正提案: 「ポーズ確定カメラ」を `--init-cameras` 信頼カメラまたは FR-009 受理済みカメラに限定してください。Stage R 不受理カメラは TOML 出力・BA・評価から除外するか、出力するなら feat-026 品質保証対象外として MVP 条件から外す必要があります。roadmap Step 3 の進行先も Step 3R に直してください。

**中**

1. **FR-006 のレポート必須項目と受け入れ基準・検証がずれている**

[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:85) では (h) Stage R 診断と (i) BAフォールバック有無が必須項目に追加されていますが、受け入れ基準はまだ (a)〜(g) です（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:86)）。design/roadmap の検証も (a)〜(g) のままです（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:248)、[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:80)）。

修正提案: FR-006 受け入れ基準、design §2.6、テスト設計、roadmap Step 6/7 の確認項目をすべて (a)〜(i) に更新してください。

2. **p0_bootstrap のゲート改訂が旧実測の事後合格化に見える余地がある**

[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:3) は旧NG記録を保持するとしていますが、[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:33) で旧実測値 15〜43px を見た後に 50px ゲートへ改訂しています。このまま旧実測を新基準で合格扱いすると、CLAUDE.md の事後解釈禁止に触れます。

修正提案: 「旧実験は旧基準でNGのまま保持し、新基準は再lock後の改訂2実行から適用する」と criteria/experiment_log に明記してください。再実行または再評価する場合も、改訂2として直前予測・実測・照合を別セクションで追記してください。

**低**
なし。
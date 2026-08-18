レビューしました。`--init-cameras` 追加の方針自体は妥当ですが、同じ事故を防ぐにはまだ不十分です。

**高**

1. **design.md §2.3 に古い Stage B スキップ条件が残っている**

[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:57) では、`--init-toml` に存在しても `--init-cameras` に指定されないカメラは `None` にすると定義されています。  
一方で [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:117) はまだ「8台全て init-toml にあるなら Stage B をスキップ」と読めます。これは今回の事故と同じ条件で Stage B をスキップする実装を誘発します。

修正提案: §2.3 のデータフロー/境界条件を、「U は §2.0 のマージ後に `rvec/tvec is None` のカメラ集合」「U が空なのは対象全カメラが `--init-cameras` で信頼対象になった場合のみ」に書き換えてください。

**中**

1. **Step 3 実験手順側で `--init-cameras` 使用が固定されていない**

[p0_bootstrap/criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p0_bootstrap/criteria.md:17) は Stage B 実行とだけ書いており、今回の inuyama 再実験で `--init-cameras cam05520125 cam05520126` を必ず指定することが固定されていません。`--init-cameras` 省略時は全カメラ信頼なので、実験手順から同じ事故を再発できます。

修正提案: p0_bootstrap の criteria/experiment 手順に、使用する初期信頼カメラを明記してください。例: `--init-toml cage_refined.toml --init-cameras cam05520125 cam05520126`。必要なら roadmap Step 3 の検証手順にも同じ注意を入れてください。

**低**
なし。
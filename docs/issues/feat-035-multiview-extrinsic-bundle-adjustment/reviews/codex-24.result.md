再レビューしました。前回の2件は解消しています。`--stop-after-stage-r` 分離で測定モードとフル実行の切り替えは成立し、MVP定義も `信頼初期 or 合意受理かつ品質受理` に更新されています。

**高**
- なし。

**中**
- `--quality-accepted-cameras` 省略時に信頼初期カメラのみでフル実行できる設計になりましたが、その結果 `ポーズ確定カメラ` が1台だけになるケースの扱いが未定義です。[design.md:185](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:185) は「指定が空なら信頼初期カメラのみ」で後段へ進めます。一方 [design.md:163](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:163) は `n_cam=1` を §2.3 で事前排除済みとしていますが、品質フィルタ後に1台へ減る経路が新設されたため、この前提は崩れています。
  - 修正提案: `ポーズ確定カメラ < 2` の場合の扱いを明記する。基本は Stage A/BA に進まずエラーメッセージを出して終了コード1が妥当です。あわせて requirements FR-005、design §2.5 境界条件、CLIテストに「品質受理0台かつ信頼初期1台」を追加する。

**低**
- なし。
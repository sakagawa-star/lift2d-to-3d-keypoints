再レビューしました。前回の中2件は解消済み、高1は大半解消されていますが、古い表現がまだ残っていて完全には閉じていません。

**高**
- [requirements.md:76](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:76) と [design.md:173](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:173) に、まだ「FR-002 初期ポーズ」「bootstrap-success camera poses」をBA/TOML出力に使えるように読める記述が残っています。これは「ポーズ確定カメラ = 信頼初期カメラ or Stage R 受理カメラ」「Stage R 不受理は BA・評価・TOML 出力から除外」という今回の修正方針と矛盾します。
  - 修正提案: FR-005 の入力を「FR-009で受理された精緻化後ポーズ、または `--init-cameras` 信頼初期ポーズ」に変更する。design §2.6 のTOML出力対象も「BA後またはfallback後のポーズ確定カメラ + N_A不足でBA非参加だがStage R受理/信頼初期のカメラ」に限定し、bootstrap-only/Stage R不受理は出力不可と明記する。

**中**
- [roadmap.md:58](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:58) は Step 3R の進行条件を「受理カメラ1台以上」としていますが、[requirements.md:135](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:135) のMVPは「inuyama 8台で出力TOML」と読めます。現状だと1台だけ受理して先へ進み、最終的に8台TOMLを満たせない経路が残ります。
  - 修正提案: 8台MVPを維持するなら、Step 3RのGo条件を「未信頼6台すべてStage R受理、合計8台がポーズ確定」にする。部分TOMLを許すなら、requirements のMVP/FR-007側を「ポーズ確定カメラのみ出力」に揃え、8台出力を必須条件から外す。

**低**
- なし。

前回指摘のうち、レポート項目(a)〜(i)の統一と、旧実測NGを保持したうえで改訂2を別セクションで再実行する実験プロトコル対応は解消されています。
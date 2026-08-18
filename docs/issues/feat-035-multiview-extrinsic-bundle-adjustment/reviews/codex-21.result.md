再レビューしました。前回3件の修正は維持されていますが、今回の「初回実測後に閾値決定」で新しい致命的な不整合が入っています。

**高**
- [requirements.md:54](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:54) と [roadmap.md:58](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:58) が、Step 3R の初回実測を見た後に受け入れ閾値と進行可否を決める流れになっています。これは [CLAUDE.md:255](/home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:255) の criteria lock と [CLAUDE.md:258](/home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:258) の事後解釈禁止に抵触します。測定実験として実施することは可能ですが、その同じ実測結果を見てから合格ラインを作り、そのままGo判定に使うのは不可です。
  - 修正提案: Step 3R初回は「測定のみ、Go/No-Goなし」と明記する。実測後にユーザーが閾値を決めたら、requirements/design/roadmap と `p0_stage_r/criteria.md` を更新して再レビュー・criteria lockし、その固定基準で再実行または保存済み結果の再評価を行ってから進行判定する。

- 「受理」の意味が分裂しています。[design.md:129](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:129) では Stage R 受理が f_c/f2/pose_diff の合意判定だけで決まりますが、[requirements.md:54](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:54) ではレンダvs実写ずれの閾値が未確定です。一方で [requirements.md:75](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:75) と [requirements.md:94](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:94) は「FR-009受理」をBA/TOML出力対象にしています。このままだと、合意判定だけ通ったが最終ずれ品質は未承認のカメラがポーズ確定扱いになります。
  - 修正提案: 状態を分けて定義する。例: `Stage R合意受理` = f_c/f2/pose_diff通過、`品質受理` = 確定済みレンダvs実写閾値通過、`ポーズ確定` = 信頼初期カメラまたは両方通過。FR-005/FR-007/design §2.5/§2.6 の対象条件は `ポーズ確定` に統一する。

**中**
- なし。

**低**
- なし。
再レビューしました。前回指摘4件は、指摘内容そのものは解消されています。

- Step2 の `N_PAIR_MIN` 暫定扱い: 解消（[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:39)）
- Step3 の進行条件・目視判定項目: 解消（[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:47)）
- Step5 のポーズ変化量ゲート化: 解消（[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:63)）
- Step7 定数反映後の再検証: 解消（[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:78)）

**高**
なし。

**中**

1. **最終コード状態の手動テストが定義されていない**

[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:70) でフロー Step7 の手動テストを実施した後、[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:77) で採用定数をコード変更しています。その後は自動/e2e 再検証のみで、ユーザー手動テストを経ずに [Step8](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:80) へ進めます。CLAUDE.md の機能追加フローでは、最終実装後に手動テスト、その後完了です。

修正提案: Step7 のパラメータ決定・定数反映を手動テスト前に移動するか、Step7 後に「最終手動テスト」を追加し、Step8 完了処理はその後にしてください。

2. **Sonnet 委任時の検証責務が CLAUDE.md とずれている**

[roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:9) は「検証と報告は Claude Code 本体」としていますが、CLAUDE.md は Sonnet サブエージェントへの必須指示として `uv run pytest -v` 実行、`tests/results/...` 保存、動作確認まで求めています（[CLAUDE.md](/home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:365)）。このままだと Step1〜5 の各分割実装が、単独で CLAUDE.md 準拠の検証完了になりません。

修正提案: 方針を「Sonnet が実装と必須検証まで実施し、Claude Code 本体が結果確認・追加の実データ確認・ユーザー報告を行う」に変更し、各コード変更ステップの完了条件に pytest 結果保存を明記してください。

**低**
なし。
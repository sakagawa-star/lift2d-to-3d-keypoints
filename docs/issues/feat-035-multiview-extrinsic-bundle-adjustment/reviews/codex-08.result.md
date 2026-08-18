レビューしました。前回までの requirements/design 側の主要指摘はロードマップ上でも概ね反映されていますが、ロードマップ自体に **中 4件**あります。高・低はありません。

**中**

1. **Step 2 が `N_PAIR_MIN` の採否判断を FR-008 より前に行っている**
   [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:36) で「接続しない場合の閾値」を問い、[同:38](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:38) で `N_PAIR_MIN` 調整込みの Go 条件になっています。一方、`N_PAIR_MIN` 等の採否は FR-008 の実験項目です。ここで閾値を実質採用すると、事前基準ロック済みの実験で決めるという `CLAUDE.md` の実験プロトコルと衝突します。

   修正提案: Step 2 は「接続可能性の事前計測」に限定し、閾値変更は暫定扱いにする。最終採用は Step 7/FR-008 の criteria に基づく、と明記してください。

2. **Step 3 が「pass line なし」なのに継続判断を発生させている**
   [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:44) では pass line なしですが、[同:46](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:46) 以降でユーザー確認・継続判断があります。これは事後的な Go/No-Go になり得ます。

   修正提案: Step 3 criteria に「進行可能条件」を明文化してください。例: FR-002 通り「1台以上 bootstrap 成功なら次へ進む、失敗カメラは除外」など。視覚確認をゲートにするなら、判定項目も criteria に固定してください。

3. **Step 5 の `pose change <50cm/10°` が requirements/design にない新規ゲートになっている**
   [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:61) の health check は、requirements/design の完了条件にはありません。失敗扱いにすると、未レビューの追加受け入れ条件になります。

   修正提案: ブロッキング条件にしないなら「参考診断値」と明記する。ブロッキング条件にするなら requirements/design か実験 criteria に追加し、根拠も書いてください。

4. **Step 7 の定数反映後の再検証が不足している**
   [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:75) で実験後にコード定数を更新しますが、その後の pytest/e2e 再実行が明記されていません。`CLAUDE.md` の機能追加フローでは、実装変更後のテスト結果保存が必要です。

   修正提案: Step 7 に「採用定数反映後、`uv run pytest -v` を再実行して `tests/results/...` に保存し、採用パラメータで e2e/report/TOML render を再確認する」を追加してください。
**結果**

致命的な指摘はありません。前回までの `pipeline_loop.py` の挙動は、`run_pipeline()` 抽出後も維持されています。

**重要度分類**

- 高: なし
- 中: なし
- 低: なし

確認済み:
- [pipeline_loop.py:161](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/pipeline_loop.py:161) 以降の `run_pipeline()` は、旧 `main` 内の反復本体と同じ順序です。失敗時中断、ポーズ更新、収束判定、履歴追加の順序は変わっていません。
- [basin_map.py:63](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/basin_map.py:63) は `run_pipeline()` を直接呼んでおり、oracle は最終誤差による分類だけに使われています。PnPや収束判定には混ざっていません。
- [basin_map.py:68](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/basin_map.py:68) の分類は実験ログの定義通り、`converged and accurate` を成功、`converged and not accurate` を silent failure、それ以外を失敗にしています。
- [basin_map.py:61](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/basin_map.py:61) で各ケース同一 seed の RNG を作り直しているため、摂動方向は全ケースで同一になります。これは「方向を固定して摂動量だけ振る」実験としては妥当です。方向依存の収束域を見る段階では、seed 軸を追加する必要があります。
- [basin_map.py:52](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/basin_map.py:52) で PLY は1回だけ読み、全ケースに同じ `gaussians` を渡しているため、構造として正しいです。

今回は実行はしていません。コードと実験ログ定義との整合レビューのみです。
---

## Claude Code の記録（2026-07-31）

run_pipeline 抽出と basin_map.py のレビュー収束（codex-17、高・中ゼロ）。収束域マップ実行へ進む。
（メモ: 方向依存の収束域を見る段階では seed 軸の追加が必要——codex 指摘の通り。今回は方向固定で摂動量のみ振る）

前回指摘3件は解消されています。

- 後方互換の「argparse usage/help は対象外」化: 解消済み
- `--frustum-depth` の `nan/inf` 拒否: 解消済み
- `--fps-camera` 側のK/解像度使用を検出するテスト: 解消済み

**高**
- 頭部7点欠損チェックが `--fps-frustum` 時限定だと明記されておらず、FR-009 と衝突します。  
  [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:126) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:194) は、頭部7点が無い入力をエラーにすると書いていますが、`args.fps_frustum` 指定時だけ、という条件がありません。既存実装は欠損マーカーを描画スキップとして許容しています（[render_keypoints.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_keypoints.py:826)）。このまま実装すると `--fps-frustum` 未指定の既存入力まで失敗し、FR-009 の既定動作不変（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:146)）を破ります。  
  **修正提案**: FR-007 #6 と design 4.3/4.6 を「`args.fps_frustum` が True の場合のみ、頭部7点構成欠損をエラー」と明記する。T-11 も `--fps-frustum --fps-camera <cam>` 指定時のテストだと明記する。

**中/低**
- 致命的な追加指摘はありません。
---

## Claude Code の対応方針（2026-08-13）

- メタ: 再レビュー（2回目）/ 対象 requirements.md, design.md / session id: 019ffa10-98e4-7da0-8021-0c7bd958e638
- 前回指摘3件（高1・中2）: 解消確認済み
- **高（頭部7点欠損チェックの条件不明記）**: 採用。requirements FR-007 #6 と design 4.3-4 / 4.6 検証6 に「`--fps-frustum` 指定時のみチェックする。未指定時は既存の欠損許容のまま」を明記。テストに T-11b（`--fps-frustum` なしでは頭部マーカーエラーにならない）を追加

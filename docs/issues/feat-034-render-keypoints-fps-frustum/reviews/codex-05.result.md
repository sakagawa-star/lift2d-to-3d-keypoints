前回指摘3件は解消されています。

- `--fps-toml` なしの組み合わせ検証条件: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:220) で反映済み
- YAMLの4キー化と `fps_toml` テスト: [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:103)、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:286) で反映済み
- `investigation.md` の修正前/修正後コード: [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/investigation.md:36) 以降で反映済み

**高**
- なし

**中**
- FR-009 の後方互換例外リストに `--fps-toml` が抜けています。  
  `--help` / usage / YAML未知キー一覧に新オプション追加を許容する対象として、[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:156) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:193) は `--fps-frustum` / `--fps-camera` / `--frustum-depth` だけを列挙しています。`--fps-toml` も parser に追加するため、厳密にはFR-009と衝突します。  
  **修正提案**: 両方の例外リストに `--fps-toml` を追加する。

- 4.8 のエラーハンドリング表で、頭部マーカー欠損エラーの条件が再び曖昧です。  
  [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:241) は `--fps-frustum` 時限定と書いていないため、4.6の明示条件とズレます。  
  **修正提案**: `頭部マーカーが構成に無い（args.fps_frustum が True の場合のみ）` と明記する。

**低**
- 致命的な指摘に限定するため、報告対象なし。
---

## Claude Code の対応方針（2026-08-13）

- メタ: イテレーション1 再レビュー（5回目）/ 対象 investigation.md, requirements.md, design.md / session id: 019ffa10-98e4-7da0-8021-0c7bd958e638
- 前回指摘3件（中3）: 解消確認済み
- **中（FR-009 例外リストに --fps-toml 欠落）**: 採用。requirements FR-009 と design 4.5 の列挙に `--fps-toml` を追加
- **中（4.8 表の頭部マーカー欠損条件が曖昧）**: 採用。design 4.8 の該当行に「`args.fps_frustum` が True の場合のみ検査」を明記

前回指摘の「頭部7点チェックは `--fps-frustum` 時限定」は、FR-007・4.3・4.6・T-11/T-11b では解消されています。

**高**
- なし

**中**
- `--fps-toml` の組み合わせ検証条件が設計内で矛盾しています。  
  要求と検証表では `--fps-frustum` なしの `--fps-toml` は終了コード2です（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:126), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:215)）。しかし直後の実装条件説明では `args.fps_toml is not None` が抜けています（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:220)）。このままだと T-21 を落とす実装になり得ます。  
  **修正提案**: 4.6 の検証2判定を `args.fps_frustum is False and (args.fps_camera is not None or args.fps_toml is not None or args.frustum_depth is not None)` に修正する。

- YAML対応の受け入れ条件が `fps_toml` 追加後も古いままです。  
  FR-006 の表には `fps_toml` が追加されていますが、本文と受け入れ基準は「3キー」のままです（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:103), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:115)）。設計側も同じく「3キー」と書きつつ4キーを追加しています（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:225)）。さらにテスト設計に `fps_toml` のYAML経由確認がありません。  
  **修正提案**: 「4キー」に直し、T-12 または新規Tとして `fps_toml: <別TOML>` が `args.fps_toml` に反映され、main 経由で別TOML側Kが使われることを追加する。

- `investigation.md` が BUGFIX_STANDARD 必須項目の「修正コード」を満たしていません。  
  基準は修正前・修正後コードの提示を要求しています（[BUGFIX_STANDARD.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/BUGFIX_STANDARD.md:41)）。現状は修正要点のみで、具体的な before/after がありません（[investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/investigation.md:30)）。  
  **修正提案**: `render_keypoints.py` の `_build_parser`、`CONFIG_CONVERTERS`、`main` のFPSカメラ選択・検証部分について、最小限の修正前/修正後コード片を investigation.md に追加する。

**低**
- 致命的な指摘に限定するため、報告対象なし。
---

## Claude Code の対応方針（2026-08-13）

- メタ: イテレーション1 レビュー（4回目）/ 対象 investigation.md, requirements.md, design.md / session id: 019ffa10-98e4-7da0-8021-0c7bd958e638
- **中（検証2の実装条件に fps_toml 欠落）**: 採用。design 4.6 の判定条件に `args.fps_toml is not None` を追加
- **中（「3キー」表記の残存・fps_toml のYAMLテスト不足）**: 採用。requirements FR-006 本文・受け入れ基準と design 4.7 を「4キー」に修正、T-12 を4キーに拡張、T-23（YAML経由の fps_toml で別TOML側Kが使われる）を追加
- **中（investigation.md に修正前後コードがない）**: 採用。`_build_parser` / `CONFIG_CONVERTERS` / main の組み合わせ検証・FPSカメラ選択の before/after コード片を 1.3 に追記

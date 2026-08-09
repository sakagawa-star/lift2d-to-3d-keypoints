<!-- メタ: 2026-08-08 / 対象: feat-027 requirements.md + design.md（ffprobe実挙動・libx264実行時依存の訂正） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(11回目) -->

**高**
なし。

**中**
なし。

**低**
なし。

前回指摘は解消されています。libx264 有効ビルドの必須条件は [requirements.md NFR-001](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:186) と [FR-005](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:109) に入り、起動前検査も [design.md §4.11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:344) に追加されています。インターフェース定義と T19 も対応済みです。

破損検出の要求も弱まっていません。FR-007 は引き続き「読み取り可能かつ期待フレーム数一致」のみスキップで、§4.7.1/T17 は ffprobe 実装差による理由文字列の揺れだけを許容しており、不合格判定自体は維持されています。

---

**Claude Code の対応方針**（再レビュー(11回目)、指摘: 高0・中0・低0）: 収束。テストコード・実装の修正（ffmpeg_has_libx264 追加、T17 期待値緩和、T19 追加）へ進む

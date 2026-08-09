<!-- メタ: 2026-08-08 / 対象: feat-027 requirements.md + design.md（ffprobe実挙動・libx264実行時依存の訂正） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(10回目・ffprobe実挙動への設計訂正) -->

**高**
なし。

**中**
1. libx264 が実行時依存に昇格しているのに、要件・起動前検査がまだ `ffmpeg` バイナリ存在確認止まりです。[requirements.md NFR-001](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:186) は `ffmpeg / ffprobe` のみ、[FR-005](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:109) も `ffmpeg` 不在時だけを開始前エラーにしています。一方で T14/T16/T17 は [design.md:467](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:467) 以降で「libx264 使用不可なら skip」としており、[§4.11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:344) の起動前検査ともズレています。  
   修正提案: NFR-001/FR-005/§4.11 に「MP4 モードは ffmpeg が `libx264` エンコーダを使用可能であること」を明記し、開始前検査に `ffmpeg -hide_banner -h encoder=libx264` 等の確認を追加してください。

**低**
なし。

破損検出要求自体は弱まっていません。[FR-007](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:124) は引き続き「読み取り可能かつ期待フレーム数一致のみスキップ」で、[§4.7.1](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:278) も `returncode != 0` と `nb_frames` 取得不能のどちらも不合格扱いです。T17 の2理由許容も、検出を緩める変更ではなく実機差分の吸収として整合しています。

---

**Claude Code の対応方針**（再レビュー(10回目・ffprobe実挙動への設計訂正)、指摘: 高0・中1・低0（FR-007の検出要求は弱まっていないと確認））: NFR-001/FR-005 に libx264 必須を明記、§4.11 に ffmpeg_has_libx264 起動前検査を追加、§1.7・T19 追加

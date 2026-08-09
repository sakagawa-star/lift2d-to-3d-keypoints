<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md（破損チャンク検出・耐久書き出しの改訂） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(7回目) -->

**高**
なし。

**中**
1. [design.md §4.7](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:263) で `chunk_dir` 作成後の親ディレクトリ fsync が未定義です。内部の `manifest.json` や chunk MP4 は `fsync_dir(chunk_dir)` されていますが、`<output>.chunks/` というディレクトリエントリ自体の永続化は親ディレクトリの fsync が必要です。[NFR-003](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:188) の「電源断後も完成済みチャンクを失わない」と厳密にはズレます。  
   修正提案: `os.makedirs(chunk_dir)` 後に `fsync_dir(os.path.dirname(chunk_dir))` を実行する手順を §4.7 に追加してください。

2. 今回追加された信頼性仕様に対して、テスト設計の対応がまだ不足しています。`manifest.json` の耐久書き出しと破損 JSON エラーは [§4.7](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:263) に追加されていますが、[T8](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:460) は build/mismatch のみです。また最終 MP4 の fsync 手順は [§4.8](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:302) にありますが、[T16](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:468) は最終パス存在確認までです。  
   修正提案: manifest 破損 JSON のエラーメッセージ確認、manifest/final MP4 の `fsync_file -> os.replace -> fsync_dir` 呼び出し順確認を T8/T16 または新規テストに追加してください。

**低**
なし。

前回指摘2件は解消されています。manifest の fsync 手順と破損 JSON エラーは §4.7 に入り、FR-008 の最終 MP4 書き出しも fsync 込みの表記に揃っています。ffprobe によるチャンク検証、破損警告メッセージ、T17 の対応も大筋で整合しています。

---

**Claude Code の対応方針**（再レビュー(7回目)、指摘: 高0・中2・低0（前回2件は解消確認））: chunk_dir 作成直後の親ディレクトリ fsync を追加。確定書き出しを durable_replace に集約。T8 拡充・T18 新設

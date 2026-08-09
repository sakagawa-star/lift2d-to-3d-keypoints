<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md（破損チャンク検出・耐久書き出しの改訂） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(6回目・破損チャンク検出の改訂) -->

**高**
なし。

**中**
1. [design.md §4.7](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:263) の `manifest.json` 書き出しがまだ `tmp -> os.replace` だけで、`fsync_file(tmp) -> os.replace -> fsync_dir(chunk_dir)` になっていません。再開判定は manifest に依存しているため、電源断で manifest が欠損・破損すると、完了済みチャンクが残っていても「manifest なし」扱いで再開不能になり得ます。  
   修正提案: manifest 書き出しにも §4.5 と同じ durable rename 手順を適用し、破損 JSON 読み込み時のエラー文言も明示してください。

2. [requirements.md FR-008](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:136) は最終 MP4 についてまだ「一時ファイルから `os.replace`」までしか書いておらず、[NFR-003](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:188) と [design.md §4.8](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:301) の `fsync_file -> os.replace -> fsync_dir` と不整合です。  
   修正提案: FR-008 も同じ fsync 手順に揃えてください。

**低**
なし。

前回指摘の `gaussians` 所有・ロード責務の問題は引き続き解消済みです。今回の信頼性改訂については、チャンク MP4 本体の fsync 手順、ffprobe による再開時検証、警告メッセージ、T17 の検証テストは概ね対応しています。残る致命的な穴は manifest 耐久性と FR-008 の要件不整合です。

---

**Claude Code の対応方針**（再レビュー(6回目・破損チャンク検出の改訂)、指摘: 高0・中2・低0）: manifest 書き出しを耐久手順に変更し破損JSONエラー文言を追加。FR-008 の最終MP4書き出しを fsync 込み表記に統一

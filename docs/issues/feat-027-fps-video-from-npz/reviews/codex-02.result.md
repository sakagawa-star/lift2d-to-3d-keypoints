<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(2回目) -->

**高**

1. [design.md:283](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:283) の `concat_chunks()` でも出力先が [design.md:291](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:291) の `<output_path>.tmp` になっていますが、ffmpeg コマンド例に `-f mp4` がありません。前回の区間MP4と同じ問題が最終MP4連結側に残っています。`.tmp` 拡張子では出力コンテナを判別できず、FR-008 の最終MP4生成が失敗します。  
修正提案: concat コマンドも出力パス直前に `-f mp4` を追加する。例: `ffmpeg -y -f concat -safe 0 -i concat_list.txt -c copy -loglevel error -f mp4 <output_tmp>`。あわせて `<output>.mp4.tmp` への concat スモークテストを追加する。

**中**

該当なし。

**低**

致命的でない表記・細部は指摘していません。

**前回指摘の確認**

前回の4点は解消されています。区間MP4の `-f mp4` は [design.md:246](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:246) と T14 に反映済み、T6 の float32 許容差は [design.md:440](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:440) で緩和済み、F=0 は [requirements.md:45](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:45) / [design.md:88](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:88) に追加済み、描画0件平均も [requirements.md:170](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:170) / [design.md:397](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:397) で定義済みです。

---

**Claude Code の対応方針**（再レビュー(2回目)、指摘: 高1・中0・低0（前回4件は解消確認））: concat_chunks 側にも -f mp4 を追加し、連結スモークテスト T16 を新設

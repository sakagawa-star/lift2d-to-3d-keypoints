<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 初回 -->

**高**

1. [design.md:237](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:237) の `start_ffmpeg_chunk()` スニペットに `-f mp4` が入っていません。一方で [design.md:248](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:248) は `.tmp` 拡張子では MP4 判別できないため `-f mp4` 必須と明記しています。スニペットどおり実装すると区間MP4生成が失敗し、FR-005/008 が成立しません。  
修正提案: ffmpeg コマンド例に `"-f", "mp4"` を出力パス直前へ追加し、`.mp4.tmp` への書き出しスモークテストをテスト設計に入れる。

2. [design.md:436](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:436) の T6 は `c2w_blender_to_viewmats()` が `float32` を返す要求と、`atol=1e-9` の同一性検証が矛盾しています。float32 の丸めを許容していないため、正しい実装でもテストで落ちる可能性が高いです。  
修正提案: `atol=1e-6` 程度に緩和するか、float64 の中間結果で逆行列性を確認し、別途「戻り値 dtype が float32」を検証する形に分ける。

**中**

1. F=0 の入力扱いが未定義です。既存 `load_npz()` は F=0 を拒否しない一方、設計は `frame_ids[0]` / `frame_ids[-1]` 前提のマニフェスト・ログを持っています（例: [design.md:260](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:260)）。境界入力でクラッシュ経路が残ります。  
修正提案: FR-001 に「F==0 はエラー終了」を明記し、§4.1 と CLI/NPZ テストに追加する。

2. 全フレーム無効の場合のログ計算が閉じていません。[design.md:168](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:168) は全黒動画継続を要求しますが、[requirements.md:169](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:169) / [design.md:393](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:393) は描画フレーム平均秒を常に出す前提です。描画フレーム数0でゼロ除算または未定義表示になります。  
修正提案: `n_rendered == 0` の場合は `描画フレーム平均: N/A（描画0）` のように仕様化し、全NaN入力のテストを追加する。

**低**

致命的でない表記・細部は指摘していません。

---

**Claude Code の対応方針**（初回、指摘: 高2・中2・低0）: 4件すべてドキュメントに反映（-f mp4 追加+T14、T6 許容誤差分離、F=0 エラー明記+T13、format_render_average 定義+T15）

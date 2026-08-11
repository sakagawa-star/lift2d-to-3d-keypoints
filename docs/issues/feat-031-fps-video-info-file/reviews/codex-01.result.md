**レビュー結果**

高: なし。

中: 1件。

- [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-031-fps-video-info-file/design.md:193) のテスト設計が純粋関数中心で、実際の自動保存経路を自動テストしていません。変更の本体は [_run_mp4_mode](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:1091) 末尾と [main の呼び出し](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:1363) なので、ヘルパ関数だけ実装して呼び忘れても予定テストを通過できます。  
  修正提案: `tests/test_feat031_info_file.py` に、`_run_mp4_mode` を monkeypatch で軽量実行する統合テストを追加してください。最低限、連結成功時に info が生成されること、連結失敗時は生成されないこと、`write_info_file` 失敗時に rc=1 になること、全チャンクスキップ時に `N/A（描画0フレーム）` になることを検証するべきです。

低: 致命観点では指摘なし。

実コード整合は確認済みです。設計書中の既存参照は、`finite_mask/n_nan/n_degen` が [render_fps_video.py:1347](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:1347)、`elapsed` と既存完了ログが [render_fps_video.py:1228](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:1228)、`_run_mp4_mode` 呼び出しが [render_fps_video.py:1363](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:1363) で一致しています。既存ログ値との対応も、設計どおり同じ変数・同じ `format_render_average()` を使う方針で矛盾はありません。
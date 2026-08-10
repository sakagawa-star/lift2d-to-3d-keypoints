<!-- メタ: 2026-08-09 / 対象: feat-029 requirements.md + design.md / session id: 019fe4ae-1cb8-7881-96dc-45c25e014ae9 / 初回 -->

**高**

1. YAML 数値キーの「型変換失敗」が `parser.error(SystemExit(2))` にならず、実装次第で `ValueError` クラッシュになります。  
   [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/requirements.md:97) は数値キーの型変換失敗をキー名つき終了コード2で検出する要求ですが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:79) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:135) は `ArgumentTypeError` だけを `parser.error` に流す設計です。既存の `_positive_float` / `_positive_int` / `_nonnegative_int` / `_crf_int` は、`fps: abc` や `gpu: abc` のような値で `float()` / `int()` 由来の `ValueError` を投げます（例: [render_fps_video.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:543), [render_fps_video.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:557)）。  
   修正提案: `parse_config_yaml` の変換部で `argparse.ArgumentTypeError` だけでなく `ValueError` / `TypeError` も捕捉し、必ず `parser.error(f"設定YAML キー '{k}': ...")` に変換する。もしくは既存の数値検証関数側で `ValueError` を `ArgumentTypeError` に包み直す。テスト設計の T10 も `crf: abc` / `fps: abc` / `gpu: abc` / `chunk_size: abc` を追加し、範囲外だけでなく型変換失敗を検証してください。

**中・低**

致命的な問題に絞ると、追加で報告すべき中・低の指摘はありません。

---

**Claude Code の対応方針**（初回、指摘: 高1・中0・低0）: parse_config_yaml の変換部で ValueError/TypeError も捕捉して parser.error に変換する仕様に修正。T10 に型変換失敗ケースを追加

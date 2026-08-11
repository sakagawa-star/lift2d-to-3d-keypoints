**高**
1. [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:48) の `--gpu default=None` + [main 内正規化](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:76) は、既存 pytest 無修正の制約に反します。既存テストは [parse_args_with_config() 単体で `args.gpu == 0`](/home/sakagawa/git/lift2d-to-3d-keypoints/tests/test_feat029_config_yaml.py:227) を固定しており、設計どおりだとここが壊れます。  
修正提案: `--gpu` の argparse default は `None` でよいが、正規化を `main` だけでなく `parse_args_with_config()` 後段に置き、`args.gpus is None and args.gpu is None` の場合だけ `args.gpu = 0` にする。`--gpus` 指定時は `args.gpu is None` を維持する。

2. FR-004 の失敗時停止設計が実プロセスで保証不足です。[requirements.md:59](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/requirements.md:59) は `.tmp` 削除・再開可能状態を要求していますが、設計の `_abort` は [worker を terminate して tmp を消すだけ](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:225) です。実装の worker 内では [ffmpeg subprocess](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:410) を使うため、worker への SIGTERM では `render_chunk()` の通常 cleanup が走らず、ffmpeg 子プロセスや `.tmp` 削除との競合が残ります。  
修正提案: worker 起動時に SIGTERM を `KeyboardInterrupt` 相当に変換して [render_chunk の ffmpeg terminate 経路](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:484) を通す、または worker/ffmpeg をプロセスグループ化して `_abort` で子プロセスまで停止する設計を明記する。さらに [requirements.md:62](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/requirements.md:62) の実 worker kill 手動確認を design の手動テストに追加する。

**中**
1. 並列性能の検証指標とログ設計が噛み合っていません。[requirements.md:48](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/requirements.md:48) は「合計スループット 1.3倍以上」を要求しますが、design は worker の `sec` を [単純加算](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:207) し、既存の [format_render_average](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:535) 形式で出すだけです。並列時の `sum(worker sec) / frames` は実効スループットではないため、1.3倍判定に使うと誤ります。  
修正提案: 並列モードでは wall-clock の render 区間秒を別に計測し、`実効スループット = total_rendered / wall_sec` または `実効秒/フレーム = wall_sec / total_rendered` をログ・受け入れ基準・手動テストに明記する。既存平均は出すなら「worker時間平均」とラベルを分ける。

**低**
なし。上記以外は致命的な不整合としては扱いません。実験 e1 の R2=1.487 と要求・設計の前提は整合しています。
**高**
1. [design.md:135](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:135) の外側 `except KeyboardInterrupt: return` が危険です。SIGTERM を KeyboardInterrupt に変換した結果、worker が `task_q.get()` 待機中、または task を取得して内側 `try` に入る前に kill されると、exitcode 0 の正常終了扱いになります。一方、親側は [exitcode 非0だけを異常死として検出](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/design.md:216) する設計なので、FR-004 の「worker kill で即時中止」に反し、タイミングによっては task 消失で分配ループが待ち続けます。  
修正提案: worker の SIGTERM/KeyboardInterrupt を静かに `return` しない。少なくとも `current_idx` を保持し、task 取得後なら `.tmp` cleanup と `result_q.put(("error", ...))` を必ず行う。task 未取得時も外部 kill は非0終了または明示的 error message にして、親が abort できるようにする。

**中**
なし。

**低**
なし。

前回指摘への対応状況:
- 高1（`--gpu` 正規化）は解消。`parse_args_with_config()` 直後に正規化する設計になり、既存 feat-029 テスト条件と整合しています。
- 高2（SIGTERM/ffmpeg/tmp）は大筋改善されていますが、上記の silent return が残っているため FR-004 はまだ閉じていません。
- 中1（並列時の実効秒/フレームと 1.3倍判定）は解消。requirements/design ともに測定式・ログラベルが分離されています。
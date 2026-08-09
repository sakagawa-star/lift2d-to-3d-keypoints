<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md（破損チャンク検出・耐久書き出しの改訂） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(8回目) -->

**高**
なし。

**中**
1. [design.md §4.5](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:221) の `durable_replace(tmp, dst)` と [§4.7](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:261) の `fsync_dir(os.path.dirname(chunk_dir))` が、ディレクトリ部分を持たない相対パスで壊れます。`--output out.mp4`、`--dump-poses poses.json`、またはカレントディレクトリの NPZ からデフォルト出力を作るケースでは `os.path.dirname(...) == ""` になり、`fsync_dir("")` が失敗します。CLI では basename のみの出力パスを禁止していないため、これは有効入力でのクラッシュにつながります。  
   修正提案: `durable_replace` と chunk_dir 親 fsync は `os.path.dirname(os.path.abspath(path))` を使う、または `os.path.dirname(path) or "."` を明記してください。T18 には basename-only の `dst_path` / `chunk_dir` ケースも追加すると、この境界条件を固定できます。

**低**
なし。

前回指摘の2件は、方針としては解消されています。chunk_dir 作成直後の親ディレクトリ fsync、manifest の耐久書き出し・破損 JSON エラー、T8/T18 の追加はいずれも反映されています。残っているのは、共通ヘルパー化で生じたパス正規化の境界条件です。

---

**Claude Code の対応方針**（再レビュー(8回目)、指摘: 高0・中1・低0（前回2件は方針解消確認））: durable_replace / chunk_dir 親 fsync の dirname を abspath 経由に変更（basename のみのパス対策）。T18 に basename ケース追加

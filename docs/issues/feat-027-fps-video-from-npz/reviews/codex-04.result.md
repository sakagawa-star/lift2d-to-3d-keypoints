<!-- メタ: 2026-08-07 / 対象: feat-027 requirements.md + design.md（複数GPU並列の後続案件分割・単一GPU直列化の改訂） / session id: 019fdb4e-226d-77c1-9a82-bdcd71a7d696 / 再レビュー(4回目・単一GPU化改訂) -->

**中**

1. [design.md:187](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:187) の `render_chunk(..., gaussians) -> tuple[int, float]` は、[requirements.md:103](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/requirements.md:103) の「PLY はGPUに1回だけロードし、全チャンクで使い回す」と噛み合っていません。設計内の疑似コードでは [design.md:193](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:193) で `gaussians = None`、[design.md:198](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:198) で必要時ロードですが、`render_chunk` が `gaussians` を引数で受けて戻り値に返さない形だと、関数内でロードした `gaussians` が呼び出し元に戻らず、チャンクごとにPLY再ロードする実装になり得ます。  
修正提案: `gaussians` の所有を main 側に寄せ、最初の有効チャンク前に main がロードして以後同じ参照を渡す設計にする。もしくは `render_chunk(...) -> tuple[gaussians, int, float]` のように戻り値へ含めるか、`RenderContext` に `gaussians` を持たせる。

**高**

なし。

**低**

なし。瑣末な表現は指摘していません。

**並列前提の残存確認**

`requirements.md` / `design.md` の対象範囲では、`--gpus`、worker、multiprocessing、task/result queue などの旧並列実装前提は消えています。FR-006 も欠番として扱われ、`--gpu` 単一指定に揃っています。

前回指摘の final concat 側 `-f mp4` も [design.md:270](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:270) と [design.md:435](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/design.md:435) で解消済みです。

---

**Claude Code の対応方針**（再レビュー(4回目・単一GPU化改訂)、指摘: 高0・中1・低0（並列前提の消し残りなしと確認））: design §4.5 で gaussians の所有を main 側に明確化（main が1回ロードし render_chunk に同一参照を渡す。render_chunk はロードしない）

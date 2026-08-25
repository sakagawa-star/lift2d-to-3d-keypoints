技術的な根拠自体は妥当です。`sched(1)["tau_px"] = 10.0` は feat-026 実装と一致し、`gate_breakdown(u_q, u_r, ..., tau_px)` の使い方も実装と対応しています: [refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:74), [refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:420)

**高**

なし。

**中**

1. `sched` の出所が design の流用元・import 方針に入っていない  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:83) で `sched(1)["tau_px"]` を使っていますが、流用元一覧と import 方針は `GATE_* / NEAR_PLANE` までで、`sched` が含まれていません: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:25), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:87)

修正提案: `refine_extrinsics.py` の流用元一覧と import 方針に `sched` を追加する。あるいは `ANCHOR_DISP_TAU_PX = 10.0` とリテラル定義し、コメントで `refine_extrinsics.sched(1)["tau_px"]` 由来と明記する。

2. requirements FR-003 のゲート定数一覧に `ANCHOR_DISP_TAU_PX` が反映されていない  
FR-003 は feat-026 のゲート定数を列挙していますが、今回追加した cond_disp の 10px が requirements 側には出ていません: [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:61)

修正提案: FR-003 の定数列挙に `ANCHOR_DISP_TAU_PX=10.0px（feat-026 sched(1).tau_px、FR-008対象外）` を追加する。あわせて [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:62) の「FR-008 の実験で見直す前提の初期値」という総称は、固定流用値も混ざるため「FR-008対象は各コメントに明記。固定流用値は出典を併記」程度に直すと矛盾が消えます。

**低**

なし。
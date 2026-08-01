**Findings**

- **高**: 二峰仲裁の受理条件がまだ広すぎます。[criteria.md:80](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:80)〜[83](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:83) は「第2クラスタ占有率 ≥ 0.2」を要求していないため、第1=39/60・第2=3/60・残り散乱のような、M3-2 では二峰扱いできない結果でも「上位2合計 42/60」で仲裁ルートに入れます。M3-2 の二峰検出は `第2クラスタ ≥ 0.2` が前提です（[criteria_m3_2.md:16](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/criteria_m3_2.md:16)、実測も 41/60 + 19/60; [experiment_log.md:245](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/experiment_log.md:245)）。  
  **修正提案**: 仲裁成立の二峰条件に `第2クラスタ占有率 ≥ 0.2（=12/60）` を追加する。さらに M3 D3 の実証範囲に寄せるなら `第1+第2 ≥ 0.9（=54/60）` も入れて、散乱30%を含むケースを多峰/不安定として No-Go にする。

- **中**: [criteria.md:98](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:98) が「合否は §6.1〜6.2 のみ」と残っており、[criteria.md:100](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:100)〜[103](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:103) と [criteria.md:117](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:117) の「§6.4 も Go 前提」と矛盾しています。実行者が §6.4 を記録扱いに戻せる余地があります。  
  **修正提案**: [criteria.md:98](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria.md:98) を「b-1〜b-4 は合否に使わない。合否は §6.1・§6.2・§6.4 で判定する」に修正する。

**前回指摘の解消状況**

- 深度ゲートの後出し変更: 解消。§4 で完全固定され、別実験に分離されています。
- 総合判定の非ゲート化: ほぼ解消。§6.4 と §7 には反映済みですが、上記の §6.3 文言だけ矛盾が残っています。
- 二峰仲裁: 部分解消。対象クラスタ・第3クラスタ・仲裁差は固定されましたが、第2クラスタ下限が抜けており、まだ Go を動かせる余地があります。
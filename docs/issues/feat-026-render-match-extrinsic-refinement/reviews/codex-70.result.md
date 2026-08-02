**指摘**

- **中** [investigation.md:11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/investigation.md:11) の「現在の動作」に、BUGFIX_STANDARD §1.1 が要求する具体的な再現手順が不足しています。`M7 手動テストのコマンド` への参照だけでは、`/clear` 後に同じ入力・同じ失敗を再現できません。  
  **修正提案**: 実行コマンド、主要入力パス、対象カメラ、失敗レポートの代表行を investigation に直接記載する。

- **中** D3 根拠の書き方が、今回の「入力全手動点の再投影中央値」と完全一致していません。[requirements.md:25](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:25)、[design.md:78](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:78)、[investigation.md:21](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/investigation.md:21) は「全手動点」仲裁を D3 で確立済みのように読めますが、D3 の実行前固定判定は [d3_arbitrate.py:5](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py:5)-[7](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_arbitrate.py:7) の通り **M3-0a inlier 名簿中央値**です。全点中央値も [d3_result.txt:5](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_result.txt:5)-[10](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/d3_result.txt:10) に記録され、A/B差は 1.311px で同じ結論ですが、lock 済み判定そのものではありません。  
  **修正提案**: 「D3 は多点手動点による仲裁を実証。固定判定は inlier 名簿中央値、全点中央値でも同じ採用結果を確認済み」と限定して書く。今回の正式仕様で全点中央値を採用する理由は「外部の inlier 名簿依存を避け、入力だけで閉じるため」と明記する。

**確認済み**

「一意6点ちょうど」の現行仕様記述は、requirements/design の有効本文では「一意6点以上・全点使用」に更新されています。実装修正範囲も `phase4/refine_extrinsics.py` と `tests/test_feat026_refine.py` に特定されており、大枠の過不足は見当たりません。
**高**

1. `criteria.md` の §4b が、後追い追補なのに「locked済み criteria に基づく採用」として扱われています。  
   [criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/criteria.md:3) は codex-34〜35 で locked としていますが、[criteria.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/criteria.md:52) の §4b は停滞事象を見た後の追補です。さらに [experiment_log.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/experiment_log.md:86) では、改訂採用値を「criteria.md 追補 §4b の結合検証規則に基づく」と書いています。これは CLAUDE.md の「実験前に criteria lock」「事後解釈禁止」と衝突します。

   修正提案: 現在の改訂を「当初 criteria の盲点による反映後再検証失敗への対応、ユーザー承認済みの設計差し戻し」として記録し、`§4b に基づく` という現在事象への適用表現は避けてください。`criteria.md` のステータスも「改訂1 draft / re-lock 待ち」に戻し、このレビュー後に改訂1として lock する扱いにする。今後の再発防止ルールとして使うなら、design §3 にも結合検証規則を追記してください。

**中**

1. 日付がこのセッションの現在日付と食い違っています。  
   このレビュー時点の環境日付は **2026-08-25 Asia/Tokyo** ですが、[experiment_log.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/experiments/p1_param_sweep/experiment_log.md:84) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:69) には **2026-08-26** の承認・改訂として記録されています。実験証跡として未来日付になるため、後で追跡不能になります。

   修正提案: 実施・承認日が 2026-08-25 ならその日付に統一してください。実際に 2026-08-26 の作業として記録するなら、このレビュー自体も 2026-08-26 以降の証跡として扱う必要があります。

**低**

該当なし。

HUBER の具体値は requirements 側で固定規定されていないため、`HUBER_PX=2.0` への改訂自体は requirements と矛盾していません。
**高**

1. 交会角重みの採用値が requirements と design 内 ADR に反映されておらず、主経路仕様が矛盾しています。  
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:167) では `w_k=1` 固定、つまり交会角重み無効化がユーザー承認済み採用値として明記されています。一方、[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:80) はまだ「クロス項には三角測量交会角に基づく重み付けを既定で適用する」としており、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:275) の ADR も「ガウス型スコア」を採用判断として残しています。実装者がどちらを正とするかで `build_cross_points` / BA 残差が変わるため、これは致命的な仕様矛盾です。

   修正提案: `requirements.md` FR-005 を「p1_param_sweep の採用結果により、主経路では交会角重みを無効化し `w_k=1` 固定。`TRI_MIN_ANGLE_DEG` の三角測量ゲートは維持」に更新してください。あわせて design ADR の該当行を「当初はガウス型を採用候補にしたが、p1_param_sweep で `w_k=1` を採用。`crossing_angle_weight` 関数と単体テストは将来再有効化用に温存」へ差し替えるべきです。

**中・低**

該当なし。

§4 の交会角重みテスト自体は、関数を温存する方針と整合しているため問題ありません。主な未解消点は、採用値 `w_k=1` が要求仕様と ADR にまだ伝播していないことです。
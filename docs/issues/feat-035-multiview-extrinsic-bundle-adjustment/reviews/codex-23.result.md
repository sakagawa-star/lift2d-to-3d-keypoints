再レビューしました。前回の2件は主経路では解消しています。design §2.4/§2.5/§2.6 は `ポーズ確定カメラ` 基準になり、`--quality-accepted-cameras` で品質受理を渡すI/Oも追加されています。

**高**
- `--quality-accepted-cameras` の省略を測定モードのトリガにしているため、品質受理カメラが0台でもフル実行したい合法ケースを実行できません。[design.md:135](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:135) では Stage R 対象0台をスキップ可能ですが、[design.md:185](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:185) では `--quality-accepted-cameras` 省略時に必ず測定モード終了、かつ `nargs='+'` なので空リスト指定もできません。全カメラが信頼初期カメラの場合、または品質受理0台だが信頼初期カメラだけで後段へ進めたい場合に詰みます。
  - 修正提案: 測定モードは `--stage-r-measure-only` / `--stop-after-stage-r` のような明示フラグに分離する。`--quality-accepted-cameras` は未指定なら空リストとして扱い、フル実行時の `ポーズ確定 = 信頼初期 ∪ 指定カメラ` を可能にする。指定カメラがある場合だけ「今回の合意受理カメラの部分集合」を検証する。

**中**
- MVP定義がまだ古い状態語を使っています。[requirements.md:139](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:139) は `Stage R 受理カメラ` と書いており、現在の定義である `Stage R 合意受理かつ品質受理` と一致していません。また `閾値確定（FR-008）はMVP後` は、Step 3R品質閾値がMVP前に必要な点と混同されやすいです。
  - 修正提案: MVPのFR-007定義を `信頼初期カメラ、または Stage R 合意受理かつ品質受理のカメラ` に更新する。MVP後に行うFR-008の閾値確定は、N_PAIR_MIN/BA重み/HOLDOUT等のパラメータ実験であり、Step 3R品質閾値はStep 3R内でMVP前に確定すると明記する。

**低**
- なし。
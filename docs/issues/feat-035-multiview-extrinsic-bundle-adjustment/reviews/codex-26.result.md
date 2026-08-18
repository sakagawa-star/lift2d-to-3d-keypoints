再レビュー結果です。案Aの大枠は整っていますが、実装判断を誤らせる致命的な点が残っています。

**高**

1. Stage F 後の採用ペアグラフの連結性条件がない  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:74), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:148), [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:67)

FR-010/Step 3F は「全ペア不採用でなければ可」「採用ペア1以上」で進める設計になっています。これだと、エピポーラフィルタ後に1ペアだけ残っても BA に進めてしまい、8台多視点の相互整合性という主目的を満たしません。Step 2 の連結性確認はフィルタ前なので、Stage F 後には保証になりません。

修正提案: Stage F 後に採用ペアグラフを構築し、「全ポーズ確定カメラ、または少なくとも BA 参加候補カメラが単一連結成分に入ること」を Go 条件として FR-010/design/roadmap/p0_epi_filter criteria に明記してください。不成立時は対象縮小・EPI_TOL再検討・方針差し戻しの扱いを事前定義する。

2. Sampson 距離の単位定義が曖昧で、フィルタが実質無効になり得る  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:147)

`d_px = sampson * f_mean` とありますが、標準的な Sampson distance は二乗距離として扱われることが多く、そのまま焦点距離を掛けると px 閾値との次元が合いません。実装者がこの通り書くと 3px 閾値が大幅に緩くなり、今回追加した偽マッチ防御が効かない可能性があります。

修正提案: 正規化座標で  
`sd = (x_j^T E x_i)^2 / denom`、`d_px = sqrt(sd) * f_mean`  
と明記するか、二乗値で比較するなら `sd <= (EPI_TOL_PX / f_mean)^2` と明記してください。テストにも 3px 相当は通過、4px 相当は除去の境界ケースを追加する。

**中**

1. EPI_TOL_PX が FR-008 の実験計画から漏れている  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:76), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:234), [roadmap.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:95)

requirements/MVP では EPI_TOL_PX を FR-008 で決定するとしている一方、design §3 と roadmap Step 7 の走査対象から抜けています。暫定 3.0px が固定値として残ると、実験プロトコル上「測ったうえで採用した」状態になりません。

修正提案: design §3 と roadmap Step 7 に EPI_TOL_PX の走査範囲、記録項目、採用ルールを追加してください。roadmap の実験プロトコル対象にも Step 3F を含めると整合します。

2. 案Aの CLI 例と `--init-cameras` 省略規則が旧事故を再発し得る  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:186), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:191)

案Aでは全8台を信頼初期として `--init-cameras` に与える方針ですが、CLI例は旧2台指定のままです。また省略時に init-toml 全カメラを信頼する規則は、feat-026 出力TOMLの無効ポーズを全台信頼してしまった事故と同種のリスクを残します。

修正提案: 案Aの標準コマンドは8台全てを `--init-cameras` に列挙する形に変更してください。さらに inuyama 主経路では `--init-cameras` を必須にする、または省略許可は「全対象カメラが Step 3M で受理済みの TOML」に限定する、と明記する。

3. 廃止済み Stage R / FR-009 への依存記述が主経路側に残っている  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:45), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:81), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:120), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:136)

FR-009 は Won’t で廃止済みなのに、FR-002 の成果物、アンカー不足カメラの最終ポーズ、手動基準点の例外条件が Stage R / FR-009 に依存した表現のままです。実装者が廃止経路を参照して判断する余地が残ります。

修正提案: 主経路の記述はすべて「信頼初期ポーズ（Step 3M の feat-026 受理済み TOML）」に置換し、FR-009 は廃止記録だけに閉じ込めてください。

**低**

なし。
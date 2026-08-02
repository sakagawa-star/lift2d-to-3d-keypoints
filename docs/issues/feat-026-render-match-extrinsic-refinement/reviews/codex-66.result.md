**高**

1. [requirements.md:11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:11) が「20点方式と統計的に不可分以上の解を自動で得る」と一般化しているのは、M4/M7 の実測上の限界と矛盾します。M4 criteria は「真値も20点方式基準もなく、診断値ベースの運用受理に限定」と明記しており、M4-2 でも手動6点再投影中央値は 2.233px→4.545px に悪化し、記録のみです。  
修正提案: 目的文を「M3 では20点方式 p95 内を確認済み。正式ツールの受理は pooled 合意・前提条件・診断値に基づく運用受理であり、真値精度や20点方式超えは主張しない」に変更する。

2. 入力点の検証が「6点ちょうど」止まりで、6個の**一意な** 2D-3D 対応であることが固定されていません。[design.md:100](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:100) は ObjectName 突合だけ、[design.md:102](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:102) は重複を「点数が6を超えるため検出」としていますが、重複1件＋欠損1件なら点数は6のまま通ります。初期6点PnPの根が壊れるため致命的です。  
修正提案: 3D CSV は `ObjectName` 一意、2D CSV は `(camera_name, ObjectName)` 一意、対象カメラは交差集合が **6 unique ObjectName ちょうど**、座標は全て finite、画像範囲内、欠損列なし、と明記する。

**中**

1. 方式Aの深度レンダ実装が自己完結していません。[design.md:114](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:114) は「findings §2 に従う」「参照実装」としていますが、`/clear` 後の実装者が `render_image(return_depth=True, distort=True)` を選ぶ余地が残ります。これは過去に RGB+ED 非対応で問題化した経路です。  
修正提案: 設計書に方式Aの擬似コードを入れる。最低限、`z_cam=(means @ R.T + t)[:,2]`、`colors=z_cam` 3ch複製、`rasterization(render_mode="RGB", with_ut=True, with_eval3d=True, packed=False, far_plane=1e10)`、`depth=accum/alpha`、`alpha<=1e-6 は NaN` まで固定する。

2. 出力 TOML が「対象外カメラ」をどう扱うか未固定です。[requirements.md:103](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:103) は入力メタデータ保持、[design.md:157](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:157) はカメラごとの出力を述べますが、`--cameras` で処理対象を絞った場合に非対象カメラを保持するかが明示されていません。実装次第で下流用 TOML からカメラが欠落します。  
修正提案: 出力は「入力 TOML の全カメラを元順で含める。受理した対象カメラだけ rotation/translation を置換し、不受理・失敗・非対象カメラは全キーを入力値のままコピー」と固定する。

**低**

なし。
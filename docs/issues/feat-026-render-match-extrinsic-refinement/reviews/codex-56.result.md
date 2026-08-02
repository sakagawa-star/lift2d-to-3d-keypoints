**高**

[criteria.md:18](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:18>)  
M3 と同一入力と書かれていますが、`phase0/data/osaka2/Config_scene.toml` は M3 が使った TOML ではありません。M3 criteria の正は `/home/sakagawa/git/pose2sim/Pose2Sim/20260418_osaka-hosp1/calibration/Config_scene.toml` です。実ファイルも内容が異なり、前者はカメラ構成・K・歪み・外部パラメータが M3 と一致しません。このままだと SIFT 再現・M3 baseline 比較・一致性判定がすべて別条件になります。

修正提案: M3 と同じ絶対パスに固定するか、同一内容のコピーを使うなら sha256 を明記して同一性を固定してください。少なくとも criteria 上で「M3 と同一」の比較対象は M3 criteria §2 の TOML に揃える必要があります。

[criteria.md:73](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:73>)  
LoFTR の採用設定選択が一意に閉じていません。設定には `outdoor` / `indoor_new` の2重みがありますが、同点時の規則は confidence 閾値しか見ていません。同じ `min N`・同じ合計 N・同じ confidence で重みだけ違う場合、実験後にどちらをフェーズ2へ進めるか動かせます。

修正提案: 最後のタイブレークに重み順を固定してください。例: `indoor_new` 優先、または `outdoor` 優先。あるいは完全同点なら両方をフェーズ2へ進める、と明記してください。

[criteria.md:35](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:35>) / [criteria.md:65](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:65>)  
縮小画像から原寸座標への逆写像が数値で固定されていません。特に MASt3R の `load_images(size=512)` は実装依存のリサイズ・形状調整が入るため、「原寸へ逆写像」「目視重畳1枚」だけでは、座標変換ミスがあってもフェーズ1/2の N や PnP 結果を汚染します。

修正提案: LoFTR fallback と MASt3R それぞれについて、逆写像式と検証基準を数値固定してください。例: 既知点の round-trip 誤差 `≤0.5px`、全マッチ座標の原寸範囲内率 `100%`、MASt3R の実際の tensor shape / true_shape からの変換式を記録、など。

**中**

[criteria.md:28](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:28>)  
LoFTR の OOM fallback が「発生した場合のみ 1280x720」とされていますが、適用単位が未定義です。カメラ単位・重み単位で混在すると、設定間比較が解像度差を含んでしまいます。

修正提案: 「full-res が1回でも OOM したら LoFTR 全設定・全カメラを 1280x720 に統一」など、fallback の適用範囲を固定してください。

**低**

該当なし。
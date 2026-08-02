再レビュー結果: 前回指摘4件はいずれも解消されています。新たな致命的問題は見つかりません。

- 高: なし
- 中: なし
- 低: なし

確認点:

- 目的文の過剰主張は、[requirements.md:11](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:11) で「真値精度や20点方式超えは主張しない」と明示され、M3/M4 の限定と整合しました。
- 入力点の一意性は、[requirements.md:42](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:42) と [design.md:100](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:100) 以降で、3D/2D CSV の重複・欠損・非有限値・画像範囲外まで閉じています。
- 方式Aは [design.md:118](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:118) 以降に擬似コードが入り、`RGB+ED` / `return_depth=True` 禁止も明示されました。
- 非対象カメラの出力TOML扱いは、[requirements.md:103](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:103) と [design.md:177](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:177) で「入力TOMLの全カメラを元順で含める」と固定されています。
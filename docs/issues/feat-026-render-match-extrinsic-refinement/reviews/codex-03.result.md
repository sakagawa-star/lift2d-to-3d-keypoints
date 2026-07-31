再レビューしました。前回の高1（silent failure 保証）は解消、前回の高2（平面退化）は方針としては解消していますが、診断対象の集合にまだ致命的な抜けがあります。前回の中1（robust LM の実装スコープ）は新規実装と明記されましたが、SciPy での実装方法の記述が不正確です。

**高**
- 退化診断を RANSAC 前の候補集合 `J` で評価しているため、外れ値が退化を隠せます。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:232) は `Cov({X^(j)}_{j∈J})` で退化診断していますが、PnP の最終的な支持集合は [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:254) の `I ⊆ J` です。`J` に散らばった外れ値が混じると `λ3/λ1` や凸包面積が良く見え、実際の RANSAC inlier `I` が単一平面でも通過します。これは前回の平面退化対策をすり抜けます。  
  修正提案: 退化診断は必ず `I` に対して実施してください。順序は `Stage 4 → lift → PnP-RANSACでI決定 → I上で退化診断 → failなら棄却 → robust最適化` が安全です。事前診断を残す場合も、最終受理条件は `I` ベースの診断にしてください。

**中**
- 「ロバスト LM」を `scipy.optimize.least_squares(loss='huber')` で実装可能とする記述が不正確です。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:260) は Huber 付き LM と読めますが、SciPy 1.15.3 では `method='lm'` は `loss='linear'` のみ対応です。ローカル環境でも `method='lm', loss='huber'` は `ValueError` になります。  
  修正提案: 表現を「ロバスト非線形最小二乗」に変更し、実装は `least_squares(method='trf', loss='huber' or 'soft_l1')` と明記してください。LM を使うならロバスト損失なしにする必要があります。

**低**
- 致命的な低重要度指摘はありません。
---

## Claude Code の対応方針（2026-07-30）

- 高1（退化診断の対象集合が J で外れ値が退化を隠せる）: Stage 6 の処理順序を「(6a) PnP-RANSAC で I 決定 → (6b) I 上で退化診断（fail なら棄却）→ (6c) I 上でロバスト最適化」に変更。診断式・条件2/3・変数表もすべて I ベースに修正し、J で診断すると外れ値が退化を隠蔽する旨を理由として明記
- 中1（least_squares(loss='huber') と LM の併用不可）: 「ロバスト非線形最小二乗」に表現を変更し、実装を `least_squares(method='trf', loss='huber' or 'soft_l1')` と明記。`method='lm'` は `loss='linear'` のみ対応のため併用不可であることを注記。データフロー図・実装スコープ表の「LM」表現も修正

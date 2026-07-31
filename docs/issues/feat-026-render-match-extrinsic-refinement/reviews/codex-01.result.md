`docs/REVIEW_CRITERIA.md` に沿って、致命度の高い成立性・数式・前提・検証計画だけを見ました。低重要度の指摘はありません。

**高**
- 初期化の「5〜6点 + RANSAC流用」前提が成り立っていません。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:119) は 5〜6点で外れ値検出を期待していますが、既存実装は 4〜5点では RANSAC なしの SQPnP、6点以上でのみ RANSAC です。参照: [estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:333)  
  修正提案: 「5点なら外れ値なし前提」と明記するか、外れ値耐性を要件にするなら `n0 >= 6` ではなく実運用上 `8〜10点程度 + 非退化配置 + RANSAC inlier 条件` を定義してください。

- 変位ゲート `||u_q - u_r|| < τ_px` と「初期角度誤差 数度」が矛盾しています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:122) では初期角度を数度許容していますが、[pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:194) では `τ_px < 格子ピッチ/2` としています。格子ピッチ 30〜60px なら `τ_px < 15〜30px`、一方 `f=1000px` で 2〜3度の回転誤差は約 35〜52px ずれます。正しい対応も落ちます。  
  修正提案: `fθ + fB/Z < τ_px < 格子ピッチ/2` が同時に満たせることを成立条件に追加し、満たせない場合は初回だけ非檻領域マスク・緩いゲート・段階的 tightening などに変更してください。

- 深度 `D` の定義と無効画素処理が不足しています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:134) は `D = Σ z_i α_i T_i` としていますが、gsplat の `RGB+ED` は expected depth、つまり alpha 正規化済みです。低 alpha 画素では深度が不安定で、既存設計でも alpha 判定を先に置く必要が明記されています。参照: [render_keypoints.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_keypoints.py:169), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-013-keypoint-overlay-render/design.md:200)  
  修正提案: `D = Σwz / Σw`、`A = Σw` を明示し、Stage 4 に `A(u_r) > τ_alpha`、finite depth、near/far 範囲チェックを追加してください。深度分散も alpha 有効画素だけで評価すべきです。

- `N >= 30` だけでは PnP の成立条件として弱すぎます。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:196) は生存対応数だけを失敗条件にしていますが、非檻領域を壁・床・什器に頼る前提では、対応点が単一平面・細い領域・同一奥行きに偏る可能性があります。その場合、点数が多くても PnP は退化・不安定です。  
  修正提案: Stage 6 前に 3D 点群の共分散固有値、深度レンジ、画像上の分布、複数面への分散を診断し、退化時は失敗扱いにしてください。実験 A でも「マッチ数」ではなくこの幾何条件を判定対象に入れるべきです。

**中**
- Stage 2 の方式が既存実装と食い違っています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:148) は「ピンホール描画後に自前ワープ」ですが、既存 `render_keypoints.py --distort` は gsplat の 3DGUT 経路で歪み付き直接レンダです。参照: [render_keypoints.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_keypoints.py:188)  
  修正提案: post-warp 方式で行くなら新規関数として RGB/depth/alpha を同じ写像でワープする仕様にしてください。3DGUT を使うなら Stage 2 の式と「NN 深度ワープ」記述を削除し、直接レンダ前提に統一してください。

- 歪み係数の対象が既存仕様より狭く、広角前提と衝突します。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:83) は 4〜5係数だけですが、既存 K 既知モードは 4/5/8/12/14 を受けます。参照: [estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:297)  
  修正提案: 4/5係数限定を要求にするなら 8以上を明確に拒否してください。既存 wide 系を使うなら OpenCV rational model の式、`undistortPoints`、gsplat 側対応可否を design で揃えてください。

- 実験 A の合格条件「非檻領域で数十点」は green light として弱いです。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:323) はマッチ数中心ですが、誤対応が幾何的に整合して間違った PnP を作るケースを検出できません。  
  修正提案: 既存20点ポーズ周辺に既知摂動を入れ、マッチング → lift → PnP で元ポーズへ戻るかを測ってください。判定は `match数`、`inlier率`、`RMSE`、`ポーズ改善量`、`点群退化指標` のセットにするべきです。
---

## Claude Code の対応方針（2026-07-30）

- 高1（5〜6点+RANSAC不整合）: 初期化の目的を「大雑把な配置（収束域に入ること）」と再定義。5〜6点・外れ値なし前提を明記し、初期値不良は下流の失敗判定（N<30）で検出する設計に変更（点数は増やさない。ユーザー判断）
- 高2（変位ゲート矛盾）: τ_px を反復依存 τ_px^(k) に変更。成立条件式を明記し、k=0 は緩いゲート＋RANSAC、k≥1 で格子ピッチ/2 未満に段階的引き締め
- 高3（深度定義・無効画素）: D をα正規化済み期待深度（Σwz/Σw）に修正、A（αマップ）を出力に追加。Stage 4 にゲート(iv)（τ_α・finite・near/far）を追加、深度分散は有効画素のみで評価
- 高4（N≥30不足）: Stage 6 に (6a) 幾何退化診断（共分散固有値・深度レンジ・画像分布）を追加。閾値割れは失敗扱い、診断値を最終出力に含める
- 中1（Stage 2 実装食い違い）: 既存実装を確認し（distort=True と return_depth=True は併用可能）、3DGUT 直接レンダ方式に全面差し替え。post-warp・NN ワープ記述を削除
- 中2（歪み係数範囲）: δ を 4/5/8 対応（rational model の式を op-1 に追加）、12/14 は明示的に対象外と定義
- 中3（実験A合格条件）: A-2（既知摂動からの end-to-end 回復検証）を追加し、判定指標をマッチ数・インライア率・RMSE・ポーズ回復量・退化診断値のセットに変更

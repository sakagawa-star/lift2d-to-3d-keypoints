再レビュー結果です。前回指摘の大半は反映されていますが、致命度の高い点がまだ 2 件あります。

**前回指摘の解消状況**
- 高1 初期化 5〜6点/RANSAC 不整合: 方針変更としては整合。5点時に外れ値検出しないことも明記済み。ただし下記「初期値不良は静かに誤らない」保証が未成立。
- 高2 変位ゲートと初期角度誤差の矛盾: 解消。`τ_px^(k)` の段階的引き締めと成立条件が追加済み。
- 高3 深度/alpha/ED 定義不足: 解消。`D=Σwz/A`、`A`、alpha gate、finite/depth range gate が追加済み。
- 高4 PnP 退化条件: 部分解消。ただし planar 退化の扱いがまだ危険。
- 中1 Stage 2 の post-warp / 3DGUT 不整合: 解消。3DGUT 直接レンダに統一済み。
- 中2 歪み係数 8 係数対応: ほぼ解消。本文は 4/5/8 対応、12/14 対象外に更新済み。
- 中3 実験 A がマッチ数だけ: 解消。摂動回復 end-to-end 検証が追加済み。

**高**
- 初期値不良が「静かに誤った結果にならない」という保証はまだ成立していません。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:129) は、収束域外なら Stage 4 の `N < 30` に落ちるため silent failure はない、としています。しかし [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:204) では初回 `τ_px^(0)` を 50〜100px 程度に緩めるため、初期値不良でも誤対応が 30 点以上残る可能性があります。周期構造・似た什器・平面壁面の誤対応が RANSAC コンセンサスを作るケースも残ります。  
  修正提案: line 129 の「静かに誤った結果が出ることはない」は削除し、「多くの場合は失敗判定に落ちるが保証ではない」に弱めてください。実装要件として、最終受理条件に `N_in`、RMSE、退化診断、最終レンダ重畳確認、可能なら複数初期摂動から同一解へ収束すること、を追加するのが妥当です。

- PnP 退化診断が planar ケースを許容しすぎています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:233) は `λ3/λ1` で共面退化を検出しつつ、「平面配置でも可解なので警告に留めてもよい」としています。しかし本手法は [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:331) で非檻領域、つまり壁・床・什器に頼る前提なので、単一平面にマッチが偏るのは現実的な主失敗パターンです。点数・深度レンジ・画像面積が十分でも、単一平面 PnP は外部パラメータ精度評価として危険です。  
  修正提案: `λ3/λ1 < τ_vol` は原則 fail にしてください。許容するなら planar 専用モードとして、IPPE/solvePnPGeneric 等で複数解を列挙し、解の分離量・最終レンダ再一致・ホールドアウト点の再投影で曖昧性を棄却する条件を明記すべきです。

**中**
- Stage 6 の「既存 phase0 資産と同型」は実装スコープを過小評価しています。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:247) はロバスト損失 `ρ` を含む LM を定義していますが、既存 K 既知モードは `solvePnPRansac` 後に `solvePnP` 精密化で、Huber 等のロバスト LM はありません。参照: [estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:333)  
  修正提案: robust LM を新規実装スコープに入れるか、Stage 6 の式を既存実装相当へ落としてください。要求化するなら前者が自然です。

**低**
- 致命的な低重要度指摘はありません。
---

## Claude Code の対応方針（2026-07-30）

- 高1（silent failure 保証未成立）: 「静かに誤った結果が出ることはない」を撤回し「多くの場合は失敗判定に落ちるが保証ではない」に修正。Stage 7 に最終受理条件（N_in・RMSE・退化診断・レンダ重畳確認・複数初期摂動からの同一解収束）を新設し、silent failure 排除はそちらで担保する構成に変更
- 高2（planar 退化の許容しすぎ）: λ_3/λ_1 < τ_vol を原則 fail に変更し、理由（非檻領域頼み前提では単一平面偏りが主失敗パターン）を明記。planar 専用モード（IPPE/solvePnPGeneric による複数解列挙＋曖昧性棄却）は本スコープ外と定義
- 中1（Stage 6 スコープ過小評価）: ロバスト損失付き LM が既存資産に存在しないことを明記し、新規実装スコープ（least_squares(loss='huber')）として定義。実装スコープ表も更新

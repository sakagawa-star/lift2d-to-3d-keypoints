4係数への絞り込み方針自体は、op-1 の式・δ 定義・I/O サマリには概ね反映されています。致命的な数式矛盾は見当たりません。

**高**
- なし。

**中**
- 4係数限定の「入口エラー」が、実装フロー上どこで保証されるか未定義です。  
  [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:83) では 5/8/12/14 を入口で明示エラーとしていますが、Stage 1 は既存 `estimate_camera_params.py --intrinsic-toml` をそのまま使う記述です。参照: [pipeline.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:129)  
  既存 K既知モードは 4/5/8/12/14 を受理し、`distortions_to_gsplat` も 4/5/8 を受理します。既存資産を変更しない方針なら、feat-026 側の入口ラッパーで `len(δ) == 4` を検証してから Stage 1/2 に渡す、と明記しないと、スコープ外係数が静かに通ります。  
  修正提案: Stage 0「入力検証」を追加し、TOML 読み込み直後に `distortions` 長が 4 以外なら終了コード1で明示エラー、と定義してください。実装スコープにも「既存資産は変更せず、feat-026入口で検証」と追記すると整合します。

**低**
- 致命的な低重要度指摘はありません。
---

## Claude Code の対応方針（2026-07-31）

- 中1（入口エラーの保証場所が未定義）: Stage 0「入力検証」を新設。TOML 読み込み直後に len(δ)≠4 なら明示エラー・終了コード1と定義し、既存資産は変更せず feat-026 入口で検証する旨を実装スコープ表にも追記

# codex-01 レビュー結果

- 日付: 2026-11-25
- 対象: `docs/issues/feat-041-intrinsic-loop-render-match/requirements.md`、`docs/issues/feat-041-intrinsic-loop-render-match/design.md`（背景として README.md）
- ストリーム: `rev-lift2d-feat-041`
- フェーズ: 反復（初回レビュー）
- 指摘数: 高 2 / 中 2 / 低 0
- トークン実測（total_token_usage 累積）: input 421,218（cached 342,016）/ output 2,622（reasoning 465）/ total 423,840
- rollout: `~/.codex/sessions/2026/11/25/rollout-2026-11-25T09-51-47-01a2102f-f427-7e61-b0de-ec186e8d7d6a.jsonl`

## 指摘

### 1. 重要度：高 — 上書き拒否時にも既存レポートを破壊する設計

design.md §4.2.7 は終了コード 2 以外で必ずレポートを書く。既存出力を検出した場合も終了コード 1 なので、既存の `loop_report.txt` を上書きする。requirements.md NFR-004（データ保全）と矛盾する。

修正案: 出力先の事前検査で拒否した場合は、レポートも書かず標準エラーだけに出力すると明記する。

### 2. 重要度：高 — `.tmp` への画像保存が失敗する

design.md §4.1.3 は `.tmp` に `cv2.imwrite` してから置換する手順。末尾が `.tmp` のファイル名では画像形式を選択できず、前処理が失敗する。ローカルの OpenCV 4.13 でも `cv2.haveImageWriter("image.png.tmp")` は False だった。

修正案: 一時ファイルを `{camera}.tmp.png` とするか、`cv2.imencode(".png", ...)` の結果を一時ファイルへ書く方式にする。

### 3. 重要度：中 — パラメータが不変でも未収束になる

design.md §4.2.5 の Δ_intr には `undistortPoints` の逆変換誤差が混入する。定義どおりの 25 格子点で、旧新とも f=1280, cx=960, cy=540, k1=0.2, k2=0.1 としたところ、同じパラメータなのに Δ_intr ≈ 0.620px となった。すべて許容範囲内だが、収束条件 0.5px を満たせない。

修正案: 同じ正規化座標を旧・新の両パラメータで投影し、その差を測る。逆変換精度も確認し、歪みありの同一パラメータを収束判定テストに追加する。

### 4. 重要度：中 — ゲート通過点が少ない場合、規定の失敗処理に到達しない

design.md §4.2.3 には、ゲート通過点数の事前検査がない。元のマッチが存在しても、通過点が 0〜3 点になると `solvePnPRansac` は False を返す前に `cv2.error` を送出する。設計された終了コード 3・理由付きレポートを保証できない。

修正案: 深度リフト前に `keep.sum() < N_MIN_INTR` を検査し、`LoopFailure` にする。OpenCV 例外を規定の失敗へ変換する方針も明記する。

## 備考（Codex 記載）

確認は文書・既存コードの照合と CPU での小規模再現まで。GPU・実データ処理は実行していない。

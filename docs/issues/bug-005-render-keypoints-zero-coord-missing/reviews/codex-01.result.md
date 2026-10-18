# Codex レビュー結果 codex-01

- 日付: 2026-10-18
- 対象ファイル: `docs/issues/bug-005-render-keypoints-zero-coord-missing/investigation.md`, `docs/issues/feat-021-render-keypoints-flexible-markers/requirements.md`, `docs/issues/feat-021-render-keypoints-flexible-markers/design.md`
- ストリーム: `rev-lift2d-bug-005`（新規起動）
- フェーズ: 反復（初回レビュー）
- 指摘数: 高 0 / 中 1 / 低 0
- トークン実測（total_token_usage 累積）: input 211,416（cached 171,008）/ output 1,724（reasoning 405）/ total 213,140
- rollout: `~/.codex/sessions/2026/10/18/rollout-2026-10-18T16-58-33-01a14e05-039a-7e23-83aa-041bac98cd67.jsonl`

## 依頼文

docs/REVIEW_CRITERIA.md および docs/BUGFIX_STANDARD.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/bug-005-render-keypoints-zero-coord-missing/investigation.md 。requirements.md / design.md を変更した場合はそれらもレビュー対象に含めること（本案件では docs/issues/feat-021-render-keypoints-flexible-markers/requirements.md と docs/issues/feat-021-render-keypoints-flexible-markers/design.md を変更済み。git diff で確認できる）。案件の概要と再現手順は docs/issues/bug-005-render-keypoints-zero-coord-missing/README.md 、変更対象コードは phase4/render_keypoints.py の extract_keypoints 。

## 指摘

### 中-1: 新しい無効判定と、feat-021 の設計の説明・互換性要件が矛盾している

- 対象: feat-021 `design.md` 78行目付近（戻り値 `valid` の説明）、feat-021 `requirements.md` 40行目付近（FR-001 受け入れ基準2）
- 内容: 設計書の擬似コードには (0,0,0) の除外が追加されたが、戻り値 `valid` の説明は「既存と同じ規則（residual >= 0 かつ座標に NaN なし）」のまま。FR-001 受け入れ基準2は旧実装との座標・valid・描画結果の一致を要求しており、座標 (0,0,0)・residual 0 の入力では今回の修正と同時に満たせない。参照箇所によって実装・受け入れ判定が変わる
- 修正提案: `valid` の説明にも原点座標の除外を追記し、旧実装との一致要件には「bug-005 で無効化する (0,0,0) のサンプルを除く」と明記する。investigation.md 1.3 の文書変更内容にも含める

ほかに重大な問題はない（文書と実装の照合のみ。テスト・GPU描画は未実行）。

## 対応

- 中-1: feat-021 `design.md` の `valid` の説明と、`requirements.md` FR-001 受け入れ基準 2(b) に bug-005 の変更を追記した。investigation.md 1.3「修正が設計書に沿っているか」に変更 3・4 として記録した
- 追加対応（Claude Code が同種の見落としを横断確認して発見）: feat-013 の文書に「『全成分0』は判定基準にしない」という過去の設計判断があったため、その見直し理由と、変更しない既存文書の一覧を investigation.md 1.3 に追記した

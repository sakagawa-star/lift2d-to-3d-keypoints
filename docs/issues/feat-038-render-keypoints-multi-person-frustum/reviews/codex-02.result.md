# Codex レビュー結果 codex-02

- 日付: 2026-10-20
- 対象: `docs/issues/feat-038-render-keypoints-multi-person-frustum/requirements.md`, `docs/issues/feat-038-render-keypoints-multi-person-frustum/design.md`
- ストリーム: `rev-lift2d-feat-038`
- フェーズ: 全文ゲート（`/new` 実施後の新しい会話）
- 指摘数: 高 0 / 中 2 / 低 0
- トークン実測: total 326,691（input 324,563 うち cached 246,784 / output 2,128 うち reasoning 402）
- rollout: `~/.codex/sessions/2026/10/20/rollout-2026-10-20T15-15-50-01a157f3-aeaf-7762-9eb0-31448187d1b1.jsonl`

## 依頼文

docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/feat-038-render-keypoints-multi-person-frustum/requirements.md docs/issues/feat-038-render-keypoints-multi-person-frustum/design.md 。本ドキュメントは対話ベースのレビューを経ている。今回は最終ゲートとして、文書全体の整合性（セクション間の矛盾、参照切れ、抜け）を含めて全文を確認して。

## 回答

[AGENTS.md適用]

両文書の全文と参照先・既存コードを確認しました。最終ゲートで解消すべき指摘は中：2件です。

1. 【中】1人モードの頭部マーカー不足メッセージが要求と設計で不一致
   - requirements.md:63 は、全員指定時のエラーに人物番号・ファイルパス・不足マーカー名を要求している。一方、design.md:133 は、1人モードでは人物番号・パスを含まない既存メッセージを維持する。1ファイルで `--fps-person all` を指定すると要求を満たさない
   - 修正案: 既存動作を維持する方針なら、FR-003 に「人物番号・パスの表示は複数人モードのみ。1人モードは既存形式」と明記する

2. 【中】既知マーカー0件の場合、全員分の不足情報を集約できない
   - design.md:157 は不足人物をすべて表示するとしているが、検査位置は既存の読み込みループの後。このループには既知マーカー0件で即時終了する処理がある（phase4/render_keypoints.py:1197）。例えば P1 が既知マーカー0件、P2 が Head 不足の場合、設計どおりでは全員分の頭部不足情報を表示できない
   - 修正案: FR-003 と設計4.3に既存入力エラーの優先順位を明記し、既知マーカー0件による早期終了を集約対象外とするか、全員指定時に検査結果を集約する設計へ変更する。該当ケースのテストも追加する

これ以外に、重大な矛盾・参照切れ・実装を妨げる抜けは見つかりませんでした。

## 対応

1. requirements.md FR-003 に「複数人モードのメッセージは人物番号・パス付き、1人モードは既存形式のまま（FR-004 により `--fps-person 1` と同一）」を明記。design.md 6章に T-13b（1人モード＋`all` の stderr が `--fps-person 1` と一致）を追加
2. 既存の読み込みループ内の検査（読み込み失敗・フレーム番号重複・既知マーカー0個）は最初の1件で即時終了する既存動作のまま、頭部マーカー不足の集約はループ通過後のみ、と requirements.md FR-003・design.md 4.3 に明記。design.md 6章に T-13a を追加

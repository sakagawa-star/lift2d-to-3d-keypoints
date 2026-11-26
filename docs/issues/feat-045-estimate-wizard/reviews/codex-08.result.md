# codex-08 レビュー結果

- 日付: 2026-11-26
- 対象: docs/issues/feat-045-estimate-wizard/investigation.md（イテレーション1）、requirements.md、design.md
- ストリーム: rev-lift2d-feat-045
- フェーズ: 反復（codex-07 の解消確認。同じ会話）
- 指摘数: 高 1 / 中 0 / 低 0（codex-07 の 2 件のうち 1 件解消、1 件未解消）
- トークン: total_tokens 2,947,959（input 2,933,916、cached 2,736,256、output 14,043。累積）。~/.codex/sessions/2026/11/26/rollout-2026-11-26T15-23-29-01a21685-fc16-7501-87de-6f2f0f0c8604.jsonl

## 判定

1. **未解消（高）** シンボリックリンク経由で所定の保存先から外れる。phase4/data 自体がリポジトリ直下へのリンクなら DATA_REAL がリポジトリ直下になり許可される。EIL が data/10F を指す場合も通る。修正案: 実体が所定の phase4/data/EIL/ 直下であることを作成直前に検証
2. **解消** 親パスの改行で共用設定YAMLが壊れる

## 対応

1. 同じ系統の指摘（リンクで所定の場所の外に出る）が codex-07・08 と 2 回続いたため、行き詰まり検出（CLAUDE.md）に従い、リンクの検査を重ねる方針を取り下げた。ユーザーに確認し（2026-11-26）、「phase4/data と phase4/data/EIL は通常のディレクトリである前提とし、リンクの検査はしない」に決定。requirements FR-001・design §4.4 `job_place_error`（改行・前後の空白の検査だけ残す）・investigation.md を変更し、テスト No.29 を縮小、No.30 を削除

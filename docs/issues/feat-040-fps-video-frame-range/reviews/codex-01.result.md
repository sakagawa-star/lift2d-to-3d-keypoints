# Codex レビュー結果 codex-01

- 日付: 2026-10-21
- 対象: `docs/issues/feat-040-fps-video-frame-range/requirements.md`, `docs/issues/feat-040-fps-video-frame-range/design.md`
- ストリーム: `rev-lift2d-feat-040`
- フェーズ: 反復（初回レビュー）
- 指摘数: 高 1 / 中 0 / 低 0
- トークン実測: total 219,695（input 218,457 うち cached 162,816 / output 1,238 うち reasoning 88）
- rollout: `~/.codex/sessions/2026/10/21/rollout-2026-10-21T10-20-53-01a15c0c-03ce-7263-85f4-2274ce153091.jsonl`

## 依頼文

docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/feat-040-fps-video-frame-range/requirements.md docs/issues/feat-040-fps-video-frame-range/design.md 。

## 回答

[AGENTS.md適用]

重大な指摘は1件です。

- 重要度: 高 — 再実行時の上書き防止要件を、設計どおりでは満たせません
  - 対象: requirements.md:112、design.md:174
  - 「範囲を変えて同じ --output で再実行すると manifest 不一致でエラーになる」としているが、既存実装は正常終了時に、既定でチャンクディレクトリと manifest を削除する（render_fps_video.py:1298）。そのため、例えば範囲1〜100で正常終了後、101〜200を同じ出力先に指定すると、--overwrite なしでも既存MP4・情報ファイルが上書きされる
  - 修正提案: 既存動作を維持する方針なら、エラーになる条件を「チャンクと manifest が残っており、実際の対象範囲が異なる場合」に限定し、正常終了後の再実行では上書きされることを両文書に明記する。常に上書きを防止する要件なら、永続的な範囲情報による照合設計が必要。あわせて、manifest の残存・削除それぞれの再実行テストを追加する

## 対応

- 既存動作を維持する方針を採用。requirements.md 4章「信頼性」を、manifest 残存時はエラー・チャンクディレクトリ削除済み（正常終了後）は上書き（範囲指定なしと同じ既存の挙動）・`--output` 省略時は範囲ごとに別名、の3項目に書き換え。design.md 4.6・4.8 を同様に修正
- テスト: T-21（manifest 残存で範囲を変えると戻り値 1、描画前に終了）を追加。manifest 削除済みの再実行は範囲指定に依存しない既存処理のため T-22 として「テスト追加なし」と理由を明記

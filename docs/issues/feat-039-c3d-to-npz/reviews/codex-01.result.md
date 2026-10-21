# Codex レビュー結果 codex-01

- 日付: 2026-10-21
- 対象: `docs/issues/feat-039-c3d-to-npz/requirements.md`, `docs/issues/feat-039-c3d-to-npz/design.md`
- ストリーム: `rev-lift2d-feat-039`
- フェーズ: 反復（初回レビュー）
- 指摘数: 高 1 / 中 1 / 低 0
- トークン実測: total 213,669（input 211,993 うち cached 177,024 / output 1,676 うち reasoning 112）
- rollout: `~/.codex/sessions/2026/10/21/rollout-2026-10-21T09-45-01-01a15beb-2cc8-74b2-9c8d-2dcc94510472.jsonl`

## 依頼文

docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/feat-039-c3d-to-npz/requirements.md docs/issues/feat-039-c3d-to-npz/design.md 。

## 回答

[AGENTS.md適用]

重大な問題を2件確認しました。

1. 重要度: 高 — 途中で切れたC3Dを正常変換し、欠落したNPZで既存出力を上書きできる
   - 対象: design.md:74
   - load_c3d_all_frames は、途中EOFでも例外を送出せず、読み取れたフレームだけ返す。実際に、宣言10フレームのC3Dを途中で切ると、警告のみで3フレームが返ることを確認した。その連番は読み戻し検証にも通るため、設計どおりでは欠落を検出できない。FR-001の全フレーム出力を保証できない
   - 修正案: FR-006に途中EOFの拒否を追加し、設計ではEOF警告をエラーにするか、C3Dの宣言フレーム数と実読込数を照合する。途中で切れた入力で終了コード1となり、既存出力が保持されるテストも必要

2. 重要度: 中 — 出力確定に失敗すると一時ファイルが残る
   - 対象: design.md:125
   - os.replace(tmp_path, out_path) が例外処理の外にある。例えば出力先が既存ディレクトリ result.npz/ の場合、一時ファイルの保存・検証後に置換が失敗し、result.npz.tmp.npz が残る。FR-005の「失敗時は一時ファイルを削除」に違反する
   - 修正案: os.replace まで後始末の対象に含め、失敗時に一時ファイルを削除する。置換失敗時の削除と既存出力の保持を確認するテストを追加する

## 対応

1. requirements.md FR-006 に 7（C3D が途中で切れている）を追加。design.md 4.2 に手順 3（py-c3d `Reader.frame_count` の宣言フレーム数と読み込みフレーム数の照合）を追加し、エラー一覧に追記。テスト T-25 を追加
2. design.md 4.4 で `os.replace` を try の中に移し、失敗時も一時ファイルを削除。requirements.md FR-005 に確定失敗時の扱いを追記。テスト T-26 を追加

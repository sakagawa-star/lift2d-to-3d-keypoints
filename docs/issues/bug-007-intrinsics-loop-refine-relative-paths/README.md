# bug-007: estimate_intrinsics_loop.py の最終精緻化で相対パスの基準がずれる

## ステータス

Closed（2026-11-25 起票、2026-11-26 完了。再現を確認し investigation.md に修正計画を保存。Codex レビュー codex-01〜07（依頼文に「致命的な点のみ」の前提。全文ゲートは 2 回）で収束、人レビュー通過。実装は Sonnet サブエージェント委任、全体 743 passed / 1 skipped。手動テスト合格（phase4/ をカレントディレクトリにして相対パスで完走、median 0.671px・合格））

## 概要

`phase4/estimate_intrinsics_loop.py`（feat-041）は、最終精緻化で `phase4/refine_extrinsics.py` を subprocess で呼ぶ（feat-041 design.md §4.2.6）。このとき `cwd=ROOT`（プロジェクトルート）で起動し、`--toml`（ループ結果 TOML）、`--ply`、`--images-dir`、`--points-3d`、`--points-2d`、`--out-toml`、`--out-report`、`--tmp-dir` に、利用者が指定した相対パス（またはそれから作ったパス）をそのまま渡す。

そのため、プロジェクトルート以外のディレクトリから相対パスで実行すると、次のことが起きる。

- 親プロセスでは読めた入力が、子プロセスでは別の場所を指して見つからない
- 相対指定した `--out-dir` / `--tmp-dir` の配下に書くはずの最終 TOML・レポート・中間ファイルが、子プロセスではプロジェクトルート基準の別の場所を指す

本案件の前から CLI でも起きる問題で、feat-042（`--config` 対応）とは関係しない。

## 再現手順（2026-11-26 に実行して再現を確認。詳細と実際のコマンドは investigation.md）

1. プロジェクトルート以外のディレクトリ（例: `phase4/`）に移動する
2. そこからの相対パスで `estimate_intrinsics_loop.py` を実行する
3. ループが収束して最終精緻化に進むと、`refine_extrinsics.py` が入力を見つけられず異常終了する（終了コード 1）、または別の場所に書き込む

## 運用上の回避（修正まで。2026-11-26 の修正で不要になった）

プロジェクトルートで実行する（README.md・design.md のコマンド例どおり）。または、すべてのパスを絶対パスで指定する。

## 関連

- feat-042 requirements.md §5.1、reviews/codex-01.result.md
- 修正案（codex-01）: 子プロセスに渡すパスを、呼び出し元のディレクトリを基準に絶対化する

# bug-007: estimate_intrinsics_loop.py の最終精緻化で相対パスの基準がずれる

## ステータス

Open（2026-11-25 起票。feat-042 の Codex レビュー codex-01 指摘1（高）で発見。feat-042 では範囲外とした）

## 概要

`phase4/estimate_intrinsics_loop.py`（feat-041）は、最終精緻化で `phase4/refine_extrinsics.py` を subprocess で呼ぶ（feat-041 design.md §4.2.6）。このとき `cwd=ROOT`（プロジェクトルート）で起動し、`--toml`（ループ結果 TOML）、`--ply`、`--images-dir`、`--points-3d`、`--points-2d`、`--out-toml`、`--out-report`、`--tmp-dir` に、利用者が指定した相対パス（またはそれから作ったパス）をそのまま渡す。

そのため、プロジェクトルート以外のディレクトリから相対パスで実行すると、次のことが起きる。

- 親プロセスでは読めた入力が、子プロセスでは別の場所を指して見つからない
- 相対指定した `--out-dir` / `--tmp-dir` の配下に書くはずの最終 TOML・レポート・中間ファイルが、子プロセスではプロジェクトルート基準の別の場所を指す

本案件の前から CLI でも起きる問題で、feat-042（`--config` 対応）とは関係しない。

## 再現手順（机上。未実行）

1. プロジェクトルート以外のディレクトリ（例: `phase4/`）に移動する
2. そこからの相対パスで `estimate_intrinsics_loop.py` を実行する
3. ループが収束して最終精緻化に進むと、`refine_extrinsics.py` が入力を見つけられず異常終了する（終了コード 1）、または別の場所に書き込む

## 運用上の回避（修正まで）

プロジェクトルートで実行する（README.md・design.md のコマンド例どおり）。または、すべてのパスを絶対パスで指定する。

## 関連

- feat-042 requirements.md §5.1、reviews/codex-01.result.md
- 修正案（codex-01）: 子プロセスに渡すパスを、呼び出し元のディレクトリを基準に絶対化する

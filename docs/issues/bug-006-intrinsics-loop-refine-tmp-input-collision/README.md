# bug-006: estimate_intrinsics_loop.py の最終精緻化の一時ファイルが入力 TOML を上書きし得る

## ステータス

Closed（2026-11-25 完了。修正: --out-dir / --tmp-dir を存在しない新規パスとして mkdir(exist_ok=False) で作成、カメラ名のパス区切りを拒否。Codex レビュー codex-01〜06（途中で「書き込む前にパスを検査する」案を行き詰まり検出により取り下げ）、テスト 674 passed / 1 skipped、手動テスト合格。起票: feat-041 イテレーション2 の Codex レビュー codex-23 指摘1（高））

## 概要

`phase4/estimate_intrinsics_loop.py`（feat-041）は、最終精緻化で `phase4/refine_extrinsics.py` を subprocess で呼ぶ（feat-041 design.md §4.2.6）。`refine_extrinsics.py` は `--out-toml` に書く前に、一時ファイル `{camera}_final.toml.tmp`（`--out-dir` 配下）に書いてから最終パスへ移動する。

`estimate_intrinsics_loop.py` の出力済み検査（design.md §4.2.1）は出力 8 パスだけを対象にしており、この一時パスは検査しない。そのため、次の場合に入力 TOML が上書きされ得る。

- `--toml` に、`--out-dir` 配下の `{camera}_final.toml.tmp` そのものを指定した場合
- `--out-dir` 配下に既存の `{camera}_final.toml.tmp` があり、それが入力 TOML を指すシンボリックリンクである場合（`--toml` が出力先の外にあっても上書きされる）

これは feat-041 の初版の設計から存在する性質で、feat-041 イテレーション2 の変更が生んだものではない。

## 再現手順（机上。実行での確認は未実施）

1. 有効な初期 TOML を `{out_dir}/{camera}_final.toml.tmp` として置く
2. `estimate_intrinsics_loop.py --toml {out_dir}/{camera}_final.toml.tmp --out-dir {out_dir} ...` を実行する
3. ループが収束して最終精緻化まで進むと、`refine_extrinsics.py` がその一時パスに書き込み、入力 TOML が上書き（移動）される

## 運用上の回避（修正まで）

- `--out-dir` には、入力と分離した新規の出力ディレクトリを指定する（既存ファイルの無いディレクトリ）
- `--toml` には `--out-dir` 配下のファイルを指定しない

## 関連

- feat-041 investigation.md イテレーション2 §2.3「今回の修正に含めないこと」
- feat-041 reviews/codex-23.result.md、codex-24.result.md

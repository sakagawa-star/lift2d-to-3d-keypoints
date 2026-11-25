# feat-042: 精緻化ツール 3 本の設定 YAML（--config）対応

## ステータス

Closed（2026-11-25 完了。Codex レビュー codex-01〜20（全文ゲート 5 回目はユーザー指示で「致命的な点のみ指摘」の前提を付けて通過）、テスト 721 passed / 1 skipped、ユーザー手動テスト合格（10F で estimate_intrinsics_loop.py を --config だけで実行し、feat041_out_it2 と同じ結果）。別案件 bug-007 を起票）

## 概要

カメラパラメータを 3DGS で精緻化する次の 3 本のツールに、設定 YAML（`--config`）でオプションをまとめて指定する機能を追加する。

| ツール | 案件 | 現在の引数 |
|---|---|---|
| `phase4/refine_extrinsics.py` | feat-026 | `--toml` `--ply` `--images-dir` `--points-3d` `--points-2d` `--out-toml` `--out-report`（必須）、`--cameras`（複数）、`--overwrite`、`--seed-base`、`--tmp-dir` |
| `phase4/adjust_extrinsics_multiview.py` | feat-035 | `--toml` `--ply` `--images-dir` `--out-toml` `--out-report`（必須）、`--init-toml`、`--init-cameras`（複数）、`--cameras`（複数）、`--overwrite`、`--seed`、`--tmp-dir`、`--fresh-match` |
| `phase4/estimate_intrinsics_loop.py` | feat-041 / bug-006 | `--toml` `--camera` `--ply` `--images-dir` `--points-3d` `--points-2d` `--out-dir`（必須）、`--tmp-dir` |

## 背景

- 3 本とも引数はコマンドラインだけで指定する。パスの多い長いコマンドになる（2026-11-25 ユーザー確認）
- phase4 では `render_keypoints.py`（feat-033）と `render_fps_video.py`（feat-029）が `--config` に対応済み。仕様は次のとおり（参考実装: `render_fps_video.py` の `load_yaml_flat` / `CONFIG_CONVERTERS` / `parse_config_yaml` / `parse_args_with_config`）
  - 優先順位は「CLI > 設定 YAML > 既定値」
  - 簡易フラットパーサーを使う（`key: value` のみ。`phase0/common.py` の `load_yaml_simple` と同仕様。新規ライブラリなし）
  - キーごとの型変換を行う。bool は小文字の `true` / `false` のみ受け付ける
  - 必須項目は「CLI または `--config` で指定」されていればよい

## ヒアリング結果（2026-11-25 確定）

| 項目 | 決定 |
|---|---|
| 基本仕様 | 既存 2 本（feat-029 / feat-033）と同じ: CLI > 設定YAML > 既定値、簡易フラットパーサー、キー名は dest 名、bool は小文字 true/false のみ、未知のキーはエラー、必須項目は CLI か YAML のどちらかにあればよい |
| 複数値（`--cameras`、`--init-cameras`）の書き方 | スペース区切り（例: `cameras: cam01 cam02`） |
| YAML 内の相対パスの基準 | カレントディレクトリ（既存 2 本と同じ。特別な処理なし） |
| 案件の分割 | 1 案件でまとめる（Codex レビューのストリームは `rev-lift2d-feat-042` の 1 本） |
| 内部の subprocess 呼び出し | `estimate_intrinsics_loop.py` → `refine_extrinsics.py` は CLI 引数のまま変えない |

## 関連

- `phase4/render_fps_video.py`（feat-029）、`phase4/render_keypoints.py`（feat-033）の `--config` 実装
- feat-041 design.md §5「設定ファイルは使わない（CLI 引数のみ）」は本案件で変更対象になる

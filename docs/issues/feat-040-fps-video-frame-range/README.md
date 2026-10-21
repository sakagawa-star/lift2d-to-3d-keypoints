# feat-040: render_fps_video.py のフレーム範囲指定（start_frame / end_frame）

## ステータス

Closed（2026-10-21 手動テスト合格・完了処理済み。Codex レビューは codex-01〜03 で収束）

## 概要

`phase4/render_fps_video.py` の MP4 モードで、出力するフレームの範囲を `--start-frame` / `--end-frame`（設定YAMLでは `start_frame` / `end_frame`）で指定できるようにする。

## 背景

- `render_fps_video.py`（feat-027）は MP4 モードで NPZ の全フレームを出力する。フレーム範囲を絞る機能が無い（静止画モードの `--still-range` は PNG 出力専用）
- ユーザーが `render_keypoints.py` と同じ `start_frame` / `end_frame` を設定YAML（`phase4/data/tokyo1/run_fps-p1.yaml`）に書いたところ、未知キーとしてエラー（終了コード2）になった（2026-10-21）
- tokyo 20261006 の NPZ は 54,007 フレーム（約30分）あり、確認用に一部区間だけ動画化したい

## 現状調査メモ（2026-10-21）

- 設定YAMLの有効キー（`CONFIG_CONVERTERS`）: `ply_path`, `npz_path`, `toml`, `camera`, `output`, `preset`, `still_dir`, `dump_poses`, `fps`, `gpu`, `gpus`, `chunk_size`, `crf`, `overwrite`, `keep_chunks`, `still_range`
- 再開機構: チャンクディレクトリ `<output>.chunks` の `manifest.json` に `fid_first` / `fid_last` を含むパラメータを保存し、再実行時に不一致なら `--overwrite` を求めてエラーにする。範囲を変えて同じ出力パスで再実行すると、この検査で止まる
- `render_keypoints.py` の `--start-frame` / `--end-frame`: フレーム番号基準・両端含む・省略時は最小/最大フレーム。範囲内に対象フレームが無ければ終了コード1
- feat-039 で変換した tokyo の NPZ は `frame_ids` が 1〜54007

## ヒアリングで確定した要件

2026-10-21 ヒアリング:

| 項目 | 決定 |
|---|---|
| 番号の基準 | NPZ の `frame_ids`（`render_keypoints.py` の `--start-frame` / `--end-frame`、既存の `--still-range` と同じ。両端を含む） |
| 範囲が frame_ids の外にはみ出したとき | 重なる部分だけ出力する（`render_keypoints.py` と同じ）。重なりが1フレームも無ければエラー |
| 出力 MP4 の既定のファイル名（`output` 省略時） | 範囲を入れる（例: `<NPZ名>_fps_1-900.mp4`）。範囲を変えても別ファイルになり、前の結果や再開用チャンクと衝突しない |
| 対象モード | MP4 モードとポーズダンプモード（`--dump-poses`）で有効。静止画モードは既存の `--still-range` を使い、併用はエラー |

## 関連ドキュメント

- `requirements.md` — 要求仕様書
- `design.md` — 機能設計書
- 関連案件: feat-027（render_fps_video.py）、feat-029（設定YAML）、feat-030（並列レンダリング）、feat-031（info ファイル）、feat-039（C3D → NPZ）

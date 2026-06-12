# feat-012: camera_pose.py カメラ名・出力先のCLIオプション化

## Status: Closed

## 概要

`phase4/camera_pose.py` はカメラオブジェクト名と出力JSONパスがスクリプト内にハードコードされており、対象カメラや blend ファイルが変わるたびにコード編集が必要で使いにくい。カメラ名（必須）と出力パス（任意）をCLIオプションで指定できるようにする。

## 経緯

- 2026-06-12: ハードコード箇所の調査を実施し、ユーザーが以下を決定
  - `--camera`: 必須オプションにする
  - `--output`: 任意。デフォルトは `data/<カメラ名>_poses.json`（カレントディレクトリ基準）
  - フレーム範囲: オプション化しない（`scene.frame_start` 〜 `scene.frame_end` のまま）
  - KIRIモディファイヤ名: オプション化せずモジュール定数（UPPER_SNAKE_CASE）に切り出す
- 作業ツリーに camera_pose.py の未コミット変更（カメラ名を `cam41520554` に書き換えたもの）が存在する。本案件の実装はコミット済みの版をベースとし、この一時変更はオプション化により不要になる

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

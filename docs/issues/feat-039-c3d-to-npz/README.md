# feat-039: C3D → NPZ 変換スクリプト

## ステータス

Closed（2026-10-21 手動テスト合格・完了処理済み。Codex レビューは codex-01〜04 で収束）

## 概要

C3D ファイル（1ファイル = 1人分の3Dキーポイント）を、`phase4/render_fps_video.py` が入力として受け付ける NPZ 形式に変換するスクリプトを新規作成する。`render_fps_video.py` 自体は変更しない。

## 背景

- `render_fps_video.py`（feat-027）の入力は NPZ のみ。位置引数 `npz_path` を `npz_to_c3d.load_npz` で読み、拡張子による形式判別は無い（2026-10-21 確認）
- 既存の変換は NPZ → C3D の一方向（`phase4/npz_to_c3d.py`、feat-018）だけで、C3D → NPZ の変換手段が無い
- Pose2Sim が出力する C3D（例: tokyo 20261006 の `m6_full_person1_0-54006.c3d`）から FPS 動画を作りたい（ユーザー要望 2026-10-21）
- `render_fps_video.py` に C3D 直読みを追加する案もあったが、ユーザーが変換スクリプト案を選択した（2026-10-21）

## 現状調査メモ（2026-10-21）

- NPZ の必須キー（`load_npz`）: `x3d_world`（(F, J, 3) float、world 座標 [m]）、`frame_ids`（(F,) int、**昇順の連番**であること）、`joint_names`（長さ J）
- `render_keypoints.py` の C3D 座標変換 `c3d_to_calib`: C3D raw `(px, py, pz)` [mm] → `(pz, px, py) × 0.001` [m]。`npz_to_c3d.py` はこの逆変換で C3D を書いている
- Pose2Sim 出力の C3D は欠損を residual ではなく座標 (0,0,0) で表す（bug-005 で判明）。`npz_to_c3d.py` 製の C3D は residual = −1 で欠損を表す

## ヒアリングで確定した要件

2026-10-21 ヒアリング:

| 項目 | 決定 |
|---|---|
| 欠損の扱い | 座標が厳密に (0,0,0) の点と residual < 0 の点を、NPZ では NaN にする（bug-005 の `extract_keypoints` と同じ判定） |
| frame_ids | C3D に記録されたフレーム番号をそのまま使う（例: Pose2Sim の 0-54006 なら 0..54006） |
| 書き出す関節 | C3D の全マーカー（`joint_names` = C3D のラベル。既知28マーカーへの絞り込みはしない） |
| frame_ids（追加確認） | 実データで Pose2Sim の C3D フレーム番号が 1〜54007（ファイル名 `0-54006` と 1 ずれ）と判明したうえで、C3D の番号のままとする（2026-10-21 再確認） |
| 出力パス | 既定は入力 C3D と同じディレクトリ・同じ名前で拡張子だけ `.npz`。`--output` で変更できる（`npz_to_c3d.py` と同じ方式） |

## 関連ドキュメント

- `requirements.md` — 要求仕様書
- `design.md` — 機能設計書
- 関連案件: feat-018（NPZ → C3D）、feat-027（render_fps_video.py）、feat-028（filter_npz.py）、bug-005（座標(0,0,0)欠損）

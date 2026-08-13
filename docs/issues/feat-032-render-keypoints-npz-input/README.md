# feat-032: render_keypoints.py の NPZ 入力対応

## ステータス

Closed（2026-08-13 手動テスト合格・完了処理済み）

## 概要

`render_keypoints.py` のキーポイント入力として、C3D に加えて NPZ（リフトアップ済み3Dキーポイント、`x3d_world` world座標メートル）を直接読み込めるようにする。従来は `npz_to_c3d.py` で NPZ → C3D に変換してから渡す必要があった。

## 背景

- NPZ は 2D→3D リフトアップパイプラインの一次成果物であり、`render_fps_video.py`（feat-027）や `filter_npz.py`（feat-028）は NPZ を直接扱う。`render_keypoints.py` だけが C3D 経由の変換を要求しており、検証ワークフローに余計な変換ステップが挟まる
- NPZ の world 座標[m]は `render_keypoints.py` のキャリブ座標と同一であることが feat-018 で確立済み（`c3d_to_calib(world_to_c3d_raw(w)) == w`）

## テストデータ

`phase4/data/session001_f145749_world300_filtered.npz`（300フレーム、22関節、NaNなし、pnp_ok=True は 9/300）

## 決定事項

- **pnp_ok=False のフレームも全フレーム描画する（pnp_ok は無視する）**: ユーザー決定（2026-08-13）。座標が入っていれば pnp_ok に関係なくキーポイントを描画する

## 関連ドキュメント

- `requirements.md` — 要求仕様書
- `design.md` — 機能設計書
- 関連案件: feat-018（NPZ→C3D変換、座標規約の確立）、feat-021（欠損マーカー許容）、feat-027（NPZ直読みFPS動画）

# feat-017: render_keypoints.py 全フレーム対応（連番PNG + MP4）

## Status: Closed（2026-06-14完了。実装・テスト・手動テスト目視確認済み）

## 概要

feat-015/016 で作った `render_keypoints.py` は C3D の**先頭フレーム1枚**のみを描画する。
本案件では C3D の**全フレーム**（または `--start-frame`/`--end-frame` で指定した範囲）の
Halpe26 キーポイントを各フレーム描画し、**連番PNG**と**MP4**で出力する。

カメラは固定のため、背景3DGSレンダリング（RGB・深度マップ・αマップ）は**1回だけ**計算し、
全フレームで使い回す。フレームごとに変わるのはキーポイントの3D座標（投影・可視判定・描画）のみ。

## 前提

- feat-016 の完了（先頭フレーム1枚の投影＋オクルージョン）
- 既存の投影・可視判定・描画ロジック（`project_keypoints` / `compute_keypoint_depth` /
  `compute_visibility` / `draw_overlay`）はフレーム単位で再利用できる

## 範囲

- C3D 全フレーム読み込み（先頭フレーム限定の `load_c3d_first_frame` を全フレーム版に拡張）
- フレーム範囲指定（`--start-frame`/`--end-frame`、C3Dフレーム番号ベース、両端含む）
- 背景3DGS（RGB・深度・α）の1回計算・全フレーム共有
- 各フレームの投影・可視判定・オーバーレイ描画
- 連番PNG出力（`--output-dir`）＋ MP4出力（`--mp4`、fps既定はC3D rate）

### 本案件で扱わないこと

- 複数カメラ一括レンダリング（将来案件）
- キーポイント描画のスタイルカスタマイズ（色・太さ・ラベル）
- dry-run モード

## 関連ドキュメント

- requirements.md / design.md
- 参考: feat-015/016 の design.md（投影・オクルージョン設計）、phase4/render.py（MP4・フレーム範囲の実装パターン）

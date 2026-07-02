# feat-021: render_keypoints.py 欠損マーカー許容（22点C3D対応）

## ステータス

Closed（2026-07-02完了。手動テストで22点C3D実データの完走・描画を確認）

## 概要

`phase4/render_keypoints.py` は Halpe26 の26マーカーが全て C3D に存在することを前提とし、
1つでも欠けると `ValueError` で終了する。しかしリフトアップ推定由来の22点C3D
（`session001_world_22pt_filtered.c3d`: Halpe26 から足先6点を除き、`Spine`/`Thorax` を加えた構成）
をレンダリングしたいという要求があり、現状ではエラーで実行できない。

```
C3DにHalpe26マーカーが不足しています: RBigToe, RSmallToe, RHeel, LBigToe, LSmallToe, LHeel
```

本案件では render_keypoints.py を「欠損マーカー許容」に変更する:

- C3D に存在するマーカーだけを描画し、端点が欠けるボーンはスキップする
- `Spine`/`Thorax` を通る体幹ボーン定義を追加する（22点C3Dで自然な骨格を描く）
- 既存の26点C3D（Halpe26 完備）に対する出力は従来と同一とする

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

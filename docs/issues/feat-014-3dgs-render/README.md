# feat-014: ピンホール3DGSレンダリング（PNG出力、GT比較）

## Status: 中止（feat-015 に作り直し）

> 2026-06-13 中止。同内容を feat-015 として作り直す。本フォルダは残す。


## 概要

feat-013（3DGSレンダリングへのキーポイント重ね描き＋オクルージョン）を分割し、その**第1ステップ**として「指定カメラ視点で3DGS点群（PLY）をピンホールカメラでレンダリングしてPNG1枚を出力する」最小機能を作る。出力PNGをグランドトゥルース（Blender+KIRIで同カメラ視点をレンダリングした `0001_color.png`）と並べて、3DGSレンダリングの正しさ（カメラ位置・向き・見え方）を検証できるようにする。

`phase4/render_keypoints.py` を3DGSレンダリングのみに作り直し、以降のステップ（feat-013 再開）でキーポイント重ね描き・オクルージョンを段階的に追加する。

## 分割の経緯

- feat-013 を一度に作った結果、歪み処理・オクルージョン・座標系・PLYデータの問題が複合し、原因切り分けが難航した
- 「シンプルな機能を一つずつ作り、積み重ねる」（CLAUDE.md 開発方針）に立ち返り、まず3DGSレンダリング単体を確実に作る

## 検証で確立済みの事実（本機能の土台）

- **座標系**: viewmat = `[[R, t],[0,0,0,1]]`（R=Rodrigues(TOML rotation), t=translation、OpenCV world-to-camera）で正しい。キーポイントがGT病室のベッド上に正確に乗ることで確認済み
- **ピンホール古典経路**: `render.py` の `render_frame` と同一（`camera_model="pinhole"`・歪みなし・`with_ut=False`・`packed=True`）。歪みを gsplat のUT経路に乗せると黒い靄・品質劣化が出るため使わない
- **near_plane**: `near_plane=0.01` だとカメラ至近1〜5cmの floater（浮遊点）を描画してしまい黒い靄になる。`near_plane=0.5` でクリップすると病室がGTと一致して鮮明に映る（実機で確認）

## 第1ステップの範囲

- 入力: PLY + TOML + `--camera`（C3D・キーポイントは使わない）
- 処理: ピンホール（歪みなし）gsplatレンダリング、`--near-plane` でfloater除去
- 出力: PNG 1枚
- 目的: GTと並べて3DGSレンダリングの正しさを検証

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

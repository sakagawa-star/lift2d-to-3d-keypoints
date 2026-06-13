# feat-015: ピンホール3DGSレンダリング（PNG出力、GT比較）

## Status: Closed

## 概要

指定したカメラ視点で3DGS点群（PLY）をピンホールカメラでレンダリングしてPNG1枚を出力する。出力PNGをグランドトゥルース（Blender+KIRIで同カメラ視点をレンダリングした `0001_color.png`）と並べて、3DGSレンダリングの正しさ（カメラ位置・向き・構図・floater除去）を目視検証できるようにする。

`phase4/render_keypoints.py` を3DGSレンダリングのみに作り直す。人体キーポイント重ね描き・オクルージョンは feat-016 で扱う。

## 経緯

feat-013（キーポイント重ね描き＋オクルージョンを一度に実装しようとした案件）と feat-014（その分割案）を中止し、機能を **feat-015（3DGSレンダリング）→ feat-016（オクルージョン）** の段階に分けて作り直す。「シンプルな機能を一つずつ作り、積み重ねる」（CLAUDE.md 開発方針）に立ち返る。

中止した feat-013 の `investigation.md` に、座標系・near_plane・確証バイアスの調査記録が残っている。

## 検証で確立済みの事実（本機能の土台）

- **座標系**: viewmat = `[[R, t],[0,0,0,1]]`（R=Rodrigues(TOML rotation), t=translation、OpenCV world-to-camera）で正しい
- **ピンホール古典経路**: `render.py` の `render_frame` と同一。歪みを gsplat のUT経路に乗せると黒い靄が出るため使わない
- **near_plane**: `0.01` だとカメラ至近1〜5cmの floater で黒い靄になる。`0.5` でクリップすると病室がGTと一致して鮮明に映る（実機確認済み）

## 範囲

- 入力: PLY + TOML + `--camera`（C3D・キーポイントは使わない）
- 処理: ピンホール（歪みなし）gsplatレンダリング、`--near-plane` でfloater除去
- 出力: PNG 1枚
- 目的: GTと並べて3DGSレンダリングの正しさを検証

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

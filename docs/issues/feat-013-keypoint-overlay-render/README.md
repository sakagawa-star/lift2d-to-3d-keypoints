# feat-013: 3DGSレンダリング画像への3Dキーポイント重ね描き

## Status: 中止（feat-015/016 に作り直し）

> 2026-06-13 中止。実装・docsが複合し原因切り分けが難航したため、feat-015（3DGSレンダリング）・feat-016（オクルージョン）に分けて作り直す。本フォルダは経緯（investigation.md の座標系・near_plane・確証バイアスの調査記録）として残す。


## 概要

指定した1台のカメラ視点で 3DGS 点群（PLY）を gsplat でレンダリングし、その画像上に人体3Dキーポイント（Halpe26）を投影して点＋ボーンで重ね描きする新規スクリプト（`phase4/render_keypoints.py`）を追加する。連番PNGまたはMP4で出力する（`render.py` の出力に近い）。約1時間分（30fps）の長尺データの静止画化・動画化を想定する。

3つの入力データは Blender 可視化用にインポートしている元ファイルをそのまま使う。**Blender は経由せず、元データ（PLY / TOML / C3D）から直接処理する。**

## 入力データ（`phase4/data/Blender/` に実データあり）

| データ | ファイル | 生成元 | 座標系 |
|---|---|---|---|
| 3DGS点群 | `point_cloud.ply` | LCC Studio（XGRIDS PortalCam） | Z-up・メートル・ネイティブ＝キャリブ座標 |
| カメラキャリブ | `Config_scene.toml` | Pose2Sim（LCC点群由来の基準点で外部パラメータ推定） | OpenCV規約 world-to-camera |
| 人体キーポイント | `20260606-osaka-hosp1_0-301_filt_butterworth_LSTM.c3d` | Pose2Sim | mm単位・Y-up系（要変換） |

## 経緯

- 2026-06-13: アイディアの壁打ち・ヒアリングを実施。座標系の整合（3つのインポート経路の調査）を行い、以下を確定した
  - **C3D生座標 → キャリブ座標の変換**: `(px, py, pz)` → `(pz, px, py) × 0.001`（Pose2Sim の `zup_to_yup` の逆変換）。5台のカメラへの再投影で人体26点がほぼ全点枠内・頭上・カメラ前方になることを実データで検証済み
  - **Pose2Sim_Blender フォークの座標変換バグ修正**: ローカルの `fix/coordinate-transform-rotation-bug` ブランチでカメラインポート時のZ軸-90°回転を除去済み。本機能はBlenderを経由しないため、`render.py` の `--rotate-z90`（本家版バグ補正用）は不要
  - ユーザー決定: 歪み係数あり（レンダリング画像も歪ませて実カメラ映像に合わせる）／描画対象は Halpe26 のみ（_study マーカー43個は除外）／実装は新規スクリプト

## ヒアリングで確定した仕様

- カメラは複数台あるが、`--camera` で指定した1台のみ使用（固定カメラ）
- 重ね描きは点＋ボーン（Halpe26スケルトン）
- 出力解像度はキャリブレーション時の画像サイズ（TOMLの `size`）と同じ
- 歪み係数: あり（projectPoints と gsplat の両方で歪みを適用し、実カメラ映像と同じ見え方にする）

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

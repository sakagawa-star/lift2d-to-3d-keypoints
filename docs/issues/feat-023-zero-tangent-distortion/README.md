# feat-023: estimate_camera_params.py 接線歪みゼロ固定オプション（--zero-tangent）

## ステータス

Closed（2026-07-22完了）

## 概要

`estimate_camera_params.py` に、接線歪み係数 p1, p2 を 0 に固定して放射歪み（k1, k2, 任意で k3）のみを推定する `--zero-tangent` オプションを追加する。

## 背景

E0085-01 カメラのデータ（`data/config_E0085.yaml` + `--fix-center`）で、最適化が p2 = 0.052 という物理的にありえない接線歪み値に収束した。原因は、画像左端ぎりぎりにある基準点（基準_018: x=4, 基準_051: x=13）の残差を接線歪み項が「吸収弁」として引き受けたこと。この2点は推定されたピンホールモデル（K, R, t のみ）では画角外（正規化x = -0.77 / -0.75、ピンホール限界 = 480/657.8 = 0.73）にあり、歪みなしのレンダリング（Blender / gsplat）では画面外になる不整合が発生した。

接線歪みは現代のレンズでは通常 |p| < 0.001 のオーダーであり、0 に固定すること（OpenCV の `CALIB_ZERO_TANGENT_DIST` 相当）は標準的な対策である。

## 関連ドキュメント

- [requirements.md](requirements.md) — 要求仕様書
- [design.md](design.md) — 機能設計書

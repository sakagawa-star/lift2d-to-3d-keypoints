# feat-019: FPS頭部追従カメラのポーズ書き出しスクリプト（ヘッドレス対応）

## ステータス

Open（調査・計画中）

## 概要

`phase4/camera_pose.py` をベースにした新規スクリプト `phase4/fps_camera_pose.py` を作成する。
FPS風頭部追従カメラの向き（顔の一次視線方向）を各フレームで計算・適用してから
`cam.matrix_world` を読み、`blender -b`（ヘッドレス）でも頭部に追従した正しい `c2w` を
JSON に書き出す。

## 背景・裏取り結果

対象 .blend（`phase4/data/Blender/2D-Lift.blend`）のカメラ向きは、Blender の
`frame_change_post` ハンドラ（Python）が毎フレーム計算して与えている。このハンドラは
.blend に保存されないため、`blender -b ... --python camera_pose.py` の別プロセスでは登録
されず発火しない。

Blender 4.5.5 で `2D-Lift.blend` を `-b` で検査した結果（2026-07-01 裏取り確定）:

- シーン構成は参考資料（`fps_camera_handler_reference.md`）と一致
  - アーマチュア `session001_f145749_world300`（22ボーン、`LEye`/`REye`/`LEar`/`REar`/`Nose`/`Head`/`Neck` を含む）
  - Empty `Cam_Anchor`：ARMATURE コンストレイントで `LEye`+`REye`（各 weight 1.0）→ 両目中点
  - カメラ `Cam_FPS`：`Cam_Anchor` の子、ローカル回転ゼロ
- `frame_set` した際のカメラ `matrix_world`：
  - **位置**はフレームごとに変化（Cam_Anchor の ARMATURE コンストレイントで追従、`-b` でも動く）
  - **向き**は全フレーム完全に同一値で凍結（frame 1/125/250 で rot=(1.4919, 0.0703, -0.1818) 一致）
    → まさにハンドラ未発火による症状（不具合 CONFIRMED）
- 参考資料の計算式を `-b` で実行すると、フレームごとに異なる正しい回転（det=1.0, 直交）を出し、
  `anchor.rotation_euler` 代入 + `view_layer.update()` で子カメラ `Cam_FPS.matrix_world` に厳密反映される
  ことも確認済み（f1=(1.364,0.064,0.088), f125=(1.446,0.053,-0.117), f250=(1.485,0.168,0.030)）

## 対象ファイル

- 新規作成: `phase4/fps_camera_pose.py`
- `phase4/camera_pose.py` は変更しない（従来のコンストレイント駆動 .blend 向けに温存）
- `phase4/render.py` は変更しない（座標規約変換は render.py の責務）

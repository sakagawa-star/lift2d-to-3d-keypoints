# feat-018: NPZ→C3D変換スクリプト（Blender io_anim_c3d 取り込み対応）

## 概要

2Dキーポイントを3Dにリフトアップした NPZ（`x3d_world`, 22関節, world座標メートル）を、
Blenderの C3D インポートアドオン `io_anim_c3d` で取り込める C3D ファイルに変換する
スクリプト `phase4/npz_to_c3d.py` を新規作成する。

最終目標は、この C3D を `render_keypoints.py` が扱う C3D と同じように扱えるようにすること。
本案件はその最初のステップとして「NPZ→C3D変換 + Blender取り込み」を担当する。

## ステータス

Open（ドキュメント作成中）

## 背景・関連

- 入力データ仕様: `phase4/data/session001_f145749_world300_spec.md`
- Blenderアドオン: `/home/sakagawa/git/io_anim_c3d/`（py-c3d 0.6.0 同梱）
- 既存の C3D 取り扱い実装: `phase4/render_keypoints.py`
  （`load_c3d_all_frames`, `c3d_to_calib`, `extract_halpe26`）

## 主要な設計判断（ヒアリング済み）

1. **座標規約**: 既存C3D互換。world `(X,Y,Z)` m → C3D raw `(Y,Z,X)` mm。
   `render_keypoints.py` の `c3d_to_calib`（`(px,py,pz)→(pz,px,py)×0.001`）で
   world座標へ復元できる。
2. **関節ラベル**: NPZ の22点（`joint_names`）をそのまま C3D ラベルにする
   （Halpe26への整形はしない）。
3. **Blender取り込み**: `io_anim_c3d` アドオンを使用。`POINT:UNITS='mm'` と
   `POINT:X_SCREEN/Y_SCREEN` で、Blender内で人体が正立（world垂直=Blender上Z）して
   見えるようにする。

## 成果物

- `phase4/npz_to_c3d.py`（新規）
- `docs/issues/feat-018-npz-to-c3d/requirements.md`
- `docs/issues/feat-018-npz-to-c3d/design.md`

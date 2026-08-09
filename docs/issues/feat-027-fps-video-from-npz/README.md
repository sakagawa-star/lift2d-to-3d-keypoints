# feat-027: NPZ直読みによる一人称視点動画一括生成（Blender廃止）

## ステータス

Open（起票済み。要求仕様書・機能設計書は未作成）

## 概要

巨大NPZ（5〜10時間分のリフトアップ済み3Dキーポイント、`npz_to_c3d.py` 入力と同一フォーマット）を直読みし、一人称視点（FPS）の動画を1本のMP4として一括生成する新プログラムを phase4 に作成する。

現行の FPS 動画生成フロー
`NPZ → npz_to_c3d.py → C3D → Blender io_anim_c3d → blender -b + fps_camera_pose.py → ポーズJSON → render.py`
から Blender・C3D 工程を廃止し、手動作業ゼロで大量生産できるようにする。既存スクリプト（`fps_camera_pose.py` / `render.py` 等）は変更せず温存する。

主要要件（詳細は `hearing-notes.md` の確定要件12項目を参照）:

- カメラポーズは頭部キーポイントから numpy で計算（`fps_camera_pose.py` の視線計算数式を移植。位置＝両目中点、向き＝一次視線）
- 内部パラメータは Calib_scene.toml 型 TOML を引数渡し（`--camera` でセクション選択、ピンホールモデル、歪み係数 D は無視）
- NaN 欠損区間（人が不在）は黒画面で出力し、タイムライン（動画時刻＝実時刻）を維持する
- 単一GPU（`--gpu` で1枚選択）の直列実装。区間MP4単位の再開が可能。複数GPUプロセス並列は後続案件に分割（2026-08-07 人レビュー決定。本案件完了後に起票）
- デバッグ用に静止画（連番PNG）出力の排他モードを持つ（`frame_ids` ベースの開始・終了フレーム範囲指定、MP4 出力なし）

## 依存関係

- **feat-028**（NPZキーポイントの時間方向平滑化スクリプト）が前段。本案件の入力は feat-028 で平滑化済みの NPZ とする。実装は feat-028 を先行させる

## 保留（本案件のスコープ外）

- 複数GPUプロセス並列（旧FR-006）: 本案件完了後に別案件として起票する（2026-08-07 人レビュー決定）
- TPS（第三者）視点で FPS カメラの向きを可視化する機能は優先度低。本案件完了後に再ヒアリングして別途起票する

## 経緯

- 2026-08-07: 壁打ちセッションで構想確定、事前ヒアリング整理資料 `hearing-notes.md` を作成
- 2026-08-07: 起票。平滑化スクリプトは feat-028 に分割（依存順に feat-028 から実装）
- 2026-08-07: requirements.md / design.md 作成。Codex レビュー3回で高・中ゼロに収束（reviews/codex-01〜03）
- 2026-08-07: 人レビューで複数GPU並列を後続案件に分割（単一GPU直列実装に変更）。requirements/design を改訂し Codex 収束（codex-04〜05）。破損チャンク検出・耐久書き出しの人レビュー指摘を反映し収束（codex-06〜09）
- 2026-08-08: Sonnet 委任で実装完了（`phase4/render_fps_video.py` + テスト37件全成功、全体回帰 331 passed）。ffprobe 実挙動・libx264 実行時依存の設計訂正（codex-10〜11）
- 2026-08-08: 等価性検証（FR-012・案A）**合格**。criteria 第3版（Blender 4.5.5 直接フルロード、codex-12〜17）で lock 後、300フレーム全件で位置差 max 1.2e-07 m・回転角度差 max 1.5e-02°（基準: 1mm / 0.1°）。記録は `experiments/a1_blender_equivalence/`
- 2026-08-08: 手動テスト（ステップ7）は**保留**。session001 の正しい PLY は `/home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply`（`project.ply` は別シーンで NG）と確認されたが、セッション用キャリブレーション TOML が未作成のため、TOML 作成後にユーザーが実行する

## 参照

- `hearing-notes.md` — 事前ヒアリング整理（確定要件12項目・技術的裏取り・未決事項・実行環境メモ。本案件と feat-028 の共通参考資料）

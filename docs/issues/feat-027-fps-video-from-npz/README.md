# feat-027: NPZ直読みによる一人称視点動画一括生成（Blender廃止）

## ステータス

Closed（2026-08-10 手動テスト合格）

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
- 2026-08-09: ユーザーが `phase4/data/Calib_FPSCamera.toml` を作成（FPSCamera、fx=fy=2133.33=40mm相当、主点=画像中心、1920x1080。仕様適合をローダーで確認済み）。正しい入力の組で `test_fps.mp4` を生成し、**手動テストは仮合格のまま保留**とした。ここまでの成果物をコミット（`8ad605b`、push 済みかは未確認）
- 2026-08-09: 旧方式（Blender経由）のFPS動画との見比べを検討したが、旧動画は現存しないと判明（`data/images/` に PNG 2枚のみ。旧ポーズJSONは2種現存）。この時点では再生成しない判断
- 2026-08-10: 方針転換。**旧Blenderパイプラインで作った映像との比較が済むまで手動テスト合格としない**（feat-029 の手動テスト合否も同条件）。比較はレンダリング前にまず**外部パラメータ（c2w）の照合**で行う（一致しなければ映像も一致しないため）。既存データの流用ではなく**データを作り直す**方針をユーザーが決定: `filter_npz` で平滑化した300フレームNPZを共通の出発点とし、旧経路（npz_to_c3d → C3D → Blender 4.5.5 io_anim_c3d 取り込み → fps_camera_pose.py）と新経路（render_fps_video.py --dump-poses）の両方に流す（時間平滑化の適用フォーマット差 C3D/NPZ による微差の混入を排除するため）。これを**実験 a2** として起票予定
- 2026-08-10: a2 の事前確認完了（詳細は `experiments/a2_rebuilt_equivalence/prechecks.md`）: io_anim_c3d は Blender 4.5.5 で有効、カメラリグの構成（Cam_Anchor の ARMATURE 制約 = LEye/REye weight 各1.0 → 両目中点）判明、リグ再構築方針確定。データ仕様書 `session001_f145749_world300_spec.md` を入手し `pnp_ok` の意味が確定（hearing-notes 未決事項0に反映。fps=30 も公式確定）
- 2026-08-10: a2 の進め方: **criteria 作成 → Codex 収束 → ユーザーに再確認 → 実行**（収束後に実行前確認を挟む方針をユーザーが指定）
- 2026-08-10: a2 criteria lock（codex-18〜20。codex-18 の高指摘=io_anim_c3d のレート適応と先頭キー frame 0 問題は、fps=30 明示・キー正規化・NPZ直接照合の fail-fast で対策。第2版で検証用 a2_scene.blend 保存を追加）。ユーザー承認のうえフェーズ0〜3 を実行し、**実験 a2 合格**: 位置差 max 6.0e-08 m・回転角度差 max 1.6e-02°（基準 1mm / 0.1°、300フレーム全件）。旧経路の先頭キーが実際に frame 0 に入り正規化が発動したこと、生データに最大 1348mm/フレームの外れ値ジャンプがあり平滑化変位 max 401mm はその補正であることも確認。記録は `experiments/a2_rebuilt_equivalence/`
- 2026-08-10: 映像比較用の新旧動画をユーザー承認のうえ生成（`experiments/a2_rebuilt_equivalence/` 配下）。画角統一は当初案（Blenderレンズ一時変更）に代え、a2 検証済み old_poses.json の fx/fy を TOML と同一値に置換した `old_poses_40mm.json` で実施（fx はレンダリング用メタデータで c2w と独立のため、より正確に K を一致させられる）。旧動画 `old_video/output.mp4`（render.py、NVENC 自動選択・約1.6Mbps）、新動画 `new_video_smoothed.mp4`（libx264 CRF18・約5.5Mbps）。両者とも h264 / 1920x1080 / 30fps / 300フレーム。PSNR（参考値）: average 40.2dB / min 31.2dB — ポーズ微小差（max 0.016° ≈ 最大0.6px 級のずれ）とエンコーダー品質差の合算として説明可能な水準
- 2026-08-10: ユーザーが a2_scene.blend の Blender 操作を目視確認（OK）、新旧動画の目視比較（だいたい同じ=期待どおり）を実施し、**手動テスト合格**。完了処理を実施しクローズ
- 後続案件の起票候補（未起票）: 複数GPU並列（旧FR-006）、`pnp_ok=False` フレームの扱い（描くか黒か）

## 参照

- `hearing-notes.md` — 事前ヒアリング整理（確定要件12項目・技術的裏取り・未決事項・実行環境メモ。本案件と feat-028 の共通参考資料）

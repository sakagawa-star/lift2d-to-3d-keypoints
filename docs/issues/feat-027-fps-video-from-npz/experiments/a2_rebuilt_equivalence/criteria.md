# 実験 a2: 再構築データによる新旧外部パラメータ照合 — 判定基準書

作成日: 2026-08-10
状態: **第2版 locked**（2026-08-10、codex-20 で高・中ゼロ収束。初版=codex-18〜19、第2版=検証用 `a2_scene.blend` 保存の追加のみで**判定基準・照合手順は初版から不変**。実験の着手はユーザー承認済み〔2026-08-10、.blend 保存追加→再lock→フェーズ0〜3 実行〕）
準拠: CLAUDE.md「実験・検証の進め方（予測→実行→照合）」、事前調査は `prechecks.md`
関連: 実験 a1（`../a1_blender_equivalence/`。判定閾値は a1 第3版 locked 値を流用する）

## 1. 答える問い

**同一の平滑化済みキーポイントデータ**を旧パイプライン（C3D → Blender io_anim_c3d → `fps_camera_pose.py`）と新実装（`render_fps_video.py`）に流したとき、両者のFPSカメラ外部パラメータ（c2w）は一致するか。

a1 との違い: a1 は既存 .blend の（出自未検証の）アクションを使った。a2 はデータを作り直して両経路の入力同一性を構成的に保証し、かつ実運用フロー（npz_to_c3d → io_anim_c3d 取り込み）をそのまま通す。比較対象は c2w のみ（内部パラメータは比較しない。a1 と同じ）。

## 2. 使用データ・環境・コマンド（固定。すべてリポジトリルートで実行）

実験フォルダ `<A2>` = `docs/issues/feat-027-fps-video-from-npz/experiments/a2_rebuilt_equivalence`

| 項目 | 値 |
|---|---|
| 生NPZ | `phase4/data/session001_f145749_world300.npz`（F=300, frame_ids 145599〜145898, NaN 0。データ仕様書 `../../session001_f145749_world300_spec.md` で確認済み） |
| フェーズ0a 平滑化 | `uv run --project phase4 python phase4/filter_npz.py phase4/data/session001_f145749_world300.npz --fps 30 --output <A2>/smoothed_world300.npz`（cutoff / max-gap はデフォルト 6.0 Hz / 10 のまま） |
| フェーズ0b C3D変換 | `uv run --project phase4 python phase4/npz_to_c3d.py <A2>/smoothed_world300.npz --output <A2>/smoothed_world300.c3d --fps 30.0` |
| フェーズ1 旧経路 | `/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b --python <A2>/run_old_pipeline_a2.py` → `<A2>/old_poses.json`。**run_old_pipeline_a2.py は本 criteria の lock 対象**。固定内容: (1) **インポート前に `scene.render.fps = 30` / `fps_base = 1.0` を明示設定**（io_anim_c3d のレート適応が scene fps / C3D fps でキー時刻をスケールするため、ヘッドレス既定 24fps のままでは対応が壊れる。30/30=1 で無害化）(2) C3D を `bpy.ops.import_anim.c3d` でインポート（`adapt_frame_rate` プロパティが存在するバージョンでは False を明示。新アーマチュアはインポート前後差分で特定）(3) **キーフレームを一様シフトで「Blender frame 1 = 先頭サンプル」に正規化**し、正規化後のキー範囲が 1〜300 でなければ exit 1（importer が先頭キーを frame 0 に置くバージョン差と、レート適応の混入をここで遮断）(4) **fail-fast フレーム対応検証**: frame 1/150/300 の LEye/REye/Nose のボーンワールド位置を `smoothed_world300.npz` の index f−1 と直接照合し、成分最大差 > 1e-5 m なら exit 1（C3D の mm float32 丸めは 1e-7 m 級のため、off-by-one・スケールずれを確実に検出できる）(5) `Cam_Anchor`/`Cam_FPS` を append〔依存で旧アーマチュアが混入した場合は削除〕→ ARMATURE 制約ターゲットを新アーマチュアの LEye/REye・weight 各1.0 で決定的に再構築 (6) フレーム範囲 1〜300 → `fps_camera_pose.export_camera_poses()` 呼び出し (7) エクスポート成功後、検証用シーンを `<A2>/a2_scene.blend` に保存（ユーザーの Blender 操作確認用。**判定には使用しない**。カメラの向きは書き出し中の一時代入のため保存ファイルでは静的）。各段の前提が崩れたら exit 1 |
| フェーズ2 新経路 | `uv run --project phase4 python phase4/render_fps_video.py /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply <A2>/smoothed_world300.npz --toml phase4/data/Calib_FPSCamera.toml --camera FPSCamera --fps 30 --dump-poses <A2>/new_poses.json`（PLY・TOML・fps はポーズ計算に影響しない引数要件。PLY は session001 の正しいシーンのものを指定） |
| フェーズ3 照合 | `uv run --project phase4 python <A2>/compare_poses_a2.py`。**compare_poses_a2.py も lock 対象** |
| Blender | 4.5.5 LTS（フルパス固定。PATH の 4.0.2 は使用禁止。a1 と同じ） |

## 3. フレーム対応の定義

C3D は 1〜300 の連番（`npz_to_c3d.py` 仕様）。旧経路 JSON の `frame` フィールド f（1〜300）と、新経路 JSON の `frames[f−1]` を対応させる。frame_id の整合は `frames[f−1].frame_id == 145598 + f` で照合スクリプトが機械検証する。

前提チェック（照合前に確認。不成立なら判定に進まず原因を調査する）:

- P0: 平滑化済みNPZ・C3D が F=300 のまま生成される（フェーズ0の出力ログで確認）
- P1a: 旧経路 JSON のフレーム数 = 300（`frame` は 1〜300 の連番）
- P1b: 旧経路ラッパー内の fail-fast フレーム対応検証（§2 フェーズ1 の (3)(4)）が通過している（exit 0 で old_poses.json が生成された時点で成立。不成立なら exit 1 で止まる）
- P2: 新経路 JSON の `num_frames` = 300、全フレーム `valid = true`、frame_id が 145599〜145898 の連番

## 4. 判定指標と閾値（lock 対象。a1 第3版 locked 値の流用）

各対応フレームについて（定義は a1 と同一）:

- **位置差** `d_pos = ‖t_old − t_new‖`（m）
- **回転角度差** `d_rot = arccos(clamp((trace(R_old^T @ R_new) − 1) / 2, −1, 1))`（度）。両 JSON とも Blenderカメラ規約の c2w を直接比較

**合格基準（a1 第3版と同一。事後変更しない）**:

| 指標 | 閾値 | 合格ライン |
|---|---|---|
| 位置差 | max(d_pos) < 1mm (0.001 m) | 300フレーム全件 |
| 回転角度差 | max(d_rot) < 0.1° | 300フレーム全件 |

- 両方満たせば**合格**。1フレームでも超えれば**不合格**とし原因を特定する
- 予測される誤差水準の参考: a1 実測は max 1.2e-07 m / 1.5e-02°。a2 は C3D の mm float32 丸めが追加で入るため位置差は a1 よりやや大きくなり得るが、閾値には十分収まる見込み（値の予測は各フェーズ直前に experiment_log.md へ記録する）

## 5. 実験手順と予測の枠（値は各フェーズ直前に experiment_log.md へ記録）

| フェーズ | 内容 | 直前に予測する項目 |
|---|---|---|
| 0 | filter_npz → npz_to_c3d でデータ生成 | 出力フレーム数、exit code、平滑化の変位規模 |
| 1 | 旧経路実行（run_old_pipeline_a2.py） | exit code、出力フレーム数（前段の結果を見て予測） |
| 2 | 新経路実行（--dump-poses） | exit code、num_frames、valid 件数 |
| 3 | compare_poses_a2.py で照合 | max(d_pos)・max(d_rot) の桁（フェーズ1・2 の結果を見て予測） |

各フェーズの実行後、予測と実測を照合して記録する。乖離した場合は原因を特定してから次フェーズに進む。

## 6. 禁止事項（事後解釈の禁止）

- 結果を見てから閾値・合格ラインを変更しない
- 本書にない条件（特定フレームの除外、指標の差し替え）を付けて合格扱いにしない
- 不合格の場合は「不合格＋原因調査」として記録する

## 7. 成果物

- `smoothed_world300.npz` / `smoothed_world300.c3d` / `old_poses.json` / `new_poses.json` / `a2_scene.blend`（中間生成物。git 管理外可。`a2_scene.blend` はフェーズ1の Blender 操作をユーザーが目視確認するための保存物で、判定には使用しない）
- `run_old_pipeline_a2.py` / `compare_poses_a2.py`（lock 対象スクリプト）
- `experiment_log.md`（予測→実測→照合の記録。判定結果を含む）

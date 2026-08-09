# 実験 a1: 旧Blenderパイプラインとのカメラポーズ等価性検証（FR-012・案A）— 判定基準書

作成日: 2026-08-08
状態: **第3版 locked**（2026-08-08、codex-16〜17 で高・中ゼロ収束。run_old_pipeline.py を含めて lock。以降、判定基準・手順の変更禁止。**§4 の判定閾値は初版から不変**）

改訂履歴:
- 2026-08-08 初版 lock（codex-12〜13）
- 2026-08-08 第2版 lock（codex-14〜15）: §2 使用データをフルNPZに訂正・旧経路を append 方式（run_old_pipeline.py、lock 対象）に変更・§3 フレーム対応を改訂
- 2026-08-08 第3版: 正しい Blender は 4.5.5（ユーザー指定 `/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64`）と判明。4.5.5 ではフルロードが成功する（診断で確認）ため append 方式を廃止し、**直接フルロード方式**に変更（run_old_pipeline.py を更新、lock 対象）。4.0.2 で生成した old_poses.json は無効とし 4.5.5 で再生成する。経緯は experiment_log.md を参照
準拠: CLAUDE.md「実験・検証の進め方（予測→実行→照合）」、requirements.md FR-012、design.md §5

## 1. 答える問い

`phase4/render_fps_video.py` に numpy 移植したFPSカメラポーズ計算（FR-003）は、旧 Blender パイプライン（`fps_camera_pose.py`）と数値等価か。

比較対象は **c2w（カメラ位置と回転）のみ**。内部パラメータは比較しない（旧は .blend カメラ由来、新は TOML 由来で入力源が異なるため。requirements FR-012 で確定済み）。

## 2. 使用データ・環境（固定）

| 項目 | 値 |
|---|---|
| NPZ（新経路入力） | `phase4/data/session001_world_22pt.npz`（フルデータ。F=194279, J=22, frame_ids 0〜194278〔frame_id = 配列インデックス〕, 有効 193939 / NaN 340〔2026-08-08 実測〕） |
| Blender 実行バイナリ | `/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender`（**4.5.5 LTS、ユーザー指定。固定**）。PATH の `/usr/bin/blender`（4.0.2）はこの .blend とバージョン不整合でフルロード時にクラッシュするため使用禁止（experiment_log.md 診断で確認） |
| .blend（旧経路入力） | `phase4/data/Blender/session001_world_22pt.blend`。アーマチュア `E00000`（22ボーン、アクション `session001_world_22pt`・フレーム範囲 41〜194239 = フルNPZ由来）と `Cam_Anchor` / `Cam_FPS`（親=Cam_Anchor）リグを含む。シーンフレーム範囲は 1〜9000（4.5.5 での診断実測） |
| 事前検証済みの対応関係 | Blender フレーム f のボーン頭ワールド位置 = フルNPZ `x3d_world[f−1]`（LEye/Nose、フレーム150/300 で成分最大差 0.000e+00。4.0.2 の append 検証時の実測。experiment_log.md フェーズ1に記録） |
| 比較ウィンドウ | Blender フレーム **41〜340**（NPZ インデックス 40〜339 = frame_id 40〜339）の300フレーム。頭部7点が全有効な最初の300フレーム連続区間（2026-08-08 実測で確定） |
| 旧経路の実行方式 | リポジトリルートで実行: `/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b --python docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/run_old_pipeline.py`。**run_old_pipeline.py は実験フォルダに保存済みで、本 criteria の lock 対象に含む**（固定内容: `bpy.ops.wm.open_mainfile` で .blend を**直接フルロード**、`scene.frame_start=41` / `frame_end=340` を設定、`sys.path` に `phase4/` を追加して `fps_camera_pose.export_camera_poses("Cam_FPS", "E00000", "Cam_Anchor", <実験フォルダ>/old_poses.json)` を呼び出し、.blend 不在時は exit 1。**ポーズ計算・リグ検証・JSON書き出しは旧スクリプトの関数をそのまま使用**する。シーン・リグ・制約は .blend のオリジナルをそのまま使うため、第2版の append 方式より旧パイプラインへの忠実度が高い） |
| 新経路コマンド | リポジトリルートで実行: `uv run --project phase4 python phase4/render_fps_video.py phase4/data/project.ply phase4/data/session001_world_22pt.npz --toml phase4/data/Blender/Config_scene.toml --camera cam41520554 --fps 30 --dump-poses docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/new_poses.json`（PLY・TOML・fps はポーズ計算に影響しない引数要件のため、実在パス `phase4/data/project.ply` / `Config_scene.toml` の `cam41520554` に固定する） |
| 照合スクリプト | `<実験フォルダ>/compare_poses.py`（本実験用の使い捨てスクリプト。実験フォルダ = `docs/issues/feat-027-fps-video-from-npz/experiments/a1_blender_equivalence/`） |

## 3. フレーム対応の定義

旧経路 JSON の `frame` フィールド f（Blender シーンフレーム、41〜340）と、新経路 JSON の `frames[f−1]`（NPZ インデックス f−1 = frame_id f−1）を対応させる。

前提チェック（照合前に確認。不成立なら判定に進まず原因を調査する）:

- P1: 旧経路 JSON のフレーム数 = 300（`frame` は 41〜340 の連番）
- P2: 新経路 JSON の `num_frames` = 194279、かつ比較ウィンドウ（インデックス 40〜339）の300フレームすべてが `valid = true`

## 4. 判定指標と閾値（lock 対象）

各対応フレームについて:

- **位置差** `d_pos = ‖t_old − t_new‖`（ユークリッド距離、単位 m）。t は c2w の並進成分
- **回転角度差** `d_rot = arccos(clamp((trace(R_old^T @ R_new) − 1) / 2, −1, 1))`（単位は度に換算）。R は c2w の回転 3x3。両 JSON とも Blenderカメラ規約の c2w なので座標変換なしで直接比較する

**合格基準（requirements FR-012 で確定済み。事後変更しない）**:

| 指標 | 閾値 | 合格ライン |
|---|---|---|
| 位置差 | max(d_pos) < 1mm (0.001m) | 比較300フレーム全件で満たす |
| 回転角度差 | max(d_rot) < 0.1° | 比較300フレーム全件で満たす |

- 両方を満たせば**合格**（FR-012 受け入れ）。1フレームでも超えれば**不合格**とし、原因を特定するまで手動テスト（ステップ7）に進まない
- 閾値の根拠: 旧経路の数値誤差（C3D の mm float32 丸め ≈ µm オーダー、mathutils の float32 演算、Euler 変換往復）より2桁以上大きく、実装ミス（軸取り違え・符号誤りは数十cm・数十°規模）より2桁以上小さい

## 5. 実験手順と予測の枠（値は各フェーズ直前に experiment_log.md へ記録）

| フェーズ | 内容 | 直前に予測する項目 |
|---|---|---|
| 1 | 旧経路実行（Blender 4.5.5 + run_old_pipeline.py の直接フルロード方式） | 出力フレーム数、exit code |
| 2 | 新経路実行（--dump-poses） | 出力フレーム数、valid 件数（フェーズ1の結果を見てから予測） |
| 3 | compare_poses.py で照合 | max(d_pos)・max(d_rot) の桁（フェーズ1・2の結果を見てから予測） |

各フェーズの実行後、予測と実測を experiment_log.md で照合する。乖離した場合は前提・理解を疑い、原因を特定してから次フェーズに進む。

## 6. 禁止事項（事後解釈の禁止）

- 結果を見てから閾値・合格ラインを変更しない
- 本書にない条件（特定フレームの除外、指標の差し替え）を付けて合格扱いにしない
- 不合格の場合は「不合格＋原因調査」として記録する（弾いて合格にしない）

## 7. 成果物

- `old_poses.json` / `new_poses.json`（git 管理外でも可。サイズ小のため実験フォルダに保存）
- `compare_poses.py`（照合スクリプト）
- `experiment_log.md`（予測→実測→照合の記録。判定結果を含む）

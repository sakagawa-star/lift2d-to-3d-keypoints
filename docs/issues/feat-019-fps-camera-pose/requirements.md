# feat-019 要求仕様書: FPS頭部追従カメラのポーズ書き出しスクリプト

## 1.1 プロジェクト概要

- **何を作るのか**: `phase4/camera_pose.py` をベースにした新規スクリプト `phase4/fps_camera_pose.py`。
  各フレームで頭部キーポイント（アーマチュアのボーン head 位置）から顔の一次視線方向を計算し、
  カメラアンカーの回転として適用してから `cam.matrix_world` を全フレーム分 JSON に書き出す。
- **なぜ作るのか**: FPS風頭部追従カメラの「向き」は Blender の `frame_change_post` ハンドラ（Python）が
  与えているが、このハンドラは .blend に保存されず `blender -b`（ヘッドレス別プロセス）では発火しない。
  その結果、位置は頭部に追従するが向きが凍結した `c2w` が書き出される。バッチ処理でも正しい向きを
  出力できるようにする。
- **誰が使うのか**: phase4 の 3DGS 一人称視点レンダリングを行う開発者（本プロジェクト作業者）。
- **どこで使うのか**: `phase4/` ディレクトリ。`blender -b <blend> --python fps_camera_pose.py -- <args>` で
  ヘッドレス実行。出力 JSON は既存 `render.py` / `render_keypoints.py` の入力になる。

## 1.2 用語定義

以下の用語は機能設計書・コード内でも同一の意味で使用する。

- **アンカー（anchor）**: カメラの親となる Empty オブジェクト。位置は Blender コンストレイントが担い、
  向きを本スクリプトが計算して `rotation_euler` に代入する。対象 .blend では `Cam_Anchor`。
- **一次視線方向（primary gaze / 前方 f）**: 眼球運動を含まない頭部姿勢としての視線方向。
  耳中点→鼻ベクトルを両目軸に直交化して求める（Frankfurt 平面ベース）。
- **頭部キーポイント**: アーマチュアの `LEye`/`REye`/`LEar`/`REar`/`Nose`/`Head`/`Neck` の各ボーンの
  head（根元）ワールド座標。
- **評価済み depsgraph**: アニメーション・コンストレイントを適用した後のオブジェクト状態を持つ
  依存グラフ（`bpy.context.evaluated_depsgraph_get()`）。frame_set 後の実際のボーン位置はここから取る。
- **c2w**: camera-to-world 変換の 4x4 行列。Blender 規約（カメラローカル -Z 前方 / +Y 上 / +X 右）。
- **KIRIモディファイヤ**: KIRI Engine アドオンが 3DGS オブジェクトに付与する Geometry Nodes モディファイヤ
  （名前 `KIRI_3DGS_Render_GN`）。書き出し中は一時無効化する。

## 1.3 機能要求一覧

### FR-001: 頭部一次視線方向の計算

- **機能名**: 頭部姿勢（一次視線方向）の回転計算
- **概要**: frame_set 済みの状態で、評価済みアーマチュアの頭部キーポイントから
  顔の一次視線方向を表す回転行列を計算し、Euler 角に変換する。
- **入力**: 評価済みアーマチュアオブジェクト（`arm.evaluated_get(depsgraph)`）と対象ボーン名。
- **出力**: `mathutils.Euler`（アンカーに代入する回転）。
- **計算式**（本書の下記式を唯一の正本とする。参考として案件フォルダに同梱した
  `fps_camera_handler_reference.md` に導出背景がある。ワールド座標で計算）:
  - `ear_mid = (LEar + REar) / 2`
  - `p = normalize(Nose - ear_mid)`（前方の手がかり）
  - `r0 = normalize(LEye - REye)`（両目軸）
  - `f = normalize(p - r0 * dot(p, r0))`（前方＝一次視線）
  - `u = normalize(f × r0)`、`head_up = normalize(Head - Neck)` との内積が負なら `u = -u`
  - `b = -f`、`r = normalize(u × b)`
  - `rot = Matrix((r, u, b)).transposed()`、戻り値 `rot.to_euler()`
- **受け入れ基準**:
  - 計算した `rot` が `determinant ≈ +1`（許容 1e-6）かつ直交行列である
  - 同一 .blend の同一フレームで、対話GUI（ハンドラ有効）時の `Cam_Anchor.rotation_euler` と一致する
  - 正規化前のベクトル長・`rot` の直交性・NaN/Inf を各フレームで検証し（FR-005）、違反時は
    JSON を書き出さず異常終了する

### FR-002: 各フレームでの姿勢適用とポーズ書き出し

- **機能名**: 頭部追従カメラポーズの全フレーム書き出し
- **概要**: `scene.frame_start`〜`scene.frame_end` の各フレームで、frame_set →（FR-001 で回転計算）→
  アンカーへ適用 → depsgraph 更新 → `cam.matrix_world` と内部パラメータを記録し、JSON に書き出す。
- **入力**: コマンドライン引数（カメラ名・アーマチュア名・アンカー名・出力パス）と対象 .blend の状態。
- **出力**: JSON ファイル。各フレーム要素は `frame`, `c2w`(4x4), `fx`, `fy`, `cx`, `cy`, `width`, `height` を持つ
  （既存 `camera_pose.py` と同一スキーマ）。
- **処理順序（厳守）**: `scene.frame_set(frame)` → 評価済み depsgraph 取得 → FR-001 で回転計算 →
  `anchor.rotation_euler` に代入 → `view_layer.update()` → `cam.matrix_world` 読み取り。
- **受け入れ基準**:
  - 複数フレームで `c2w` の回転成分が変化する（全フレーム同一値でない）
  - JSON のフレーム数が `frame_end - frame_start + 1` と一致する
  - 出力スキーマが既存 `camera_pose.py` と同一で、`render.py` がそのまま読み込める

### FR-003: コマンドライン引数

- **機能名**: 実行パラメータの指定
- **概要**: カメラ名・アーマチュア名・アンカー名・出力パスを CLI で指定する。
- **入力**: `--` 以降のスクリプト引数（Blender 規約）。
  - `--camera <name>`（必須）: カメラオブジェクト名（例: `Cam_FPS`）
  - `--armature <name>`（任意、デフォルト `session001_f145749_world300`）: アーマチュア名
  - `--anchor <name>`（任意、デフォルト `Cam_Anchor`）: アンカー Empty 名
  - `--output <path>`（任意、デフォルト `data/<カメラ名>_poses.json`）: 出力 JSON パス
- **出力**: 解析済み引数（内部）。省略時のデフォルト補完。
- **受け入れ基準**:
  - `--camera` 未指定時は argparse がエラー終了する
  - `--output` 省略時に `data/<カメラ名>_poses.json` が補完される（既存 `camera_pose.py` と同一）

### FR-004: 入力検証とエラー処理

- **機能名**: シーン構成の検証
- **概要**: 必要なオブジェクト・ボーンがシーンに存在するかを検証し、無ければ明示的に失敗する。
- **入力**: シーン内オブジェクト・ボーン。
- **出力**: stderr へのエラーメッセージと `sys.exit(1)`。
- **検証項目と挙動**（すべてフレームループ開始前に実施）:
  - カメラ（`--camera`）がアクティブシーンに存在しないか、type が `CAMERA` でない → エラー終了
  - アーマチュア（`--armature`）が存在しないか、type が `ARMATURE` でない → エラー終了
  - アンカー（`--anchor`）が存在しないか、type が `EMPTY` でない → エラー終了
  - **カメラの親がアンカーでない（`cam.parent != anchor`）→ エラー終了**
    （アンカーの回転が `cam.matrix_world` に反映される親子関係が本スクリプトの前提のため）
  - **カメラのローカル回転がゼロでない（`cam.rotation_euler` の各成分 abs > `ANG_TOL=1e-4` rad）→ エラー終了**
    （ローカル回転が残ると計算した視線方向に混ざり、誤った向きになるため。参考資料の「ローカル回転ゼロ」前提。
    フレームループ前の fail-fast チェック）
  - 対象ボーン（`LEye`/`REye`/`LEar`/`REar`/`Nose`/`Head`/`Neck`）のいずれかが
    アーマチュアの pose.bones に無い → 不足ボーン名を列挙してエラー終了
  - `scene.frame_start > scene.frame_end` → エラー終了
- **受け入れ基準**:
  - 上記いずれの不足時も stderr にメッセージを出し、終了コード 1 で終了する（無言で誤出力しない）
- **設計方針（検証手段）**: アンカーの位置追従はコンストレイント種別・ターゲットの内部を
  introspection するのではなく、上記「カメラ位置＝両目中点」の出力レベル検証で担保する
  （実装方式に依存せず堅牢。コンストレイントが別方式に変わっても正しく機能する）。

### FR-005: 姿勢計算結果の健全性検証（各フレーム）

- **機能名**: 縮退・非直交・NaN/Inf の検出、およびカメラ出力変換の整合性検証
- **概要**: FR-001 の計算過程で幾何縮退や数値異常が生じた場合、または適用した回転・位置が
  `cam.matrix_world` に正しく反映されていない場合、誤った `c2w` を出力せず異常終了する。
- **入力**: FR-001 の中間ベクトル（`p`, `r0`, `f`, `u`, `head_up`, `b`, `r`）と回転行列 `rot`、
  および適用後の `cam.matrix_world`・評価済み両目中点。
- **出力**: 違反時は stderr へフレーム番号付きメッセージ + `sys.exit(1)`（JSON は書き出さない）。
- **検証項目**（`EPS = 1e-8`, `ORTHO_TOL = 1e-6`, `POS_TOL = 1e-4` m）:
  - **(a) 計算段階**（`compute_anchor_rotation_euler` 内）:
    - 各 `.normalized()` 直前のベクトル長が `EPS` 未満 → 縮退（例: `LEye == REye`、`Nose == ear_mid`、
      `p ∥ r0`、`Head == Neck`）としてエラー終了
    - `abs(rot.determinant() - 1.0) > ORTHO_TOL` または直交でない（`rot @ rot.transposed()` が単位行列から
      `ORTHO_TOL` 超乖離）→ エラー終了
    - いずれかの成分が NaN/Inf → エラー終了
  - **(b) 出力段階**（各フレーム、アンカー回転適用 + `view_layer.update()` 後、JSON append 前）:
    - 位置整合: `cam.matrix_world.translation` と評価済み両目中点（`(LEye+REye)/2`）の距離が
      `POS_TOL` を超える → エラー終了（位置追従の崩れ・親子オフセット混入を検出）
    - 回転整合: `cam.matrix_world.to_3x3()` と計算した回転 `rot`（= `rot_euler.to_matrix()`）の
      各成分差の最大が `ORTHO_TOL` を超える → エラー終了（`matrix_parent_inverse` の回転成分等による
      向き混入を検出）
- **受け入れ基準**:
  - (a)(b) のいずれか1フレームでも違反したら、JSON を書き出さずに終了コード 1 で終了する
  - 裏取り済み事実: 対象 .blend では全フレームで位置 dist=0、回転は計算値と厳密一致するため、
    正常系ではこの検証を素通りする

## 1.4 非機能要求

- **パフォーマンス**: 250 フレーム程度の書き出しが実用時間（数秒〜十数秒）で完了すること
  （厳密な数値目標は課さない。frame_set のデプスグラフ評価が支配的）。
- **対応環境**: Blender 4.5.5 LTS（`/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender`）、
  `blender -b`（ヘッドレス）。GPU 不要。
- **信頼性**: シーンを破壊しない。KIRIモディファイヤの表示状態は書き出し後に元へ戻す。
  出力 JSON は原子的に書き出す（一時ファイルに書いてから `os.replace` で確定）。書き込み失敗・
  中断時に既存の有効な JSON を破壊しないこと。異常検出時（FR-004/FR-005）は JSON を一切書き出さない。
- **セキュリティ**: 対象外。

## 1.5 制約条件

- **使用必須**: Blender Python API（`bpy`, `mathutils`）、`numpy`（既存 `camera_pose.py` と同様、
  `matrix_world` の JSON 変換に使用）、標準ライブラリ（`argparse`, `json`, `os`, `sys`）。
- **使用禁止**: 追加の外部ライブラリ導入は行わない。
- **スコープ制約**:
  - 変更してよいのは新規ファイル `phase4/fps_camera_pose.py` のみ。
    `phase4/camera_pose.py` と `phase4/render.py` は変更しない。
  - 座標規約変換（`blender_to_opencv_c2w`、`rotate_z90`）は `render.py` の責務。本スクリプトは
    Blender 規約の `cam.matrix_world` をそのまま出力し、追加の規約変換を入れない（二重変換防止）。
- **ネットワーク**: 不要（オフライン動作）。

## 1.6 優先順位（MoSCoW）

- **Must**: FR-001, FR-002, FR-003, FR-004, FR-005（すべて MVP）
- **Should**: なし
- **Could**: 正常時にも `determinant`/直交性を毎フレーム INFO ログ出力する（`--verbose` 等）。今回は範囲外
  （FR-005 は異常時のみ出力する）。
- **Won't**: アーマチュアが無い .blend（コンストレイント駆動の `FPS-camera.blend` 等）への自動対応。
  それは既存 `camera_pose.py` の担当とし、本スクリプトはアーマチュア構成専用とする。

- **MVP の範囲**: FR-001〜FR-005 を満たし、`blender -b 2D-Lift.blend --python fps_camera_pose.py --
  --camera Cam_FPS` で頭部追従した `c2w` を含む JSON を出力できること。

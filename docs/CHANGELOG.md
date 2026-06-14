# CHANGELOG

## リリース履歴

### 2026-06-14

- **feat-016**: キーポイントのオクルージョン（深度による前後判定）
  - `phase4/render_keypoints.py` を拡張し、3DGSレンダリング背景に人体キーポイント（C3D, Halpe26）の先頭フレーム1枚を点＋ボーンで重ね描き。深度比較で前後関係（オクルージョン）を反映し、手前の3DGSに隠れる点・ボーンを隠蔽
  - `render_image` に `return_depth` 引数を追加（既定 False で feat-015 と同一挙動の後方互換）。True 時は `render_mode="RGB+ED"` で `(bgr, depth_map, alpha_map)` を返す
  - 新規関数: `load_c3d_first_frame`（先頭フレームのみ）/`c3d_to_calib`（(px,py,pz)mm→(pz,px,py)m）/`extract_halpe26`（residual・NaNで有効判定）/`project_keypoints`（歪みなし投影）/`compute_keypoint_depth`/`compute_visibility`/`draw_overlay`
  - `compute_visibility` は valid→深度有効性（finite かつ >near_plane の背面ガード）→画像外→α→深度比較の順で判定。α判定を深度比較より先に置き低α画素の不安定な深度を回避。背景判定はαマップ（accumulation, 内部閾値0.5）で行う
  - ボーンは線分を `BONE_SAMPLES=24` 分割して部分隠蔽（障害物を横切るボーンが途中で切れる）。色分けは被験者の解剖学的左右（右=赤/左=青/体幹・顔=緑）、点=黄
  - CLI: `c3d_path` を必須位置引数として追加（feat-015 の旧2引数CLIは意図的に廃止）。`--no-occlusion`（深度経路を通さず全点手前描画）/`--occlusion-margin`（既定0.05m）を追加
  - 新規依存なし（`c3d>=0.6.0` は導入済み）。`render.py` 非改変
  - テスト: `tests/test_feat016_keypoints.py` 新規23件、`tests/test_feat015_render.py` の main系3テストを新CLI・新スタブ署名に更新（全86件成功）
  - 手動テスト（実機GPU）でキーポイントの位置・姿勢の妥当性とオクルージョンの効果を目視確認

- **feat-015**: ピンホール3DGSレンダリング（PNG出力、GT比較）
  - `phase4/render_keypoints.py` を3DGSレンダリングのみに作り直し（中止した feat-013 のキーポイント重ね描き・C3D・MP4・dry-run・フレーム範囲機能を全削除）
  - `render_image` を追加（gsplat ピンホール古典経路。`render.py` の `render_frame` と同一の `camera_model="pinhole"`/`with_ut=False`/`packed=True`、歪み・UT・深度なし）
  - `main(argv=None)` を「PLY+TOML+カメラ → PNG 1枚」に書き直し。`--camera`/`--near-plane`（既定0.1）/`--output`/`--background` を提供
  - viewmat は TOML から直接構成（`[[R, t],[0,0,0,1]]`）。主点ずれも TOML の K をそのまま反映
  - `--near-plane` でカメラ至近の floater を除去（`0.01` の黒い靄 → `0.5` で病室が鮮明、実機で目視確認）
  - PNG保存は `cv2.imwrite` の `False` 返却（権限・エンコード失敗）と `cv2.error` 送出（不正な拡張子等、OpenCV 4.13.0 で確認）の両系統を扱い、失敗時は終了コード1
  - 旧テスト `tests/test_feat013_render_keypoints.py` を削除し、`tests/test_feat015_render.py` を新規作成（13件、全成功）
  - 手動テスト（実機GPU）でカメラ位置・向き・構図がGTと一致、floater除去効果を確認。GTより左右視野がわずかに狭いが、ピンホール（歪み再現はスコープ外）に起因する仕様内の差として完了

### 2026-06-12

- **feat-012**: camera_pose.py カメラ名・出力先のCLIオプション化
  - `--camera`（必須）でカメラオブジェクト名、`--output`（任意、デフォルト `data/<カメラ名>_poses.json`）で出力先を指定可能に（`blender ... --python camera_pose.py -- --camera <名前>` 形式）
  - カメラがシーンに不在・非カメラ型・フレーム範囲不正の場合はエラー終了（シーン内カメラ一覧を stderr に表示）
  - KIRIモディファイヤの一時無効化を try/finally 化し、元の表示状態に復元するよう修正
  - テスト追加: `tests/test_feat012_camera_pose_args.py`（8件、全成功）

### 2026-06-07

- **feat-011**: visualize_points_2d.py 基準点番号ラベル表示オプション
  - `--label` オプションで各基準点の近傍に番号（ObjectName の最後の数字列、例: `基準_01` → `01`）を緑色・黒縁取りで描画
  - 画像端でははみ出さない位置にラベルをクランプ
  - 未指定時は従来と完全同一の動作（後方互換）
  - テスト追加: `tests/test_feat011_point_labels.py`（10件、全成功）

### 2026-04-28

- **feat-010**: 2D座標を静止画上にプロットして可視化（`phase0/visualize_points_2d.py`）
  - `points_2d.csv` の入力ミスを目視確認するための新規スクリプト
  - config.yaml に `image_<camera_name>` キーで画像パスを記述、対象カメラの2D点を元画像に重ねて描画
  - `<入力画像>_annotated.<拡張子>` として同ディレクトリに保存
- **bug-001**: visualize_points_2d.py のパス解決を既存スクリプトと統一
  - `points_2d` および `image_<camera_name>` のパスを config ファイルのあるディレクトリ基準で解決するよう修正

### 2026-03-23

- **feat-009**: 三角測量による外部パラメータ検証（`phase0/verify_triangulation.py`）
  - 推定結果TOMLと2D観測データから、カメラペアごとに三角測量で3D座標を復元
  - 既知3D基準点座標との差（ユークリッド距離）でカメラパラメータの妥当性を検証
  - カメラペア別・カメラ別のサマリー表示

- **feat-008**: estimate_camera_params.py 複数カメラ一括推定
  - `target_camera` にカンマ区切りで複数カメラを指定可能に（K既知モードのみ）
  - `--output` オプションで推定結果を1つのTOMLファイルにまとめて出力
  - TOML出力を `_format_toml_section` 共通関数化（標準出力とファイル出力の数値精度を統一）
  - 通常モード（K未知）で複数カメラ指定時のエラーチェックを追加

### 2026-03-21

- **feat-007**: render.py MP4ファイル保存機能
  - `--mp4` オプションでPNG連番の代わりにMP4動画として出力
  - NVENCが利用可能ならハードウェアエンコード、利用不可なら libx264 にフォールバック
  - `--mp4-fps` でフレームレート指定（デフォルト30fps）

- **feat-006**: render.py ドライランモード
  - `--dry-run` オプションでPNG保存をスキップし、レンダリング性能を計測可能に
  - 通常モード・dry-run両方で処理時間を表示

- **feat-005**: render.py フレーム範囲指定オプション
  - `--start-frame` / `--end-frame` でレンダリングするフレーム番号の範囲を指定可能に
  - 未指定時は全フレーム（従来動作）

- **feat-004**: gsplatバッチレンダリングパイプライン（`phase4/render.py`）
  - PLYファイルとカメラポーズJSONを受け取り、gsplat（CUDA）でフレームごとにPNG画像を出力
  - Blender→OpenCV座標系変換、PLY属性の活性化関数変換（sigmoid, exp, normalize）
  - `--rotate-z90` オプション: Blenderアドオン旧バージョンのバグによるZ軸90度ずれを補正
  - `--background` オプション: 背景色指定

### 2026-03-20

- **feat-001**: 内部パラメータ既知での外部パラメータ推定ツール `estimate_extrinsic.py` を新規作成（Stage 1）
  - Calib_scene.toml形式のTOMLファイルから内部パラメータを読み込み、R, tのみを推定
  - solvePnPRansac + solvePnP(ITERATIVE) の二段構えで外れ値検出・精密化
  - 依存関係に `tomli` を追加

- **feat-002**: `estimate_camera_params.py` に広角レンズ用8係数歪みモデルを追加
  - `--wide` オプションで8係数（k1, k2, p1, p2, k3, k4, k5, k6）の歪みモデルを使用可能に
  - TOML/CSV出力の8係数対応、solvePnP初期値推定のエラーハンドリング追加

- **feat-003**: `estimate_extrinsic.py` を `estimate_camera_params.py` に統合（Stage 2）
  - `--intrinsic-toml` オプションでK既知モード（R, tのみ推定）を追加
  - 重複関数を `common.py` に切り出し
  - `estimate_extrinsic.py` を削除


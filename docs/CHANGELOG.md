# CHANGELOG

## リリース履歴

### 2026-07-01

- **feat-019**: FPS頭部追従カメラのポーズ書き出しスクリプト（ヘッドレス対応）
  - `phase4/fps_camera_pose.py` を新規作成（`camera_pose.py` はコンストレイント駆動 .blend 向けに温存）。FPS風頭部追従カメラの向き（顔の一次視線方向）を各フレームで計算・適用してから `cam.matrix_world` を読み、`blender -b`（ヘッドレス）でも頭部追従した `c2w` を JSON 出力する
  - 背景: 向きを与える `frame_change_post` ハンドラは .blend に保存されず `-b` の別プロセスで発火しないため、位置は追従するが向きが凍結した `c2w` が出ていた（対象 `data/Blender/2D-Lift.blend` で裏取り確定）。同じ計算をスクリプトに内蔵して解消
  - 姿勢計算（Frankfurt平面ベース）: 耳中点→鼻を両目軸に直交化した前方 `f`、`u=f×r0`（頭頂 `Head-Neck` で符号確定）、`b=-f`、`r=u×b`、`rot=Matrix((r,u,b)).transposed()` を `to_euler()`。ボーン位置は評価済み depsgraph（`arm.evaluated_get`）から取得、アンカー回転適用後に `view_layer.update()` で子カメラへ反映
  - 検証（FR-004/FR-005）: カメラ/アーマチュア/アンカーの存在・型、`cam.parent==anchor`、カメラのローカル回転ゼロを起動時にチェック。各フレームで縮退・行列式・直交性・NaN/Inf、および `cam.matrix_world` の位置=両目中点・向き=計算値の整合を検証し、違反時は JSON を書かず `exit(1)`
  - 出力は既存 `camera_pose.py` と同一スキーマ（`frame,c2w,fx,fy,cx,cy,width,height`）。原子的書き出し（一時ファイル→`os.replace`）。CLI: `--camera`（必須）/`--armature`/`--anchor`/`--output`
  - 実行環境は Blender 4.5.5 LTS（`/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender`）

### 2026-06-25

- **feat-018**: NPZ→C3D変換スクリプト（Blender io_anim_c3d 取り込み対応）
  - `phase4/npz_to_c3d.py` を新規作成。2Dキーポイントを3Dにリフトアップした NPZ（`x3d_world` (F,22,3) world座標メートル、`frame_ids`、`joint_names`）を、Blenderの C3D インポートアドオン `io_anim_c3d` で取り込める C3D に変換する CLI（Blender非依存、NumPy + py-c3d 0.6.0）
  - 座標規約は `render_keypoints.c3d_to_calib`（raw (px,py,pz)mm → (pz,px,py)×0.001 m）と互換。world `(X,Y,Z)` m を C3D raw `(Y,Z,X)×1000` mm で書き出し、読み戻し誤差 6e-8 m
  - 関節は NPZ の22ラベルをそのまま `POINT:LABELS` に書き出し（Halpe26整形はしない）。`MANUFACTURER:SOFTWARE` は未設定（io_anim_c3d のラベル間引きを回避）
  - 各点の residual は座標が有限なら 0.0（有効）、NaN/Inf なら -1.0（無効）
  - C3Dフレーム番号は **1始まり連番（1..F）**。py-c3d 0.6.0 のヘッダ first/last frame が 16bit で、65535超の絶対 `frame_ids`（本データ145599〜）を `set_start_frame` に渡すと読み戻しが破綻するため（実測: first=145595/count=301）。絶対番号はC3Dに保持せず対応をログ出力
  - Blender正立: `POINT:UNITS='mm'`、`POINT:X_SCREEN='+Z'`/`POINT:Y_SCREEN='+Y'`。io_anim_c3d は pose bone のローカル座標 + +Z向きボーンの rest 行列 `B=Rx(+90)` で描画するため、表示鉛直は `Y_SCREEN` で決まる。`+Y` で表示鉛直 = world鉛直 `Z_w`（恒等表示で正立）
  - 安全策: 出力パスは拡張子 `.c3d` 必須・入力NPZと同一パスは拒否。書き出しは一時ファイル→py-c3d読み戻し検証（フレーム数・ラベル一致）→`os.replace` のアトミック確定
  - CLI: `npz_to_c3d.py <npz_path> [--output PATH] [--fps 30.0]`（`allow_abbrev=False`）
  - 新規依存なし（c3d は既に phase4 で使用）。`render_keypoints.py` 非改変
  - テスト: `tests/test_feat018_npz_to_c3d.py` 新規12件（座標ラウンドトリップ・メタデータ・出力パス検証・軸解釈・検証エラー）。T-3 は**実物の io_anim_c3d コード + ボーン rest 行列 + 単位換算**で正立を検証（手計算モデルの落とし穴を排除）
  - Codexレビュー計6サイクル（高: フレーム番号16bit破綻・入力上書き・旧方針残存・world鉛直軸の誤認を解消）。手動テストで Blender 取り込みの正立を確認。**軸設計で2回（`-Y`/`+X`）上下逆を出したが、原因は pose-bone rest 行列の見落とし。実物コード実行＋過去実測の再現で `+Y` を確定**

### 2026-06-14

- **feat-017**: render_keypoints.py 全フレーム対応（連番PNG + MP4）
  - `phase4/render_keypoints.py` を、C3Dの先頭フレーム1枚のみから**全フレーム**（または `--start-frame`/`--end-frame` で指定した範囲、両端含む）の描画に拡張。連番PNG（`frame_<C3Dフレーム番号:06d>.png`）と MP4 を出力
  - `load_c3d_first_frame` を `load_c3d_all_frames`（全フレーム読み込み、`(labels, frames_data, point_rate)` を返す）に置き換え。`point_rate` は `getattr`+`try/except` で安全取得し取得不能時は `0.0`
  - 新規関数 `start_ffmpeg`（`render.py` のffmpegパイプ方式を移植。NVENC→libx264フォールバック。fpsは小数のまま `-r` に渡す）
  - カメラ固定のため背景レンダリング（RGB・深度・α）は**ループ前に1回だけ**計算し全フレームで共有（gsplat呼び出しはフレーム数によらず1回）
  - `--mp4` 時はffmpegをフレームループ前に起動（不在時はPNGも出さず終了コード1）。各フレームをRGB化して `stdin` に書き込み、`try/finally` でプロセスをクリーンアップ。`stdin.write` の `BrokenPipeError`/`OSError` を捕捉してffmpeg途中終了に対応
  - MP4 fps決定: `--mp4-fps`（float、小数保持）＞ C3D rate ＞ 30フォールバック（警告）。29.97/59.94 等の非整数rateでも動画の実時間がずれない
  - CLI変更（破壊的）: 単一PNG用の `--output` を廃止し連番PNG用の `--output-dir`（既定 `./data/keypoints_<カメラ名>/`）に置換。`--start-frame`/`--end-frame`/`--mp4`/`--mp4-fps` を追加。`allow_abbrev=False` で旧 `--output` の前方一致誤マッチを防止（1フレームだけ確認したい場合は `--start-frame N --end-frame N` で代替）
  - 新規依存なし。`render.py` 非改変
  - テスト: `tests/test_feat017_all_frames.py` 新規13件、`tests/test_feat015_render.py` のスタブを `load_c3d_all_frames` 署名・`--output-dir` に追従修正（全99件成功、1件は実データなしでskip）
  - Codexレビュー3サイクルで高1・中6を解消（fpsのfloat保持・point_rate取得の堅牢化・MoSCoW矛盾・ffmpeg不在時の挙動統一・プロセスクリーンアップ）。手動テスト（実機GPU）で全フレームの連番PNG/MP4出力を確認

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


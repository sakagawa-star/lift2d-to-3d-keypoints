# CHANGELOG

## リリース履歴

### 2026-08-13

- **feat-033**: render_keypoints.py の YAML 設定ファイル読み込み
  - `--config <YAML>` を追加。フラット `key: value` の簡易パーサー（feat-029 と同方式）で16キー（ply_path/toml_path/keypoints_path/camera/near_plane/output_dir/background/no_occlusion/occlusion_margin/start_frame/end_frame/mp4/mp4_fps/no_png/no_keypoints/distort）を読み込む。優先順位は CLI 明示指定 > 設定YAML > 既定値（フラグ系は CLI から true 方向のみ上書き可）。`background` はスペース区切り float 3個
  - 汎用部品（`load_yaml_flat` / `_yaml_bool` / `parse_config_yaml`）は `render_fps_video.py` の feat-029 実装を import 再利用（`parse_config_yaml` に省略可能引数 `converters` を追加。省略時は従来挙動で feat-029 テスト無変更）。必須3項目（ply_path/toml_path/camera）は CLI か設定YAMLのどちらかで指定すればよく、統合後に検証する（欠落時のエラー文言は「必須項目が未指定です」形式に変更、終了コード2は維持）
  - テスト: `tests/test_feat033_config_yaml.py` 新規21件（全体回帰 425 passed / 1 skipped、既存テストは無変更で全件パス）。実データで設定YAMLのみでの実行と `--mp4-fps 25` の CLI 上書きを確認。Codex レビュー2回で高・中ゼロ収束（codex-01〜02）。実装は Sonnet サブエージェント委任。手動テスト合格（2026-08-13）

- **feat-032**: render_keypoints.py の NPZ 入力対応
  - キーポイント入力として C3D に加え NPZ（`x3d_world` world座標[m]、`npz_to_c3d.py` 入力と同一フォーマット）を直接読み込めるようにした。拡張子 `.npz`（大小無視）で自動判別。`npz_to_c3d.py` の `load_npz` / `world_to_c3d_raw` を再利用して C3D raw(mm) に変換し、既存パイプラインへ無変更で合流（往復変換は恒等、feat-018 で確立）。NPZ→C3D の事前変換工程が不要に
  - `pnp_ok` は参照せず全フレーム描画（有効性は座標の有限性のみで判定、NaN 関節は描画スキップ。ユーザー決定 2026-08-13）。フレーム番号は NPZ の絶対 `frame_ids`（連番PNG名・`--start-frame`/`--end-frame` とも）。NPZ はレート情報が無いため `--mp4-fps` 未指定時は fps=30 + 警告
  - 位置引数 `c3d_path` を `keypoints_path` に改名。NPZ 読み込み例外の正規化（zip 破損・切り詰め等も終了コード1）と、C3D 経路を含むファイル不存在時のメッセージ化（トレースバック廃止）
  - テスト: `tests/test_feat032_npz_input.py` 新規17件（11項目、異常系7ケース parametrize。全体回帰 404 passed / 1 skipped）。実データ（session001 300フレーム NPZ、22/28マーカー）で MP4 生成を確認。Codex レビュー2回で高・中ゼロ収束（codex-01〜02）。実装は Sonnet サブエージェント委任。手動テスト合格（2026-08-13）

### 2026-08-11

- **feat-031**: render_fps_video.py レンダリング情報テキストの自動保存
  - MP4 一括生成モードの連結成功時に `<MP4名>_info.txt`（例: `test_fps.mp4` → `test_fps_info.txt`）を常時自動保存（オプション不要）。記載10項目: 動画ファイル名・総所要時間・描画フレーム平均・総フレーム数・フレームレート・NaN黒フレーム数/番号・縮退黒フレーム数/番号・解像度
  - 黒フレーム番号は NPZ の frame_id 基準・連続区間圧縮（例 `145609-145613, 145699`、0件は `なし`）。値はすべて実行ログと同一ソース（NaN/縮退は main の既存マスクから導出、平均は既存 `format_render_average`＝並列時は実効秒/フレーム）
  - 書き出しは `durable_replace` で耐久化。連結成功後にのみ書き、書き出し失敗は終了コード1（MP4 は保持）。再開実行でも保存され、所要時間・平均は「その実行の」値（全チャンクスキップ時は `N/A（描画0フレーム）`）
  - テスト: `tests/test_feat031_info_file.py` 新規13件（純関数10件 + `_run_mp4_mode` 統合3件。全体回帰 387 passed / 1 skipped）。GPU実機確認3件（NaN 入りNPZ での区間圧縮表記・再開時の上書きを実確認）。Codex レビュー2回で高・中ゼロ収束（codex-01〜02）。実装は Sonnet サブエージェント委任。手動テスト合格（2026-08-11）

- **feat-030**: render_fps_video.py のチャンク並列レンダリング（feat-027 旧FR-006 の後続）
  - `--gpus <カンマ区切りGPU IDリスト>` を追加（YAML キー `gpus` も対応、`--gpu`/`gpu` と排他、MP4モード専用）。要素数=ワーカー数、同一IDの繰り返しで同一GPUに複数ワーカー（例 `0,0`）、複数GPUは `0,1,...`。未指定時は従来どおり直列（完全後方互換、既存テスト無修正で全件通過）
  - 方式: multiprocessing（spawn）のワーカー子プロセスへチャンクを動的分配（タスクキュー）。チャンク書き出し経路（render_chunk 正常系・durable_replace・ffprobe 検査・連結）は無改変流用で出力等価。`gpus` はマニフェスト非対象のため直列⇔並列・ワーカー構成変更をまたぐ再開が可能
  - 失敗時即時中止: ワーカー例外・異常死（exitcode 監視）で全ワーカー停止→終了コード1→`.tmp` 掃除（完成済みチャンクは保持、再実行で再開）。ワーカーは SIGTERM→KeyboardInterrupt 変換で ffmpeg を確実に後始末し、exitcode 0 は正常終了のみ
  - render_chunk の KeyboardInterrupt 捕捉節を修正（stdin クローズ→terminate→タイムアウト付き wait→kill）: 実機確認で SIGTERM 時に ffmpeg が stdin パイプ読み取りブロックのまま終了せずワーカーが永久待機するデッドロックを発見・解消（design §4.7、実装フェーズの想定外事象として調査・設計・レビューを経て修正）
  - 事前調査（実験 e1、criteria lock 方式）: 単一GPU（RTX 5060 Ti）で2プロセス同時実行の合計スループット R2=1.487（Go 基準 1.30）を確認してから設計。実装後のGPU実機確認でスループット比 1.469・直列→並列の再開互換・ワーカー kill 時の即時中止/`.tmp` なし/ffmpeg 残存なし/再開可を確認
  - 性能: ローカル `--gpus 0,0` で直列比約1.47倍（フルNPZ 19.4万フレームの見込み 約71分→約48分。実測は 2627秒＝約44分、実効 0.013秒/フレーム）
  - A100 サーバー（GPU 7枚）で `--gpus 5,5,6,6`（GPU 5・6 に各2ワーカー）の正常動作を確認（2026-08-11、Closed 後の追加確認）。ローカルでは検証できなかった非0GPU ID・複数GPU分配・GPUあたり複数ワーカーの3経路が実機で確認された
  - テスト: `tests/test_feat030_parallel_render.py` 新規14件（全体回帰 374 passed / 1 skipped）。Codex レビュー7回（criteria 2回 + 設計4回 + §4.7 追補1回）で高・中ゼロ収束。実装は Sonnet サブエージェント委任。手動テスト合格（2026-08-11）

### 2026-08-10

- **feat-027**: NPZ直読みによる一人称視点動画一括生成（Blender廃止）
  - 新規 `phase4/render_fps_video.py`: 巨大NPZ（`npz_to_c3d.py` 入力と同一フォーマット）を直読みし、頭部7点（LEye/REye/LEar/REar/Nose/Head/Neck）からFPSカメラポーズを numpy で計算（`fps_camera_pose.py` の数式移植。位置=両目中点、向き=一次視線）、3DGS（PLY）を gsplat でレンダリングして1本のMP4を生成。Blender・C3D 工程を廃止
  - 内部パラメータは Calib_scene.toml 型 TOML（`--camera` 選択、ピンホール・D無視）。NaN・縮退フレームは黒画面でタイムライン維持（縮退は警告ログ）。単一GPU直列（複数GPU並列は人レビューで後続案件に分割）
  - 信頼性: チャンク（既定10000フレーム）単位の区間MP4 + ffmpeg concat 連結。確定書き出しは全経路 fsync 込みの耐久リネーム（`durable_replace`）、再開時は ffprobe メタデータ検査で破損チャンクを検出して作り直し（ファイル存在だけで完成扱いしない）。エンコーダは libx264 固定（A100 は NVENC 非搭載）で起動前に使用可否を検査
  - 排他モード: `--still-range`（frame_id 範囲の連番PNG）と `--dump-poses`（c2w の JSON ダンプ、GPU不要）
  - 等価性検証（FR-012・案A）: 旧 Blender パイプラインとの c2w 照合で合格。a1（既存 .blend、300フレーム）位置差 max 1.2e-07 m・回転角度差 max 1.5e-02°、a2（filter_npz 平滑化済みNPZを共通出発点に実運用フロー〔npz_to_c3d → io_anim_c3d → リグ再構築〕でデータ再構築、Blender 4.5.5）位置差 max 6.0e-08 m・回転角度差 max 1.6e-02°（いずれも基準 1mm / 0.1°、criteria lock 方式。`docs/issues/feat-027-fps-video-from-npz/experiments/`）
  - 手動テスト: 正しい PLY（HandaiHospital-20251024-01）+ `Calib_FPSCamera.toml` で MP4 生成、a2 の Blender シーン目視確認、K を統一した新旧動画の目視比較（だいたい同じ=ポーズ残差 max 0.6px 級 + エンコーダ差で説明可能）で合格（2026-08-10）
  - テスト: `tests/test_feat027_render_fps_video.py` 新規37件。Codex レビュー20回（codex-01〜20。設計10回 + 実装後訂正 + 実験 criteria）で高・中ゼロ収束。実装は Sonnet サブエージェント委任
- **feat-029**: render_fps_video.py の YAML 設定ファイル読み込み
  - `--config <YAML>` を追加。フラット `key: value` の簡易パーサー（phase0 と同方式、新規ライブラリなし）で15キー（ply_path/npz_path/toml/camera/fps/output/gpu/chunk_size/crf/preset/overwrite/keep_chunks/still_range/still_dir/dump_poses）を読み込む。優先順位は CLI 明示指定 > YAML > 既定値
  - 値には CLI と同一の型検証を適用（型変換失敗・範囲違反・未知キー・bool不正・config連鎖はすべてキー名つきエラー）。必須項目は CLI/YAML の統合後に検証。`--config` なしの挙動は完全後方互換
  - テスト: `tests/test_feat029_config_yaml.py` 新規24件（全体回帰 355 passed / 1 skipped）。手動テストで YAML 経由と CLI 直接指定の出力 MP4 が MD5 完全一致を確認し合格（2026-08-10）。Codex レビュー3回で高・中ゼロ収束（codex-01〜03）。実装は Sonnet サブエージェント委任

### 2026-08-07

- **feat-028**: NPZキーポイントの時間方向平滑化スクリプト
  - 新規 `phase4/filter_npz.py`: 規約NPZ（`x3d_world`/`frame_ids`/`joint_names`、`npz_to_c3d.py` 入力と同一フォーマット）を Butterworth 2次 filtfilt（ゼロ位相、カットオフ既定 6.0 Hz）で時間方向に平滑化し、新しいNPZに書き出す。feat-027（NPZ直読みFPS動画一括生成）の前段として C3D・Blender 工程なしで平滑化できるようにするもの
  - NaN 無効サンプルは feat-020 と同一方式: `--max-gap`（既定10）以下のギャップは線形補間をフィルタ計算にのみ使用、超えるギャップはセグメント分割、10サンプル未満のセグメントは平滑化スキップ（ユーザー確定仕様）、出力では NaN を NaN のまま維持
  - 追加キー（`pnp_ok`, `coord_system` 等）は無加工で出力にコピー（`pnp_ok` の意味はデータ作成者に確認中のため解釈しない）。`x3d_world` の dtype は入力と同一（計算は float64）。`--fps` は必須引数（NPZ に記録がなく、誤った既定値は平滑化強度を黙って変えるため）
  - 原子的書き出し + 読み戻し検証（キー集合・形状・dtype・非有限マスク・frame_ids・joint_names・追加キーの入力一致を毎回検証）
  - テスト: `tests/test_feat028_filter_npz.py` 新規18件。全体 294 passed / 1 skipped（`tests/results/feat-028_test_result.txt`）
  - 実データ確認: 194,279フレーム・22関節でフレーム間変位 11.8→4.6 mm。feat-020 `filter_c3d.py` との2経路出力比較で数値等価を確認（差は float32 丸めのみ、最大 0.17 µm。`docs/issues/feat-028-npz-temporal-filter/experiments/c3d-npz-equivalence/`）
  - Codex レビュー3サイクルで高・中ゼロ収束（codex-01〜03）。実装は Sonnet サブエージェント委任。手動テスト合格（2026-08-07）

### 2026-08-03

- **update-002**: 「予測→実行→照合」プロトコルの開発ルール組み込み
  - CLAUDE.md 開発方針に「実験・検証の進め方（予測→実行→照合）」を新設。数値判定を伴う実験・検証に、判定基準の事前定義（criteria lock）・フェーズ直前の数値予測と実測照合・事後解釈での Go/No-Go 判定禁止を義務付け
  - feat-026 で試験導入した運用（M1 の「条件付きGo」事後解釈の撤回を起点に M2〜M4・M3b で確立）の正式ルール化
  - 適用対象は数値判定を伴う実験・検証に限定（通常の実装・pytest・数値判定を伴わない調査は対象外）
- **update-001**: 開発ドキュメントテンプレート改訂の取り込み（update-XXXフロー導入）
  - CLAUDE.md に「ドキュメント更新フロー（update-XXX 案件）」を新設（README.md 調査 + design.md 反映設計の2点構成、Codex 3観点レビュー、テスト不要）。ドキュメント作成ルール・案件ディレクトリ構成・Codex レビュー節を案件種別 update に対応
  - bug フローの明確化（記録先は案件フォルダの README.md、requirements/design 変更時の保存・レビュー対象化）
  - BUGFIX_STANDARD.md: エラーメッセージ（完全なトレースバック）・修正コード（前後）の記録を必須項目に追加、ステップ4〜5 を Codex 再帰レビュー運用に整合
  - REVIEW_CRITERIA.md: テンプレート版で全置換（観点の汎用化、ドキュメントレビューの重要度分類 1.10 を新設）
  - .gitignore: `.claude/settings.local.json` と `.claude/handovers/` を ignore に追加し、追跡済み分（settings.local.json 1件 + handovers 14件）を追跡解除（ワークツリー・git 履歴には残存）
  - 反映元: DEV_TEMPLATE/template コミット 715db11。取り込み選別・Codex レビューは `docs/issues/update-001-adopt-dev-template/`

### 2026-08-02

- **feat-026**: レンダリング＋自動マッチングによる外部パラメータ自動リファイン
  - 新規 `phase4/refine_extrinsics.py`: 手動プロット（一意6点以上・全点使用）+ 3DGS レンダ + LoFTR 自動マッチングで、K既知の外部パラメータをカメラ一括で精緻化。サンプリング型判定（3チェーン×20サンプルの pooled 合意 f_c≥0.7、二峰は手動点仲裁）で受理し、Calib_scene.toml 型の新規 TOML と診断レポートを出力（入力は不変・非対象カメラも保持）
  - 新規 `matcher_lab/loftr_cli.py`: LoFTR（kornia 0.8.3、outdoor 重み、1280x720、conf≥0.2）推論 CLI。オフライン動作（重み sha256 検証）・決定的・CUDA OOM は終了コード42
  - 新規 uv 環境 `matcher_lab/`（Python 3.12 / torch cu130。LoFTR が Python≥3.11 必須のため phase4 と分離し subprocess 連携）
  - 検証フェーズ（criteria lock 方式、詳細は `docs/issues/feat-026-render-match-extrinsic-refinement/`）: M1 少点数PnP分布 → M2 合成収束域（50cm/10°）→ M3 実写 Go → M3-1/M3-2/D3 サンプリング型判定・二峰仲裁の確立 → M4 檻 SIFT No-Go → M4-1 深度ゲート再較正 No-Go → M3b/M4-2 で LoFTR への変更により病院・檻とも Go（ゲート後 N は SIFT 比 20〜100倍、檻の深度除外率も 60〜75%→28〜32% に半減）
  - 手動テスト: 病院5カメラ 受理5/5（単峰 f_c=1.000、手動点PnP の外れ値検出が M3-0a 名簿と完全一致）、檻6点 受理（M4-2 実測と全数値一致=決定論の確認）
  - ステップ7 差し戻し1件: 入力点数「ちょうど6点」→「6点以上・全点使用」（病院の既存多点 CSV を無加工で使用可能に。investigation.md イテレーション1）
  - テスト: `tests/test_feat026_refine.py` 新規（クラスタ分析・逆写像・ゲート分類・TOML出力・多点PnP/仲裁）。全体 276 passed / 1 skipped（`tests/results/feat-026_test_result.txt`）。ルート pyproject.toml に pytest `testpaths=["tests"]` を追加（実験ファイルの誤収集を解消）
  - Codex レビュー: criteria/スクリプト/要求・設計/差し戻しの全段で再帰レビュー収束（codex-56〜71）。実装は Sonnet サブエージェント委任、成果は「診断値ベースの運用受理」に限定（真値精度は主張しない）

### 2026-07-28

- **bug-004**: estimate_camera_params.py Codex コードレビュー（方式2）と指摘対応
  - bug-003 修正後のコードに対する Codex 第三者コードレビューで検出した重要度「高」3件・「中」2件を修正
  - (H-1) `--output` に入力TOMLと同じパスを指定すると入力を破壊できた問題: 同一パス（`resolve()` 比較）を関数入口でエラー化し、書き込みは `.tmp` 経由の原子的書き込みに変更
  - (H-2) config / `--intrinsic-toml` / `--output` のパス種別未検証: `is_file()` 検証と `--output` のディレクトリ・親ディレクトリ検証を `_run_extrinsic_estimation` に集約（CLI・関数直呼びの両経路を保護）
  - (H-3) 内部パラメータ TOML の値の形状未検証: `matrix`（3x3）・`distortions`（長さ4/5/8/12/14）・`size`（2要素の正の有限数）を検証し、不正はメッセージ＋終了コード1
  - (M-1) K既知モードで `--wide --zero-tangent` 併用が feat-023 要求（警告して無視）に反しエラー終了していた問題: 排他チェックを K未知モード限定に修正
  - (M-2) TOML の `fisheye = true` を無視して通常歪みモデルで誤処理していた問題: 明示的な未対応エラーに変更（feat-001 requirements/design、feat-003 design に変更注記を追記。README にも制約を記載）
  - 正常系（正しいパス・正しいTOML・fisheye なし/false）の出力は修正前と同一
  - テスト: `tests/test_bug004_toml_output_guards.py` 新規23件（同一パス防止・原子性・パス種別・形状検証・排他条件・fisheye、CLI/直呼び両経路）。全体 248 passed / 1 skipped（`tests/results/bug-004_test_result.txt`）
  - Codexレビュー4サイクル（コードレビュー1回 + 修正計画3回。高: 関数直呼び経路のガード漏れ・`--output` 親ディレクトリ検証、中: fisheye 仕様変更の正式化を解消）。実装は Sonnet サブエージェントが investigation.md 準拠で実施。手動テストで同一パス指定時のエラーと入力TOML無傷を確認

### 2026-07-27

- **bug-003**: estimate_camera_params.py 通常モードの入力・収束ガード不足
  - コードレビュー（REVIEW_CRITERIA.md 準拠）で検出した重要度「高」1件・「中」3件を修正。通常モード（K未知）の異常系が feat-002 FR-6 の方針（日本語メッセージ＋終了コード1）に反しトレースバック終了していた問題
  - (a) マッチ点数が `MIN_POINTS_NO_DIST`（6点）未満なら明示エラーで終了（従来: 4〜5点で `least_squares` の ValueError、0〜3点で cv2.error。未使用だった同定数の実装漏れも解消）
  - (b) `least_squares` の収束失敗（`status <= 0`）時に警告を表示（処理は継続、出力・終了コードは不変）
  - (c) config 必須キー（target_camera/points_3d/points_2d）の欠落・空文字、CSV の不在・ディレクトリ指定を通常モード・K既知モードの両方でロード前に検証しエラー終了
  - (d) 内部パラメータ TOML のセクション内キー（matrix/distortions/size）欠落は即終了コード1（複数カメラ中1台の破損でも成功扱いにしない）。セクション自体の不在は従来どおり警告＋スキップ
  - 正常系（6点以上・入力完備・収束成功）の出力は修正前と完全同一。実データ config_E0085.yaml で同一結果（RMSE 1.77px）を確認
  - テスト: `tests/test_bug003_input_guards.py` 新規21件（点数境界・関数/CLI両経路のキー検証・K既知経路・TOML破損 vs 不在の区別・収束警告の monkeypatch 検証）。全体 225 passed / 1 skipped（`tests/results/bug-003_test_result.txt`）
  - Codexレビュー3サイクル（高: 要求への紐付け不足・TOML破損時に終了コード1にならない矛盾、中: is_file 検証・K既知経路のテスト不足を解消）。実装は Sonnet サブエージェントが investigation.md 準拠で実施

- **feat-025**: estimate_camera_params.py 推定パラメータの範囲チェック（警告出力）
  - K未知モードの推定結果に対し、物理的に妥当な範囲を外れたパラメータを警告表示する `[パラメータ範囲チェック]` セクションを再投影誤差の直後に追加。過去に p1=0.5 以上の異常値に長期間気づけなかった問題（RMSE が小さくてもパラメータ相殺で個々の値が異常になり得る）への対策
  - チェック項目（S = max(幅, 高さ)）: fx, fy ∈ [0.3S, 3.0S]、fx/fy 比 ∈ [0.9, 1.1]、cx/cy = 画像中心 ±10%（主点推定時のみ）、|k1|,|k2|,|k3| ≤ 1.0（`--wide` 以外）、|p1|,|p2| ≤ 0.01（`--zero-tangent` 以外。`--wide` でもチェック）。閾値は定数として定義
  - 警告時は対処ヒントを表示（`--wide` 時は併用不可の `--zero-tangent` を勧めず `--fix-center`/標準モデル切り替えを提示）。表示のみで終了コード・TOML/CSV 出力は不変。K既知モードは対象外
  - テスト: `tests/test_feat025_param_range_check.py` 新規17件（`check_param_ranges` ユニット14件 + 合成データによる `run_estimation` 統合3件）。全体 204 passed / 1 skipped
  - Codexレビュー3サイクル（高: FR-002/003 の自動テスト不足、中: `--wide` 時ヒントの矛盾・併用不可組み合わせの記述曖昧を解消）。実装は Sonnet サブエージェントが design.md 準拠で実施

### 2026-07-23

- **feat-024**: render_keypoints.py 歪みモデル対応レンダリング（GT比較用）
  - `estimate_camera_params.py` の推定結果を視覚検証するため、`--no-keypoints`（静止画モード: 3DGS背景のみの `still_<カメラ名>.png` を1枚出力、`c3d_path` 省略必須）と `--distort`（TOML歪み係数による歪みモデルレンダリング、静止画モード専用）を追加
  - 歪みは gsplat 1.5.3 の 3DGUT 経路（`with_ut=True, with_eval3d=True, packed=False` + `radial_coeffs`/`tangential_coeffs`）を採用。事前スパイクで feat-013 の「UT経路=黒い靄」が `near_plane=0.01` の floater との交絡だったことを実証し（UT+ゼロ歪みはピンホールと画素差0.81/255で一致）、方式を確定。`render_image` の誤った docstring 記述も修正
  - `distortions_to_gsplat` で TOML `distortions`（長さ4/5/8）を radial 6要素 + tangential 2要素に詰め替え。`render_image` は `distort=False` を末尾引数追加で既存呼び出し互換
  - 静止画モードで無意味な7オプション（--mp4/--mp4-fps/--no-png/--start-frame/--end-frame/--no-occlusion/--occlusion-margin）は `parser.error()` で明示拒否。`--occlusion-margin` は default=None 化で明示指定を厳密検出（検証後に既定値0.05を補完）。動画モードの動作は変更前と完全同一
  - E0085 実データで検証: 歪みONで基準_018/051（ピンホールでは画角外）が2D観測値と約2px以内の位置に画面内描画されることを確認（feat-023 の起点問題を視覚的に解決）
  - テスト: `tests/test_feat024_distort_render.py` 新規20件（係数変換・バリデーション・rasterization引数のフェイクtorch/gsplat注入検証・出力ファイル名）。全体 187 passed / 1 skipped
  - Codexレビュー: スパイク計画1サイクル（高: packed=False 必須の見落とし）+ 本設計2サイクル（中4件: 上書き方針・バリデーション順序・係数事前検証・rootテスト環境）。実装は Sonnet サブエージェントが design.md 準拠で実施

### 2026-07-22

- **bug-002**: fps_camera_pose.py デフォルトアーマチュア名変更にテスト・ドキュメント未追随
  - ユーザーが手動で行った仕様変更（`DEFAULT_ARMATURE_NAME` を `session001_f145749_world300` → `E00000`。現用 .blend のアーマチュア改名に追随）を正式に取り込み、未追随だったテスト・ドキュメントを同期。`uv run pytest -v` が常に1件失敗する状態を解消
  - 変更: テスト期待値を `E00000` に更新、feat-019 の requirements/design/README に変更注記、参考資料に冒頭注記、CLAUDE.md の既定値記述を更新。`fps_camera_pose.py` docstring と feat-019 ドキュメント・CLAUDE.md の実行例パスを現存しない `2D-Lift.blend` から実在する `session001_world_22pt.blend` に更新
  - 後方互換: 旧名アーマチュアの .blend をデフォルト実行すると構成検証で exit(1) する明示的な非互換（移行手順: `--armature session001_f145749_world300` を明示指定）。現用 .blend に旧名は含まれず（バイナリ検索で0件）、旧名デフォルト運用は現存しない
  - 検証: pytest 全件 167 passed / 1 skipped（`tests/results/bug-002_test_result.txt`）。Blender 4.5.5 ヘッドレスで `session001_world_22pt.blend` を `--armature` 省略実行し exit 0・9000フレームのポーズJSON生成を確認
  - Codexレビュー3サイクル（高: bugfix/仕様変更の位置づけ・feat-019文書の一貫性・実機確認の欠如、中: 後方互換影響の明記・docstring実行例の取り残しを解消）。実装は Sonnet サブエージェントが investigation.md 準拠で実施

- **feat-023**: estimate_camera_params.py 接線歪みゼロ固定オプション（--zero-tangent）
  - 通常モード（K未知）で接線歪み係数 p1, p2 を 0 に固定し放射歪み（k1, k2、`--k3` 併用時は k3 も）のみを推定する `--zero-tangent` フラグを追加。E0085-01 で画像左端の基準点（基準_018/051）の残差を接線歪みが吸収し p2=0.052 という物理的にありえない値に収束した問題（OpenCV `CALIB_ZERO_TANGENT_DIST` 相当の対策）
  - 投影関数4つ（`project_dist2`/`project_dist3` の主点固定・主点推定版）と最小点数定数4つ（DIST2: 13/10、DIST3: 14/11）を追加。既存8分岐には触れず、`--zero-tangent` なしの動作・出力は変更前と完全同一
  - `--fix-center`/`--k3` と併用可（10〜13変数の4モード）。`--wide` とは併用不可でエラー終了（終了コード1）、`--intrinsic-toml` 指定時は既存フラグ同様に警告して無視
  - 出力互換: TOML `distortions` は既存の4（k3時5）要素レイアウトのまま p1, p2 位置に 0.0（`convert_toml_to_csv.py`・K既知モード入力の互換維持）。`run_estimation` の `zero_tangent` 引数は末尾追加で既存呼び出し互換
  - E0085 実データ確認: p2 異常が解消し樽型放射歪み（k1=-0.232, k2=0.281）の妥当な解に収束（RMSE 2.85→3.86 px）
  - テスト: `tests/test_feat023_zero_tangent.py` 新規15件（投影関数一致・4モード真値復元・併用バリデーション・最小点数境界4種・既存経路回帰なし）
  - Codexレビュー3サイクル（中: run_estimation 引数順の互換性・点数不足時のk3出力形式の矛盾・Must範囲テスト不足を解消）。実装は Sonnet サブエージェントが design.md 準拠で実施

### 2026-07-03

- **feat-022**: render_keypoints.py --no-png オプション（MP4のみ出力）
  - `--mp4` 時も連番PNGが無条件に保存され、`cv2.imwrite`（zlib圧縮、フルHD 1枚あたり約30〜80ms）が数万フレーム処理の支配的ボトルネックになっていた問題への対処。`--no-png --mp4` で連番PNG保存をスキップしMP4のみを出力する
  - `--no-png` 単独指定（`--mp4` なし）は出力が何もなくなるため、重い処理（C3D/PLY読み込み）より前に `parser.error()` で拒否（終了コード2）
  - 非破壊方針: 過去の実行で出力ディレクトリに残った `frame_*.png` は削除・上書きしない（この実行で新規生成しないことのみ保証）
  - PNGスキップ時の進捗表示は `[i/n] frame <C3Dフレーム番号> -> mp4` 形式。`--no-png` なしの出力・表示は変更前と完全に同一（後方互換）
  - テスト: `tests/test_feat022_no_png.py` 新規7件（パース/バリデーション3件＋モックによる実体検証4件: imwrite 0回・ffmpeg書き込みフレーム数分・進捗行に `.png` なし・既存PNG非破壊）
  - Codexレビュー3サイクル（高: 残存PNGと受け入れ基準の矛盾、中: 実体検証テストの欠如・変更対象1ファイル制約とテスト追加の矛盾を解消）。実機手動テストで確認済み

### 2026-07-02

- **feat-021**: render_keypoints.py 欠損マーカー許容（22点C3D対応）
  - `phase4/render_keypoints.py` を、C3D に Halpe26 の全26マーカーが揃っていなくても実行できるように変更。リフトアップ推定由来の22点C3D（足先6点なし、Spine/Thorax あり。`npz_to_c3d.py` 出力形式）が「Halpe26マーカーが不足しています」の ValueError で描画できなかった問題を解消
  - 既知マーカーを `KEYPOINT_NAMES`（Halpe26 26点 + Spine/Thorax の計28点）に拡張。C3Dに存在しない既知マーカーは `valid=False` として点・ボーンを描画スキップ（`extract_halpe26` → `extract_keypoints` にリネーム。例外を投げない）
  - 体幹ボーンは C3D の構成で切り替え（`build_skeleton(present)` 新設、`HALPE26_SKELETON` 定数は廃止）: Spine/Thorax 両方あれば Neck–Thorax–Spine–Hip の3本、片方のみは経由2本、無ければ従来の Neck–Hip 直結。体幹ボーンは従来と同位置に挿入し描画順を維持（26点C3Dの描画内容は変更前と同一。pytest でスケルトン完全一致・extract同値性を検証）
  - 起動時にマーカー構成を報告（`キーポイント: N/28 マーカーを描画対象とします` + 欠損一覧）。既知マーカー0個の場合のみ PLY/torch ロード前にエラー終了（終了コード1）
  - CLI引数は無変更。`draw_overlay` に `skeleton` 引数を追加
  - テスト: `tests/test_feat021_flexible_markers.py` 新規9件（extract 22点/26点・build_skeleton 各構成・描画順・FR-005 main早期エラー経路）。`tests/test_feat016_keypoints.py` を新APIに追随
  - Codexレビュー1サイクル（中: 既存テスト追随のスコープ漏れ・FR-005テスト欠如・バイト一致基準の検証手段を解消）。手動テストで22点C3D実データの完走・描画を確認

- **feat-020**: C3Dキーポイントの時間方向平滑化スクリプト
  - `phase4/filter_c3d.py` を新規作成。リフトアップ推定由来のC3Dキーポイント（フレーム間ジッターで"カクカク"）を、2次Butterworthローパス + `scipy.signal.filtfilt`（実効4次・ゼロ位相）で時間方向に平滑化し、新しいC3Dとして書き出すCLI（Blender・GPU不要、phase4 venv）
  - パイプライン設計: `fps_camera_pose.py` 内でのフィルタ案は却下し、C3D→C3D の独立前処理とした。平滑化済みC3Dは Blender io_anim_c3d 取り込み（→ `fps_camera_pose.py`）と `render_keypoints.py` の全下流に一括で効く
  - フィルタ: カットオフ周波数 `--cutoff`（既定 6.0 Hz、モーキャプ慣用値）1個で強さを調整。サンプリング周波数はC3Dの point rate から取得（欠損時のみ `--rate` で補完。有効rateとの併存はエラー）
  - 無効サンプル（residual<0/NaN）: `--max-gap`（既定10フレーム）以下のギャップのみ線形補間してフィルタ計算に使い、超えるギャップはセグメント分割して独立フィルタ（長い欠損の直線補間値がfiltfilt経由で有効サンプルを汚染するのを防ぐ）。出力では無効サンプルを無効のまま維持（補間値を有効値として書かない）。長さ10未満のセグメントはスキップ＋警告
  - 入力規約検証: 対象は本プロジェクト規約のC3D（feat-018出力）に限定。POINT:UNITS=mm、X_SCREEN=+Z/Y_SCREEN=+Y（欠落もエラー）、first_frame 1〜65534（py-c3d 16bit制約）を検証し、規約外はエラー終了（単位・軸の暗黙変更によるデータ破損防止）
  - 安全策: 出力パス拡張子 `.c3d` 必須、入力との同一パス拒否（realpath + samefile でsymlink経由の同一実体も拒否）、一時ファイル→読み戻し検証→`os.replace` のアトミック書き出し
  - 依存追加: phase4 に `scipy>=1.11`（1.15.3 導入）。`docs/TECH_STACK.md` 更新
  - テスト: `tests/test_filter_c3d.py` 新規14件（フィルタ特性・セグメント分割・無効サンプル維持・C3D往復・全バリデーション）。実行は `uv run --project phase4 --with pytest pytest tests/test_filter_c3d.py -v`
  - Codexレビュー計4サイクル（高: メタデータ維持矛盾・無制限ギャップ補間・スクリーン軸欠落許容、中: 16bit制約・makedirs("")・symlink判定を解消）
  - 手動テストで「フィルタ後C3Dが90度回転して見える」報告があったが、調査（素通しバイト一致・メタデータ一致・Procrustes回転角0.000度）で `filter_c3d.py` 起因ではないと確定。Blender新規ファイルへの取り込み直しで位置一致をユーザー確認（`docs/issues/feat-020-c3d-temporal-filter/investigation.md`）

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


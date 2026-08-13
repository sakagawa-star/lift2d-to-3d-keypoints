# BACKLOG

## 案件一覧

| ID | タイプ | タイトル | ステータス |
|----|--------|---------|-----------|
| feat-001 | 機能追加 | 内部パラメータ既知での外部パラメータ推定（Stage 1） | Closed |
| feat-002 | 機能追加 | estimate_camera_params.py 広角レンズ対応 | Closed |
| feat-003 | 機能追加 | estimate_extrinsic.py を estimate_camera_params.py に統合（Stage 2） | Closed |
| feat-004 | 機能追加 | gsplatバッチレンダリングパイプライン | Closed |
| feat-005 | 機能追加 | render.py フレーム範囲指定オプション | Closed |
| feat-006 | 機能追加 | render.py ドライランモード | Closed |
| feat-007 | 機能追加 | render.py MP4ファイル保存機能 | Closed |
| feat-008 | 機能追加 | estimate_camera_params.py 複数カメラ一括推定 | Closed |
| feat-009 | 機能追加 | 三角測量による外部パラメータ検証 | Closed |
| feat-010 | 機能追加 | 2D座標を静止画上にプロットして可視化 | Closed |
| bug-001 | 不具合 | visualize_points_2d.py のパス解決が既存スクリプトと不整合 | Closed |
| feat-011 | 機能追加 | visualize_points_2d.py 基準点番号ラベル表示オプション | Closed |
| feat-012 | 機能追加 | camera_pose.py カメラ名・出力先のCLIオプション化 | Closed |
| feat-013 | 機能追加 | 3DGSレンダリング画像への3Dキーポイント重ね描き | 中止（feat-015/016 に作り直し） |
| feat-014 | 機能追加 | ピンホール3DGSレンダリング（PNG出力、GT比較） | 中止（feat-015 に作り直し） |
| feat-015 | 機能追加 | ピンホール3DGSレンダリング（PNG出力、GT比較） | Closed |
| feat-016 | 機能追加 | キーポイントのオクルージョン（深度による前後判定） | Closed |
| feat-017 | 機能追加 | render_keypoints.py 全フレーム対応（連番PNG + MP4） | Closed |
| feat-018 | 機能追加 | NPZ→C3D変換スクリプト（Blender io_anim_c3d 取り込み対応） | Closed |
| feat-019 | 機能追加 | FPS頭部追従カメラのポーズ書き出しスクリプト（ヘッドレス対応） | Closed |
| feat-020 | 機能追加 | C3Dキーポイントの時間方向平滑化スクリプト | Closed |
| feat-021 | 機能追加 | render_keypoints.py 欠損マーカー許容（22点C3D対応） | Closed |

| feat-022 | 機能追加 | render_keypoints.py --no-png オプション（MP4のみ出力） | Closed |
| feat-023 | 機能追加 | estimate_camera_params.py 接線歪みゼロ固定オプション（--zero-tangent） | Closed |
| bug-002 | 不具合 | fps_camera_pose.py デフォルトアーマチュア名変更にテスト・ドキュメント未追随 | Closed |
| feat-024 | 機能追加 | render_keypoints.py 歪みモデル対応レンダリング（GT比較用） | Closed |
| feat-025 | 機能追加 | estimate_camera_params.py 推定パラメータの範囲チェック（警告出力） | Closed |
| bug-003 | 不具合 | estimate_camera_params.py 通常モードの入力・収束ガード不足 | Closed |
| bug-004 | 不具合 | estimate_camera_params.py Codex コードレビュー（方式2）と指摘対応 | Closed |
| feat-026 | 機能追加 | レンダリング＋自動マッチングによる外部パラメータ自動リファイン | Closed |
| update-001 | ドキュメント更新 | 開発ドキュメントテンプレート改訂の取り込み（update-XXXフロー導入） | Closed |
| update-002 | ドキュメント更新 | 「予測→実行→照合」プロトコルの開発ルール組み込み | Closed |
| feat-027 | 機能追加 | NPZ直読みによる一人称視点動画一括生成（Blender廃止。複数GPU並列は後続案件に分割） | Closed |
| feat-028 | 機能追加 | NPZキーポイントの時間方向平滑化スクリプト（feat-027の前段） | Closed |
| feat-029 | 機能追加 | render_fps_video.py の YAML 設定ファイル読み込み（長いCLIの省力化） | Closed |
| feat-030 | 機能追加 | render_fps_video.py のチャンク並列レンダリング（feat-027 旧FR-006 の後続。単一/複数GPU対応の `--gpus`） | Closed |
| feat-031 | 機能追加 | render_fps_video.py レンダリング情報テキストの自動保存（`<MP4名>_info.txt`） | Closed |
| feat-032 | 機能追加 | render_keypoints.py の NPZ 入力対応（拡張子判別、C3D と併存） | Closed |
| feat-033 | 機能追加 | render_keypoints.py の YAML 設定ファイル読み込み（--config、feat-029 の展開） | Closed |
| feat-034 | 機能追加 | render_keypoints.py FPSカメラ視錐台ワイヤフレームの重ね描き（オクルージョン考慮） | Closed |

# feat-029: render_fps_video.py の YAML 設定ファイル読み込み

## ステータス

Closed（2026-08-10 手動テスト合格）

## 概要

`phase4/render_fps_video.py` の CLI オプションは必須引数が多く（PLY・NPZ・--toml・--camera・--fps）、毎回長いコマンドを打つ必要がある。オプション内容を YAML ファイルに書いて読み込ませる機能を追加し、コマンド入力を短くする。

- 方式はユーザー指定で **YAML 形式**（2026-08-09 ヒアリングで確定。argparse @ファイル方式・TOML 方式は不採用）
- 対象は `render_fps_video.py` のみ（他スクリプトへの展開は本案件のスコープ外）

## 依存関係

- feat-027（`render_fps_video.py` 本体）。feat-027 は手動テスト仮合格・保留中だが、本案件はオプション入力系のみの追加であり並行可能

## 経緯

- 2026-08-09: 起票。実現方式のヒアリングで YAML 形式を確定（優先順位=CLI が YAML を上書き、パーサー=簡易フラットパーサー・新規ライブラリなし）
- 2026-08-09: requirements.md / design.md 作成。Codex レビュー3回で高・中ゼロ収束（reviews/codex-01〜03。指摘: YAML数値の型変換失敗の未捕捉クラッシュ、公開関数の戻り値仕様の揺れ→いずれも反映）。人レビュー承認
- 2026-08-09: Sonnet 委任で実装完了。新規テスト `tests/test_feat029_config_yaml.py` 24件成功、feat-027 テスト37件成功のまま（後方互換 NFR-001 確認）、全体回帰 355 passed / 1 skipped
- 2026-08-10: 手動テストで、YAML 指定（`test_fps.mp4`）と CLI 直接指定（`test_fps_bak.mp4`）の出力 MP4 が **MD5 で完全一致**することを確認。ただしユーザー判断により、**旧 Blender パイプライン映像との比較（feat-027 側の検証）が完了するまで手動テスト合格としない**。合格保留のまま feat-027 の実験 a2 の完了を待つ
- 2026-08-10: feat-027 側の検証（実験 a2 合格・新旧動画の目視比較）が完了し、**手動テスト合格**。完了処理を実施しクローズ

## 参照

- `phase4/render_fps_video.py` — 対象スクリプト（feat-027）
- `phase0/estimate_camera_params.py` ほか — プロジェクト既存の簡易YAML読み込み（`load_yaml_simple`、フラット `key: value`）の前例

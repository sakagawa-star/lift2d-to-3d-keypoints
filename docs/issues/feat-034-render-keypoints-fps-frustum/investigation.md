# feat-034 手動テスト差し戻しの調査・修正計画

基準: `docs/BUGFIX_STANDARD.md`（開発フロー ステップ7 で発見された問題の差し戻し運用）

## イテレーション1 (2026-08-13)

### 1.1 問題の特定

- **分類**: 要求仕様作成時のヒアリング漏れ（実装は現行仕様どおりに動作しており、実装バグではない）
- **対応する要求ID**: FR-003（FPSカメラの内部パラメータ取得）
- **対応する設計セクション**: design.md 4.3-3（FPSカメラ選択）、4.6（CLI）、7 D-10（K は位置引数 `toml_path` と同じ TOML から取得）
- **現在の動作**: `--fps-camera` は位置引数 `toml_path` で渡した TOML 内のカメラ名しか指定できない。FPSカメラの内部パラメータが**別の TOML ファイル**にある場合、視錐台の FOV をそのカメラで指定する手段がない
- **期待する動作**（ユーザー指摘 2026-08-13 手動テスト）: FPSカメラの内部パラメータを観察カメラとは別の TOML ファイルから読み込めること
- **エラーメッセージ**: なし（エラーになる不具合ではなく、機能の入力手段の欠落）

### 1.2 原因分析

- **原因箇所**: requirements.md FR-003 の入力定義（「既存のキャリブレーションTOML（位置引数 `toml_path` と同じファイル）内のカメラから取得する」）と design.md D-10（別TOML指定オプションを設けないと明示決定）
- **原因の説明**: 壁打ちヒアリング（2026-08-13）で「カメラのパラメーターはTOMLファイルから取得する」と確定した際、取得元ファイルが観察カメラと同一か別かを確認しなかった。設計では D-10 で「同一キャリブファイルに入っている」前提を置いたが、この前提をユーザーに確認していなかった
- **根本原因 or 表面的原因**: 根本原因（要求定義の入力仕様が不足。コードは仕様に忠実）

### 1.3 修正内容

- **変更対象ファイル**:
  1. `docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md` — FR-003 に `--fps-toml` を追加、FR-006 に設定キー `fps_toml` を追加、FR-007 に検証を追加、CLI 仕様例を更新（変更案は本計画と同時に反映済み。差分は requirements.md を参照）
  2. `docs/issues/feat-034-render-keypoints-fps-frustum/design.md` — 4.3-3 / 4.6 / 4.7 / 4.8 / 6（テスト）/ 7 D-10 を改訂（同上）
  3. `phase4/render_keypoints.py` — `--fps-toml` オプションの追加（実装はドキュメントレビュー収束・ユーザー承認後）
  4. `tests/test_feat034_fps_frustum.py` — テスト追加（同上）
- **変更しないファイル**: `render_fps_video.py`（引き続き import のみ）。既存の観察カメラの TOML 読み込み経路（`load_cameras_toml(args.toml_path)`）は変更しない
- **修正の要点**（コード変更の設計は design.md 改訂版が正）:
  - `--fps-toml <パス>` を追加（default: None）。省略時は従来どおり `toml_path` の TOML から `--fps-camera` を検索する（後方互換）
  - 指定時は `load_cameras_toml(args.fps_toml)` で**別途読み込んだ**辞書から `select_camera(…, args.fps_camera)` で選択する
  - `--fps-toml` は `--fps-frustum` 専用（なしで指定はエラー、終了コード 2）
  - ファイル不存在・TOMLパース不能は stderr 表示のうえ終了コード 1
  - 設定YAMLキー `fps_toml`（文字列パス）を追加
- **修正コード**（意図伝達用。最終仕様は design.md 改訂版が正）:

  `_build_parser()` — 修正前:

  ```python
  parser.add_argument("--fps-camera", default=None,
                      help="視錐台のFOVに使うTOML内カメラ名（--fps-frustum 専用）")
  ```

  修正後（`--fps-toml` を追加）:

  ```python
  parser.add_argument("--fps-camera", default=None,
                      help="視錐台のFOVに使うTOML内カメラ名（--fps-frustum 専用）")
  parser.add_argument("--fps-toml", default=None,
                      help="--fps-camera を検索するTOMLファイル（--fps-frustum 専用、"
                           "省略時: 位置引数 toml_path と同じファイル）")
  ```

  `CONFIG_CONVERTERS` — 修正前:

  ```python
  "fps_frustum": _yaml_bool, "fps_camera": str, "frustum_depth": float,
  ```

  修正後:

  ```python
  "fps_frustum": _yaml_bool, "fps_camera": str, "fps_toml": str, "frustum_depth": float,
  ```

  `main()` の組み合わせ検証 — 修正前:

  ```python
  if not args.fps_frustum:
      for name, given in [("--fps-camera", args.fps_camera is not None),
                          ("--frustum-depth", args.frustum_depth is not None)]:
  ```

  修正後（`--fps-toml` を追加。`--no-keypoints` の forbidden リストにも `("--fps-toml", args.fps_toml is not None)` を追加する）:

  ```python
  if not args.fps_frustum:
      for name, given in [("--fps-camera", args.fps_camera is not None),
                          ("--fps-toml", args.fps_toml is not None),
                          ("--frustum-depth", args.frustum_depth is not None)]:
  ```

  `main()` の FPSカメラ選択 — 修正前:

  ```python
  fps_cam = None
  if args.fps_frustum:
      try:
          fps_cam = select_camera(cameras, args.fps_camera)
      except ValueError as e:
          print(e, file=sys.stderr)
          return 1
  ```

  修正後（検索先辞書を分岐。エラーメッセージ文言は design.md 4.3-3 が正）:

  ```python
  fps_cam = None
  if args.fps_frustum:
      if args.fps_toml is None:
          fps_cameras = cameras  # 従来どおり toml_path の辞書（後方互換）
      else:
          try:
              fps_cameras = load_cameras_toml(args.fps_toml)
          except FileNotFoundError:
              print(f"エラー: --fps-toml のファイルが見つかりません: {args.fps_toml}",
                    file=sys.stderr)
              return 1
          except tomli.TOMLDecodeError as e:
              print(f"エラー: --fps-toml をTOMLとして解釈できません: {args.fps_toml}: {e}",
                    file=sys.stderr)
              return 1
      try:
          fps_cam = select_camera(fps_cameras, args.fps_camera)
      except ValueError as e:
          print(e, file=sys.stderr)
          return 1
  ```

- **修正が設計書に沿っているか**: 設計書自体の変更を伴う（D-10 の決定を改訂）。設計書の変更案を同時に保存する（BUGFIX_STANDARD 2.2 準拠）

### 1.4 影響範囲

- **他の機能への影響**: `--fps-frustum` 機能内に閉じる。`--fps-toml` 省略時の動作は現行実装と同一（FPSカメラ選択の分岐が1つ増えるのみ）。観察カメラ・キーポイント・レンダリング経路は無変更
- **リグレッションリスク**: FPSカメラ選択部の分岐追加による既存テスト（T-10 ほか）への影響。既存テストは `--fps-toml` を渡さないため挙動不変を全件回帰で確認する

### 1.5 確認方法

- **自動テスト**（design.md 6章に追加する T-20〜T-22）:
  - T-20: 観察カメラ TOML と別の TOML（K・解像度が異なるカメラ入り）を `--fps-toml` に渡し、`compute_frustum_vertices` に渡る K・解像度が別TOML側のカメラと一致する（monkeypatch 捕捉、T-19 と同方式）
  - T-21: `--fps-toml` を `--fps-frustum` なしで指定 → SystemExit(2)
  - T-22: `--fps-toml` に不存在パス → 戻り値 1・stderr にパス表示 / パース不能ファイル → 戻り値 1
  - 回帰: 既存テスト全件パス
- **手動テスト**: 実データで `--fps-toml <別TOML> --fps-camera <そのTOML内カメラ名>` を指定して実行し、開始サマリに該当カメラ名・解像度が表示され、視錐台が描画されることを確認する

# bug-002 調査・修正計画

## イテレーション1 (2026-07-22)

### 0. 本案件の位置づけ（bugfix + 仕様変更の取り込み）

本件は「コードのバグ」ではなく、ユーザーが 2026-07-22 に手動で行った仕様変更
（`DEFAULT_ARMATURE_NAME` を `"E00000"` へ変更）に対し、テスト・要求仕様書・設計書・
CLAUDE.md が未追随である不整合の解消である。ユーザーは同日の対話で
「変更を正式採用してテストを追随させる」方針を明示的に承認済み。

BUGFIX_STANDARD.md 2.2「修正によって設計書の変更が必要になる場合は、設計書の変更案も
併せて提示する。設計書を変更せずにコードだけ変えてはならない」に従い、本 investigation.md
で feat-019 の要求仕様書・設計書の変更案を併せて提示し、承認を得た上で一括で反映する。
（要求仕様に該当記述が存在するため feat 案件ではなく bug 案件として扱い、仕様変更の
承認記録は本ドキュメントが担う。）

### 1.1 不具合の特定

- **対応する要求ID**: feat-019 requirements.md の CLI 仕様
  「`--armature <name>`（任意、デフォルト `session001_f145749_world300`）: アーマチュア名」
  （`docs/issues/feat-019-fps-camera-pose/requirements.md:77`）
- **対応する設計セクション**: feat-019 design.md の定数定義
  `DEFAULT_ARMATURE_NAME = "session001_f145749_world300"`
  （`docs/issues/feat-019-fps-camera-pose/design.md:57`）
- **現在の動作**: `phase4/fps_camera_pose.py:24` の `DEFAULT_ARMATURE_NAME` が
  `"E00000"` に変更済み（未コミット）。テスト
  `tests/test_feat019_fps_camera_pose_args.py:110` は旧値を期待しているため、
  `uv run pytest -v` で `TestConstants::test_default_names` が
  `AssertionError: assert 'E00000' == 'session001_f145749_world300'` で失敗する。
  CLAUDE.md:105 の `--armature（既定 session001_f145749_world300）` も旧値のまま。
- **期待する動作**: 変更後仕様（デフォルト `"E00000"`。セクション0の承認に基づく）を
  正とし、feat-019 requirements.md / design.md をセクション1.3 の変更案どおり更新した上で、
  テスト・CLAUDE.md を新仕様に一致させる。`uv run pytest -v` が本件起因の失敗なしで
  完走する。

### 1.2 原因分析

- **原因箇所**: `phase4/fps_camera_pose.py:24`（仕様変更済みコード）と
  `tests/test_feat019_fps_camera_pose_args.py:110`（旧仕様を期待するテスト）の不整合
- **原因の説明**: コードの手動編集が開発フロー（ドキュメント先行）の外で行われ、
  テスト・ドキュメントとの同期が取れていないため
- **根本原因 or 表面的原因**: 根本原因は「仕様変更がドキュメント・テストに未反映であること」。
  本修正はドキュメント・テストを承認済みの新仕様に同期させることで根本解決となる

### 1.3 修正内容

- **変更対象ファイル**:
  1. `phase4/fps_camera_pose.py` — 2つの扱いに分けて変更する:
     - **定数（既存の未コミット変更の正式な取り込み）**: `DEFAULT_ARMATURE_NAME` は旧値
       `"session001_f145749_world300"` から `"E00000"` に変更済み（ユーザー編集）。
       追加編集なしでこの状態をコミット対象に含める
     - **モジュール docstring（本件での追加修正）**: 冒頭の実行例が現存しない
       `data/Blender/2D-Lift.blend` を参照しているため、実在する
       `data/Blender/session001_world_22pt.blend` に更新する（変更対象8のドキュメント
       実行例更新と同内容をコード側にも適用）
  2. `tests/test_feat019_fps_camera_pose_args.py:110` —
     `assert DEFAULT_ARMATURE_NAME == 'session001_f145749_world300'` を
     `assert DEFAULT_ARMATURE_NAME == 'E00000'` に変更（1行）
  3. `docs/issues/feat-019-fps-camera-pose/requirements.md:77` —
     デフォルト値を `E00000` に変更し、注記
     「（2026-07-22 bug-002: デフォルトを session001_f145749_world300 から E00000 に変更）」
     を付す
  4. `docs/issues/feat-019-fps-camera-pose/design.md:57` —
     定数定義のコード例を `DEFAULT_ARMATURE_NAME = "E00000"` に変更し、同様の注記を付す
  5. `docs/issues/feat-019-fps-camera-pose/README.md:24` —
     アーマチュア名の記述に同様の注記を付す（現在仕様と当時の構成記録が混在しないよう、
     旧名の直後に変更注記を明示）
  6. `docs/issues/feat-019-fps-camera-pose/fps_camera_handler_reference.md` —
     冒頭に1行注記「本資料は開発当時の .blend（アーマチュア session001_f145749_world300）
     に基づく参考資料。2026-07-22 bug-002 でデフォルトアーマチュア名は E00000 に変更された」
     を追加（本文は歴史的記録として変更しない）
  7. `CLAUDE.md:105` — `--armature（既定 session001_f145749_world300）` を
     `--armature（既定 E00000）` に変更
  8. feat-019 ドキュメント内のデフォルト依存の実行例・受け入れ条件
     （`docs/issues/feat-019-fps-camera-pose/requirements.md:167`、
     `docs/issues/feat-019-fps-camera-pose/design.md:324`）および CLAUDE.md の 1b 実行例 —
     対象 .blend パスを現存しない `data/Blender/2D-Lift.blend` から実在する
     `data/Blender/session001_world_22pt.blend` に更新する
     （当該 .blend のアーマチュアは E00000 に改名済みのため `--armature` 省略で
     デフォルト値が機能する。セクション1.5 の確認事実を参照）
- **変更しないファイル**:
  - `tests/test_feat019_fps_camera_pose_args.py` のデフォルト値以外のテスト —
    引数パース等の検証は本変更の影響を受けない
- **修正が設計書に沿っているか**: 既存設計書とは矛盾するため、上記3〜6のとおり
  feat-019 側ドキュメントの変更案を本計画に含めた（BUGFIX_STANDARD.md 2.2 準拠）

### 1.4 影響範囲

- **他の機能への影響**: `fps_camera_pose.py` の既存デフォルト実行ワークフローに対する
  **明示的な後方互換破壊**である。旧名アーマチュア（session001_f145749_world300）の .blend
  を `--armature` 省略で処理すると、アーマチュアが見つからず構成検証で exit(1) する。
  ただし 2026-07-22 の確認で、現用の対象 .blend
  （`phase4/data/Blender/session001_world_22pt.blend`）のアーマチュアは既に E00000 に
  改名済みで旧名は含まれておらず（バイナリ検索で旧名0件・E00000 2件）、旧名でのデフォルト
  実行ワークフローは現存しない。コード内で `DEFAULT_ARMATURE_NAME` を参照するのは
  `phase4/fps_camera_pose.py` の argparse デフォルトと当該テストのみで、他スクリプトへの
  波及はない
- **リグレッションリスク**: 旧名アーマチュアのまま保存された過去の .blend（バックアップ等）
  を将来デフォルト実行した場合に exit(1) する。移行手順は
  `--armature session001_f145749_world300` の明示指定（エラーメッセージで名前不一致が
  判明するため復旧は容易）。この移行手順は feat-019 requirements.md の変更注記にも記載する

### 1.5 確認方法

- **自動テスト**: `uv run pytest -v` を全件実行し、
  `test_feat019_fps_camera_pose_args.py` を含む全テストがパスすること
  （skip を除く）。出力を `tests/results/bug-002_test_result.txt` に保存する
- **事前確認済みの事実（2026-07-22）**: 対象 .blend は
  `phase4/data/Blender/session001_world_22pt.blend`（ユーザー指定）。バイナリ検索で
  `E00000` 2件・`Cam_FPS` 1件・`Cam_Anchor` 2件を含み、旧名
  `session001_f145749_world300` は0件であることを確認済み
- **手動テスト（実機確認）**: 定数テストの書き換えだけでは「デフォルト値が実データで
  機能するか」を検証できないため、Blender ヘッドレスで以下を確認する。
  `phase4/` ディレクトリで実行:
  ```bash
  /home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender \
      -b data/Blender/session001_world_22pt.blend \
      --python fps_camera_pose.py -- --camera Cam_FPS \
      --output /tmp/bug-002_check.json
  ```
  `--armature` **省略**（デフォルト E00000 が使われる）で exit 0 となり、
  `/tmp/bug-002_check.json` が生成されることを確認する。
  カメラ名・アンカー名は .blend の実体（Cam_FPS / Cam_Anchor = feat-019 の既定値）を使う。
  旧名アーマチュアの .blend は現存しないため、旧名明示指定のテストは対象外とする
- **テスト項目（完了判定）**: 上記の自動テスト全件パス + 手動テスト（デフォルト実行の成功）
  の両方が確認できた時点で修正完了とする

# feat-001 ロードマップ: 内部パラメータ既知での外部パラメータ推定

## 背景・目的

現在の `estimate_camera_params.py` はカメラの内部パラメータ（K, 歪み係数）が未知であることを前提としている。チェスボードやChArUcoで内部パラメータを推定済みのカメラに対して、外部パラメータ（R, t）のみを求める機能がない。

K既知であれば推定変数が6個（R, tのみ）に減り、精度・安定性が大幅に向上する。

## 確定事項（設計方針）

Claude Codeが要求仕様書・機能設計書を作成する際のインプットとする。

### ツール構成

- **Stage 1**: `estimate_extrinsic.py` としてミニマムツールを新規作成し、テストする
- **Stage 2**: テスト完了後に `estimate_camera_params.py` に統合する

### 内部パラメータの入力

- **Calib_scene.toml** 形式のファイルから読み込む（K既知のカメラの内部パラメータがTOML形式で保存されているため）
- TOMLファイルのファイル名はアプリ起動時の引数渡し
- TOMLファイル内に複数のカメラのパラメーターが記述されていることを想定すること。config.yamlに記述されているカメラ名(target_camera)のみを対象とすること。
- TOML読み込みには `tomli` ライブラリを使用する（pyproject.toml に依存追加）

### 推定手法

- `cv2.solvePnPRansac` → `cv2.solvePnP(ITERATIVE, useExtrinsicGuess=True)` の二段構え
  - RANSAC: 外れ値（基準点の座標ミス）を自動検出・除外する
  - ITERATIVE精密化: RANSAC結果を初期値にして、inlier点のみで再度推定し精度を上げる
- 基準点が4〜5点の場合: RANSACなしで `solvePnP(ITERATIVE)` のみ実行
- 基準点が3点以下の場合: エラー終了
- **この手法は今回の新機能（K既知→R,t推定）にのみ適用する。既存の `estimate_camera_params.py` は変更しない**

### 出力

- Calib_scene.toml 形式で標準出力（現在の `estimate_camera_params.py` と同じ方式）

### 検証方法

- 再投影誤差（RMSE）で評価する（Ground Truthは無い）
- RANSAC結果（inlier/outlier）を表示して、基準点データの品質も確認できるようにする

## Stage 1: ミニマムツール作成

CLAUDE.md の「機能ごとの開発フロー」に従う。

| ステップ | 内容 | 担当 |
|----------|------|------|
| 1. 案件作成 | `docs/issues/feat-001-estimate-extrinsic/` 作成、BACKLOG.md追加 | Claude Code |
| 2. 調査・計画 | 既存コード（`estimate_camera_params.py`, `phase0_verification.py`）を読み、要求仕様書・機能設計書を作成 | Claude Code |
| 3. ドキュメント保存 | `requirements.md`, `design.md` を案件フォルダに保存 | Claude Code |
| 4. レビュー | Subagent + Atsushi でレビュー | 両方 |
| 5. 実装 | `phase0/estimate_extrinsic.py` 新規作成、pyproject.toml に `tomli` 追加 | Claude Code |
| 6. テスト | pytest作成・実行、結果を `tests/results/feat-001_test_result.txt` に保存 | Claude Code |
| 7. 手動テスト | Atsushiが実機で動作確認 | Atsushi |
| 8. 完了 | BACKLOG, CHANGELOG, CLAUDE.md 更新 | Claude Code |

## Stage 2: estimate_camera_params.py への統合

Stage 1 の完了・実機テスト後に着手する。別案件（feat-002等）として管理する。

### 統合方針

`estimate_camera_params.py` に `--intrinsic-toml` と `--intrinsic-camera` オプションを追加する。

**オプション指定時（K既知モード）:**
- Calib_scene.toml 形式のファイルからK, 歪み係数を読み込む
- TOMLファイルのファイル名はアプリ起動時の引数渡し
- TOMLファイル内に複数のカメラのパラメーターが記述されていることを想定すること。config.yamlに記述されているカメラ名(target_camera)のみを対象とすること。
- Stage 1 の手法（solvePnPRansac → ITERATIVE精密化）でR, tのみを推定する

**オプション未指定時（K未知モード、既存動作）:**
- 以下の動作を一切変更しない:
  - 2D-3D基準点対応から、内部パラメータ（fx, fy, cx, cy）、歪み係数（k1, k2, p1, p2、オプションでk3）、外部パラメータ（R, t）を同時に推定する
  - `cv2.solvePnP` で初期値を生成し、`scipy.optimize.least_squares`（Levenberg-Marquardt法）で全パラメータを非線形最適化する
  - `--fix-center`（主点を画像中心に固定）、`--k3`（k3も推定）の既存オプションが従来通り動作する
  - 基準点数が不足した場合に歪み係数を0に固定するフォールバックが従来通り動作する

### 共通化

`estimate_extrinsic.py` と `estimate_camera_params.py` に重複している関数（load_yaml_simple, load_points_3d, load_points_2d, match_points）を共通モジュールに切り出す。統合完了後、`estimate_extrinsic.py` の廃止を判断する。

### テスト

以下の2種類のテストを作成・実行する。

**新機能テスト（K既知モード）:**
- `--intrinsic-toml` と `--intrinsic-camera` を指定した場合に、TOMLから内部パラメータを読み込み、R, tのみを推定できること
- RANSAC結果（inlier/outlier）が正しく表示されること
- 再投影誤差が出力されること

**リグレッションテスト（K未知モード、既存動作の保護）:**
- `--intrinsic-toml` を指定しない場合に、従来通り全パラメータ（K, 歪み, R, t）が同時推定されること
- `--fix-center`、`--k3` の既存オプションが従来通り動作すること
- `--fix-center` と `--intrinsic-toml` の組み合わせなど、新旧オプションの同時指定時の挙動を定義・テストすること
- 基準点数不足時のフォールバック（歪み係数0固定）が従来通り動作すること

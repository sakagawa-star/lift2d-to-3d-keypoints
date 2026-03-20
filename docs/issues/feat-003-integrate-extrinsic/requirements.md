# feat-003 要求仕様書: estimate_extrinsic.py を estimate_camera_params.py に統合（Stage 2）

## 目的

`estimate_extrinsic.py`（K既知での外部パラメータ推定）の機能を `estimate_camera_params.py` に統合する。重複関数を共通モジュールに切り出す。統合完了後、`estimate_extrinsic.py` を削除する。

## 背景

- feat-001（Stage 1）で `estimate_extrinsic.py` をスタンドアロンツールとして作成・テスト済み
- `estimate_extrinsic.py` と `estimate_camera_params.py` に `load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` が重複実装されている
- 1つのツールに統合し、オプションで切り替える設計にする

## 機能要件

### FR-1: `--intrinsic-toml` オプションの追加

- `--intrinsic-toml <toml_path>` オプションを `estimate_camera_params.py` に追加する
- 指定時（K既知モード）: TOMLファイルからK, 歪み係数を読み込み、R, tのみを推定する
- 未指定時（K未知モード）: 既存動作を一切変更しない
- TOMLファイル内に複数カメラのパラメータが記述されている場合、config.yaml の `target_camera` に一致するセクションのみを使用する（ロードマップの `--intrinsic-camera` オプションは不要。`target_camera` で代用する）
- TOML読み込みには `tomli` ライブラリを使用する（pyproject.toml に追加済み）

### FR-2: K既知モードの推定手法

`estimate_extrinsic.py` と同じ手法を使用する:

- 6点以上: `cv2.solvePnPRansac`（iterationsCount=1000, reprojectionError=8.0）で外れ値検出 → inlier点のみで `cv2.solvePnP(ITERATIVE, useExtrinsicGuess=True)` で精密化
- 4〜5点: `cv2.solvePnP(SOLVEPNP_SQPNP)` のみ（RANSACなし）
- 3点以下: エラー終了（終了コード1）
- `solvePnPRansac` の失敗、`inliers_idx` が `None`、inlier数が4点未満の場合はエラー終了
- `solvePnP` の失敗時もエラー終了

### FR-3: K既知モードの出力

- Calib_scene.toml 形式で標準出力に出力する（既存のK未知モードと同じ出力位置）
- 出力内容: 入力の内部パラメータ（K, 歪み係数）＋ 推定した外部パラメータ（rotation, translation）
- camera_params.csv 形式の出力は行わない（K既知モードでは内部パラメータを推定していないため）
- RANSAC使用時: inlier/outlier の分類結果を表示する
- 再投影誤差（RMSE）を全基準点で計算・表示する
- 各点の再投影誤差を表示する（outlier点には `✗ [outlier]` マークを付ける）

### FR-4: K既知モードと既存オプションの組み合わせ

- `--intrinsic-toml` 指定時は `--k3`, `--wide`, `--fix-center` を全て無視する（K既知モードでは内部パラメータを推定しないため、これらのオプションは無意味）
- 無視したオプションがある場合、警告メッセージを表示する

### FR-5: K既知モードの画像サイズ

- `--intrinsic-toml` 指定時は画像サイズをTOMLの `size` から取得する
- config.yaml の `image_width`, `image_height` は無視する

### FR-6: 既存動作の保持（リグレッション防止）

`--intrinsic-toml` 未指定時の以下の動作を一切変更しない:

- 全パラメータ（K, 歪み, R, t）の同時推定
- `--fix-center`, `--k3`, `--wide` の動作
- 点数不足時のフォールバック（歪み係数0固定）
- 再投影誤差の評価閾値
- TOML/CSV出力フォーマット

### FR-7: 共通モジュールの切り出し

- `load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` を共通モジュール `phase0/common.py` に切り出す
- `estimate_camera_params.py` と `estimate_extrinsic.py` の重複実装を `common.py` からのインポートに置き換える

### FR-8: estimate_extrinsic.py の削除

- 統合完了後、`estimate_extrinsic.py` を削除する
- `estimate_extrinsic.py` のテスト（`tests/test_estimate_extrinsic.py`）は `estimate_camera_params.py` のK既知モードテストに置き換える

## 非機能要件

### NF-1: 変更しないファイル

- `phase0/phase0_verification.py`
- `phase0/convert_toml_to_csv.py`

## コマンドラインインターフェース

```
# K未知モード（既存動作、変更なし）
python estimate_camera_params.py config.yaml
python estimate_camera_params.py config.yaml --fix-center
python estimate_camera_params.py config.yaml --wide

# K既知モード（新規）
python estimate_camera_params.py config.yaml --intrinsic-toml data/ufukui/05520125_intrinsics.toml
```

## テスト

### テスト項目

1. `--intrinsic-toml` でTOMLから内部パラメータを読み込み、R, tのみを推定できること
2. 6点以上でRANSAC + ITERATIVE精密化が実行されること
3. 4〜5点でSQPNPが実行されること
4. 3点以下でエラー終了すること
5. RANSAC結果（inlier/outlier）が正しく表示されること
6. `--intrinsic-toml` と `--k3`/`--wide`/`--fix-center` の同時指定で警告が表示されること
7. `--intrinsic-toml` 未指定時の既存動作が変わらないこと（リグレッション）
8. 共通モジュール（common.py）からのインポートが正しく動作すること
9. TOML出力が正しいフォーマットであること

### テストデータ

- 内部パラメータTOML: `phase0/data/ufukui/05520125_intrinsics.toml`（カメラ名: `cam05520125`）
- config.yaml: `phase0/data/config_lab2.yaml`
- 合成データ（既知のK, R, tから生成）も使用する

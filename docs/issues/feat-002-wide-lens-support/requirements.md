# feat-002 要求仕様書: estimate_camera_params.py 広角レンズ対応

## 目的

既存の `estimate_camera_params.py` に広角レンズ用8係数歪みモデル（k1, k2, p1, p2, k3, k4, k5, k6）を追加する。`estimate_camera_params_wide.py` の8係数関連コードを移植し、CSV出力など参考実装にない部分は新規追加する。

## 背景

- 広角レンズ（画角60〜100°）では標準の4係数歪みモデルでは歪み補正が不十分
- `estimate_camera_params_wide.py` として独立実装済みだが、本体に統合して一元管理する

## 機能要件

### FR-1: `--wide` オプションの追加

- `--wide` オプションを追加し、広角レンズ用8係数歪みモデルを使用可能にする
- OpenCVの歪み係数順序: `[k1, k2, p1, p2, k3, k4, k5, k6]`
- `--wide` と `--k3` が同時に指定された場合、`--wide` を優先し `--k3` は無視する

### FR-2: 8係数歪みモデルの推定

- 主点推定モード: 18変数（fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6, r1, r2, r3, t1, t2, t3）
- 主点固定モード: 16変数（fx, fy, k1, k2, p1, p2, k3, k4, k5, k6, r1, r2, r3, t1, t2, t3）
- 推定手法は既存と同じ: `cv2.solvePnP` で初期値 → `scipy.optimize.least_squares`（method='lm'）で最適化

### FR-3: 最小点数の制御

- 8係数の最小点数:
  - 主点推定: 20点（18変数）
  - 主点固定: 18点（16変数）
- 点数不足の場合は歪み係数を0に固定する（既存動作と同じフォールバック）

### FR-4: 結果出力

#### Calib_scene.toml 形式

- `--wide` 使用時は `distortions` に8要素を出力する
- 8係数出力時に `convert_toml_to_csv.py` との非互換を警告メッセージで表示する

#### camera_params.csv 形式

- `--wide` 使用時は8係数対応のヘッダーとデータを出力する
- ヘッダー: `camera_name,width,height,fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6,r1,r2,r3,t1,t2,t3`

### FR-5: 既存動作の保持（リグレッション防止）

以下の既存動作を一切変更しない:

- `--wide` なしの場合の全動作（4係数、5係数、歪みなし）
- `--fix-center` の動作
- `--k3` の動作（`--wide` 未指定時）
- 点数不足時のフォールバック（歪み係数0固定）。`--wide` 指定時でも点数不足なら既存の歪みなしモードにフォールバックする
- 再投影誤差の評価閾値（< 5px 優秀, < 10px 良好, < 20px 許容範囲, >= 20px 要確認）。`--wide` 使用時も同じ閾値を使用する（参考実装 `estimate_camera_params_wide.py` では異なる閾値を使用しているが、本体統合時は既存閾値で統一する）
- 各点の再投影誤差の閾値（< 10px で ✓）。`--wide` 使用時も同じ閾値を使用する

### FR-6: solvePnP 初期値推定の失敗ハンドリング

- `cv2.solvePnP` による初期値推定が失敗（`success=False`）した場合、エラーメッセージを出力して終了コード1で終了する
- これは既存コードにない新規追加のエラーハンドリングである
- `run_estimation` 関数は正常終了時に `return 0`、エラー時に `return 1` を返すように変更する
- `main()` 関数は `run_estimation` の戻り値を受け取り、終了コードに反映する

## 非機能要件

### NF-1: 既存コードへの影響

- `phase0_verification.py` は変更しない
- `convert_toml_to_csv.py` は変更しない

## コマンドラインインターフェース

```
python estimate_camera_params.py config.yaml                          # 標準（4係数）
python estimate_camera_params.py config.yaml --k3                     # 5係数
python estimate_camera_params.py config.yaml --fix-center             # 4係数 + 主点固定
python estimate_camera_params.py config.yaml --fix-center --k3        # 5係数 + 主点固定
python estimate_camera_params.py config.yaml --wide                   # 広角8係数（新規）
python estimate_camera_params.py config.yaml --wide --fix-center      # 広角8係数 + 主点固定（新規）
```

## テスト

### テスト項目

1. `--wide` で8係数歪みモデルが使用されること
2. `--wide --fix-center` で8係数 + 主点固定が動作すること
3. `--wide` で点数不足時に歪み係数が0に固定されること
4. `--wide --k3` 同時指定で `--wide` が優先されること
5. TOML出力の `distortions` が8要素であること
6. CSV出力のヘッダーに k3, k4, k5, k6 が含まれること
7. 8係数TOML出力時に非互換警告が表示されること
8. `--wide` なしの場合の既存動作が変わらないこと（リグレッションテスト）
9. solvePnP 初期値推定失敗時にエラー終了すること

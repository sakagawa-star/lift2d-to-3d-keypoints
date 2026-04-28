# 機能設計書: feat-010 2D座標可視化

## ファイル構成

新規追加:
- `phase0/visualize_points_2d.py`

変更なし:
- `phase0/common.py`（既存の `load_yaml_simple`, `load_points_2d` を再利用）
- 既存 `config_*.yaml`（利用時にユーザーが `image_<camera_name>` キーを追記する）

## 依存ライブラリ

- `opencv-python`（既存）: 画像入出力・描画
- `numpy`（既存）

追加インストールは不要。

## 既存共通関数の利用

`phase0/common.py` の以下の関数をそのまま利用する：

- `load_yaml_simple(yaml_path: str) -> dict`
- `load_points_2d(csv_path: str, camera_name: str) -> dict`
  - 戻り値は `{ObjectName: [X, Y]}` 形式で、**呼び出し時点でカメラ名フィルタ済み**

したがって設計上、別途フィルタ関数は不要。

## モジュール構成

### `visualize_points_2d.py`

```
main()
  ├─ argparse で config_path, --camera を受け取る
  ├─ config = load_yaml_simple(config_path)
  ├─ camera_name = resolve_target_camera(config, args.camera)
  ├─ config_dir = Path(args.config).resolve().parent
  ├─ image_path = resolve_image_path(config, camera_name, config_dir)
  ├─ points = load_points_2d(str(config_dir / config['points_2d']), camera_name)
  │     # {ObjectName: [X, Y]} のフィルタ済みdict
  ├─ 0件チェック → エラー終了
  ├─ (描画数, スキップ数) = draw_points_on_image(image_path, points, output_path, config)
  └─ 完了メッセージ出力（出力パス、描画点数、範囲外スキップ数）
```

### 各関数仕様

#### `resolve_target_camera(config: dict, camera_arg: str | None) -> str`

- `camera_arg` が指定されていればそれを返す
- 指定なしのとき:
  - `config['target_camera']` がカンマを含む → エラー終了（`--camera` 必須メッセージ）
  - 単一カメラ名 → それを返す（前後空白は strip）

#### `resolve_image_path(config: dict, camera_name: str, config_dir: Path) -> Path`

- `key = f"image_{camera_name}"` で config から取得
- 存在しなければエラー終了（必要キー名をメッセージで提示）
- 取得した値を `config_dir / value` で解決する（既存スクリプトと同じ慣例。`value` が絶対パスのときは `Path` 演算子の仕様により絶対パスがそのまま採用される）
- ファイル存在チェック（無ければエラー終了）

#### `draw_points_on_image(image_path: Path, points: dict, output_path: Path, config: dict) -> tuple[int, int]`

- `cv2.imread(str(image_path))` で読み込み（None ならエラー終了）
- 画像 `shape` と config の `image_width`/`image_height` を比較。不一致なら標準出力に警告して処理継続
- 各 `(name, [x, y])` について:
  - 整数化
  - 画像範囲外なら警告出力しスキップ
  - `cv2.circle(img, (x, y), radius=6, color=(0, 0, 255), thickness=2)` 等で描画（マーカー意匠は実装裁量）
- `cv2.imwrite(str(output_path), img)` で保存
- 戻り値: `(描画点数, 範囲外スキップ数)`

## 出力パス決定ロジック

```python
output_path = image_path.with_name(f"{image_path.stem}_annotated{image_path.suffix}")
```

例: `data/osaka2/cam05520126.jpg` → `data/osaka2/cam05520126_annotated.jpg`

既存ファイルがあっても上書き保存する。

## エラーハンドリング

| 状況 | 動作 |
|---|---|
| config ファイルが存在しない | エラー終了 |
| `image_<camera_name>` キー欠落 | エラー終了（必要キー名をメッセージで提示） |
| 画像ファイルが存在しない / 読み込み失敗（`cv2.imread` が None） | エラー終了 |
| 対象カメラの2D点が0件（`load_points_2d` の戻り値が空） | エラー終了 |
| 画像サイズが config の `image_width`/`image_height` と異なる | 警告出力（処理継続） |
| 画像範囲外の点 | 警告出力、当該点はスキップ |

## 実行例

```bash
cd phase0

# config の target_camera が単一カメラの場合
uv run python visualize_points_2d.py data/config_osaka2.yaml

# target_camera が複数カメラの場合は --camera 必須
uv run python visualize_points_2d.py data/config_osaka2.yaml --camera cam05520126
```

事前にconfigへ追記（config ファイルのあるディレクトリからの相対パス、または絶対パス）:
```yaml
image_cam05520126: osaka2/cam05520126.jpg
image_cam41520557: osaka2/cam41520557.jpg
```

## テスト方針

- `tests/test_visualize_points_2d.py` を追加
- 小さなダミー画像（例: 100x100、白塗り）と数点の2D点CSVを用意し、関数単位でテスト:
  - `resolve_target_camera`: 単一指定／カンマ複数 + `--camera` なしでエラー／`--camera` 指定で上書き
  - `resolve_image_path(config, camera_name, config_dir)`: 取得成功、キー欠落時のエラー、ファイル不在時のエラー、相対パスが `config_dir` 基準で解決されること、絶対パスが `config_dir` に依存せずそのまま採用されること
  - `draw_points_on_image`: 出力ファイル生成、範囲外カウント、画像shape保持
- 結果は `tests/results/feat-010_test_result.txt` に保存

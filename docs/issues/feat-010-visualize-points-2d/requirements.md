# 要求仕様書: feat-010 2D座標可視化

## 目的

`points_2d.csv` の入力ミスを視覚的に確認するため、指定カメラの2D点を元の静止画に重ねてプロットした画像を生成する。

## スコープ

- 新規スクリプト `phase0/visualize_points_2d.py` を追加する
- 既存スクリプト・既存データ形式（`points_2d.csv`、`config.yaml` の既存キー）は変更しない
- 推定処理（パラメータ計算）は行わない。可視化のみ

## 入力

### コマンドライン引数

```
uv run python visualize_points_2d.py <config.yaml> [--camera <camera_name>]
```

- 第1引数: 設定YAMLファイル（必須）
- `--camera <camera_name>`: 可視化対象のカメラ名（省略時は config の `target_camera` を使用。`target_camera` がカンマ区切り複数指定の場合は `--camera` 必須）

### config.yaml への追加キー

カメラごとに対応する静止画パスを記述する。キー名は `image_<camera_name>`。

```yaml
target_camera: cam05520126
points_2d: points_2d_osaka2.csv
image_width: 1920
image_height: 1080

# 追加キー（カメラごとの画像パス）
image_cam05520126: osaka2/cam05520126.jpg
image_cam41520557: osaka2/cam41520557.jpg
```

- パスは **config ファイルのあるディレクトリからの相対パス、または絶対パス**で指定する（既存 `points_2d` キーと同じ慣例）
- 1カメラずつ実行する想定だが、複数カメラ分のキーを同じconfigに併記して構わない（使用するのは指定カメラのキーのみ）
- 既存configに `image_<camera_name>` が無い場合はエラー終了する

### 既存入力データ

- `points_2d.csv`（既存形式、`ObjectName,camera_name,X,Y`）
- 静止画ファイル（jpg/png 等、OpenCV で読める形式）

## 出力

- 入力画像と同じディレクトリに `<入力画像名のstem>_annotated.<元拡張子>` で保存
  - 例: `data/osaka2/cam05520126.jpg` → `data/osaka2/cam05520126_annotated.jpg`
- 静止画上に、指定カメラに属する2D点（`points_2d.csv` で `camera_name` が一致する全行）をマーカーで描画する
- ラベル（ObjectName）は描画しない

## 描画仕様

- マーカー形状・色・サイズは実装者裁量。視認性を最優先とし、画像サイズに対して点が見える大きさにする
- 画像サイズは元画像のサイズをそのまま使う（リサイズしない）

## エラー・警告動作（要求レベル）

| 状況 | 動作 |
|---|---|
| config ファイルが存在しない | エラー終了 |
| `image_<camera_name>` キー欠落 | エラー終了 |
| 画像ファイルが存在しない / 読み込み失敗 | エラー終了 |
| 対象カメラの2D点が0件 | エラー終了 |
| 画像サイズが config の `image_width`/`image_height` と異なる | 警告出力して処理継続 |
| 画像範囲外の点 | 警告出力、当該点はスキップ（描画しない） |

## 制約

- 簡易YAMLパーサーは `key: value` フラット構造のみ対応。新キーもこの形式に従う

## 品質基準

- 指定カメラの全2D点が画像上に描画されること（範囲外を除く）
- 点座標 (X, Y) は画像ピクセル座標としてそのまま用いる（OpenCV 慣例: 左上原点、X が横、Y が縦）

## 非スコープ

- 3D点の投影描画（推定後の検証は別案件）
- 複数カメラ同時可視化（1カメラずつ実行する）
- ラベル描画
- GUI 表示

# feat-008: estimate_camera_params.py 複数カメラ一括推定 — 要求仕様書

## 目的

config.yaml の `target_camera` に複数カメラを指定し、`--intrinsic-toml` モードで全カメラの外部パラメータを一括推定する。結果を1つのTOMLファイルに出力する。

## 背景

- 同一環境に6台のカメラが配置されている
- 現行は1台ごとに config.yaml の `target_camera` を書き換えて実行する必要がある
- 後続の三角測量検証（feat-009）で、全カメラの推定結果をまとめたTOMLが必要

## 変更対象

`phase0/estimate_camera_params.py` の `--intrinsic-toml` モード（K既知モード）のみ。
通常モード（K未知）は変更しない。

## 入力の変更

### config.yaml

`target_camera` にカンマ区切りで複数カメラを指定可能にする。
`load_yaml_simple` の変更は不要（文字列として取得後にスクリプト側でカンマ分割する）。

```yaml
# 既存（1台）— 引き続き動作する
target_camera: cam05520125

# 新規（複数台）
target_camera: cam05520125, cam05520126, cam05520128, cam05520129, cam41520554, cam41520556
```

### --intrinsic-toml

全カメラの内部パラメータを1つのTOMLファイルにまとめて渡す。
各カメラはTOMLのセクション名（`[cam05520125]` など）で区別される。
既存の `load_intrinsic_toml` はセクション名でカメラを検索する仕組みなので、変更不要。

```bash
# 1台の場合（既存と同じ）
uv run python estimate_camera_params.py data/config_lab2.yaml --intrinsic-toml data/ufukui/cam05520125_intrinsics.toml

# 複数台の場合（全カメラの内部パラメータをまとめた1ファイルを指定）
uv run python estimate_camera_params.py data/config_lab2.yaml --intrinsic-toml data/ufukui/intrinsics_all.toml
```

## 出力の変更

### 標準出力

各カメラの推定結果を順番に表示する。カメラごとに既存と同じフォーマット（ヘッダー、推定手法、RANSAC結果、外部パラメータ、再投影誤差の詳細、TOML形式出力）を繰り返す。

### ファイル出力（新規）

`--output` オプションで出力先TOMLファイルを指定可能にする。
`--output` は `--intrinsic-toml` と併用時のみ有効。`--intrinsic-toml` なしで指定された場合は警告を出して無視する。

```bash
uv run python estimate_camera_params.py data/config_lab2.yaml \
  --intrinsic-toml data/ufukui/intrinsics_all.toml \
  --output data/ufukui/extrinsic_all.toml
```

出力TOMLフォーマット:
```toml
[cam05520125]
name = "cam05520125"
size = [1920.0, 1080.0]
matrix = [[1177.5557, 0.0, 956.5687], [0.0, 1178.0205, 494.813], [0.0, 0.0, 1.0]]
distortions = [-1.9025, 4.5769, -0.0056, -0.0028, -4.4761, -1.8658, 4.5091, -4.4473]
rotation = [0.123456, -0.234567, 0.345678]
translation = [1.234, -2.345, 3.456]
fisheye = false

[cam05520126]
name = "cam05520126"
...
```

## 後方互換性

- `target_camera` に1台だけ指定 + `--intrinsic-toml` にファイルを指定 → 現行と同じ動作
- `--output` を指定しない場合 → 標準出力のみ（現行と同じ）

## 制約

- `--intrinsic-toml` モード（K既知モード）のみが対象。通常モード（K未知）には影響しない
- 通常モード（K未知）で `target_camera` に複数カメラが指定された場合、エラーメッセージを出して終了する
- 1台でも推定に失敗した場合、そのカメラはスキップし警告を出す。他のカメラの推定は続行する
- 全カメラの推定が失敗した場合、`--output` 指定時もファイルは出力しない。エラーメッセージを表示する
- `--output` 指定時、既存ファイルがあれば上書きする
- `--output` 指定時、出力先ディレクトリが存在しない場合はエラーメッセージを表示して終了する
- `target_camera` のパース時、空文字列（`cam01, , cam02` など）はフィルタする。重複するカメラ名もフィルタする

## 品質基準

- 1台指定時の推定結果が現行と完全に一致すること

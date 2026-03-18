# 技術スタック

## プロジェクト基盤

| 項目 | 値 | ソース |
|---|---|---|
| 言語 | Python >=3.10 | pyproject.toml |
| パッケージ管理 | uv（pyproject.toml + uv.lock） | CLAUDE.md |
| 対象OS | 未定義 | — |

## ライブラリ一覧

### 外部ライブラリ（pyproject.toml で管理）

| ライブラリ名 | バージョン | 用途（1行） | 使用箇所（モジュール名） | 選定理由（1行） |
|---|---|---|---|---|
| numpy | >=2.2.6 | 行列演算・数値計算 | estimate_camera_params.py, phase0_verification.py | 未定義 |
| opencv-python | >=4.13.0.92 | solvePnP・projectPoints によるカメラキャリブレーション | estimate_camera_params.py, phase0_verification.py | 未定義 |
| scipy | >=1.15.3 | least_squares（Levenberg-Marquardt法）によるパラメータ最適化 | estimate_camera_params.py, phase0_verification.py | 未定義 |

### Blender内蔵ライブラリ

| ライブラリ名 | バージョン | 用途（1行） | 使用箇所（モジュール名） | 選定理由（1行） |
|---|---|---|---|---|
| bpy | Blender同梱版 | Blenderシーンからの3D基準点座標抽出 | blender/mk_points_3d.py | Blenderスクリプトとして実行するため |

### 標準ライブラリ

| ライブラリ名 | 用途（1行） | 使用箇所（モジュール名） |
|---|---|---|
| argparse | コマンドライン引数の解析 | estimate_camera_params.py, phase0_verification.py, convert_toml_to_csv.py |
| csv | CSV ファイルの読み書き | estimate_camera_params.py, phase0_verification.py, convert_toml_to_csv.py, blender/mk_points_3d.py |
| pathlib | ファイルパス操作 | estimate_camera_params.py, phase0_verification.py, convert_toml_to_csv.py |
| re | TOML ファイルの正規表現パース | convert_toml_to_csv.py, blender/mk_points_3d.py |
| os | ファイルパス操作 | blender/mk_points_3d.py |

## バージョン固定ポリシー

| 項目 | 内容 |
|---|---|
| 管理ファイル | pyproject.toml（定義）、uv.lock（ロックファイル） |
| バージョン指定方針 | `>=下限` 指定（互換性のある最新版を許容） |
| ロック運用 | uv.lock で再現可能なインストールを保証 |

## 制約・禁止事項

- 要求仕様書（docs/issues/）が未作成のため、制約条件の記載なし
- 使用禁止ライブラリの定義なし

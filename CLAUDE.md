# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## セッション引き継ぎ

- セッション開始時にプロジェクトルートの `.claude/handovers/` ディレクトリを確認し、ファイルが存在すれば最新のものを読み込む
- セッション終了時や作業の区切りでは `/handover` の実行を促す

## プロジェクト概要

2D-3D点対応からカメラの内部パラメータ（焦点距離、主点）・外部パラメータ（回転、並進）・歪み係数を推定するシステム。OpenCVのsolvePnPで初期値を求め、SciPyのLevenberg-Marquardt法で最適化する。

### 主な機能

- 2D-3D点対応によるカメラ内部・外部パラメータ推定
- 歪み係数（k1, k2, p1, p2, optional k3）の推定
- Ground Truthとの比較検証（レベル1: K既知、レベル2: K未知）
- Calib_scene.toml / camera_params.csv 形式での結果出力

## 技術スタック

- **言語**: Python 3.10
- **パッケージ管理**: uv（pyproject.tomlで管理）
- **数値計算**: NumPy
- **最適化**: SciPy（least_squares, Levenberg-Marquardt法）
- **カメラキャリブレーション**: OpenCV（solvePnP, projectPoints）
- **3Dモデル**: Blender（.blend / .plyファイル）

## 環境セットアップ

```bash
uv sync
```

## ディレクトリ構成

```
lift2d-to-3d-keypoints/
├── CLAUDE.md
├── README.md
├── pyproject.toml
├── uv.lock
├── docs/                        # ドキュメント（案件管理）
│   ├── BACKLOG.md               # 案件一覧
│   ├── CHANGELOG.md             # リリース履歴
│   └── issues/                  # 個別案件フォルダ
├── phase0/                      # メインスクリプト群
│   ├── estimate_camera_params.py      # カメラパラメータ推定（メイン）
│   ├── estimate_extrinsic.py          # 外部パラメータ推定（K既知）
│   ├── phase0_verification.py         # 検証スクリプト
│   ├── convert_toml_to_csv.py         # TOML→CSV変換
│   ├── blender/                        # Blenderスクリプト
│   │   └── mk_points_3d.py           # 3D基準点CSV生成
│   └── data/                          # データファイル（gitignore）
│       ├── config_*.yaml              # カメラ別設定ファイル
│       ├── kijunten_locations*.csv    # 3D基準点データ
│       ├── points_2d*.csv             # 2D画像座標データ
│       ├── Calib_scene*.toml          # キャリブレーション結果
│       └── *.blend / *.ply           # 3Dモデルファイル
└── tests/                       # テストコード
    └── results/                 # テスト結果保存先
```

## スクリプト実行

すべてのスクリプトは `phase0/` ディレクトリで実行する。

```bash
cd phase0

# 通常（主点も推定）
uv run python estimate_camera_params.py data/config.yaml

# 主点固定（cx, cyを画像中心に固定）
uv run python estimate_camera_params.py data/config.yaml --fix-center

# 主点固定 + k3
uv run python estimate_camera_params.py data/config.yaml --fix-center --k3

# 広角レンズ（8係数歪みモデル）
uv run python estimate_camera_params.py data/config.yaml --wide

# 広角 + 主点固定
uv run python estimate_camera_params.py data/config.yaml --wide --fix-center

# 推定結果の検証（Ground Truth比較、レベル1/2検証）
uv run python phase0_verification.py data/config.yaml

# 外部パラメータ推定（K既知）
uv run python estimate_extrinsic.py data/config_lab2.yaml data/ufukui/05520125_intrinsics.toml

# TOML→CSV変換
uv run python convert_toml_to_csv.py
```

## アーキテクチャ

### データフロー

1. YAML設定ファイル → 対象カメラ名、入力CSV、画像サイズを指定
2. 3D基準点CSV (`kijunten_locations.csv`) と 2D画像座標CSV (`points_2d.csv`) を読み込み
3. ObjectNameで3D-2D点をマッチング
4. `cv2.solvePnP` で初期値算出 → `scipy.optimize.least_squares` (method='lm') で全パラメータ最適化
5. 再投影誤差(RMSE)で評価、結果をTOML形式とCSV形式で出力

### 入力データフォーマット

**kijunten_locations.csv**（3D基準点座標、全カメラ共通）
```csv
ObjectName,X,Y,Z
基準_01,-0.0199,-0.2968,-0.1913
基準_02,...
```

**points_2d.csv**（2D画像座標、全カメラ分を縦持ち、行の順番自由）
```csv
ObjectName,camera_name,X,Y
基準_01,cam01,780,913
基準_02,cam01,1877,483
基準_01,cam02,523,845
...
```

**config.yaml**（簡易YAMLパーサーで読み込み、`key: value` のフラット構造のみ対応）
```yaml
target_camera: cam01
camera_params: camera_params.csv
points_3d: kijunten_locations.csv
points_2d: points_2d.csv
image_width: 960
image_height: 540
```

- `camera_params` は検証スクリプト（`phase0_verification.py`）のみ使用
- `image_width`, `image_height` のデフォルトは 960x540

### 出力形式

- **Calib_scene.toml**: `matrix`(3x3), `distortions`, `rotation`(Rodrigues), `translation`
- **camera_params.csv**: camera_name, width, height, fx, fy, cx, cy, k1, k2, p1, p2, [k3,] r1, r2, r3, t1, t2, t3

### 推定モード（estimate_camera_params.py）

| オプション | 歪み係数 | 主点 |
|---|---|---|
| （なし） | k1, k2, p1, p2 | 推定 |
| `--fix-center` | k1, k2, p1, p2 | 画像中心に固定 |
| `--fix-center --k3` | k1, k2, p1, p2, k3 | 画像中心に固定 |
| `--wide` | k1, k2, p1, p2, k3, k4, k5, k6 | 推定 |
| `--wide --fix-center` | k1, k2, p1, p2, k3, k4, k5, k6 | 画像中心に固定 |

## コーディング規約

- **命名規則**:
  - クラス名: PascalCase
  - 関数・メソッド: snake_case
  - プライベートメソッド: `_` プレフィックス
  - 定数: UPPER_SNAKE_CASE
- **型ヒント**: 関数シグネチャに型ヒントを使用
- **コメント・出力メッセージ**: 日本語

## 開発方針

- **シンプルな機能を一つずつ作り、積み重ねて目的を達成する**
- 大きな機能を一度に作らない。小さく作って動作確認し、次の機能へ進む

### 機能ごとの開発フロー

各機能について、以下のフローを**厳守**する。**planモードは使わない**（通常モードで調査・計画を行う）。

1. **案件作成** → `docs/issues/{type}-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する
2. **調査・計画** → 通常モードで既存コードを調査し、要求仕様書と機能設計書を作成する
3. **ドキュメント保存** → 要求仕様書を `docs/issues/{案件フォルダ}/requirements.md`、機能設計書を `docs/issues/{案件フォルダ}/design.md` にファイル保存する。**保存が完了するまで実装に進んではならない**
4. **レビュー（Subagent + 人）** → 保存されたドキュメントをSubagent（Agentツール）でレビューする。ユーザーも同時にレビューする
5. **修正（必要な場合）** → レビューで問題があれば、再調査してドキュメントを更新する。**ステップ2〜4を問題がなくなるまで繰り返す**
6. **実装** → ドキュメント（要求仕様書・機能設計書・CLAUDE.md）を読んで実装する。実装完了後、「テスト」セクションのルールに従ってテストを作成・実行する
7. **手動テスト（実機確認）** → ユーザーが実機でテストする。以下の問題があれば修正計画を `docs/issues/{案件フォルダ}/investigation.md` に追記する（上書きしない。イテレーション番号を付けて履歴を残す）。**ユーザーの承認を得た上で、ステップ2〜7を繰り返す**（コード修正はステップ6で行う。ステップ7で直接コードを編集してはならない）
   - 不具合の発見
   - 要求通りに実装されていない
   - 要求仕様作成時のヒアリング漏れ
8. **完了** → README.mdのStatusをClosedに変更、`docs/BACKLOG.md` のステータスを更新、`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する

### 不具合修正フロー（bug-XXX 案件）

既存機能の不具合を修正する場合、以下のフローを**厳守**する。
機能追加（feat-XXX）は「機能ごとの開発フロー」に従うこと。

1. **案件作成** → `docs/issues/bug-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する。`README.md` に不具合の概要と再現手順を記録する
2. **調査・修正計画** → 既存コードを調査する。修正計画を `docs/issues/{案件フォルダ}/investigation.md` に記録する。**この時点でコードを編集してはならない**
3. **ドキュメント保存** → investigation.md の保存を確認する。要求仕様書・機能設計書の変更が必要な場合は変更案も作成し保存する。**保存が完了するまで実装に進んではならない**
4. **レビュー（Subagent + 人）** → 保存されたドキュメントをSubagent（Agentツール）でレビューする。ユーザーも同時にレビューする
5. **修正（必要な場合）** → レビューで問題があれば、再調査してドキュメントを更新する。**ステップ2〜4を問題がなくなるまで繰り返す**
6. **実装** → 承認された修正計画に沿ってコードを修正する。計画にない変更が必要になった場合は中断して報告する。実装完了後、「テスト」セクションのルールに従ってテストを作成・実行する
7. **手動テスト（実機確認）** → ユーザーが実機でテストする。問題があれば investigation.md にイテレーション番号を付けて追記し、**ユーザーの承認を得た上で、ステップ2〜7を繰り返す**（コード修正はステップ6で行う。ステップ7で直接コードを編集してはならない）
8. **完了** → README.mdのStatusをClosedに変更、`docs/BACKLOG.md` のステータスを更新、`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する

### ドキュメント作成ルール

- **実装前に必ず「要求仕様書」と「機能設計書」を作成し、案件フォルダにファイル保存すること**
- ドキュメントが保存されていない場合は、**実装を中止**する
- 要求仕様書：何を達成すべきか（入出力、制約、品質基準）
- 機能設計書：どう実現するか（モジュール構成、アルゴリズム、データ構造）
- ドキュメントは `docs/issues/{案件フォルダ}/` に置く（`requirements.md`, `design.md`）
- **/clear 後でも実装がスムーズにできるよう、必要な情報を全て記述する**
- 暗黙知に頼らず、**自己完結したドキュメント**にする（前の会話コンテキストがなくても実装できること）
- 不具合修正時は修正計画を `investigation.md` に記録すること

### テスト

- テストは `tests/` ディレクトリに置く
- **テスト実行はSubagent（Agentツール）を使う**
- テスト実行コマンド: `uv run pytest -v`
- **テスト結果は `tests/results/` にファイル保存する**
  - ファイル名：`{type}-{number}_test_result.txt`（例：`feat-001_test_result.txt`）
  - 内容：pytest の `-v` 出力をそのまま保存する

## ドキュメント管理ルール

### 案件管理

- **BACKLOG.md**: 全案件（バグ・機能追加）の一覧とステータス
- **CHANGELOG.md**: リリース履歴
- **issues/**: 個別案件のフォルダ

### 案件ディレクトリ構成

```
docs/issues/
└── {type}-{number}-{slug}/    # 例: bug-001-xxx, feat-002-yyy
    ├── README.md              # 概要、ステータス、再現手順
    ├── investigation.md       # 不具合の調査・修正計画
    ├── requirements.md        # 要求仕様書（機能追加時）
    └── design.md              # 機能設計書（機能追加時）
```

### 命名規則

- フォルダ名は英語で統一（例: `bug-001-reprojection-error`, `feat-002-multi-camera-support`）
- 案件フォルダは完了後も削除・移動しない

## 注意事項

- 各スクリプトに `load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` が重複実装されている（共通モジュール化されていない）
- CSVファイルとBlenderファイル(.blend)はgitignoreされている

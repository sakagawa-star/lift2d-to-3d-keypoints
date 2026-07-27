# CLAUDE.md

このファイルはClaude Codeがプロジェクトを理解するためのガイドです。

## セッション引き継ぎ

- セッション開始時にプロジェクトルートの `.claude/handovers/` ディレクトリを確認し、ファイルが存在すれば最新のものを読み込む
- セッション終了時や作業の区切りでは `/handover` の実行を促す

## プロジェクト概要

2D-3D点対応からカメラの内部パラメータ（焦点距離、主点）・外部パラメータ（回転、並進）・歪み係数を推定するシステム。OpenCVのsolvePnPで初期値を求め、SciPyのLevenberg-Marquardt法で最適化する。

### 目標
- 2D-3D点対応によるカメラ内部・外部パラメータ推定
- 歪み係数（k1, k2, p1, p2, optional k3、広角時は8係数）の推定
- Ground Truthとの比較検証（レベル1: K既知、レベル2: K未知）
- Calib_scene.toml / camera_params.csv 形式での結果出力
- 推定結果を用いた gsplat バッチレンダリング（phase4）

### 背景
- 初期値は `cv2.solvePnP` で求め、`scipy.optimize.least_squares`（method='lm'）で全パラメータを最適化する二段構えを採る
- 内部パラメータが既知（TOML提供）の場合は R, t のみを推定する K既知モードを持つ
- phase4 は phase0 とは独立した目的（3DGS のバッチレンダリング）であり、KIRI Engine アドオンで Blender に 3DGS を展開する前提

## 技術スタック

- **言語**: Python 3.10
- **パッケージ管理**: uv（phase0 はルートの `pyproject.toml`、phase4 は `phase4/pyproject.toml` で独立管理）
- **主要フレームワーク/ライブラリ**:
  - NumPy（数値計算）
  - SciPy（`least_squares`, Levenberg-Marquardt法）
  - OpenCV（`solvePnP`, `projectPoints`）
  - Blender（.blend / .ply ファイル、3Dモデル・カメラポーズ書き出し）
  - gsplat（CUDA、phase4 のバッチレンダリング）
- **詳細**: `docs/TECH_STACK.md` を参照
- **注意**: phase4 の gsplat レンダリングは環境変数 `TORCH_CUDA_ARCH_LIST="9.0+PTX"` が必須（gtune2 環境: RTX 5060 Ti + CUDA 12.6）。理由と恒久対策は「スクリプト実行 > phase4」を参照

## 環境セットアップ

phase0 と phase4 は独立した uv 環境を持つ。

```bash
# phase0（プロジェクトルート）
uv sync

# phase4（独立環境）
uv sync --project phase4
```

## スクリプト実行

### phase0（カメラパラメータ推定）

スクリプトは `phase0/` ディレクトリで実行する。

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

# 接線歪みゼロ固定（p1=p2=0、放射歪みのみ推定。--fix-center/--k3 と併用可、--wide とは併用不可）
uv run python estimate_camera_params.py data/config.yaml --fix-center --zero-tangent

# 推定結果の検証（Ground Truth比較、レベル1/2検証）
uv run python phase0_verification.py data/config.yaml

# 外部パラメータ推定（K既知モード）
uv run python estimate_camera_params.py data/config_lab2.yaml --intrinsic-toml data/ufukui/05520125_intrinsics.toml

# TOML→CSV変換（入力TOML・出力CSVの2引数が必須）
uv run python convert_toml_to_csv.py data/Calib_scene.toml data/camera_params.csv
```

### phase4（gsplatバッチレンダリング）

スクリプトは `phase4/` ディレクトリで実行する（phase0 とは独立した uv 環境）。

ワークフロー: KIRI Engine アドオンで Blender に 3DGS を展開してカメラパスを作成 → `camera_pose.py` でカメラポーズを JSON に書き出し → `render.py` で gsplat レンダリング。

```bash
cd phase4

# 1. カメラポーズ書き出し（Blender内スクリプト。GUIのScriptingタブまたはヘッドレスで実行）
#    --camera でカメラオブジェクト名を指定する（必須）
#    出力先: --output で指定。省略時は data/<カメラ名>_poses.json
blender -b data/FPS-camera.blend --python camera_pose.py -- --camera FPSCamera

# 1b. FPS頭部追従カメラのポーズ書き出し（feat-019。アーマチュア＋アンカー＋子カメラ構成の .blend 用）
#     向きを与える frame_change_post ハンドラは -b で発火しないため、姿勢計算をスクリプトに内蔵。
#     ヘッドレスでも頭部追従した c2w を出力する。実行は Blender 4.5.5（下記フルパス）。
#     --camera（必須）、--armature（既定 E00000）、--anchor（既定 Cam_Anchor）、--output
/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender -b data/Blender/session001_world_22pt.blend \
    --python fps_camera_pose.py -- --camera Cam_FPS --output data/Cam_FPS_poses.json

# 1c. C3Dキーポイントの時間方向平滑化（feat-020。Blender・GPU不要）
#     リフトアップ推定由来のジッターを Butterworth 2次 filtfilt（ゼロ位相）で除去し、
#     <入力>_filtered.c3d に書き出す。平滑化済みC3Dを Blender に再インポートして
#     fps_camera_pose.py / render_keypoints.py の入力に使う。
#     --cutoff でカットオフ周波数（既定 6.0 Hz。下げるほど滑らか）、--max-gap で
#     線形補間する欠損ギャップ長の上限（既定10フレーム。超過はセグメント分割）。
#     入力は本プロジェクト規約のC3D（npz_to_c3d.py 出力: mm / +Z / +Y）限定。
uv run python filter_c3d.py data/session001_world_22pt.c3d

# 2. バッチレンダリング（dry-run: 画像保存なしで動作確認・速度計測）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render.py data/project.ply data/FPSCamera_poses.json --dry-run

# 3. バッチレンダリング（連番PNG + MP4出力）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render.py data/project.ply data/FPSCamera_poses.json --mp4

# 4. ピンホール3DGSレンダリング＋人体キーポイント重ね描き（オクルージョン考慮、連番PNG/MP4。feat-015/016/017/021）
#    PLY + キャリブTOML + C3D（Halpe26 + Spine/Thorax の既知28マーカー、欠損許容、全フレーム）
#    + --camera を渡す。C3Dに無い既知マーカーは点・ボーンを描画スキップ（22点C3D対応）。
#    --near-plane でカメラ至近のfloaterを除去（0.01:黒い靄 → 0.5:鮮明）。
#    深度比較で手前の3DGSに隠れる点・ボーンを隠蔽。--no-occlusion で隠蔽OFF（比較用）、
#    --occlusion-margin で深度マージン調整（既定0.05m）。
#    カメラ固定のため背景レンダリングはループ前に1回だけ計算し全フレームで共有する。
#    出力は --output-dir に連番PNG（frame_<C3Dフレーム番号:06d>.png）。--mp4 でMP4も出力
#    （fps既定はC3D rate、--mp4-fps で上書き。小数fps保持）。範囲は --start-frame/--end-frame
#    （両端含む）。1フレームだけ見たい場合は --start-frame N --end-frame N で絞る。
#    数万フレーム規模でMP4だけ欲しい場合は --no-png でPNG保存をスキップすると大幅に速い
#    （--mp4 と併用必須。feat-022）。
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    data/Blender/keypoints.c3d \
    --camera cam41520554 --near-plane 0.5 --output-dir /tmp/keypoints --mp4

# 5. 静止画モード（feat-024。キャリブ推定結果のGT比較用）
#    --no-keypoints で C3D 不要（c3d_path は省略必須）。3DGS背景のみの
#    still_<カメラ名>.png を1枚出力（再実行時は上書き）。--distort で TOML の歪み係数
#    （長さ4/5/8対応）による歪みモデルレンダリング（gsplat 3DGUT 経路。--distort は
#    --no-keypoints 専用）。--mp4/--start-frame 等の動画系オプションとは併用不可。
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    --camera cam41520554 --near-plane 0.5 --no-keypoints --distort \
    --output-dir /tmp/calib_check
```

**`TORCH_CUDA_ARCH_LIST="9.0+PTX"` は必須**（2026-06-11 時点、マシン gtune2 の環境）:

- gsplat 1.5.3 には torch 2.10+cu128 向けのビルド済み wheel がなく、初回実行時に CUDA 拡張が JIT コンパイルされる
- 環境変数なしだと GPU（RTX 5060 Ti = sm_120）が検出され、システムの nvcc（CUDA 12.6）が sm_120 非対応のため `nvcc fatal: Unsupported gpu architecture 'compute_120'` でビルドが失敗する
- この環境変数で compute_90 の PTX を生成し、ドライバの JIT 変換で sm_120 上で実行する（ビルドキャッシュは `~/.cache/torch_extensions/py310_cu128/gsplat_cuda/`）
- 恒久対策は CUDA Toolkit 12.8 以上のインストール（その場合この環境変数は不要になる）

## テストデータ

データファイルは各 phase の `data/` ディレクトリに置く（**git管理外**）。

- `phase0/data/`
  - `config_*.yaml`: カメラ別設定ファイル
  - `kijunten_locations*.csv`: 3D基準点データ（全カメラ共通）
  - `points_2d*.csv`: 2D画像座標データ（全カメラ分を縦持ち）
  - `Calib_scene*.toml`: キャリブレーション結果
  - `*.blend` / `*.ply`: 3Dモデルファイル
- `phase4/data/`: PLY・カメラポーズJSON・Blenderファイル等

## ディレクトリ構成（主要部分）

```
lift2d-to-3d-keypoints/
├── CLAUDE.md                          # 本ファイル
├── README.md
├── pyproject.toml                     # phase0 の uv パッケージ管理
├── uv.lock
├── docs/                              # ドキュメント（案件管理 + 開発プロセス基準）
│   ├── BACKLOG.md                     # 案件一覧
│   ├── CHANGELOG.md                   # リリース履歴
│   ├── BUGFIX_STANDARD.md             # 不具合修正の記述基準
│   ├── DESIGN_STANDARD.md             # 機能設計書の記述基準
│   ├── REQUIREMENTS_STANDARD.md       # 要求仕様書の記述基準
│   ├── REVIEW_CRITERIA.md             # レビュー基準
│   ├── TECH_STACK.md                  # 技術スタック詳細
│   ├── codex-exec-ubuntu24-bwrap-fix.md # codex exec の bwrap エラー対策（Ubuntu 24系）
│   └── issues/                        # 個別案件フォルダ
├── phase0/                            # カメラパラメータ推定スクリプト群
│   ├── estimate_camera_params.py      # カメラパラメータ推定（メイン、K既知モード対応）
│   ├── common.py                      # 共通関数モジュール
│   ├── phase0_verification.py         # 検証スクリプト
│   ├── verify_triangulation.py        # 三角測量による外部パラメータ検証
│   ├── visualize_points_2d.py         # 2D座標を静止画上にプロットして可視化
│   ├── convert_toml_to_csv.py         # TOML→CSV変換
│   ├── blender/                       # Blenderスクリプト
│   │   └── mk_points_3d.py            # 3D基準点CSV生成
│   └── data/                          # データファイル（gitignore）
├── phase4/                            # gsplatバッチレンダリング（独立した uv 環境）
│   ├── pyproject.toml                 # uv パッケージ管理
│   ├── camera_pose.py                 # Blenderからカメラポーズを書き出すスクリプト
│   ├── fps_camera_pose.py             # FPS頭部追従カメラのポーズ書き出し（ヘッドレスで向きを計算・内蔵。feat-019）
│   ├── render.py                      # バッチレンダリングスクリプト
│   ├── render_keypoints.py            # 3DGSレンダリング＋人体キーポイント重ね描き（オクルージョン考慮、全フレーム連番PNG/MP4出力、欠損マーカー許容、--no-png でMP4のみ出力、--no-keypoints/--distort で歪み対応静止画モード。feat-015/016/017/021/022/024）
│   ├── npz_to_c3d.py                  # NPZ（リフトアップ済み3Dキーポイント）→ C3D 変換（Blender io_anim_c3d 取り込み対応。feat-018）
│   ├── filter_c3d.py                  # C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相。feat-020）
│   └── data/                          # データファイル（gitignore）
└── tests/                             # テストコード
    └── results/                       # テスト結果保存先
```

## アーキテクチャ

### データフロー

1. YAML設定ファイル → 対象カメラ名、入力CSV、画像サイズを指定
2. 3D基準点CSV (`kijunten_locations.csv`) と 2D画像座標CSV (`points_2d.csv`) を読み込み
3. ObjectNameで3D-2D点をマッチング
4. `cv2.solvePnP` で初期値算出 → `scipy.optimize.least_squares`（method='lm'）で全パラメータ最適化
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
| `--zero-tangent` | k1, k2（p1, p2 は0固定。`--k3` 併用で k3 も推定、`--fix-center` 併用可、`--wide` とは併用不可） | オプションに従う |
| `--intrinsic-toml` | TOML読み込み（K既知、R,tのみ推定） | TOML読み込み |

## ドメイン知識

- 各スクリプトに `load_yaml_simple`, `load_points_3d`, `load_points_2d`, `match_points` が重複実装されている（一部は `phase0/common.py` に切り出し済みだが、完全には共通モジュール化されていない）
- CSVファイルとBlenderファイル(.blend)はgitignoreされている
- 3D-2D点のマッチングは ObjectName をキーに行う（行の順番には依存しない）

## 開発方針

- **シンプルな機能を一つずつ作り、積み重ねて目的を達成する**
- 大きな機能を一度に作らない。小さく作って動作確認し、次の機能へ進む

### 機能追加フロー（feat-XXX 案件）

新機能を追加する場合、以下のフローを**厳守**する。**planモードは使わない**（通常モードで調査・計画を行う）。

1. **案件作成** → `docs/issues/feat-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する
2. **調査・計画** → 通常モードで既存コードを調査し、要求仕様書（`docs/REQUIREMENTS_STANDARD.md` 準拠）と機能設計書（`docs/DESIGN_STANDARD.md` 準拠）を作成する
3. **ドキュメント保存** → 要求仕様書を `docs/issues/{案件フォルダ}/requirements.md`、機能設計書を `docs/issues/{案件フォルダ}/design.md` にファイル保存する。**保存が完了するまで実装に進んではならない**
4. **レビュー（Codex → 人）** → 保存されたドキュメントを **Codex** でレビューする。実行方法は後述の「Codexによるレビューの実行方法」を参照。**まず Codex の再帰レビュー（修正→再レビュー）を重要度「高・中」がゼロに収束するまで回し、その後に人（ユーザー）がレビューする**（収束前に人レビューはしない）。レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
5. **修正（必要な場合）** → レビューで問題があれば、再調査してドキュメントを更新する。**ステップ2〜4を問題がなくなるまで繰り返す**
6. **実装** → ドキュメント（要求仕様書・機能設計書・CLAUDE.md）を読んで実装する。実装は後述の「実装の実行方法（Sonnetサブエージェント）」に従い、Sonnet サブエージェントに委任する。実装完了後、「テスト」のルールに従ってテストを実行する
7. **手動テスト** → ユーザーがテストする。以下の問題があれば `docs/BUGFIX_STANDARD.md` に従って修正計画を `docs/issues/{案件フォルダ}/investigation.md` に追記する（上書きしない。イテレーション番号を付けて履歴を残す）。**ユーザーの承認を得た上で、ステップ2〜7を繰り返す**（コード修正はステップ6で行う。ステップ7で直接コードを編集してはならない）
   - 不具合の発見
   - 要求通りに実装されていない
   - 要求仕様作成時のヒアリング漏れ
8. **完了** → `docs/BACKLOG.md` のステータスを Closed に更新する。`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する

### 不具合修正フロー（bug-XXX 案件）

既存機能の不具合を修正する場合、以下のフローを**厳守**する。

1. **案件作成** → `docs/issues/bug-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する。`README.md` に不具合の概要と再現手順を記録する
2. **調査・修正計画** → `docs/BUGFIX_STANDARD.md` に従い、既存コードを調査する。修正計画を `docs/issues/{案件フォルダ}/investigation.md` に記録する。**この時点でコードを編集してはならない**
3. **ドキュメント保存** → investigation.md の保存を確認する。**保存が完了するまで実装に進んではならない**
4. **レビュー（Codex → 人）** → 保存されたドキュメントを **Codex** でレビューする。実行方法は後述の「Codexによるレビューの実行方法」を参照。**まず Codex の再帰レビュー（修正→再レビュー）を重要度「高・中」がゼロに収束するまで回し、その後に人（ユーザー）がレビューする**（収束前に人レビューはしない）。レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
5. **修正（必要な場合）** → レビューで問題があれば、再調査してドキュメントを更新する。**ステップ2〜4を問題がなくなるまで繰り返す**
6. **実装** → 承認された修正計画に沿ってコードを修正する。実装は後述の「実装の実行方法（Sonnetサブエージェント）」に従い、Sonnet サブエージェントに委任する。計画にない変更が必要になった場合は中断して報告する
7. **手動テスト** → ユーザーがテストする。問題があれば `docs/BUGFIX_STANDARD.md` に従って investigation.md にイテレーション番号を付けて追記し、**ユーザーの承認を得た上で、ステップ2〜7を繰り返す**（コード修正はステップ6で行う。ステップ7で直接コードを編集してはならない）
8. **完了** → `docs/BACKLOG.md` のステータスを Closed に更新する。`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する

### ドキュメント作成ルール

- **実装前に必ずドキュメントを作成し、案件フォルダにファイル保存すること**
- ドキュメントが保存されていない場合は、**実装を中止**する
- 機能追加時: 要求仕様書（`docs/REQUIREMENTS_STANDARD.md` 準拠）と機能設計書（`docs/DESIGN_STANDARD.md` 準拠）を作成する
- 不具合修正時: `docs/BUGFIX_STANDARD.md` の基準に従い、修正計画を `investigation.md` に記録する
- レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
- ドキュメントは `docs/issues/{案件フォルダ}/` に置く（`requirements.md`, `design.md`, `investigation.md`）
- **/clear 後でも実装がスムーズにできるよう、必要な情報を全て記述する**
- 暗黙知に頼らず、**自己完結したドキュメント**にする（前の会話コンテキストがなくても実装できること）
- ライブラリの追加・変更・削除を行った場合は `docs/TECH_STACK.md` も更新すること
- 新規ライブラリ導入時は用途・選定理由・バージョンを `TECH_STACK.md` に追記すること

### 案件ディレクトリ構成

```
docs/issues/
└── {type}-{number}-{slug}/    # 例: bug-001-xxx, feat-001-yyy
    ├── README.md              # 概要、ステータス、再現手順
    ├── requirements.md        # 要求仕様書（機能追加時、REQUIREMENTS_STANDARD.md 準拠）
    ├── design.md              # 機能設計書（機能追加時、DESIGN_STANDARD.md 準拠）
    ├── investigation.md       # 不具合の調査・修正計画（BUGFIX_STANDARD.md 準拠）
    └── reviews/               # Codexレビューの結果（codex-NN.result.md のみ git 管理。full.log は gitignore）
```

### 命名規則

- フォルダ名は英語で統一（例: `bug-001-reprojection-error`, `feat-002-multi-camera-support`）
- 案件フォルダは完了後も削除・移動しない

### Codexによるレビューの実行方法

機能追加・不具合修正フローのステップ4（レビュー）では、Claude Code 自身が `codex exec` コマンドを実行して Codex にレビューさせる。Subagent は使わない。**Codex は逐次（前回セッションを `resume` で継続）で回し、重要度「高・中」がゼロに収束してから人レビューに進む**。並列にはしない（再レビューの収束確認＝「前回指摘が直ったか」の判定に前回文脈の引き継ぎが必要なため）。

使用するモデルは `~/.codex/config.toml` のデフォルト設定に従う。本ファイルのコマンドにはモデル指定（`-m`）を書かない。モデルを切り替えたい場合は `~/.codex/config.toml` を編集する（全プロジェクト共通で反映される）。

**実行はバックグラウンドで行う**: Codex のレビューは reasoning effort の設定によっては1回10分を超えることがあり、Claude Code の Bash ツールのタイムアウト上限（最大10分）に抵触する。`codex exec` はバックグラウンド実行（`run_in_background`）とし、完了通知後に `codex-NN.result.md` を読んで指摘を確認する。`-o` による結果のファイル保存はこの運用を前提としている。

> **Ubuntu 24系で `bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted` が出る場合**は、`docs/codex-exec-ubuntu24-bwrap-fix.md` を参照して AppArmor プロファイルを追加すること（ホスト側の user namespace 制限が原因。Codex のバグではない）。

#### 出力の保存（結果と過程を分離）

- レビュー結果と過程ログは **案件フォルダの `docs/issues/{案件フォルダ}/reviews/`** に保存する（事前に `mkdir -p` する）。
- **初回から `-o`（`--output-last-message`）を必ず付ける**。`-o` で最終レビュー結果だけを `codex-NN.result.md` に書き、stdout 全体（過程ログ）は `> codex-NN.full.log 2>&1` で別ファイルに保存する（混在させない）。
- ファイル名はレビュー回ごとに連番（`codex-01`, `codex-02`, …）。
- `result.md` のみ git 管理し、`full.log` は `.gitignore`（`docs/issues/*/reviews/*.full.log`）でローカルのみとする（リポジトリ肥大回避）。
- `result.md` には Codex の生出力に加え、Claude Code の対応方針を追記してよい（冒頭に日付・対象・session id・初回/再の定型メタを置くと追いやすい）。

#### 初回レビュー（機能追加の場合）

```bash
mkdir -p docs/issues/{案件フォルダ}/reviews
codex exec -o docs/issues/{案件フォルダ}/reviews/codex-01.result.md \
  "docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/{案件フォルダ}/requirements.md docs/issues/{案件フォルダ}/design.md 。瑣末な点へのクソリプはしないで、致命的な点のみ指摘して。発見した問題を重要度(高/中/低)で分類し、修正提案とともに報告すること。" \
  > docs/issues/{案件フォルダ}/reviews/codex-01.full.log 2>&1
```

#### 初回レビュー（不具合修正の場合）

```bash
mkdir -p docs/issues/{案件フォルダ}/reviews
codex exec -o docs/issues/{案件フォルダ}/reviews/codex-01.result.md \
  "docs/REVIEW_CRITERIA.md および docs/BUGFIX_STANDARD.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/{案件フォルダ}/investigation.md 。瑣末な点へのクソリプはしないで、致命的な点のみ指摘して。発見した問題を重要度(高/中/低)で分類し、修正提案とともに報告すること。" \
  > docs/issues/{案件フォルダ}/reviews/codex-01.full.log 2>&1
```

#### 再レビュー（共通）

ドキュメントを更新して再レビューする場合、最初のレビューの文脈を保持するため**同一セッションを `resume` で継続**する。セッション ID は `codex-01.full.log` 冒頭の `session id:` 行に記録されるので、それを明示指定する（`--last` は別の codex 実行が挟まると意図しないセッションを掴む恐れがあるため使わない）。連番を1つ進める:

```bash
codex exec resume {SESSION_ID} -o docs/issues/{案件フォルダ}/reviews/codex-02.result.md \
  "ドキュメントを更新したので再レビューして。前回と同じ基準で。前回指摘が解消されたかを含めて確認して。瑣末な点へのクソリプはしないで、致命的な点のみ指摘して。重要度(高/中/低)で分類し、修正提案とともに報告すること。" \
  > docs/issues/{案件フォルダ}/reviews/codex-02.full.log 2>&1
```

**注意**: `resume`（セッション継続）を使わないと最初のレビューの文脈が失われる。`-o` と `> ...full.log 2>&1` は毎回付け、連番（`codex-03`, `codex-04`, …）を進める。

#### レビュー終了条件

重要度「高」「中」の指摘がゼロに収束するまで、修正 → 再レビュー（連番を進める）を繰り返す。**収束したら人（ユーザー）レビューに進む**（収束前に人レビューはしない）。

### 実装の実行方法（Sonnetサブエージェント）

機能追加・不具合修正フローのステップ6（実装）は、Claude Code 自身が直接コードを書くのではなく、Agent ツールで **model: sonnet** を指定したサブエージェントに委任する（feat-023 / bug-002 で確立した運用。2026-07-22）。

#### サブエージェントへの指示に必ず含めること

1. **必読ドキュメントと読む順序**: CLAUDE.md → 案件ドキュメント（機能追加は `requirements.md` と `design.md`、不具合修正は `investigation.md`）→ 変更対象コード → 参考にする既存テスト
2. **厳密準拠の指示**: 設計書・修正計画に厳密に従うこと。書かれていない独自判断・改善・リファクタは一切禁止
3. **想定外事象の扱い**: 想定外の事象（設計書どおりに実装できない、ドキュメントと実コードの矛盾、テストが通らない等）が発生したら、**その場で回避策を実装せず直ちに中断**し、何が起きたか・どこまで完了したかを報告して終了すること。報告を受けたら「調査・計画 → requirements.md / design.md（または investigation.md）の修正」のステップに**必ず戻る**（レビューを経てから実装を再開する）
4. **検証まで実施**: `uv run pytest -v` の全件実行、`tests/results/{type}-{number}_test_result.txt` への出力保存、ドキュメントに定義された動作確認（実データ実行等）
5. **禁止事項**: git commit / push はサブエージェントに行わせない。BACKLOG.md / CHANGELOG.md / CLAUDE.md の更新も行わせない（完了ステップ8で Claude Code 本体が行う）
6. **報告形式**: 変更ファイル一覧、テスト結果サマリ、動作確認結果、想定外事象の有無

#### 委任しない作業

- 調査・計画、ドキュメント作成、Codexレビューの実行と指摘反映、完了処理（ステップ8）、git 操作は Claude Code 本体が行う

### コードレビュー

- レビューでは重要度(高/中/低)で分類し、修正提案とともに報告する
- 重要度:高と中は修正対象とする
- レビュー基準の詳細は `docs/REVIEW_CRITERIA.md` を参照

### テスト

- テストは `tests/` ディレクトリに置く
- テスト実行コマンド: `uv run pytest -v`
- **テスト結果は `tests/results/` にファイル保存する**
  - ファイル名：`{type}-{number}_test_result.txt`（例：`feat-001_test_result.txt`）
  - 内容：pytest の `-v` 出力をそのまま保存する

## Claude Code 運用ルール

### Bash 実行時のルール

- **`cd <path> && <command>` の連結は禁止。** Bashツールはプロジェクト作業ディレクトリで動くため `cd` は不要。連結すると先頭トークンが `cd` になり、`.claude/settings.json` / `.claude/settings.local.json` のallowlist（例: `Bash(codex exec *)`、`Bash(git status)`）が一致せず、毎回パーミッションプロンプトが発生する
- 別ディレクトリで実行する必要がある場合は、コマンド側のオプションを使う（例: `git -C <path> status`、`uv run --project phase4 ...`）
- どうしても複数コマンド連結が必要な場合も、先頭トークンが安全・許可済みであるかを確認してから書く

### git 操作の実行方法（Opusサブエージェント）

git のコミット・プッシュは、Claude Code 本体が直接実行するのではなく、Agent ツールで **model: opus** を指定したサブエージェントに委任する（2026-07-27 に確立した運用）。

#### サブエージェントへの指示に必ず含めること

1. **コミット内容の背景**: 何をなぜ変更したか（コミットメッセージ作成に必要な情報）を要約して渡す
2. **ステージ対象の明示**: コミットに含めるファイルを列挙する。`.claude/settings.local.json` と `.claude/handovers/` 配下は含めない
3. **コミットメッセージ**: 日本語。末尾トレーラーは `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`
4. **Bashルールの継承**: `cd <path> && <command>` 連結禁止、`git -C <path> ...` 形式を使う
5. **失敗時の扱い**: コンフリクト・push拒否等が起きたら対処（rebase, reset, force push 等）せず、状況をそのまま報告して終了する

#### 委任しない作業

- コミット可否の判断・タイミング（ユーザーの指示を受けて Claude Code 本体が起動する）
- コミット後の結果検証（`git log -1 --stat` 等での確認は本体が行う）

## コーディング規約

- **命名規則**:
  - クラス名: PascalCase
  - 関数・メソッド: snake_case
  - プライベートメソッド: `_` プレフィックス
  - 定数: UPPER_SNAKE_CASE
- **型ヒント**: 関数シグネチャに型ヒントを使用
- **コメント・出力メッセージ**: 日本語

## 完了済み案件

詳細は `docs/BACKLOG.md`（一覧）および `docs/CHANGELOG.md`（リリース履歴）を参照。

- **feat-001**: 内部パラメータ既知での外部パラメータ推定（Stage 1）（2026-03-20完了、Calib_scene.toml から K を読み込み R, t のみ推定）
- **feat-002**: estimate_camera_params.py 広角レンズ対応（2026-03-20完了、`--wide` で8係数歪みモデル）
- **feat-003**: estimate_extrinsic.py を estimate_camera_params.py に統合（Stage 2）（2026-03-20完了、`--intrinsic-toml` で K既知モード）
- **feat-004**: gsplatバッチレンダリングパイプライン（2026-03-21完了、PLY + カメラポーズJSON → PNG）
- **feat-005**: render.py フレーム範囲指定オプション（2026-03-21完了、`--start-frame`/`--end-frame`）
- **feat-006**: render.py ドライランモード（2026-03-21完了、`--dry-run` で性能計測）
- **feat-007**: render.py MP4ファイル保存機能（2026-03-21完了、`--mp4`、NVENC/ libx264 フォールバック）
- **feat-008**: estimate_camera_params.py 複数カメラ一括推定（2026-03-23完了、K既知モードでカンマ区切り指定）
- **feat-009**: 三角測量による外部パラメータ検証（2026-03-23完了、`verify_triangulation.py`）
- **feat-010**: 2D座標を静止画上にプロットして可視化（2026-04-28完了、`visualize_points_2d.py`）
- **bug-001**: visualize_points_2d.py のパス解決を既存スクリプトと統一（2026-04-28完了）
- **feat-011**: visualize_points_2d.py 基準点番号ラベル表示オプション（2026-06-07完了、`--label`）
- **feat-012**: camera_pose.py カメラ名・出力先のCLIオプション化（2026-06-12完了、`--camera`/`--output`）
- **feat-013**: 3DGSレンダリング画像への3Dキーポイント重ね描き（中止。feat-015/016 に分割して作り直し）
- **feat-014**: ピンホール3DGSレンダリング（PNG出力、GT比較）（中止。feat-015 に作り直し）
- **feat-015**: ピンホール3DGSレンダリング（PNG出力、GT比較）（2026-06-14完了、`render_keypoints.py` を3DGSレンダリングのみに作り直し、`--camera`/`--near-plane`/`--output`/`--background`）
- **feat-016**: キーポイントのオクルージョン（深度による前後判定）（2026-06-14完了、`render_keypoints.py` に人体キーポイント（C3D, Halpe26 先頭フレーム）重ね描き＋深度比較によるオクルージョンを追加。`c3d_path`必須引数・`--no-occlusion`・`--occlusion-margin`、`render_image` に `return_depth` 追加）
- **feat-017**: render_keypoints.py 全フレーム対応（連番PNG + MP4）（2026-06-14完了、C3D全フレーム描画。`load_c3d_all_frames`/`start_ffmpeg` 追加、背景レンダリングをループ前1回計算で共有。`--output-dir`/`--start-frame`/`--end-frame`/`--mp4`/`--mp4-fps`、`--output` 廃止）
- **feat-018**: NPZ→C3D変換スクリプト（Blender io_anim_c3d 取り込み対応）（2026-06-25完了、`phase4/npz_to_c3d.py` 新規。world(X,Y,Z)m→C3D raw(Y,Z,X)×1000 mm で `c3d_to_calib` 互換。C3Dフレームは1始まり（py-c3d 16bit制約回避）。Blender正立は `UNITS='mm'`/`X_SCREEN='+Z'`/`Y_SCREEN='+Y'`（io_anim_c3d は pose bone ローカル座標+rest行列で描画するため表示鉛直は Y_SCREEN で決まる）。出力は一時ファイル→読み戻し検証→`os.replace` のアトミック確定）
- **feat-019**: FPS頭部追従カメラのポーズ書き出しスクリプト（ヘッドレス対応）（2026-07-01完了、`phase4/fps_camera_pose.py` 新規。`camera_pose.py` は温存。向きを与える `frame_change_post` ハンドラが `-b` で発火せず向きが凍結する問題を、Frankfurt平面ベースの姿勢計算を内蔵して解消。ボーンは評価済み depsgraph から取得、アンカー回転適用後 `view_layer.update()` で子カメラへ反映。起動時＋全フレームで構成・縮退・直交・位置/向き整合を検証し違反時 exit(1)。出力は camera_pose.py と同一スキーマ＋原子的書き出し。`--camera`/`--armature`/`--anchor`/`--output`。実行は Blender 4.5.5）
- **feat-020**: C3Dキーポイントの時間方向平滑化スクリプト（2026-07-02完了、`phase4/filter_c3d.py` 新規。C3D→C3Dの独立前処理で、リフトアップ推定ジッターを Butterworth 2次 filtfilt（実効4次・ゼロ位相）で除去。`--cutoff`（既定6.0Hz）/`--rate`（point rate欠損時の補完専用）/`--max-gap`（既定10。超過ギャップはセグメント分割で独立フィルタ、無効サンプルは無効のまま維持）/`--output`（既定 `<入力>_filtered.c3d`）。入力は本プロジェクト規約のC3D（mm / +Z / +Y、first_frame 1〜65534）限定で規約外はエラー。phase4 に scipy>=1.11 追加）
- **feat-021**: render_keypoints.py 欠損マーカー許容（22点C3D対応）（2026-07-02完了、既知マーカーを `KEYPOINT_NAMES`（Halpe26 26点 + Spine/Thorax の28点）に拡張し、C3Dに無い既知マーカーは valid=False で点・ボーンを描画スキップ。`extract_halpe26` → `extract_keypoints`、`HALPE26_SKELETON` 定数を `build_skeleton(present)` に置換（体幹は Spine/Thorax の有無で Neck–Thorax–Spine–Hip ⇄ Neck–Hip を切り替え、同位置挿入で描画順維持＝26点C3Dの描画は変更前と同一）。起動時にマーカー構成を報告、既知マーカー0個のみエラー。CLI無変更）
- **feat-022**: render_keypoints.py --no-png オプション（MP4のみ出力）（2026-07-03完了、`--no-png --mp4` で連番PNG保存（`cv2.imwrite`。数万フレーム処理の支配的ボトルネック）をスキップしMP4のみ出力。`--no-png` 単独指定は重い処理前に `parser.error()` で拒否（終了コード2）。既存 `frame_*.png` は削除・上書きしない非破壊方針。PNGスキップ時の進捗表示は `[i/n] frame <番号> -> mp4`。`--no-png` なしの挙動は変更前と完全同一）
- **feat-024**: render_keypoints.py 歪みモデル対応レンダリング（GT比較用）（2026-07-23完了、`--no-keypoints` で C3D 不要の静止画モード（`still_<カメラ名>.png` 1枚出力）、`--distort` で TOML 歪み係数（長さ4/5/8）による歪みレンダリング（gsplat 1.5.3 の 3DGUT 経路: `with_ut=True, with_eval3d=True, packed=False` + radial/tangential 係数。静止画モード専用）。スパイクで feat-013 の「UT経路=黒い靄」は near_plane=0.01 の floater との交絡だったと実証して方式決定。動画系7オプションとの併用は parser.error() で拒否。既存の動画モードは変更前と完全同一。E0085 実データで、ピンホールでは画角外だった基準_018/051 が歪みONで2D観測値と約2px以内に画面内描画されることを確認）
- **feat-023**: estimate_camera_params.py 接線歪みゼロ固定オプション（--zero-tangent）（2026-07-22完了、通常モードで p1, p2 を0固定し放射歪みのみ推定（`CALIB_ZERO_TANGENT_DIST` 相当）。画像端の基準点偏在で接線歪みが誤差吸収弁となり異常値（E0085-01 の p2=0.052）に収束する問題への対策。`project_dist2`/`project_dist3` の主点固定・推定版4関数と最小点数定数（DIST2: 13/10、DIST3: 14/11）を追加。`--fix-center`/`--k3` 併用可、`--wide` 併用はエラー、`--intrinsic-toml` 時は警告して無視。出力TOML/CSVは既存レイアウトのまま p1, p2 に 0.0。`--zero-tangent` なしの挙動は変更前と完全同一）

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
- **注意**: phase4 の gsplat レンダリングは環境変数 `TORCH_CUDA_ARCH_LIST="9.0+PTX"` が必須（gtune2 環境: RTX 5060 Ti + CUDA 12.6）。理由と恒久対策は「スクリプト実行」を参照

## 環境セットアップ

phase0 と phase4、matcher_lab は独立した uv 環境を持つ。

```bash
# phase0（プロジェクトルート）
uv sync

# phase4（独立環境）
uv sync --project phase4

# matcher_lab（独立環境。学習ベースマッチャー評価、feat-026 候補3）
uv sync --project matcher_lab
```

## スクリプト実行

各スクリプトの実行方法・コマンド例・オプション一覧は `README.md` を参照。phase0 のスクリプトは `phase0/`、phase4 のスクリプトは `phase4/` ディレクトリで実行する。

**`TORCH_CUDA_ARCH_LIST="9.0+PTX"` は必須**（2026-06-11 時点、マシン gtune2 の環境。phase4 の render.py / render_keypoints.py）:

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
│   ├── render_keypoints.py            # 3DGSレンダリング＋人体キーポイント重ね描き（オクルージョン考慮、全フレーム連番PNG/MP4出力、欠損マーカー許容、キーポイント入力は C3D/NPZ 両対応〔拡張子 .npz で判別、pnp_ok は無視して全フレーム描画〕、--no-png でMP4のみ出力、--no-keypoints/--distort で歪み対応静止画モード。feat-015/016/017/021/022/024/032）
│   ├── npz_to_c3d.py                  # NPZ（リフトアップ済み3Dキーポイント）→ C3D 変換（Blender io_anim_c3d 取り込み対応。feat-018）
│   ├── filter_c3d.py                  # C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相。feat-020）
│   ├── filter_npz.py                  # NPZキーポイントの時間方向平滑化（NPZ→NPZ、C3D・Blender不要、NaN区間維持・追加キー無加工コピー。feat-028）
│   ├── refine_extrinsics.py           # 手動点（一意6点以上）+ LoFTR 自動マッチングによる外部パラメータ精緻化バッチ（K既知、サンプリング型受理判定、Calib_scene.toml型出力+診断レポート。feat-026）
│   ├── render_fps_video.py            # NPZ直読みFPS動画一括生成（頭部キーポイントからポーズをnumpy計算、ピンホールK、NaN=黒画面、チャンク分割+耐久書き出し+ffprobe破損検査つき再開、静止画/ポーズダンプ排他モード、--config YAML対応、--gpus でチャンク並列レンダリング〔単一GPU複数ワーカー/複数GPU対応、動的分配、失敗時即時中止〕、完了時に <MP4名>_info.txt を自動保存。feat-027/029/030/031）
│   └── data/                          # データファイル（gitignore）
├── matcher_lab/                       # 学習ベースマッチャー環境（独立した uv 環境。feat-026）
│   ├── pyproject.toml                 # uv パッケージ管理（Python 3.12 / torch cu130 / kornia）
│   ├── loftr_cli.py                   # LoFTR 推論 CLI（refine_extrinsics.py から subprocess で呼ばれる正式コンポーネント。feat-026）
│   ├── loftr_smoke.py                 # LoFTR（kornia）の環境疎通スモークテスト
│   └── mast3r_smoke.py                # MASt3R（~/git/mast3r + ~/data/models/mast3r/）の環境疎通スモークテスト
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
8. **完了** → `docs/BACKLOG.md` のステータスを Closed に更新する。`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する。`README.md` に記載済みの内容（コマンド、CLIオプション、入力/出力形式、既定値、実行環境・依存条件）に変更があった場合は `README.md` を最新に更新する

### 不具合修正フロー（bug-XXX 案件）

既存機能の不具合を修正する場合、以下のフローを**厳守**する。

1. **案件作成** → `docs/issues/bug-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する。案件フォルダの `docs/issues/bug-{number}-{slug}/README.md` に不具合の概要と再現手順を記録する（ルートの `README.md` ではない）
2. **調査・修正計画** → `docs/BUGFIX_STANDARD.md` に従い、既存コードを調査する。修正計画を `docs/issues/{案件フォルダ}/investigation.md` に記録する。**この時点でコードを編集してはならない**
3. **ドキュメント保存** → investigation.md の保存を確認する。調査の結果 `requirements.md` / `design.md` の修正が必要になった場合は、それらも併せて保存する。**保存が完了するまで実装に進んではならない**
4. **レビュー（Codex → 人）** → 保存されたドキュメントを **Codex** でレビューする。実行方法は後述の「Codexによるレビューの実行方法」を参照。**まず Codex の再帰レビュー（修正→再レビュー）を重要度「高・中」がゼロに収束するまで回し、その後に人（ユーザー）がレビューする**（収束前に人レビューはしない）。レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
5. **修正（必要な場合）** → レビューで問題があれば、再調査してドキュメントを更新する。**ステップ2〜4を問題がなくなるまで繰り返す**
6. **実装** → 承認された修正計画に沿ってコードを修正する。実装は後述の「実装の実行方法（Sonnetサブエージェント）」に従い、Sonnet サブエージェントに委任する。計画にない変更が必要になった場合は中断して報告する
7. **手動テスト** → ユーザーがテストする。問題があれば `docs/BUGFIX_STANDARD.md` に従って investigation.md にイテレーション番号を付けて追記し、**ユーザーの承認を得た上で、ステップ2〜7を繰り返す**（コード修正はステップ6で行う。ステップ7で直接コードを編集してはならない）
8. **完了** → `docs/BACKLOG.md` のステータスを Closed に更新する。`docs/CHANGELOG.md` に完了内容を記録する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する。`README.md` に記載済みの内容（コマンド、CLIオプション、入力/出力形式、既定値、実行環境・依存条件）に変更があった場合は `README.md` を最新に更新する

### ドキュメント更新フロー（update-XXX 案件）

開発プロセスを定める運用ドキュメント（`CLAUDE.md`、`docs/` 直下の基準書・BACKLOG・CHANGELOG、`.gitignore` 等）の改訂は **update-XXX 案件**として扱い、以下のフローを**厳守**する。典型例:

- 本プロジェクトのコピー元テンプレートリポジトリ（開発ドキュメントテンプレート）の改訂の取り込み
- ドキュメント間の二重管理・不整合の解消、運用ルールの新設・変更

**ソースコード・テストコードの変更は含まない。** 作業中にコード変更が必要と判明した場合は中断し、feat/bug 案件として起票し直す。個別機能のドキュメント（案件フォルダ内の requirements.md 等）の修正は元案件側で扱い、update 案件にはしない。

要求仕様書・機能設計書は作らず、README.md（調査）と design.md（反映設計）の2点で代替する。

1. **案件作成** → `docs/issues/update-{number}-{slug}/` フォルダを作成し、`docs/BACKLOG.md` に追加する。slug は変更の目的がわかる名前にする（例: `adopt-dev-template`）
2. **調査** → 現状と変更理由を調査し、案件フォルダの `README.md` に記録する。テンプレート取り込みの場合は反映元パス・コミットID・差分の全量と、取り込む/取り込まない の選別と理由を書く。**この時点で反映先を編集してはならない**
3. **設計・保存** → 変更対象ファイルごとに「どのセクションを・どう変えるか」を `design.md` に書いてファイル保存する。自己完結（/clear 後でも design.md だけで反映作業ができる）・曖昧表現禁止。全置換に後処理が伴う場合は、変更方式の一覧・該当セクション・実施手順のすべてに明記する。完了処理（BACKLOG・CHANGELOG の更新）も設計に含める。**保存が完了するまで反映に進んではならない**
4. **レビュー（Codex → 人）** → 「Codexによるレビューの実行方法」に従う（バックグラウンド実行、`-o` + full.log 分離、`resume` による逐次再レビュー、重要度「高・中」ゼロ収束後に人レビュー）。レビュー対象は `README.md` と `design.md`。レビュー観点は次の3点を明示して依頼する:
   1. 反映計画の自己完結性（design.md だけで作業ができるか）
   2. 情報の喪失（削除・置換対象に、他所に存在しない情報が含まれていないか）
   3. 変更後のドキュメント間整合性（参照切れ、矛盾、案件の漏れ・重複）
5. **反映** → design.md に厳密に従って編集する。実装は Claude Code 本体が行ってよい（転記・削除中心で Sonnet 委任のオーバーヘッドに見合わないため。分量が大きい機械的変更では委任も可。どちらにするかは design.md に明記する）。設計にない変更が必要になったら中断してステップ2に戻る。反映後 `git diff` で「意図した変更のみか・保持対象が変わっていないか」を検証する
6. **完了** → `docs/BACKLOG.md` のステータスを Closed に更新する。`docs/CHANGELOG.md` に完了内容を記録し、案件 README のステータスを Closed に更新する。ファイルの追加・削除があった場合は `CLAUDE.md` のディレクトリ構成を最新に更新する
7. **テスト** → コード変更がないためテスト（自動・手動とも）は不要（不要であることを design.md に明記する）

#### 運用メモ

- 汎用性のある改善は、完了後にコピー元の開発ドキュメントテンプレートリポジトリへの還元（テンプレート側の update-XXX 案件）を検討する
- ルートの `.gitignore` がグローバル gitignore（`~/.gitignore_global`）の影響で未追跡になる環境では、コミットに含める際に `git add -f` が必要

### 実験・検証の進め方（予測→実行→照合）

数値判定を伴う実験・検証（案件の調査フェーズでの実験、閾値・パラメータの妥当性検証、性能測定等）では、以下のプロトコルを**厳守**する（feat-026 で試験導入し確立した運用。2026-07-31〜08-02）。

1. **判定基準の事前定義**: 実験の実行前に「答える問い」「判定閾値」「合格ライン（何%・何件なら合格か）」を数値で criteria 文書（案件フォルダ配下、例: `docs/issues/{案件フォルダ}/experiments/{実験名}/criteria.md`）に文書化する。数値で定義できない場合は、その実験を「基準を測ることがゴールの実験」と明示的に再定義する（未知の量を既知のように扱わない）。criteria 文書は実行前に「Codexによるレビューの実行方法」に従って Codex レビューを行い（レビュー依頼時に対象の criteria 文書と関連ドキュメントを明示する。結果は案件フォルダの `reviews/` に案件の連番を進めて保存）、重要度「高・中」ゼロに収束させてから実験に着手する（criteria lock）
2. **直前予測**: 各フェーズの実行直前に、入出力の予測を数値で実験ログ（例: `experiment_log.md`）に記録してから実行する。予測は前段の実測結果を使って直前に立てる（全フェーズ一括の事前予測はしない——前段の結果で後段の内容が変わるため）。予測する項目の枠（何を予測するか）は criteria 文書に事前定義してよい（値はフェーズ直前に確定する）
3. **照合**: 実行後に予測と実測を照合して実験ログに記録する。乖離した場合はそのフェーズの前提・理解を疑い、原因を特定してから次フェーズに進む
4. **事後解釈の禁止**: 基準のない実験は「測定」であって「判定」ではない。結果を見てから合格ラインを定めることや、事前基準にない条件を付けて合格扱いにすること（事後解釈での Go/No-Go 判定）を行わない

対象外: 通常の実装・自動テスト（pytest はテストコード自体が期待値＝予測の文書化に当たる）、および数値判定を伴わない調査（コード読解、ドキュメント調査等）。実運用の例は `docs/issues/feat-026-render-match-extrinsic-refinement/experiments/` の criteria 文書・experiment_log.md を参照。

### ドキュメント作成ルール

- **実装（反映）前に必ず案件種別に応じたドキュメントを作成し、案件フォルダにファイル保存すること**
  - feat: 要求仕様書（`requirements.md`、`docs/REQUIREMENTS_STANDARD.md` 準拠）と機能設計書（`design.md`、`docs/DESIGN_STANDARD.md` 準拠）
  - bug: 修正計画（`investigation.md`、`docs/BUGFIX_STANDARD.md` 準拠）。要求仕様書・機能設計書の変更が必要な場合はその変更案も併せて保存する
  - update: 調査記録（`README.md`）と反映設計書（`design.md`）。詳細は「ドキュメント更新フロー（update-XXX 案件）」に従う
- ドキュメントが保存されていない場合は、**実装を中止**する
- レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
- ドキュメントは `docs/issues/{案件フォルダ}/` に置く。ファイル名は上記の案件種別ごとの必須ドキュメント定義に従う
- **/clear 後でも実装がスムーズにできるよう、必要な情報を全て記述する**
- 暗黙知に頼らず、**自己完結したドキュメント**にする（前の会話コンテキストがなくても実装できること）
- ライブラリの追加・変更・削除を行った場合は `docs/TECH_STACK.md` も更新すること
- 新規ライブラリ導入時は用途・選定理由・バージョンを `TECH_STACK.md` に追記すること

### 案件ディレクトリ構成

```
docs/issues/
└── {type}-{number}-{slug}/    # 例: bug-001-xxx, feat-001-yyy, update-001-zzz
    ├── README.md              # 概要、ステータス、再現手順
    ├── requirements.md        # 要求仕様書（機能追加時、REQUIREMENTS_STANDARD.md 準拠）
    ├── design.md              # 機能設計書（機能追加時、DESIGN_STANDARD.md 準拠）
    ├── investigation.md       # 不具合の調査・修正計画（BUGFIX_STANDARD.md 準拠）
    └── reviews/               # Codexレビューの結果（codex-NN.result.md のみ git 管理。full.log は gitignore）
```

update 案件は requirements.md / investigation.md を持たず、README.md（調査）・design.md（反映設計）・reviews/ で構成される。

### 命名規則

- フォルダ名は英語で統一（例: `bug-001-reprojection-error`, `feat-002-multi-camera-support`）
- 案件フォルダは完了後も削除・移動しない

### Codexによるレビューの実行方法

機能追加・不具合修正・ドキュメント更新フローのステップ4（レビュー）では、Claude Code 自身が `codex exec` コマンドを実行して Codex にレビューさせる。Subagent は使わない。**Codex は逐次（前回セッションを `resume` で継続）で回し、重要度「高・中」がゼロに収束してから人レビューに進む**。並列にはしない（再レビューの収束確認＝「前回指摘が直ったか」の判定に前回文脈の引き継ぎが必要なため。初回の発見網羅性を上げたい大規模案件でのみ「初回だけ多観点並列→以降逐次」を検討）。

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
  "docs/REVIEW_CRITERIA.md および docs/BUGFIX_STANDARD.md の基準に従い、以下のドキュメントをレビューせよ: docs/issues/{案件フォルダ}/investigation.md 。requirements.md / design.md を変更した場合はそれらもレビュー対象に含めること。瑣末な点へのクソリプはしないで、致命的な点のみ指摘して。発見した問題を重要度(高/中/低)で分類し、修正提案とともに報告すること。" \
  > docs/issues/{案件フォルダ}/reviews/codex-01.full.log 2>&1
```

#### 初回レビュー（ドキュメント更新の場合）

```bash
mkdir -p docs/issues/{案件フォルダ}/reviews
codex exec -o docs/issues/{案件フォルダ}/reviews/codex-01.result.md \
  "以下のドキュメントをレビューせよ: docs/issues/{案件フォルダ}/README.md docs/issues/{案件フォルダ}/design.md 。レビュー観点は次の3点: (1) 反映計画の自己完結性（design.md だけで作業ができるか） (2) 情報の喪失（削除・置換対象に、他所に存在しない情報が含まれていないか） (3) 変更後のドキュメント間整合性（参照切れ、矛盾、案件の漏れ・重複）。瑣末な点へのクソリプはしないで、致命的な点のみ指摘して。発見した問題を重要度(高/中/低)で分類し、修正提案とともに報告すること。" \
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

1. **必読ドキュメントと読む順序**: CLAUDE.md → 案件ドキュメント（機能追加は `requirements.md` と `design.md`、不具合修正は `investigation.md` と、変更した場合は関連する `requirements.md` / `design.md` も必読）→ 変更対象コード → 参考にする既存テスト
2. **厳密準拠の指示**: 設計書・修正計画に厳密に従うこと。書かれていない独自判断・改善・リファクタは一切禁止
3. **想定外事象の扱い**: 想定外の事象（設計書どおりに実装できない、ドキュメントと実コードの矛盾、テストが通らない等）が発生したら、**その場で回避策を実装せず直ちに中断**し、何が起きたか・どこまで完了したかを報告して終了すること。報告を受けたら「調査・計画 → requirements.md / design.md（または investigation.md）の修正」のステップに**必ず戻る**（レビューを経てから実装を再開する）
4. **検証まで実施**: `uv run pytest -v` の全件実行、`tests/results/{type}-{number}_test_result.txt` への出力保存、ドキュメントに定義された動作確認（実データ実行等）
5. **禁止事項**: git commit / push はサブエージェントに行わせない。BACKLOG.md / CHANGELOG.md / CLAUDE.md / README.md の更新も行わせない（完了ステップ8で Claude Code 本体が行う）
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


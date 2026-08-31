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
├── AGENTS.md                          # Codex が起動時に読む指示ファイル（レビュー定型指示。update-003）
├── README.md
├── pyproject.toml                     # phase0 の uv パッケージ管理
├── uv.lock
├── docs/                              # ドキュメント（案件管理 + 開発プロセス基準）
│   ├── BACKLOG.md                     # 案件一覧
│   ├── CHANGELOG.md                   # リリース履歴
│   ├── BUGFIX_STANDARD.md             # 不具合修正の記述基準
│   ├── DESIGN_STANDARD.md             # 機能設計書の記述基準
│   ├── HERDR_SETUP.md                 # Herdr エージェント連携セットアップ手順（update-003）
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
│   ├── render_keypoints.py            # 3DGSレンダリング＋人体キーポイント重ね描き（オクルージョン考慮、全フレーム連番PNG/MP4出力、欠損マーカー許容、キーポイント入力は C3D/NPZ 両対応〔拡張子 .npz で判別、pnp_ok は無視して全フレーム描画〕、--no-png でMP4のみ出力、--no-keypoints/--distort で歪み対応静止画モード、--config YAML対応〔CLI > YAML > 既定値〕、--fps-frustum でFPSカメラ視錐台ワイヤフレーム重ね描き〔FOVは --fps-camera/--fps-toml のTOMLから、奥行き --frustum-depth 既定0.5m、頭部点欠損フレームは非描画〕。feat-015/016/017/021/022/024/032/033/034）
│   ├── npz_to_c3d.py                  # NPZ（リフトアップ済み3Dキーポイント）→ C3D 変換（Blender io_anim_c3d 取り込み対応。feat-018）
│   ├── filter_c3d.py                  # C3Dキーポイントの時間方向平滑化（Butterworth 2次 filtfilt・ゼロ位相。feat-020）
│   ├── filter_npz.py                  # NPZキーポイントの時間方向平滑化（NPZ→NPZ、C3D・Blender不要、NaN区間維持・追加キー無加工コピー。feat-028）
│   ├── refine_extrinsics.py           # 手動点（一意6点以上）+ LoFTR 自動マッチングによる外部パラメータ精緻化バッチ（K既知、サンプリング型受理判定、Calib_scene.toml型出力+診断レポート。feat-026）
│   ├── adjust_extrinsics_multiview.py # 8台多視点同時外部パラメータ調整（MASt3Rクロス対応点〔エピポーラ整合フィルタつき〕+ 3DGSアンカーのバンドル調整、K既知。初期ポーズは全台 feat-026 精緻化済みを --init-cameras で明示指定。ホールドアウト評価・診断レポート・ポーズ確定カメラのみのTOML出力。feat-035）
│   ├── render_fps_video.py            # NPZ直読みFPS動画一括生成（頭部キーポイントからポーズをnumpy計算、ピンホールK、NaN=黒画面、チャンク分割+耐久書き出し+ffprobe破損検査つき再開、静止画/ポーズダンプ排他モード、--config YAML対応、--gpus でチャンク並列レンダリング〔単一GPU複数ワーカー/複数GPU対応、動的分配、失敗時即時中止〕、完了時に <MP4名>_info.txt を自動保存。feat-027/029/030/031）
│   └── data/                          # データファイル（gitignore）
├── matcher_lab/                       # 学習ベースマッチャー環境（独立した uv 環境。feat-026）
│   ├── pyproject.toml                 # uv パッケージ管理（Python 3.12 / torch cu130 / kornia）
│   ├── loftr_cli.py                   # LoFTR 推論 CLI（refine_extrinsics.py から subprocess で呼ばれる正式コンポーネント。feat-026）
│   ├── mast3r_cli.py                  # MASt3R ペアマッチング CLI（adjust_extrinsics_multiview.py から subprocess で呼ばれる正式コンポーネント。feat-035）
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
4. **レビュー（Codex → 人）** → 保存されたドキュメントを **Codex** でレビューする。実行方法は後述の「Codexによるレビューの実行方法（Herdr 対話方式）」を参照。**まず Codex の再帰レビュー（修正→再レビュー）を重要度「高・中」がゼロに収束するまで回し、その後に人（ユーザー）がレビューする**（収束前に人レビューはしない）。レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
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
4. **レビュー（Codex → 人）** → 保存されたドキュメントを **Codex** でレビューする。実行方法は後述の「Codexによるレビューの実行方法（Herdr 対話方式）」を参照。**まず Codex の再帰レビュー（修正→再レビュー）を重要度「高・中」がゼロに収束するまで回し、その後に人（ユーザー）がレビューする**（収束前に人レビューはしない）。レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと
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
4. **レビュー（Codex → 人）** → 「Codexによるレビューの実行方法（Herdr 対話方式）」に従う（重要度「高・中」ゼロ収束後に人レビュー）。レビュー対象は `README.md` と `design.md`。レビュー観点は次の3点を明示して依頼する:
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

1. **判定基準の事前定義**: 実験の実行前に「答える問い」「判定閾値」「合格ライン（何%・何件なら合格か）」を数値で criteria 文書（案件フォルダ配下、例: `docs/issues/{案件フォルダ}/experiments/{実験名}/criteria.md`）に文書化する。数値で定義できない場合は、その実験を「基準を測ることがゴールの実験」と明示的に再定義する（未知の量を既知のように扱わない）。criteria 文書は実行前に「Codexによるレビューの実行方法（Herdr 対話方式）」に従って Codex レビューを行い（レビュー依頼時に対象の criteria 文書と関連ドキュメントを明示する。結果は案件フォルダの `reviews/` に案件の連番を進めて保存）、重要度「高・中」ゼロに収束させてから実験に着手する（criteria lock）
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
    └── reviews/               # Codexレビューの結果（codex-NN.result.md。git 管理）
```

update 案件は requirements.md / investigation.md を持たず、README.md（調査）・design.md（反映設計）・reviews/ で構成される。

### 命名規則

- フォルダ名は英語で統一（例: `bug-001-reprojection-error`, `feat-002-multi-camera-support`）
- 案件フォルダは完了後も削除・移動しない

### Codexによるレビューの実行方法（Herdr 対話方式）

機能追加・不具合修正・ドキュメント更新フローのステップ4（レビュー）では、Herdr の隣接ペインで対話モードの codex を稼働させ、Claude Code が `herdr` CLI 経由でレビューを依頼する。

**前提環境（必須・フォールバックなし）**: Herdr（`HERDR_ENV=1` の Herdr 管理ペインで Claude Code が稼働していること）と codex CLI。セットアップは `docs/HERDR_SETUP.md` を参照。環境が満たされない場合はレビューを実施せず、ユーザーに報告して指示を仰ぐ。なお `docs/HERDR_SETUP.md` に記載のコマンド例のタイムアウト値（120000）はセットアップ時の動作確認・切り分け用であり、レビュー運用では本節の値を使う。

**モデル・reasoning effort 等の設定**: `~/.codex/config.toml` のデフォルトに従う（起動コマンドでモデル指定をしない）。ストリーム起動後にレビューごとの切替は行わない。変更が必要な場合はエージェントを終了し、設定変更後に再起動する。AGENTS.md も起動時にのみ読み込まれるため、変更した場合は稼働中のレビューストリームを終了して起動し直す。

#### レビューストリーム

- **1ストリーム = codex 1体との1本の会話**。レビューの並列化・文脈保持・記録の単位
- 命名規則: `rev-lift2d-{案件ID}`（プロジェクト略称は `lift2d`。例: `rev-lift2d-update-003`）。案件を細分化して並列に進める場合は `rev-lift2d-{案件ID}-{細分化単位}`（例: `rev-lift2d-feat-036-p1`）。細分化単位の識別子は、細分化を決めた時点で案件ドキュメント（README.md または design.md の分割定義）に明記する（名前をタスクから決定論的に導出するため）
- **排他規則: 1ストリームに同時に流せる依頼は1本のみ**。ストリーム間は並列可（想定: 案件間並列、案件内の細分化単位間並列）。作業中の codex への追加依頼はエラーにならず同一会話に混入するため、規則で防ぐしかない
- 他の Claude Code や他案件のストリームには一切 `prompt` を送らない

#### エージェントのライフサイクル

1. **生存確認と状態確認**: 依頼前に `herdr agent list` で当該ストリーム名を確認する。存在する場合は `agent_status` を見て次のとおり扱う（レビューのたびに起動しない）:
   - `idle` / `done`: そのまま再利用して依頼する
   - `working`: 依頼中の作業が終わるまで待つ（`herdr agent wait {ストリーム名} --timeout 1800000`。タイムアウト値はレビュー依頼の待機と同じ値を使う）。作業中の codex への追加依頼は同一会話に混入するため送ってはならない
   - `blocked` / `unknown`: `agent read --source visible` で画面を確認し、原因が特定できて安全に解消できる場合（例: ディレクトリ信頼確認）のみ対処する。それ以外はユーザーに報告する
2. **起動（不在時のみ）**: ペインを分割して起動する

   ```bash
   herdr pane split --current --direction right --cwd "$PWD" --no-focus
   # 戻り値 JSON の .result.pane.pane_id を次で使う
   herdr agent start {ストリーム名} --kind codex --pane {pane_id} -- --sandbox read-only --ask-for-approval never
   ```

   `--sandbox read-only --ask-for-approval never` は必須（承認なし運用と書き込み権を同時に与えない・レビュアーの役割強制・同時書き込み競合の防止）
3. **起動直後の画面確認（必須）**: `herdr agent read {ストリーム名} --source visible` で画面を読み、入力待ち（プロンプト表示）であることを確認してから依頼する。更新ダイアログ・信頼確認ダイアログが表示されたまま依頼を送ると、依頼文がダイアログを誤操作する（実績あり: 更新ダイアログを押して codex が自己更新・終了した）。ディレクトリ信頼確認は `herdr agent send-keys {ストリーム名} enter` で継続してよい。それ以外の想定外ダイアログはユーザーに報告する
4. **終了**: 案件（細分化単位）のレビューが収束し人レビューを通過したら、完了処理の際に `/quit` を送ってエージェントを終了し、`herdr pane close {pane_id}` でペインを撤去する。`{pane_id}` は `herdr agent list` の当該エージェントの `pane_id` フィールドから取得する（新規起動時・再利用時とも、この方法なら取得できる）

#### 依頼の送り方

- `herdr agent prompt {ストリーム名} "{依頼文}" --wait --timeout 1800000` を **Bash の `run_in_background` で実行**する（レビューは10分を超えることがあり、フォアグラウンドでは Bash のタイムアウト上限に抵触する。`--wait` は送信後5秒以内の状態変化を要求するため、誤送信は `agent_prompt_stalled` で早期に露見する）
- 完了通知後、`herdr agent read {ストリーム名} --source recent-unwrapped --lines {十分な行数}` で結果を回収する。**回答冒頭に「[AGENTS.md適用]」マーカーが無い場合は `AGENTS.md` が読み込まれていない**ため、結果を採用せずユーザーに報告する
- `agent_prompt_stalled` が返ったら、画面を読んで原因（ダイアログ・エージェント消滅等）を確認する

#### レビューの進め方

1. **初回レビュー**: 案件種別ごとの基準と対象ファイルを指定して依頼する（依頼文は後述）
2. **修正**: 高・中の指摘は**全件まとめて反映**してから次の依頼に進む（1件ごとに再レビューを回さない）
3. **再レビュー（解消確認）**: 「前回の指摘に全件対応してドキュメントを更新した。各指摘が解消されたかを確認し、指摘ごとに解消/未解消を判定して報告して。変更点に新たな問題があれば併せて指摘して。」と依頼する。diff の添付は不要（codex が自身の記憶と再読で解消判定できることを検証済み）。高・中ゼロになるまで 2〜3 を反復する
4. **全文ゲート**: 高・中ゼロになったら `/new` を送って会話をクリアし（「新しい目」の確保。ペイン・エージェント名はそのまま）、全文・全基準のレビューを依頼する
5. **ゲートで指摘が出たら**: 全件まとめて反映し、同じ会話（`/new` はしない）で解消確認を依頼する
6. **収束**: 全文ゲートで高・中ゼロが出たら人（ユーザー）レビューに進む（収束前に人レビューはしない）
- 同一会話の回数上限は設けない（案件が適正サイズなら問題にならない。運用して必要になったら検討する）。「行き詰まり検出」（本ファイル「Claude Code 運用ルール」参照）は常時適用する
- レビュー実行時は `docs/REVIEW_CRITERIA.md` の基準に従うこと

#### 依頼文の基準部分（案件種別ごと）

依頼は**一括型**とする: 1回の依頼で対象ファイルを全件指定し、全文をレビューさせる（1ファイルずつの逐次依頼はしない。ドキュメント間整合性の判定に同一ターンでの突き合わせが必要なため）。

定型指示（瑣末な指摘の抑止・重要度(高/中/低)分類・修正提案の要求・適用マーカー）はリポジトリ直下の `AGENTS.md` が起動時に供給するため、**依頼文には書かない**。

- **機能追加**: 「docs/REVIEW_CRITERIA.md の基準に従い、以下のドキュメントをレビューせよ: {requirements.md と design.md のパス} 。」
- **不具合修正**: 「docs/REVIEW_CRITERIA.md および docs/BUGFIX_STANDARD.md の基準に従い、以下のドキュメントをレビューせよ: {investigation.md のパス} 。requirements.md / design.md を変更した場合はそれらもレビュー対象に含めること。」
- **ドキュメント更新**: 「以下のドキュメントをレビューせよ: {README.md と design.md のパス} 。レビュー観点は次の3点: (1) 反映計画の自己完結性 (2) 情報の喪失 (3) 変更後のドキュメント間整合性。」
- **全文ゲート**: 上記に「本ドキュメントは対話ベースのレビューを経ている。今回は最終ゲートとして、文書全体の整合性（セクション間の矛盾、参照切れ、抜け）を含めて全文を確認して。」を加える

#### 結果の保存

- Claude Code が `agent read` で回収した内容を整形し、`docs/issues/{案件フォルダ}/reviews/codex-NN.result.md` に保存する（連番はストリームごとではなく案件で通し。事前に `mkdir -p`）。git 管理する。並列実行時は、依頼の開始前に親 Claude Code が各ストリームへ保存先の連番を割り当てて指示に含める（実行中に各自が採番すると競合するため）
- 冒頭メタ: 日付・対象ファイル・ストリーム名・フェーズ（反復/ゲート。`/new` の実施を明記）・指摘数（高/中/低）・トークン実測。トークンは `~/.codex/sessions/` 配下の当該 rollout jsonl（`total_token_usage` の累積値）から取得し、jsonl のパスも記録する
- 過程ログ（旧 full.log）は保存しない（過程は rollout jsonl と Herdr ペインに残る）

#### サブエージェントへの委任（並列レビュー時）

複数ストリームを並列に回す場合、各ストリームの定型作業（生存確認→起動→依頼送信→待機→回収→result.md 保存）はサブエージェントに委任してよい。指示に必ず含めること:

1. 担当ストリーム名（明示。それ以外のエージェントへの `prompt` は禁止）
2. 依頼文・対象ファイル・result.md の保存先と連番（連番は親が事前に確定して渡す）
3. 起動時の画面確認手順と、想定外事象（`agent_prompt_stalled`・ダイアログ・エージェント消滅）が起きたら中断して報告する指示

指摘への対応（ドキュメント修正）と収束判定は Claude Code 本体が行う。

> **Ubuntu 24系で `bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted` が出る場合**は、`docs/codex-exec-ubuntu24-bwrap-fix.md` を参照して AppArmor プロファイルを追加すること（ホスト側の user namespace 制限が原因。Codex のバグではない）。

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

- 調査・計画、ドキュメント作成、Codexレビューの指摘反映と収束判定、完了処理（ステップ8）、git 操作は Claude Code 本体が行う（レビューの定型作業の委任は「Codexによるレビューの実行方法（Herdr 対話方式）」の規定に従う）

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

### 行き詰まり検出（全作業共通・必須）

レビュー対応・デバッグ・実装・調査を問わず、あらゆる反復作業に適用する。

- **同じ指摘の再発、または同じ原因での失敗が2回続いたら、事前の想定自体が間違っている**。3回目の試行に進んではならない
- 発動したら: いまの解決案を明示的に取り下げ、前提（原因の想定・アプローチ・そもそもの問い）に立ち返って代替案を挙げ、方針を選び直す。必要なら調査からやり直す
- 再検討の結果、同じ解決案に行き着いた場合の再採用は可（検討し直したこと自体に意味がある）
- 禁止: 微修正を重ねて試行回数を積むこと

### Bash 実行時のルール

- **`cd <path> && <command>` の連結は禁止。** Bashツールはプロジェクト作業ディレクトリで動くため `cd` は不要。連結すると先頭トークンが `cd` になり、`.claude/settings.json` / `.claude/settings.local.json` のallowlist（例: `Bash(herdr *)`、`Bash(git status)`）が一致せず、毎回パーミッションプロンプトが発生する
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


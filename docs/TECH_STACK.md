# 技術スタック

本プロジェクトは独立した3つの uv 環境で構成される。

- **phase0**（プロジェクトルート）: カメラパラメータ推定（NumPy / SciPy / OpenCV）
- **phase4**（`phase4/`）: gsplat バッチレンダリング（torch / gsplat、CUDA）
- **matcher_lab**（`matcher_lab/`）: 学習ベースマッチャー（LoFTR / MASt3R）の評価実験環境（feat-026 候補3。2026-08-02 新設）

バージョンは `pyproject.toml`（定義・下限）と `uv.lock`（固定）に基づく。phase0 / phase4 は Python 3.10、matcher_lab は Python 3.12 環境での固定値を記載する。

## プロジェクト基盤

| 項目 | 値 | ソース |
|---|---|---|
| 言語 | Python 3.10（`requires-python = ">=3.10"`） | pyproject.toml / .python-version |
| パッケージ管理 | uv（phase0・phase4 で別環境。各 `pyproject.toml` + `uv.lock`） | CLAUDE.md |
| 対象OS | Linux（開発機 gtune2） | 環境 |
| GPU | NVIDIA RTX 5060 Ti（sm_120） ※phase4 のみ | 環境 |
| システムCUDA Toolkit | 12.6（nvcc）※sm_120 非対応のため後述の回避策が必要 | 環境 |

## phase0（カメラパラメータ推定）

### 外部ライブラリ（ルート `pyproject.toml`）

| ライブラリ名 | 定義 | uv.lock 固定 | 用途（1行） | 使用箇所 | 選定理由（1行） |
|---|---|---|---|---|---|
| numpy | >=2.2.6 | 2.2.6 | 行列演算・数値計算 | estimate_camera_params.py, phase0_verification.py, verify_triangulation.py, common.py | 数値計算の標準ライブラリ |
| opencv-python | >=4.13.0.92 | 4.13.0.92 | solvePnP・projectPoints によるカメラキャリブレーション | estimate_camera_params.py, phase0_verification.py, verify_triangulation.py | カメラキャリブレーションの定番 |
| scipy | >=1.15.3 | 1.15.3 | least_squares（Levenberg-Marquardt法）によるパラメータ最適化 | estimate_camera_params.py, phase0_verification.py | 非線形最小二乗最適化に使用 |
| tomli | >=2.0.0 | 2.4.0 | Calib_scene.toml の読み込み（K既知モード等） | estimate_camera_params.py, convert_toml_to_csv.py | Python 3.10 に tomllib が無いため |
| c3d | >=0.6.0 | 0.6.0 | C3D モーションキャプチャデータの読み込み | （phase0 スクリプトでの直接使用なし。phase4/render_keypoints.py で使用） | キーポイント入力形式対応 |

### 開発用依存（dependency-groups.dev）

| ライブラリ名 | 定義 | uv.lock 固定 | 用途 | 使用箇所 |
|---|---|---|---|---|
| pytest | >=9.0.2 | 9.0.2 | テスト実行 | tests/ |

### Blender内蔵ライブラリ

| ライブラリ名 | バージョン | 用途（1行） | 使用箇所 |
|---|---|---|---|
| bpy | Blender同梱版 | Blenderシーンからの3D基準点座標抽出 | blender/mk_points_3d.py |

### 標準ライブラリ

| ライブラリ名 | 用途（1行） | 使用箇所 |
|---|---|---|
| argparse | コマンドライン引数の解析 | 各メインスクリプト |
| csv | CSV ファイルの読み書き | 各メインスクリプト, blender/mk_points_3d.py |
| pathlib | ファイルパス操作 | 各メインスクリプト |
| re | TOML 等の正規表現パース | convert_toml_to_csv.py, blender/mk_points_3d.py |
| sys / os | 終了コード・パス操作 | 各メインスクリプト |
| itertools (combinations) | カメラペアの全組み合わせ生成 | verify_triangulation.py |
| collections (defaultdict) | カメラ別・ペア別の集計 | verify_triangulation.py |

## phase4（gsplatバッチレンダリング）

### 外部ライブラリ（`phase4/pyproject.toml`）

| ライブラリ名 | 定義 | uv.lock 固定 | 用途（1行） | 使用箇所 | 選定理由（1行） |
|---|---|---|---|---|---|
| torch | >=2.0 | 2.10.0（+cu128） | gsplat のテンソル演算・座標変換（CUDA） | render.py, render_keypoints.py | gsplat のバックエンド |
| gsplat | >=1.0.0,<2.0.0 | 1.5.3 | 3D Gaussian Splatting のラスタライズ | render.py, render_keypoints.py | 3DGS レンダリングの中核 |
| numpy | （指定なし） | 2.2.6 | 配列演算・座標変換 | camera_pose.py, render.py, render_keypoints.py | 数値計算の標準ライブラリ |
| plyfile | （指定なし） | 1.1.3 | PLY（3DGS）ファイルの読み込み | render.py | PLY パースの定番 |
| Pillow | （指定なし） | 12.1.1 | レンダリング結果の PNG 保存 | render.py | 画像入出力 |
| opencv-python | >=4.13.0.92 | 4.13.0.92 | キーポイント重ね描き・画像処理 | render_keypoints.py | 描画・画像処理 |
| tomli | >=2.4.1 | 2.4.1 | カメラパラメータ TOML の読み込み | render_keypoints.py | Python 3.10 に tomllib が無いため |
| c3d | >=0.6.0 | 0.6.0 | C3D キーポイントデータの読み書き | render_keypoints.py, npz_to_c3d.py, filter_c3d.py | キーポイント入力形式対応 |
| scipy | >=1.11 | 1.15.3 | butter/filtfilt によるC3Dキーポイントの時間方向平滑化（ゼロ位相ローパス） | filter_c3d.py | ゼロ位相Butterworthフィルタはモーキャプ後処理の標準手法で、SciPyが事実上の標準実装（feat-020） |
| packaging | >=26.0 | 26.0 | gsplat の CUDA 拡張 JIT ビルド時のバージョン判定 | （ビルド時依存） | gsplat ビルドが要求 |
| setuptools | >=82.0.1 | 82.0.1 | gsplat の CUDA 拡張 JIT ビルド | （ビルド時依存） | gsplat ビルドが要求 |

### Blender内蔵ライブラリ

| ライブラリ名 | バージョン | 用途（1行） | 使用箇所 |
|---|---|---|---|
| bpy | Blender同梱版 | カメラポーズ（外部パラメータ）の書き出し | camera_pose.py |

### 標準ライブラリ

| ライブラリ名 | 用途（1行） | 使用箇所 |
|---|---|---|
| argparse | コマンドライン引数の解析 | camera_pose.py, render.py, render_keypoints.py |
| json | カメラポーズ JSON の読み書き | camera_pose.py, render.py |
| subprocess | ffmpeg 呼び出し（MP4 エンコード） | render.py, render_keypoints.py |
| shutil | ファイル・ディレクトリ操作 | render.py, render_keypoints.py |
| time | レンダリング処理時間の計測 | render.py, render_keypoints.py |
| sys / os | 終了コード・パス操作 | camera_pose.py, render.py, render_keypoints.py |

## phase4 の CUDA 実行環境（重要）

phase4 の gsplat レンダリングは環境変数 `TORCH_CUDA_ARCH_LIST="9.0+PTX"` を付けて実行する必要がある（2026-06-11 時点、開発機 gtune2）。

```bash
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render.py data/project.ply data/FPSCamera_poses.json --mp4
```

- gsplat 1.5.3 には torch 2.10+cu128 向けのビルド済み wheel がなく、初回実行時に CUDA 拡張が JIT コンパイルされる
- 環境変数なしだと GPU（RTX 5060 Ti = sm_120）が検出され、システムの nvcc（CUDA 12.6）が sm_120 非対応のため `nvcc fatal: Unsupported gpu architecture 'compute_120'` でビルドが失敗する
- この環境変数で compute_90 の PTX を生成し、ドライバの JIT 変換で sm_120 上で実行する（ビルドキャッシュは `~/.cache/torch_extensions/py310_cu128/gsplat_cuda/`）
- 恒久対策は CUDA Toolkit 12.8 以上のインストール（その場合この環境変数は不要になる）
- なお torch 同梱の CUDA ランタイムは cu128（`nvidia-cuda-runtime-cu12 12.8.90`）。問題はシステム nvcc（12.6）が JIT ビルドに使われる点にある

## matcher_lab（学習ベースマッチャー評価。feat-026 候補3）

phase4 とは独立した実験環境。LoFTR（kornia 0.8.3 以降）が Python>=3.11 を要求し、phase4（Python 3.10）に同居できないため新設した（2026-08-02）。

### 外部ライブラリ（`matcher_lab/pyproject.toml`）

| ライブラリ名 | 定義 | uv.lock 固定 | 用途（1行） | 使用箇所 | 選定理由（1行） |
|---|---|---|---|---|---|
| kornia | >=0.8.3 | 0.8.3 | LoFTR（detector-free 学習ベースマッチャー）の組み込み実装 | loftr_smoke.py | 公式 zju3dv 実装より依存が軽く、pip のみで導入可（Apache 2.0） |
| torch | >=2.0 | 2.13.0（+cu130） | LoFTR / MASt3R の推論バックエンド | loftr_smoke.py, mast3r_smoke.py | 両マッチャーの必須依存 |
| torchvision | （指定なし） | 0.28.0（+cu130） | dust3r の画像前処理（transforms） | mast3r 経由 | dust3r/requirements.txt 記載 |
| roma | （指定なし） | 1.5.7 | dust3r の回転表現ユーティリティ | mast3r 経由 | dust3r/requirements.txt 記載 |
| einops | （指定なし） | 0.8.2 | MASt3R/dust3r のテンソル整形 | mast3r 経由 | dust3r/requirements.txt 記載 |
| huggingface-hub[torch] | >=0.22 | 1.26.0 | dust3r のモデル読み込み基盤 | mast3r 経由 | dust3r/requirements.txt 記載 |
| tqdm / scipy / matplotlib / trimesh | （指定なし） | 4.70.0 / 1.18.0 / 3.11.1 / 5.0.0 | dust3r の推論・可視化系依存 | mast3r 経由 | dust3r/requirements.txt 記載（gradio / tensorboard / pyglet はデモ・学習用のため除外） |
| numpy / opencv-python / pillow | （指定なし） | 2.5.1 / 5.0.0.93 / 12.3.0 | 画像入出力・配列演算 | 各スクリプト | 共通基盤 |

### 外部アセット（git 管理外）

| アセット | 場所 | 入手元 | ライセンス |
|---|---|---|---|
| MASt3R リポジトリ（dust3r / croco サブモジュール込み） | `~/git/mast3r` | `git clone --recursive https://github.com/naver/mast3r` | CC BY-NC-SA 4.0（非商用研究は許諾。本プロジェクトは大学研究のため使用可） |
| MASt3R チェックポイント（2.75GB） | `~/data/models/mast3r/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth` | download.europe.naverlabs.com | 同上 + CHECKPOINTS_NOTICE |
| LoFTR 重み（outdoor、44.2MB） | `~/.cache/torch/hub/checkpoints/loftr_outdoor.ckpt` | kornia が初回実行時に自動ダウンロード | Apache 2.0 |

### CUDA 実行環境（phase4 との違い）

- torch 2.13.0+cu130 の PyPI 標準 wheel は RTX 5060 Ti（sm_120）をネイティブサポートし、**CUDA 拡張の JIT コンパイルが発生しないため `TORCH_CUDA_ARCH_LIST` は不要**（phase4 の gsplat とは事情が異なる）
- MASt3R の RoPE2D CUDA カーネル（curope）は未コンパイル。起動時に `Warning, cannot find cuda-compiled version of RoPE2D, using a slow pytorch version instead` が出るが、純 torch フォールバックで動作する（速度低下のみ）

## バージョン固定ポリシー

| 項目 | 内容 |
|---|---|
| 管理ファイル | 各 `pyproject.toml`（定義）、各 `uv.lock`（ロックファイル） |
| バージョン指定方針 | 主要ライブラリは `>=下限` 指定（互換性のある最新版を許容）。gsplat のみメジャー上限を固定（`<2.0.0`） |
| ロック運用 | uv.lock で再現可能なインストールを保証（phase0・phase4 で別々に管理） |
| numpy / scipy の補足 | Python 3.10 では numpy 2.2.6 / scipy 1.15.3 に固定（より新しい版は Python 3.11+ が必要）。uv.lock には Python バージョンマーカー別に複数版が記録される |

## 制約・禁止事項

- phase0 と phase4 は依存が大きく異なるため、**必ず別 uv 環境で管理する**（同一環境に混ぜない）
- phase4 では torch/gsplat/bpy/c3d を pytest（ルート環境）に巻き込まないよう、一部は関数内 import としている
- 使用禁止ライブラリの定義は現時点でなし

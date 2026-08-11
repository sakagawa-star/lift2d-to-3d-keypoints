# 実験 e1: 単一GPU上のチャンク並列（案C）の効果測定 — 判定基準

- **案件**: feat-030（render_fps_video.py のチャンク並列レンダリング）
- **作成日**: 2026-08-10
- **状態**: **locked**（2026-08-10 Codex レビュー収束: codex-01 高2件 → 修正 → codex-02 指摘ゼロ。実験の実行開始はユーザーの承認を得てから）

## 1. 背景と答える問い

`phase4/render_fps_video.py` は「GPU描画 → GPU→CPU転送 → uint8変換 → ffmpeg(libx264) 書き込み」を1プロセスで完全直列に回している（feat-027 実測: 約0.022秒/描画フレーム）。案C（チャンク並列）は複数ワーカープロセスに別チャンクを割り当て、同一GPUを時分割共有させることで、あるプロセスのCPU作業中に別プロセスがGPUを使う重なりを作る。

- **Q1**: 単一GPU（RTX 5060 Ti 16GB）上で、同一のレンダリング処理を2プロセス同時実行すると、合計スループット（描画フレーム/秒）は単独実行の何倍になるか。
- **Q2**: 1プロセスあたりのVRAM使用量はいくらで、16GBに2〜3プロセスが同居できるか。
- **Q3**（補助・判定には使わない）: 単独実行中のGPU使用率はどの程度か（並列余地の裏付けデータ）。

## 2. 判定基準（実行前に lock する）

### 2.1 主判定: スループット比 R2

定義: `R2 = (2プロセス同時の合計スループット) / (単独実行のスループット)`（算出方法は §5）

| 条件 | 判定 |
|---|---|
| R2 ≥ 1.30 | **Go**: 案Cは単一GPUでも効果あり。feat-030 のスコープに単一GPU高速化を含める |
| R2 < 1.10 | **No-Go**: 単一GPUでは効果なし（誤差・オーバーヘッド域）。案Cの価値は複数GPU分配機構のみとし、単一GPU高速化は案A/案Bの検討に回す |
| 1.10 ≤ R2 < 1.30 | **グレー**: Phase 3（3プロセス）を実施し、R3 ≥ 1.30 なら Go。R3 < 1.30 なら測定結果一式を添えてユーザー判断に委ねる |

閾値の根拠: フルNPZ（19.4万フレーム、単独実測見込み 約70〜80分）に対し、1.30倍は約17〜18分の短縮で分配機構の実装工数に見合う下限として設定。1.10未満は測定誤差とプロセス切替オーバーヘッドで説明できる域。

### 2.2 前提条件: VRAM 収容判定

Phase 1 で単独実行のプロセスVRAMピーク M1 [GiB] を実測し、次で判定する（GPU総量 16311 MiB ≒ 15.9 GiB、デスクトップ常駐分 D は Phase 0 で実測。マージン 1 GiB）:

- `2 × M1 + D + 1 GiB > 15.9 GiB` → Phase 2 は実施不可。「2プロセス不成立（VRAM不足）」として **No-Go** を記録（このPCでは案C不成立。A100〔80GB〕での成立性は別途机上評価）
- `3 × M1 + D + 1 GiB > 15.9 GiB` → Phase 3 は実施不可（グレー時はその旨を記録してユーザー判断へ）

### 2.3 判定しないこと（スコープ外）

- 出力MP4の正しさ（既存実装を無改変で使うため、feat-027 の等価性検証とテストで担保済み）
- 案A（プロセス内パイプライン）・案B（バッチレンダリング）の効果（本実験の対象外。No-Go 時の代替候補として名前のみ残す)
- A100 実機での並列効果（本実験は「このPCでの効果」のみを答える）

## 3. 実験環境（固定条件)

- マシン: gtune2（RTX 5060 Ti 16GB、i7-14700F 28論理コア、RAM 31GiB、ドライバ 580.95.05)
- デスクトップ環境（Xorg / gnome-shell 等）が同一GPUを常時使用中（Phase 0 で実測記録）。実験中はレンダリング以外の重いGPU/CPU作業を行わない
- 実行環境: `uv run --project phase4`、環境変数 `TORCH_CUDA_ARCH_LIST="9.0+PTX"` 必須（CLAUDE.md 参照）。gsplat の JIT ビルドキャッシュは構築済み
- ffmpeg: libx264 有効のユーザービルド（feat-027 で確認済み）

## 4. 入力データと実行コマンド

### 4.1 入力

- **NPZ**: `phase4/data/session001_f145749_world300.npz`（300フレーム・NaNゼロ＝全フレーム描画対象）を**時間方向に10回タイル**した 3000 フレームの実験用NPZを、本実験フォルダの固定スクリプト `make_tiled_npz.py`（入力・出力・タイル回数=10 をコード内定数で固定、引数なし）で生成する
  - 生成コマンド（リポジトリルートで実行）:
    ```bash
    uv run --project phase4 python \
      docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/make_tiled_npz.py
    ```
  - 生成物: `experiments/e1_single_gpu_concurrency/tmp/tiled_3000.npz`（git 管理外）
  - キー: `x3d_world (3000,22,3)` = 元の (300,22,3) を10回連結、`frame_ids (3000,)` = 145599 からの昇順連番（145599..148598）、`joint_names` = 無変更
  - **Phase 0 合格条件（すべて満たすこと。1つでも外れたら原因特定まで実験を進めない）**:
    1. `make_tiled_npz.py` の出力表示が `shape=(3000, 22, 3)`、`frame_ids=145599..148598`、`NaN数=0`
    2. ポーズダンプ検証: 下記コマンドで `--dump-poses` を実行し、JSON 内の `"valid": true` のフレーム数が **3000/3000**
       ```bash
       uv run --project phase4 python phase4/render_fps_video.py \
         /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply \
         docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/tiled_3000.npz \
         --toml phase4/data/Calib_FPSCamera.toml --camera FPSCamera --fps 30 \
         --dump-poses docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/poses_check.json
       uv run --project phase4 python -c "import json; d = json.load(open('docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/poses_check.json')); print(sum(1 for f in d['frames'] if f['valid']), '/', d['num_frames'])"
       ```
  - 理由: 全フレーム有効の一様なGPU負荷を既知データから決定的に作れる（NaN黒フレームはGPUを使わないため混入させない）。カメラポーズが300フレーム周期で繰り返すことは速度測定に影響しない
- **PLY**: `/home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply`（session001 の正しいPLY、139MB）
- **TOML**: `phase4/data/Calib_FPSCamera.toml`、カメラ名 `FPSCamera`

### 4.2 実行コマンド（Phase 1 の単独実行。Phase 2/3 は出力先だけ変えて同時起動）

```bash
PYTHONUNBUFFERED=1 TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/render_fps_video.py \
  /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply \
  docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/tiled_3000.npz \
  --toml phase4/data/Calib_FPSCamera.toml --camera FPSCamera \
  --fps 30 --gpu 0 --chunk-size 500 --crf 18 --preset medium \
  --output docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/run_p1.mp4 \
  2>&1 | python3 docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/ts_filter.py \
  > docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/tmp/run_p1.log
```

- `PYTHONUNBUFFERED=1` と `ts_filter.py`（本実験フォルダの固定スクリプト。各行に epoch 秒を前置）で、全ログ行に到着時刻を付与して保存する
- `--chunk-size 500` → 3000フレームが6チャンクに分割され、チャンク完了ログが6行得られる。実装の実ログ形式（`render_fps_video.py:953`）は次のとおり:
  ```
  チャンク NNNNN (fid FFFFFF-FFFFFF) 完了: 描画 n/N フレーム, S.S秒
  ```
- 出力・チャンクディレクトリ・ログはすべて `tmp/` 配下（git 管理外）。Phase 2 は `run_p2a.mp4` / `run_p2b.mp4`（ログは `run_p2a.log` / `run_p2b.log`）、Phase 3 は `run_p3a/b/c.mp4` とし、チャンクディレクトリ（`<output>.chunks`）が衝突しないことを起動前に確認する

### 4.3 計測方法

- **スループット**: ts_filter.py 経由で保存した各プロセスのログから、正規表現
  `チャンク ([0-9]{5}) \(fid [0-9]+-[0-9]+\) 完了: 描画 ([0-9]+)/[0-9]+ フレーム, ([0-9.]+)秒`
  で `(chunk_index, n_rendered, sec)` を抽出する（行頭の epoch 秒がタイムスタンプ）
- **GPU使用率・VRAM（GPU全体）**: 実験実行中に
  `nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used --format=csv,noheader,nounits -lms 500 > tmp/gpu_pN.csv`
  を並走させる（timestamp 列はローカル時刻 `YYYY/MM/DD HH:MM:SS.mmm`。ts_filter の epoch 秒と同一マシン時計なので相互変換して突き合わせる）
- **プロセス別VRAM**: `nvidia-smi --query-compute-apps=pid,used_gpu_memory --format=csv,noheader,nounits -lms 1000 > tmp/vram_pN.csv` を併用（M1 は最大値のみ使うためタイムスタンプ不要）
- **定常区間ウィンドウ（プロセス単位）**: `[チャンク00000完了行の epoch 秒, チャンク00005完了行の epoch 秒]`
- **同時性の担保（Phase 2/3）**: 全プロセスを5秒以内に起動する。全プロセスの定常区間ウィンドウについて `重なり率 = intersection_duration / union_duration` を算出し、**0.80 未満なら測定不成立として再実行**する（判定変更ではなく手続き規定）

## 5. 指標の算出方法

- **定常 秒/フレーム（プロセス単位)**: チャンク0を除外し（CUDA初期化・カーネル初回起動のウォームアップを含むため）、チャンク1〜5 の §4.3 抽出値による `Σsec / Σn_rendered`（= 2500フレーム分）
- **単独スループット**: `S1 = 1 / t1`（t1 = Phase 1 の定常 秒/フレーム）
- **2プロセス合計スループット**: `S2 = 1/t2a + 1/t2b`（t2a, t2b = 各プロセスの定常 秒/フレーム）
- **R2 = S2 / S1**、Phase 3 も同様に `R3 = (1/t3a + 1/t3b + 1/t3c) / S1`
- **M1**: Phase 1 実行中のプロセス別VRAMログの最大値
- **GPU使用率（Q3）**: Phase 1 の定常区間ウィンドウ（§4.3）内のタイムスタンプを持つ `gpu_p1.csv` の utilization.gpu 行の中央値

## 6. 実験手順と予測の枠

各 Phase の実行直前に、以下の項目を予測して `experiment_log.md` に記録してから実行する（予測値はフェーズ直前に確定。CLAUDE.md「実験・検証の進め方」準拠）。

| Phase | 内容 | 直前に予測する項目 |
|---|---|---|
| 0 | 前提確認: デスクトップ常駐VRAM（D）の記録、`make_tiled_npz.py` によるタイルNPZ生成と §4.1 合格条件（shape (3000,22,3)・frame_ids 145599..148598・NaN数 0・有効ポーズ 3000/3000）の検証 | タイルNPZの形状・frame_ids範囲・NaN数・有効ポーズ数 |
| 1 | 単独実行 ×1本 | t1（秒/フレーム）、M1（GiB）、GPU使用率中央値 |
| 2 | 2プロセス同時（§2.2 のVRAM判定を通過した場合のみ） | t2a・t2b、R2 |
| 3 | 3プロセス同時（**グレー判定時のみ実施**。§2.2 のVRAM判定を通過した場合のみ） | t3a〜c、R3 |

照合: 各 Phase 実行後、予測と実測を experiment_log.md に並記する。乖離した場合は原因を特定してから次 Phase に進む。

## 7. 成果物

- `experiments/e1_single_gpu_concurrency/experiment_log.md` — 予測・実測・照合・判定（git 管理）
- `experiments/e1_single_gpu_concurrency/make_tiled_npz.py` / `ts_filter.py` — lock 対象の固定スクリプト（git 管理）
- `experiments/e1_single_gpu_concurrency/tmp/` — タイルNPZ・出力MP4・チャンクディレクトリ・stdout ログ・nvidia-smi CSV（git 管理外）
- 判定結果（Go / No-Go / グレー→ユーザー判断)を本案件 README.md とスコープ決定に反映する

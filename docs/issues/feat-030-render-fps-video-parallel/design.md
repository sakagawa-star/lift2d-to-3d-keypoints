# feat-030 機能設計書: render_fps_video.py のチャンク並列レンダリング

- 作成日: 2026-08-10
- 準拠: `docs/DESIGN_STANDARD.md`
- 対応要求: `requirements.md`（feat-030）

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 `--gpus` オプション | §4.1（CLI/YAML）、§4.2（検証） |
| FR-002 動的分配による並列レンダリング | §4.3（アーキテクチャ）、§4.4（ワーカー）、§4.5（分配ループ） |
| FR-003 再開機構との互換 | §4.6 |
| FR-004 失敗時の即時中止 | §4.5（分配ループのエラー処理） |
| FR-005 進捗ログと集計 | §8（ログ設計） |

## 1.2 システム構成

変更対象は `phase4/render_fps_video.py` の1ファイルのみ。新規モジュール・新規ファイルは作らない（テストは `tests/test_feat030_parallel_render.py` を新設）。

```
phase4/render_fps_video.py
  ├─ 既存（無改変で流用）: render_chunk, durable_replace, verify_chunk_mp4,
  │   concat_chunks, plan_chunks, build_manifest, manifest_mismatch_keys,
  │   write_manifest, load_manifest, chunk_filename, format_render_average
  ├─ 変更: _build_parser（--gpus 追加、--gpu の default を None に変更）,
  │   CONFIG_CONVERTERS（"gpus" 追加）, main（排他検証・正規化・GPU範囲検証）,
  │   _run_mp4_mode（直列/並列の分岐。直列経路の処理内容は不変）
  └─ 新規: _gpus_list, select_pending_chunks, cleanup_tmp_files,
      _worker_main, _dispatch_loop, _run_parallel_chunks
```

依存方向: `_run_mp4_mode → _run_parallel_chunks → _dispatch_loop`、`_worker_main → render_chunk`。循環依存なし。ワーカーは `multiprocessing`（spawn）の子プロセスとして `_worker_main` を実行する。

## 1.3 技術スタック

- Python 3.10、`multiprocessing`（標準ライブラリ。**start method は "spawn" 固定**。fork は CUDA 初期化と両立しないため禁止）
- torch / gsplat / numpy / ffmpeg(libx264) / tomli: 既存のまま。**新規外部依存なし**（`docs/TECH_STACK.md` の更新不要）
- 選定理由: 子プロセス方式は e1 実験（2プロセス同時で R2=1.487）の実測構成と同じ実行形態であり、CUDAコンテキストをプロセス単位で分離できる

## 1.4 各機能の詳細設計

### §4.1 CLI / YAML（FR-001）

**argparse 変更**（`_build_parser`）:

```python
parser.add_argument("--gpu", type=_nonnegative_int, default=None,
                    help="使用GPU ID（default: 0。--gpus と併用不可）")
parser.add_argument("--gpus", type=_gpus_list, default=None,
                    help="並列ワーカーのGPU IDリスト（カンマ区切り、要素数=ワーカー数。"
                         "例: 0,0 はGPU 0に2ワーカー。--gpu と併用不可）")
```

- `--gpu` の default を `0` から `None` に変更する（併用検出のため）。**正規化は `parse_args_with_config()` の `parser.parse_args(argv)` 直後に行う**: `if args.gpus is None and args.gpu is None: args.gpu = 0`。これにより `parse_args_with_config()` 単体の戻り値でも未指定時 `args.gpu == 0` となり、既存テスト（`tests/test_feat029_config_yaml.py` の `args.gpu == 0` 検証）が無修正で通る。`--gpus` 指定時は `args.gpu is None` を維持する。help 文言は上記のとおり更新する。

**新規関数 `_gpus_list`**（argparse type / YAML コンバータ兼用）:

```python
def _gpus_list(value: str) -> list[int]:
    """カンマ区切り非負整数リストをパースする。空要素・非整数・負数は
    argparse.ArgumentTypeError を送出する。"""
```

処理: `value.split(",")` の各要素を strip し、(a) 空文字列 → エラー「空の要素があります: {value}」、(b) `int()` 変換失敗 → エラー「整数ではありません: {要素}」、(c) 負数 → エラー「非負整数である必要があります: {要素}」。全要素成功で `list[int]` を返す（順序保持・重複可）。

**YAML**: `CONFIG_CONVERTERS` に `"gpus": _gpus_list` を追加する。YAML 値は文字列（例: `gpus: 0,0`）で CLI と同形式。既存の CLI > YAML > デフォルトの優先規則・未知キー検証は変更しない。

### §4.2 検証と正規化（FR-001。main 内、既存の検証群と同じ位置関係）

main の処理順（既存の順序に以下を挿入。番号は挿入順）:

1. **排他検証（parser.error、終了コード2）**: 既存の排他制約ブロック（`--still-range`/`--dump-poses` の検証）の直後に追加:
   - `args.gpus is not None and args.gpu is not None` → `parser.error("--gpu と --gpus は併用できません（設定YAMLの gpu / gpus キー経由の指定を含む）")`
   - `args.gpus is not None and exclusive_mode` → `parser.error("--gpus は --still-range / --dump-poses と併用できません")`
2. **正規化**: §4.1 のとおり `parse_args_with_config()` 内で実施済み（main では行わない）。以降、直列モード・still モードの既存コード（`torch.cuda.set_device(args.gpu)` 等）は int の `args.gpu` を前提にでき、変更不要。並列モードでは `args.gpu` は `None` のままで参照しない
3. **GPU範囲検証（終了コード1）**: 既存の `if mp4_mode or still_mode:` ブロック内の device_count 検証を分岐:
   ```python
   n_gpu = torch.cuda.device_count()
   if args.gpus is not None:
       bad = [g for g in args.gpus if g >= n_gpu]
       if bad:
           print(f"エラー: 指定GPU ID {', '.join(map(str, bad))} が範囲外です"
                 f"（利用可能: 0〜{n_gpu - 1}）", file=sys.stderr)
           return 1
   elif args.gpu >= n_gpu:
       （既存のエラー処理のまま）
   ```

### §4.3 並列モードのアーキテクチャ（FR-002）

`_run_mp4_mode` を次のとおり再構成する。**既存の直列経路の処理内容・出力・ログは一切変えない**（コードの移動のみ許す）:

```
（既存のまま）chunks 計画 → チャンクヘッダ表示 → manifest 検証/作成 →
             完成済みチャンクの ffprobe 検査 → skip_indices 確定
（移動）     config 辞書の構築を torch import より前（分岐の外）へ移動
             ※config は torch に依存しない（width/height/fx/fy/cx/cy/fps/crf/
               preset/chunk_dir/black_frame のみ）ため挙動不変
（分岐）     if args.gpus is None:
                 import torch; torch.cuda.set_device(args.gpu)   # 既存位置から分岐内へ
                 （既存の直列チャンクループをそのまま）
             else:
                 rc, total_rendered, total_render_sec = _run_parallel_chunks(
                     args, chunks, skip_indices, viewmats, valid, config, chunk_dir)
                 if rc != 0:
                     return rc
（既存のまま）連結 → keep_chunks 処理 → 総所要/平均サマリ → return 0
```

データフロー（並列モード）:

- 入力: `chunks: list[dict]`（plan_chunks の戻り値）、`viewmats: np.ndarray (F,4,4) float32`、`valid: np.ndarray (F,) bool`、`config: dict`（上記キー）、`args.ply_path: str`、`args.gpus: list[int]`
- ワーカーへの受け渡し: spawn の `Process(args=...)` による pickle。サイズ見積り: フルNPZ（F=194,240）で viewmats 約12.4MB + valid 0.2MB + chunks（20要素の小辞書）。ワーカー数7でも一時的な複製は約90MBで問題ない
- タスクキュー `task_q`: 未完成チャンクの `index`（int）を積み、末尾にワーカー数分の `None`（番兵）を積む
- 結果キュー `result_q`: ワーカー→親のメッセージ（§4.5 のプロトコル）
- 出力: 直列モードと同一（チャンクディレクトリ内の区間MP4群 → 既存 concat）

### §4.4 ワーカープロセス（FR-002）

```python
def _worker_main(worker_idx: int, gpu_id: int, chunks: list[dict],
                 viewmats: np.ndarray, valid: np.ndarray, config: dict,
                 ply_path: str, task_q, result_q) -> None:
```

処理ロジック（擬似コード。設計意図の伝達用であり、そのままコピーするものではない）:

```
signal.signal(signal.SIGTERM, _sigterm_to_interrupt)   # SIGTERM→KeyboardInterrupt 変換
import torch                       # 関数内 import（既存方針どおり）
torch.cuda.set_device(gpu_id)
chunk_by_index = {c["index"]: c for c in chunks}
gaussians = None
current_idx = None                 # 処理中チャンク（未取得時は None）
try:
    loop:
        current_idx = None
        idx = task_q.get()         # ブロッキング取得（SIGTERM はここでも KeyboardInterrupt になる）
        if idx is None: return     # 番兵 → 正常終了（exitcode 0）。これが唯一の exitcode 0 経路
        current_idx = idx
        chunk = chunk_by_index[idx]
        try:
            if valid[chunk["i0"]:chunk["i1"]].any() and gaussians is None:
                from render import load_ply
                print(f"[W{worker_idx}] PLYファイル読み込み中: {ply_path}", flush=True)
                gaussians = load_ply(ply_path)
            n_rendered, sec = render_chunk(chunk, viewmats, valid, config, gaussians)
        except BaseException as e:
            tmp = os.path.join(config["chunk_dir"], chunk_filename(chunk) + ".tmp")
            存在すれば os.remove（OSError は握りつぶす）
            result_q.put(("error", worker_idx, idx, f"{type(e).__name__}: {e}"))
            sys.exit(1)            # 失敗ワーカーは非0終了（メッセージ+exitcode の二重検出）
        result_q.put(("done", worker_idx, idx, n_rendered, sec))
except KeyboardInterrupt:
    # SIGTERM（親の _abort または外部 kill）。沈黙の exitcode 0 終了は禁止:
    # 親が必ず検出できるよう、メッセージ送信を試みたうえで非0終了する
    if current_idx is not None:
        tmp = os.path.join(config["chunk_dir"], chunk_filename(chunk_by_index[current_idx]) + ".tmp")
        存在すれば os.remove（OSError は握りつぶす）
        result_q.put を試みる: ("error", worker_idx, current_idx, "SIGTERM により停止")
        （put の例外は握りつぶす。親が先に終了しているケース）
    sys.exit(1)                    # task 未取得時も非0終了 → 親の exitcode 監視で検出
```

**SIGTERM→KeyboardInterrupt 変換**（モジュールレベルに新設）:

```python
def _sigterm_to_interrupt(signum, frame):
    """ワーカー用: SIGTERM を KeyboardInterrupt に変換する（render_chunk の
    既存 KeyboardInterrupt 経路で ffmpeg を確実に terminate させるため）。"""
    raise KeyboardInterrupt
```

SIGTERM（親の `_abort` による terminate、または外部からの `kill -TERM`）の到達タイミング別の挙動（すべて親が検出可能。**exitcode 0 になるのは番兵受領の正常終了のみ**）:

| 到達タイミング | 経路 | ffmpeg 停止 | `.tmp` 削除 | 親の検出手段 |
|---|---|---|---|---|
| `render_chunk()` 実行中 | render_chunk 既存の KeyboardInterrupt 経路（ffmpeg terminate → wait → re-raise、`render_fps_video.py:484-487`）→ 内側 `except BaseException` | ワーカー自身 | ワーカー自身 | "error" メッセージ + exitcode 1 |
| タスク取得後〜render_chunk 開始前 | 外側 `except KeyboardInterrupt`（`current_idx` 保持） | 未起動 | ワーカー自身（存在すれば） | "error" メッセージ + exitcode 1 |
| `task_q.get()` 待機中 | 外側 `except KeyboardInterrupt`（`current_idx is None`） | 未起動 | 対象なし | exitcode 1（メッセージなし → 親の exitcode 監視） |

- 取得済みタスクが消失しても、親は上記いずれかの検出で即時中止（FR-004）に入るため、分配ループが未完了タスクを待ち続けることはない
- 親起動の `_abort` 中に届く "error" メッセージ・非0 exitcode は、親が既に中止処理中のため単に無視される（`_abort` は terminate → join → kill エスカレーション → `.tmp` 掃除を完遂する）

- PLY はワーカーごとに遅延ロードする（直列モードと同じ「最初の有効フレームを含むチャンクで初回ロード」。全チャンク黒ならロードしない）。VRAM はワーカーあたり約630MiB（e1 実測）
- ワーカーの stdout 出力は PLY ロード行のみ。チャンク完了ログは親が result_q 受信時に出力する（行の混線防止）
- `torch.cuda.set_device(gpu_id)` により `render_chunk` / `render_frame` 内の `device="cuda"` は割当GPUを指す（`CUDA_VISIBLE_DEVICES` は使わない。ADR-4）

### §4.5 分配ループ（FR-002 / FR-004）

```python
def _run_parallel_chunks(args, chunks, skip_indices, viewmats, valid, config,
                         chunk_dir) -> tuple[int, int, float]:
    """並列モードの本体。(終了コード, 描画フレーム総数, 実効描画秒) を返す。
    実効描画秒＝ワーカー起動直前から全チャンク完了受信までの wall-clock 秒
    （ワーカーの CUDA 初期化・PLY ロードを含む。ワーカー別所要秒の単純加算は
    並列では実効スループットにならないため使わない）。
    終了コード 0 以外のとき呼び出し元は即 return する。"""
```

処理ロジック:

1. スキップ表示: `chunk["index"] in skip_indices` のチャンクに対し直列と同一の `チャンク NNNNN スキップ（完成済み）` を表示する
2. `pending = select_pending_chunks(chunks, skip_indices)`。**`len(pending) == 0` なら (0, 0, 0.0) を返す**（ワーカーを起動しない。境界条件）
3. `並列: {len(args.gpus)}ワーカー（GPU割当: {','.join(map(str, args.gpus))}）` を表示する
4. `ctx = multiprocessing.get_context("spawn")` で `task_q` / `result_q` を作り、pending の index 全件 + ワーカー数分の `None` を `task_q` に積む
5. `wall_t0 = time.time()` を記録し、ワーカーを `ctx.Process(target=_worker_main, args=(...), daemon=True)` で全数起動する
6. `rc, total_rendered, worker_sec_sum = _dispatch_loop(procs, result_q, len(pending), chunks, chunk_dir)` を呼ぶ（`worker_sec_sum` はワーカー別所要秒の合算。参考表示にのみ使う）
7. `render_wall_sec = time.time() - wall_t0` を算出する
8. `rc == 0` のとき `ワーカー描画時間合計: {worker_sec_sum:.1f}秒（実効 {render_wall_sec:.1f}秒, {len(args.gpus)}ワーカー）` を表示する
9. `(rc, total_rendered, render_wall_sec)` を返す。呼び出し元（`_run_mp4_mode` の共通末尾）はこれを `total_render_sec` として既存サマリ（`format_render_average`）に渡すため、**並列時の「描画フレーム平均」は実効秒/フレームを意味する**（直列時は従来どおりチャンク所要合算/フレーム。意味の違いは本設計で定義）

```python
def _dispatch_loop(procs, result_q, pending_count, chunks, chunk_dir
                   ) -> tuple[int, int, float]:
```

- `procs` は `.exitcode` / `.is_alive()` / `.terminate()` / `.join(timeout)` を持つオブジェクトのリスト（テストではフェイクを注入する。§7）
- メッセージプロトコル（result_q の要素）:
  - `("done", worker_idx: int, chunk_index: int, n_rendered: int, sec: float)`
  - `("error", worker_idx: int, chunk_index: int, message: str)`

処理ロジック（すべての分岐を明記）:

```
remaining = pending_count; total_rendered = 0; total_sec = 0.0
chunk_by_index = {c["index"]: c for c in chunks}
try:
    while remaining > 0:                          # 終了条件: 全 pending の done 受信
        try:
            msg = result_q.get(timeout=1.0)
        except queue.Empty:
            dead = [(i, p) for i, p in enumerate(procs)
                    if p.exitcode not in (None, 0)]
            if dead:
                i, p = dead[0]
                stderr へ「エラー: ワーカー W{i} が異常終了しました（exit code {p.exitcode}）」
                _abort(procs, chunk_dir); return (1, total_rendered, total_sec)
            continue
        if msg[0] == "done":
            _, w, idx, n_rendered, sec = msg
            c = chunk_by_index[idx]
            print(f"[W{w}] チャンク {idx:05d} (fid {c['fid0']}-{c['fid1']}) 完了: "
                  f"描画 {n_rendered}/{c['i1'] - c['i0']} フレーム, {sec:.1f}秒")
            total_rendered += n_rendered; total_sec += sec; remaining -= 1
        elif msg[0] == "error":
            _, w, idx, message = msg
            stderr へ「エラー: ワーカー W{w} でチャンク {idx:05d} が失敗しました: {message}」
            _abort(procs, chunk_dir); return (1, total_rendered, total_sec)
except KeyboardInterrupt:
    stderr へ「中断されました」
    _abort(procs, chunk_dir); return (130, total_rendered, total_sec)
# 正常終了: 全ワーカーの回収
for p in procs:
    p.join(timeout=30)
    if p.is_alive(): p.terminate(); p.join(timeout=5)
return (0, total_rendered, total_sec)
```

`_abort(procs, chunk_dir)`（`_dispatch_loop` 内からのみ呼ぶ内部処理。関数化するかインライン化するかは実装に委ねず**関数化する**）:

```python
def _abort(procs, chunk_dir) -> None:
    for p in procs:
        if p.is_alive(): p.terminate()      # SIGTERM（ワーカー側で KeyboardInterrupt に変換され
                                            # render_chunk が ffmpeg を terminate する。§4.4）
    for p in procs:
        p.join(timeout=10)
    for p in procs:                          # SIGTERM で終わらない場合のエスカレーション
        if p.is_alive():
            p.kill()                         # SIGKILL
            p.join(timeout=5)
    cleanup_tmp_files(chunk_dir)             # 全ワーカー停止後に掃除（書き込み競合なし）
```

- `.tmp` の削除は**全ワーカーの停止を確認した後**に行う（実行中プロセスとの競合防止）
- SIGKILL にエスカレーションした場合、ワーカーの ffmpeg 子プロセスは stdin パイプの EOF（親ワーカー消滅）で自然終了する。ffmpeg プロセスが残存しないことは手動テストで確認する（§テスト設計）

```python
def cleanup_tmp_files(chunk_dir: str) -> None:
    """chunk_dir 直下の *.tmp を削除する（OSError は個別に握りつぶす）。"""

def select_pending_chunks(chunks: list[dict], skip_indices: set[int]) -> list[dict]:
    """skip_indices に含まれない chunk を元の順序で返す。"""
```

エラーハンドリング一覧:

| エラー | 検出方法 | 処理 | 終了コード |
|---|---|---|---|
| `--gpus` 形式不正 | `_gpus_list` の ArgumentTypeError | argparse がエラー表示 | 2 |
| `--gpu`/`--gpus` 併用、排他モード併用 | main の排他検証 | parser.error | 2 |
| GPU ID 範囲外 | main の device_count 検証 | stderr 表示 | 1 |
| ワーカー内の例外（ffmpeg 異常・CUDA エラー含む） | result_q の "error" メッセージ | 即時中止（_abort）、stderr にワーカー番号・チャンク番号・原因 | 1 |
| ワーカーの異常死（メッセージなし。OOM kill 等） | ポーリング時の `exitcode not in (None, 0)` | 即時中止（_abort）、stderr に exit code | 1 |
| Ctrl-C | KeyboardInterrupt | 即時中止（_abort）、「中断されました」 | 130 |
| 連結失敗 | 既存 concat_chunks の RuntimeError | 既存処理のまま | 1 |

境界条件:

- pending 0件（全チャンク完成済み）: ワーカー非起動で (0,0,0.0)。共通末尾の連結へ（直列の全スキップ時と同じ結果）
- ワーカー数 > pending 数: 余剰ワーカーは番兵を即取得して正常終了。問題なし
- 全フレーム黒（valid すべて False）: 全ワーカーが PLY 未ロードのまま黒チャンクを処理。直列と同じ結果
- ワーカー数 1（`--gpus 0`）: 並列機構を1ワーカーで実行（直列経路には落とさない）。A100 での1GPU=1ワーカー動作確認と同型
- F=0: 既存の main 冒頭検証で拒否済み（本設計に流入しない）

### §4.6 再開機構との互換（FR-003）

- マニフェスト（`build_manifest`）に `gpus` は**追加しない**。したがって直列→並列、並列→直列、ワーカー構成変更をまたぐ再開が既存の照合のまま成立する（ADR-3）
- 完成済みチャンクの ffprobe 検査・破損チャンク削除・skip_indices 構築は既存コード（分岐より前）を無改変で共有する
- 中止時（FR-004）に `.tmp` を削除するため、再実行時のチャンクディレクトリには「完成済み区間MP4」と「未着手」のみが残る

### §4.7 render_chunk の KeyboardInterrupt 経路の修正（FR-004。2026-08-10 実機確認3の結果を受けた追加設計）

**実機で確認した不具合**: `--gpus 0,0` 実行中のワーカーへ外部から `kill -TERM` を送ると、`_sigterm_to_interrupt` により `render_chunk()` 内で KeyboardInterrupt が発生するが、既存の捕捉節（`proc.terminate(); proc.wait(); raise`、`render_fps_video.py:487-490`）は **ffmpeg の stdin を閉じずに** terminate/wait する。ffmpeg は open されたままの stdin パイプの読み取りでブロックしており、この状態では SIGTERM を処理せず終了しない（実機で220秒超のデッドロックを確認。ffmpeg 子プロセスを SIGKILL するとワーカーは1秒以内に正常に後続処理へ進んだため、原因はこの箇所に限定される）。結果、ワーカーの exitcode が確定せず、親の異常検知（§4.5）が機能しない。

**修正**: `render_chunk()` の KeyboardInterrupt 捕捉節を次のとおり変更する（この節のみ。正常系は無改変。requirements.md 制約条件の例外規定に対応）:

```python
except KeyboardInterrupt:
    if proc.stdin and not proc.stdin.closed:
        try:
            proc.stdin.close()          # EOF 送出。パイプ読み取りブロックを解く
        except OSError:
            pass
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:   # なお終了しない場合のエスカレーション
        proc.kill()
        proc.wait()
    raise
```

- stdin クローズで ffmpeg は EOF を受けて読み取りブロックから抜け、terminate（SIGTERM）を処理して終了する。万一10秒で終了しない場合は SIGKILL で確実に回収する
- 直列モードの Ctrl-C（SIGINT はプロセスグループ全体に届き ffmpeg 自身も SIGINT で終了する既存挙動）には影響しない。stdin クローズが先行しても ffmpeg の終了が早まるだけで、`raise` により従来どおり `中断されました`（終了コード130）に到達する
- 本修正は書きかけ `.tmp` の破棄経路であり、確定済み区間MP4の生成過程（正常系）には一切影響しない
- §4.4 の到達タイミング表の1行目（`render_chunk()` 実行中）の「ffmpeg 停止」は、本修正後の経路（stdin クローズ → terminate → タイムアウト付き wait → kill）を指す

## 1.5 状態遷移

該当なし（GUI・常駐状態を持たない。分配ループの終了条件・分岐は §4.5 に記載）。

## 1.6 ファイル・ディレクトリ設計

- 入出力ファイル・チャンクディレクトリ（`<output>.chunks/`）・区間MP4命名・マニフェスト（manifest.json）は feat-027 のまま変更なし
- 設定YAML: キー `gpus`（文字列、例 `gpus: 0,0`）を追加。デフォルトなし（未指定なら直列）。他キーは feat-029 のまま

## 1.7 インターフェース定義

| 関数 | シグネチャ | 責務 |
|---|---|---|
| `_gpus_list` | `(value: str) -> list[int]` | カンマ区切り非負整数のパース（argparse type / YAML コンバータ） |
| `select_pending_chunks` | `(chunks: list[dict], skip_indices: set[int]) -> list[dict]` | 未完成チャンクの抽出（順序保持） |
| `cleanup_tmp_files` | `(chunk_dir: str) -> None` | `*.tmp` の削除 |
| `_worker_main` | `(worker_idx: int, gpu_id: int, chunks: list[dict], viewmats: np.ndarray, valid: np.ndarray, config: dict, ply_path: str, task_q, result_q) -> None` | ワーカー本体（§4.4） |
| `_dispatch_loop` | `(procs, result_q, pending_count: int, chunks: list[dict], chunk_dir: str) -> tuple[int, int, float]` | 結果集約・失敗検知・中止（§4.5）。第3要素はワーカー別所要秒の合算 |
| `_abort` | `(procs, chunk_dir: str) -> None` | 全ワーカー停止（SIGTERM→SIGKILL エスカレーション）と `.tmp` 掃除 |
| `_sigterm_to_interrupt` | `(signum, frame) -> None` | ワーカー用 SIGTERM→KeyboardInterrupt 変換（§4.4） |
| `_run_parallel_chunks` | `(args, chunks, skip_indices, viewmats, valid, config, chunk_dir) -> tuple[int, int, float]` | 並列モードの組み立て（§4.5 手順1〜9）。第3要素は実効描画秒（wall-clock） |

呼び出し方向: `main → _run_mp4_mode → _run_parallel_chunks → _dispatch_loop → _abort`。`_worker_main` は子プロセスのエントリポイントで `render_chunk` のみを呼ぶ。

## 1.8 ログ・デバッグ設計

print ベース（既存方針を踏襲。ログレベル機構は導入しない）:

| タイミング | 出力先 | 形式 |
|---|---|---|
| 起動（並列時のみ追加） | stdout | `並列: {N}ワーカー（GPU割当: {id,id,...}）` |
| チャンクヘッダ | stdout | 直列: 既存のまま（`使用GPU: {gpu}` を含む）。並列: `使用GPU: {gpu}` の部分を `使用GPU: 並列（--gpus）` に置き換えた同形式の行 |
| スキップ | stdout | 既存と同一 `チャンク NNNNN スキップ（完成済み）` |
| PLYロード（ワーカー） | stdout | `[W{k}] PLYファイル読み込み中: {path}`（flush=True） |
| チャンク完了（親が出力） | stdout | `[W{k}] チャンク NNNNN (fid F-F) 完了: 描画 n/N フレーム, S.S秒` |
| ワーカー失敗 | stderr | `エラー: ワーカー W{k} でチャンク NNNNN が失敗しました: {原因}` |
| ワーカー異常死 | stderr | `エラー: ワーカー W{k} が異常終了しました（exit code {n}）` |
| 並列集計（並列時のみ、正常終了時） | stdout | `ワーカー描画時間合計: {S.S}秒（実効 {S.S}秒, {N}ワーカー）` |
| 完了サマリ | stdout | 既存と同一形式（総所要・描画フレーム平均）。並列時の「描画フレーム平均」は実効秒/フレーム（§4.5 手順9） |

## 2.4 設計判断の記録（ADR）

| ID | 採用 | 却下案と理由 |
|---|---|---|
| ADR-1 | ワーカー=multiprocessing（spawn）子プロセス + 動的タスクキュー | (a) 自スクリプトの subprocess CLI 再帰起動: 引数の再構築・二重パース・チャンク割当の受け渡しが複雑。(b) 静的均等分割: 黒フレームチャンク（GPU不使用で高速）と描画チャンクの所要が不均一で、遅いワーカーが律速になる。(c) fork: CUDA と両立しない |
| ADR-2 | `--gpus` 指定時のみ並列経路。未指定は既存直列経路を無改変で通す | 既定を並列化する案: ヒアリングで「既定は現行と同じ直列」と決定（2026-08-10）。既存の使い方・YAML・テストへの影響ゼロを優先 |
| ADR-3 | `gpus` はマニフェスト非対象 | マニフェストに含める案: ワーカー構成は出力に影響しない実行時パラメータであり、含めると「直列で始めた処理を並列で再開」ができなくなる |
| ADR-4 | ワーカー内 `torch.cuda.set_device(gpu_id)` | `CUDA_VISIBLE_DEVICES` を子プロセス環境に設定する案: spawn では環境変数設定のタイミング制御が不確実（親の CUDA 初期化状態に依存しない保証がしにくい）。set_device は既存直列コードと同じ API で、e1 でも同方式の2プロセス動作を実測済み |
| ADR-5 | 失敗時は即時中止（全ワーカー停止） | 他ワーカー完走案: ヒアリングで即時中止と決定（2026-08-10）。エラーの早期発覚を優先。完成済みチャンク保持+再開で継続性は担保 |
| ADR-6 | チャンク完了ログは親プロセスが result_q 受信時に出力 | ワーカーが直接 print する案: 複数プロセスの stdout 行が混線し得る。集計（total_rendered/total_sec）にも受信が必要なため親出力に一本化 |
| ADR-7 | ワーカーごとに PLY を個別ロード（共有しない） | 共有メモリ/親ロード後の受け渡し案: gaussians は GPU テンソル辞書でプロセス間共有が困難。VRAM 実測 630MiB/ワーカーで個別ロードのコストは許容範囲（e1） |
| ADR-8 | render_chunk の KeyboardInterrupt 節を最小修正（stdin クローズ + タイムアウト付き wait + kill。§4.7） | (a) render_chunk 完全無改変のままワーカーの SIGTERM ハンドラを `os._exit(1)` にする案: デッドロックは回避できるが、ワーカー自身の `.tmp` 削除とエラーメッセージ送信が失われ、§4.4 のグレースフル停止設計が崩れる。stdin 未クローズという根本原因も残る。(b) 放置: 実機確認3で FR-004 が実証不能（2026-08-10 に220秒超のデッドロックを実測） |

## テスト設計（tests/test_feat030_parallel_render.py 新設）

実行環境の前提: ルートの uv 環境（torch なし・ffmpeg あり）。既存 `tests/test_feat027_render_fps_video.py` と同じ流儀（`sys.path.insert` で `phase4/` を解決、純粋関数ユニット + CLI 検証 + フェイク注入）。**既存テストは無修正で全件通ること**（回帰確認、制約条件）。

| # | テスト | 検証内容 |
|---|---|---|
| 1 | `test_gpus_list_valid` | `"0"→[0]`、`"0,0"→[0,0]`、`"0,1,2"→[0,1,2]`、`" 0 , 1 "→[0,1]` |
| 2 | `test_gpus_list_invalid` | `""`・`"0,"`・`"a"`・`"-1"`・`"0,,1"` が ArgumentTypeError |
| 3 | `test_cli_gpu_and_gpus_exclusive_exits_2` | `--gpu 0 --gpus 0,0` で SystemExit(2) |
| 4 | `test_cli_gpus_with_still_range_exits_2` / `test_cli_gpus_with_dump_poses_exits_2` | 排他モード併用で SystemExit(2) |
| 5 | `test_cli_gpu_default_normalized` | `--gpu`/`--gpus` とも未指定→ main 内正規化で `args.gpu == 0` になる経路（dump-poses モードで確認） |
| 6 | `test_yaml_gpus_key` | `parse_config_yaml` が `gpus: 0,1` を `{"gpus": [0,1]}` に変換 |
| 7 | `test_yaml_gpu_plus_cli_gpus_exclusive_exits_2` | YAML `gpu: 0` + CLI `--gpus 0,0` で SystemExit(2) |
| 8 | `test_gpus_out_of_range_exits_1` | フェイク torch（`sys.modules["torch"]` に `cuda.device_count=lambda: 1` を注入）+ 実在する PLY/NPZ/TOML の tmp ファイルで `--gpus 0,5` → 終了コード1・stderr に範囲外メッセージ |
| 9 | `test_select_pending_chunks` | skip_indices 反映と順序保持 |
| 10 | `test_cleanup_tmp_files` | `*.tmp` のみ削除・他ファイル保持 |
| 11 | `test_dispatch_loop_all_done` | フェイク procs（exitcode 0）+ 事前に "done" を積んだ `queue.Queue` → rc 0、集計値、`[W{k}] チャンク ...` 形式の stdout |
| 12 | `test_dispatch_loop_error_aborts` | "error" メッセージ → rc 1、全フェイク proc の terminate 呼び出し記録、chunk_dir の `.tmp` が削除済み、完成済みMP4ファイルは残存 |
| 13 | `test_dispatch_loop_dead_worker_aborts` | メッセージなし + フェイク proc の exitcode=1 → rc 1、exit code 入りの stderr |
| 14 | `test_render_chunk_keyboardinterrupt_closes_stdin_then_kills` | §4.7 の後始末順序。フェイク torch（`sys.modules` 注入）+ `start_ffmpeg_chunk` をモンキーパッチしてフェイク proc を返し、`stdin.write` で KeyboardInterrupt を送出 → (a) stdin.close → terminate → wait(timeout=10) の呼び出し順、(b) wait が TimeoutExpired を送出するケースで kill → wait が呼ばれること、(c) KeyboardInterrupt が re-raise されること |

フェイク proc は `exitcode` 属性・`is_alive()`・`terminate()`・`join(timeout)` を持つ最小クラスをテスト内に定義する。`result_q` は `queue.Queue`（`get(timeout=)` 互換）を使う。テスト14のフェイク proc は `stdin`（`write`/`close`/`closed`）・`wait(timeout=)`・`kill` も持ち、呼び出し順を記録する。

GPU 実機での動作確認（実装ステップ内で実施し報告する。合否判定は手動テストのユーザーに委ねる。**2026-08-10 の実装1回目で確認1〔スループット比1.469で合格〕・確認2〔再開互換合格〕は実施済み。確認3は §4.7 のデッドロックにより中断→§4.7 反映後に確認3のみ再実施する**）:

1. e1 の `tiled_3000.npz`・同一パラメータで直列と `--gpus 0,0` を実行し、(a) 出力MP4のフレーム数一致、(b) 合計スループット比 ≥ 1.3（FR-002 受け入れ基準2）を確認する。**スループット比の測定方法は実験 e1 と同一**: チャンク完了ログの各所要秒から、ウォームアップ分（直列は先頭チャンク、並列は各ワーカーの初回チャンク）を除外して秒/フレームを算出し、`比 = Σ(並列各ワーカーの 1/秒フレーム) / (直列の 1/秒フレーム)` とする（e1 の R2 算出式と同じ。wall-clock 実効値は 3000 フレーム規模では起動オーバーヘッド約15秒が支配的になるため合否判定には使わず、参考記録とする）
2. 直列で1チャンク完成後に Ctrl-C → `--gpus 0,0` で再開し、スキップログと最終MP4生成を確認する（FR-003）
3. `--gpus 0,0` 実行中に片方のワーカープロセスへ `kill -TERM` を送り、(a) 即時中止（終了コード1・stderr のワーカー異常メッセージ）、(b) チャンクディレクトリに `.tmp` が残らない、(c) ffmpeg プロセスが残存しない（`pgrep -f ffmpeg` で確認）、(d) 再実行で完成済みチャンクがスキップされ完走する、を確認する（FR-004）

## 完了処理（ステップ8で Claude Code 本体が行う）

- `docs/BACKLOG.md` を Closed に、`docs/CHANGELOG.md` に完了記録
- ルート `README.md` の render_fps_video.py オプション表に `--gpus` を追記（YAML キー `gpus` も）
- `CLAUDE.md` のディレクトリ構成にある render_fps_video.py の説明に並列対応を追記（feat-030）
- `docs/TECH_STACK.md` は変更不要（新規依存なし）

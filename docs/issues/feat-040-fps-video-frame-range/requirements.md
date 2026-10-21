# feat-040 要求仕様書: render_fps_video.py のフレーム範囲指定（start_frame / end_frame）

作成日: 2026-10-21
基準: `docs/REQUIREMENTS_STANDARD.md`
入力資料: 2026-10-21 ヒアリング（結果は `README.md` の「ヒアリングで確定した要件」に記録）

## 1. プロジェクト概要

- **何を作るのか**: `phase4/render_fps_video.py` に `--start-frame` / `--end-frame`（設定YAMLでは `start_frame` / `end_frame`）を追加し、MP4 モードとポーズダンプモードで処理するフレームの範囲を指定できるようにする
- **なぜ作るのか**: 現状は MP4 モードで NPZ の全フレームを出力する。tokyo 20261006 の NPZ は 54,007 フレーム（約30分）あり、変換結果（feat-039）の確認や一部区間の動画化のために全フレームを描画するのは時間がかかる
- **誰が使うのか**: 本プロジェクトの研究者
- **どこで使うのか**: phase4 venv（Linux、torch/gsplat、CUDA GPU 必須。ポーズダンプモードは GPU 不要）。コマンド例に付す `TORCH_CUDA_ARCH_LIST="9.0+PTX"` はローカル機（gtune2）向けの値（`CLAUDE.md` の「スクリプト実行」参照）

## 2. 用語定義

機能設計書・コード内でも同じ用語を使う。

| 用語 | 定義 |
|---|---|
| frame_ids | NPZ の `frame_ids`（昇順の連番の整数。`npz_to_c3d.load_npz` が検証する） |
| 範囲指定 | `--start-frame` と `--end-frame` の少なくとも一方を指定すること（設定YAMLでの指定を含む） |
| 指定範囲 | 範囲指定の値が表す区間 [start, end]（両端を含む）。`--start-frame` 省略時の start は −∞、`--end-frame` 省略時の end は +∞ とみなす |
| 対象フレーム | NPZ のフレームのうち、frame_ids が指定範囲に入るもの。範囲指定が無いときは NPZ の全フレーム |
| 実範囲 | 対象フレームの frame_ids の最小値と最大値（`<最小>-<最大>`） |
| MP4 モード | `--still-range` と `--dump-poses` のどちらも指定しない動作（既存） |
| ポーズダンプモード | `--dump-poses` を指定した動作（既存） |
| 静止画モード | `--still-range` を指定した動作（既存） |

## 3. 機能要求一覧

### FR-001: `--start-frame` / `--end-frame` の追加

- **概要**: 処理するフレームの範囲を frame_ids で指定するオプションを追加する
- **入力**: `--start-frame <整数>`、`--end-frame <整数>`（どちらも省略可。片方だけの指定も可）
- **出力**: 対象フレームだけを処理した結果（FR-002）
- **詳細**:
  - 値は frame_ids と同じ番号（0 始まりの並び順ではない）。両端を含む
  - 値は整数（負数も受け付ける）。整数として解釈できない値は終了コード 2
  - 省略時の動作は従来と同じ（全フレームを処理する）
- **受け入れ基準**: frame_ids 1〜100 の NPZ で `--start-frame 10 --end-frame 20` を指定すると、frame_ids 10〜20 の 11 フレームが処理される
- **優先度**: Must

### FR-002: 対象フレームの決め方（重なる部分だけ）

- **概要**: frame_ids が指定範囲に入るフレームだけを対象フレームにする
- **詳細**:
  - 指定範囲が frame_ids の範囲からはみ出していても、エラーにせず重なる部分だけを対象にする（例: frame_ids 1〜54007 で `--start-frame 0 --end-frame 900` → 対象は 1〜900）
  - 対象フレームが1個も無い場合（重なりが無い、または start > end）は、指定値と NPZ の frame_ids の範囲を表示して終了コード 1
  - 範囲指定があるときは、対象フレームの数と実範囲を標準出力に表示する
- **受け入れ基準**: frame_ids 1〜100 の NPZ で、`--start-frame 0 --end-frame 10` は 1〜10 の 10 フレーム、`--start-frame 95` は 95〜100 の 6 フレーム、`--start-frame 200` は終了コード 1、`--start-frame 20 --end-frame 10` は終了コード 1
- **優先度**: Must

### FR-003: MP4 モードでの範囲指定

- **概要**: MP4 モードで、対象フレームだけを描画した MP4 を出力する
- **詳細**:
  - MP4 のフレーム数は対象フレームの数。先頭フレームは実範囲の最小の frame_id
  - チャンク分割・チャンクのファイル名・再開判定（manifest）・NaN／縮退フレームの黒画面・情報ファイル（`<MP4名>_info.txt`）は、対象フレームだけを入力とみなして従来と同じ規則で作る（例: 情報ファイルの総フレーム数は対象フレームの数、NaN黒フレーム番号は対象フレーム内の frame_id だけ）
  - 並列レンダリング（`--gpus`）と併用できる
- **受け入れ基準**: frame_ids 1〜100 の NPZ で `--start-frame 10 --end-frame 20` の MP4 は 11 フレーム。情報ファイルの総フレーム数が 11
- **優先度**: Must

### FR-004: MP4 の既定の出力ファイル名に実範囲を入れる

- **概要**: `--output` 省略時の MP4 パスに、範囲指定があれば実範囲を入れる
- **詳細**:
  - 範囲指定あり・`--output` 省略: `<NPZ と同じディレクトリ>/<NPZ の basename（拡張子なし）>_fps_<実範囲の最小>-<実範囲の最大>.mp4`（例: `m6_full_person1_0-54006_fps_1-900.mp4`）
  - 実範囲は指定値ではなく、はみ出しを除いた実際の範囲（例: `--start-frame 0 --end-frame 900`、frame_ids 1〜54007 → `_fps_1-900`）
  - 負の frame_id は `-` 付きの10進表記のまま入れる（例: `_fps_-5-3`）
  - 範囲指定なし・`--output` 省略: 従来どおり `<NPZ の basename>_fps.mp4`
  - `--output` を指定した場合は、範囲指定の有無に関係なく `--output` のパスを使う
  - 情報ファイル名（`<MP4名>_info.txt`）とチャンクディレクトリ名（`<MP4パス>.chunks`）は従来どおり MP4 パスから決まる
- **受け入れ基準**: frame_ids 1〜100 の NPZ `a.npz` で `--start-frame 10 --end-frame 20`（`--output` なし）の出力が `a_fps_10-20.mp4`。範囲指定なしでは `a_fps.mp4`
- **優先度**: Must

### FR-005: ポーズダンプモードでの範囲指定

- **概要**: ポーズダンプモードで、対象フレームだけのポーズを JSON に書き出す
- **詳細**: JSON の `num_frames` は対象フレームの数、`frames` は対象フレームだけ（frame_id の昇順）。`npz_path` は従来どおり
- **受け入れ基準**: frame_ids 1〜100 の NPZ で `--start-frame 10 --end-frame 20 --dump-poses p.json` の `num_frames` が 11、`frames[0].frame_id` が 10、`frames[-1].frame_id` が 20
- **優先度**: Must

### FR-006: 静止画モードとの併用禁止

- **概要**: 静止画モード（`--still-range`）と範囲指定は併用できない
- **詳細**: `--still-range` と `--start-frame` / `--end-frame` のいずれかを同時に指定すると、終了コード 2（`parser.error`。PLY・NPZ のロード前）
- **受け入れ基準**: `--still-range 1 5 --start-frame 1` が終了コード 2
- **優先度**: Must

### FR-007: 設定YAMLでの指定

- **概要**: 設定YAML（`--config`、feat-029）に `start_frame` / `end_frame` キーを追加する
- **詳細**:
  - 値は整数。整数として解釈できない値は `設定YAML キー 'start_frame': <メッセージ>` の形式で終了コード 2（既存の `parse_config_yaml` の処理）
  - 優先順位（CLI > 設定YAML > 既定値）は feat-029 と同一
- **受け入れ基準**: `start_frame: 10` / `end_frame: 20` が CLI の `--start-frame 10 --end-frame 20` と同じ結果になる。`start_frame: 1.5` は終了コード 2
- **優先度**: Must

### FR-008: 範囲指定なしの後方互換

- **概要**: 範囲指定が無いときの動作は feat-031 時点と同一とする
- **詳細**:
  - MP4・情報ファイル・ポーズダンプ JSON・静止画 PNG、標準出力、終了コード、既存オプションの挙動を変えない
  - **対象外（変わることを許容する）**: `--help` 表示（オプションが2個増える）、設定YAMLの未知キーエラーの有効キー一覧（`start_frame` / `end_frame` が増える）
- **受け入れ基準**: 既存テストが全件パスする（既存テストの期待値修正は行わない）
- **優先度**: Must

## 4. 非機能要求

- **パフォーマンス**: 描画・エンコードの処理時間は対象フレームの数に比例する（範囲外のフレームは描画しない）。NPZ の読み込みは従来どおり全フレームに対して行う。ポーズ計算（`compute_fps_poses`）は対象フレームだけに対して行う（ポーズはフレームごとに独立に計算されるため、絞り込んでも各フレームの結果は全フレームで計算した場合と同じ）
- **対応環境**: phase4 venv（既存と同じ）
- **信頼性**: 範囲を変えて同じ MP4 パス（`--output` で同じパスを指定）で再実行したときの扱いは、既存の再開機構（feat-027）のまま変えない
  - チャンクディレクトリ（`<MP4パス>.chunks`）と `manifest.json` が残っている場合（前回が中断した、または `--keep-chunks` で正常終了した）: manifest の `num_frames` / `fid_first` / `fid_last` が一致しないため終了コード 1 になり、`--overwrite` を求める。別範囲のチャンクを連結することはない
  - チャンクディレクトリが無い場合（前回が `--keep-chunks` なしで正常終了し、チャンクディレクトリが削除済み）: 照合する manifest が無いため、新しい範囲で最初から描画し、既存の MP4 と情報ファイルを上書きする。これは範囲指定なしで同じ出力先に再実行したときと同じ既存の挙動であり、本案件では上書き防止を追加しない
  - `--output` 省略時は範囲ごとに別の名前になる（FR-004）ため、範囲を変えて再実行しても前の MP4 を上書きしない

## 5. 制約条件

- 新規ライブラリは追加しない
- 変更対象は `phase4/render_fps_video.py` とテストのみ。`render_keypoints.py`（`render_fps_video` を import している）の動作は変えない
- 既存の関数（`plan_chunks` / `build_manifest` / `build_info_text` / `_run_dump_poses` / `_run_mp4_mode` / `_run_still_mode` / `compute_fps_poses`）のシグネチャ・動作は変えない

## 6. 優先順位

- **Must**: FR-001〜FR-008
- **Won't（本案件ではやらない）**:
  - 静止画モードでの範囲指定（既存の `--still-range` を使う）
  - 複数区間（飛び飛び）の指定
  - 0 始まりの並び順での指定
  - 範囲が違うチャンクの再利用（範囲を変えたら `--overwrite` で作り直す）

MVP = Must 全件。

## CLI 仕様（要求レベル）

```bash
# phase4/ ディレクトリ。frame_ids 1〜900 だけを MP4 にする（出力は NPZ と同じ場所の <NPZ名>_fps_1-900.mp4）
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_fps_video.py \
    /home/sakagawa/data/PLY/tokyo-20261006-01/point_cloud/iteration_100/point_cloud.ply \
    data/tokyo1/m6_full_person1_0-54006.npz \
    --toml data/Calib_FPSCamera.toml --camera FPSCamera --fps 30 \
    --start-frame 1 --end-frame 900

# 設定YAML
#   start_frame: 0
#   end_frame: 900
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_fps_video.py --config data/tokyo1/run_fps-p1.yaml
```

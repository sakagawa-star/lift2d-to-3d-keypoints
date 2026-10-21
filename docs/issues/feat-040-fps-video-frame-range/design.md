# feat-040 機能設計書: render_fps_video.py のフレーム範囲指定（start_frame / end_frame）

作成日: 2026-10-21
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-040-fps-video-frame-range/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 `--start-frame` / `--end-frame` の追加 | 4.1 |
| FR-002 対象フレームの決め方 | 4.4 |
| FR-003 MP4 モードでの範囲指定 | 4.4, 4.6 |
| FR-004 既定の出力ファイル名に実範囲 | 4.3, 4.5 |
| FR-005 ポーズダンプモードでの範囲指定 | 4.4, 4.6 |
| FR-006 静止画モードとの併用禁止 | 4.2 |
| FR-007 設定YAML | 4.1 |
| FR-008 範囲指定なしの後方互換 | 4.3〜4.6, 7(D-1, D-2) |

## 2. システム構成

### 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `phase4/render_fps_video.py` | `_build_parser` に2オプション追加、`CONFIG_CONVERTERS` に2キー追加、関数 `select_frame_range` / `range_output_name` の追加、`main` の変更、モジュール docstring への追記 |
| `tests/test_feat040_frame_range.py` | 新規テスト |

既存の関数（`plan_chunks` / `build_manifest` / `build_info_text` / `_run_dump_poses` / `_run_mp4_mode` / `_run_still_mode` / `compute_fps_poses`）は変更しない。既存テストは変更しない。`render_keypoints.py` は変更しない（`render_fps_video` から import している名前〔`parse_config_yaml` 等〕の動作は変わらない。`parse_config_yaml` の既定の変換表 `CONFIG_CONVERTERS` は `render_keypoints.py` からは `converters=` で自前の表を渡して使っているため影響しない）。

### 依存関係

既存と同じ。新しい import は無い。

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / torch / gsplat / ffmpeg。バージョンは `docs/TECH_STACK.md` のとおり）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 CLI と設定YAML（FR-001, FR-007）

`_build_parser()` の `--dump-poses` の定義の直後（`--config` の直前）に追加する:

```python
parser.add_argument("--start-frame", type=int, default=None,
                    help="処理する最初の frame_id（NPZ の frame_ids 基準、この番号を含む。"
                         "MP4 / --dump-poses 用。省略時: NPZ の最初。--still-range と併用不可）")
parser.add_argument("--end-frame", type=int, default=None,
                    help="処理する最後の frame_id（同、この番号を含む。省略時: NPZ の最後）")
```

`CONFIG_CONVERTERS` に `"start_frame": int, "end_frame": int` を `"still_range": _yaml_still_range,` の後に追加する。

- `int` は従来の他キーと同じく、変換失敗時に `ValueError` を送出し、既存の `parse_config_yaml` が `設定YAML キー 'start_frame': ...` として終了コード 2 にする
- argparse の `type=int` は CLI の不正値（例: `1.5`）で終了コード 2 にする

### 4.2 静止画モードとの併用検査（FR-006）

`main()` の既存の検査 `if args.still_range is not None and args.dump_poses is not None:` の直後に追加する:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
range_given = args.start_frame is not None or args.end_frame is not None
if args.still_range is not None and range_given:
    parser.error("--still-range と --start-frame / --end-frame は同時指定できません")
```

- 既存の排他モード検査（`--output` / `--overwrite` / `--keep-chunks` の禁止）は変更しない。ポーズダンプモードでは `--start-frame` / `--end-frame` を使える

### 4.3 出力パスの決定の変更（FR-004, FR-008）

既存の `output_path` の決定ブロック（`if mp4_mode:` で `output_path is None` なら `<NPZ名>_fps.mp4` を作り、拡張子を検査する部分）を次に置き換える:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
output_path = args.output
if mp4_mode:
    if output_path is None and not range_given:
        output_path = os.path.join(
            os.path.dirname(os.path.abspath(args.npz_path)),
            os.path.splitext(os.path.basename(args.npz_path))[0] + "_fps.mp4",
        )                                                # 従来と同じ
    if output_path is not None and os.path.splitext(output_path)[1].lower() != ".mp4":
        print(f"エラー: --output の拡張子が .mp4 ではありません: {output_path}",
              file=sys.stderr)
        return 1
```

- 範囲指定あり・`--output` 省略のときは、ここでは `output_path` を `None` のままにし、対象フレームが決まった後（4.5）に決める。その名前は必ず `.mp4` で終わるため、拡張子検査は不要
- 範囲指定なしのときは従来と同じ処理・同じタイミング（FR-008）

### 4.4 対象フレームの絞り込み（FR-002, FR-003, FR-005）

#### `select_frame_range(frame_ids, start_frame, end_frame) -> np.ndarray`

`format_frame_ranges` の直後（`build_info_text` の前）に追加する。

- 入力: `frame_ids` (F,) int64（昇順の連番）、`start_frame` int | None、`end_frame` int | None
- 出力: (F,) bool のマスク。`frame_ids` が指定範囲（両端含む）に入る要素が True
- 処理:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
mask = np.ones(len(frame_ids), dtype=bool)
if start_frame is not None:
    mask &= frame_ids >= start_frame
if end_frame is not None:
    mask &= frame_ids <= end_frame
return mask
```

- `start_frame > end_frame` のときは全要素 False になる
- 例外は送出しない（空マスクの扱いは main で行う）

#### main での絞り込み

既存の2行の表示（`print(f"NPZ読み込み: {args.npz_path}")` と `print(f"  フレーム数 F={F}, 関節数 J={J}, frame_ids ...")`）の直後、`compute_fps_poses` の呼び出しの前に追加する:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if range_given:
    mask = select_frame_range(frame_ids, args.start_frame, args.end_frame)
    n_sel = int(np.count_nonzero(mask))
    if n_sel == 0:
        print(f"エラー: 指定範囲 (start={args.start_frame}, end={args.end_frame}) に"
              f"該当するフレームがありません（NPZ の frame_ids: "
              f"{int(frame_ids[0])}〜{int(frame_ids[-1])}）", file=sys.stderr)
        return 1
    F_all = F
    x3d_world = x3d_world[mask]
    frame_ids = frame_ids[mask]
    F = n_sel
    print(f"フレーム範囲: {F_all} フレーム中 {F} フレームを対象"
          f"（frame_ids {int(frame_ids[0])}〜{int(frame_ids[-1])}）")
```

- 絞り込み後の `x3d_world` / `frame_ids` / `F` を以降の処理（`compute_fps_poses`、有効フレームの集計表示、`nan_fids` / `degen_fids`、縮退警告、`_run_dump_poses` / `_run_mp4_mode`）にそのまま渡す。以降の既存コードは変更しない
  - 結果: 有効フレームの表示（`有効フレーム: {n_valid} / {F}...`）・縮退警告・情報ファイル・manifest・チャンク・ダンプ JSON は、すべて対象フレームだけを入力とみなした値になる（FR-003, FR-005）
- `frame_ids` は昇順の連番のため、`frame_ids[mask]` も昇順の連番になる（`plan_chunks` / `_run_still_mode` の前提を満たす。静止画モードはここに到達しない〔4.2〕）
- 絞り込み位置を NPZ・TOML の読み込みと頭部マーカー検査の後にする理由: これらのエラーを範囲指定の有無に関係なく従来と同じ順序で出すため
- 範囲指定なし（`range_given` が False）のときはこのブロックを通らない（FR-008）

### 4.5 既定の出力ファイル名（FR-004）

#### `range_output_name(npz_path: str, fid_first: int, fid_last: int) -> str`

`select_frame_range` の直後に追加する。

- 戻り値: `os.path.join(os.path.dirname(os.path.abspath(npz_path)), f"{os.path.splitext(os.path.basename(npz_path))[0]}_fps_{fid_first}-{fid_last}.mp4")`
- 例: `("/d/a.npz", 1, 900)` → `/d/a_fps_1-900.mp4`、`("/d/a.npz", -5, 3)` → `/d/a_fps_-5-3.mp4`

#### main での決定

4.4 のブロックの直後（`compute_fps_poses` の前）に追加する:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if mp4_mode and output_path is None:
    output_path = range_output_name(args.npz_path, int(frame_ids[0]), int(frame_ids[-1]))
```

- この時点で `output_path is None` になるのは、MP4 モード・範囲指定あり・`--output` 省略のときだけ（4.3）
- 引数の `frame_ids` は絞り込み後のもの。したがって名前の範囲は実範囲（はみ出しを除いた範囲）になる

### 4.6 各モードへの影響のまとめ

| モード | 範囲指定なし | 範囲指定あり |
|---|---|---|
| MP4 | 従来どおり | 対象フレームだけで `_run_mp4_mode`。出力名は `--output` または `<NPZ名>_fps_<最小>-<最大>.mp4` |
| ポーズダンプ | 従来どおり | 対象フレームだけで `_run_dump_poses` |
| 静止画 | 従来どおり | 4.2 で終了コード 2 |

- 範囲を変えて同じ `--output` で再実行したときの扱いは既存の `_run_mp4_mode` のまま変えない（要求 4章「信頼性」）:
  - `<MP4パス>.chunks/manifest.json` が残っている場合: 既存の manifest 照合で `num_frames` / `fid_first` / `fid_last` が不一致となり、終了コード 1（`--overwrite` を求める既存メッセージ）。照合は描画の前に行われる
  - `<MP4パス>.chunks` が無い場合（前回が `--keep-chunks` なしで正常終了すると、`_run_mp4_mode` の末尾で `shutil.rmtree(chunk_dir)` により削除される）: 新しい範囲で最初から描画し、既存の MP4 と情報ファイルを上書きする（範囲指定なしの再実行と同じ既存の挙動）
  - `--output` 省略時は範囲ごとに別名になるため衝突しない

### 4.7 モジュール docstring

既存のモジュール docstring の「ポーズダンプモード（GPU不要）:」の例の後に、次の段落を追加する:

```
フレーム範囲指定（feat-040。MP4 / ポーズダンプモード）: --start-frame / --end-frame
（設定YAMLでは start_frame / end_frame）で NPZ の frame_ids の範囲（両端含む）を指定する。
はみ出した部分は無視し、重なりが無ければエラー。--output 省略時の MP4 名は
<NPZ名>_fps_<最小>-<最大>.mp4（実際に出力した範囲）。--still-range とは併用不可:
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python render_fps_video.py \\
      data/<PLY> data/<NPZ> --toml data/<TOML> --camera <カメラ名> --fps 30 \\
      --start-frame 1 --end-frame 900
```

### 4.8 エラーハンドリング一覧

| エラー | 検出 | 処理 | 終了コード |
|---|---|---|---|
| `--start-frame` / `--end-frame` が整数でない（CLI） | argparse `type=int` | argparse 標準 | 2 |
| `start_frame` / `end_frame` が整数でない（設定YAML） | `parse_config_yaml`（既存） | `設定YAML キー '...': ...` | 2 |
| `--still-range` と範囲指定の併用 | 4.2 | `parser.error` | 2 |
| 対象フレームが0個（重なりなし・start > end） | 4.4 | stderr 表示 | 1 |
| 範囲を変えて同じ `--output` で再実行（チャンクディレクトリと manifest が残っている場合） | `_run_mp4_mode`（既存） | 既存メッセージ | 1 |

### 4.9 境界条件

- **start == end**: 1 フレームの MP4（またはダンプ）
- **start だけ指定**: start 以降の全フレーム。出力名の最大は NPZ の最後の frame_id
- **end だけ指定**: NPZ の最初から end まで
- **指定範囲が NPZ 全体を含む**（例: frame_ids 1〜100 で 0〜1000）: 全フレームが対象だが、範囲指定ありなので `--output` 省略時の名前は `<NPZ名>_fps_1-100.mp4`（範囲指定なしの `<NPZ名>_fps.mp4` とは別名）
- **対象フレームがすべて NaN／縮退**: 従来の全フレーム NaN と同じく黒画面の MP4 になる

## 5. ログ・デバッグ設計

- print ベース（既存と同じ）
- 追加する標準出力は、範囲指定ありのときの `フレーム範囲: ...` の1行（4.4）のみ。追加する標準エラーは 4.4 のエラー1種
- 範囲指定なしのときの標準出力は変えない

## 6. テスト設計

`tests/test_feat040_frame_range.py` を新規作成する。ルート環境（`uv run pytest`）には torch が無いため、MP4 モードの main を通すテストは次のスタブで行う（GPU・ffmpeg 不要）:

- `sys.modules["torch"]` に、`cuda.device_count()` が 1 を返す偽モジュール（`types.SimpleNamespace`）を `monkeypatch.setitem` で入れる
- `render_fps_video.shutil.which` を常に `"/usr/bin/fake"` を返す関数に、`render_fps_video.ffmpeg_has_libx264` を常に True を返す関数に置き換える
- `render_fps_video._run_mp4_mode` を、受け取った `output_path` / `frame_ids` / `nan_fids` / `degen_fids` を記録して 0 を返す関数に置き換える

NPZ・TOML は既存 `tests/test_feat027_render_fps_video.py` の `_pose_row` / `_write_npz` / `_TOML_TEMPLATE` と同じ形式のものを新規テストファイル内に複製して作る（既存テストファイルは変更しない）。`pytest.importorskip("tomli")` は既存の dump テストと同じく付ける。

| # | テスト | 期待値 |
|---|---|---|
| T-01 | `select_frame_range`: frame_ids 1〜10 で (3, 5) / (None, 8) / (4, None) / (None, None) / (0, 100) / (20, 30) / (5, 3) | 3〜5 / 1〜8 / 4〜10 / 全部 / 全部 / なし / なし |
| T-02 | `range_output_name`: `("/d/a.npz", 1, 900)` / `("rel/b.npz", -5, 3)` | `/d/a_fps_1-900.mp4` / `<abspath(rel)>/b_fps_-5-3.mp4` |
| T-03 | パーサー: `--start-frame 10 --end-frame 20` | `args.start_frame == 10`、`args.end_frame == 20` |
| T-04 | パーサー: `--start-frame 1.5` | SystemExit(2) |
| T-05 | 設定YAML: `start_frame: 10` / `end_frame: 20` | 同上の値 |
| T-06 | 設定YAML: `start_frame: 1.5` | SystemExit(2)、stderr に `start_frame` |
| T-07 | 設定YAML の値を CLI で上書き（YAML `start_frame: 10`、CLI `--start-frame 5`） | `args.start_frame == 5` |
| T-08 | main: `--still-range 1 2 --still-dir d --start-frame 1` / `--end-frame 1` | SystemExit(2)、stderr に `--still-range` |
| T-09 | main ダンプ: frame_ids 1〜10 の NPZ、`--start-frame 3 --end-frame 5 --dump-poses p.json` | 戻り値 0、`num_frames == 3`、frame_id が 3, 4, 5 |
| T-10 | main ダンプ: 同じ NPZ で範囲なしのダンプと T-09 のダンプ | T-09 の各 frame の `valid` / `c2w` が、範囲なしダンプの同じ frame_id の値と一致 |
| T-11 | main ダンプ: `--start-frame 0 --end-frame 3`（はみ出し） | `num_frames == 3`、frame_id 1〜3。stdout に `フレーム範囲: 10 フレーム中 3 フレームを対象（frame_ids 1〜3）` |
| T-12 | main ダンプ: `--start-frame 20` / `--start-frame 5 --end-frame 3` | 戻り値 1、stderr に `該当するフレームがありません` と `1〜10`。p.json が作られない |
| T-13 | main ダンプ: 範囲指定なしの stdout | `フレーム範囲` を含まない |
| T-14 | main MP4（スタブ）: `--start-frame 3 --end-frame 5`、`--output` なし | `_run_mp4_mode` に渡る `output_path` が `<NPZと同じディレクトリ>/<NPZ名>_fps_3-5.mp4`、`frame_ids` が [3, 4, 5] |
| T-15 | main MP4（スタブ）: `--start-frame 0 --end-frame 3` | `output_path` が `..._fps_1-3.mp4` |
| T-16 | main MP4（スタブ）: 範囲指定なし、`--output` なし | `output_path` が `..._fps.mp4`、`frame_ids` が 1〜10（従来どおり） |
| T-17 | main MP4（スタブ）: `--start-frame 3 --output x.mp4` | `output_path` が `x.mp4` |
| T-18 | main MP4（スタブ）: `--start-frame 3 --output x.avi` | 戻り値 1、stderr に `拡張子が .mp4 ではありません` |
| T-19 | main MP4（スタブ）: frame_id 4 の Nose を NaN にした NPZ で `--start-frame 3 --end-frame 5` / `--start-frame 6` | 前者は `nan_fids == [4]`、後者は `nan_fids == []` |
| T-21 | 再実行（manifest 残存）: `<output>.chunks/manifest.json` を、frame_ids 1〜3 を対象にした実行と同じ内容（`build_manifest(args, 3, 1, 3, 1920, 1080)`。args は main と同じ ply/npz/toml/camera/fps/chunk_size/crf/preset）で事前に作る。main を `_run_mp4_mode` をスタブ化せず（torch・which・libx264 のスタブのみ）、`--start-frame 4 --end-frame 6 --output <output>` で実行 | 戻り値 1、stderr に `パラメータが変更されています` と `fid_first` を含む。`render_fps_video.render_chunk` を呼ばれたら AssertionError を送出する関数に置き換えておき、送出されないこと（`_run_mp4_mode` は manifest 照合で描画前に `return 1` する） |
| T-22 | 再実行（manifest 削除済み）の扱いは既存の挙動（範囲指定に依存しない `_run_mp4_mode` の処理）のため、本案件では新しいテストを追加しない。範囲指定時に `_run_mp4_mode` へ渡る `output_path` / `frame_ids` が正しいことは T-14〜T-17 で確認する | （テストなし） |
| T-20 | 回帰: 既存テスト全件 | `uv run pytest -v` 全件パス（変更前 612 passed / 1 skipped + 本案件追加分。feat-039 の未コミット分を含む） |

既存テストが失敗した場合は、実装を中断して報告する（CLAUDE.md「実装の実行方法」の想定外事象の扱い）。

テスト結果は `tests/results/feat-040_test_result.txt` に `-v` 出力を保存する。

### 実データでの動作確認（実装ステップ内で実施）

プロジェクトルートで実行する。入力: PLY `/home/sakagawa/data/PLY/tokyo-20261006-01/point_cloud/iteration_100/point_cloud.ply`、NPZ `phase4/data/tokyo1/m6_full_person1_0-54006.npz`（feat-039 で変換済み。frame_ids 1〜54007）、`--toml phase4/data/Calib_FPSCamera.toml --camera FPSCamera --fps 30`。出力はスクラッチディレクトリ配下に `--output` / `--dump-poses` で指定する（NPZ と同じディレクトリには書かない）。

1. **ポーズダンプ**: `--start-frame 0 --end-frame 900 --dump-poses <スクラッチ>/p.json` が終了コード 0、`num_frames == 900`、最初の frame_id 1、最後の frame_id 900
2. **MP4**: `TORCH_CUDA_ARCH_LIST="9.0+PTX"` を付け、`--start-frame 1 --end-frame 300 --output <スクラッチ>/r.mp4` が終了コード 0。`ffprobe` で `r.mp4` のフレーム数が 300、`r_info.txt` の総フレーム数が 300
3. ユーザー指定範囲（0〜900）・`--output` 省略・`--gpus 0,0` での確認は手動テスト（ステップ7）でユーザーが行う

## 7. 設計判断（ADR 簡易版）

- **D-1 絞り込みは NPZ 読み込み直後（ポーズ計算の前）に配列を切り出して行う**: 以降の既存関数（チャンク・manifest・情報ファイル・ダンプ）を変更せずに対象フレームだけの結果にできる。ポーズ計算はフレームごとに独立なので結果は変わらない。各モード関数に範囲を渡して中で絞る案は、変更箇所が増え後方互換の検証範囲が広がるため却下
- **D-2 範囲指定なしのときは既存コードの経路をそのまま通す**: 出力名の決定タイミング・表示を変えないため、範囲指定ありのときだけ出力名の決定を後ろにずらす
- **D-3 はみ出しは重なる部分だけ出力**: ユーザー選択（2026-10-21）。`render_keypoints.py` と同じ。`--still-range` の「はみ出しはエラー」とは異なるが、`--still-range` は変更しない
- **D-4 既定の出力名に実範囲を入れる**: ユーザー選択（2026-10-21）。指定値ではなく実範囲にするのは、ファイル名から実際に入っているフレームが分かるようにするため
- **D-5 静止画モードとは併用不可**: ユーザー選択（2026-10-21）。範囲の指定方法を `--still-range` と二重にしないため
- **D-6 引数名は `--start-frame` / `--end-frame`**: `render_keypoints.py` / `render.py` と同じ名前にし、設定YAMLのキー名も揃える

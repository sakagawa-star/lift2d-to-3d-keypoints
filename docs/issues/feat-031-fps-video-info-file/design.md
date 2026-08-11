# feat-031 機能設計書: render_fps_video.py レンダリング情報テキストの自動保存

- 作成日: 2026-08-11
- 準拠: `docs/DESIGN_STANDARD.md`
- 対応要求: `requirements.md`（feat-031）

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 自動保存 | §4.1（保存パス）、§4.3(書き出しタイミングとエラー処理) |
| FR-002 記載項目と形式 | §4.2（テキスト生成） |
| FR-003 再開実行時の値の意味 | §4.2（値のソース）、§4.3 |

## 1.2 システム構成

変更対象は `phase4/render_fps_video.py` の1ファイルのみ（テストは `tests/test_feat031_info_file.py` を新設）。

```
phase4/render_fps_video.py
  ├─ 変更: main（NaN/縮退の frame_id リストを算出して _run_mp4_mode へ渡す）,
  │   _run_mp4_mode（シグネチャ拡張 + 完了時の情報ファイル書き出し）
  ├─ 新規: format_frame_ranges, info_file_path, build_info_text, write_info_file
  └─ 既存（無改変で流用）: durable_replace, format_render_average
```

依存方向: `_run_mp4_mode → write_info_file → durable_replace`、`_run_mp4_mode → build_info_text → format_frame_ranges`。循環依存なし。

## 1.3 技術スタック

- Python 3.10 標準ライブラリのみ（新規外部依存なし。`docs/TECH_STACK.md` の更新不要）

## 1.4 各機能の詳細設計

### §4.1 保存パスと収集データ（FR-001）

**保存パス**（新規関数）:

```python
def info_file_path(output_path: str) -> str:
    """最終MP4パスから情報ファイルパスを導出する。
    例: /a/b/test_fps.mp4 → /a/b/test_fps_info.txt"""
    return os.path.splitext(output_path)[0] + "_info.txt"
```

**NaN/縮退 frame_id リストの算出**（main の変更。既存の集計行 `render_fps_video.py:1347-1352` の直後に追加。既存の `finite_mask` / `valid` / `n_nan` / `n_degen` の計算・ログ出力は無改変）:

```python
nan_fids = [int(f) for f in frame_ids[~finite_mask]]
degen_fids = [int(f) for f in frame_ids[finite_mask & ~valid]]
```

- `frame_ids` は昇順連番（`load_npz` が保証）のため、両リストは昇順で得られる
- 実行ログの「NaN欠損: {n_nan}, 縮退: {n_degen}」と同一のマスクから導出するため、件数は必ずログと一致する（集計の二重実装をしない。ADR-3）

**`_run_mp4_mode` のシグネチャ拡張**:

```python
def _run_mp4_mode(args, output_path, frame_ids, intrinsics, viewmats, valid,
                  nan_fids: list[int], degen_fids: list[int]) -> int:
```

main の呼び出し（`render_fps_video.py:1363`）に `nan_fids, degen_fids` を追加する。still / dump モードの呼び出しは変更しない（情報ファイル非対象）。

### §4.2 テキスト生成（FR-002 / FR-003）

**連続区間圧縮**（新規関数）:

```python
def format_frame_ranges(fids: list[int]) -> str:
    """昇順の frame_id リストを連続区間圧縮の文字列にする。
    連続する番号は「始点-終点」、単独の番号は単独表記、", " 区切り。
    空リストは「なし」を返す。
    例: [1200..1239, 35001, 47800..47989] → "1200-1239, 35001, 47800-47989"
        [5] → "5" / [] → "なし""""
```

処理: 空なら `"なし"`。先頭から走査し、`fids[i+1] == fids[i] + 1` が続く限り同一区間にまとめる。区間の始点==終点なら `f"{start}"`、それ以外は `f"{start}-{end}"`。`", ".join(...)` で連結。

**テキスト組み立て**（新規関数）:

```python
def build_info_text(video_name: str, elapsed_sec: float, render_avg: str,
                    total_frames: int, fps: float, nan_fids: list[int],
                    degen_fids: list[int], width: int, height: int) -> str:
    """情報ファイルの本文（10行、末尾改行あり、UTF-8想定）を生成する。"""
```

出力（この10行を正とする。f-string の書式まで固定）:

```
動画ファイル: {video_name}
総所要時間: {elapsed_sec:.1f}秒
描画フレーム平均: {render_avg}
総フレーム数: {total_frames}
フレームレート: {fps}
NaN黒フレーム数: {len(nan_fids)}
NaN黒フレーム番号: {format_frame_ranges(nan_fids)}
縮退黒フレーム数: {len(degen_fids)}
縮退黒フレーム番号: {format_frame_ranges(degen_fids)}
解像度: {width}x{height}
```

値のソース（すべて既存の実行ログと同一の値。FR-003 の再開時の意味もこれで確定）:

| 引数 | ソース |
|---|---|
| `video_name` | `os.path.basename(output_path)` |
| `elapsed_sec` | 既存の `elapsed`（`render_fps_video.py:1228`。この実行の wall-clock） |
| `render_avg` | 既存の `format_render_average(total_rendered, total_render_sec)` の戻り値（直列/並列/描画0フレームの意味は feat-030 までの定義のまま） |
| `total_frames` | `F`（NPZ のフレーム数） |
| `fps` | `args.fps`（float。`30.0` のように出力される） |
| `nan_fids` / `degen_fids` | §4.1 で算出したリスト |
| `width` / `height` | `intrinsics["width"]` / `intrinsics["height"]` |

**書き出し**（新規関数）:

```python
def write_info_file(path: str, text: str) -> None:
    """text を UTF-8 で path に耐久書き出しする（.tmp に書いて durable_replace）。
    失敗時は OSError を送出する（呼び出し元で処理）。"""
```

処理: `path + ".tmp"` に `open(..., "w", encoding="utf-8")` で書き、`f.flush()` 後に close、`durable_replace(tmp, path)`。既存ファイルは `durable_replace` の `os.replace` により上書きされる。

### §4.3 書き出しタイミングとエラー処理（FR-001）

`_run_mp4_mode` の共通末尾を次のとおり変更する（既存の出力行・順序は無改変。**追加**のみ）:

```
（既存のまま）concat_chunks 成功 → keep_chunks 処理 → elapsed 算出 →
             「完了: 総所要 ...」表示 → 「最終MP4: ...」表示
（追加）     info_path = info_file_path(output_path)
             text = build_info_text(os.path.basename(output_path), elapsed,
                                    format_render_average(total_rendered, total_render_sec),
                                    F, args.fps, nan_fids, degen_fids, width, height)
             try:
                 write_info_file(info_path, text)
             except OSError as e:
                 print(f"エラー: 情報ファイルの書き出しに失敗しました: {e}", file=sys.stderr)
                 return 1        # MP4 は確定済みのまま残る
             print(f"情報ファイル: {info_path}")
（既存のまま）return 0
```

エラーハンドリング一覧:

| エラー | 検出方法 | 処理 | 終了コード |
|---|---|---|---|
| 情報ファイル書き出し失敗（権限・ディスク等） | `write_info_file` の OSError | stderr にメッセージ。MP4 はそのまま残る | 1 |
| 連結失敗 | 既存処理 | 既存のまま（情報ファイルは書かれない。書き出しは連結成功後のため） | 1 |

境界条件:

- 描画0フレーム（全チャンク完成済みの再実行）: `render_avg` は既存の `N/A（描画0フレーム）`。情報ファイルは通常どおり保存（FR-003）
- NaN・縮退が0件: 件数 `0`、番号 `なし`
- 全フレーム黒: NaN/縮退の合計が F。番号は全区間の圧縮表記
- 並列モード（`--gpus`）: `total_render_sec` は feat-030 定義の実効描画秒 → `render_avg` は実効秒/フレーム（実行ログと同一）

## 1.5 状態遷移

該当なし。

## 1.6 ファイル・ディレクトリ設計

- 情報ファイル: `<MP4パスの拡張子除去>_info.txt`（MP4 と同じディレクトリ）。UTF-8・LF・10行+末尾改行。既存ファイルは上書き
- 一時ファイル: `<情報ファイルパス>.tmp`（durable_replace 用。成功時は残らない）

## 1.7 インターフェース定義

| 関数 | シグネチャ | 責務 |
|---|---|---|
| `info_file_path` | `(output_path: str) -> str` | 情報ファイルパスの導出 |
| `format_frame_ranges` | `(fids: list[int]) -> str` | 連続区間圧縮（空は `なし`） |
| `build_info_text` | `(video_name: str, elapsed_sec: float, render_avg: str, total_frames: int, fps: float, nan_fids: list[int], degen_fids: list[int], width: int, height: int) -> str` | 本文10行の生成 |
| `write_info_file` | `(path: str, text: str) -> None` | 耐久書き出し（OSError 送出） |

呼び出し方向: `main → _run_mp4_mode →（build_info_text / write_info_file）`。

## 1.8 ログ・デバッグ設計

- 追加ログは成功時の `情報ファイル: {path}`（stdout）と失敗時の `エラー: 情報ファイルの書き出しに失敗しました: {e}`（stderr）の2つのみ。既存ログは一切変更しない

## 2.4 設計判断の記録（ADR）

| ID | 採用 | 却下案と理由 |
|---|---|---|
| ADR-1 | オプションなしの常時保存 | `--info-out` 等のオプション化: ヒアリングで「自動生成・指定不要」と決定（2026-08-11）。テキスト1つの生成コストは無視できる |
| ADR-2 | 書き出し失敗は終了コード1（MP4 は保持） | 失敗を無視して0で終える案: 記録が残らなかったことに気づけない。MP4 生成そのものは巻き戻さない |
| ADR-3 | NaN/縮退の内訳は main の既存マスク（`finite_mask`/`valid`）から導出 | 情報ファイル用に再集計する案: 実行ログとの不一致リスク（二重実装）を作るだけ |
| ADR-4 | 書き出しは連結成功後・既存サマリ表示の後 | 連結前に書く案: 連結が失敗すると「存在するのに動画がない」情報ファイルが残る |

## テスト設計（tests/test_feat031_info_file.py 新設）

実行環境: ルートの uv 環境（torch 不要。純粋関数のみ）。既存テストは無修正で全件通ること。

| # | テスト | 検証内容 |
|---|---|---|
| 1 | `test_format_frame_ranges_empty` | `[]` → `なし` |
| 2 | `test_format_frame_ranges_single` | `[5]` → `5` |
| 3 | `test_format_frame_ranges_consecutive` | `[1,2,3]` → `1-3` |
| 4 | `test_format_frame_ranges_mixed` | `[1200..1239]+[35001]+[47800..47989]` → `1200-1239, 35001, 47800-47989` |
| 5 | `test_format_frame_ranges_adjacent_singles` | `[1,3,5]` → `1, 3, 5` |
| 6 | `test_info_file_path` | `/a/b/test_fps.mp4` → `/a/b/test_fps_info.txt` |
| 7 | `test_build_info_text_full` | NaN・縮退ありの固定入力で10行の完全一致（行順・書式・末尾改行） |
| 8 | `test_build_info_text_zero_black` | NaN・縮退0件で `0` / `なし`、描画0フレームで `N/A（描画0フレーム）` がそのまま入る |
| 9 | `test_write_info_file_content_and_overwrite` | tmp_path に書いた内容の一致・既存ファイルの上書き・`.tmp` が残らないこと |
| 10 | `test_write_info_file_uses_durable_replace` | `durable_replace` をモンキーパッチして呼ばれることを検証 |
| 11 | `test_mp4_mode_writes_info_on_success` | `_run_mp4_mode` を直接呼ぶ統合テスト（下記セットアップ）で、rc 0・情報ファイルが生成され、全チャンクスキップのため `描画フレーム平均: N/A（描画0フレーム）`、NaN/縮退の行が引数どおりであること（FR-001/FR-003） |
| 12 | `test_mp4_mode_no_info_on_concat_failure` | 同セットアップで `concat_chunks` を RuntimeError 送出にモンキーパッチ → rc 1・情報ファイルが**生成されない**こと（ADR-4） |
| 13 | `test_mp4_mode_rc1_on_info_write_failure` | 同セットアップで `write_info_file` を OSError 送出にモンキーパッチ → rc 1・stderr に「情報ファイルの書き出しに失敗しました」 |

テスト11〜13の `_run_mp4_mode` 統合セットアップ（torch・ffmpeg・GPU 不要で自動保存経路を通す）:

- 小さな入力を組み立てる: `F=10`・`chunk_size=5`（2チャンク）・`frame_ids = np.arange(10)`・`valid` 全 True・`viewmats = np.zeros((10,4,4), np.float32)`・`intrinsics = {"width":64, "height":64, "fx":1.0, "fy":1.0, "cx":32.0, "cy":32.0}`
- `args` は `argparse.Namespace` で必要属性を明示構築（`gpus=None, gpu=0, chunk_size=5, crf=18, preset="medium", fps=30.0, overwrite=False, keep_chunks=True, ply_path=<tmp内ダミーファイル>, npz_path=<同>, toml=<同>, camera="cam"`）
- 全チャンクを「完成済み」に見せる: `chunk_dir = <output_path>.chunks` を作成し、`write_manifest(chunk_dir, build_manifest(args, ...))` と各 `chunk_filename(chunk)` の空ファイルを事前配置し、`verify_chunk_mp4` を `(True, "")` を返すようモンキーパッチ（レンダリング経路を通さない）
- `sys.modules["torch"]` にフェイク（`cuda.set_device` を持つ）を注入（直列分岐の `import torch` / `set_device` 対策。既存テストと同じ手法）
- `concat_chunks` をテスト11・13では no-op（出力パスに空ファイル作成）にモンキーパッチ

GPU 実機での動作確認（実装ステップ内で実施し報告する）:

1. e1 の `tiled_3000.npz`（NaN 0件）で `--gpus 0,0` 実行 → `<出力名>_info.txt` が生成され、総所要・描画フレーム平均・総フレーム数 3000・フレームレート 30.0・NaN/縮退 0件 `なし`・解像度 1920x1080 が実行ログと一致すること
2. NaN を含む一時NPZ（`phase4/data/session001_f145749_world300.npz` をコピーし、配列インデックス 10〜14 と 100 の頭部7点を NaN にしたもの。作成スクリプトは実験用に都度書き捨てで可、置き場所は `docs/issues/feat-031-fps-video-info-file/tmp/`〔git 管理外〕）で直列実行 → `NaN黒フレーム数: 6`・`NaN黒フレーム番号: 145609-145613, 145699` を確認
3. 確認2の出力を `--overwrite` なしで再実行（全チャンク完成済み）→ 情報ファイルが上書きされ `描画フレーム平均: N/A（描画0フレーム）`、NaN 項目は確認2と同一であること

## 完了処理（ステップ8で Claude Code 本体が行う）

- `docs/BACKLOG.md` を Closed に、`docs/CHANGELOG.md` に完了記録
- ルート `README.md` の render_fps_video.py セクションに情報ファイルの説明を追記
- `CLAUDE.md` のディレクトリ構成にある render_fps_video.py の説明に追記（feat-031）
- `docs/TECH_STACK.md` は変更不要（新規依存なし）

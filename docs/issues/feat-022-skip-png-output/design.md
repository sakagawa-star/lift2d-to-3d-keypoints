# feat-022 機能設計書: render_keypoints.py --no-png オプション（MP4のみ出力）

## 1.1 対応要求マッピング

| 要求ID | 内容 | 設計セクション |
|---|---|---|
| FR-001 | --no-png によるPNG保存スキップ | 1.4.1, 1.4.3 |
| FR-002 | --no-png 単独指定のエラー化 | 1.4.2 |
| FR-003 | 進捗表示の文言調整 | 1.4.4 |

## 1.2 システム構成

プロダクションコードの変更対象は `phase4/render_keypoints.py` の1ファイルのみ。モジュール構成・依存関係の変更なし。テストは `tests/test_feat022_no_png.py` を新規追加する（テスト方針セクション参照）。

変更箇所は3関数中2関数:

| 関数 | 変更内容 |
|---|---|
| `_build_parser()` | `--no-png` 引数を追加 |
| `main()` | 引数バリデーション追加、フレームループ内のPNG保存を条件分岐化、進捗表示の文言分岐 |
| `start_ffmpeg()` ほか | 変更なし |

## 1.3 技術スタック

- Python 3.10 / phase4 uv 環境（`phase4/pyproject.toml`）
- 使用ライブラリ: 既存のまま（argparse 標準ライブラリのみ追加使用箇所あり）。新規ライブラリなし
- `docs/TECH_STACK.md` の更新: 不要（ライブラリ変更なし）

## 1.4 各機能の詳細設計

### 1.4.1 --no-png 引数の追加（FR-001）

`_build_parser()` の `--mp4-fps` 定義の直後（`return parser` の直前）に追加する:

```python
parser.add_argument("--no-png", action="store_true",
                    help="連番PNG保存をスキップしMP4のみ出力する（--mp4 と併用必須）")
```

（設計書のコード例は意図の伝達が目的であり、そのままコピーして使うものではない）

- 入力: CLIフラグ。型 bool、デフォルト False
- argparse により `args.no_png` として参照される（ハイフンはアンダースコアに変換される）
- `allow_abbrev=False` は現行のまま維持する

### 1.4.2 引数バリデーション（FR-002）

`main()` 冒頭、`args = _build_parser().parse_args(argv)` の直後（カメラ選択・C3D読み込みより前）に追加する:

```python
if args.no_png and not args.mp4:
    parser.error("--no-png は --mp4 と併用してください（両方省略時は連番PNGのみ出力）")
```

- **検出方法**: `args.no_png == True` かつ `args.mp4 == False`
- **リカバリ動作**: なし。`parser.error()` により usage とエラーメッセージを標準エラー出力へ表示し、終了コード2で即終了する
- **実装上の注意**: 現行の `main()` は `_build_parser().parse_args(argv)` とパーサを変数に保持していない。バリデーションで `parser.error()` を呼ぶため、`parser = _build_parser()` / `args = parser.parse_args(argv)` の2行に分ける
- **設計判断（ADR）**:
  - 採用案: `parser.error()` による終了コード2。argparse の引数不正と同じ経路・同じ表示形式（usage付き）に揃える
  - 却下案: `print(..., file=sys.stderr); return 1`（既存のカメラ名不正等と同じ経路）。引数の組み合わせ不正は argparse レイヤの誤りであり、usage が表示される `parser.error()` の方がユーザーに親切なため却下

### 1.4.3 フレームループ内のPNG保存スキップ（FR-001）

`main()` のフレームループ内、連番PNG保存ブロック（`png_path = ...` から `if not ok: ... break` まで）全体を `if not args.no_png:` で囲む。

処理ロジック（変更後のフレームループの構造。番号は1フレームあたりの手順）:

1. キーポイント抽出・座標変換・投影・可視判定（変更なし）
2. `draw_overlay` でオーバーレイ画像生成（変更なし。PNGスキップ時もMP4に同じ画を書くため必須）
3. `args.no_png` が False の場合のみ: `png_path` を組み立て `cv2.imwrite` で保存。失敗（`cv2.error` 送出 or False 返却）時は従来どおりエラー表示・`rc = 1`・`break`
4. `ffmpeg_proc` が非None の場合: RGB変換して stdin へ書き込み（変更なし）
5. 進捗表示（1.4.4 の分岐）

- ループ終了条件: 従来と同一（全フレーム処理完了、またはPNG保存失敗/ffmpeg書き込み失敗による break）
- `os.makedirs(output_dir, exist_ok=True)` は変更しない。`--no-png` 時も `output.mp4` の出力先として output_dir は必要
- 完了時のサマリ表示（`完了: {output_dir} ...`）は変更しない
- **残存PNGの扱い（FR-001 出力の非破壊方針に対応）**: 過去の実行で output_dir に残った `frame_*.png` は削除・上書き・検査のいずれも行わない。本機能は「この実行でPNGを新規生成しない」ことのみを保証する
  - **設計判断（ADR）**: 採用案=非破壊（何もしない）。却下案=`--no-png` 時に既存 `frame_*.png` を削除（ユーザーの過去成果物を暗黙に破壊するリスクがあるため却下）、既存PNG存在時にエラー（MP4のみ更新したい再実行が不便になるため却下）

### 1.4.4 進捗表示の文言分岐（FR-003）

フレームループ末尾の進捗表示を以下の分岐にする:

- `args.no_png == False`（従来）: `  [{i+1}/{n}] {png_path}` に、`ffmpeg_proc` 非None なら末尾 ` -> mp4` を付ける（変更前と同一の出力）
- `args.no_png == True`: `  [{i+1}/{n}] frame {fr['frame_no']} -> mp4`（FR-002 により `--no-png` 時は必ず `--mp4` ありなので ` -> mp4` は常に付く）

実装上の注意: `png_path` の変数定義は 1.4.3 で `if not args.no_png:` ブロック内に移動するため、スキップ時の進捗表示で `png_path` を参照してはならない（未定義変数となる）。分岐は `args.no_png` で行う。

### エラーハンドリング（機能全体）

| エラー | 検出方法 | 処理 |
|---|---|---|
| `--no-png` 単独指定 | `args.no_png and not args.mp4` | `parser.error()`、usage表示、終了コード2（1.4.2） |
| PNG保存失敗 | `cv2.imwrite` の False返却 / `cv2.error` | 従来どおり（`--no-png` 時はPNG保存自体が実行されないため発生しない） |
| ffmpeg 途中終了 | `stdin.write` の BrokenPipeError/OSError | 従来どおり（変更なし） |
| ffmpeg 異常終了コード | finally節の `returncode` 検査 | 従来どおり（変更なし） |

### 境界条件

- 対象フレームが1件の場合: 従来と同一の挙動（PNGスキップ時はMP4が1フレームの動画になる。ffmpeg/エンコーダの1フレームMP4生成は feat-017 時点の挙動を変更しない）
- `--no-png --mp4` で `--output-dir` が存在しない場合: 従来どおり `os.makedirs` で作成する
- `--no-png` なしの全ケース: 出力・表示とも変更前と完全に同一であること

## 1.5 状態遷移

該当なし（バッチ処理スクリプト。GUI・ステートフル処理なし）。

## 1.6 ファイル・ディレクトリ設計

- 変更ファイル: プロダクションコードは `phase4/render_keypoints.py` のみ。追加ファイルは `tests/test_feat022_no_png.py`（新規）と `tests/results/feat-022_test_result.txt`（テスト結果）
- 出力パス規約の変更: なし
  - 連番PNG: `<output_dir>/frame_<C3Dフレーム番号:06d>.png`（`--no-png` 時は生成しない）
  - MP4: `<output_dir>/output.mp4`（従来どおり）
  - `output_dir` デフォルト: `./data/keypoints_<カメラ名>/`（従来どおり）
- 設定ファイル: なし

## 1.7 インターフェース定義

- CLI（追加分のみ）:
  - `--no-png`: bool フラグ、デフォルト False。`--mp4` と併用必須（単独指定は終了コード2）
- 関数シグネチャの変更: なし（`_build_parser() -> argparse.ArgumentParser`、`main(argv=None) -> int` とも従来どおり）
- モジュール間呼び出し方向の変更: なし

## 1.8 ログ・デバッグ設計

print ベースの現行方式を踏襲する（loggingモジュールは導入しない）。

| 出力ポイント | 内容 |
|---|---|
| 引数バリデーション失敗時（stderr） | argparse usage + 「--no-png は --mp4 と併用してください（両方省略時は連番PNGのみ出力）」 |
| MP4出力開始時（stdout、既存） | `MP4出力: <path> (fps=<fps>)`（変更なし） |
| 毎フレーム進捗（stdout） | 1.4.4 のとおり。PNGスキップ時は `  [i/n] frame <番号> -> mp4` |
| 完了時（stdout、既存） | `完了: <output_dir> (<件数> フレーム, <秒>秒)` + `MP4保存: <path>`（変更なし） |

## テスト方針

`CLAUDE.md` のテスト規約に従い `tests/` に pytest を置き（ファイル名 `tests/test_feat022_no_png.py`）、結果を `tests/results/feat-022_test_result.txt` に保存する。

### 自動テスト（GPU・ffmpeg・実データ不要）

**パース・バリデーション系**:

1. `_build_parser()` が `--no-png` を受理し `args.no_png == True` になること
2. `--no-png` 単独指定で `main()` が SystemExit（code 2）となること（存在しないファイルパスを渡しても引数エラーが先に出ることで、重い処理より前の検証であることを確認する）
3. `--no-png` なしのパース結果で `no_png` のデフォルトが False であること

**フレームループ実体系（モックテスト。FR-001/FR-003 の実体検証）**:

`main()` を実行可能にするため、以下を `unittest.mock.patch` でスタブ化する:

- `render_keypoints.load_cameras_toml` → 最小カメラ辞書（例: 4x4画素、K/rvec/tvec はダミー値）を返す
- `render_keypoints.load_c3d_all_frames` → 既知マーカー1個以上・3フレーム分のダミー `frames_data`（`residual` 全て有効）と `point_rate=30.0` を返す
- `render.load_ply` / `render.print_ply_summary` → ダミー戻り値（`sys.modules["render"]` ごとモック化し torch 依存を遮断する）
- `render_keypoints.render_image` → 背景 `(H,W,3) uint8` と `depth_map`/`alpha_map`（全画素で遠方深度・α=0、キーポイントが隠れない値）を返す
- `render_keypoints.start_ffmpeg` → `stdin.write` を記録するモックプロセス（`returncode=0`、`stdin.closed=False`）と ダミーmp4パスを返す
- `render_keypoints.cv2.imwrite` → 呼び出し回数を記録し True を返す

検証項目:

4. `--no-png --mp4` で `main()` が 0 を返し、`cv2.imwrite` の呼び出しが **0回**、モックffmpeg `stdin.write` の呼び出しが **フレーム数と同数**（3回）であること（FR-001）
5. 同一のモック構成で `--mp4` のみ（`--no-png` なし）では `cv2.imwrite` がフレーム数と同数（3回）呼ばれること（対照テスト。スキップが `--no-png` に連動している証明）
6. `--no-png --mp4` 実行時の標準出力（capsys で捕捉）の進捗行に `.png` が含まれず、`frame` と `-> mp4` が含まれること（FR-003）
7. `--no-png --mp4` 実行時、`--output-dir` に事前に置いた既存 `frame_*.png` が実行後も残存し内容が変化しないこと（FR-001 受け入れ基準4）

### 手動テスト（フロー ステップ7）

実レンダリング（実PLY/TOML/C3D、GPU使用）で以下を確認する:

- 空ディレクトリへの `--no-png --mp4` 実行で `output.mp4` のみ生成、`frame_*.png` 0枚
- 同一範囲の `--mp4`（PNGあり）との処理時間比較でPNG保存分の短縮を確認
- `--no-png` なしの従来コマンドの出力が変更前と同一

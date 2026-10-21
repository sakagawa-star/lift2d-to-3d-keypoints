# feat-039 機能設計書: C3D → NPZ 変換スクリプト

作成日: 2026-10-21
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-039-c3d-to-npz/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 C3D を NPZ に変換 | 4.2, 4.3, 4.5 |
| FR-002 欠損点を NaN | 4.3 |
| FR-003 フレーム番号はそのまま | 4.2, 4.3 |
| FR-004 出力パス | 4.1, 4.5 |
| FR-005 書き出しの安全性 | 4.4 |
| FR-006 入力のエラー | 4.2, 4.6 |
| FR-007 実行時の表示 | 4.5, 5 |

## 2. システム構成

### 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `phase4/c3d_to_npz.py` | 新規作成 |
| `tests/test_feat039_c3d_to_npz.py` | 新規テスト |

既存ファイルは変更しない。

### 依存関係

`phase4/c3d_to_npz.py` は次をトップレベルで import する（phase4 ディレクトリで実行するため、同ディレクトリのモジュールを直接 import できる。既存の `render_keypoints.py` → `render_fps_video.py` と同じ方式）。

| import 元 | 名前 | 用途 |
|---|---|---|
| `render_keypoints` | `load_c3d_all_frames`, `c3d_to_calib` | C3D の読み込みと座標変換（要求 5章「既存と同じ結果」） |
| `render_fps_video` | `HEAD_JOINT_NAMES` | 頭部7点の警告（FR-007） |
| `npz_to_c3d` | `load_npz` | 読み戻し検証（FR-005。`render_fps_video.py` と同じ読み込み関数で検証する） |
| 標準ライブラリ | `argparse`, `os`, `sys`, `tempfile` | CLI・パス操作・一時ファイル（4.4） |

- `render_keypoints` はトップレベルで `cv2` / `tomli` / `render_fps_video` を import するが、いずれも phase4 venv にあり、torch / gsplat は関数内 import のため読み込まれない（GPU 不要）
- `c3d`（py-c3d）は `load_c3d_all_frames` の関数内で import される
- 循環依存は無い（既存モジュールは `c3d_to_npz` を import しない）

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / py-c3d。バージョンは `docs/TECH_STACK.md` のとおり）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 CLI（FR-004）

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="C3D（1ファイル=1人の3Dキーポイント）→ NPZ 変換（render_fps_video.py 入力用）",
        allow_abbrev=False,
    )
    parser.add_argument("c3d_path", help="入力 C3D パス")
    parser.add_argument("--output", default=None,
                        help="出力 NPZ パス（省略時: 入力と同じ basename で拡張子 .npz）")
    return parser
```

- `allow_abbrev=False` は既存スクリプト（`npz_to_c3d.py` / `render_keypoints.py`）と同じ
- オプションは `--output` だけ（`--config` は設けない）

### 4.2 読み込み（FR-001, FR-003, FR-006）

`read_c3d(c3d_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str], float]`

- 戻り値: `raw_mm` (F, J, 3) float64、`residual` (F, J) float64、`frame_ids` (F,) int64、`labels` (長さ J の list[str])、`point_rate` float
- 処理:
  1. `labels, frames_data, point_rate = load_c3d_all_frames(c3d_path)`
     - `FileNotFoundError` はそのまま送出する（main で捕捉）
     - `ValueError`（フレーム0件。`load_c3d_all_frames` が送出）はそのまま送出する
     - それ以外の例外（py-c3d が C3D として解釈できない場合に送出する例外。種類は py-c3d の実装に依存する）は `ValueError(f"C3Dとして読み込めません: {c3d_path}: {e}")` に変換して送出する
  2. `labels` の検査（順に）:
     - `len(labels) == 0` → `ValueError(f"C3Dにマーカーがありません: {c3d_path}")`
     - 空文字列のラベルがある → `ValueError(f"C3Dに空のマーカー名があります（{i}番目。0始まり）: {c3d_path}")`（最初に見つかった添字 i）
     - 重複がある → `ValueError(f"C3Dのマーカー名が重複しています: {重複名をC3D内の初出順に ', ' で連結}: {c3d_path}")`
  3. 途中で切れた C3D の検査（FR-006-7）: C3D をもう一度開いてヘッダだけを読み、宣言フレーム数を取得して読み込みフレーム数と比べる

     ```python
     # 意図伝達用の擬似コード（そのままコピーしない）
     import c3d  # 関数内 import（load_c3d_all_frames と同じ流儀）
     with open(c3d_path, "rb") as f:
         declared = int(c3d.Reader(f).frame_count)   # = last_frame - first_frame + 1
     if len(frames_data) != declared:
         raise ValueError(f"C3Dのフレームが途中で欠けています（宣言 {declared} フレーム、"
                          f"読み込み {len(frames_data)} フレーム）: {c3d_path}")
     ```

     - py-c3d の `read_frames()` は `range(first_frame, last_frame + 1)` を回し、途中で EOF に達すると警告を出して読めたフレームだけを返す（例外を送出しない）。`frame_count` は同じ `first_frame` / `last_frame` から計算されるため、正常な C3D では必ず一致する
     - 2回目の `c3d.Reader(f)` はヘッダとパラメータだけを読み、フレームデータは読まない
     - 読み込みフレーム数が宣言より多いことは `read_frames()` の実装上起きないが、`!=` で比べ、多い場合も同じエラーにする
  4. `raw_mm = np.stack([fr["data"] for fr in frames_data])`、`residual = np.stack([fr["residual"] for fr in frames_data])`、`frame_ids = np.array([fr["frame_no"] for fr in frames_data], dtype=np.int64)`
- `labels` は `load_c3d_all_frames` が strip 済みで返す
- `frame_ids` は py-c3d の `read_frames()` が `range(first_frame, last_frame + 1)` で返す番号そのもので、昇順の連番になる（FR-003）。本関数では連番の検査をしない（4.4 の読み戻し検証で `load_npz` が連番を検査する）

### 4.3 変換（FR-001, FR-002）

`convert_to_world(raw_mm: np.ndarray, residual: np.ndarray) -> tuple[np.ndarray, dict[str, int]]`

- 入力: `raw_mm` (F, J, 3) float64、`residual` (F, J) float64
- 出力: `x3d_world` (F, J, 3) float64、`counts`（欠損点の内訳。キー `"missing"`, `"residual"`, `"zero"`, `"nan"`, `"total"`。値は int）
- 処理:

```python
# 意図伝達用の擬似コード（そのままコピーしない）
by_residual = ~(residual >= 0)                          # (F,J)  (a) 負または NaN
by_nan = np.isnan(raw_mm).any(axis=-1)                  # (F,J)  (b)
by_zero = np.all(raw_mm == 0.0, axis=-1)                # (F,J)  (c) 厳密一致。NaN を含む点は False
missing = by_residual | by_nan | by_zero
x3d_world = c3d_to_calib(raw_mm)                        # (F,J,3) float64
x3d_world[missing] = np.nan                             # 3成分とも NaN
counts = {"missing": int(missing.sum()), "residual": int(by_residual.sum()),
          "zero": int(by_zero.sum()), "nan": int(by_nan.sum()), "total": int(missing.size)}
```

- 判定は `render_keypoints.extract_keypoints` の valid（`residual >= 0` かつ NaN なし かつ 全成分 0.0 でない）の否定と同じ。`extract_keypoints` は既知28マーカー・1フレーム単位の関数のため再利用せず、全マーカー・全フレーム一括で同じ判定を行う（D-3）
- (a) を `residual < 0` ではなく `~(residual >= 0)` で計算する理由: residual が NaN のとき `residual < 0` は False になるが、`extract_keypoints` は `residual >= 0` が False となり無効にする。判定を一致させるため NaN の residual も欠損点にする
- `+inf` / `-inf` を含む座標は欠損点にしない（`extract_keypoints` と同じ。py-c3d が inf を返すことは想定しない）

### 4.4 書き出し（FR-001, FR-005）

`write_npz(out_path: str, x3d_world: np.ndarray, frame_ids: np.ndarray, joint_names: list[str]) -> None`

```python
# 意図伝達用の擬似コード（そのままコピーしない）
names_arr = np.array(joint_names, dtype=str)          # numpy Unicode 文字列配列 (J,)
out_dir = os.path.dirname(os.path.abspath(out_path))
# 出力と同じディレクトリに一意な一時ファイルを排他的に作る（O_CREAT|O_EXCL。既存のファイル・リンクは開かない）
fd, tmp_path = tempfile.mkstemp(dir=out_dir, prefix=os.path.basename(out_path) + ".",
                                suffix=".tmp.npz")
try:
    with os.fdopen(fd, "wb") as fh:                   # mkstemp が作ったファイルだけに書く
        np.savez(fh, x3d_world=x3d_world, frame_ids=frame_ids, joint_names=names_arr)
    _verify_npz(tmp_path, x3d_world, frame_ids, joint_names)
    os.replace(tmp_path, out_path)                    # 確定も後始末の対象に含める
except Exception:
    if os.path.exists(tmp_path):
        os.remove(tmp_path)
    raise
```

- 一時ファイルは `tempfile.mkstemp`（標準ライブラリ。`import tempfile` を追加）で作る。`mkstemp` は新しい名前のファイルを排他的に作成するため、既存のファイルやシンボリックリンクを開いて書き換えることがない。名前は `<出力ファイル名>.<ランダム文字列>.tmp.npz`
- `np.savez` にはパスではなくファイルオブジェクト（`os.fdopen(fd, "wb")`）を渡す。パスを渡すと `np.savez` が拡張子を調整したり別名で開き直したりする余地があるため、`mkstemp` が作ったファイルだけに書くようにする
- 出力ディレクトリが存在しない場合は `mkstemp` が `FileNotFoundError` を送出し、一時ファイルは作られない（main の except で終了コード 1）
- except で削除するのは `mkstemp` が返した `tmp_path` だけ（既存の他のファイルは削除しない）

- `os.replace` が失敗した場合（例: 出力パスが既存のディレクトリで `IsADirectoryError`）も except で一時ファイルを削除して再送出する。`os.replace` は失敗時に置き換え先を変更しないため、既存のファイル・ディレクトリは残る（FR-005）。成功した場合は一時ファイルが出力パスに移っているため、except に入らない

- 圧縮しない（`np.savez`。既存 `filter_npz.py` と同じ）
- `_verify_npz(path, x3d_world, frame_ids, joint_names) -> None`:
  1. `x, f, n = load_npz(path)`（`npz_to_c3d.load_npz`。キー・形状・`frame_ids` の連番を検査し、違反は `ValueError`）
  2. 次のいずれかが成り立たなければ `RuntimeError("読み戻し検証失敗: <項目名> が一致しません")`:
     - `x.shape == x3d_world.shape`
     - `np.array_equal(np.isnan(x), np.isnan(x3d_world))`（項目名 `NaN の位置`）
     - `np.array_equal(x[~np.isnan(x)], x3d_world[~np.isnan(x3d_world)])`（項目名 `x3d_world`）
     - `np.array_equal(f, frame_ids)`（項目名 `frame_ids`）
     - `n == list(joint_names)`（項目名 `joint_names`）
- `load_npz` は `allow_pickle=True` で読むが、本スクリプトは `joint_names` を Unicode 文字列配列（object 型ではない）で書くため pickle は使われない

### 4.5 main（FR-001, FR-004, FR-007）

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    out_path = args.output if args.output else os.path.splitext(args.c3d_path)[0] + ".npz"

    # 出力パス検証（C3D 読み込み前）
    if os.path.splitext(out_path)[1].lower() != ".npz":
        print(f"エラー: 出力パスの拡張子が .npz ではありません: {out_path}", file=sys.stderr)
        return 1
    if os.path.abspath(out_path) == os.path.abspath(args.c3d_path):
        print(f"エラー: 出力パスが入力 C3D と同一です（上書き防止）: {out_path}", file=sys.stderr)
        return 1

    try:
        print(f"C3D読み込み: {args.c3d_path}")
        raw_mm, residual, frame_ids, labels, point_rate = read_c3d(args.c3d_path)
        F, J = raw_mm.shape[0], raw_mm.shape[1]
        print(f"  フレーム数 F={F}, マーカー数 J={J}, rate={point_rate} Hz")
        print(f"  frame_ids 範囲: {int(frame_ids[0])}〜{int(frame_ids[-1])}（C3D フレーム番号をそのまま使用）")

        x3d_world, counts = convert_to_world(raw_mm, residual)
        print(f"  欠損点（NaN にした点）: {counts['missing']}/{counts['total']} 点"
              f"（residual<0またはNaN: {counts['residual']}, 座標(0,0,0): {counts['zero']}, "
              f"座標NaN: {counts['nan']}。重複あり）")

        missing_head = [n for n in HEAD_JOINT_NAMES if n not in labels]
        if missing_head:
            print(f"警告: 頭部マーカーが C3D にありません: {', '.join(missing_head)}"
                  "（この NPZ は render_fps_video.py の入力に使えません）", file=sys.stderr)

        write_npz(out_path, x3d_world, frame_ids, labels)
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1

    print(f"完了: {out_path}（{F} フレーム, {J} マーカー）")
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

- 出力ディレクトリが存在しない場合、`np.savez` が `FileNotFoundError` を送出し、上記の except で終了コード 1 になる（ディレクトリは作らない）
- 拡張子の比較は大小無視（`.NPZ` も可）。省略時の出力パスは入力の拡張子（`.c3d` / `.C3D` 等）を `.npz` に置き換えたもの

### 4.6 エラーハンドリング一覧

| エラー | 検出 | メッセージ（stderr） | 終了コード |
|---|---|---|---|
| 出力の拡張子が `.npz` でない | 4.5 | `エラー: 出力パスの拡張子が .npz ではありません: ...` | 1 |
| 出力パスが入力と同一 | 4.5 | `エラー: 出力パスが入力 C3D と同一です（上書き防止）: ...` | 1 |
| 入力ファイルが無い | 4.2-1（`FileNotFoundError`） | `エラー: <例外メッセージ>` | 1 |
| C3D として読めない | 4.2-1 | `エラー: C3Dとして読み込めません: ...` | 1 |
| フレーム0件 | 4.2-1（既存 `ValueError`） | `エラー: C3Dにフレームがありません: ...` | 1 |
| マーカー0個 | 4.2-2 | `エラー: C3Dにマーカーがありません: ...` | 1 |
| 空のマーカー名 | 4.2-2 | `エラー: C3Dに空のマーカー名があります（i番目。0始まり）: ...` | 1 |
| マーカー名の重複 | 4.2-2 | `エラー: C3Dのマーカー名が重複しています: ...` | 1 |
| C3D が途中で切れている | 4.2-3 | `エラー: C3Dのフレームが途中で欠けています（宣言 N フレーム、読み込み M フレーム）: ...` | 1 |
| 書き込み失敗（ディレクトリ無し・権限・出力パスが既存ディレクトリ） | 4.4（`OSError`） | `エラー: <例外メッセージ>` | 1 |
| 読み戻し検証失敗 | 4.4（`ValueError` / `RuntimeError`） | `エラー: <例外メッセージ>` | 1 |
| 頭部7点が無い | 4.5 | `警告: 頭部マーカーが C3D にありません: ...`（変換は続行） | 0 |
| 引数の誤り（位置引数なし・未知オプション） | argparse | argparse 標準 | 2 |

### 4.7 境界条件

- **フレーム1件**: 変換する（`load_npz` の連番検査は F ≥ 2 のときだけ行われる）
- **全点が欠損点**: 変換する（`x3d_world` が全 NaN の NPZ になる）。`render_fps_video.py` では全フレームが黒画面になる
- **マーカー1個**: 変換する
- **開始フレームが 65535 を超える C3D**: py-c3d が返す番号をそのまま使う（`npz_to_c3d.py` の注記にある py-c3d ヘッダの 16bit 制約は書き込み側の問題。本スクリプトは py-c3d の読み取り結果を正とし、補正しない）

## 5. ログ・デバッグ設計

- print ベース（既存 `npz_to_c3d.py` と同じ。ログレベル機構は導入しない）
- 標準出力: 4.5 の「C3D読み込み」「フレーム数…」「frame_ids 範囲」「欠損点」「完了」の各行
- 標準エラー: エラー（`エラー: ...`）と頭部マーカーの警告（`警告: ...`）

## 6. テスト設計

`tests/test_feat039_c3d_to_npz.py` を新規作成する。`sys.path` に `phase4` を追加して `c3d_to_npz` を import する（既存 `tests/test_feat018_npz_to_c3d.py` と同じ方式）。テスト用の C3D は、`npz_to_c3d.write_c3d`（開始フレーム 1、NaN の点を residual −1・座標0で書く）または py-c3d の `c3d.Writer` を直接使って `tmp_path` に作る。

| # | テスト | 期待値 |
|---|---|---|
| T-01 | 往復: 有限値の world 座標 W (F=5, J=4) を `world_to_c3d_raw` → `write_c3d` で C3D にし、`main` で NPZ に変換 | 終了コード 0。`load_npz` で読める。`x3d_world` が W と `np.allclose`（atol=1e-6 m。C3D は float32 で保存されるため）、`joint_names` 一致、`frame_ids == [1..5]` |
| T-02 | 欠損 (a): W の一部を NaN にして `write_c3d`（residual −1 で書かれる） | その点が NaN、それ以外は有限 |
| T-03 | 欠損 (c): C3D raw 座標 (0,0,0)・residual 0 の点を含む C3D（`c3d.Writer` で作成） | その点が NaN |
| T-04 | 1成分だけ非ゼロ: raw (0, 0, 5) mm・residual 0 の点 | NaN にならず、world 座標 `(0.005, 0, 0)`（`c3d_to_calib` の式） |
| T-05 | `convert_to_world` 単体: (a) residual −1、(b) 座標に NaN、(c) (0,0,0)、residual NaN、有効点を混在 | 欠損点 4 種がすべて NaN、有効点は `c3d_to_calib` の値。`counts` の各値が一致 |
| T-06 | 開始フレーム 100 の C3D（`c3d.Writer.set_start_frame(100)`、F=3） | `frame_ids == [100, 101, 102]` |
| T-07 | 出力パス既定値: `a.c3d` のみ指定 | `a.npz` が作られる |
| T-08 | `--output` 指定 | 指定パスに作られる |
| T-09 | `--output b.txt` | 終了コード 1、ファイルが作られない |
| T-10 | `--output` が入力と同一パス | 終了コード 1 |
| T-11 | 入力ファイルが無い | 終了コード 1、stderr に `エラー:` |
| T-12 | C3D でないファイル（テキストを書いた `bad.c3d`） | 終了コード 1、stderr に `C3Dとして読み込めません` |
| T-13 | フレーム0件（`load_c3d_all_frames` を monkeypatch で `ValueError("C3Dにフレームがありません: ...")` を送出） | 終了コード 1 |
| T-14 | マーカー0個（`load_c3d_all_frames` を monkeypatch で labels `[]`） | 終了コード 1、stderr に `マーカーがありません` |
| T-15 | 空のマーカー名（monkeypatch で labels `["Head", ""]`） | 終了コード 1、stderr に `空のマーカー名` |
| T-16 | マーカー名の重複（monkeypatch で labels `["Head", "Neck", "Head"]`） | 終了コード 1、stderr に `重複` と `Head` |
| T-17 | 出力ディレクトリが存在しない | 終了コード 1、ディレクトリが作られない |
| T-18 | 読み戻し検証失敗（`c3d_to_npz.load_npz` を monkeypatch で `frame_ids` を変えて返す） | 終了コード 1、出力パスにファイルが無く、出力ディレクトリに `*.tmp.npz` が残らない |
| T-19 | 既存ファイルの上書き | 出力パスの既存ファイルが新しい内容になる |
| T-20 | 頭部マーカーの無い C3D（labels に `Head` なし） | 終了コード 0、stderr に `警告: 頭部マーカー` と `Head` |
| T-21 | 頭部7点がそろった C3D | stderr に `警告` を含まない |
| T-22 | 標準出力 | `frame_ids 範囲: 1〜5`、`欠損点（NaN にした点）:` を含む |
| T-23 | 出力 NPZ のキー | `np.load(...).files` が `x3d_world`, `frame_ids`, `joint_names` の3個だけ。`joint_names` の dtype が Unicode 文字列型（`dtype.kind == "U"`）、`frame_ids` が int64、`x3d_world` が float64 |
| T-25 | 途中で切れた C3D: `write_c3d` で F=10 の C3D を作り、ファイル末尾を切り詰めて読み込めるフレームを10未満にする（切り詰めたファイルを `load_c3d_all_frames` で読んだフレーム数が 1 以上 10 未満であることをテスト内で確認してから実行する）。出力パスには事前に別内容のファイルを置く | 終了コード 1、stderr に `途中で欠けています`、出力パスのファイルが事前の内容のまま、出力ディレクトリに `*.tmp.npz` が残らない |
| T-26 | 出力パスが既存のディレクトリ（`tmp_path / "result.npz"` をディレクトリとして作成） | 終了コード 1、`tmp_path` 直下に `*.tmp.npz` が残らない、`result.npz` がディレクトリのまま |
| T-27 | 既存の一時ファイル名を壊さない: 出力 `out.npz` に対し、`out.npz.tmp.npz` を別内容のファイルとして、また別ファイル `victim.npz` を指すシンボリックリンク `out.npz.x.tmp.npz` を事前に置いて正常変換する | 終了コード 0。`out.npz.tmp.npz` と `victim.npz` の内容が実行前と同一、シンボリックリンクもそのまま。実行後の `*.tmp.npz` は事前に置いた2個だけ |
| T-24 | 回帰: 既存テスト全件 | `uv run pytest -v` 全件パス（変更前 586 passed / 1 skipped + 本案件追加分） |

既存テストが失敗した場合は、実装を中断して報告する（CLAUDE.md「実装の実行方法」の想定外事象の扱い）。

テスト結果は `tests/results/feat-039_test_result.txt` に `-v` 出力を保存する。

### 実データでの動作確認（実装ステップ内で実施）

入力: `/home/sakagawa/git/pose2sim/Pose2Sim/20261006-tokyo/m6_full_person1/pose-3d/m6_full_person1_0-54006.c3d`。出力はスクラッチディレクトリ配下に `--output` で指定する（入力と同じディレクトリには書かない）。

1. **変換**: 終了コード 0、表示の F=54007・J=26・rate=30.0・`frame_ids 範囲: 1〜54007`、処理時間（60 秒以内。FR 4章）を記録する
2. **欠損点の一致**: 同じ C3D を `render_keypoints.load_c3d_all_frames` で読み、`render_keypoints.extract_keypoints` を全フレームに適用した既知マーカーの valid の否定と、出力 NPZ の同じマーカー・同じフレームの NaN の位置が一致すること（スクラッチのスクリプトで確認）
3. **座標の一致**: 2 で有効な点について、`c3d_to_calib(raw)` と出力 NPZ の `x3d_world` が完全一致（`np.array_equal`）すること
4. **render_fps_video.py に入力できること**: 出力 NPZ で `render_fps_video.py` のポーズダンプモード（`--dump-poses`。GPU 不要）を実行し、終了コード 0 であること。PLY は `/home/sakagawa/data/PLY/tokyo-20261006-01/point_cloud/iteration_100/point_cloud.ply`、`--toml /home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/Calib_FPSCamera.toml`、`--camera FPSCamera`、`--fps 30` を使う（2026-10-21 時点の `phase4/data/tokyo1/run_keypoints_tokyo1.yaml` の `ply_path` / `fps_toml` / `fps_camera` と同じ値）。ダンプ JSON の `num_frames` が 54007、`frames` の最初の `frame_id` が 1 であること

## 7. 設計判断（ADR 簡易版）

- **D-1 変換スクリプトとして新規作成する**: ユーザー選択（2026-10-21）。`render_fps_video.py` に C3D 直読みを足す案は却下
- **D-2 C3D の読み込み・座標変換は `render_keypoints` の関数を再利用する**: 同じ C3D を `render_keypoints.py` で描いたときと同じ座標・フレーム番号になることを保証するため。py-c3d を直接読む重複実装は却下
- **D-3 欠損判定は全マーカー・全フレーム一括で再実装する**: `extract_keypoints` は既知28マーカーに絞り1フレームずつ処理するため、全マーカーを書き出す本スクリプトには使えない。判定式は `extract_keypoints` と同じにし、実データ確認 2 で一致を検証する
- **D-4 frame_ids は C3D フレーム番号のまま**: ユーザー選択（2026-10-21）。Pose2Sim の C3D では元動画のフレーム番号と 1 ずれることを了承済み。開始番号の指定オプション・0 始まりへの付け直しは却下
- **D-5 全マーカーを書き出す**: ユーザー選択（2026-10-21）。`render_fps_video.py` は名前で頭部7点を引くため、余分なマーカーがあっても問題ない
- **D-6 頭部7点が無くてもエラーにしない（警告のみ）**: 本スクリプトは汎用の C3D → NPZ 変換であり、`render_fps_video.py` 以外の用途（`filter_npz.py`、`render_keypoints.py` の NPZ 入力）でも使えるようにするため
- **D-7 point rate は NPZ に保存しない**: 既存 NPZ の規約（必須3キー）に合わせる。`render_fps_video.py` は `--fps` でフレームレートを受け取る
- **D-8 読み戻し検証に `npz_to_c3d.load_npz` を使う**: `render_fps_video.py` が使う読み込み関数そのもので検証し、`render_fps_video.py` に入力できることを書き出し時点で保証する

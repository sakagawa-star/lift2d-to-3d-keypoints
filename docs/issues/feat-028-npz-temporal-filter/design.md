# feat-028 機能設計書: NPZキーポイントの時間方向平滑化スクリプト

## 1.1 対応要求マッピング

対応する要求仕様書: `docs/issues/feat-028-npz-temporal-filter/requirements.md`

| 要求ID | 設計セクション |
|---|---|
| FR-001 NPZ読み込みと規約検証 | 1.4.1 |
| FR-002 Butterworthゼロ位相フィルタ | 1.4.2 |
| FR-003 無効サンプルの取り扱い | 1.4.3 |
| FR-004 NPZ書き出し | 1.4.4 |
| FR-005 パラメータ検証 | 1.4.5 |
| FR-006 ログ出力 | 1.8 |

## 1.2 システム構成

- 新規ファイル: `phase4/filter_npz.py`（単一ファイル、クラスなし・関数のみ）
- 既存ファイルの変更: なし（scipy は feat-020 で phase4 に導入済み。`docs/TECH_STACK.md` の
  更新も不要）
- 依存方向: `filter_npz.py` → NumPy / SciPy のみ。
  既存スクリプト（`npz_to_c3d.py`, `filter_c3d.py`）への import 依存は持たない
  （プロジェクト慣例に従い必要な関数は本ファイル内に実装する。ADR-7）。

```
phase4/
├── filter_npz.py      # 新規: NPZ→NPZ 時間方向平滑化
├── filter_c3d.py      # 参考: lowpass_filtfilt / split_segments / filter_marker の流用元
└── npz_to_c3d.py      # 参考: load_npz の検証ロジックの流用元
```

パイプライン上の位置（feat-027 構想。hearing-notes 参照）:

```
巨大NPZ --(filter_npz.py)--> NPZ(平滑化済み) --> feat-027 FPS一括レンダリング
                                             --> （従来経路も可: npz_to_c3d.py → ...）
```

## 1.3 技術スタック

| 項目 | 内容 |
|---|---|
| 言語 | Python 3.10 |
| 実行環境 | phase4 uv 環境（`uv run --project phase4 python filter_npz.py ...`）。Blender・GPU不要 |
| NumPy | 配列処理・NPZ入出力（既存依存） |
| SciPy | `scipy.signal.butter` / `scipy.signal.filtfilt`（feat-020 で導入済み、`scipy>=1.11`）。選定理由: ゼロ位相Butterworthはモーションキャプチャ後処理の標準手法 |

新規ライブラリ追加なし。

## 1.4 各機能の詳細設計

### 1.4.1 NPZ読み込みと規約検証（FR-001）

**関数**: `load_npz_keypoints(npz_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray], np.dtype]`

- 入力: NPZファイルパス。
- 出力:
  - `x3d`: (F, J, 3) float64（計算用に float64 へ変換済み）、ワールド座標 [m]
  - `frame_ids`: 入力の配列そのまま（dtype 変換しない。書き出し用）
  - `joint_names_arr`: 入力の配列そのまま（dtype 変換しない。書き出し用。表示用の
    `list[str]` が必要な場合は呼び出し側で `[str(n) for n in joint_names_arr.reshape(-1)]`
    により派生させる）
  - `extras`: 必須3キー以外の全キーの `{キー名: 配列}` 辞書（配列は読み込んだまま無加工）
  - `x3d_dtype`: 入力 `x3d_world` の dtype（書き出し時の変換先）
- 処理手順:
  1. `np.load(npz_path, allow_pickle=True)` で開く（`npz_to_c3d.load_npz` と同一）。
  2. 必須キー `("x3d_world", "frame_ids", "joint_names")` の存在を検証。欠落キーが
     あれば `ValueError`（欠落キー名とパスをメッセージに含める）。
  3. `x3d_world` の検証: `ndim == 3` かつ `shape[2] == 3` かつ
     `np.issubdtype(dtype, np.floating)`。違反は `ValueError`。
     F = shape[0], J = shape[1]。F == 0 / J == 0 も `ValueError`。
  4. `frame_ids` の検証: (a) dtype が整数型（`np.issubdtype(frame_ids.dtype, np.integer)`）
     でなければ `ValueError`（文字列・object・浮動小数点配列で例外が素通りするのを防ぐ）。
     (b) `frame_ids.ndim == 1` かつ `frame_ids.shape == (F,)`（規約の形状。入力配列を
     無加工で書き戻すため、`(F,1)` のような規約外形状を通すと出力も規約外になる）。
     (c) F >= 2 のとき、`int64` へ変換した配列で `np.all(np.diff(ids64) == 1)`
     （昇順連番。`npz_to_c3d.load_npz` と同一基準）。違反は `ValueError`。
     連番判定は int64 変換値で行い、戻り値・書き出しには入力配列をそのまま使う。
  5. `joint_names` の検証: `joint_names_arr.ndim == 1` かつ
     `joint_names_arr.shape == (J,)`（規約の形状）。違反は `ValueError`。
     配列はそのまま戻り値とする（str 化は表示時のみ）。
  6. 必須3キー以外の全キーを `extras` に読み込んだまま格納する（値の検証はしない。
     ADR-3）。
- 実データ規模（F=194,279〜2,160,000）でも読み込みは一括で行う（非機能要求の範囲内）。

### 1.4.2 Butterworthゼロ位相フィルタ（FR-002）

**関数**: `lowpass_filtfilt(series: np.ndarray, cutoff: float, fps: float) -> np.ndarray`

feat-020 `filter_c3d.lowpass_filtfilt` と同一実装（この式を正本とする）:

```python
# 意図伝達用スニペット（そのままコピーする目的ではない）
from scipy.signal import butter, filtfilt
b, a = butter(N=2, Wn=cutoff / (fps / 2.0), btype="low")   # 2次
return filtfilt(b, a, series, axis=0)                       # 往復適用で実効4次・ゼロ位相
```

- 入力: `series` (N,) または (N, 3) float64（時間軸は axis=0）、
  `cutoff` [Hz]（0 < cutoff < fps/2 は呼び出し側で検証済み）、`fps` [Hz]。
- 出力: 同形状の平滑化済み配列。
- フィルタ次数は 2 で固定（`filtfilt` 往復で実効4次）。次数のCLIオプションは設けない。
- `filtfilt` のパディングは既定（`padlen = 3 * max(len(a), len(b)) = 9`）を使う。
  適用には時系列長 N >= `MIN_FILTER_SAMPLES = 10`（padlen + 1）が必要。

### 1.4.3 無効サンプルの取り扱い（FR-003）

feat-020 の `split_segments` / `filter_marker` / `filter_all_markers` と同一のロジック。
相違点は (a) residual が存在しないため有効判定が座標のみになること、
(b) 出力の無効位置が「入力値のまま」（= NaN のまま）になることの2点。

**関数**: `split_segments(valid: np.ndarray, max_gap: int) -> list[tuple[int, int]]`

feat-020 と同一（有効フラグ (F,) bool → セグメント `[(lo, hi), ...]`、両端含む・両端は
有効サンプル。隣接する有効サンプル間の無効数が max_gap を超える箇所で区切る。
先頭・末尾の無効区間はどのセグメントにも含まれない。有効0個なら空リスト）。

**関数**: `filter_joint(x3d_j: np.ndarray, cutoff: float, fps: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int]]]`

関節1本分 (F, 3) float64 を受け取り、`(平滑化後座標 (F,3), スキップしたセグメント)` を返す。

- 有効フラグ: `valid = np.isfinite(x3d_j).all(axis=1)`（(F,) bool）。
- 処理手順（feat-020 `filter_marker` と同一構造）:
  1. `segments = split_segments(valid, max_gap)` を得る。
  2. 出力配列を入力座標のコピーで初期化する（無効位置の NaN/Inf はこの時点で
     出力に引き継がれ、以後変更されない）。
  3. 各セグメント `(lo, hi)` について:
     a. セグメント長 `hi - lo + 1 < MIN_FILTER_SAMPLES` なら平滑化せずスキップリストに
        追加して次へ（値は入力のまま。2026-08-07 ユーザー確定事項）。
     b. 区間 `[lo, hi]` を切り出し、区間内の無効サンプル（長さ max_gap 以下が保証済み）を
        成分ごとに `np.interp`（有効サンプルの添字と値による線形補間）で埋める。
     c. 埋めた区間に `lowpass_filtfilt` を適用する。
     d. `[lo, hi]` かつ `valid` の位置**のみ**フィルタ結果で置き換える
        （無効位置・区間外は入力値のまま。補間値は出力に反映されない）。

**関数**: `filter_all_joints(x3d: np.ndarray, cutoff: float, fps: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int, int]]]`

(F, J, 3) 全体に対し、関節 j = 0..J-1 のループで `filter_joint` を呼ぶ。
戻り値は `(平滑化後 (F,J,3), スキップした (関節添字, lo, hi) のリスト)`。

### 1.4.4 NPZ書き出し（FR-004）

**関数**: `write_npz(out_path: str, x3d_filtered: np.ndarray, x3d_dtype: np.dtype, invalid_mask_in: np.ndarray, frame_ids: np.ndarray, joint_names_arr: np.ndarray, extras: dict[str, np.ndarray]) -> None`

- `invalid_mask_in` は**入力NPZ由来**の無効マスク `~np.isfinite(入力x3d_world)`（(F, J, 3)
  bool）。呼び出し側（main）が `load_npz_keypoints` の戻り値（float64 変換後だが非有限
  位置は変換で不変）から計算して渡す。
- 処理手順:
  1. `x3d_out = x3d_filtered.astype(x3d_dtype)`（入力 dtype に戻す。ADR-4）。
  2. `frame_ids`・`joint_names` は**入力から読み込んだ配列オブジェクトをそのまま**書く
     （str リストではなく元の配列。dtype・値を完全一致で維持する）。
  3. `extras` の全キーも読み込んだ配列をそのまま書く。
  4. 一時ファイルへ `np.savez(tmp_path, x3d_world=x3d_out, frame_ids=..., joint_names=...,
     **extras)` で書く（非圧縮。ADR-5）。tmp_path は `out_path + ".tmp.npz"`
     （`np.savez` は拡張子 `.npz` を強制付与するため、`.npz` で終わる一時パスにして
     付与による名前ずれを防ぐ）。
  5. 読み戻し検証 `_verify_npz`（下記）に成功した場合のみ `os.replace(tmp_path, out_path)`。
     例外時は tmp を削除して re-raise（feat-020 と同一パターン）。

**関数**: `_verify_npz(npz_path: str, expected_shape: tuple, expected_dtype: np.dtype, invalid_mask_in: np.ndarray, frame_ids: np.ndarray, joint_names_arr: np.ndarray, extras: dict[str, np.ndarray]) -> None`

読み戻して以下を検証し、不一致は `RuntimeError`。比較基準はすべて**入力NPZ由来の値**
（出力配列との自己比較はしない）:

1. キー集合 == `{"x3d_world", "frame_ids", "joint_names"} ∪ extras.keys()`。
2. 読み戻した `x3d_world` の形状 == `expected_shape`（入力の (F, J, 3)）、
   dtype == `expected_dtype`（入力 dtype）。
3. 読み戻した `x3d_world` の非有限マスク `~np.isfinite(...)` が `invalid_mask_in`
   （入力NPZ由来）と完全一致（無効サンプル位置の維持を、フィルタ処理・dtype 変換を
   含むエンドツーエンドで検証する。float64→float32 変換でのオーバーフローによる
   Inf 発生もここで検出される）。
4. 読み戻した `frame_ids` が入力の配列と `np.array_equal` で一致。
5. 読み戻した `joint_names` が入力の配列と `np.array_equal` で一致。
6. `extras` の各キーが入力の配列と `np.array_equal` で一致（object 配列も同関数で比較可能）。

### 1.4.5 パラメータ検証とエラー処理（FR-005）

`main()` で以下の順に検証し、違反時は日本語メッセージを stderr に出力して return 1
（feat-020 と同じ `try/except (ValueError, RuntimeError, FileNotFoundError, OSError)` 構造。
検証 1〜5 は NPZ 読み込み前に行う）。

| # | 条件 | メッセージ内容（要素） |
|---|---|---|
| 1 | 出力パス拡張子が `.npz` でない | 出力パスの拡張子が .npz ではない旨とパス |
| 2 | `os.path.realpath(出力) == os.path.realpath(入力)`、または出力パスが既存ファイルで `os.path.samefile(入力, 出力)` が真 | 出力パスが入力と同一（上書き防止）の旨とパス |
| 3 | `--fps <= 0` | fps は正の値である旨と指定値（`--fps` 未指定は argparse `required=True` のエラーに任せる） |
| 4 | `--cutoff <= 0` | カットオフ周波数は正の値である旨と指定値 |
| 5 | `--max-gap <= 0` | 最大補間ギャップ長は正の整数である旨と指定値 |
| 6 | `--cutoff >= fps / 2` | カットオフがNyquist周波数以上である旨と両方の値 |
| 7 | 入力NPZが開けない/読めない | 例外メッセージ（FileNotFoundError/OSError/ValueError/zipfile.BadZipFile 経由。BadZipFile は OSError を継承しないため except 節に `Exception` は使わず、`np.load` 呼び出しを try で包み `ValueError` に変換する） |
| 8 | 必須キー欠落 | 欠落キー名とパス（1.4.1 で送出） |
| 9 | `x3d_world` の形状・型不正（ndim≠3、第3軸≠3、非浮動小数点、F=0、J=0） | 実際の形状・dtype（1.4.1 で送出） |
| 10 | `frame_ids` の非整数型・形状が (F,) でない・非連番 | 実際の dtype／形状／非連番の旨（1.4.1 で送出） |
| 11 | `joint_names` の形状が (J,) でない | 実際の形状（1.4.1 で送出） |

検証 6 は fps・cutoff の確定後（NPZ読み込み前）に行える（fps は CLI のみで決まるため。
feat-020 と異なり入力ファイルとの rate 併存問題は存在しない）。

### 境界条件

- F == 0 / J == 0: エラー終了（検証9）。
- F == 1: 全セグメントが長さ1 < 10 でスキップ（警告表示）。エラーにはせず書き出す。
- 全サンプル無効の関節: セグメント0個、全フレーム入力値（NaN）のまま出力。
- 全関節・全フレーム無効（実データの黒区間のみを切り出したような入力）: 全関節が上記と
  なり、平滑化変位の統計は「有効サンプルなし」と表示（1.8）。エラーにしない。
- 追加キーなし: `extras` 空辞書。ログには「なし」と表示。
- `--cutoff` が Nyquist にごく近い値: 検証6は `>=` 比較。fps/2 未満なら許可。

## 1.5 状態遷移

該当なし（ワンショットのCLIバッチ処理）。

## 1.6 ファイル・ディレクトリ設計

- 入力: 任意パスのNPZファイル（位置引数 `npz_path`）。
- 出力: `--output` 指定パス。省略時は `<入力の拡張子除去>_filtered.npz`
  （例: `data/session001_world_22pt.npz` → `data/session001_world_22pt_filtered.npz`）。
- 出力ディレクトリ: `out_dir = os.path.dirname(out_path)` が空文字列でない場合のみ
  `os.makedirs(out_dir, exist_ok=True)`（feat-020 と同一）。
- 設定ファイルは使わない（CLIオプションのみ）。

### 設計判断の記録（ADR）

- **ADR-1: NPZ→NPZ の独立スクリプトとする（2026-08-07 ユーザー決定・案B）。**
  却下案: feat-027 レンダリングプログラムへの内蔵 → 平滑化前後の検証・使い回しが
  できず、feat-027 の責務も肥大するため却下。中間成果物（平滑化済みNPZ）が残ることで
  カットオフ調整時の再実行・検証がしやすい。
- **ADR-2: フィルタは feat-020 と同一（2次Butterworth + filtfilt、実効4次・ゼロ位相、
  デフォルトカットオフ 6.0 Hz、`--max-gap` 既定 10）。** feat-020 の ADR-2/ADR-5 の
  検討結果を踏襲する。同一データ系（リフトアップ由来キーポイント）に対する実績があり、
  C3D経由とNPZ直行の2経路で平滑化強度を揃える。
- **ADR-3: 追加キー（`pnp_ok`, `coord_system` を含む必須3キー以外のすべて）は検証せず
  無加工で出力にコピーする。** `pnp_ok` の意味はデータ作成者に確認中（2026-08-07 時点
  回答待ち）だが、無加工コピーなら意味の確定を待たずに実装でき、情報も失わない。
  却下案: (a) 追加キーを捨てる → feat-027 で使う可能性のある情報の喪失。
  (b) 意味を解釈して処理 → 意味未確定のため不可能。
- **ADR-4: `x3d_world` の出力 dtype は入力と同一にする（float32 入力なら float32）。**
  フィルタ計算は float64 で行う。却下案: float64 で出力 → ファイルサイズが倍増し
  （10時間データで約0.6GB→1.2GB）、下流（feat-027）が受け取る精度も向上しない
  （座標値のスケールは数m・float32 の相対精度 1e-7 で mm 未満）。
- **ADR-5: `np.savez`（非圧縮）で書き出す。** 却下案: `np.savez_compressed` →
  浮動小数点ノイズを含む座標は圧縮率が低く、巨大データでの圧縮時間に見合わない。
  入力と同じ非圧縮形式で速度を優先する。
- **ADR-6: `frame_ids` の昇順連番（隣接差分1）を必須とする。** `npz_to_c3d.load_npz` と
  同一基準。実データは連番であることを確認済み（欠損は NaN 埋めで表現される）。
  却下案: 非連番を許してギャップを欠損扱い → 欠損表現が2系統になり feat-027 との
  整合が複雑化するため、規約外としてエラーにする。
- **ADR-7: 既存モジュール（`filter_c3d.py`, `npz_to_c3d.py`）から import せず、同等関数を
  本ファイル内に実装する。** プロジェクト慣例（feat-020 design 1.2 と同一判断）。
  スクリプト間の暗黙結合を避け、各スクリプトを単独で読める状態を保つ。

## 1.7 インターフェース定義

```python
# 定数
MIN_FILTER_SAMPLES: int = 10       # filtfilt 既定 padlen(9) + 1
DEFAULT_CUTOFF: float = 6.0        # カットオフ周波数の既定値 [Hz]（feat-020 と同一）
DEFAULT_MAX_GAP: int = 10          # 最大補間ギャップ長の既定値 [フレーム]（feat-020 と同一）
REQUIRED_KEYS: tuple = ("x3d_world", "frame_ids", "joint_names")

def load_npz_keypoints(npz_path: str) -> tuple[
        np.ndarray,              # x3d (F,J,3) float64（計算用）
        np.ndarray,              # frame_ids（入力配列そのまま）
        np.ndarray,              # joint_names_arr（入力配列そのまま。表示用の str 化は呼び出し側）
        dict[str, np.ndarray],   # extras（追加キー、無加工）
        np.dtype]: ...           # x3d_world の入力 dtype
    # 規約検証（必須キー・形状・dtype・frame_ids整数型・連番・長さ・F/J>0）も行う（違反は ValueError）

def lowpass_filtfilt(series: np.ndarray, cutoff: float, fps: float) -> np.ndarray: ...

def split_segments(valid: np.ndarray, max_gap: int) -> list[tuple[int, int]]: ...

def filter_joint(x3d_j: np.ndarray, cutoff: float, fps: float,
                 max_gap: int) -> tuple[np.ndarray, list[tuple[int, int]]]: ...
    # x3d_j: (F,3) float64 → (filtered (F,3), スキップセグメント [(lo,hi),...])

def filter_all_joints(x3d: np.ndarray, cutoff: float, fps: float,
                      max_gap: int) -> tuple[np.ndarray, list[tuple[int, int, int]]]: ...
    # (F,J,3) → (filtered (F,J,3), スキップした (関節添字, lo, hi) リスト)

def write_npz(out_path: str, x3d_filtered: np.ndarray, x3d_dtype: np.dtype,
              invalid_mask_in: np.ndarray, frame_ids: np.ndarray,
              joint_names_arr: np.ndarray, extras: dict[str, np.ndarray]) -> None: ...
    # invalid_mask_in: 入力NPZ由来の無効マスク ~isfinite(入力x3d_world) (F,J,3) bool

def _verify_npz(npz_path: str, expected_shape: tuple, expected_dtype: np.dtype,
                invalid_mask_in: np.ndarray, frame_ids: np.ndarray,
                joint_names_arr: np.ndarray, extras: dict[str, np.ndarray]) -> None: ...
    # 比較基準はすべて入力NPZ由来の値（出力配列との自己比較はしない）

def main(argv=None) -> int: ...
```

### CLI

```
uv run --project phase4 python filter_npz.py <npz_path> --fps HZ [--output PATH] [--cutoff HZ] [--max-gap N]
```

| 引数 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `npz_path` | str（位置） | 必須 | 入力NPZパス（規約NPZ） |
| `--fps` | float | **必須**（`required=True`） | サンプリング周波数 [Hz]。NPZに記録がなく、誤った既定値はカットオフの実効強度を黙って変えるため既定値を持たない |
| `--output` | str | `<入力>_filtered.npz` | 出力NPZパス |
| `--cutoff` | float | 6.0 | ローパスのカットオフ周波数 [Hz] |
| `--max-gap` | int | 10 | 線形補間で埋める無効サンプル連続数の上限 [フレーム]。超えるギャップはセグメント分割 |

argparse は `allow_abbrev=False`（既存スクリプトと同一）。

- モジュール間呼び出し方向: `main → load_npz_keypoints / filter_all_joints → filter_joint →
  lowpass_filtfilt`、`main → write_npz → _verify_npz`。循環なし。
- scipy の import は関数内 import とする（プロジェクト慣例。ルート venv での pytest 収集時の
  ImportError 回避）。numpy はモジュールレベルで import する。

## 1.8 ログ・デバッグ設計（FR-006）

logging モジュールは使わず print で標準出力に日本語表示する（既存スクリプトと同一方針）。
エラーは stderr（`print(..., file=sys.stderr)`）＋ 終了コード1。

正常系の出力ポイント（順序どおり）:

```
NPZ読み込み: <入力パス>
  フレーム数 F=<F>, 関節数 J=<J>, dtype=<dtype>, 追加キー: <キー名, ...>   # なければ「なし」
フィルタ: Butterworth 2次 filtfilt（ゼロ位相）, カットオフ <cutoff> Hz, fps <fps> Hz, 最大補間ギャップ <max_gap> フレーム
  無効サンプル: <関節名>=<件数>, ...        # 1件以上の関節のみ。無ければ「なし」
警告: 関節 <関節名> のセグメント [<lo>..<hi>]（長さ <n> < 10）は平滑化をスキップしました   # 該当時のみ
  平滑化変位 [mm]: 平均=<mean>, 最大=<max>                 # 全有効サンプルの |filtered - original| ノルム × 1000
  フレーム間変位 [mm]: 平滑化前 平均=<mean>, 平滑化後 平均=<mean>   # ジッター低減の定量確認
完了: <出力パス>（<F> フレーム, <J> 関節）
```

- 座標は [m] だが、変位の表示は読みやすさのため mm（×1000）とし、単位をログに明記する。
- フレーム間変位: 関節ごとに、隣接フレーム双方が有効サンプルであるペアについて
  `norm(x[t+1] - x[t])` を取り、全関節・全ペアの平均を計算する。「平滑化前」は入力座標、
  「平滑化後」は出力座標で同一のペア集合に対して計算する。有効ペアが0個の場合は
  「有効サンプルなし」と表示する。
- 平滑化変位: 有効サンプルが0個の場合は「有効サンプルなし」と表示する。

## テスト設計

`tests/test_feat028_filter_npz.py` を新規作成し、実行結果を
`tests/results/feat-028_test_result.txt` に保存する。

注意: ルート venv には scipy がないため、`pytest.importorskip("scipy")` を各テスト
モジュール冒頭で使うか、`uv run --project phase4 --with pytest pytest` で実行する
（feat-020 と同一の注意点。実行方法は実装時に確定し、テスト結果ファイルに実行コマンドを
記録する）。

| テスト | 対応要求 | 内容 |
|---|---|---|
| test_constant_series_preserved | FR-002 | 一定値入力 → 出力一致（atol=1e-6） |
| test_lowfreq_preserved_highfreq_attenuated | FR-002 | 低周波保存（振幅誤差5%以内）・高周波減衰（10%未満）・ピーク位置ずれ≦1フレーム |
| test_all_valid_joint_filtered | FR-003 | 全サンプル有効の関節が全フレーム平滑化される |
| test_gap_interpolation_and_nan_kept | FR-003 | max_gap以下の中間ギャップ: 有効位置平滑化・NaN位置が出力でもNaN |
| test_long_gap_splits_segments | FR-003 | max_gap超のギャップ: セグメント独立フィルタ（一方の値を大きく変えても他方の結果が不変） |
| test_edge_nan_untouched | FR-003 | 先頭/末尾のNaN区間が入力のまま |
| test_short_segment_skipped | FR-003 | 長さ9のセグメント → スキップ・入力のまま・警告表示 |
| test_roundtrip_npz | FR-001/004 | 合成NPZを書く→filter→読み戻しでキー集合・形状・frame_ids・joint_names一致、**非有限マスクが入力と一致** |
| test_extras_passthrough | FR-004 | 追加キー（bool配列・str配列）が出力で完全一致 |
| test_dtype_preserved | FR-004 | float32入力 → 出力x3d_worldがfloat32 |
| test_output_same_as_input_rejected | FR-005 | 入出力同一パスで終了コード1 |
| test_output_symlink_to_input_rejected | FR-005 | symlink経由の同一実体で終了コード1 |
| test_cutoff_validation | FR-005 | cutoff<=0 / cutoff>=Nyquist で終了コード1 |
| test_fps_validation | FR-005 | fps<=0 で終了コード1 |
| test_missing_key_rejected | FR-005 | 必須キー欠落NPZで終了コード1 |
| test_nonconsecutive_frame_ids_rejected | FR-005 | frame_ids非連番NPZで終了コード1 |
| test_noninteger_frame_ids_rejected | FR-005 | frame_idsが非整数型（float/str）のNPZで終了コード1 |
| test_bad_extension_rejected | FR-005 | 出力拡張子が.npz以外で終了コード1 |

### 手動テスト（フロー ステップ7、requirements 末尾に対応）

- コマンド: `uv run --project phase4 python filter_npz.py data/session001_world_22pt.npz --fps 30`
  （phase4 ディレクトリで実行。fps はデータ提供元の回答が得られるまで 30 を暫定使用）
- 確認: 終了コード0、1.8 の全ログ項目、フレーム間変位（平滑化後 < 平滑化前）、
  出力 `data/session001_world_22pt_filtered.npz` の生成。

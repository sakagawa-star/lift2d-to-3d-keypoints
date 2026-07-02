# feat-020 機能設計書: C3Dキーポイントの時間方向平滑化スクリプト

## 1.1 対応要求マッピング

対応する要求仕様書: `docs/issues/feat-020-c3d-temporal-filter/requirements.md`

| 要求ID | 設計セクション |
|---|---|
| FR-001 C3D読み込み | 1.4.1 |
| FR-002 Butterworthゼロ位相フィルタ | 1.4.2 |
| FR-003 無効サンプルの取り扱い | 1.4.3 |
| FR-004 C3D書き出し | 1.4.4 |
| FR-005 パラメータ検証 | 1.4.5 |
| FR-006 ログ出力 | 1.8 |

## 1.2 システム構成

- 新規ファイル: `phase4/filter_c3d.py`（単一ファイル、クラスなし・関数のみ）
- 既存ファイルの変更: `phase4/pyproject.toml`（scipy 追加）、`docs/TECH_STACK.md`（scipy 追記）
- 依存方向: `filter_c3d.py` → NumPy / SciPy / py-c3d。
  既存スクリプト（`npz_to_c3d.py`, `render_keypoints.py`, `fps_camera_pose.py`）への
  import 依存は持たない（プロジェクト慣例に従い必要な関数は本ファイル内に実装する）。

```
phase4/
├── filter_c3d.py      # 新規: C3D→C3D 時間方向平滑化
├── npz_to_c3d.py      # 参考: write_c3d / _verify_c3d のコードパターン流用元
└── render_keypoints.py # 参考: load_c3d_all_frames のコードパターン流用元
```

パイプライン上の位置:

```
NPZ --(npz_to_c3d.py)--> C3D --(filter_c3d.py)--> C3D(平滑化済み)
                                                    ├--> Blender io_anim_c3d 取り込み（手作業）
                                                    │      └--> fps_camera_pose.py
                                                    └--> render_keypoints.py
```

## 1.3 技術スタック

| 項目 | 内容 |
|---|---|
| 言語 | Python 3.10 |
| 実行環境 | phase4 uv 環境（`uv run --project phase4 python filter_c3d.py ...`）。Blender・GPU不要 |
| NumPy | 配列処理（既存依存） |
| SciPy | `scipy.signal.butter` / `scipy.signal.filtfilt`。**新規追加**（`scipy>=1.11`）。選定理由: ゼロ位相Butterworthフィルタはモーションキャプチャ後処理の標準手法で、SciPyが事実上の標準実装 |
| py-c3d | C3D読み書き（`c3d>=0.6.0`、既存依存） |

`phase4/pyproject.toml` の `dependencies` に `"scipy>=1.11"` を追加し、
`docs/TECH_STACK.md` に用途・選定理由・バージョンを追記する。

## 1.4 各機能の詳細設計

### 1.4.1 C3D読み込み（FR-001）

**関数**: `load_c3d(c3d_path: str) -> tuple[np.ndarray, np.ndarray, list[str], float, int]`

- 入力: C3Dファイルパス。
- 出力:
  - `points_mm`: (F, J, 3) float64、C3D raw座標 mm
  - `residuals`: (F, J) float64
  - `labels`: 長さ J の str リスト（strip済み）
  - `point_rate`: float [Hz]。取得不能（None・非数値）なら 0.0
  - `first_frame`: int（`reader.first_frame`）
- 処理: `render_keypoints.load_c3d_all_frames` と同じパターンで py-c3d Reader から
  全フレームを読む。フレームごとの dict ではなく (F, J, 3) の一括配列に積む。
- F == 0 なら `ValueError("C3Dにフレームがありません: <パス>")`。
- 入力規約の検証（本関数内で行い、違反時は `ValueError`）:
  - `POINT:UNITS` パラメータの文字列値（`.strip()` 後）が `"mm"` でなければエラー
    （パラメータが存在しない場合もエラー。feat-018 出力には必ず存在する）。
  - `POINT:X_SCREEN` / `POINT:Y_SCREEN` パラメータが存在し、`.strip()` 後の値が
    `"+Z"` / `"+Y"` に完全一致しなければエラー（欠落もエラー。欠落を許すと規約外C3Dが
    通過し、出力時に規約値が付与されて姿勢解釈が黙って変わるため）。
  - パラメータ取得は `reader.get('POINT:UNITS')` の戻り（`None` またはパラメータ
    オブジェクトの `string_value`）を使う。

### 1.4.2 Butterworthゼロ位相フィルタ（FR-002）

**関数**: `lowpass_filtfilt(series: np.ndarray, cutoff: float, rate: float) -> np.ndarray`

- 入力: `series` (N,) または (N, C) float64（時間軸は axis=0）、
  `cutoff` [Hz]（0 < cutoff < rate/2 は呼び出し側で検証済み）、`rate` [Hz]。
- 出力: 同形状の平滑化済み配列。
- 処理（この式を正本とする）:

```python
# 意図伝達用スニペット（そのままコピーする目的ではない）
from scipy.signal import butter, filtfilt
b, a = butter(N=2, Wn=cutoff / (rate / 2.0), btype="low")   # 2次
return filtfilt(b, a, series, axis=0)                        # 往復適用で実効4次・ゼロ位相
```

- フィルタ次数は 2 で固定（`filtfilt` の往復適用により実効4次。バイオメカニクス分野の
  慣例「4th-order zero-lag Butterworth」に相当）。次数のCLIオプションは設けない。
- `filtfilt` のパディングは既定（`padtype='odd'`, `padlen = 3 * max(len(a), len(b)) = 9`）を
  使う。したがって適用には時系列長 N >= `MIN_FILTER_SAMPLES = 10`（padlen + 1）が必要。
  この定数は呼び出し側（1.4.3）が使用する。

### 1.4.3 無効サンプルの取り扱い（FR-003）

**関数**: `split_segments(valid: np.ndarray, max_gap: int) -> list[tuple[int, int]]`

有効フラグ (F,) bool と最大補間ギャップ長から、セグメント `[lo, hi]`（両端含む・
両端は有効サンプル）のリストを返す。

- 処理手順:
  1. 有効サンプルの添字列を取り、隣接する有効添字の差 `d` を順に見る。
  2. `d - 1 <= max_gap`（間の無効サンプル数が max_gap 以下）なら同一セグメントに含める。
     `d - 1 > max_gap` ならそこでセグメントを区切る。
  3. 各セグメントは `(最初の有効添字, 最後の有効添字)` のタプル。有効サンプルが
     0 個なら空リストを返す。
- 先頭・末尾の無効区間はどのセグメントにも含まれない（セグメント両端は有効サンプル）。

**関数**: `filter_marker(points_mm: np.ndarray, residuals: np.ndarray, cutoff: float, rate: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int]]]`

マーカー1本分 `points_mm` (F, 3)、`residuals` (F,) を受け取り、
`(平滑化後座標 (F,3), スキップしたセグメント [(lo, hi), ...])` を返す。

- 有効フラグ: `valid = (residuals >= 0) & np.isfinite(points_mm).all(axis=1)`（(F,) bool）。
- 処理手順:
  1. `segments = split_segments(valid, max_gap)` を得る。
  2. 出力配列を入力座標のコピーで初期化する。
  3. 各セグメント `(lo, hi)` について:
     a. セグメント長 `hi - lo + 1 < MIN_FILTER_SAMPLES` ならそのセグメントは
        フィルタせずスキップリストに追加して次へ（値は入力のまま）。
     b. 区間 `[lo, hi]` を切り出し、区間内の無効サンプル（ギャップ、長さは
        max_gap 以下であることが split_segments で保証される）を成分ごとに
        `np.interp`（有効サンプルの添字と値による線形補間）で埋める。
     c. 埋めた区間に `lowpass_filtfilt` を適用する。
     d. `[lo, hi]` かつ `valid` の位置**のみ**フィルタ結果で置き換える
        （無効位置・区間外は入力値のまま）。
- セグメントごとに独立にフィルタするため、長いギャップを挟んだ別セグメントの値が
  混入しない（レビュー指摘への対応。ADR-5）。
- residual は本関数では変更しない（無効サンプルは書き出し時（1.4.4）に入力の residual を
  そのまま使うことで無効のまま維持される）。
- ギャップ補間値は手順 3c の入力にのみ使い、出力には反映しない（手順 3d で valid 位置
  しか置き換えないため自動的に満たされる）。

**関数**: `filter_all_markers(points_mm: np.ndarray, residuals: np.ndarray, cutoff: float, rate: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int, int]]]`

(F, J, 3) 全体に対し、マーカー j = 0..J-1 のループで `filter_marker` を呼ぶ。
戻り値は `(平滑化後 (F,J,3), スキップした (マーカー添字, lo, hi) のリスト)`。

### 1.4.4 C3D書き出し（FR-004）

**関数**: `write_c3d(out_path: str, points_mm: np.ndarray, residuals: np.ndarray, labels: list[str], rate: float, first_frame: int) -> None`

- `npz_to_c3d.write_c3d` と同じ py-c3d Writer パターンを使う。相違点:
  - residual は入力C3Dから引き継いだ値を使う（`residuals[f, j] < 0` または座標が非有限なら
    無効: 座標 0.0・residual -1.0 で書く。それ以外は座標をそのまま・residual は
    引き継ぎ値を書く）。
  - `set_start_frame(first_frame)`（入力C3Dの開始フレーム番号を維持。1.6 の ADR-3）。
  - 単位は `POINT_UNITS = "mm"`、スクリーン軸は `X_SCREEN = "+Z"` / `Y_SCREEN = "+Y"` の
    固定値（`npz_to_c3d.py` と同一の定数。ADR-4）。
- 原子的書き出し: `out_path + ".tmp"` に書く → `_verify_c3d`（py-c3d で読み戻し、
  フレーム数・ラベル一致を検証）→ 成功時のみ `os.replace(tmp, out_path)`。
  例外時は tmp を削除して re-raise（`npz_to_c3d.py` と同一パターン）。

### 1.4.5 パラメータ検証とエラー処理（FR-005）

`main()` で以下の順に検証し、違反時は日本語メッセージを stderr に出力して return 1
（`npz_to_c3d.main` と同じ `try/except (ValueError, RuntimeError, FileNotFoundError, OSError)` 構造）。

| # | 条件 | メッセージ内容（要素） |
|---|---|---|
| 1 | 出力パス拡張子が `.c3d` でない | 出力パスの拡張子が .c3d ではない旨とパス |
| 2 | `os.path.realpath(出力) == os.path.realpath(入力)`、または出力パスが既存ファイルで `os.path.samefile(入力, 出力)` が真（シンボリックリンク経由の同一実体も拒否） | 出力パスが入力と同一（上書き防止）の旨とパス |
| 3 | `--cutoff <= 0` | カットオフ周波数は正の値である旨と指定値 |
| 4 | `--max-gap <= 0` | 最大補間ギャップ長は正の整数である旨と指定値 |
| 5 | 入力C3Dが開けない/読めない | 例外メッセージ（FileNotFoundError/OSError/ValueError 経由） |
| 6 | F == 0 | C3Dにフレームがない旨（1.4.1 内で送出） |
| 7 | 単位（POINT:UNITS）が mm でない | 単位が mm でない旨と実際の値（1.4.1 内で送出） |
| 8 | スクリーン軸（POINT:X_SCREEN / POINT:Y_SCREEN）が欠落、または +Z/+Y と異なる | スクリーン軸が欠落／規約と異なる旨と実際の値（1.4.1 内で送出） |
| 9 | `first_frame` が 1〜65534 の範囲外 | 開始フレーム番号が対応範囲外である旨と値（py-c3d 16bit制約） |
| 10 | rate 決定: C3D point_rate > 0 なら `--rate` 指定は禁止（併存でエラー）。point_rate <= 0 なら `--rate` を採用し、`--rate` 未指定ならエラー | 併存時: point rate があるため --rate は指定できない旨と両方の値。欠損時: point rate が取得できない旨と `--rate` での指定を促す文言 |
| 11 | `--cutoff >= rate / 2` | カットオフがNyquist周波数以上である旨と両方の値 |

検証 1〜4 は C3D 読み込み前に行う（重い処理の前に弾く）。6〜8 は読み込み中（1.4.1）、
9〜11 は読み込み後。採用された rate（point_rate または `--rate`）はフィルタ計算と
出力C3Dの point rate の双方に使う（point_rate 欠損時の `--rate` はその欠損値の補完で
あり、有効な point rate の上書きは禁止するため、入力と出力で再生速度が変わることはない）。

### 境界条件

- F == 0: エラー終了（検証6）。
- F == 1、またはセグメント長が `MIN_FILTER_SAMPLES` 未満: そのセグメントは
  平滑化スキップ（入力値のまま出力）＋警告表示。全セグメントがスキップされても
  エラーにはせず、C3Dはそのまま書き出す（警告で気付ける）。
- 全サンプル無効のマーカー: セグメントが0個になり全フレーム入力の無効表現のまま出力。
- J == 0（マーカー0件）: write_c3d で `ValueError`（`npz_to_c3d.py` と同じ）。
- `--cutoff` が Nyquist にごく近い値: 検証7は `>=` 比較。`rate/2` 未満なら許可
  （butter の Wn < 1.0 が保証される）。

## 1.5 状態遷移

該当なし（ワンショットのCLIバッチ処理）。

## 1.6 ファイル・ディレクトリ設計

- 入力: 任意パスのC3Dファイル（位置引数 `c3d_path`）。
- 出力: `--output` 指定パス。省略時は `<入力の拡張子除去>_filtered.c3d`
  （例: `data/keypoints.c3d` → `data/keypoints_filtered.c3d`）。
- 出力ディレクトリ: `out_dir = os.path.dirname(out_path)` とし、`out_dir` が
  空文字列でない場合のみ `os.makedirs(out_dir, exist_ok=True)` を呼ぶ
  （カレントディレクトリへの出力で `os.makedirs("")` が失敗するのを防ぐ）。
- 設定ファイルは使わない（CLIオプションのみ）。

### 設計判断の記録（ADR）

- **ADR-1: 独立スクリプト（C3D→C3D）とする。** 却下案: (a) `fps_camera_pose.py` 内で
  ボーン位置を平滑化 → カメラだけ滑らかになり映像内の人体のカクつきが残るため却下。
  (b) `npz_to_c3d.py` に `--smooth` オプション追加 → 既存C3D・他所由来のC3Dに適用
  できないため却下。独立スクリプトは全下流（Blenderアーマチュア、fps_camera_pose.py、
  render_keypoints.py）に一括で効き、フィルタ前後の比較もファイル単位でできる。
- **ADR-2: フィルタは2次Butterworth + filtfilt（実効4次・ゼロ位相）固定。**
  却下案: 移動平均（動きの速い成分が鈍る・慣例でない）、Savitzky-Golay（パラメータが
  窓幅と次数の2つになる）、One Euro Filter（リアルタイム向けでオフライン一括処理には
  利点がない）。カットオフ周波数1つで強さを調整でき、モーキャプ後処理の標準手法である
  Butterworthゼロ位相を採用。デフォルトカットオフは 6.0 Hz（バイオメカニクスで
  人体動作の平滑化に慣用される値。対象データは座位・低速なので更に下げる余地があるが、
  それは `--cutoff` で実データを見て調整する）。
- **ADR-3: 開始フレーム番号は入力C3Dから引き継ぐ。対応範囲は 1〜65534 とし、
  範囲外はエラー終了する。** py-c3d 0.6.0 のヘッダは 16bit で 65535 以上の開始フレームを
  正しく保持できない（feat-018 で1固定にした理由と同じ制約）。却下案: (a) 1固定 →
  他所由来のC3Dのフレーム番号対応を壊す。(b) 範囲外を黙って正規化 → フレーム対応の
  暗黙変更はバグの温床になるため、明示的なエラーとする。現行パイプラインの入力
  （feat-018 出力）は常に1始まりであり実害はない。
- **ADR-4: 入力は本プロジェクト規約のC3D（mm / +Z / +Y）に限定し、規約外はエラー
  終了する。出力の単位・スクリーン軸は規約の固定値で書く。** 却下案: (a) 入力の
  メタデータを読み取ってそのまま引き継ぐ → py-c3d Reader からのパラメータ読み出しは
  揺れがあり、誤読すると下流（Blender io_anim_c3d）でスケール・姿勢が黙って壊れる。
  (b) 規約外入力を黙って規約値に正規化 → 単位・軸の暗黙変更はデータ破損に直結する
  ため却下。入力検証（1.4.1）でエラーにすることで「維持」と「固定値書き込み」が
  等価であることを保証する。
- **ADR-5: 最大補間ギャップ長（`--max-gap`、既定10フレーム）以下のギャップのみ
  線形補間してフィルタし、それを超えるギャップではセグメント分割して独立にフィルタ
  する。出力では無効サンプルを無効のまま維持する。** 却下案: (a) 無効を含むマーカーは
  エラー → 現行データで将来NaNが混じった途端に使えなくなる。(b) 補間値を有効値として
  出力 → 欠損の捏造になる。(c) ギャップ長無制限の補間 → 長い欠損を人工的な直線で
  つなぐと、その値が filtfilt を介して周辺の有効サンプルに混入し軌跡を壊すため却下
  （レビュー指摘）。補間はフィルタの連続性確保のためだけに使う。既定10フレームは
  30Hz換算で約0.33秒（この程度なら線形補間がフィルタに与える影響は局所的）。

## 1.7 インターフェース定義

```python
# 定数
MIN_FILTER_SAMPLES: int = 10       # filtfilt 既定 padlen(9) + 1
DEFAULT_MAX_GAP: int = 10          # 最大補間ギャップ長の既定値 [フレーム]
MAX_FIRST_FRAME: int = 65534       # py-c3d 16bit制約による開始フレーム上限
POINT_UNITS: str = "mm"
X_SCREEN: str = "+Z"
Y_SCREEN: str = "+Y"

def load_c3d(c3d_path: str) -> tuple[np.ndarray, np.ndarray, list[str], float, int]: ...
    # 戻り値: (points_mm (F,J,3), residuals (F,J), labels, point_rate, first_frame)
    # 単位・スクリーン軸の規約検証も行う（違反は ValueError）

def lowpass_filtfilt(series: np.ndarray, cutoff: float, rate: float) -> np.ndarray: ...

def split_segments(valid: np.ndarray, max_gap: int) -> list[tuple[int, int]]: ...
    # valid: (F,) bool → セグメント [(lo, hi), ...]（両端含む・両端は有効サンプル）

def filter_marker(points_mm: np.ndarray, residuals: np.ndarray, cutoff: float,
                  rate: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int]]]: ...
    # points_mm: (F,3), residuals: (F,) → (filtered (F,3), スキップセグメント [(lo,hi),...])

def filter_all_markers(points_mm: np.ndarray, residuals: np.ndarray, cutoff: float,
                       rate: float, max_gap: int) -> tuple[np.ndarray, list[tuple[int, int, int]]]: ...
    # (F,J,3), (F,J) → (filtered (F,J,3), スキップした (マーカー添字, lo, hi) リスト)

def write_c3d(out_path: str, points_mm: np.ndarray, residuals: np.ndarray,
              labels: list[str], rate: float, first_frame: int) -> None: ...

def _verify_c3d(c3d_path: str, expected_frames: int, expected_labels: list[str]) -> None: ...

def main(argv=None) -> int: ...
```

### CLI

```
uv run --project phase4 python filter_c3d.py <c3d_path> [--output PATH] [--cutoff HZ] [--rate HZ] [--max-gap N]
```

| 引数 | 型 | 既定値 | 説明 |
|---|---|---|---|
| `c3d_path` | str（位置） | 必須 | 入力C3Dパス |
| `--output` | str | `<入力>_filtered.c3d` | 出力C3Dパス |
| `--cutoff` | float | 6.0 | ローパスのカットオフ周波数 [Hz] |
| `--rate` | float | None | C3Dの point rate が欠損（0以下）の場合の補完値 [Hz]。有効な point rate があるC3Dに指定するとエラー（検証10） |
| `--max-gap` | int | 10 | 線形補間で埋める無効サンプル連続数の上限 [フレーム]。超えるギャップはセグメント分割 |

argparse は `allow_abbrev=False`（`npz_to_c3d.py` と同一）。

- モジュール間呼び出し方向: `main → load_c3d / filter_all_markers → filter_marker →
  lowpass_filtfilt`、`main → write_c3d → _verify_c3d`。循環なし。
- py-c3d と scipy の import は関数内 import とする（プロジェクト慣例。ルート venv での
  pytest 収集時の ImportError 回避）。numpy はモジュールレベルで import する。

## 1.8 ログ・デバッグ設計（FR-006）

logging モジュールは使わず print で標準出力に日本語表示する（既存スクリプトと同一方針）。
エラーは stderr（`print(..., file=sys.stderr)`）＋ 終了コード1。

正常系の出力ポイント（順序どおり）:

```
C3D読み込み: <入力パス>
  フレーム数 F=<F>, 関節数 J=<J>, point rate=<rate> Hz, 開始フレーム=<first_frame>
フィルタ: Butterworth 2次 filtfilt（ゼロ位相）, カットオフ <cutoff> Hz, 最大補間ギャップ <max_gap> フレーム
  無効サンプル: <ラベル>=<件数>, ...        # 1件以上のマーカーのみ。無ければ「なし」
警告: マーカー <ラベル> のセグメント [<lo>..<hi>]（長さ <n> < 10）は平滑化をスキップしました   # 該当時のみ
  平滑化変位 [mm]: 平均=<mean>, 最大=<max>   # 全有効サンプルの |filtered - original| ノルム
完了: <出力パス>（<F> フレーム, <J> 関節）
```

## テスト設計

`tests/test_filter_c3d.py` を新規作成し、`uv run pytest -v` で実行、結果を
`tests/results/feat-020_test_result.txt` に保存する。

注意: ルート venv には scipy / c3d がないため、scipy・c3d を要するテストは
`pytest.importorskip` を使うか、`uv run --project phase4 pytest` で実行する。
実行方法は実装時に確定し、テスト結果ファイルに実行コマンドを記録する。

| テスト | 対応要求 | 内容 |
|---|---|---|
| test_constant_series_preserved | FR-002 | 一定値入力 → 出力一致（atol=1e-6） |
| test_lowfreq_preserved_highfreq_attenuated | FR-002 | 低周波保存・高周波減衰・ピーク位置ずれ≦1フレーム |
| test_gap_interpolation_and_invalid_kept | FR-003 | max_gap以下の中間ギャップ: 有効位置平滑化・無効位置維持 |
| test_long_gap_splits_segments | FR-003 | max_gap超のギャップ: セグメント独立フィルタ（一方のセグメントの値を大きく変えても他方の結果が不変） |
| test_edge_invalid_untouched | FR-003 | 先頭/末尾の無効区間が入力のまま |
| test_short_segment_skipped | FR-003 | 長さ9のセグメント → スキップ・入力のまま |
| test_roundtrip_c3d | FR-001/004 | 合成C3Dを書く→filter→読み戻しでフレーム数・ラベル・rate・開始フレーム一致 |
| test_output_same_as_input_rejected | FR-005 | 入出力同一パスで終了コード1 |
| test_output_symlink_to_input_rejected | FR-005 | 入力が実ファイルへのsymlink・出力が実ファイルパスの場合に終了コード1 |
| test_cutoff_validation | FR-005 | cutoff<=0 / cutoff>=Nyquist で終了コード1 |
| test_rate_conflict_rejected | FR-005 | 有効な point rate を持つC3Dに --rate 指定で終了コード1 |
| test_wrong_units_rejected | FR-005 | POINT:UNITS が mm でないC3Dで終了コード1 |
| test_screen_axis_missing_or_wrong_rejected | FR-005 | スクリーン軸パラメータ欠落／規約外（+Y/+X 相当）のC3Dで終了コード1 |

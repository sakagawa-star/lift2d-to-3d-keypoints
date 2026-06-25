# feat-018 機能設計書: NPZ→C3D変換スクリプト

> 本設計書のコード例は「意図の伝達」が目的であり、そのままコピーして使うものではない。

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 NPZ読み込みと検証 | 1.4.1 `load_npz` |
| FR-002 座標変換 | 1.4.2 `world_to_c3d_raw` |
| FR-003 C3D書き出し | 1.4.3 `write_c3d` |
| FR-004 CLI | 1.7 `_build_parser` / `main` |
| FR-005 Blender取り込み確認 | 1.4.3（X_SCREEN/Y_SCREEN/UNITS）+ 手動テスト |

## 1.2 システム構成

- 新規ファイル: `phase4/npz_to_c3d.py`（単一スクリプト、Blender非依存）。
- 既存への影響なし（`render_keypoints.py` 等は変更しない）。
- 依存方向: `npz_to_c3d.py` → `numpy`, `c3d`（py-c3d）。循環依存なし。

```
phase4/
├── npz_to_c3d.py        # 本案件で新規作成
├── render_keypoints.py  # 既存（参照のみ。c3d_to_calib の逆変換を本スクリプトで実装）
└── data/
    ├── session001_f145749_world300.npz   # 入力例
    └── session001_f145749_world300.c3d   # 出力例（--output 省略時）
```

### モジュール構成（関数）

| 関数 | 責務 |
|---|---|
| `load_npz(npz_path)` | NPZ読み込み・検証（FR-001） |
| `world_to_c3d_raw(x3d_world)` | world(m)→C3D raw(mm) 軸入替変換（FR-002） |
| `write_c3d(out_path, raw_mm, frame_ids, joint_names, fps)` | C3D書き出し（FR-003） |
| `_build_parser()` | argparse 定義（FR-004） |
| `main(argv=None)` | 全体制御・終了コード（FR-004） |

## 1.3 技術スタック

- **言語**: Python 3.10
- **ライブラリ**:
  - NumPy（数値計算。既存）
  - py-c3d 0.6.0（`import c3d`。phase4 venv に導入済み。`c3d.Writer` を使用）
- **パッケージ管理**: uv（`phase4/pyproject.toml`、独立環境）
- **選定理由**: py-c3d は `render_keypoints.py` の C3D 読み込みで既に使用しており、
  io_anim_c3d も同じ py-c3d 0.6.0 を同梱しているため、書き出しと取り込みの整合が取りやすい。
- **新規ライブラリ追加なし**。

## 1.4 各機能の詳細設計

### 1.4.1 `load_npz`（FR-001）

#### データフロー

- 入力: `npz_path: str`（NPZファイル）。
- 中間: `numpy.load(npz_path, allow_pickle=True)`。
- 出力: タプル `(x3d_world, frame_ids, joint_names)`
  - `x3d_world`: `np.ndarray`, shape `(F, J, 3)`, dtype float64 へ変換, 単位 m。
  - `frame_ids`: `np.ndarray`, shape `(F,)`, dtype int64 へ変換。
  - `joint_names`: `list[str]`, 長さ J。

#### 処理ロジック

```
d = np.load(npz_path, allow_pickle=True)
必須キー = {"x3d_world", "frame_ids", "joint_names"}
1. 必須キーが d.files に全て存在するか確認。欠ければ ValueError。
2. x3d_world = np.asarray(d["x3d_world"], dtype=np.float64)
   - x3d_world.ndim == 3 かつ x3d_world.shape[2] == 3 を確認。違えば ValueError。
   F, J = x3d_world.shape[0], x3d_world.shape[1]
3. frame_ids = np.asarray(d["frame_ids"]).reshape(-1).astype(np.int64)
   - len(frame_ids) == F を確認。違えば ValueError。
   - F >= 2 のとき np.all(np.diff(frame_ids) == 1) を確認。違えば ValueError
     （連番でない）。
4. joint_names = [str(n) for n in d["joint_names"].tolist()]
   - len(joint_names) == J を確認。違えば ValueError。
5. return (x3d_world, frame_ids, joint_names)
```

- 分岐は全て「条件を満たさなければ `ValueError(具体メッセージ)`」。`main` で捕捉して終了コード1。

#### 境界条件

- F == 1（1フレーム）: 連番チェックはスキップ（`F >= 2` ガード）。書き出しは可能。
- F == 0: `x3d_world.shape[0] == 0`。`write_c3d` で「フレームが無い」エラーとする（1.4.3）。

### 1.4.2 `world_to_c3d_raw`（FR-002）

#### データフロー

- 入力: `x3d_world: np.ndarray`, shape `(F,J,3)`, m。
- 出力: `raw_mm: np.ndarray`, shape `(F,J,3)`, float64, mm。

#### 処理ロジック

```
X = x3d_world[..., 0]; Y = x3d_world[..., 1]; Z = x3d_world[..., 2]
raw_mm = np.stack([Y, Z, X], axis=-1) * 1000.0
return raw_mm
```

- 根拠: `render_keypoints.c3d_to_calib` は raw `(px,py,pz)` mm → `(pz,px,py)×0.001` m。
  これを world で復元するには `px=Y, py=Z, pz=X`（×1000）とすればよい。
  すなわち `c3d_to_calib(world_to_c3d_raw(w)) == w`。

#### 境界条件

- NaN/Inf を含む点はそのまま伝播する（無効化は 1.4.3 の residual で行う）。

### 1.4.3 `write_c3d`（FR-003）

#### データフロー

- 入力:
  - `out_path: str`
  - `raw_mm: np.ndarray`, shape `(F,J,3)`, mm
  - `frame_ids: np.ndarray`, shape `(F,)`, int
  - `joint_names: list[str]`, 長さ J
  - `fps: float`
- 出力: `out_path` に C3D ファイルを書き出す（戻り値なし）。

#### 処理ロジック

```
F, J = raw_mm.shape[0], raw_mm.shape[1]
1. F == 0 なら ValueError（書き出すフレームが無い）。J == 0 なら ValueError。
2. writer = c3d.Writer(point_rate=fps, point_units='mm')
3. writer.set_point_labels(joint_names)
4. writer.set_screen_axis('+Z', '+Y')   # Blender 正立（1.6 ADR-2、world鉛直Z→Blender上Z）
5. writer.set_start_frame(1)             # 1始まり固定（ADR-5。絶対frame_idsは埋め込まない）
6. 各フレーム f について point 配列 (J,5) を作る:
     points = np.zeros((J, 5), dtype=np.float32)
     有限判定 finite = np.isfinite(raw_mm[f]).all(axis=1)   # (J,)
     points[:, 0:3] = np.where(finite[:, None], raw_mm[f], 0.0)  # 無効点座標は0
     points[:, 3] = np.where(finite, 0.0, -1.0)   # residual（有効0 / 無効-1）
     points[:, 4] = 0.0                            # camera mask
     analog = np.zeros((0, 1), dtype=np.float32)   # アナログ無し（py-c3d 0.6.0 で動作確認済み。
                                                    #   読み戻し時に "No analog data" 警告が出るが無害）
     writer.add_frames([(points, analog)])
7. アトミック書き出し（ADR-6）:
     tmp = out_path + ".tmp"（同一ディレクトリ）
     with open(tmp, 'wb') as h: writer.write(h)
     読み戻し検証: c3d.Reader(tmp) で frame 数 == F、point_labels == joint_names を確認。
       不一致なら tmp を削除して RuntimeError。
     os.replace(tmp, out_path)   # 検証成功後にのみ最終パスへ確定
```

> 補足: フレーム番号を 1 始まりにする根拠は ADR-5。`set_start_frame(int(frame_ids[0]))`
> （145599）は py-c3d 0.6.0 で `first_frame=145595` / `count=301` / EOF 警告となり破綻する
> ことを実測で確認済み。`set_start_frame(1)` は `first_frame=1` / `count=F` で正常。

- **py-c3d の癖（実装時に確認）**: `add_frames` が要求する point 列数（4 or 5）と空 analog の
  形状は py-c3d のバージョンで差がある。0.6.0 の `Writer.from_reader` / `add_frames` 実装と
  `reader.read_frames` の戻り値（`render_keypoints` では `points[:, :3]` と `points[:, 3]` を
  使用 = 5列）を基準にして、point は (J,5)、analog は空配列で合わせる。実装直後に
  「書き出し→py-c3d で読み戻し」のラウンドトリップ確認を行う（下記テスト方針）。

#### エラーハンドリング

| エラー | 検出 | 処理 |
|---|---|---|
| フレーム0件 | `F == 0` | `ValueError`。`main` で捕捉し終了コード1。出力ファイルは作らない。 |
| 関節0件 | `J == 0` | `ValueError`。同上。 |
| 出力先ディレクトリ不在 | `open` が `FileNotFoundError` | メッセージを stderr、終了コード1。一時ファイルは残さない。 |
| 読み戻し検証の不一致 | `Reader` のフレーム数/ラベルが不一致 | 一時ファイルを削除して `RuntimeError`。最終パスは変更しない（ADR-6）。 |
| 16bit フレーム上限 | （設計で回避） | フレーム番号を 1 始まり固定にしたため発生しない（ADR-5）。 |

#### 境界条件

- J == 0: ラベル空。`set_point_labels([])` 後、point (0,5)。py-c3d が拒否する可能性があるため
  J == 0 は `ValueError` とする（関節が無い NPZ は対象外）。
- 無効サンプル（NaN）: residual を `-1.0`。io_anim_c3d / render_keypoints いずれも
  `residual >= 0` を有効条件にしているため、無効点は両方で正しくスキップされる。

### Blender 取り込み時の座標変換（検証根拠、FR-005）

取り込み後に画面表示されるワールド座標は、**2段の変換**で決まる（investigation.md イテレーション2）。

1. `use_manual_orientation=False` 時、io_anim_c3d は `global_orient = O_data.T`
   （`O_sys = I`、`sys_axis_up=[0,0,1]`, `sys_axis_forw=[0,1,0]`）で `p_local = O_data.T @ p_raw` を計算し、
   これを **pose bone の `location`** に書き込む（c3d_importer.py:228）。
2. pose bone の `location` は**ボーンのローカル座標**。ボーンは rest 状態で +Z 向き
   （head=(0,0,0)→tail=(0,0,bone_size), c3d_importer.py:337-338）に作られ、その rest 行列
   `B`（local→world）は Blender の `vec_roll_to_mat3`（roll=0）で `Rx(+90)`、すなわち
   `B = [[1,0,0],[0,0,-1],[0,1,0]]`（`(lx,ly,lz)→(lx,-lz,ly)`）。
3. よって**表示ワールド座標** `p_world_disp = B @ p_local = B @ (O_data.T @ p_raw)`。
   - 表示鉛直成分 `p_world_disp.z = (O_data.T @ p_raw).y = axis_y · p_raw` となり、
     **鉛直の上下は `Y_SCREEN`（axis_y）だけで決まる**（`X_SCREEN` は鉛直に無関係）。

- world の鉛直（上）軸は **Z**（spec §4 本文「y=垂直」は「←キャリブ依存」の注記どおり
  本データには当てはまらず、実測で全フレーム頭が +Z 側）。`p_raw = (Y_w, Z_w, X_w)`。
- `X_SCREEN='+Z'`（axis_x=[0,0,1]）, `Y_SCREEN='+Y'`（axis_y=[0,1,0]）とすると:
  - `axis_y · p_raw = Z_w` → 表示鉛直 = `Z_w`（world の真の鉛直）→ **人体が正立**。
  - 全体は `p_world_disp = (X_w, Y_w, Z_w)`（恒等＝Blender が world をそのまま表示）。
- 反証データ（investigation.md イテレーション2 で実測一致）:
  - `Y_SCREEN='-Y'` → 表示鉛直 = −Z_w（頭が下＝上下逆）。
  - `Y_SCREEN='+X'` → 表示鉛直 = +Y_w（非鉛直、頭が下＝上下逆）。

> 注: 最終確定は実装時に「writer で書く → 実物の `C3DParseDictionary.axis_interpretation` で
> global_orient を取得 → ボーン rest 行列 `B` を適用」して、実データの Head の表示ワールド Z が
> 両Ankle より大きい（正立）ことを数値確認する。`B` を省いた手計算だけでは実機を再現できない
> （イテレーション1の失敗要因）。ずれた場合も X/Y_SCREEN の符号・軸のみ調整し、生レイアウトと
> render_keypoints 互換は変更しない。

## 1.6 ファイル・ディレクトリ設計

- 入力: 任意の NPZ パス（CLI 指定）。
- 出力: `--output` 指定パス。省略時は `os.path.splitext(npz_path)[0] + '.c3d'`。
- 出力 C3D のフレーム番号は **1..F の連番**（ADR-5）。NPZ の絶対 `frame_ids` は C3D には
  保持せず、C3D フレーム `i` と `frame_ids[i-1]` の対応を標準出力に表示する。
  （したがって `render_keypoints.py` で連番PNGを出すと `frame_000001.png`〜 となり、
  絶対動画フレーム番号にはならない。）

## 1.7 インターフェース定義

```python
def load_npz(npz_path: str) -> tuple[np.ndarray, np.ndarray, list[str]]: ...
def world_to_c3d_raw(x3d_world: np.ndarray) -> np.ndarray: ...
def write_c3d(
    out_path: str,
    raw_mm: np.ndarray,
    frame_ids: np.ndarray,
    joint_names: list[str],
    fps: float,
) -> None: ...
def _build_parser() -> argparse.ArgumentParser: ...
def main(argv=None) -> int: ...
```

- CLI（`_build_parser`）:
  - 位置引数 `npz_path: str`
  - `--output`（`default=None`）
  - `--fps`（`type=float`, `default=30.0`）
  - `allow_abbrev=False`（既存 `render_keypoints.py` と方針統一）
- `main` の処理順（FR-004）:
  1. 引数解析。`out_path = args.output or (splitext(npz_path)[0] + '.c3d')`。
  2. 出力パス検証（重い NPZ ロード前に弾く）:
     - `out_path` の拡張子が `.c3d`（`lower()`）でなければ stderr + return 1。
     - `os.path.abspath(out_path) == os.path.abspath(npz_path)` なら stderr + return 1。
  3. `load_npz` → `world_to_c3d_raw` → `write_c3d`。各 `ValueError`/`RuntimeError` を捕捉し
     stderr へ出力、return 1。
  4. C3D フレーム番号と NPZ `frame_ids` の対応をログ出力（FR-003）。成功で return 0。
- `if __name__ == "__main__": sys.exit(main())`。

## 1.8 ログ・デバッグ設計

- 標準出力（INFO 相当、print）:
  - 入力 NPZ パス・フレーム数 F・関節数 J・フレーム番号範囲・fps
  - 出力 C3D パス・「完了」メッセージ
- 標準エラー（ERROR 相当、`print(..., file=sys.stderr)`）: 検証エラー・書き出しエラーの内容
- 既存スクリプト（`render_keypoints.py`）に倣い、ロギングライブラリは使わず print を用いる。

## テスト方針（実装後）

`tests/` に pytest を追加し、結果を `tests/results/feat-018_test_result.txt` に保存する。

- **T-1 ラウンドトリップ（座標）**: ダミー NPZ（小規模、既知座標, NaN含む1点）を
  `world_to_c3d_raw` → `write_c3d` → py-c3d `Reader` で読み戻し、
  `c3d_to_calib(raw)` が元 `x3d_world` に一致（1e-4 m）すること。NaN点が residual<0 で
  読めること。
- **T-2 メタデータ**: 読み戻した C3D の `point_labels`・`point_rate`・`first_frame`（= 1）・
  フレーム数（= F）・`POINT:UNITS`・`X_SCREEN/Y_SCREEN` が設計値と一致。
- **T-2b 出力パス検証**: 拡張子が `.c3d` 以外、または出力==入力 NPZ のとき終了コード1で
  出力ファイルが生成されないこと。
- **T-3 軸解釈（ボーン rest 行列込み）**: 実物の `io_anim_c3d.c3d_parse_dictionary` を
  （`__init__` の bpy import を回避して）読み込み、生成 C3D に対し `axis_interpretation([0,0,1],[0,1,0])`
  で global_orient を取得 → ボーン rest 行列 `B=[[1,0,0],[0,0,-1],[0,1,0]]` を適用した
  表示ワールド座標で、Head の z が両Ankle の z より大きい（正立）ことを確認する
  （実データ NPZ 非存在時は skip）。手計算モデル単体は実機を再現できないため使わない。
- **T-4 検証エラー**: キー欠落・非連番 frame_ids・形状不一致で `ValueError`/終了コード1。

## 設計判断の記録（ADR）

- **ADR-1 座標規約 = 既存C3D互換（軸入替+mm）**
  - 採用: world `(X,Y,Z)`m → raw `(Y,Z,X)`mm。
  - 理由: `render_keypoints.c3d_to_calib` の**座標変換**でそのまま world に復元できる
    （座標規約の互換のみを指す。フレーム番号は 1..F でありPNG命名は ADR-5 の通り変わる）。
  - 却下: world をそのまま ×1000（軸保存）。→ render_keypoints 側に別途軸調整が必要になり
    二重管理になる。
- **ADR-2 Blender 正立 = X_SCREEN='+Z' / Y_SCREEN='+Y'**
  - 採用: 生レイアウトを変えずに、X/Y_SCREEN だけで Blender の見た目を正立させる。
    io_anim_c3d は pose bone ローカル座標 + rest 行列 `B` で描画するため、表示鉛直 = `axis_y · p_raw`。
    `Y_SCREEN='+Y'` で表示鉛直 = world 鉛直 `Z_w`、全体は `p_world_disp=(X_w,Y_w,Z_w)`（恒等）。
  - 補足（試行錯誤の記録）: 当初 `Y_SCREEN='-Y'`（world鉛直=Y 前提）→ 上下逆（イテレーション1）。
    次に `Y_SCREEN='+X'`（ボーン rest 行列を見落とした手計算）→ なお上下逆（イテレーション2で発覚）。
    最終的にボーン rest 行列 `B` を含むモデルが実測2点を再現し、`Y_SCREEN='+Y'` が正立と確定。
  - 理由: render_keypoints は X/Y_SCREEN を無視して生 mm を直読みするため、X/Y_SCREEN を
    どう設定しても render_keypoints の挙動に影響しない（2消費者を独立に満たせる）。
  - 却下: 生レイアウトを Blender 都合で変える。→ ADR-1（render_keypoints 互換）と両立しない。
- **ADR-3 関節ラベル = NPZ22点そのまま**
  - 採用: `joint_names` をそのまま `POINT:LABELS` に書く。
  - 理由: ヒアリング結果。Blender 取り込み・可視化には十分。Halpe26整形は範囲外。
  - 却下: Halpe26 26点へ整形。→ Spine/Thorax の破棄・足6点の欠損補完が必要で、本案件の
    目的（Blender取り込み）に対して過剰。
- **ADR-4 residual = 有限なら0/非有限なら-1、pnp_ok は未使用**
  - 採用: 幾何的に有効（有限）な点は全て有効サンプルとする。
  - 理由: body-prior 由来の face 点も座標としては有効。`pnp_ok` は精度フラグであり
    有効/無効の判定ではない。C3D に精度フラグを載せる仕組みは範囲外。
  - 却下: `pnp_ok==False` の face 点を無効化。→ 多くのフレームで顔が消え、可視化目的に反する。
- **ADR-5 C3D フレーム番号 = 1 始まり固定**
  - 採用: `set_start_frame(1)`。C3D フレームは 1..F。NPZ 絶対フレーム番号は埋め込まず、
    対応をログ出力する。
  - 理由: py-c3d 0.6.0 のヘッダ first/last frame は 16bit（65535上限）。本案件の開始
    145599 を渡すと読み戻しが破綻する（実測: `first_frame=145595`, `count=301`, EOF）。
    1 始まりなら `first_frame=1`, `count=F` で正常に読み戻せる（実測確認済み）。
  - 却下: `set_start_frame(frame_ids[0])`（絶対番号保持）→ 16bit 破綻。
  - 却下: py-c3d を vendor/patch して 32bit フレーム対応 → `render_keypoints.py` の読み取りも
    巻き込む大改修で「小さく作る」方針に反する。必要なら別案件で検討（範囲外）。
- **ADR-6 アトミック書き出し + 入力上書き防止**
  - 採用: 一時ファイルに書く → py-c3d で読み戻し検証（frame数・ラベル）→ `os.replace` で確定。
    加えて出力パスが入力 NPZ と同一、または拡張子が `.c3d` 以外ならエラー。
  - 理由: `--output input.npz` のような誤指定で入力データを破壊する事故を防ぎ、検証に通った
    C3D のみを残す（信頼性要求）。
  - 却下: `open(out_path,'wb')` に直接書く → 検証前に最終ファイルを上書きし、失敗時に壊れた
    C3D・破壊された入力が残りうる。

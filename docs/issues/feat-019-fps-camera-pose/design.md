# feat-019 機能設計書: FPS頭部追従カメラのポーズ書き出しスクリプト

## 1.1 対応要求マッピング

| 要求ID | 内容 | 設計セクション |
|--------|------|----------------|
| FR-001 | 頭部一次視線方向の計算 | 1.4.1 `compute_anchor_rotation_euler` |
| FR-002 | 各フレームでの姿勢適用とポーズ書き出し | 1.4.2 `export_camera_poses` |
| FR-003 | コマンドライン引数 | 1.4.3 `parse_args` / `default_output_path` |
| FR-004 | 入力検証とエラー処理 | 1.4.4 検証ロジック |
| FR-005 | 姿勢計算結果の健全性検証 | 1.4.1 / 1.4.5 `validate_rotation` |

## 1.2 システム構成

新規ファイル 1 本のみ。既存 `camera_pose.py` の構造を踏襲する。

```
phase4/
├── camera_pose.py         # 既存（変更しない）
├── fps_camera_pose.py     # 新規（本設計書の対象）
└── render.py              # 既存（変更しない）
```

モジュール依存: `fps_camera_pose.py` → `bpy`, `mathutils`, `numpy`, 標準ライブラリ。
他プロジェクトモジュールへの依存なし（循環依存なし）。

関数構成（すべて `fps_camera_pose.py` 内、モジュールレベル関数）:

| 関数 | 責務 |
|------|------|
| `extract_script_args(argv)` | `--` 以降のスクリプト引数を取り出す（既存 camera_pose.py と同一実装を流用） |
| `default_output_path(camera_name)` | デフォルト出力パス生成（同上） |
| `parse_args(argv)` | 引数解析＋デフォルト補完 |
| `compute_anchor_rotation_euler(arm_eval)` | 評価済みアーマチュアから一次視線方向の Euler を計算（FR-001, FR-005） |
| `validate_rotation(rot, frame)` | 回転行列の det/直交性/NaN・Inf を検証（FR-005(a)） |
| `validate_camera_transform(cam, rot_euler, eyes_mid, frame)` | 各フレームの cam 位置・向き整合を検証（FR-005(b)） |
| `export_camera_poses(camera_name, armature_name, anchor_name, output_path)` | 全フレーム書き出し（FR-002, FR-004, FR-005） |
| `main()` | エントリポイント |

## 1.3 技術スタック

- **言語**: Python 3.10（Blender 4.5.5 同梱の Python）
- **実行環境**: Blender 4.5.5 LTS（`/home/sakagawa/Downloads/apps/blender-4.5.5-linux-x64/blender`）、`blender -b`
- **ライブラリ**:
  - `bpy` / `mathutils`（Blender 同梱）: シーン操作・行列/ベクトル演算。`Matrix`, `Vector`, `Euler` を使用
  - `numpy`（既存 camera_pose.py と同様）: `matrix_world` → ネスト list への変換
  - 標準ライブラリ: `argparse`, `json`, `os`, `sys`, `math`（NaN/Inf 判定）, `tempfile`（原子的書き出し）
- **パッケージ管理**: Blender 同梱 Python のため phase4 の uv 環境には依存しない（既存 camera_pose.py と同様）
- **選定理由**: 既存 `camera_pose.py` と同一構成を踏襲し、差分を姿勢計算のみに限定するため。

## 1.4 各機能の詳細設計

### 定数定義（モジュールレベル）

```python
KIRI_MODIFIER_NAME = "KIRI_3DGS_Render_GN"          # 既存 camera_pose.py と同一
DEFAULT_ARMATURE_NAME = "session001_f145749_world300"
DEFAULT_ANCHOR_NAME = "Cam_Anchor"
# 姿勢計算に使うボーン名（Halpe/参考資料準拠、固定）
BONE_LEYE = "LEye"; BONE_REYE = "REye"
BONE_LEAR = "LEar"; BONE_REAR = "REar"
BONE_NOSE = "Nose"; BONE_HEAD = "Head"; BONE_NECK = "Neck"
REQUIRED_BONES = [BONE_LEYE, BONE_REYE, BONE_LEAR, BONE_REAR, BONE_NOSE, BONE_HEAD, BONE_NECK]

EPS = 1e-8          # 正規化前ベクトル長の縮退判定しきい値
ORTHO_TOL = 1e-6    # determinant/直交性の許容誤差
ANG_TOL = 1e-4      # カメラのローカル回転ゼロ判定（rad）
POS_TOL = 1e-4      # カメラ位置＝両目中点の許容誤差（m）
```

### 1.4.1 `compute_anchor_rotation_euler`（FR-001）

**シグネチャ**:
```python
def compute_anchor_rotation_euler(arm_eval: "bpy.types.Object", frame: int) -> "mathutils.Euler":
```
（`frame` は FR-005 の縮退エラーメッセージにフレーム番号を含めるために受け取る）

**データフロー**:
- 入力: 評価済みアーマチュアオブジェクト `arm_eval`（`arm.evaluated_get(depsgraph)` の戻り）と現在フレーム番号。
  `arm_eval.matrix_world`（4x4）と `arm_eval.pose.bones[name].head`（ローカル、`Vector` len=3）を使う。
- 中間: 各ボーン head のワールド座標 `mw @ bone.head`（`Vector`, 単位 m）。
- 出力: `mathutils.Euler`（ラジアン、順序 'XYZ'）。

**処理ロジック**（要求仕様 FR-001 の式を唯一の正本とする。同梱 `fps_camera_handler_reference.md` は
導出背景の参考であり、取得元は評価済み depsgraph 経由に置き換える）:
```python
from mathutils import Matrix, Vector   # 関数内 import（pytest での bpy import 回避に合わせる）
mw = arm_eval.matrix_world
B = arm_eval.pose.bones
LEye = mw @ B[BONE_LEYE].head
REye = mw @ B[BONE_REYE].head
LEar = mw @ B[BONE_LEAR].head
REar = mw @ B[BONE_REAR].head
Nose = mw @ B[BONE_NOSE].head
Head = mw @ B[BONE_HEAD].head
Neck = mw @ B[BONE_NECK].head
ear_mid = (LEar + REar) * 0.5

# FR-005: 各 normalize 前に元ベクトル長を検証（EPS 未満なら縮退 → _fail でエラー終了）
p_raw  = Nose - ear_mid;         _require_len(p_raw,  "Nose-ear_mid", frame)
r0_raw = LEye - REye;            _require_len(r0_raw, "LEye-REye",    frame)
p  = p_raw.normalized()                 # 前方の手がかり（耳中点→鼻）
r0 = r0_raw.normalized()                # 両目軸（直交化の基準）
f_raw  = p - r0 * p.dot(r0);     _require_len(f_raw,  "f(p⊥r0)",      frame)  # p∥r0 で縮退
f  = f_raw.normalized()                 # 前方＝一次視線（両目軸に直交）
u  = f.cross(r0).normalized()           # 上（暫定。f⊥r0 が保証されるので長さ≈1）
hu_raw = Head - Neck;            _require_len(hu_raw, "Head-Neck",    frame)
head_up = hu_raw.normalized()           # 頭頂方向（符号確定の基準）
if u.dot(head_up) < 0:
    u = -u
b = -f                                  # カメラ ローカル +Z = 後方
r = u.cross(b).normalized()             # r = u×b で右手系（det=+1）を保証
rot = Matrix((r, u, b)).transposed()    # 列 = (右, 上, 後方)
validate_rotation(rot, frame)           # FR-005: det/直交/NaN・Inf 検証
return rot.to_euler()
```
※このコード例は意図の伝達が目的だが、計算式・外積順序・正規化は**この通りに実装する**こと
（`Matrix.to_euler()` は行列式+1の直交行列を要求するため、`r = u×b` の導出順序を変えてはならない）。
`_require_len(vec, name, frame)` は `vec.length < EPS` のとき 1.4.4 の様式で
`エラー: フレーム <frame>: ベクトル <name> が縮退しています (length=...)` を stderr 出力して `sys.exit(1)`
する内部ヘルパ。

**境界条件（FR-005 でカバー）**:
- ゼロ長・縮退ベクトル（例: `LEye == REye` で `r0` が長さ0、`Nose == ear_mid`、`p ∥ r0` で `f` が0、
  `Head == Neck`）: 上記 `_require_len` が各 normalize 前に検出し、JSON を書き出さず異常終了する。
  今回の入力（3Dリフトアップ結果）は各キーポイントが物理的に異なる位置を持つ前提だが、
  異常データ混入時の無言誤出力を防ぐため検証を必須とする（要求 FR-005）。

### 1.4.2 `export_camera_poses`（FR-002）

**シグネチャ**:
```python
def export_camera_poses(camera_name: str, armature_name: str,
                        anchor_name: str, output_path: str) -> int:
```
戻り値: 書き出したフレーム数。

**処理ステップ**:
1. `import bpy`（関数内 import。pytest 環境での import エラー回避、既存踏襲）、
   `from mathutils import Vector`（位置整合検証用。`Matrix` は `compute_anchor_rotation_euler` 側で import）。
2. `scene = bpy.context.scene`、`view_layer = bpy.context.view_layer`。
3. オブジェクト取得と検証（FR-004、詳細は 1.4.4）:
   - `cam = scene.objects.get(camera_name)` → None または type≠CAMERA でエラー終了
   - `arm = scene.objects.get(armature_name)` → None または type≠ARMATURE でエラー終了
   - `anchor = scene.objects.get(anchor_name)` → None または type≠EMPTY でエラー終了
   - `cam.parent is not anchor` → エラー終了（アンカー回転が cam.matrix_world に反映される前提のため）
   - `max(abs(a) for a in cam.rotation_euler) > ANG_TOL` → エラー終了（カメラのローカル回転が非ゼロ）
   - 現行フレームで一度 `arm.pose.bones` を参照し、`REQUIRED_BONES` の不足をチェック（不足名を列挙して終了）
4. `scene.frame_start > scene.frame_end` ならエラー終了。
   （位置・回転の整合性検証はステップ6のフレームループ内で全フレーム実施する。FR-005(b)）
5. KIRIモディファイヤの一時無効化（既存 camera_pose.py と同一。`show_viewport` を False にし、
   `try/finally` で元の状態へ復元）。
6. フレームループ（`for frame in range(scene.frame_start, scene.frame_end + 1)`）:
   ```
   scene.frame_set(frame)
   depsgraph = bpy.context.evaluated_depsgraph_get()
   arm_eval = arm.evaluated_get(depsgraph)
   rot_euler = compute_anchor_rotation_euler(arm_eval, frame)   # 縮退/非直交なら内部で exit(1)
   anchor.rotation_euler = rot_euler
   view_layer.update()                     # アンカー回転を子カメラ matrix_world に反映
   # FR-005(b): 適用後のカメラ変換整合性を全フレーム検証（違反なら exit(1)）
   eyes_mid = (arm_eval.matrix_world @ arm_eval.pose.bones[BONE_LEYE].head
               + arm_eval.matrix_world @ arm_eval.pose.bones[BONE_REYE].head) * 0.5
   validate_camera_transform(cam, rot_euler, eyes_mid, frame)
   render = scene.render
   focal_px = (cam.data.lens / cam.data.sensor_width) * render.resolution_x
   camera_data.append({
       'frame': frame,
       'c2w': np.array(cam.matrix_world).tolist(),
       'fx': focal_px, 'fy': focal_px,
       'cx': render.resolution_x / 2, 'cy': render.resolution_y / 2,
       'width': render.resolution_x, 'height': render.resolution_y,
   })
   ```
7. `finally` で KIRIモディファイヤ復元。
8. **原子的書き出し**（FR: 信頼性）: 出力ディレクトリを `os.makedirs(exist_ok=True)` した後、
   同一ディレクトリの一時ファイル（例: `<output_path>.tmp` または `tempfile.NamedTemporaryFile(dir=...)`）
   に `json.dump` → `flush`/close し、`os.replace(tmp_path, output_path)` で原子的に確定する。
   途中で例外が出た場合は一時ファイルを削除して再送出する（既存の有効な JSON を破壊しない）。
   feat-018 の NPZ→C3D と同方針。
9. `len(camera_data)` を返す。

**設計判断（原子的書き出し）**:
- 採用: 一時ファイル → `os.replace`。同一ファイルシステム内の rename は原子的で、書き込み中断・
  失敗時も既存 JSON が保たれる。
- 却下: `open(output_path, 'w')` で直接書く（既存 camera_pose.py の方式）。書き込み途中の中断で
  既存の有効 JSON を壊すため、信頼性要求を満たさない。

**設計判断（順序・評価タイミング）**:
- `frame_set` → `evaluated_depsgraph_get` → `arm.evaluated_get` の順を守る。非評価の
  `bpy.data.objects[...]` からボーンを読むと frame_set 前の古いポーズを読む恐れがあるため。
  （裏取り検証で評価済み経由が正しいフレーム位置を返すことを確認済み）
- `anchor.rotation_euler` 代入後に `view_layer.update()` を明示的に呼ぶ。裏取りで、これにより
  子カメラ `Cam_FPS.matrix_world` に代入した回転が厳密反映されることを確認済み。
- **アンカー回転の復元は行わない**: 本スクリプトは `-b` の別プロセスで動き .blend を保存しない
  ため、シーンへの副作用は永続しない。KIRIモディファイヤは表示状態の可視副作用があるため復元する
  （既存踏襲）が、アンカー回転はプロセス終了で破棄されるため復元不要とする。
  - 却下案: 各フレームでアンカー回転を退避・復元 → 保存しないプロセスでは無意味に複雑化するため却下。

**データフロー（出力スキーマ）**: 既存 `camera_pose.py` と完全同一。
各要素 `{frame:int, c2w:list[4][4] float, fx:float, fy:float, cx:float, cy:float, width:int, height:int}`。
`render.py` の `validate` が要求する必須フィールド（`frame,width,height,fx,fy,cx,cy,c2w`）をすべて含む。

### 1.4.3 `parse_args` / `default_output_path` / `extract_script_args`（FR-003）

- `extract_script_args(argv)`: 既存 camera_pose.py と同一（`--` 以降を返す。無ければ空リスト）。
- `default_output_path(camera_name)`: `os.path.join("data", f"{camera_name}_poses.json")`（同上）。
- `parse_args(argv)`:
  ```python
  parser = argparse.ArgumentParser(prog="fps_camera_pose.py",
      description="Blenderシーンから頭部追従カメラのポーズを書き出す")
  parser.add_argument("--camera", required=True, help="カメラオブジェクト名")
  parser.add_argument("--armature", default=DEFAULT_ARMATURE_NAME, help="アーマチュア名")
  parser.add_argument("--anchor", default=DEFAULT_ANCHOR_NAME, help="アンカーEmpty名")
  parser.add_argument("--output", default=None,
                      help="出力JSONパス (default: data/<カメラ名>_poses.json)")
  args = parser.parse_args(argv)
  if args.output is None:
      args.output = default_output_path(args.camera)
  return args
  ```

### 1.4.4 入力検証とエラー処理（FR-004）

すべて stderr 出力 + `sys.exit(1)`（既存 camera_pose.py のエラー様式に合わせる）。

| 検出条件 | メッセージ（stderr） |
|----------|----------------------|
| カメラが None | `エラー: オブジェクト '<name>' がシーンに見つかりません` + シーン内カメラ一覧 |
| カメラ type≠CAMERA | `エラー: オブジェクト '<name>' はカメラではありません (type=...)` |
| アーマチュアが None | `エラー: アーマチュア '<name>' がシーンに見つかりません` + シーン内アーマチュア一覧 |
| アーマチュア type≠ARMATURE | `エラー: オブジェクト '<name>' はアーマチュアではありません (type=...)` |
| アンカーが None | `エラー: アンカー '<name>' がシーンに見つかりません` |
| アンカー type≠EMPTY | `エラー: オブジェクト '<name>' は Empty ではありません (type=...)` |
| カメラの親≠アンカー | `エラー: カメラ '<cam>' の親がアンカー '<anchor>' ではありません (parent=...)` |
| カメラのローカル回転≠0 | `エラー: カメラ '<cam>' のローカル回転がゼロではありません (rotation_euler=...)` |
| 位置追従不整合（各フレーム） | `エラー: フレーム <frame>: カメラ位置が両目中点と一致しません (dist=<値> m)` |
| 向き不整合（各フレーム） | `エラー: フレーム <frame>: カメラの向きが計算値と一致しません (max_dev=<値>)` |
| ボーン不足 | `エラー: アーマチュア '<name>' に必要なボーンがありません: <不足名のカンマ区切り>` |
| frame_start > frame_end | `エラー: フレーム範囲が不正です (frame_start=... > frame_end=...)` |

**エラーハンドリング方針**: リトライやフォールバックは行わない（構成不正は即失敗が正しい）。
ログは stderr への print のみ（既存スクリプト群と同様、logging モジュールは使わない）。

### 1.4.5 `validate_rotation` / `_require_len`（FR-005）

**シグネチャ**:
```python
def validate_rotation(rot: "mathutils.Matrix", frame: int) -> None:   # 違反時 sys.exit(1)
def _require_len(vec: "mathutils.Vector", name: str, frame: int) -> None:  # 縮退時 sys.exit(1)
```

**`_require_len`**: `vec.length < EPS` なら
`エラー: フレーム <frame>: ベクトル <name> が縮退しています (length=<値>)` を stderr 出力 → `sys.exit(1)`。

**`validate_rotation`**:
- NaN/Inf: `rot` の全 9 成分に `math.isfinite` でないものがあれば
  `エラー: フレーム <frame>: 回転行列に NaN/Inf が含まれます` → `sys.exit(1)`。
- determinant: `abs(rot.determinant() - 1.0) > ORTHO_TOL` なら
  `エラー: フレーム <frame>: 回転行列の行列式が1ではありません (det=<値>)` → `sys.exit(1)`。
- 直交性: `rot @ rot.transposed()` と単位行列の各成分差の最大が `ORTHO_TOL` 超なら
  `エラー: フレーム <frame>: 回転行列が直交していません (max_dev=<値>)` → `sys.exit(1)`。

**`validate_camera_transform`**（FR-005(b)、フレームループ内で呼ぶ）:
```python
def validate_camera_transform(cam, rot_euler, eyes_mid, frame: int) -> None:  # 違反時 sys.exit(1)
```
- 位置整合: `(cam.matrix_world.translation - eyes_mid).length > POS_TOL` なら
  `エラー: フレーム <frame>: カメラ位置が両目中点と一致しません (dist=<値> m)` → `sys.exit(1)`。
- 回転整合: `cam.matrix_world.to_3x3()` と `rot_euler.to_matrix()` の各成分差の最大が `ORTHO_TOL` 超なら
  `エラー: フレーム <frame>: カメラの向きが計算値と一致しません (max_dev=<値>)` → `sys.exit(1)`
  （`matrix_parent_inverse` の回転成分やカメラローカル回転の混入を検出）。

**設計判断**: 検証は「JSON を書き出す前」に各フレームで行い、1フレームでも違反があれば
即 `sys.exit(1)` する（部分的な JSON も残さない）。フォールバック補正（直交化など）は行わない
——縮退・非直交は入力データ異常の兆候であり、黙って補正すると誤った `c2w` を隠すため。
位置・回転の整合性は「計算値が実際の `cam.matrix_world` に出ているか」を出力側で毎フレーム確認するもので、
親子変換・コンストレイントの実装方式に依存しない（構造 introspection を避けた堅牢な検証）。

### 1.5 状態遷移

ステートフル処理・GUI なし。該当なし。

### 1.6 ファイル・ディレクトリ設計

- 入力: `blender -b <blend>` で開くシーン（コマンドライン）。追加の入力ファイルなし。
- 出力: JSON。パス規約は既存 `camera_pose.py` と同一（`--output` 指定、省略時 `data/<カメラ名>_poses.json`）。
  出力ディレクトリが無ければ `os.makedirs(exist_ok=True)` で作成。
- JSON スキーマ: 1.4.2 記載（既存と同一）。

### 1.7 インターフェース定義

公開関数シグネチャ（すべて `fps_camera_pose.py`）:
```python
def extract_script_args(argv: list[str]) -> list[str]
def default_output_path(camera_name: str) -> str
def parse_args(argv: list[str]) -> argparse.Namespace
def compute_anchor_rotation_euler(arm_eval, frame: int) -> "mathutils.Euler"  # bpy 型注釈は文字列 or 省略
def validate_rotation(rot, frame: int) -> None
def validate_camera_transform(cam, rot_euler, eyes_mid, frame: int) -> None
def _require_len(vec, name: str, frame: int) -> None
def export_camera_poses(camera_name: str, armature_name: str, anchor_name: str, output_path: str) -> int
def main() -> None
```
- `compute_anchor_rotation_euler` の責務: 姿勢計算＋健全性検証（FR-005）。シーン取得・代入は行わない
  （テスト容易性のため分離。`arm_eval` を渡せば `bpy` シーンなしでも計算部を検証可能）。
- 呼び出し方向: `main` → `parse_args` / `export_camera_poses` →
  `compute_anchor_rotation_euler` → `validate_rotation` / `_require_len`。循環なし。

### 1.8 ログ・デバッグ設計

- 既存スクリプト群と統一し、`logging` は使わず `print` を用いる。
- 成功時（`main`）: `print(f"カメラポーズ書き出し完了: {output_path} ({num_frames} フレーム)")`（既存と同一文言）。
- エラー時: 1.4.4 のメッセージを `print(..., file=sys.stderr)` で出力し `sys.exit(1)`。
- デバッグ用の determinant/直交性出力は今回スコープ外（要求 Could）。

## 2. 検証計画（実装後）

CLAUDE.md「テスト」の方針に従い、以下を実施し結果を `tests/results/feat-019_test_result.txt` に保存する。
（`bpy` 依存のため pytest では `compute_anchor_rotation_euler` の純関数部分＋引数解析を対象とし、
統合検証は Blender 実行ログで確認する。）

1. **ヘッドレス出力**: `blender -b data/Blender/2D-Lift.blend --python fps_camera_pose.py -- --camera Cam_FPS`
   で JSON が出力され、フレーム数 = `frame_end - frame_start + 1` である。
2. **向きの追従**: 出力 JSON の複数フレームで `c2w` の回転成分が変化する（凍結していない）。
3. **GUI一致（最終判定基準）**: 対話GUIでハンドラ有効時の同フレーム `Cam_FPS.matrix_world` と、
   バッチ出力 `c2w` が一致する（同一計算のため厳密一致するはず）。裏取り時点で計算値
   f1=(1.364,0.064,0.088) 等を確認済み。
4. **エラー系**: 存在しないカメラ名／アーマチュア名を渡すと stderr にメッセージが出て終了コード 1。

## 3. 既存 `camera_pose.py` との差分要約

| 項目 | camera_pose.py | fps_camera_pose.py |
|------|----------------|--------------------|
| 姿勢の出所 | frame_set 後の `cam.matrix_world` をそのまま | frame_set 後に頭部から回転を計算しアンカーへ適用してから読む |
| 必須引数 | `--camera` | `--camera` |
| 追加引数 | なし | `--armature`, `--anchor` |
| 前提シーン | 姿勢がコンストレイント等で確定済み | アーマチュア＋アンカー＋子カメラ構成 |
| 出力スキーマ | 同一 | 同一 |

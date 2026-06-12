# 機能設計書: feat-012 camera_pose.py カメラ名・出力先のCLIオプション化

## 1. 変更箇所

- `phase4/camera_pose.py` — 全面リファクタリング（フラットスクリプト → 関数分割）
- `tests/test_feat012_camera_pose_args.py` — 新規（bpy 非依存ロジックのユニットテスト）
- `CLAUDE.md` — 完了時にスクリプト実行セクションの phase4 手順1を更新（「7. CLAUDE.md への影響」参照）

## 2. 現状のコード（ベースライン）

実装はコミット済みの版（カメラ名 `FPSCamera`）をベースとする。作業ツリーの未コミット変更（`cam41520554`）は本実装で置き換えられて消える。

```python
import bpy
import numpy as np
import json

scene = bpy.context.scene
cam = bpy.data.objects['FPSCamera']

# 3DGSオブジェクトのKIRIモディファイヤを一時無効化
kiri_objects = [obj for obj in scene.objects if 'KIRI_3DGS_Render_GN' in obj.modifiers]
for obj in kiri_objects:
    obj.modifiers['KIRI_3DGS_Render_GN'].show_viewport = False

camera_data = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    render = scene.render
    focal_px = (cam.data.lens / cam.data.sensor_width) * render.resolution_x
    camera_data.append({
        'frame': frame,
        'c2w': np.array(cam.matrix_world).tolist(),
        'fx': focal_px,
        'fy': focal_px,
        'cx': render.resolution_x / 2,
        'cy': render.resolution_y / 2,
        'width': render.resolution_x,
        'height': render.resolution_y,
    })

# モディファイヤを元に戻す
for obj in kiri_objects:
    obj.modifiers['KIRI_3DGS_Render_GN'].show_viewport = True

with open('/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/FPS-camera_poses.json', 'w') as f:
    json.dump(camera_data, f)
```

## 3. モジュール構成

bpy 非依存の純粋ロジック（引数処理）と bpy 依存処理（書き出し）を関数で分離する。
**`import bpy` はモジュールトップに置かず、bpy 依存関数の内部で行う。** これにより pytest 実行環境（Blender 外）から `camera_pose` モジュールを import してユニットテストできる。

```python
"""Blenderシーンからカメラポーズを書き出すスクリプト（Blender内実行用）

実行例:
    blender -b data/scene.blend --python camera_pose.py -- --camera cam41520554
"""
import argparse
import json
import os
import sys

import numpy as np

# KIRI Engine アドオンが3DGSオブジェクトに付与するモディファイヤ名（固定）
KIRI_MODIFIER_NAME = "KIRI_3DGS_Render_GN"


def extract_script_args(argv: list[str]) -> list[str]: ...
def default_output_path(camera_name: str) -> str: ...
def parse_args(argv: list[str]) -> argparse.Namespace: ...
def export_camera_poses(camera_name: str, output_path: str) -> int: ...
def main() -> None: ...


if __name__ == "__main__":
    main()
```

Blender の `--python` はスクリプトを `__main__` として実行するため、`if __name__ == "__main__":` ガードは Blender 内実行・pytest からの import の両方で正しく機能する。

## 4. 関数仕様

### 4.1 extract_script_args

```python
def extract_script_args(argv: list[str]) -> list[str]:
    """コマンドライン引数から '--' 以降のスクリプト引数を取り出す。

    Blender は '--' 以降の引数を無視するため、そこをスクリプト用に使う。
    '--' がない場合は空リストを返す。
    """
    if "--" in argv:
        return argv[argv.index("--") + 1:]
    return []
```

- 最初の `--` を区切りとする（`list.index` は最初の出現位置を返す）

### 4.2 default_output_path

```python
def default_output_path(camera_name: str) -> str:
    """デフォルトの出力JSONパス（カレントディレクトリ基準）を返す。"""
    return os.path.join("data", f"{camera_name}_poses.json")
```

### 4.3 parse_args

```python
def parse_args(argv: list[str]) -> argparse.Namespace:
    """スクリプト引数を解析する。--output 省略時はデフォルトパスを補完する。"""
    parser = argparse.ArgumentParser(
        prog="camera_pose.py",
        description="Blenderシーンからカメラポーズを書き出す",
    )
    parser.add_argument("--camera", required=True, help="カメラオブジェクト名")
    parser.add_argument("--output", default=None,
                        help="出力JSONパス (default: data/<カメラ名>_poses.json)")
    args = parser.parse_args(argv)
    if args.output is None:
        args.output = default_output_path(args.camera)
    return args
```

- `prog="camera_pose.py"` を明示する理由: Blender 経由実行時は `sys.argv[0]` が `blender` になり、usage 表示が紛らわしくなるため
- `--camera` 未指定時は argparse が usage を表示して `SystemExit`（終了コード2）

### 4.4 export_camera_poses

```python
def export_camera_poses(camera_name: str, output_path: str) -> int:
    """カメラポーズを全フレーム分JSONに書き出し、フレーム数を返す。"""
    import bpy  # Blender内でのみ利用可能（pytest環境でのimportエラー回避のため関数内import）

    scene = bpy.context.scene
    # 検索範囲は現在のシーン（bpy.data.objects だと他シーンのオブジェクトも拾い、
    # frame_set のデプスグラフ評価対象外の古いポーズを黙って書き出す恐れがある）
    cam = scene.objects.get(camera_name)
    cameras = [obj.name for obj in scene.objects if obj.type == 'CAMERA']
    if cam is None:
        print(f"エラー: オブジェクト '{camera_name}' がシーンに見つかりません", file=sys.stderr)
        print(f"シーン内のカメラ: {cameras}", file=sys.stderr)
        sys.exit(1)
    if cam.type != 'CAMERA':
        print(f"エラー: オブジェクト '{camera_name}' はカメラではありません (type={cam.type})", file=sys.stderr)
        print(f"シーン内のカメラ: {cameras}", file=sys.stderr)
        sys.exit(1)

    if scene.frame_start > scene.frame_end:
        print(f"エラー: フレーム範囲が不正です (frame_start={scene.frame_start} > frame_end={scene.frame_end})",
              file=sys.stderr)
        sys.exit(1)

    # 3DGSオブジェクトのKIRIモディファイヤを一時無効化（元の表示状態を記録して復元する）
    kiri_modifiers = [obj.modifiers[KIRI_MODIFIER_NAME]
                      for obj in scene.objects if KIRI_MODIFIER_NAME in obj.modifiers]
    original_states = [mod.show_viewport for mod in kiri_modifiers]
    for mod in kiri_modifiers:
        mod.show_viewport = False

    try:
        camera_data = []
        for frame in range(scene.frame_start, scene.frame_end + 1):
            scene.frame_set(frame)
            render = scene.render
            focal_px = (cam.data.lens / cam.data.sensor_width) * render.resolution_x
            camera_data.append({
                'frame': frame,
                'c2w': np.array(cam.matrix_world).tolist(),
                'fx': focal_px,
                'fy': focal_px,
                'cx': render.resolution_x / 2,
                'cy': render.resolution_y / 2,
                'width': render.resolution_x,
                'height': render.resolution_y,
            })
    finally:
        # 例外発生時もモディファイヤの表示状態を元に戻す
        for mod, state in zip(kiri_modifiers, original_states):
            mod.show_viewport = state

    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(camera_data, f)
    return len(camera_data)
```

- ポーズ書き出しループ・焦点距離計算・JSONフィールドは現状のコードと同一（変更しない）
- 現状からの改善: モディファイヤ復元を try/finally 化、復元値を「元の値」に変更（要求 3.5）
- `os.path.dirname` が空文字（ファイル名のみ指定）の場合は makedirs をスキップする

### 4.5 main

```python
def main() -> None:
    args = parse_args(extract_script_args(sys.argv))
    num_frames = export_camera_poses(args.camera, args.output)
    print(f"カメラポーズ書き出し完了: {args.output} ({num_frames} フレーム)")
```

## 5. エラー処理まとめ

| ケース | 動作 | 終了コード |
|--------|------|-----------|
| `--camera` 未指定（`--` なし含む） | argparse が usage 表示 | 2 |
| 指定オブジェクトがシーンに存在しない | エラーメッセージ + シーン内カメラ一覧（stderr） | 1 |
| 指定オブジェクトがカメラ型でない | エラーメッセージ + シーン内カメラ一覧（stderr） | 1 |
| `frame_start > frame_end`（書き出し0件） | エラーメッセージ（stderr）。空JSONは出力しない | 1 |

エラーメッセージは stderr に出力する（Blender のログが混ざる stdout と区別するため。argparse のエラーも stderr に出るので整合する）。

備考: Blender のバックグラウンドモード（`-b`）では、`--python` スクリプト内の `sys.exit(N)`（argparse の SystemExit 含む）はそのまま Blender プロセスの終了コードになる。一方、捕捉されない一般の例外では Blender はトレースバックを表示するだけで終了コード0のまま終わる（`--python-exit-code` で変更可能だが、運用はユーザーに委ねる。要求 6 のスコープ外項目参照）。

## 6. テスト設計

### 6.1 ユニットテスト（pytest、bpy 非依存）

ファイル: `tests/test_feat012_camera_pose_args.py`

`phase4/` を `sys.path` に追加して `import camera_pose` する（既存テストが `phase0/` に対して行っているのと同じ方式）。`import bpy` は関数内に隔離されているため、Blender 外でも import 可能。モジュールトップの `import numpy` はルートの uv 環境に存在するため問題ない。

| # | テスト | 期待結果 |
|---|--------|---------|
| 1 | `extract_script_args(['blender', '-b', 'x.blend', '--python', 'camera_pose.py', '--', '--camera', 'cam1'])` | `['--camera', 'cam1']` |
| 2 | `extract_script_args(['blender', '-b', 'x.blend', '--python', 'camera_pose.py'])`（`--` なし） | `[]` |
| 3 | `extract_script_args([..., '--'])`（`--` が末尾） | `[]` |
| 3b | `extract_script_args(['blender', '--', '--camera', 'cam1', '--', 'x'])`（`--` が複数） | `['--camera', 'cam1', '--', 'x']`（最初の `--` を区切りとする仕様の固定） |
| 4 | `default_output_path('cam41520554')` | `os.path.join('data', 'cam41520554_poses.json')` |
| 5 | `parse_args(['--camera', 'cam1'])` | `camera == 'cam1'`, `output == default_output_path('cam1')` |
| 6 | `parse_args(['--camera', 'cam1', '--output', 'out/poses.json'])` | `output == 'out/poses.json'` |
| 7 | `parse_args([])`（`--camera` なし） | `SystemExit` かつ終了コード2（`pytest.raises(SystemExit)` で捕捉し `e.value.code == 2` を検証） |

- テスト実行はSubagent（Agentツール）を使う。コマンド: `uv run pytest -v`（リポジトリルートで実行）
- 結果は `tests/results/feat-012_test_result.txt` に保存する

### 6.2 手動テスト（実機 Blender、ユーザー実施）

`phase4/` ディレクトリで実行する。

| # | 手順 | 期待結果 |
|---|------|---------|
| 1 | `blender -b data/<実在blend> --python camera_pose.py -- --camera <実在カメラ名>` | `data/<カメラ名>_poses.json` が生成され、完了メッセージにフレーム数が表示される |
| 2 | 手順1のJSONを `render.py` に渡して `--dry-run` 実行（`TORCH_CUDA_ARCH_LIST="9.0+PTX"` を付けること。CLAUDE.md 参照） | 必須フィールド検証を通りレンダリングが走る |
| 3 | `--output data/custom_poses.json` を付けて実行 | 指定パスに生成される |
| 4 | `--camera 存在しない名前` で実行 | エラーメッセージとシーン内カメラ一覧が表示され、終了コード1 |
| 5 | `--` 以降なしで実行 | argparse の usage が表示される |

## 7. CLAUDE.md への影響（完了時に更新）

「スクリプト実行 > phase4」を以下のように更新する:

- 手順1のコマンド例を `blender -b data/FPS-camera.blend --python camera_pose.py -- --camera FPSCamera` に変更（デフォルト出力 `data/FPSCamera_poses.json`）
- 「前提: シーンに FPSCamera という名前のカメラが存在すること」を「`--camera` でカメラオブジェクト名を指定する（必須）」に変更
- 「出力先: ...（スクリプト内に絶対パスでハードコード）」を「出力先: `--output` で指定。省略時は `data/<カメラ名>_poses.json`」に変更
- **手順2・3（render.py の実行例）のJSONパスを `data/FPS-camera_poses.json` から `data/FPSCamera_poses.json` に変更する**（手順1のデフォルト出力と繋がるようにする。変更しないと手順1の出力と手順2の入力が食い違う）

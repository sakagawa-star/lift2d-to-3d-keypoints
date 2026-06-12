# 要求仕様書: feat-012 camera_pose.py カメラ名・出力先のCLIオプション化

## 1. 背景・目的

`phase4/camera_pose.py` は Blender シーンからカメラポーズを書き出すスクリプトだが、カメラオブジェクト名（`bpy.data.objects['FPSCamera']`）と出力JSONパス（マシン固有の絶対パス `/home/sakagawa/.../FPS-camera_poses.json`）がハードコードされている。対象カメラや blend ファイルが変わるたびにコード編集が必要で使いにくい。CLIオプションで指定できるようにする。

## 2. 追加するCLIオプション

Blender の `--python` スクリプトに引数を渡すため、コマンドラインの `--` 以降の引数を camera_pose.py が解析する。

| 引数 | 必須 | デフォルト | 説明 |
|------|------|-----------|------|
| `--camera` | Yes | なし | ポーズを書き出すカメラオブジェクト名 |
| `--output` | No | `data/<カメラ名>_poses.json` | 出力JSONパス（実行時カレントディレクトリ基準の相対パス、または絶対パス） |

## 3. 動作仕様

### 3.1 引数解析

- `blender -b <blendファイル> --python camera_pose.py -- --camera <名前> [--output <パス>]` 形式で実行する
- `sys.argv` の最初の `--` より後ろのみを解析対象とする。`--` がない場合は引数なしとして扱う
- `--camera` 未指定時は argparse の必須エラー（usage 表示）で終了する

### 3.2 カメラ検証

- カメラの検索範囲は**現在のシーン**（`scene.objects`）とする。blendファイル内の他シーンのオブジェクトは対象外
- 指定された名前のオブジェクトがシーンに存在しない場合、エラーメッセージとシーン内のカメラオブジェクト名一覧を表示し、終了コード1で終了する
- オブジェクトは存在するがカメラ型（`object.type == 'CAMERA'`）でない場合も同様にエラー終了する
- エラーメッセージは標準エラー出力（stderr）に表示する（Blenderのログが混ざる標準出力と区別するため）

### 3.3 出力

- `--output` 未指定時は `data/<カメラ名>_poses.json` に出力する（実行時カレントディレクトリ基準。`phase4/` で実行する運用のため `phase4/data/` 配下に出力される）
- 出力先ディレクトリが存在しない場合は作成する
- JSONの内容・形式は現状と同一とする。各フレームのフィールド: `frame`, `c2w`(4x4), `fx`, `fy`, `cx`, `cy`, `width`, `height`（`render.py` の `load_camera_json()` の必須フィールド検証を通ること）
- 書き出すフレーム数が0になる場合（`frame_start > frame_end`。Python APIからは設定可能）は、空のJSONを出力せずエラーメッセージを表示して終了コード1で終了する（空JSONは `render.py` 側でゼロ除算エラーになるため）
- 書き出し完了時に出力パスとフレーム数を表示する

### 3.4 既存動作の維持

- 書き出すフレーム範囲は現状どおり `scene.frame_start` 〜 `scene.frame_end`（オプション化しない）
- KIRIモディファイヤ（`KIRI_3DGS_Render_GN`）の一時無効化と復元は現状どおり行う。モディファイヤ名はオプション化せず、モジュール定数（UPPER_SNAKE_CASE）に切り出す
- 焦点距離・主点の計算式は現状のまま変更しない

### 3.5 改善点（現状の不具合の修正）

- 書き出し処理の途中で例外が発生した場合でも、KIRIモディファイヤの表示状態が元に戻ること（現状は途中エラーで無効化されたまま残る）
- モディファイヤの復元は「無効化前の元の値」に戻すこと（現状は無条件に `True` を設定するため、元から非表示だったモディファイヤを表示状態にしてしまう）

## 4. 使用例

```bash
cd phase4

# デフォルト出力先（data/cam41520554_poses.json）に書き出し
blender -b data/scene.blend --python camera_pose.py -- --camera cam41520554

# 出力先を指定して書き出し
blender -b data/FPS-camera.blend --python camera_pose.py -- --camera FPSCamera --output data/FPS-camera_poses.json
```

## 5. 品質基準

- `--camera` 未指定で実行するとエラー終了すること
- 存在しないカメラ名を指定すると、エラーメッセージ（シーン内カメラ一覧付き）を表示し非ゼロ終了コードで終了すること
- 出力JSONが `render.py` でそのまま読み込めること（必須フィールド検証を通る）
- bpy 非依存の純粋ロジック（引数切り出し・引数解析・デフォルト出力パス生成）のユニットテストを `tests/` に追加し、`uv run pytest -v` が全て成功すること
- bpy 依存部分（カメラ検証・ポーズ書き出し）は実機 Blender での手動テストで確認する

## 6. 対象外（スコープ外）

- フレーム範囲のオプション化（`render.py` の `--start-frame` / `--end-frame` で下流でフィルタ可能なため不要と判断）
- KIRIモディファイヤ名のオプション化（KIRI Engine アドオン固有の固定名のため、定数化のみ行う）
- 主点 cx, cy のレンズシフト（`shift_x` / `shift_y`）対応（現状どおり解像度中心固定）
- `sensor_fit` が VERTICAL / AUTO のカメラへの対応（現状どおり水平フィット前提の焦点距離計算。縦長解像度では fx, fy が不正確になる既知の制約）
- `render.resolution_percentage`（解像度パーセンテージ）の考慮（現状どおり `resolution_x` / `resolution_y` をそのまま使用。100%以外に設定されたシーンでは width / height / fx が実レンダリング解像度とずれる既知の制約）
- 複数カメラの一括書き出し
- スクリプト内の例外発生時に Blender プロセスの終了コードを非ゼロにする保証（`--python-exit-code` オプションの利用はユーザーの運用に委ねる。明示的なエラー検出は `sys.exit(1)` で対応する）

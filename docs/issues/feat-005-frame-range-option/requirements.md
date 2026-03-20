# 要求仕様書: feat-005 render.py フレーム範囲指定オプション

## 1. 背景・目的

render.py で全1792フレームをレンダリングすると時間がかかる。特定のフレーム範囲だけレンダリングしたい場合がある。

## 2. 追加するCLIオプション

| 引数 | 必須 | デフォルト | 説明 |
|------|------|-----------|------|
| `--start-frame` | No | なし（JSONの最初のフレームから） | レンダリング開始フレーム番号 |
| `--end-frame` | No | なし（JSONの最後のフレームまで） | レンダリング終了フレーム番号（この番号を含む） |

## 3. 動作仕様

- `--start-frame` と `--end-frame` はカメラポーズJSON内の `frame` フィールドの値で指定する
- `--start-frame` のみ指定: そのフレームからJSON末尾まで
- `--end-frame` のみ指定: JSON先頭からそのフレームまで
- 両方指定: `start-frame <= frame <= end-frame` の範囲
- 両方未指定: 全フレーム（従来動作）
- 範囲内に該当するフレームがない場合はエラーメッセージを表示して終了

## 4. 使用例

```bash
# フレーム100から200まで
uv run python render.py data/project.ply data/FPS-camera_poses.json --start-frame 100 --end-frame 200

# フレーム500以降すべて
uv run python render.py data/project.ply data/FPS-camera_poses.json --start-frame 500

# フレーム50まで
uv run python render.py data/project.ply data/FPS-camera_poses.json --end-frame 50
```

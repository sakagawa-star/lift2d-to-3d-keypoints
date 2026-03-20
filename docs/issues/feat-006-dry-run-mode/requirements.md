# 要求仕様書: feat-006 render.py ドライランモード

## 1. 背景・目的

render.py で大量フレームをレンダリングする際、PNG保存がボトルネックになっている。レンダリング処理のみ実行しファイル保存をスキップするモードを追加し、GPU性能の計測やレンダリングの動作確認に使えるようにする。

## 2. 追加するCLIオプション

| 引数 | 必須 | デフォルト | 説明 |
|------|------|-----------|------|
| `--dry-run` | No | off | レンダリングのみ実行し、PNG保存をスキップする |

## 3. 動作仕様

- `--dry-run` 指定時:
  - レンダリング処理（gsplat rasterization）は通常通り実行する
  - PNG保存処理をスキップする（GPU→CPU転送もスキップする）
  - 出力ディレクトリの作成をスキップする
  - 各フレームの進捗表示は通常通り行う
- `--dry-run` 未指定時: 従来動作（変更なし）
- 共通（両モード）:
  - 完了時に合計処理時間とフレームあたりの平均時間を表示する

## 4. 使用例

```bash
# 全フレームをドライラン（PNG保存なし）
uv run python render.py data/project.ply data/FPS-camera_poses.json --dry-run

# フレーム範囲指定と組み合わせ
uv run python render.py data/project.ply data/FPS-camera_poses.json --dry-run --start-frame 1 --end-frame 100
```

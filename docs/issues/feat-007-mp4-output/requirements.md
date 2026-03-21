# 要求仕様書: feat-007 render.py MP4ファイル保存機能

## 1. 背景・目的

render.py で大量フレームをレンダリングする際、PNG保存（圧縮+ディスクI/O）がボトルネックになっている。MP4動画としてフレームをストリーム書き出しすることで、PNG保存より高速な出力を実現する。特にNVENC（NVIDIAハードウェアエンコーダ）を活用できれば、CPU負荷を大幅に削減できる。

## 2. 追加するCLIオプション

| 引数 | 必須 | デフォルト | 説明 |
|------|------|-----------|------|
| `--mp4` | No | off | PNG連番の代わりにMP4動画として出力する |
| `--mp4-fps` | No | 30 | MP4のフレームレート |

## 3. 動作仕様

- `--mp4` 指定時:
  - PNG連番ではなく、1つのMP4ファイルとして出力する
  - 出力パス: `{output-dir}/output.mp4`
  - フレームレートは `--mp4-fps` で指定（デフォルト30fps）
  - エンコーダ: ffmpegを使用。NVENCが利用可能なら `h264_nvenc`、利用不可なら `libx264` にフォールバック
  - フレームは1枚ずつレンダリングしてffmpegにパイプで渡す（全フレームをメモリに保持しない）
  - 全フレームが同一解像度（width x height）であることを前提とする。不一致の場合はエラー終了する
  - 出力ディレクトリが存在しない場合は作成する
  - NVENCでエンコードに問題が出た場合は `libx264` フォールバックを使用する
- `--mp4` 未指定時: 従来動作（PNG連番出力、変更なし）
- `--mp4` と `--dry-run` の併用: `--dry-run` が優先（PNG保存もMP4保存もスキップ）

## 4. 依存パッケージ

- `ffmpeg`: システムにインストールされていること（Pythonパッケージ追加は不要、subprocessで呼び出す）

## 5. 使用例

```bash
# MP4出力（デフォルト30fps）
uv run python render.py data/project.ply data/FPS-camera_poses.json --mp4

# MP4出力（60fps）
uv run python render.py data/project.ply data/FPS-camera_poses.json --mp4 --mp4-fps 60

# フレーム範囲指定と組み合わせ
uv run python render.py data/project.ply data/FPS-camera_poses.json --mp4 --start-frame 1 --end-frame 100

# rotate-z90と組み合わせ
uv run python render.py data/project.ply data/FPS-camera_poses.json --mp4 --rotate-z90
```

## 6. スコープ外

- 音声トラックの追加
- 動画コーデックの選択（H.264固定）
- ビットレートの詳細制御

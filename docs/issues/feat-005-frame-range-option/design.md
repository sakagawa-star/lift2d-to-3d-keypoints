# 機能設計書: feat-005 render.py フレーム範囲指定オプション

## 1. 変更箇所

`phase4/render.py` の `main()` 関数のみ変更。

## 2. 実装

### 2.1 インポートの追加

```python
import sys
```

### 2.2 CLI引数の追加

```python
parser.add_argument("--start-frame", type=int, default=None,
                    help="レンダリング開始フレーム番号")
parser.add_argument("--end-frame", type=int, default=None,
                    help="レンダリング終了フレーム番号（この番号を含む）")
```

### 2.3 フレームフィルタリング

`load_camera_json()` の後、レンダリングループの前にフィルタリングを行う。

```python
frames = load_camera_json(args.camera_json)

# フレーム範囲でフィルタリング
if args.start_frame is not None or args.end_frame is not None:
    start = args.start_frame if args.start_frame is not None else -float("inf")
    end = args.end_frame if args.end_frame is not None else float("inf")
    frames = [f for f in frames if start <= f["frame"] <= end]
    if len(frames) == 0:
        print(f"エラー: 指定範囲 (start={args.start_frame}, end={args.end_frame}) に該当するフレームがありません")
        sys.exit(1)
```

### 2.4 進捗表示の変更

フィルタリング後のフレーム数で進捗を表示する（既存ロジックは `enumerate(frames)` を使っているため、フィルタリング後のリストに対してそのまま動作する）。

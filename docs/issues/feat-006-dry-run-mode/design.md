# 機能設計書: feat-006 render.py ドライランモード

## 1. 変更箇所

`phase4/render.py` の `main()` 関数のみ変更。

## 2. 実装

### 2.1 インポートの追加

```python
import time
```

### 2.2 CLI引数の追加

```python
parser.add_argument("--dry-run", action="store_true",
                    help="レンダリングのみ実行し、PNG保存をスキップする")
```

### 2.3 出力ディレクトリの作成

`--dry-run` 時は出力ディレクトリの作成をスキップする。

```python
if not args.dry_run:
    os.makedirs(args.output_dir, exist_ok=True)
```

### 2.4 バッチレンダリングループの変更

- dry-run時はPNG保存とGPU→CPU転送をスキップする
- dry-run時はループ前後に `torch.cuda.synchronize()` を呼び出し、GPU処理の完了を待ってから時間を計測する（CUDAは非同期実行のため、synchronizeなしでは正確な計測ができない）
- 処理時間表示は通常モード・dry-runモード両方で行う

```python
start_time = time.time()

with torch.no_grad():
    if args.dry_run:
        torch.cuda.synchronize()

    for i, frame in enumerate(frames):
        # ... 座標変換・レンダリング（変更なし）...

        # PNG保存（dry-run時はGPU→CPU転送もスキップ）
        if not args.dry_run:
            out_path = os.path.join(args.output_dir, f"frame_{frame['frame']:06d}.png")
            img_np = (image.cpu().numpy() * 255).astype(np.uint8)
            Image.fromarray(img_np).save(out_path)
            print(f"  [{i + 1}/{len(frames)}] {out_path}")
        else:
            print(f"  [{i + 1}/{len(frames)}] frame {frame['frame']} (dry-run)")

    if args.dry_run:
        torch.cuda.synchronize()

elapsed = time.time() - start_time
print(f"\n完了: {elapsed:.1f}秒 ({elapsed / len(frames):.3f}秒/フレーム)")
```

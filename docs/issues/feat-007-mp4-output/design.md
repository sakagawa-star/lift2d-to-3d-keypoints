# 機能設計書: feat-007 render.py MP4ファイル保存機能

## 1. 変更箇所

`phase4/render.py` の `main()` 関数のみ変更。新規関数の追加はなし。

## 2. 実装

### 2.1 インポートの追加

```python
import subprocess
import shutil
```

### 2.2 CLI引数の追加

```python
parser.add_argument("--mp4", action="store_true",
                    help="PNG連番の代わりにMP4動画として出力する")
parser.add_argument("--mp4-fps", type=int, default=30,
                    help="MP4のフレームレート (default: 30)")
```

### 2.3 ffmpegプロセスの起動

MP4モード時、**フレーム範囲フィルタリングの後、バッチレンダリングループの前**にffmpegをサブプロセスとして起動する。

```python
ffmpeg_proc = None
if args.mp4 and not args.dry_run:
    os.makedirs(args.output_dir, exist_ok=True)
    mp4_path = os.path.join(args.output_dir, "output.mp4")

    # ffmpegの存在確認
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin is None:
        print("エラー: ffmpegが見つかりません")
        sys.exit(1)

    # NVENCの利用可能性を確認
    result = subprocess.run(
        [ffmpeg_bin, "-hide_banner", "-encoders"],
        capture_output=True, text=True
    )
    use_nvenc = "h264_nvenc" in result.stdout
    encoder = "h264_nvenc" if use_nvenc else "libx264"
    print(f"MP4エンコーダ: {encoder}")

    # 全フレームの解像度が一致していることを検証
    width = frames[0]["width"]
    height = frames[0]["height"]
    for i, f in enumerate(frames):
        if f["width"] != width or f["height"] != height:
            print(f"エラー: フレーム {f['frame']} の解像度 ({f['width']}x{f['height']}) が"
                  f"最初のフレーム ({width}x{height}) と異なります")
            sys.exit(1)

    ffmpeg_cmd = [
        ffmpeg_bin,
        "-y",                          # 上書き許可
        "-f", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}",
        "-r", str(args.mp4_fps),
        "-i", "pipe:0",                # stdinから読み込み
        "-c:v", encoder,
        "-pix_fmt", "yuv420p",         # 互換性のため
        "-loglevel", "error",          # stderrへの出力を抑制
        mp4_path,
    ]
    ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
```

**注意**: NVENCで `yuv420p` のエンコードに問題が出た場合は、`libx264` フォールバックを使用する。

### 2.4 バッチレンダリングループの変更

MP4モード時はffmpegのstdinにフレームデータを書き込む。PNG保存の分岐に追加する。

```python
        # 出力（モード別）
        if args.dry_run:
            print(f"  [{i + 1}/{len(frames)}] frame {frame['frame']} (dry-run)")
        elif args.mp4:
            img_np = (image.cpu().numpy() * 255).astype(np.uint8)
            ffmpeg_proc.stdin.write(img_np.tobytes())
            print(f"  [{i + 1}/{len(frames)}] frame {frame['frame']} -> mp4")
        else:
            out_path = os.path.join(args.output_dir, f"frame_{frame['frame']:06d}.png")
            img_np = (image.cpu().numpy() * 255).astype(np.uint8)
            Image.fromarray(img_np).save(out_path)
            print(f"  [{i + 1}/{len(frames)}] {out_path}")
```

### 2.5 ffmpegプロセスの終了

レンダリングループ完了後にffmpegのstdinを閉じ、プロセスの終了を待つ。

```python
if ffmpeg_proc is not None:
    ffmpeg_proc.stdin.close()
    ffmpeg_proc.wait()
    if ffmpeg_proc.returncode != 0:
        stderr = ffmpeg_proc.stderr.read().decode()
        print(f"ffmpegエラー:\n{stderr}")
        sys.exit(1)
    print(f"MP4保存: {mp4_path}")
```

### 2.6 出力ディレクトリの作成

MP4モード時は2.3で作成するため、既存の `os.makedirs` の条件を変更する。

```python
# 出力ディレクトリ作成
if not args.dry_run and not args.mp4:
    os.makedirs(args.output_dir, exist_ok=True)
```

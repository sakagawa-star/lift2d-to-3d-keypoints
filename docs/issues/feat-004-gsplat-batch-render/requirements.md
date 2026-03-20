# 要求仕様書: feat-004 gsplatバッチレンダリングパイプライン

## 1. 背景・目的

KIRI Engine Blenderアドオンで3DGSシーンのカメラ映像をレンダリングしているが、CPUボトルネックにより遅い。gsplat（CUDA）を用いたバッチレンダリングスクリプトで置き換え、高速化する。

## 2. 入力

### 2.1 PLYファイル（3DGSシーン）

3D Gaussian Splattingの学習済みモデル。以下の属性を含む：

| 属性 | 説明 |
|------|------|
| `x`, `y`, `z` | ガウシアン中心座標（means） |
| `f_dc_0`, `f_dc_1`, `f_dc_2` | 球面調和関数DC成分（色） |
| `f_rest_*` | 球面調和関数の高次成分（オプション、SH degree > 0の場合） |
| `opacity` | 不透明度（logit値、sigmoid変換が必要） |
| `scale_0`, `scale_1`, `scale_2` | スケール（log値、exp変換が必要） |
| `rot_0`, `rot_1`, `rot_2`, `rot_3` | 回転クォータニオン（正規化が必要） |

### 2.2 カメラポーズJSON

フレームごとのカメラパラメータを格納するJSONファイル。

```json
[
  {
    "frame": 1,
    "width": 1920,
    "height": 1080,
    "fx": 2133.33,
    "fy": 2133.33,
    "cx": 960.0,
    "cy": 540.0,
    "c2w": [
      [r00, r01, r02, tx],
      [r10, r11, r12, ty],
      [r20, r21, r22, tz],
      [0, 0, 0, 1]
    ]
  }
]
```

- トップレベルはフレームの配列（オブジェクトでラップしない）
- `frame`: フレーム番号（整数）
- `c2w`: Blenderカメラローカル座標系（+Y上、-Z前方）のcamera-to-world行列（4x4）
- 内部パラメータ（fx, fy, cx, cy）はフレームごとに指定可能

### 2.3 コマンドライン引数

```
uv run python render.py <ply_path> <camera_json_path> --output-dir <output_dir>
```

| 引数 | 必須 | デフォルト | 説明 |
|------|------|-----------|------|
| `ply_path` | Yes | - | 3DGS PLYファイルパス |
| `camera_json_path` | Yes | - | カメラポーズJSONパス |
| `--output-dir` | No | `./data/images` | 出力ディレクトリ |
| `--rotate-z90` | No | off | カメラポーズをワールドZ軸回りに+90度回転させる（下記参照） |

**`--rotate-z90` の使用条件**: KIRI Engine Blenderアドオンの旧バージョンでエクスポートしたカメラポーズが、3DGSのPLYファイルに対してワールドZ軸回りに90度ずれている場合に使用する。現行バージョンではバグが修正されており、このオプションは不要。回転方向が逆の場合は実装側で行列を転置して対応する。

## 3. 出力

- 出力ディレクトリに連番PNG画像を出力
- ファイル名: `frame_{frame:06d}.png`
- 画像サイズ: カメラポーズJSONの `width` x `height`

## 4. 座標系変換

Blender座標系とgsplat/OpenCV座標系の違いを吸収する。

カメラローカル座標系の比較:

| | Blender（カメラローカル） | gsplat/OpenCV（カメラローカル） |
|---|---------|---------------|
| 上 | +Y | -Y |
| 前方 | -Z | +Z |
| 右 | +X | +X |

変換行列（Blender camera-to-world → OpenCV camera-to-world）:

```
M_convert = [[1,  0,  0, 0],
             [0, -1,  0, 0],
             [0,  0, -1, 0],
             [0,  0,  0, 1]]

c2w_opencv = c2w_blender @ M_convert
```

gsplatのrasterizeにはworld-to-camera（view matrix）を渡すため、c2w_opencvの逆行列を使う。

## 5. PLY属性の変換処理

PLYファイルの生データに以下の活性化関数を適用する：

| 属性 | 変換 |
|------|------|
| `opacity` | `sigmoid(x)` |
| `scale_*` | `exp(x)` |
| `rot_*` | `normalize(quaternion)` |
| `f_dc_*`, `f_rest_*` | 生のSH係数のまま保持（gsplatが内部でSH→RGB変換を行う）。+0.5シフトの要否はStep 2で検証する |

## 6. スコープ外

- レンズ歪み補正（gsplatは理想ピンホールモデルでレンダリングする。歪みが必要な場合は後処理で対応）
- CPUフォールバック

## 7. 品質基準

- レンダリング結果がKIRI Engineの出力と視覚的に一致すること（目視確認）
- CUDA GPUが利用可能であること

## 8. 開発ステップ

### Step 1: PLYロードと基本確認
- PLYファイルを読み込み、各属性（means, scales, quats, opacities, colors）のshape・値域が正しいことをコンソール出力で確認する
- 活性化関数適用前後の値域を表示する

### Step 2: 固定カメラで1枚レンダリング
- ハードコードした仮カメラポーズで1枚レンダリングし、3DGSとして意味のある画が出ることを確認する
- 出力画像をPNGで保存する
- SH係数の+0.5シフトの要否を検証する（画が暗すぎる場合はシフトが必要）

### Step 3: カメラポーズJSONの読み込み
- カメラポーズJSONを読み込み、全フレームの連番PNGを出力する
- Blender→OpenCV座標系変換を適用する

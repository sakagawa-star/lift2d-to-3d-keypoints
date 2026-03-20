# 機能設計書: feat-004 gsplatバッチレンダリングパイプライン

## 1. ディレクトリ構成

```
phase4/
├── pyproject.toml          # uv パッケージ管理
├── render.py               # メインスクリプト（バッチレンダリング）
└── data/images/            # デフォルト出力ディレクトリ（gitignore、data/配下）
```

## 2. 依存パッケージ

```toml
[project]
name = "gsplat-batch-render"
requires-python = ">=3.10"
dependencies = [
    "torch>=2.0",
    "gsplat>=1.0.0,<2.0.0",
    "numpy",
    "plyfile",
    "Pillow",
]
```

**注意**: gsplatはバージョンによりAPI が大きく変わるため、メジャーバージョンを固定する。`torch` はgsplatが要求するCUDAバージョンと互換性があること。

- `gsplat`: 3D Gaussian Splattingのラスタライゼーション（CUDA）
- `plyfile`: PLYファイルの読み込み
- `Pillow`: PNG画像の保存
- `torch`: テンソル操作・CUDA

## 3. モジュール設計

`render.py` に全機能を実装する（単一ファイル構成）。

### 3.1 関数一覧

| 関数名 | 引数 | 戻り値 | 説明 |
|--------|------|--------|------|
| `load_ply(path)` | `str` | `dict[str, Tensor]` | PLYを読み込み、活性化関数適用済みテンソルを返す |
| `load_camera_json(path)` | `str` | `list[dict]` | カメラポーズJSONを読み込み、必須フィールドを検証する |
| `rotate_z90(c2w)` | `Tensor (4x4)` | `Tensor (4x4)` | c2wをワールドZ軸回りに90度回転（アドオンバグ補正） |
| `blender_to_opencv_c2w(c2w)` | `Tensor (4x4)` | `Tensor (4x4)` | Blender→OpenCV座標系変換 |
| `render_frame(gaussians, camera)` | `dict, dict` | `Tensor (H,W,3)` | 1フレームをレンダリング |
| `main()` | - | - | CLI引数パース、バッチレンダリングループ |

### 3.2 load_ply(path) の詳細

```python
def load_ply(path: str) -> dict[str, torch.Tensor]:
    """PLYファイルを読み込み、活性化関数適用済みテンソルを返す。

    Returns:
        {
            "means": Tensor (N, 3),       # ガウシアン中心座標
            "scales": Tensor (N, 3),      # exp(scale_raw)
            "quats": Tensor (N, 4),       # normalized quaternion
            "opacities": Tensor (N,),     # sigmoid(opacity_raw)
            "sh0": Tensor (N, 1, 3),      # DC成分 (SH degree 0)
            "sh_rest": Tensor (N, K, 3),  # 高次SH成分（存在する場合）
        }
    """
```

**PLY属性の読み込みと変換処理**:

1. `plyfile.PlyData.read(path)` でPLYを読み込む
2. 各属性を抽出:
   - `means`: `x`, `y`, `z` → そのまま
   - `scales`: `scale_0`, `scale_1`, `scale_2` → `torch.exp()`
   - `quats`: `rot_0`, `rot_1`, `rot_2`, `rot_3` → `F.normalize()`
   - `opacities`: `opacity` → `torch.sigmoid()`
   - `sh0`: `f_dc_0`, `f_dc_1`, `f_dc_2` → reshape to (N, 1, 3)
   - `sh_rest`: `f_rest_0` ~ `f_rest_*` → reshape to (N, K, 3)（Kは属性数/3）。`f_rest_*`属性が存在しない場合（SH degree 0）は `torch.zeros(N, 0, 3)` を返す

**注意**: `sh0`と`sh_rest`は生のSH係数のまま保持する（gsplatが内部でSH→RGBの変換を行う）。`sh_rest`が空テンソル `(N, 0, 3)` の場合、`sh_degree=0` となる。

### 3.3 load_camera_json(path) の詳細

```python
def load_camera_json(path: str) -> list[dict]:
    """カメラポーズJSONを読み込み、必須フィールドを検証する。

    JSONはフレームの配列（トップレベルがリスト）。
    必須フィールド（各フレーム）:
        frame (int), width (int), height (int),
        fx (float), fy (float), cx (float), cy (float),
        c2w (4x4 list)

    フィールドが欠損している場合はValueErrorを送出する。
    """
```

### 3.4 rotate_z90(c2w) の詳細

Blenderアドオン（KIRI Engine）の旧バージョンのバグにより、エクスポートされたカメラポーズが3DGSのPLYファイルに対してワールドZ軸回りに90度ずれている場合がある。この関数はそのずれを補正する。

- **ワールド座標系での回転**のため、c2wに対して**左乗算**する（右乗算はカメラローカル軸の回転になり目的と異なる）
- 回転方向: +90度（反時計回り、上から見て）。テストデータで検証し、逆方向（-90度）が必要な場合は行列を転置する

```python
def rotate_z90(c2w: torch.Tensor) -> torch.Tensor:
    """c2wをワールドZ軸回りに+90度回転させる（Blenderアドオンのバグ補正用）。

    ワールド座標系での回転のため左乗算: c2w_corrected = R_z90 @ c2w
    blender_to_opencv_c2w の右乗算（カメラローカル軸の変換）とは異なる。
    """
    r = torch.tensor([
        [0, -1, 0, 0],
        [1,  0, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=c2w.dtype)
    return r @ c2w
```

### 3.5 blender_to_opencv_c2w(c2w) の詳細

```python
def blender_to_opencv_c2w(c2w: torch.Tensor) -> torch.Tensor:
    """Blender座標系のcamera-to-worldをOpenCV座標系に変換する。

    Blender: +X右, +Y上, -Z前方
    OpenCV:  +X右, -Y上, +Z前方

    変換: Y軸とZ軸を反転（カメラのローカル軸に対して）
    c2w_opencv = c2w_blender @ M_convert
    M_convert = diag(1, -1, -1, 1)
    """
    convert = torch.tensor([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0, -1, 0],
        [0,  0,  0, 1],
    ], dtype=c2w.dtype)
    return c2w @ convert
```

### 3.6 render_frame(gaussians, camera) の詳細

gsplatの `rasterization` APIを使用してレンダリングする。

```python
from gsplat import rasterization

def render_frame(gaussians: dict, camera: dict) -> torch.Tensor:
    """1フレームをレンダリングする。

    Args:
        gaussians: load_ply()の戻り値
        camera: {
            "width": int,
            "height": int,
            "fx": float, "fy": float, "cx": float, "cy": float,
            "viewmat": Tensor (4x4),  # world-to-camera (OpenCV座標系)
            "background": Tensor (1, 3) or None,  # 背景色
        }

    Returns:
        Tensor (H, W, 3) float32 [0, 1]
    """
    # カメラ内部パラメータ行列
    K = torch.tensor([
        [camera["fx"], 0, camera["cx"]],
        [0, camera["fy"], camera["cy"]],
        [0, 0, 1],
    ], device="cuda", dtype=torch.float32)

    # gsplat rasterization呼び出し
    # viewmatは (1, 4, 4)、Kは (1, 3, 3) にバッチ次元追加
    # SH次数の算出: sh0(1) + sh_rest(K) の合計係数数から逆算
    # SH degree d に対し係数数 = (d+1)^2 → d = sqrt(total_coeffs) - 1
    total_sh_coeffs = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
    sh_degree = int(total_sh_coeffs ** 0.5) - 1

    rendered, alphas, meta = rasterization(
        means=gaussians["means"],
        quats=gaussians["quats"],
        scales=gaussians["scales"],
        opacities=gaussians["opacities"],
        colors=torch.cat([gaussians["sh0"], gaussians["sh_rest"]], dim=1),
        viewmats=camera["viewmat"].unsqueeze(0),
        Ks=K.unsqueeze(0),
        width=camera["width"],
        height=camera["height"],
        sh_degree=sh_degree,
        near_plane=0.01,
        far_plane=1e10,
        backgrounds=camera.get("background"),  # (1, 3) or None
    )

    return rendered.squeeze(0).clamp(0, 1)  # (H, W, 3)
```

### 3.7 main() の詳細

```python
def main():
    parser = argparse.ArgumentParser(description="gsplat batch renderer")
    parser.add_argument("ply_path", help="3DGS PLY file")
    parser.add_argument("camera_json", help="Camera poses JSON")
    parser.add_argument("--output-dir", default="./data/images", help="Output directory")
    parser.add_argument("--background", type=float, nargs=3, default=[0, 0, 0],
                        help="Background color RGB [0-1] (default: 0 0 0)")
    parser.add_argument("--rotate-z90", action="store_true",
                        help="カメラポーズをZ軸回りに90度回転（Blenderアドオンのバグ補正用）")
    args = parser.parse_args()

    # 背景色テンソル
    background = torch.tensor([args.background], dtype=torch.float32, device="cuda")

    # PLY読み込み（CUDA転送）
    gaussians = load_ply(args.ply_path)

    # カメラポーズ読み込み
    frames = load_camera_json(args.camera_json)

    # 出力ディレクトリ作成
    os.makedirs(args.output_dir, exist_ok=True)

    # バッチレンダリング
    for frame in frames:
        # Blender→OpenCV座標変換
        c2w_blender = torch.tensor(frame["c2w"], dtype=torch.float32)
        if args.rotate_z90:
            c2w_blender = rotate_z90(c2w_blender)
        c2w_opencv = blender_to_opencv_c2w(c2w_blender)
        viewmat = torch.inverse(c2w_opencv).cuda()

        camera = {
            "width": frame["width"],
            "height": frame["height"],
            "fx": frame["fx"], "fy": frame["fy"],
            "cx": frame["cx"], "cy": frame["cy"],
            "viewmat": viewmat,
            "background": background,
        }

        # レンダリング
        image = render_frame(gaussians, camera)

        # PNG保存
        img_np = (image.cpu().numpy() * 255).astype(np.uint8)
        Image.fromarray(img_np).save(
            os.path.join(args.output_dir, f"frame_{frame['frame']:06d}.png")
        )
```

## 4. 開発ステップごとの実装範囲

### Step 1: PLYロードと基本確認

**実装する関数**: `load_ply()`

**確認出力の内容**:
```
=== PLY Load Summary ===
File: scene.ply
Num Gaussians: 1234567
means:      shape=(1234567, 3)   min=-10.23  max=15.67
scales:     shape=(1234567, 3)   min=0.001   max=2.34   (after exp)
quats:      shape=(1234567, 4)   min=-1.0    max=1.0    (normalized)
opacities:  shape=(1234567,)     min=0.001   max=0.999  (after sigmoid)
sh0:        shape=(1234567, 1, 3)
sh_rest:    shape=(1234567, 15, 3)  (SH degree 3)
```

**テスト**: 実際のPLYファイルで実行し、shape・値域が妥当であることを目視確認する。

### Step 2: 固定カメラで1枚レンダリング

**実装する関数**: `render_frame()` + mainの簡易版

**仮カメラポーズ**:
- 原点から少し離れた位置にカメラを配置（シーンの中心を向く）
- 具体的な値は実際のシーンに合わせて調整する

**確認方法**: 出力PNGが3DGSとして意味のある画であること（黒一色やノイズではないこと）を目視確認する。

### Step 3: カメラポーズJSONの読み込み

**実装する関数**: `load_camera_json()`, `blender_to_opencv_c2w()`, `main()`の完成版

**確認方法**: 複数フレームの連番PNGが出力され、カメラの動きに応じてビューが変化していることを目視確認する。

# 機能設計書: feat-015 ピンホール3DGSレンダリング（PNG出力）

## 1. 対象ファイル

`phase4/render_keypoints.py` を**3DGSレンダリングのみに作り直す**（中止した feat-013 の実装＝現在の `render_keypoints.py` の機能を削除し縮小）。

- **残す関数**: `load_cameras_toml`, `select_camera`, `camera_to_viewmat`
- **削除**: `HALPE26_NAMES`/`HALPE26_SKELETON`/`NAME_TO_IDX`/色定数, `distortion_to_gsplat`, `load_c3d_frames`, `c3d_to_calib`, `extract_halpe26`, `project_keypoints`, `draw_overlay`, 旧 `render_background`, MP4/ffmpeg・フレーム範囲・dry-run・`--no-keypoints` 関連
- **追加**: `render_image`（gsplatピンホールレンダリング）
- `main` を「PLY+TOML+カメラ → PNG 1枚」に書き直す

**テスト**: `tests/test_feat015_render.py` を新規作成（純粋ロジック）。既存 `tests/test_feat013_render_keypoints.py` は削除した関数（`distortion_to_gsplat`, `extract_halpe26` 等）を import しており残すと collect 失敗するため、**削除する**（キーポイント・オクルージョンのテストは feat-016 で新規作成）。テスト結果は `tests/results/feat-015_test_result.txt`。

**依存**: `opencv-python`, `tomli` を使用。`c3d` は本ステップ未使用だが pyproject からは外さない（feat-016 で使う）。`render.py` の `load_ply`, `print_ply_summary` を再利用。

## 2. モジュール構成

```python
"""ピンホールカメラによる3DGS点群レンダリング（PNG出力）"""
import argparse, os, sys, time
import numpy as np
import cv2          # ルートvenv(pytest)にもある → top-level
import tomli        # 同上
# torch/gsplat と render.load_ply は関数内 import（純粋ロジックをルートvenvのpytestでテストするため。feat-013と同方針）
```

定数なし（Halpe26等は削除）。

## 3. 関数仕様

### 3.1 `load_cameras_toml(toml_path: str) -> dict[str, dict]`（既存維持）
- `tomli.load` し、`matrix` キーを持つテーブルのみカメラとして `{"name","K","D","rvec","tvec","width","height"}` で返す
- `D`（distortions）は読むがレンダリングでは未使用（ピンホール）。feat-016 で使うため保持
- bpy/CUDA非依存 → テスト対象

### 3.2 `select_camera(cameras: dict, name: str) -> dict`（既存維持）
- `name` 不在なら利用可能カメラ名一覧つき `ValueError` → main で捕捉し stderr・終了コード1
- テスト対象

### 3.3 `camera_to_viewmat(cam: dict) -> np.ndarray`（既存維持）
- `R, _ = cv2.Rodrigues(cam["rvec"])`、`viewmat = [[R, t],[0,0,0,1]]`（4x4 float32）
- OpenCV world-to-camera をそのまま gsplat viewmat とする
- テスト対象

### 3.4 `render_image(gaussians, cam, near_plane, background=(0.0,0.0,0.0)) -> np.ndarray`
- gsplat でピンホールレンダリングし、`(H,W,3) BGR uint8` を返す
- `torch`, `from gsplat import rasterization` を関数内 import
- **device/dtype は CUDA・float32 を明示する**（旧 `render_background` と同じ。`cam["K"]` は float64 numpy、`camera_to_viewmat` は CPU float32 を返すので、明示しないと gsplat の CUDAカーネルでデバイス不一致／dtype エラーになる）
- 実コード相当の snippet（**`render.py` の `render_frame` と同一の古典経路**。歪み・UT・深度なし）:
  ```python
  device = gaussians["means"].device
  viewmat = torch.from_numpy(camera_to_viewmat(cam)).to(device)          # float32 → CUDA
  K = torch.tensor(cam["K"], dtype=torch.float32, device=device)
  bg = torch.tensor(background, dtype=torch.float32, device=device)
  total_sh = gaussians["sh0"].shape[1] + gaussians["sh_rest"].shape[1]
  sh_degree = int(total_sh ** 0.5) - 1
  rendered, alphas, _ = rasterization(
      means=gaussians["means"], quats=gaussians["quats"], scales=gaussians["scales"],
      opacities=gaussians["opacities"],
      colors=torch.cat([gaussians["sh0"], gaussians["sh_rest"]], dim=1),
      viewmats=viewmat.unsqueeze(0), Ks=K.unsqueeze(0),
      width=cam["width"], height=cam["height"], sh_degree=sh_degree,
      near_plane=near_plane, far_plane=1e10,
      camera_model="pinhole", with_ut=False, packed=True)
  # camera_model="pinhole"・with_ut=False・packed=True は gsplat のデフォルト値（= render.py render_frame と同一の古典経路）
  # だが、gsplat のバージョン差異でデフォルトが変わっても古典ピンホール経路を保証するため、要求仕様 3.2 に合わせ明示的に渡す
  result = (rendered.squeeze(0) + (1 - alphas.squeeze(0)) * bg).clamp(0, 1)
  bgr = (result.cpu().numpy() * 255).astype(np.uint8)[:, :, ::-1].copy()  # RGB→BGR
  return bgr
  ```
- `rasterization` 呼び出しは `with torch.no_grad():` 下で行う（推論専用、メモリ削減。既存実装踏襲）
- CUDA依存 → 手動テスト

### 3.5 `main(argv=None)`
- テスト容易性のため `argv` 引数を受け `_build_parser().parse_args(argv)` に渡す（省略時は `sys.argv`）
- 処理順: 引数解析 → `load_cameras_toml` → `select_camera`（不在は stderr＋exit1）→ `from render import load_ply, print_ply_summary`（関数内）→ `load_ply`＋`print_ply_summary` → `render_image` → PNG保存（下記）
- **`from render import` は `select_camera` 成功後に置く**: ①誤ったカメラ名を torch/gsplat の重いロード前に早期検出できる ②`render.py` は `torch` を top-level import しておりルートvenv（pytest）では `import render` 自体が失敗する。`select_camera` を先に済ませれば、不在カメラの終了経路を render スタブ無しで非skipテストできる
- 出力ディレクトリ作成は**空dirnameをガード**する: `d = os.path.dirname(output); if d: os.makedirs(d, exist_ok=True)`（カレント直下出力時の `makedirs("")` 例外を防ぐ）
- **PNG保存は `cv2.imwrite` の戻り値チェックと例外捕捉の両方を行う**（要求仕様 3.4）。`cv2.imwrite` は権限不足・エンコード失敗時に `False` を返す一方、**不正な拡張子・拡張子なしの場合は `cv2.error` 例外を送出する**（OpenCV 4.13.0 で実挙動確認済み）。戻り値チェックだけだと例外で traceback クラッシュし、例外捕捉だけだと `False` を見逃すため、両方が必要:
  ```python
  try:
      ok = cv2.imwrite(output, bgr)
  except cv2.error as e:
      print(f"エラー: PNGの保存に失敗しました: {output}: {e}", file=sys.stderr)
      return 1
  if not ok:
      print(f"エラー: PNGの保存に失敗しました: {output}", file=sys.stderr)
      return 1
  ```
- 保存成功時に出力パスと所要時間を表示する
- `main()` は終了コードを `return` し、`sys.exit(main())` で呼ぶ（成功時 0、`select_camera` 不在・保存失敗時 1）

## 4. CLI 仕様

```
uv run python render_keypoints.py <ply> <toml> --camera <名前> [options]
```

| 引数 | 必須 | デフォルト | 説明 |
|---|---|---|---|
| `ply_path` | Yes | - | 3DGS PLYパス |
| `toml_path` | Yes | - | キャリブTOMLパス |
| `--camera` | Yes | - | TOML内の対象カメラ名 |
| `--near-plane` | No | 0.1 | near クリップ距離[m]（floater除去用） |
| `--output` | No | `./data/render_<カメラ名>.png` | 出力PNGパス |
| `--background` | No | `0 0 0` | 背景色RGB[0-1] |

実行例:
```bash
cd phase4
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    data/Blender/point_cloud.ply data/Blender/Config_scene.toml \
    --camera cam41520554 --near-plane 0.5 --output /tmp/render.png
```

## 5. 確定事項（検証済み）

- **viewmat=[R|t]**: キーポイントがGT病室のベッド上に正確に乗ることで座標系の正しさを確認済み
- **ピンホール古典経路**: `render.py` の `render_frame` と同一。歪みを gsplat のUT経路に乗せると黒い靄・品質劣化が出るため不使用
- **near_plane**: `0.01` だとカメラ至近1〜5cmの floater を描画し黒い靄＋深度汚染（depth median 0.028m）。`0.05〜0.5` で floater がクリップされ病室がGTと一致して鮮明（輝度138、深度 median 1.944m）。デフォルト `0.1`（floater を除去しつつ手前の物体は残すバランス）

## 6. エラー処理

| 状況 | 動作 |
|---|---|
| `--camera` がTOMLに無い | `select_camera` が `ValueError`（カメラ名一覧つき）→ main で捕捉し stderr表示、終了コード1 |
| `cv2.imwrite` が `False`（書き込み権限不足・エンコード失敗等） | stderrにエラー表示、終了コード1（PNG未生成を成功扱いしない。要求仕様 3.4） |
| `cv2.imwrite` が `cv2.error` 送出（不正な拡張子・拡張子なし等） | `except cv2.error` で捕捉し stderrにエラー表示、終了コード1（traceback クラッシュさせない。要求仕様 3.4） |
| PLY/TOMLファイル不在 | `load_ply`/`tomli.load` の標準例外に委譲（トレースバック表示。スコープ最小化のため独自処理しない） |

## 7. テスト設計

### 7.1 ユニット（純粋ロジック、bpy/CUDA非依存）— `tests/test_feat015_render.py`
合成データのみで完結。実データ（TOML）依存テストは `@pytest.mark.skipif(not Path(...).exists())` でスキップ。

- `camera_to_viewmat`: 既知 rvec/tvec で Rodrigues 整合・4x4・最下行 `[0,0,0,1]`・回転部分が直交（合成データ）
- `select_camera`（**合成データ・非skip**）: 合成 dict `{"camA": {...}}` で存在名は返す、不在名で `ValueError`（要求3.1のエラー終了経路を実データ無し環境でも回帰検出する）
- `load_cameras_toml` / `select_camera`（**実データ依存・skipif**）: 実データTOMLで5カメラ取得・各フィールド形状の確認
- **`main` のPNG保存失敗・`False`返却（合成データ・非skip）**: 合成TOMLを `tmp_path` に書き、`cv2.imwrite` を monkeypatch して `False` を返させ、`main(argv=[...])` が終了コード1を返し stderr にエラーメッセージを出すことを確認する（要求仕様 3.4 の回帰検出）。`render.py` は torch を top-level import しルートvenvで `import render` が失敗するため、`sys.modules["render"]` にダミーモジュール（`load_ply`/`print_ply_summary` スタブ）を仕込み、`render_keypoints.render_image` も monkeypatch でスタブ化して保存直前まで到達させる
- **`main` のPNG保存失敗・`cv2.error`送出（合成データ・非skip）**: 前項と同じスタブ構成で、`cv2.imwrite` を monkeypatch して `cv2.error` を送出させ、`main` が traceback でクラッシュせず終了コード1を返し stderr にエラーメッセージを出すことを確認する（不正拡張子等の回帰検出。OpenCV 4.13.0 で実挙動確認済み）
- **`main` の `--camera` 不在（合成データ・非skip）**: 合成TOMLを `tmp_path` に書き存在しないカメラ名を渡すと、`main(argv=[...])` が終了コード1を返し stderr に利用可能カメラ名一覧を出すことを確認する（要求3.1のmain終了経路を実データ無し環境で回帰検出。`select_camera` が `from render import` より前なので render スタブ不要）

### 7.2 手動テスト（実機 GPU、ユーザー実施）
- `--near-plane 0.5` でレンダリングし、GT（`0001_color.png`）と並べて病室として一致することを目視確認
- `--near-plane 0.01`（靄）と `0.5`（鮮明）を比較し、floater除去の効果を確認

## 8. CLAUDE.md への影響（完了時に更新）

- ディレクトリ構成: `render_keypoints.py` の役割（ピンホール3DGSレンダリング。feat-016 でキーポイント重ね描き・オクルージョンを追加予定）
- phase4 セクションに本スクリプトの実行例を追記

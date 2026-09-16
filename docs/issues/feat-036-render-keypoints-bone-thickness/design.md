# feat-036 機能設計書: render_keypoints.py ボーン線の太さ変更（視錐台ワイヤフレームは現状維持）

作成日: 2026-09-15
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-036-render-keypoints-bone-thickness/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 ボーン線の太さ変更 | 4.1, 4.2, 6（T-01, T-02, T-03） |
| FR-002 視錐台ワイヤフレームの太さ据え置き | 4.1, 4.3, 6（T-01, T-04, T-05） |
| FR-003 キーポイント円の据え置き | 4.1, 6（T-01） |
| FR-004 既存動作の不変 | 4.4, 6（T-06） |

## 2. システム構成

### モジュール構成

| ファイル | 変更内容 |
|---|---|
| `phase4/render_keypoints.py` | 定数 `LINE_THICKNESS` を削除し、`BONE_THICKNESS` と `FRUSTUM_THICKNESS` を新設。`draw_overlay()` の `cv2.line` 2か所・`draw_frustum()` の `cv2.line` 2か所と `draw_frustum()` の docstring 1行の参照先を差し替え |
| `tests/test_feat036_bone_thickness.py` | 新規テスト |

`render_fps_video.py`・他スクリプト・既存テストファイルは変更しない（リポジトリ内の `.py` で `LINE_THICKNESS` を参照するのは `phase4/render_keypoints.py` のみであることを 2026-09-15 に `grep` で確認済み）。

### 依存関係

変更なし。

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / OpenCV）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 定数（FR-001/002/003）

**変更前（HEAD コミット 7f45093、`phase4/render_keypoints.py:86-87`）**:

```python
POINT_RADIUS = 4
LINE_THICKNESS = 2
```

**作業ツリーの現状**: 起票時点で `LINE_THICKNESS = 20` にコミット外で書き換えられている（README.md「作業ツリーの状態」参照）。実装では、作業ツリーが HEAD の値（2）か書き換え後の値（20）かに関係なく、最終状態を下記「変更後」と一致させる。

**変更後**（同じ位置。`LINE_THICKNESS` の行を削除し、次の2行に置き換える。`POINT_RADIUS = 4` は変更しない）:

```python
POINT_RADIUS = 4
BONE_THICKNESS = 20           # ボーン線の太さ [px]（feat-036 で 2 → 20）
FRUSTUM_THICKNESS = 2         # FPSカメラ視錐台ワイヤフレームの線の太さ [px]（feat-036 でボーンと分離、値は従来どおり）
```

- `LINE_THICKNESS` という名前はモジュールから削除する（互換エイリアスは残さない。参照元が本ファイル内のみのため）
- `FRUSTUM_THICKNESS` は「FPSカメラ視錐台ワイヤフレーム（feat-034）」の定数群（`FRUSTUM_COLOR` 等、`phase4/render_keypoints.py:96-101`）ではなく、上記のとおり `POINT_RADIUS` の直後に置く（線の太さの定数を1か所に並べて見比べられるようにするため）

### 4.2 `draw_overlay()` の変更（FR-001）

`cv2.line` の第5引数 `LINE_THICKNESS` を `BONE_THICKNESS` に差し替える。対象は次の2か所のみ（行番号は HEAD 時点）:

| 位置 | 経路 | 変更前 | 変更後 |
|---|---|---|---|
| `phase4/render_keypoints.py:554` | `occlusion=False` | `cv2.line(img, pa, pb, color, LINE_THICKNESS)` | `cv2.line(img, pa, pb, color, BONE_THICKNESS)` |
| `phase4/render_keypoints.py:570` | `occlusion=True`（部分隠蔽） | `cv2.line(img, pa, pb, color, LINE_THICKNESS)` | `cv2.line(img, pa, pb, color, BONE_THICKNESS)` |

- `cv2.circle(img, center, POINT_RADIUS, POINT_COLOR, -1)`（`:579`）は変更しない
- `lineType` 引数は追加しない（既定の `cv2.LINE_8` のまま）
- docstring は変更しない（`LINE_THICKNESS` への言及がないため）

### 4.3 `draw_frustum()` の変更（FR-002）

| 位置 | 経路 | 変更前 | 変更後 |
|---|---|---|---|
| `phase4/render_keypoints.py:667`（docstring） | — | `色 FRUSTUM_COLOR、太さ LINE_THICKNESS。draw_overlay 自体は変更しない。` | `色 FRUSTUM_COLOR、太さ FRUSTUM_THICKNESS。`（feat-034 時点の「draw_overlay 自体は変更しない。」は、本案件で draw_overlay の定数参照を変更するため削除する。codex-02 の低指摘への対応） |
| `phase4/render_keypoints.py:689` | `occlusion=False` | `cv2.line(img, pa, pb, FRUSTUM_COLOR, LINE_THICKNESS)` | `cv2.line(img, pa, pb, FRUSTUM_COLOR, FRUSTUM_THICKNESS)` |
| `phase4/render_keypoints.py:704` | `occlusion=True`（部分隠蔽） | `cv2.line(img, pa, pb, FRUSTUM_COLOR, LINE_THICKNESS)` | `cv2.line(img, pa, pb, FRUSTUM_COLOR, FRUSTUM_THICKNESS)` |

### 4.4 変更しないもの（FR-004）

- CLI（`_build_parser`）、設定YAML（`CONFIG_CONVERTERS`）、`main()`、静止画モード（`_run_still_mode`）
- モジュール docstring（`LINE_THICKNESS` への言及がないため）
- 描画順（視錐台 → ボーン → キーポイント円）、色、`BONE_SAMPLES`、オクルージョン判定
- `README.md`（ボーン線・視錐台の太さの記載がないため更新不要。2026-09-15 に `grep` で確認済み）
- Closed 案件 feat-034 のドキュメント（`LINE_THICKNESS` の記述は当時の履歴としてそのまま残す）

### 4.5 エラーハンドリング

新たなエラー経路はない。

### 4.6 境界条件

- ボーン線の端点がともに画像外の場合の扱いは `cv2.line` の既存動作（クリッピング）のまま。太さ変更による新たな境界条件はない
- `pa == pb`（長さ0の線分）の場合、`cv2.line` は太さ 20 の点（直径約20pxの円）を描く。これは既存動作（太さ2の点）と同じ挙動の太さ違いであり、特別扱いしない

## 5. ログ・デバッグ設計

変更なし（出力メッセージの追加・変更はない）。

## 6. テスト設計

`tests/test_feat036_bone_thickness.py` を新規作成する。gsplat/torch を import しない範囲で合成データを使う（fixture の作り方は `tests/test_feat016_keypoints.py` の `TestDrawOverlay._setup` と `tests/test_feat034_fps_frustum.py` の `_frustum_test_verts` / `_frustum_test_cam` の流儀に従い、テストファイル内に同等の合成データ生成を持つ。既存テストファイルからの import はしない）。

`import` は既存テストと同じく `sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))` の後に `import render_keypoints` と `from render_keypoints import (...)` で行う。

### cv2.line 呼び出しの捕捉方法（T-02〜T-05 共通）

`pytest` の `monkeypatch` で `render_keypoints.cv2.line` を、元の `cv2.line` を呼んだうえで `thickness` 引数を記録するラッパーに差し替える。

```python
# 意図伝達用（そのままコピーしない）
recorded = []
orig_line = render_keypoints.cv2.line
def spy_line(img, pt1, pt2, color, thickness=1, *args, **kwargs):
    recorded.append(thickness)
    return orig_line(img, pt1, pt2, color, thickness, *args, **kwargs)
monkeypatch.setattr(render_keypoints.cv2, "line", spy_line)
```

各テストでは「`recorded` が空でない（1回以上呼ばれた）」ことと「`recorded` の全要素が期待値と等しい」ことの両方を検証する（呼び出しゼロで空集合の全称判定が真になる見逃しを防ぐため）。

| # | テスト | 入力 | 期待値 |
|---|---|---|---|
| T-01 | 定数値 | `render_keypoints` モジュール | `BONE_THICKNESS == 20`、`FRUSTUM_THICKNESS == 2`、`POINT_RADIUS == 4`、`hasattr(render_keypoints, "LINE_THICKNESS") is False` |
| T-02 | `draw_overlay` occlusion=False のボーン線太さ | 100x100 黒画像、28点を `TestDrawOverlay._setup` と同じ格子配置、`valid`・`kp_visible` 全 True、スケルトンは `build_skeleton(set(HALPE26_NAMES))`、`depth_map=None, alpha_map=None, margin=0.05, near_plane=0.1, occlusion=False` | `recorded` が空でなく、全要素が 20 |
| T-03 | `draw_overlay` occlusion=True のボーン線太さ | T-02 と同じ点・スケルトン、`depth_map = np.full((100,100), 100.0, float32)`、`alpha_map = np.zeros((100,100), float32)`（全サンプル可視）、`occlusion=True` | `recorded` が空でなく、全要素が 20 |
| T-04 | `draw_frustum` occlusion=False の線太さ | 200x100 黒画像、`_frustum_test_verts()` / `_frustum_test_cam(200, 100)` と同一の頂点・カメラ、`depth_map=None, alpha_map=None, margin=0.05, near_plane=0.1, occlusion=False` | `recorded` が空でなく、全要素が 2 |
| T-05 | `draw_frustum` occlusion=True の線太さ | T-04 と同じ頂点・カメラ、`depth_map = np.full((100,200), 100.0, float32)`、`alpha_map = np.ones((100,200), float32)`（全サンプル手前＝可視）、`occlusion=True` | `recorded` が空でなく、全要素が 2 |
| T-06 | 回帰: 既存テスト全件 | `uv run pytest -v` | 全件パス（本案件の追加分を含む） |

- テスト結果は `tests/results/feat-036_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する

### 実データでの動作確認（実装ステップ内で実施）

`phase4/` ディレクトリで、feat-033/034 の動作確認と同じ session001 実データを使う。設定YAML（`data/run_keypoints.yaml`）は `no_png: true` を含み PNG が出力されないため使わず、すべて CLI で指定する。フレーム範囲は NPZ の絶対 `frame_ids`（145599〜145898）で 5 フレームに絞る:

```bash
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply \
    /home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/Blender/handai-hosp1_20251024.toml \
    /home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/session001_f145749_world300_filtered.npz \
    --camera int_cam02_img --near-plane 0.5 \
    --fps-frustum --fps-toml /home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/Calib_FPSCamera.toml --fps-camera FPSCamera \
    --start-frame 145700 --end-frame 145704 \
    --output-dir /home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/feat036_check
```

（パス・カメラ名は 2026-09-15 時点の `phase4/data/run_keypoints.yaml` の値。いずれかのファイルが存在しない場合は想定外事象として中断・報告する）

確認項目:
1. 正常終了（終了コード 0）し、`data/feat036_check/frame_145700.png`〜`frame_145704.png` の 5 枚が出力される
2. 出力PNGを目視し、ボーン線が太く（20px）描かれ、マゼンタの視錐台ワイヤフレームが従来の細さ（2px）であることを確認する。確認に使った PNG のパスを報告に含める（視錐台がフレーム内に描かれない場合〔頭部点の無効・画角外〕はその旨を報告する）

## 7. 設計判断（ADR 簡易版）

- **D-1 定数を2つに分離し `LINE_THICKNESS` は削除**: ボーン用 `BONE_THICKNESS` と視錐台用 `FRUSTUM_THICKNESS` を新設する。`LINE_THICKNESS` をボーン用として残し視錐台用だけ新設する案は、名前が「全般の線の太さ」を示し用途が曖昧なまま残るため却下。互換エイリアスは参照元がファイル内のみのため不要
- **D-2 CLI/YAML 化しない**: ヒアリングで「定数の変更のみ」と確定（2026-09-15）。解像度・カメラごとの調整が必要になった場合は後続案件とする
- **D-3 キーポイント円は据え置き**: ヒアリングで確定（2026-09-15）。円（直径8px）がボーン線（20px）の幅の内側に収まる見た目上の影響は requirements.md「既知の見た目上の影響」で許容済み
- **D-4 テストは `cv2.line` の引数捕捉で行う**: 画素数や線幅の画像計測による検証は、`cv2.line` のラスタライズ仕様（丸い線端・LINE_8 の画素配置）に依存して期待値が不安定になるため却下。要求は「どの太さを渡すか」であり、引数の捕捉で過不足なく検証できる
- **D-5 実装は Sonnet サブエージェントに委任**: CLAUDE.md「実装の実行方法（Sonnetサブエージェント）」に従う

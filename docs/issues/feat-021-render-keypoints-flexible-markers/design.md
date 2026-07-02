# feat-021 機能設計書: render_keypoints.py 欠損マーカー許容（22点C3D対応）

## 1.1 対応要求マッピング

要求仕様書: `docs/issues/feat-021-render-keypoints-flexible-markers/requirements.md`

| 要求ID | 対応設計セクション |
|---|---|
| FR-001 欠損マーカー許容 | 1.4.1（定数拡張）, 1.4.2（extract_keypoints）, 1.4.4（main） |
| FR-002 ボーンの欠損スキップ | 1.4.3（build_skeleton） |
| FR-003 体幹ボーン拡張 | 1.4.3（build_skeleton の体幹規則） |
| FR-004 マーカー構成レポート | 1.4.4（main の起動時報告） |
| FR-005 最低マーカー数エラー | 1.4.2（extract 前検証）, 1.4.4（main のエラー経路）, 2.（main エラー経路テスト） |

既存テストの新API追随は 1.4.6 に記す。

## 1.2 システム構成

変更対象は以下の3ファイル（要求仕様 1.5 制約条件）。
モジュール構成・依存関係（`render.py` の `load_ply` を関数内 import する構造）は変更しない。

```
phase4/
└── render_keypoints.py                # 本案件で変更（定数・extract・スケルトン構築・main）
    └── render.py                      # 変更しない（load_ply, print_ply_summary を利用）
tests/
├── test_feat016_keypoints.py          # 本案件で変更（新APIへの追随。1.4.6）
└── test_feat021_flexible_markers.py   # 本案件で新規作成（2. テスト設計）
```

## 1.3 技術スタック

- Python 3.10 / uv（phase4 独立環境）
- 使用ライブラリ: numpy, opencv-python, tomli, c3d, torch, gsplat（すべて既存。追加なし）
- 実行時環境変数: `TORCH_CUDA_ARCH_LIST="9.0+PTX"`（既存のまま）

## 1.4 各機能の詳細設計

### 1.4.1 定数の拡張（FR-001）

`HALPE26_NAMES`（26名）は変更せず残し、描画対象の既知マーカー全体を表す定数を追加する:

```python
# 既知マーカー（描画対象）: Halpe26 の26点 + 体幹中間点2点。計28点
KEYPOINT_NAMES = HALPE26_NAMES + ["Spine", "Thorax"]
```

- `NAME_TO_IDX` は `KEYPOINT_NAMES` 基準に変更する（28名。先頭26要素の
  インデックスは `HALPE26_NAMES` と同一のため既存参照は互換）。
- 以降、配列サイズ 26 をハードコードしている箇所（`extract` の `(26,3)`・`(26,)`、
  `draw_overlay` の `range(len(HALPE26_NAMES))`、`main` の `np.ones(26)`）は
  すべて `len(KEYPOINT_NAMES)`（=28）基準に置き換える。

**設計判断（ADR）**: 「C3Dのラベルをそのまま全部描く」案は却下。未知ラベルは
スケルトン定義・左右色分けを持たず、描画規則を定義できないため、既知マーカー28名に
限定する（要求仕様 1.2「既知マーカー」）。

### 1.4.2 extract_keypoints（FR-001, FR-005）

`extract_halpe26` を `extract_keypoints` にリネームし、欠損マーカー許容にする。

**シグネチャ**:

```python
def extract_keypoints(
    labels: list[str], data: np.ndarray, residual: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
```

**入力**:
- `labels`: C3Dマーカー名リスト（strip済み、`load_c3d_all_frames` の戻り値）
- `data`: `(n_markers, 3)` float64、mm
- `residual`: `(n_markers,)` float64（< 0 は無効サンプル）

**出力**:
- `kpts`: `(28, 3)` float64、mm。欠損マーカーの行は `0.0` で埋める
  （`NaN` は使わない。`project_keypoints` の `nan_to_num` に依存せず決定的にする）
- `valid`: `(28,)` bool。欠損マーカーは常に `False`。存在するマーカーは
  既存と同じ規則（`residual >= 0` かつ座標に NaN なし）

**処理ロジック**（擬似コード）:

```
name_to_c3d_idx = {name: i for i, name in enumerate(labels)}
kpts = zeros((28, 3)); valid = zeros(28, bool)
for i, name in enumerate(KEYPOINT_NAMES):
    if name not in name_to_c3d_idx:
        continue                          # 欠損: kpts=0, valid=False のまま
    idx = name_to_c3d_idx[name]
    kpts[i] = data[idx]
    valid[i] = residual[idx] >= 0 and not isnan(data[idx]).any()
return kpts, valid
```

**エラーハンドリング**: 本関数は例外を投げない（欠損は valid=False で表現）。
既知マーカー0個の検証は main 側で行う（1.4.4。理由: 全フレームで同一の判定を
毎フレーム繰り返さないため、かつ FR-004 の報告と一箇所にまとめるため）。

**境界条件**:
- 既知マーカーが1個だけ存在: その点のみ描画され、ボーンは0本（build_skeleton が空を返す）
- 全マーカーが無効サンプルのフレーム: valid が全 False となり、そのフレームは背景のみのPNG
  （既存の無効サンプル時挙動と同じ）

### 1.4.3 build_skeleton（FR-002, FR-003）

モジュール定数 `HALPE26_SKELETON` を、体幹以外の固定部分と体幹の動的部分に分けて
構築する関数に置き換える。

**シグネチャ**:

```python
def build_skeleton(present: set[str]) -> list[tuple[str, str, str]]:
```

**入力**: `present` = C3Dラベルに存在する既知マーカー名の集合
（フレームごとの valid ではない。C3D全体で固定）。

**出力**: `(始点名, 終点名, 部位)` のリスト。両端点とも `present` に含まれるボーンのみ。

**処理ロジック**:

1. 体幹ボーンを以下の規則で決める（部位はすべて `"C"`）:
   - `Thorax ∈ present` かつ `Spine ∈ present`:
     `[("Neck","Thorax"), ("Thorax","Spine"), ("Spine","Hip")]`
   - `Thorax ∈ present` のみ: `[("Neck","Thorax"), ("Thorax","Hip")]`
   - `Spine ∈ present` のみ: `[("Neck","Spine"), ("Spine","Hip")]`
   - どちらも欠損: `[("Neck","Hip")]`（体幹フォールバックボーン = 従来定義）
2. ベーススケルトン（現行 `HALPE26_SKELETON` の25本）の `("Neck","Hip","C")`
   （リスト内3番目）を、手順1の体幹ボーン列で**同じ位置に**置き換える。
3. 結果リストから、両端点のいずれかが `present` に含まれないボーンを除外する。

**リスト内の位置を維持する理由（ADR）**: ボーンは先に描いたものの上に後のボーンが
重なるため、描画順が変わると出力画像が変わる。26点C3D（Spine/Thorax なし）で
変更前と同一の描画内容（FR-001 受け入れ基準2(a): スケルトンが `HALPE26_SKELETON`
と25本・同順で完全一致）を保証するには、体幹フォールバック時のリストが現行
`HALPE26_SKELETON` と完全に同一（順序含む）でなければならない。
末尾追加ではなく同位置置換とする。

**境界条件**:
- `Neck` または `Hip` が欠損: 体幹ボーンは手順3の除外規則で自然に消える
  （例: Neck 欠損なら Neck を端点とするボーンが全て除外される）。特別扱いしない。
- `present` が空: 空リストを返す（ただし main 側の FR-005 検証で先に終了するため、
  実行時には到達しない）。

**呼び出し側**: `draw_overlay` は `HALPE26_SKELETON`（モジュール定数）の参照を
引数 `skeleton: list[tuple[str, str, str]]` に置き換える。main が起動時に一度だけ
`build_skeleton` を呼び、全フレームで同じリストを渡す。

### 1.4.4 main の変更（FR-001, FR-004, FR-005）

C3D読み込み直後の検証ブロック（現行の `extract_halpe26(...)` 呼び出し、
render_keypoints.py:520-529 付近）を以下に置き換える:

```
labels, frames_data, point_rate = load_c3d_all_frames(...)   # 既存のまま
present = {name for name in KEYPOINT_NAMES if name in labels}
if not present:
    標準エラー出力: "C3Dに既知マーカーが1つもありません（labels: <labelsをカンマ区切り>）"
    return 1
missing = [name for name in KEYPOINT_NAMES if name not in present]
標準出力: "キーポイント: {len(present)}/{len(KEYPOINT_NAMES)} マーカーを描画対象とします"
if missing:
    標準出力: "欠損マーカー（描画スキップ）: {', '.join(missing)}"
skeleton = build_skeleton(present)
```

- 検証・報告は PLY 読み込み（`from render import load_ply` = torch ロード）より前に置く
  （FR-005 受け入れ基準。現行コードの検証位置と同じ）。
- フレームループ内: `extract_halpe26` 呼び出しを `extract_keypoints` に変更。
  no-occlusion 時の `kp_visible = np.ones(26, ...)` を `np.ones(len(KEYPOINT_NAMES), ...)` に変更。
  `draw_overlay` に `skeleton` を渡す。
- `missing` の列挙順は `KEYPOINT_NAMES` 順とする（実行ごとに順序が変わらない決定的出力）。

### 1.4.5 docstring・コメントの更新

- モジュール docstring・`c3d_path` の argparse help の「Halpe26」表記を
  「Halpe26 + Spine/Thorax（既知28マーカー。欠損許容）」の趣旨に更新する。
- 定数セクションのコメント（`render_keypoints.py:35` 付近）に既知マーカー28名と
  欠損許容の方針を1〜2行で記す。

### 1.4.6 既存テストの更新（tests/test_feat016_keypoints.py）

旧APIに依存する既存テストを新APIへ追随させる（テストの検証意図は変えない）:

1. import の `extract_halpe26` を `extract_keypoints` に変更する（該当行: 25行目付近）。
2. `extract_halpe26` を呼ぶ各テスト（82〜124行目付近）:
   - 呼び出しを `extract_keypoints` に変更する。
   - 戻り値 shape の期待を `(26, 3)`/`(26,)` から `(28, 3)`/`(28,)` に変更する
     （合成データは Halpe26 の26ラベルのみのため、追加2要素 = Spine/Thorax は
     `valid=False`・座標 `0.0` を期待する）。
   - 「マーカー不足で ValueError」を検証するテスト（124行目付近の
     `pytest.raises(ValueError)`）は、新仕様では例外を投げないため
     「欠損マーカーの valid が False になる」ことの検証に書き換える。
3. `draw_overlay` を呼ぶ各テスト（240〜269行目付近）: `cam` の直後に
   `skeleton` 引数を追加する。渡す値は `build_skeleton(set(HALPE26_NAMES))`
   （変更前と同一のスケルトンで従来の検証意図を維持）。
4. valid/kp_visible の配列を `np.ones(26)` 等で作っている箇所は
   `len(KEYPOINT_NAMES)`（=28）に合わせる。

## 1.5 状態遷移

該当なし（CLIバッチ処理。状態を持たない）。

## 1.6 ファイル・ディレクトリ設計

入出力ファイルのパス規約・命名規則（`frame_<C3Dフレーム番号:06d>.png`、
`output.mp4`、`--output-dir` 既定値）は一切変更しない。

## 1.7 インターフェース定義

**変更する公開関数**:

| 関数 | 変更内容 |
|---|---|
| `extract_halpe26(labels, data, residual)` | `extract_keypoints` にリネーム。戻り値 shape が (26,\*) → (28,\*)。欠損時 ValueError → valid=False |
| `draw_overlay(image, kpts_calib, pts2d, valid, kp_visible, cam, depth_map, alpha_map, margin, near_plane, occlusion=True)` | 引数 `skeleton: list[tuple[str, str, str]]` を `cam` の直後に追加。内部の `HALPE26_SKELETON`・`range(len(HALPE26_NAMES))` 参照を `skeleton`・`range(len(KEYPOINT_NAMES))` に変更 |

**追加する公開関数**:

| 関数 | シグネチャ |
|---|---|
| `build_skeleton` | `(present: set[str]) -> list[tuple[str, str, str]]` |

**変更しない関数**: `load_cameras_toml`, `select_camera`, `camera_to_viewmat`,
`render_image`, `load_c3d_all_frames`, `c3d_to_calib`, `project_keypoints`,
`compute_keypoint_depth`, `compute_visibility`, `start_ffmpeg`, `_build_parser`
（CLI 引数は変更しない）。

**削除する定数**: `HALPE26_SKELETON`（`build_skeleton` 内のベース定義に移す。
モジュールレベルには残さない — 残すと build_skeleton と二重定義になり不整合の温床になるため）。

呼び出し方向: main → build_skeleton / extract_keypoints / draw_overlay（一方向。循環なし）。

## 1.8 ログ・デバッグ設計

print ベースの既存方針を踏襲する（logging モジュールは導入しない）。

| 出力ポイント | 出力先 | 内容 |
|---|---|---|
| C3D読み込み直後（既知マーカー0個） | stderr | `C3Dに既知マーカーが1つもありません（labels: ...）` |
| C3D読み込み直後（正常） | stdout | `キーポイント: N/28 マーカーを描画対象とします` |
| 同上（欠損あり時のみ） | stdout | `欠損マーカー（描画スキップ）: ...` |

その他のログ出力は既存のまま変更しない。

## 2. テスト設計

`tests/test_feat021_flexible_markers.py` を新規作成する（GPU不要。純粋関数 +
monkeypatch による main の早期エラー経路のみ対象。レンダリング全体の確認は
手動テストで行う）。

**純粋関数テスト**:

- `test_extract_keypoints_22pt`: 22点C3D相当の labels/data/residual を合成し、
  足先6点の valid=False・存在22点の valid/座標が正しいこと（FR-001）
- `test_extract_keypoints_26pt`: Halpe26完備 labels で先頭26点の座標・valid が
  変更前 `extract_halpe26` の結果と同値（合成データに期待値をハードコード）、
  Spine/Thorax の valid=False であること（FR-001 受け入れ基準2(b)）
- `test_build_skeleton_full28`: 28点 present で体幹が Neck–Thorax–Spine–Hip の
  3本になり Neck–Hip がないこと（FR-003）
- `test_build_skeleton_22pt`: 22点 present で足先ボーン6本が除外されること（FR-002）
- `test_build_skeleton_halpe26`: Spine/Thorax なし26点 present で結果が
  変更前 `HALPE26_SKELETON`（25本・同順。テスト内に期待値リストをハードコード）と
  完全一致すること（FR-001 受け入れ基準2(a)）
- `test_build_skeleton_spine_only` / `test_build_skeleton_thorax_only`:
  片方のみの体幹規則（FR-003）

**main のエラー経路テスト（FR-005）**:

- `test_main_no_known_markers`:
  1. `monkeypatch.setattr` で `render_keypoints.load_c3d_all_frames` を、
     既知マーカーを含まない labels（例: `["Foo", "Bar"]`）と1フレームの
     ダミーデータを返す関数に差し替える
  2. `monkeypatch.delitem(sys.modules, "render", raising=False)` で
     render モジュールを未ロード状態にする
  3. 実在するTOML（テスト内で一時生成）と適当なダミーパス（PLY/C3D は
     モックにより実体不要）で `main([...])` を呼ぶ
  4. 検証: 戻り値が 1、capsys の stderr に
     `C3Dに既知マーカーが1つもありません` を含む、
     `"render" not in sys.modules`（= PLY/torch ロード前に終了した）

**手動テスト**:

- 22点C3D実データ（`session001_world_22pt_filtered.c3d`）で完走・目視確認
  （FR-001〜FR-004）
- 26点C3D実データが利用可能な場合のみ: 変更前後の出力PNGのバイト一致確認
  （FR-001 受け入れ基準2の補助確認。無い場合は省略可 — 必須基準は上記 pytest）

テスト結果は `tests/results/feat-021_test_result.txt` に保存する。

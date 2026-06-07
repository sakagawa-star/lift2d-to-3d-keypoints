# 機能設計書: feat-011 基準点番号ラベル表示オプション

## 1. 変更対象

- `phase0/visualize_points_2d.py` のみ（common.py は変更しない）
- テスト: `tests/test_feat011_point_labels.py` を新規作成

## 2. 現状の構造（前提知識）

`visualize_points_2d.py` の構成:

- `resolve_target_camera(config, camera_arg)` — 対象カメラ名決定
- `resolve_image_path(config, camera_name, config_dir)` — `image_<camera_name>` キーから画像パス解決
- `draw_points_on_image(image_path, points, output_path, config)` — 描画本体。`points` は `{ObjectName: [x, y]}` の dict（`common.load_points_2d` の戻り値）。各点について:
  ```python
  cv2.circle(img, (ix, iy), radius=8, color=(0, 0, 255), thickness=2)   # 赤円
  cv2.circle(img, (ix, iy), radius=1, color=(0, 255, 255), thickness=-1) # 黄中心点
  ```
- `main()` — argparse（`config` 位置引数、`--camera` オプション）→ 読み込み → 描画 → 結果表示

## 3. 設計

### 3.1 CLIオプション追加（main内）

```python
parser.add_argument(
    "--label",
    action="store_true",
    help="基準点の番号（ObjectNameの数字部分）をラベル表示する",
)
```

`draw_points_on_image` に `show_label: bool = False` 引数を追加し、`args.label` を渡す。

### 3.2 番号抽出関数（新規）

`import re` はファイル先頭のインポート群（既存の `import argparse` 等と同じ場所）に追加する。

```python
def extract_point_number(name: str) -> str:
    """ObjectNameから番号部分を抽出する

    最後に現れる連続数字列を返す（例: '基準_01' → '01'）。
    数字が含まれない場合は ObjectName 全体を返す。
    """
    matches = re.findall(r"\d+", name)
    return matches[-1] if matches else name
```

### 3.3 ラベル描画関数（新規）

```python
def draw_label(img, label: str, ix: int, iy: int, w: int, h: int) -> None:
    """点の近傍に番号ラベルを描画する（画像端でははみ出さない位置に調整）"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(label, font, scale, thickness)

    # 既定位置: 点の右上（マーカー半径8pxを避けるオフセット）
    tx = ix + 10
    ty = iy - 10
    # 右端ではみ出す場合は左側へ
    if tx + tw > w:
        tx = ix - 10 - tw
    # 上端ではみ出す場合は下側へ
    if ty - th < 0:
        ty = iy + 10 + th
    # 最終クランプ（左端・下端を含む全方向ではみ出さないことを保証）
    # MARGIN: 黒縁取り（thickness+2）とアンチエイリアスの滲みを見込んだ余白
    MARGIN = 3
    tx = max(MARGIN, min(tx, w - tw - MARGIN))
    ty = max(th + MARGIN, min(ty, h - baseline - MARGIN))

    # 縁取り（黒で太く）→ 本体（緑）の二度描きで視認性を確保
    cv2.putText(img, label, (tx, ty), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, label, (tx, ty), font, scale, (0, 255, 0), thickness, cv2.LINE_AA)
```

### 3.4 draw_points_on_image の変更

シグネチャ:

```python
def draw_points_on_image(
    image_path: Path,
    points: dict,
    output_path: Path,
    config: dict,
    show_label: bool = False,
) -> tuple[int, int]:
```

描画ループ内（既存の circle 2行の直後）:

```python
if show_label:
    draw_label(img, extract_point_number(name), ix, iy, w, h)
```

- 範囲外スキップの分岐より後に置くため、ラベルは画像範囲内の点にのみ付く
- `w`, `h` は既存の `h, w = img.shape[:2]`（関数冒頭）で定義済みのローカル変数を流用する。**`draw_label` の引数順は `(w, h)`** であり、`img.shape` の順序（`h, w`）と逆である点に注意

### 3.5 main の変更

```python
drawn, skipped = draw_points_on_image(
    image_path, points, output_path, config, show_label=args.label
)
```

結果表示に `ラベル表示: 有効/無効` の1行を追加する。

## 4. データ構造

変更なし。`points` dict（`{ObjectName: [x, y]}`）をそのまま使用し、キーから番号を抽出する。

## 5. テスト設計

`tests/test_feat011_point_labels.py`:

1. **extract_point_number のユニットテスト**
   - `基準_01` → `01`（先頭ゼロ保持）
   - `基準_12` → `12`
   - `pt3_cam01_05` → `05`（最後の数字列）
   - `marker` → `marker`（数字なし → 全体を返す）
2. **draw_points_on_image の動作テスト**（tmp_path に小さなテスト画像を生成して実行。**可逆圧縮のPNG形式を使用**すること——JPEGは非可逆のためピクセル一致比較が成立しない）
   - `show_label=False` 時の出力が従来実装と同一（ピクセル一致）であること
   - `show_label=True` 時の出力が `False` 時と異なる（ラベルが描画されている）こと
   - 画像の**4隅すべて**の付近の点でも例外なく出力されること。「ラベルが画像内に収まる」検証は厳密なピクセル判定ではなく**例外なく描画が完了し画像が出力されること**を基準とする（縁取り・アンチエイリアスの滲みによる数pxの誤差を許容）
   - 「従来実装と同一」の検証方法: 同一テスト内で circle 2行のみの参照描画を再現して比較する（git旧版の実行は不要）
3. テスト実行はSubagentで `uv run pytest -v` を実行し、結果を `tests/results/feat-011_test_result.txt` に保存する

## 6. 影響範囲

- `--label` 未指定時はコードパスが変わらないため、既存動作（feat-010、bug-001の挙動）に影響なし
- CLAUDE.md のディレクトリ構成更新は不要（既存ファイルの変更のみ。テストファイル追加は tests/ 配下）

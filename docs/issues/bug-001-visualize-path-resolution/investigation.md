# 調査・修正計画: bug-001 visualize_points_2d.py パス解決不整合

## 調査

### 既存スクリプトのパス解決方式

`phase0/estimate_camera_params.py` ではパスを config ファイルのあるディレクトリを基準に解決している（同様の処理が複数箇所で使われている）。

```python
config_dir = Path(config_path).parent
points_2d_path = config_dir / config['points_2d']
```

これにより、`phase0/data/config_osaka2.yaml` 内に `points_2d: points_2d_osaka2.csv` と書かれていれば、`phase0/data/points_2d_osaka2.csv` として解決される。

### 現在の visualize_points_2d.py

```python
points = load_points_2d(config['points_2d'], camera_name)  # CWD相対のまま
...
image_path = Path(config[key])  # CWD相対のまま
```

config 値をそのまま `Path` / `open` に渡しているため、CWD（phase0/）相対として解釈され、既存 config と互換性がない。

## 修正方針

`config_dir = Path(config_path).parent` を基準にパスを解決するよう修正する（既存スクリプトと完全に揃える）。

### 修正対象

`phase0/visualize_points_2d.py`

1. `main()` で `config_dir = Path(args.config).resolve().parent` を算出
2. `main()` 内のパス解決順序を以下に統一:
   - config 読込 → `camera_name` 解決 → **`config_dir` 算出** → `load_points_2d` 呼び出し（CSVパス解決）→ 0件チェック → `resolve_image_path(config, camera_name, config_dir)` → 描画
3. `points_2d` パス: `load_points_2d(str(config_dir / config['points_2d']), camera_name)`
4. `resolve_image_path()` のシグネチャを `(config, camera_name, config_dir)` に変更し、`config_dir / config[key]` で解決
5. 値が絶対パスの場合は `Path` の演算子の仕様により絶対パスがそのまま採用されるため、特別な分岐は不要

### テスト修正

`tests/test_visualize_points_2d.py`:

`resolve_image_path` のシグネチャ変更により以下の既存テスト3件は呼び出し形式の更新が必須（破壊的変更）：

- `test_resolve_image_path_ok` → `config_dir` 引数を渡す
- `test_resolve_image_path_missing_key_exits` → 同上
- `test_resolve_image_path_missing_file_exits` → 同上

加えて新規テストを追加:

- 相対パス（config 値が `"img.jpg"`）を `config_dir / "img.jpg"` として解決できること
- 絶対パス（config 値が絶対パス）が `config_dir` に依存せずそのまま採用されること（A案の正当性担保）

### ドキュメント修正

要求仕様書・機能設計書のうち「CWD（phase0/）からの相対」と書いている箇所を「config ファイルのあるディレクトリからの相対」に修正する。

- `requirements.md`: `image_<camera_name>` の説明箇所、設定例
- `design.md`: `resolve_image_path` 仕様、シーケンス、解決基準の記述

## 実装外の判断

- 既存 `config_osaka2.yaml` 等は書き換えない（互換性保持）
- 既存スクリプトの挙動は変更しない（visualize_points_2d.py を既存スクリプト側に揃える）

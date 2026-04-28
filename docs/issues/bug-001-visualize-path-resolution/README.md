# bug-001: visualize_points_2d.py のパス解決が既存スクリプトと不整合

## 概要

`visualize_points_2d.py` が `points_2d` および `image_<camera_name>` のパスをCWD（phase0/）相対として解決しているが、既存スクリプト（`estimate_camera_params.py` 等）は **config ファイルのあるディレクトリ** を基準に解決している。
そのため、既存の `config_osaka2.yaml` をそのまま使うとCSV/画像が見つからずエラーになる。

## ステータス

Closed

## 再現手順

```bash
cd phase0
uv run python visualize_points_2d.py data/config_osaka2.yaml --camera cam05520126
```

```
FileNotFoundError: [Errno 2] No such file or directory: 'points_2d_osaka2.csv'
```

`data/config_osaka2.yaml` には `points_2d: points_2d_osaka2.csv` と書かれており、
既存スクリプトは `data/points_2d_osaka2.csv` として解決するが、
`visualize_points_2d.py` は CWD 相対として解決してしまう。

## 影響範囲

- feat-010 の visualize_points_2d.py のパス解決ロジックのみ
- 既存 config_*.yaml には変更不要（既存記法のまま動くようにする）

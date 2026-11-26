# feat-044 実データでの確認記録（design.md §6.4）

実施日: 2026-11-26。作業ディレクトリ: `phase4/data/10F/feat044_check/`（`mkdir` で新規作成）。GPU 不使用。
`job_feat043.yaml`・`feat043_*` は未編集・未削除。

## 準備（手順 1・2）

`job.yaml`（`job_feat043.yaml` の写しから `expect_model`・`expect_focal_mm`・`tmp_dir` を削除し、`photo_info` を足し、
`images_dir` / `init_toml` / `points_2d` / `out_dir` を `feat044_check/` 配下に変更。`overwrite: true` は残した）:

```
camera: s25_20261124_152436
image: phase4/data/10F/s25_original/20261124_152436.jpg
points_2d_original: phase0/data/points_2d_10F.csv
overwrite: true
images_dir: phase4/data/10F/feat044_check/images
init_toml: phase4/data/10F/feat044_check/init.toml
points_2d: phase4/data/10F/feat044_check/points_2d.csv
points_3d: phase0/data/kijunten_locations_10F.csv
ply: phase4/data/10F/point_cloud.ply
out_dir: phase4/data/10F/feat044_check/out
photo_info: phase4/data/10F/feat044_check/photo_info.yaml
```

## 結果

| 手順 | コマンド（要旨） | 終了コード | 結果 |
|---|---|---|---|
| 3 | `uv run --project phase4 python phase4/extract_photo_info.py --config .../job.yaml` | 0 | `photo_info.yaml` が作られた |
| 4 | `uv run --project phase4 python phase4/prepare_phone_photo.py --config .../job.yaml` | 0 | 初期焦点距離 f = 1275.710654032817、2D 点 6 点 |
| 5 | `cmp` 3 件（PNG・init.toml・points_2d.csv を feat-043 の出力と比較） | 0 / 0 / 0 | バイト単位で一致 |
| 6.1 | cv2 で Exif なし `noexif.jpg` を作成、`len(Image.open(...).getexif())` | 0 / 出力 `0` | Exif は空 |
| 6.2 | `extract_photo_info.py --image noexif.jpg --photo-info noexif_photo_info.yaml` | 1 | `noexif_photo_info.yaml` は作られず、stderr に手で書く方法（書式）が出た |
| 6.3 | `noexif_photo_info.yaml` に `focal_length_35mm: 23` を手で書く | - | - |
| 6.4 | `prepare_phone_photo.py --image noexif.jpg --photo-info noexif_photo_info.yaml --camera ... --images-dir noexif_images --init-toml noexif_init.toml --points-2d-original phase0/data/points_2d_10F.csv --points-2d noexif_points_2d.csv` | 0 | 完走 |
| 6.5 | `cmp noexif_init.toml feat043_init.toml` / `cmp noexif_points_2d.csv feat043_points_2d.csv` | 0 / 0 | 一致（前処理後画像は比較しない） |
| 7 | 削除前の `job_feat043.yaml` を書き出し・前処理・推定に `--config` で渡す | 2 / 2 / 2 | いずれも「設定YAMLに未知のキーがあります: expect_model, expect_focal_mm」 |

`photo_info.yaml` の内容:

```
# 写真情報ファイル（extract_photo_info.py が Exif から書き出し。feat-044）
focal_length_35mm: 23
```

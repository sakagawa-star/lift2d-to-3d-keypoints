# feat-043 実データでの確認（design.md §6.2）

実施日: 2026-11-25。プロジェクトルートで実行。共用 YAML: `phase4/data/10F/job_feat043.yaml`（requirements.md §7 の例に `overwrite: true`、`tmp_dir` を加え、出力系を `feat043_*` に変更）。

## 事前確認

`job_feat043.yaml`、`feat043_images`、`feat043_init.toml`、`feat043_points_2d.csv`、`feat043_out`、`feat043_tmp`（いずれも `phase4/data/10F/`）が `test ! -e` かつ `test ! -L` を満たすことを確認（すべて不在）。

## 前処理

`uv run --project phase4 python phase4/prepare_phone_photo.py --config phase4/data/10F/job_feat043.yaml` → 終了コード 0（2D 点 6 点、f = 1275.710654032817）。

| 比較（`cmp`） | 結果 |
|---|---|
| `feat043_images/s25_20261124_152436.png` と `s25_images/s25_20261124_152436.png` | 一致（0） |
| `feat043_init.toml` と `s25_20261124_152436_init.toml` | 一致（0） |
| `feat043_points_2d.csv` と `points_2d_s25_20261124_152436.csv` | 一致（0） |

## 推定

`TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_intrinsics_loop.py --config phase4/data/10F/job_feat043.yaml` → 終了コード 0（`overwrite` などの前処理のキーは無視されて完走）。

| 比較 | 結果 |
|---|---|
| `cmp feat043_out/s25_20261124_152436_loop.toml feat041_out_it2/...` | 一致（0） |
| `cmp feat043_out/s25_20261124_152436_final.toml feat041_out_it2/...` | 一致（0） |
| loop_report の反復行（`i=` で始まる行 5 行、末尾の所要時間を除く）の `diff` | 差なし |
| eval.txt の `median_raw`・`valid_ratio_raw`・判定の `diff` | 差なし |

比較した反復行（feat043_out 側。feat041_out_it2 側と同一）:

```
i=0 | MASt3R マッチ2151 N=1996 inl=1650 RMSE=2.351px | f=1273.86 cx=957.10 cy=537.62 k1=0.0500 k2=-0.0421 | Δ=17.413px | 被覆16/16 | lsq_status=2
i=1 | LoFTR マッチ6939 N=6046 inl=4461 RMSE=1.373px | f=1278.24 cx=960.04 cy=539.19 k1=0.0722 k2=-0.0596 | Δ=14.655px | 被覆15/16 | lsq_status=2
i=2 | LoFTR マッチ7172 N=6271 inl=5687 RMSE=1.288px | f=1280.03 cx=961.22 cy=540.47 k1=0.0764 k2=-0.0596 | Δ=6.684px | 被覆16/16 | lsq_status=2
i=3 | LoFTR マッチ7211 N=6314 inl=5853 RMSE=1.241px | f=1280.70 cx=961.53 cy=540.90 k1=0.0784 k2=-0.0606 | Δ=2.143px | 被覆16/16 | lsq_status=2
i=4 | LoFTR マッチ7218 N=6332 inl=5519 RMSE=1.230px | f=1279.99 cx=960.53 cy=541.23 k1=0.0781 k2=-0.0604 | Δ=1.872px | 被覆16/16 | lsq_status=2
```

eval.txt: `valid_ratio_raw: 0.20911892361111112`、`median_raw: 0.6713431477546692`、`判定: 合格`

## 結論

合格条件（前処理・推定とも終了コード 0、全 cmp 一致、反復行・eval の 3 項目一致）をすべて満たした。

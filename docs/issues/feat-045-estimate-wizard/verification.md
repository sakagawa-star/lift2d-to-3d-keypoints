# feat-045 実データ確認（design §6.3）

日付: 2026-11-26 / 環境: gtune2（RTX 5060 Ti）

## 比較元
5 ファイルの SHA-256 が design §6.3 の表と全件一致したため、feat-044 の手動テスト出力を比較元に使用（再生成なし）。
`phase4/data/10F/feat045_check` は実行前に存在しなかった。

## 実行コマンド
```bash
printf '%s\n' phase4/data/10F/s25_original/20261124_152436.jpg phase4/data/10F/feat045_check/job \
  s25_20261124_152436 phase0/data/points_2d_10F.csv phase0/data/kijunten_locations_10F.csv y \
  phase4/data/10F/point_cloud.ply y \
| TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_wizard.py
```

## 結果
1. 終了コード: 0（3 ツールが順に実行され、「完了しました」。評価: 合格 median=0.671px）
2. cmp（入力コピー）: original.jpg / points_2d_original.csv / points_3d.csv / point_cloud.ply はすべて元と一致（cmp 終了コード 0）
3. cmp（前処理出力）: images/s25_20261124_152436.png・init.toml・points_2d.csv は feat044_images/・feat044_init.toml・feat044_points_2d.csv と一致（0）
4. cmp（推定出力）: out/s25_20261124_152436_loop.toml・_final.toml は feat044_out/ の同名ファイルと一致（0）
5. 画像の確認（Read ツールで目視）:
   - crop_area.png: y=376 と y=2623 に赤線と文字（左上・左下）があり、線の外側の上下の帯が暗い。範囲内は原本どおりの明るさ。左右端の各 1 列の暗さは目視では判別できない（画素値の検証は pytest No.2 で実施）
   - points_check.png: 6 点に赤い円と緑のラベル 001〜006 が描かれ、位置は基準点（モニタ角・壁の継ぎ目・梁・ケージ枠・箱角など）に合っている

---

# イテレーション1（作業フォルダを名前で受け付ける修正後の再確認）

日付: 2026-11-26 / 環境: gtune2（RTX 5060 Ti）

## 比較元
5 ファイルの SHA-256 が design §6.3 の表と全件一致したため、feat-044 の手動テスト出力をそのまま比較元に使用（再生成なし）。作業フォルダ `phase4/data/EIL/feat045-check` は実行前に存在しなかった。既存の `phase4/data/10F/feat045_check/` は触っていない。

## 実行コマンド
```bash
printf '%s\n' phase4/data/10F/s25_original/20261124_152436.jpg feat045-check \
  s25_20261124_152436 phase0/data/points_2d_10F.csv phase0/data/kijunten_locations_10F.csv y \
  phase4/data/10F/point_cloud.ply y \
| TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/estimate_wizard.py
```

## 結果
1. 終了コード: 0（3 ツールが順に実行され「完了しました」。評価: 合格 median=0.671px。作業フォルダの説明・カメラ名の説明・「3DGS の PLY ファイルのパス」の質問が表示された）
2. cmp（入力コピー）: phase4/data/EIL/feat045-check/ の original.jpg / points_2d_original.csv / points_3d.csv / point_cloud.ply はすべて元と一致（cmp 終了コード 0）
3. cmp（前処理出力）: images/s25_20261124_152436.png・init.toml・points_2d.csv は feat044_images/・feat044_init.toml・feat044_points_2d.csv と一致（0）
4. cmp（推定出力）: out/s25_20261124_152436_loop.toml・_final.toml は feat044_out/ の同名ファイルと一致（0）
5. 画像の確認（Read ツールで目視）:
   - crop_area.png: y=376 と y=2623 に赤線と文字があり、線の外側の上下の帯が暗い。範囲内は原本どおり。左右端の各 1 列は目視では判別できない（画素値は pytest No.2 で検証）
   - points_check.png: 6 点に赤い円と緑のラベル 001〜006 が描かれ、位置は基準点に合っている

作業フォルダは phase4/data/EIL/feat045-check に作られ、リポジトリ直下には何も作られていない。

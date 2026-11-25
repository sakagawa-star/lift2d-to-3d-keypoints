# e3_10F_it2 実験ログ

- 案件: feat-041 イテレーション2（criteria.md に従う。criteria.md は変更しない）
- 実行者: ステップ6 の実装担当（Sonnet サブエージェント）
- 日付: 2026-11-25

## 1. 事前確認（bug-006 の回避条件。実行前）

- `test ! -e phase4/data/10F/feat041_out_it2 && test ! -L phase4/data/10F/feat041_out_it2`: 成功（存在しない）
- `test ! -e phase4/data/10F/feat041_tmp_it2 && test ! -L phase4/data/10F/feat041_tmp_it2`: 成功（存在しない）
- 入力ファイル（`--toml` `s25_20261124_152436_init.toml`、`--ply` `point_cloud.ply`、`--images-dir` `s25_images`、`--points-3d` `phase0/data/kijunten_locations_10F.csv`、`--points-2d` `points_2d_s25_20261124_152436.csv`）は、すべて通常ファイル（`ls -l` で確認。シンボリックリンクではない）で、`phase4/data/10F/feat041_out_it2` の外にある
- 実装後の `uv run pytest -v`: 669 passed, 1 skipped（tests/results/feat-041_test_result.txt）

## 2. 予測（実行直前。参考実行 `feat041_out_it1_dump/conv2/` と `feat041_out_it1/` の loop_report の実測値に基づく）

| 項目 | 予測 |
|---|---|
| 収束する反復番号 | i=4（i=3 の Δ=2.143 は 2.0 以上、i=4 の Δ=1.872 は 2.0 未満。LoFTR の揺れで ±1 反復ずれ得る） |
| 最終 f | 約 1279.99（1279〜1281） |
| 最終 cx、cy | 約 960.5、541.2（±1.5px） |
| 最終 k1、k2 | 約 0.0781、-0.0604（±0.004） |
| 最終精緻化の受理の種類 | 受理(単峰) |
| 評価 median | 約 0.671px（0.55〜0.8） |
| 評価 p95 | 約 4.085px（3.5〜4.7）。記録のみ |
| 評価 valid_ratio | 約 0.209（0.18〜0.24） |
| 全体の所要時間 | 約 3 分（旧実行。2〜6 分。60 分以内） |

## 3. 実行（design.md §9 手順 6 のコマンドを 1 回だけ実行。再試行なし）

コマンドの出力から書き写し:

```
EXIT_CODE=0
Command terminated by signal: 行なし
	Command being timed: "uv run --project phase4 python phase4/estimate_intrinsics_loop.py --toml phase4/data/10F/s25_20261124_152436_init.toml --camera s25_20261124_152436 --ply phase4/data/10F/point_cloud.ply --images-dir phase4/data/10F/s25_images --points-3d phase0/data/kijunten_locations_10F.csv --points-2d phase4/data/10F/points_2d_s25_20261124_152436.csv --out-dir phase4/data/10F/feat041_out_it2 --tmp-dir phase4/data/10F/feat041_tmp_it2"
	User time (seconds): 479.67
	System time (seconds): 147.36
	Percent of CPU this job got: 429%
	Elapsed (wall clock) time (h:mm:ss or m:ss): 2:25.96
	Average shared text size (kbytes): 0
	Average unshared data size (kbytes): 0
	Average stack size (kbytes): 0
	Average total size (kbytes): 0
	Maximum resident set size (kbytes): 6130244
	Average resident set size (kbytes): 0
	Major (requiring I/O) page faults: 16
	Minor (reclaiming a frame) page faults: 24799911
	Voluntary context switches: 39352
	Involuntary context switches: 25776688
	Swaps: 0
	File system inputs: 6144
	File system outputs: 299456
	Socket messages sent: 0
	Socket messages received: 0
	Signals delivered: 0
	Page size (bytes): 4096
	Exit status: 0
```

反復ログ（標準出力）:

```
i=0 | MASt3R マッチ2151 N=1996 inl=1650 RMSE=2.351px | f=1273.86 cx=957.10 cy=537.62 k1=0.0500 k2=-0.0421 | Δ=17.413px | 被覆16/16 | lsq_status=2 | 7.0s
i=1 | LoFTR マッチ6939 N=6046 inl=4461 RMSE=1.373px | f=1278.24 cx=960.04 cy=539.19 k1=0.0722 k2=-0.0596 | Δ=14.655px | 被覆15/16 | lsq_status=2 | 2.1s
i=2 | LoFTR マッチ7172 N=6271 inl=5687 RMSE=1.288px | f=1280.03 cx=961.22 cy=540.47 k1=0.0764 k2=-0.0596 | Δ=6.684px | 被覆16/16 | lsq_status=2 | 2.1s
i=3 | LoFTR マッチ7211 N=6314 inl=5853 RMSE=1.241px | f=1280.70 cx=961.53 cy=540.90 k1=0.0784 k2=-0.0606 | Δ=2.143px | 被覆16/16 | lsq_status=2 | 2.1s
i=4 | LoFTR マッチ7218 N=6332 inl=5519 RMSE=1.230px | f=1279.99 cx=960.53 cy=541.23 k1=0.0781 k2=-0.0604 | Δ=1.872px | 被覆16/16 | lsq_status=2 | 2.1s
```

## 4. criteria.md §2 の 6 項目

| # | 項目 | 実測値・確認結果 | 判定 |
|---|---|---|---|
| 1 | 終了状態 | `EXIT_CODE=0`、`Command terminated by signal` の行なし、loop_report の終了状態「成功」 | 合格 |
| 2 | 集計対象画素の割合 | `valid_ratio_raw: 0.20911892361111112` ≥ 0.10。`n_valid` = 433629、433629 / 2073600 = 0.20911892361111112 で一致 | 合格 |
| 3 | 中央値 | `median_raw: 0.6713431477546692` ≤ 1.0 | 合格 |
| 4 | 判定の記載 | eval.txt の判定「合格」。合格ラインの行は `合格ライン: median <= 1.000 px（valid_ratio >= 0.100）。p95 は記録のみ` で一致 | 合格 |
| 5 | NFR-001 | Elapsed (wall clock) time 2:25.96（約 2 分 26 秒）≤ 60 分 | 合格 |
| 6 | 成果物の読み取り | final.toml を `render_keypoints.load_cameras_toml` + `select_camera` で読めた（f=1279.987、cx=960.527、cy=541.231、D=[0.07807, -0.06044, 0, 0]、1920x1080）。評価 4 ファイル（`_eval.txt`、`_eval_blend.png`、`_eval_checker.png`、`_eval_flowmag.png`）が存在し、PNG 3 枚は `cv2.imread` で読めて shape (1080, 1920, 3) = 1920x1080 | 合格 |

記録のみ: p95 = 4.085px、mean = 1.338px、max = 92.365px

**総合判定: 合格**（6 項目すべて満たす）

## 5. 予測と実測の照合

| 項目 | 予測 | 実測 | 乖離 |
|---|---|---|---|
| 収束する反復 | i=4 | i=4（Δ=1.872） | なし |
| 最終 f | 約 1279.99 | 1279.987 | なし |
| 最終 cx、cy | 約 960.5、541.2 | 960.527、541.231 | なし |
| 最終 k1、k2 | 約 0.0781、-0.0604 | 0.07807、-0.06044 | なし |
| 受理の種類 | 受理(単峰) | 受理(単峰)（単峰高合意 f_c=1.000, 第2=0.000） | なし |
| median | 約 0.671px | 0.6713px | なし |
| p95 | 約 4.085px | 4.085px | なし |
| valid_ratio | 約 0.209 | 0.2091 | なし |
| 全体の所要時間 | 約 3 分（2〜6 分） | 2 分 26 秒 | 範囲内。refine_extrinsics.py が 128.8s、評価 0.4s、ループ反復は合計約 15s |

乖離した項目はない。

## 6. 参考実行（`feat041_out_it1_dump/conv2/`）との比較

| 項目 | 参考実行 | 本実行 |
|---|---|---|
| 収束反復 | i=4 | i=4 |
| 反復 i=0〜4 の f・cx・cy・k1・k2・Δ | `feat041_out_it1/` の loop_report と同一 | 同一（小数 2〜4 桁の表示で全反復一致） |
| 最終 f / cx / cy | 1279.987 / 960.527 / 541.231 | 1279.987 / 960.527 / 541.231（loop.toml と一致） |
| 受理 | 受理(単峰) | 受理(単峰) |
| median | 0.671 | 0.671（raw 0.6713431477546692） |
| p95 | 4.085 | 4.085 |
| valid_ratio | 0.209 | 0.209 |
| mean / max | 1.338 / 92.365 | 1.338 / 92.365 |
| 判定 | 不合格（旧基準: p95 ≤ 3.0 を含む） | 合格（新基準: median のみ。参考実行の値は判定に使っていない） |

値は参考実行と一致した（同じ処理のため）。

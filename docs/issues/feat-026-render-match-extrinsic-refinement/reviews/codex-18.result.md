**高**

なし。

**中**

1. [pipeline.md:140](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/pipeline.md:140) の Stage 1.5 補正式が `R^(0) ← M R^(0)` だけで、`t` の更新が書かれていません。  
   実装側は [coarse_align_rescue.py:141](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:141) で `C0=-R0^T t0` を固定し、[coarse_align_rescue.py:143](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:143) で `t_corr=-R_corr C0` にしていて正しいです。  
   ただし docs の式だけ読むと、`R` だけ差し替えて `t` を据え置く誤実装を誘発します。その場合カメラ中心が変わります。  
   修正案: Stage 1.5 の式に `C^(0)=-R^(0)^T t^(0)`、`R^(0)←M R^(0)`、`t^(0)←-R^(0) C^(0)` を明記してください。

2. [coarse_align_rescue.py:77](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:77) のトリミング後の支持点数を検証せず、[coarse_align_rescue.py:142](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/a1_synthetic/coarse_align_rescue.py:142) で補正を適用しています。  
   生マッチ数 `n_raw` は30以上でも、2°トリミング後の `n_used` が極端に少ない場合、Kabsch 補正としては不成立です。現状は `n_used=0〜2` でも前段で推定された `M` を使ってしまいます。  
   修正案: `estimate_yaw_correction` 後に `n_used < MIN_RAW_MATCHES`、少なくとも `< 3` なら粗整列不成立として扱う。さらに厳密には、最後の `use` で `M` を1回再推定してから返すのがよいです。

**低**

なし。

確認済み:
- `r_q ≈ M r_r` に対する `R_corr = M @ R0` の合成方向は正しいです。
- 摂動再現は `basin_map.py` と同じ `SEED=1`、同じ `perturb_pose`、同じループ構成なので一致します。
- 生SIFTマッチの向きは `pipeline_loop.py` と同じ `knnMatch(des_r, des_q)`、`queryIdx→u_r` / `trainIdx→u_q` です。
- `findings.md` と `pipeline.md` は候補段階・検証中であることを明記しており、正式仕様化済みとは読みにくいです。
- 構文確認のみ実施し、実験実行はしていません。
---

## Claude Code の対応方針（2026-07-31）

- 中1（pipeline.md の補正式に t 更新がない）: Stage 1.5 の式に C^(0)=−R^T t、R←MR、t←−RC の3行を明記し、「R だけ差し替えて t を据え置くとカメラ中心が動く」誤実装への警告を注記
- 中2（トリミング後の支持点数未検証）: estimate_yaw_correction を最終インライア集合で M を再推定してから返す形に修正し、呼び出し側で n_used < 30 なら「粗整列不成立」として補正を適用せず記録するガードを追加

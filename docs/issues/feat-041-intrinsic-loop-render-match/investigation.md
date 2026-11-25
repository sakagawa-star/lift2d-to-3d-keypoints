# feat-041 調査・修正計画

本書は CLAUDE.md 機能追加フロー ステップ7（手動テスト）で見つかった問題の修正計画を、`docs/BUGFIX_STANDARD.md` に従ってイテレーション番号付きで追記する。過去のイテレーションは上書きしない。

---

## イテレーション1 (2026-11-25)

### 1.1 不具合の特定

- **対応する要求ID**: FR-003（反復の1回分）、FR-005（収束判定）。結果として requirements.md §7 の合格ライン（重ね合わせ評価の中央値 ≤ 1.0px かつ 95 パーセンタイル値 ≤ 3.0px）を満たさない
- **対応する設計セクション**: design.md §4.2.3（`run_loop_iteration`、`call_mast3r`）、§4.2.5（Δ_intr による収束判定）、§10 ADR「ループのマッチャー」
- **現在の動作**（design.md §9 の手順 3 を実行。2026-11-25 10:11、出力 `phase4/data/10F/feat041_out/`、中間 `phase4/data/10F/feat041_tmp/`）:

  `s25_20261124_152436_loop_report.txt` の反復行:

  ```
  i=0 | マッチ2151 N=1996 inl=1650 RMSE=2.351px | f=1273.86 cx=957.10 cy=537.62 k1=0.0500 k2=-0.0421 | Δ=17.413px | 被覆16/16 | lsq_status=2 | 17.6s
  i=1 | マッチ2064 N=1908 inl=1633 RMSE=1.004px | f=1275.31 cx=958.00 cy=539.38 k1=0.0492 k2=-0.0272 | Δ=11.551px | 被覆16/16 | lsq_status=2 | 6.2s
  i=2 | マッチ2056 N=1884 inl=1689 RMSE=0.672px | f=1275.24 cx=957.66 cy=539.08 k1=0.0493 k2=-0.0266 | Δ=0.843px | 被覆16/16 | lsq_status=2 | 6.2s
  i=3 | マッチ2055 N=1893 inl=1690 RMSE=0.561px | f=1276.11 cx=958.16 cy=538.99 k1=0.0500 k2=-0.0267 | Δ=1.719px | 被覆16/16 | lsq_status=2 | 6.2s
  i=4 | マッチ2055 N=1892 inl=1697 RMSE=0.000px | f=1276.11 cx=958.16 cy=538.99 k1=0.0500 k2=-0.0267 | Δ=0.000px | 被覆16/16 | lsq_status=3 | 6.2s
  ```

  最終精緻化は受理された。評価結果: `n_valid=390585 valid_ratio=0.188 median=1.171px p95=13.125px mean=2.780px max=70.934px 判定=不合格`。比較用の測定（疎通確認の TOML、歪みゼロ）は median 2.066px、p95 19.991px。

- **期待する動作**: requirements.md §7「重ね合わせ評価の判定が合格（中央値 ≤ 1.0px かつ 95 パーセンタイル値 ≤ 3.0px）」。また FR-005 の収束は「内部パラメータが落ち着いた」ことを表すべきで、マッチ座標が変化しなくなったことによる見かけの収束であってはならない
- **エラーメッセージ**: なし（終了コード 0、評価の判定が「不合格」）

### 1.2 原因分析

- **原因箇所**: `phase4/estimate_intrinsics_loop.py` の `call_mast3r` と `run_loop_iteration`（MASt3R のマッチ座標をそのまま 2D-3D 対応に使う箇所）。設計上は design.md §4.2.3 と §10 ADR「ループのマッチャー: MASt3R」
- **原因の説明**:
  1. `matcher_lab/mast3r_cli.py` は画像を長辺 512px に縮小して推論し、マッチ座標を 1920x1080 に逆写像する。このため、マッチ座標は 1920/512 = 3.75px 刻みの格子点に限られる（写真側・レンダ側とも）
  2. 保存済みの中間ファイル `feat041_tmp/iter*_matches.npz` で、写真側とレンダ側のマッチ座標の差（変位）を調べた結果:
     - i = 1 以降、マッチの 71〜89% で変位がちょうど 0 だった（i = 2〜4 は 86〜89%）
     - 0 でない変位も 3.75px の倍数だけだった
  3. 写真とレンダのずれが 3.75px 未満になると、変位はほとんど 0 か 3.75px のどちらかに丸められる。そのため、残っている 1px 前後のずれを測れない
  4. i = 4 では、変位 0 のマッチ（u_q = u_r）が大多数になった。これらは、レンダに使ったパラメータで 3D に戻して投影し直すと、u_q と完全に一致する。その結果、同時最小二乗は初期値のまま停止した（RMSE 0.000px、`lsq_status=3`）。Δ_intr = 0 で「収束」と判定されたが、これはずれが無くなったからではない。マッチの刻みより小さいずれが見えなくなったことによる見かけの収束である
  5. 比較のため、同じ実行の最終精緻化（`refine_extrinsics.py`、LoFTR）の中間ファイル `feat041_tmp/refine/*_A_j0*_matches.npz` も調べた:
     - 変位がちょうど 0 のマッチの割合は 0.0
     - 変位の中央値は 1.36〜2.28px、90 パーセンタイル値は 11〜13px で、連続した値をとる
     - ゲート通過数 N は約 5300〜6000、外れ値除去後は約 3700〜5400 点、RMSE は 1.05〜2.27px（`s25_20261124_152436_refine_report.txt`）
     - LoFTR（`matcher_lab/loftr_cli.py`）は 1280x720 で推論し、係数 1.5 で 1920x1080 に戻す。写真側（image0）の座標は粗い段階の格子位置に固定され、レンダ側（image1）の座標だけがサブピクセルに補正される
     - 本ループでは、深度リフトをレンダ側の座標で行い、写真側の座標は観測値として使う。このため、写真側が格子位置でも、写真のその位置に対応するレンダ側の位置がサブピクセルで得られれば、ずれの情報は残る
- **根本原因 or 表面的原因**: 根本原因は、ループで使うマッチャーの座標分解能（3.75px）が、合格ライン（中央値 1.0px）より粗いことである。収束判定（Δ_intr）は仕様どおりに動いている。見かけの収束は、上記の分解能不足から生じた結果である

### 1.3 修正内容

**方針（ユーザー決定 2026-11-25「5で」）**: i = 0 は MASt3R のまま、i ≥ 1 は LoFTR に切り替える。

- i = 0 では、内部パラメータが Exif の初期値で、歪みもゼロなので、写真とレンダのずれが大きい（今回の実行で Δ_intr = 17.4px）。§10 ADR のとおり、見た目の違いや拡大率の差に強い MASt3R を使う
- i ≥ 1 では、i = 0 の推定で写真とレンダのずれは数 px 以内に縮まっている。ここで、座標を連続値で得られる LoFTR を使う
- LoFTR の呼び出しは、`refine_extrinsics.py` の既存関数 `call_matcher` と定数 `LOFTR_CONF_TH`（0.2）を import して使う。最終精緻化と同じ方法で、既存コードは変更しない

#### 変更対象ファイル

| ファイル | 変更内容 |
|---|---|
| `docs/issues/feat-041-intrinsic-loop-render-match/requirements.md` | 下記「requirements.md の変更案」 |
| `docs/issues/feat-041-intrinsic-loop-render-match/design.md` | 下記「design.md の変更案」 |
| `phase4/estimate_intrinsics_loop.py` | 下記「修正コード」 |
| `tests/test_feat041_intrinsics_loop.py` | テスト 16、17、18 を追加（design.md §8 の変更案） |

#### 変更しないファイル

| ファイル | 理由 |
|---|---|
| `phase4/refine_extrinsics.py` | `call_matcher`、`check_matcher_meta`、`MatcherOOM`、`LOFTR_CONF_TH` を import するだけ。requirements.md §5 の既存コード変更禁止を守る |
| `matcher_lab/loftr_cli.py`、`matcher_lab/mast3r_cli.py` | subprocess で呼ぶだけ |
| `phase4/evaluate_overlay.py` | 評価方法（DIS）は変えない。推定に使わない方法で採点するという §10 ADR の判断は、マッチャーを LoFTR に変えても成り立つ |
| `phase4/prepare_phone_photo.py` | 前処理は今回の問題に関係しない |

#### requirements.md の変更案

1. §1 の表の `estimate_intrinsics_loop.py` の行: 「レンダ → MASt3R マッチング → …」を「レンダ → マッチング（i = 0 は MASt3R、i ≥ 1 は LoFTR）→ …」に変える
2. §2 用語「マッチ」: 「（MASt3R が出力）」を「（反復 i = 0 は MASt3R、i ≥ 1 は LoFTR が出力）」に変える
3. FR-003 概要: 「前処理後画像との MASt3R マッチから」を「前処理後画像とのマッチ（i = 0 は MASt3R、i ≥ 1 は LoFTR）から」に変える
4. FR-003 受け入れ基準 2 を次に置き換える:
   > 2. マッチは、反復 i = 0 では `matcher_lab/mast3r_cli.py` を subprocess で呼んで得る。終了コード 42（CUDA OOM）の場合は `--resolution 1280x720` で1回だけ再試行する。反復 i ≥ 1 では、`matcher_lab/loftr_cli.py` を `refine_extrinsics.py` の `call_matcher` 経由で呼んで得る。信頼度が `refine_extrinsics.py` の `LOFTR_CONF_TH` 未満のマッチは使わない。LoFTR の終了コード 42（CUDA OOM）は再試行せず、その反復の失敗とする（`loftr_cli.py` に解像度を変えるオプションが無いため）
5. §5 使用ライブラリ: 「matcher_lab: MASt3R」を「matcher_lab: MASt3R、LoFTR（kornia）」に変える
6. §5 GPU メモリ: 「MASt3R を別プロセスで実行する」を「MASt3R・LoFTR を別プロセスで実行する」に変える

#### design.md の変更案

1. §1.1 の表に行を追加: `| matcher_lab/loftr_cli.py | 既存・変更しない | 反復 i ≥ 1 のマッチ。refine_extrinsics.call_matcher 経由の subprocess で呼ぶ |`
2. §1.2 の依存関係に追加: `estimate_intrinsics_loop.py ──import──> refine_extrinsics.call_matcher ──subprocess──> matcher_lab/loftr_cli.py（matcher_lab 環境）`
3. §2: 「MASt3R は matcher_lab 環境の既存 CLI `matcher_lab/mast3r_cli.py` を subprocess で使う」の後に「LoFTR は同じ環境の既存 CLI `matcher_lab/loftr_cli.py` を `refine_extrinsics.call_matcher` 経由で使う」を加える
4. §3.1 に定数を追加: `| LOFTR_FROM_ITER | 1 | この反復番号以降は LoFTR でマッチを求める（それより前は MASt3R） |`。また、import する定数の一覧（§3.1 末尾の段落）に `LOFTR_CONF_TH`(0.2) を加える
5. §4.2.1 の終了コード 3 の説明に「LoFTR の OOM」を加える
6. §4.2.3 の擬似コードの `u_q, u_r, meta = call_mast3r(...)` と `check_matcher_meta(meta)` の2行を次に置き換える:

   ```
   if i < LOFTR_FROM_ITER:
       matcher_name = "MASt3R"
       u_q, u_r, meta = call_mast3r(query_png, render_png, tmp_dir / f"iter{i:02d}_matches.npz")
   else:
       matcher_name = "LoFTR"
       try:
           u_q, u_r, conf, meta = call_matcher(query_png, render_png, tmp_dir / f"iter{i:02d}_matches.npz")
       except MatcherOOM:
           → 失敗「LoFTR OOM」
       sel = conf >= LOFTR_CONF_TH
       u_q, u_r = u_q[sel], u_r[sel]          # check_matcher_meta の後でもよい（meta は全マッチの値）
   check_matcher_meta(meta)                    # 両マッチャーとも同じ関数で検査する
   ```

   - マッチ数（レポートの「マッチ」）は、LoFTR の場合は信頼度で選んだ後の件数とする（`refine_extrinsics.py` の「選別後」と同じ意味）
   - 戻り値の `diag` に `"matcher": matcher_name` を加える
   - `call_matcher` が送出する `MatcherEnvError`（終了コード 42 以外の非 0 終了）は、MASt3R の場合と同じく終了コード 1 とする（§4.2.1 の既存の扱い）
7. §4.2.3 の標準出力の進捗表示: 「反復 i={i}: MASt3R 実行中...」を「反復 i={i}: {matcher_name} 実行中...」に変える
8. §5 中間ファイル: `iter{i:02d}_matches.npz` は、i = 0 では MASt3R の NPZ（キー `u_a`, `u_b`, `meta`）、i ≥ 1 では LoFTR の NPZ（キー `u_q`, `u_r`, `conf`, `meta`）であることを明記する
9. §7 レポートの反復行の書式: 「マッチ」の前にマッチャー名を入れる。例:

   ```
   i=2 | LoFTR マッチ6843 N=5410 inl=3902 RMSE=1.102px | f=1283.41 cx=957.88 cy=543.02 k1=0.0123 k2=-0.0451 | Δ=0.731px | 被覆14/16 | lsq_status=1 | 41.2s
   ```

   §4.2.7 の反復行の項目一覧にも「マッチャー名」を加える
10. §8 テストに追加:

    | # | 対象 | 内容 | 期待 |
    |---|---|---|---|
    | 16 | `run_loop_iteration` のマッチャー切り替え | レンダ・`call_mast3r`・`call_matcher` をモックで置き換え、i = 0、1、2 で呼ぶ。`call_matcher` のモックは、信頼度 0.1 と 0.5 のマッチを混ぜて返す | i = 0 では `call_mast3r` だけが呼ばれ、i = 1、2 では `call_matcher` だけが呼ばれる。i ≥ 1 では、信頼度 0.1 のマッチがゲートに渡されない（`gate_breakdown` に渡る点数が信頼度 0.5 の件数と一致する）。`diag["matcher"]` が i = 0 で "MASt3R"、i ≥ 1 で "LoFTR" |
    | 17 | LoFTR の OOM | i = 1 で `call_matcher` のモックが `MatcherOOM` を送出する | `LoopFailure`（理由に「LoFTR OOM」を含む） |

11. §9 実データでの動作確認: 手順 3 の出力先を `phase4/data/10F/feat041_out_it1`、中間を `phase4/data/10F/feat041_tmp_it1` に変える（前回の結果を比較のため残す）。実験ログは `experiments/e2_10F_loftr/experiment_log.md` に書く。合格ラインは変えない
12. §10 ADR「ループのマッチャー」の行を次に置き換える:

    | 判断 | 採用 | 却下案と理由 |
    |---|---|---|
    | ループのマッチャー | i = 0 は MASt3R、i ≥ 1 は LoFTR（feat-041 イテレーション1、ユーザー決定） | 全反復 MASt3R: 長辺 512px で推論するため座標が 3.75px 刻みになり、1px 未満のずれを測れない（実データで i ≥ 2 のマッチの 86〜89% が変位 0 になり、見かけの収束になった）。全反復 LoFTR: 視点・拡大率の差に弱く、内部パラメータが数 % ずれ、歪みもゼロの i = 0 に不利。MASt3R のマッチの局所相関によるサブピクセル補正、MASt3R の切り出し原寸処理、密なオプティカルフロー、画像を直接比べる最適化: 検討したが、既存部品で実装でき、連続値が得られることをデータで確認済みの LoFTR を採用した（ユーザー決定） |

#### 収束判定の変更（codex-09 指摘1 で追加）

上記のマッチャー切り替えだけでは、i = 0（MASt3R）で Δ_intr < 0.5px になると、その場で収束と判定してループを終える。この場合、LoFTR の反復は一度も行われない。初期値が真値に近く、i = 0 のマッチの大半が変位 0 に丸められると、今回と同じ見かけの収束が i = 0 で起きる。Codex による CPU の合成例では、最大約 1.02px のずれがあるのに Δ_intr ≈ 5.1×10⁻¹³px、`lsq_status=3` になった。最終精緻化は内部パラメータを固定するので、この見落としを補えない。

そこで、収束と判定するのは i ≥ `LOFTR_FROM_ITER` の反復だけにする。

- requirements.md FR-005 受け入れ基準 1 に、「収束と判定してよいのは LoFTR でマッチを求めた反復（i ≥ 1）だけ。i = 0 では Δ_intr がしきい値未満でも次の反復に進む」を加える
- design.md:
  - §3.1 `CONV_PX` の説明を「i ≥ `LOFTR_FROM_ITER` の反復で Δ_intr < 0.5px なら収束」にする
  - §4.2.1 の擬似コードの収束判定を `is_converged(i, delta)` にする
  - §4.2.5 の終了条件に同じ条件と理由を書く
  - §6.2 に `is_converged(i: int, delta: float) -> bool` を加える
  - §8 にテスト 18 を加える
- 範囲チェック（`check_ranges`）は従来どおり全反復で行う

#### 修正コード（`phase4/estimate_intrinsics_loop.py`）

修正前（`run_loop_iteration` の一部）:

```python
    print(f"反復 i={i}: MASt3R 実行中...")
    u_q, u_r, meta = call_mast3r(Path(query_png), render_png,
                                 Path(tmp_dir) / f"iter{i:02d}_matches.npz")
    check_matcher_meta(meta)
    n_match = len(u_q)
```

修正後:

```python
    out_npz = Path(tmp_dir) / f"iter{i:02d}_matches.npz"
    if i < LOFTR_FROM_ITER:
        matcher_name = "MASt3R"
        print(f"反復 i={i}: {matcher_name} 実行中...")
        u_q, u_r, meta = call_mast3r(Path(query_png), render_png, out_npz)
    else:
        matcher_name = "LoFTR"
        print(f"反復 i={i}: {matcher_name} 実行中...")
        try:
            u_q, u_r, conf, meta = call_matcher(Path(query_png), render_png, out_npz)
        except MatcherOOM as e:
            raise LoopFailure("LoFTR OOM") from e
        sel = conf >= LOFTR_CONF_TH
        u_q, u_r = u_q[sel], u_r[sel]
    check_matcher_meta(meta)
    n_match = len(u_q)
```

その他:

- import に `LOFTR_CONF_TH`、`MatcherOOM`、`call_matcher` を加える（`from refine_extrinsics import (...)`）
- 定数 `LOFTR_FROM_ITER = 1` を追加する
- `diag` に `"matcher": matcher_name` を加える
- `_format_iter_line` の先頭を `f"i={i} | {diag['matcher']} マッチ{diag['matches']} ..."` にする
- 関数 `is_converged(i: int, delta: float) -> bool`（`return i >= LOFTR_FROM_ITER and delta < CONV_PX`）を追加し、`_run` のループの `if delta < CONV_PX:` を `if is_converged(i, delta):` に置き換える
- モジュール docstring の「MASt3Rマッチ」を「マッチ（i=0 は MASt3R、i≥1 は LoFTR）」に変える

**修正が設計書に沿っているか**: §10 ADR「ループのマッチャー」と矛盾するため、上記の design.md 変更案で ADR を更新する。それ以外は既存設計の枠内の変更である。

#### 今回の修正に含めないこと

- **見かけの収束を検出する仕組み**（例: 変位 0 のマッチの割合の監視、`lsq_status` の監視）: 今回の見かけの収束は、MASt3R の座標の刻みが原因である。i ≥ 1 で LoFTR を使えば、写真側とレンダ側の座標は一般に一致しないので、同じ現象は起きない。検出の仕組みは、LoFTR でも同様の現象が出た場合に別のイテレーションで扱う
- **収束しきい値 `CONV_PX`（0.5px）の見直し**: 下記 1.4 のリスクが実データで起きた場合に、別のイテレーションで扱う

### 1.4 影響範囲

- **他の機能への影響**: `estimate_intrinsics_loop.py` の反復 i ≥ 1 だけ。`prepare_phone_photo.py`、`evaluate_overlay.py`、`refine_extrinsics.py`（最終精緻化の subprocess 呼び出しを含む）の動作は変わらない
- **リグレッションリスク**:
  1. **反復ごとの所要時間**: 今回の `refine_extrinsics.py` は LoFTR を含む反復 63 回を 127.4s で終えており、1 回あたり約 2s である。MASt3R（今回 6.2s/反復）より短いので、所要時間は増えない。NFR-001 の上限（全体 60 分以内）に対し、前回の実行は全体で約 3 分だった（ループ約 42s + `refine_extrinsics.py` 127.4s + 評価 0.4s。PLY 読み込み等を除く）
  2. **収束しない可能性**: LoFTR のマッチのノイズにより、反復ごとの推定値の揺れが Δ_intr 0.5px を下回らず、8 反復で収束しない可能性がある。その場合は終了コード 3（未収束）になり、反復行の Δ の推移がレポートに残る
  3. **反復ごとのゲート・RANSAC しきい値**（i ≥ 1 で 30px、3px）は変えない。LoFTR の変位の 90 パーセンタイル値は 11〜13px、`refine_extrinsics.py` の RMSE は 1.05〜2.27px で、どちらもしきい値の範囲内に収まる
  4. **既存テスト**: テスト 7b、14b は `run_loop_iteration` を i = 0 で呼び、`call_mast3r` をモックしている（`tests/test_feat041_intrinsics_loop.py` の 201 行目、224 行目）。i = 0 は MASt3R のままなので、既存テストは変更なしで通る

### 1.5 確認方法

- **自動テスト**: design.md §8 のテスト 16、17、18 を追加し、`uv run pytest -v` の全件実行結果を `tests/results/feat-041_test_result.txt` に上書き保存する。既存テスト（1〜15、6b、7b、7c、7d、14b）がすべて通ること
- **手動テスト（実データ）**: design.md §9 の手順 3 を、変更後の出力先（`feat041_out_it1`、`feat041_tmp_it1`）で実行する。手順 1（前処理）と手順 2（比較用の測定）は、前回の出力をそのまま使い、再実行しない
  - 合格ライン（requirements.md §7、変更なし）: `feat041_out_it1/s25_20261124_152436_eval.txt` の判定が「合格」（中央値 ≤ 1.0px かつ 95 パーセンタイル値 ≤ 3.0px）
  - CLAUDE.md の実験プロトコルに従い、実行直前に予測値（反復数、各反復の LoFTR マッチ数・N・RMSE の範囲、最終 f・k1・k2、評価の中央値・p95）を `experiments/e2_10F_loftr/experiment_log.md` に記録し、実行後に実測値と照合する
  - 合否にかかわらず、前回の結果（median 1.171px、p95 13.125px）と比較した表を実験ログに残す
  - NFR-001: レポートの反復ごとの所要時間、`refine_extrinsics.py` の所要時間、評価の所要時間を合計し、さらにコマンド全体を `/usr/bin/time -v` で計測する（Elapsed wall clock time）。全体が 60 分以内であることを確認して実験ログに記録する（codex-11 指摘1）

---

## イテレーション2 (2026-11-25)

### 2.1 不具合・要求漏れの特定

イテレーション1 の実装後の手動テスト（design.md §9 手順 5、出力 `phase4/data/10F/feat041_out_it1/`、実験ログ `experiments/e2_10F_loftr/experiment_log.md`）と、その後の追加測定（`experiments/m1_p95_tail/experiment_log.md`）で、次の 3 点が見つかった。

#### 問題A: 収束しきい値が LoFTR のばらつきより小さく、収束しない

- **対応する要求ID / 設計セクション**: FR-005 / design.md §3.1 `CONV_PX`、§4.2.5
- **現在の動作**: 8 反復で未収束（終了コード 3）。Δ_intr の推移は 17.41（i=0, MASt3R）、14.66、6.68、2.14、1.87、1.18、1.04、1.48（i=1〜7, LoFTR）。i=4 以降は 1.0〜1.9px の範囲で増減し、単調に減らない。f は 1279.99〜1280.73、主点は 0.5〜1px、k1・k2 は 0.003 程度、反復ごとに揺れる
- **期待する動作**: FR-005「Δ_intr が収束しきい値未満になった反復で収束とし、ループを終える」。内部パラメータが落ち着いた時点でループを終え、最終精緻化と評価まで進む
- **エラーメッセージ**: なし（レポートの終了状態「未収束」）

#### 問題B: 未収束のとき、最後の反復の値が残らない

- **対応する設計セクション**: design.md §4.2.1（未収束は TOML を書かずに終了 3）、§4.2.6
- **現在の動作**: 未収束で終了すると、ループ結果 TOML が書かれない。レポートには f、cx、cy、k1、k2 だけが残り、回転・並進は残らない。ユーザーが TOML を 3DGS に取り込んで確認したいと要望したが、出力からは作れなかった（使い捨てスクリプトでループを再現して書き出した。結果はレポートと全反復で一致）
- **期待する動作（要求漏れ）**: 未収束でも、最後の反復の値を TOML として確認できる（ユーザー要望 2026-11-25）

#### 問題C: p95 が旧合格ライン（3.0px）を超えた。p95 を押し上げる原因は未特定（要求漏れ: 合否基準に根拠がなかった）

- **対応する要求ID**: FR-008 受け入れ基準 3、requirements.md §7 合格ライン
- **現在の動作**: CONV_PX = 2.0 相当（i=4 で収束）の値から既存ツールで最終精緻化（受理）と評価を行った結果、median 0.671px、p95 4.085px、valid_ratio 0.209 で不合格（出力 `phase4/data/10F/feat041_out_it1_dump/conv2/`。正式な実行ではない）
- **追加測定の結果**（`experiments/m1_p95_tail/experiment_log.md`。いずれも測定で、判定には使っていない）:
  - ずれ > 3px の画素（有効画素の 8.6%）は画像全体に散らばり、最下段（y ≥ 900）に 49.9% が集まる
  - 目視で挙げた 3 領域（布団・ブラインド・黒フレーム）を除いても p95 は 3.976px で、ほぼ下がらない（median は 0.541px）
  - 深度帯ごとの p95 はどの帯も 3.2px 以上（5〜10m の帯を除く）
  - ずれの向きは、240x180 のセルの中ではそろう（最下段の一貫性 C の中央値 0.81）が、主点から見た放射成分の符号はセルごとにそろわず（正 4・負 4）、隣のセルで反転する。原因の扱いは §2.2 のとおり（原因未特定）
  - ユーザー情報: 3DGS の撮影（PortalCam、魚眼で前と左右を同時撮影）は高さ約 150cm で水平に移動しただけで、部屋の下側はあまり撮っていない
- **期待する動作（要求漏れ）**: 合否基準は、推定したカメラパラメータの良否を表すべきである。カメラパラメータ以外の要因（3DGS の再現品質など）で決まる量で合否を決めるべきではない。また、元の合格ライン（median ≤ 1.0px かつ p95 ≤ 3.0px）には特に根拠がなかった（ユーザー 2026-11-25）

### 2.2 原因分析

- **問題A**:
  - **原因箇所**: `phase4/estimate_intrinsics_loop.py` の定数 `CONV_PX = 0.5` と `is_converged`。設計は design.md §3.1
  - **原因の説明**: Δ_intr は、5x5 格子点の移動量の最大値である。この値は画像の隅で決まり、隅では k1・k2 の小さな揺れが大きく出る。LoFTR のマッチの誤差（RMSE 約 1.2px）による反復ごとの推定値の揺れは、Δ_intr にして 1.0〜1.9px あった。0.5px はこの揺れより小さいので、届かない
  - **根本原因 or 表面的原因**: 根本原因。しきい値が推定の揺れの大きさに合っていない
- **問題B**:
  - **原因箇所**: `_run` の `if not converged:` の分岐（TOML を書く前に return 3）
  - **根本原因 or 表面的原因**: 設計どおりの動作で、要求漏れである
- **問題C**:
  - **原因箇所**: `phase4/evaluate_overlay.py` の `judge`（`PASS_P95_PX = 3.0` を合否に使う）。設計は design.md §3.2、§4.3.3
  - **確かめた事実**（m1、m1b）: p95 を押し上げるずれ（> 3px）は最下段に半分が集まる。目視の 3 領域を除いても p95 はほぼ下がらない。ずれの向きはセルの中ではそろうが、最下段 8 セルで一貫性 C の中央値は 0.81、放射成分 Rr の符号は正 4・負 4 で、隣のセルで急に反転する。m1b のログには「3DGS 側」という分類が書かれているが、その分類の条件は排他的でなく（例: 正 6・負 2 のとき両方に当てはまる読み方がある）、criteria lock も経ていないので、根拠には使わない（codex-17 指摘4）。主点の一様なずれでも、放射成分に射影すると左右で符号が変わり得るので、この符号の分布だけではカメラパラメータの誤差を除外できない（codex-23 指摘4）
  - **原因未特定。仮説**: 3DGS の下側の撮影不足（ユーザー情報）による、床・机の下・椅子まわりの局所的な形状誤差。m1b の測定では、3DGS の形状誤差、DIS の誤差、姿勢の誤差のそれぞれの寄与は分離できていない。また、「中央値がカメラパラメータの良否を表す」ことも検証していない
  - **根本原因 or 表面的原因**: p95 を押し上げる原因は未特定（上記の仮説）。合否基準から p95 を外すのは、根拠の無かった初版の基準を、上記の測定事実と仮説に基づいてユーザーが改めたものである（ユーザー決定 2026-11-25）

### 2.3 修正内容

**方針（ユーザー決定 2026-11-25）**:

1. **収束しきい値**: `CONV_PX` を 0.5px から 2.0px に変える。収束条件は Δ_intr < CONV_PX なので、10F の 1 回の実行で観測した LoFTR 反復の Δ_intr の揺れ（i ≥ 4 で 1.040〜1.872px）の上限 1.872px を上回る値として 2.0px を選んだ。今回のデータでは i=4（Δ_intr = 1.872px）で収束する。根拠はこの 1 回の観測だけである（codex-17 指摘5）
2. **未収束時の TOML**: 未収束で終了するときも、最後の反復の値で `{camera}_loop.toml` を書く。終了コードは 3 のまま、最終精緻化と評価は行わない
3. **合否基準（案A）**: 合格は「集計対象画素の割合 ≥ 0.10 かつ中央値 ≤ 1.0px」だけにする。p95 は記録するが、合否には使わない

**事後に決めた基準であることの明記**: 3 の基準は、CONV_PX = 2.0 相当の参考実行の評価値（median 0.671px、p95 4.085px）と、追加測定（m1、m1b）の結果を見た後に決めた。根拠は、元の基準に根拠がなかったこと、および p95 を押し上げるずれが上記の観測事実のとおりだった（原因は未特定）という測定事実（原因は未特定。仮説は 3DGS の下側の再現誤差。2.2）である。合否の判定は、この基準を文書で確定し、レビューを経た後の正式な実行（下記 2.5）で行う。参考実行の値（0.671 / 4.085）は判定に使わない。

#### 変更対象ファイル

| ファイル | 変更内容 |
|---|---|
| `requirements.md` | 下記「requirements.md の変更案」 |
| `design.md` | 下記「design.md の変更案」 |
| `phase4/estimate_intrinsics_loop.py` | `CONV_PX = 2.0`。未収束の分岐で `{camera}_loop.toml` を書いてから終了 3 |
| `phase4/evaluate_overlay.py` | `judge` から p95 の条件を外す。定数 `PASS_P95_PX` を削除する。`{out_prefix}.txt` の合格ラインの行を変える |
| `tests/test_feat041_intrinsics_loop.py` | テスト 13 の期待値を変え、境界ケースを加える。テスト 18 の期待値を `CONV_PX = 2.0` に合わせて変える。テスト 19、20、21 を追加し、テスト 7c の記述を変える（下記「最終ゲート（codex-17）による追加の変更」） |

#### 変更しないファイル

| ファイル | 理由 |
|---|---|
| `phase4/refine_extrinsics.py`、`matcher_lab/*`、`phase4/render_keypoints.py`、`phase4/render.py` | requirements.md §5 の既存コード変更禁止。今回の問題に関係しない |
| `phase4/prepare_phone_photo.py` | 前処理は今回の問題に関係しない |

#### requirements.md の変更案

1. FR-005 受け入れ基準 2 を次に置き換える:
   > 2. 反復回数の上限（機能設計書）に達しても収束しない場合は、最後の反復の値でループ結果 TOML を書き出したうえで、終了コード 3 で終了する（`refine_extrinsics.py` と重ね合わせ評価は行わない）
2. FR-007 受け入れ基準: 「最後に終了状態（…）」の後に「未収束の場合は、ループ結果 TOML を最後の反復の値で書き出したことも記録される」を加える
3. FR-008 受け入れ基準 3 を次に置き換える:
   > 3. 集計対象の画素の割合が 10% 以上の場合に、ずれの大きさの中央値 ≤ 1.0px のとき「合格」、それ以外は「不合格」と判定する（10% 未満は基準 2 の「判定不能」が優先する）。95 パーセンタイル値は記録するが、判定には使わない（10F の測定で、95 パーセンタイル値を押し上げるずれ（> 3px）は最下段に集まり、240x180 のセルの中では向きがそろう（一貫性 C の中央値 0.81）が、主点から見た放射成分の符号はセルごとに正 4・負 4 で、隣のセルで反転した。原因は未特定で、仮説は 3DGS の下側の撮影不足による局所的な再現誤差。investigation.md イテレーション2、ユーザー決定）
4. §7 合格ライン: 「（ずれの中央値 ≤ 1.0px かつ 95 パーセンタイル値 ≤ 3.0px）。ユーザー決定 2026-11-25」を「（ずれの中央値 ≤ 1.0px。95 パーセンタイル値は記録のみ）。ユーザー決定 2026-11-25（イテレーション2 で 95 パーセンタイル値の条件を外した。元の基準には根拠がなく、新しい基準は参考実行と追加測定の結果を見た後に決めたもの。investigation.md イテレーション2）」に変える
5. §8 変更履歴に追加: 「2026-11-25: investigation.md イテレーション1（i ≥ 1 を LoFTR、i = 0 では収束としない）、イテレーション2（収束しきい値、未収束時の TOML 出力、合否基準から 95 パーセンタイル値を外す）を反映」

#### design.md の変更案

1. §3.1 `CONV_PX`: 値を 2.0 にし、説明を「収束しきい値。i ≥ `LOFTR_FROM_ITER` の反復で Δ_intr < 2.0px なら収束（イテレーション2 で 0.5 → 2.0。LoFTR の反復ごとの揺れが Δ_intr で 1.0〜1.9px あるため）」にする
2. §3.2: `PASS_P95_PX` の行を削除する。`PASS_MEDIAN_PX` の説明を「合格ライン（中央値）。95 パーセンタイル値は記録のみで判定に使わない」にする
3. §4.2.1 の擬似コード: `if not converged: レポートに「未収束」と記録して終了(3)` を次に置き換える:

   ```
   if not converged:
       ループ結果 TOML を最後の反復の値（params）で書く（§4.2.6 と同じ書式・同じパス）
       レポートに「未収束（ループ結果 TOML は最後の反復の値）」と記録して終了(3)
   ```

   - 反復途中の失敗（`LoopFailure`）・範囲外のときは TOML を書かない。収束後の逆歪み精度検査の不合格は、初版から TOML を書いた後に検査するので TOML が残る（終了状態は「ループ失敗（最終精緻化の前提違反…）」）。終了の場面ごとの終了コード・終了状態・TOML の有無を design.md §4.2.1 に表で定める（codex-23 指摘3）
4. §4.2.5 終了条件: 「Δ_intr < 0.5」を「Δ_intr < 2.0（`CONV_PX`）」に変える。「i = 7 の反復を終えても収束しなければ終了コード 3」の後に「（最後の反復の値でループ結果 TOML を書く。§4.2.1）」を加える
5. §4.2.7: 終了状態「未収束」を「未収束（ループ結果 TOML は最後の反復の値）」に変える
6. §4.3.3 judge を次に置き換える:

   ```
   judge:
     valid_ratio < 0.10 → "判定不能"
     median <= 1.0 → "合格"
     それ以外 → "不合格"
   ```

7. §4.3.4 `{out_prefix}.txt` の合格ラインの行の書式を `合格ライン: median <= 1.000 px（valid_ratio >= 0.100）。p95 は記録のみ` にする
8. §8 テスト:
   - テスト 13 を「valid_ratio 0.09 / (median 1.0, p95 3.0) / (1.01, 3.0) / (1.0, 50.0) → 判定不能 / 合格 / 不合格 / 合格」に変える
   - テスト 18 を「(i, delta) = (0, 0.0)、(0, 1.99)、(1, 1.99)、(1, 2.0)、(2, 0.0) → False、False、True、False、True」に変える
   - テスト 19 を追加:

     | # | 対象 | 内容 | 期待 |
     |---|---|---|---|
     | 19 | 未収束時のループ結果 TOML | `estimate_intrinsics_loop` のモジュール属性を monkeypatch で置き換えて `main` を呼ぶ。置き換える属性: `load_cameras_toml`、`select_camera`、`load_points_3d_csv`、`load_points_2d_csv`、`validate_camera`（`{"ok": True, ...}`）、`pnp_manual`、`median_reproj_px`、`run_loop_iteration`（f を 1280 と 1300 で交互に返し、毎回 Δ_intr ≥ 2.0 にする）。`sys.modules["render"]` に `load_ply` を持つダミーモジュールを入れる | 終了コード 3。`{camera}_loop.toml` が存在し、`render_keypoints.load_cameras_toml` で読むと f が最後の反復（i = 7）の値と一致する。`{camera}_final.toml` と評価の 4 ファイルは存在しない。レポートに「未収束（ループ結果 TOML は最後の反復の値）」がある |

9. §9 に手順 6 を追加する（下記 2.5 の内容）
10. §10 ADR:
    - 行を追加: `| 収束しきい値 | CONV_PX = 2.0px（イテレーション2） | 0.5px: LoFTR の反復ごとの揺れ（Δ_intr で 1.0〜1.9px）より小さく、10F の実データで 8 反復しても収束しなかった。直近数回の平均で判定する方式: 揺れをならせるが設計変更が大きい。反復回数の固定: 収束の判定がなくなる |`
    - 「評価のずれの測り方」の行の後に追加: `| 合否基準 | 中央値 ≤ 1.0px のみ。p95 は記録のみ（イテレーション2、ユーザー決定） | 中央値 ≤ 1.0px かつ p95 ≤ 3.0px（初版）: 根拠がなかった。10F の測定で、p95 を押し上げるずれは最下段に集まり、240x180 のセルの中では向きがそろう（一貫性 C の中央値 0.81）が、主点から見た放射成分の符号はセルごとに正 4・負 4 で、隣のセルで反転した（原因は未特定。仮説は 3DGS の下側の撮影不足による局所的な再現誤差。カメラパラメータ・DIS・3DGS の各寄与は分離していない）。3DGS の再現が不確かな領域を除いて p95 を測る: 領域を決める客観的な基準が無い。新しい基準は、参考実行と測定の結果を見た後に決めた（investigation.md イテレーション2） |`

#### 最終ゲート（codex-17）による追加の変更

| 指摘 | 変更 |
|---|---|
| 1 テスト 19 が最後の反復の値か確かめられない | design.md §8 テスト 19 を強化。反復ごとに固有の f・cx・cy・k1・k2・rvec・tvec を返すモックにし、呼び出しが 8 回であること、読み戻した K・D・rvec・tvec が i = 7 の値と一致することを確かめる |
| 2 同じ出力先で再実行して未収束になると新旧が混在する | `estimate_intrinsics_loop.py` から `--overwrite` を廃止する。出力 8 ファイルのいずれかが存在すれば（`os.path.lexists`。リンク先の無いシンボリックリンクを含む）、従来どおり処理開始前に終了コード 1。再実行は新しい `--out-dir` か、利用者による手動削除で行う（requirements.md NFR-004、design.md §4.2.1）。テスト 7c の記述を変え、テスト 20 を置き換える。経緯は下記「削除案の取り下げ」 |
| 3 丸めた出力では境界値を確かめられない | eval.txt に `valid_ratio_raw` と `median_raw`（`repr(float)`）を加える。文字列の組み立てを `format_eval_txt` に分け（design.md §4.3.4、§6.3）、テスト 21 を追加。テスト 13 に、小数 3 桁で同じ表示になる境界の両側を加える。criteria.md の判定は `_raw` の値で行う |
| 4 m1b の分類が排他的でなく criteria lock も無い | §2.2 の根拠を実測値（C の中央値 0.81、Rr の符号 正 4・負 4）に置き換え、分類は根拠に使わないと明記。m1 の実験ログに同じ注記を追記 |
| 5 CONV_PX の選定理由が判定式と逆 | 「観測した揺れの上限 1.872px を上回る値」に修正（§2.3、design.md §3.1、ADR） |
| 6 正式な判定の実行者が不一致 | ステップ6 の実装担当による design.md §9 手順 6 の 1 回の実行を正式な結果とし、ユーザーはステップ7 で出力を確認する（§2.5、design.md §9、criteria.md §2） |

**削除案の取り下げ（codex-18・codex-19、行き詰まり検出）**: 指摘2 に対して最初は「`--overwrite` 時に前回の出力 8 ファイルを処理開始前に削除する」案を採った。しかし、この案から新しい指摘が 2 回続いた。codex-18 では、入力 TOML を削除し得る（高）、削除の途中失敗（中）。codex-19 では、入力検証が削除より後になる（中）、リンク先の無いシンボリックリンクがすり抜ける（中）。CLAUDE.md の行き詰まり検出に従い、削除案を取り下げて代替案を比べた。

| 案 | 評価 |
|---|---|
| 削除を続け、検査を足す | 例外ケースが続けて見つかっており、採らない |
| 前回の出力を退避（名前変更）する | 削除と同じ種類の問題（衝突・途中失敗・リンク）が残る |
| **`--overwrite` を廃止し、出力ファイルが 1 つでもあれば拒否する（採用）** | ファイルを削除・上書きする処理そのものがなくなり、入力の削除は起きない。逐次実行（前の実行が終了してから次を起動する）では、前回の出力との混在も防げる（同時実行は対象外。下記「今回の修正に含めないこと」）。運用でも毎回新しい出力先（`feat041_out_it1`、`_it2`）を使っている |

コードの追加変更（下記「修正コード」に加えて）:

- `phase4/estimate_intrinsics_loop.py`: argparse から `--overwrite` を削除する。`main` の出力済み検査を、`args.overwrite` の条件を外し、存在判定を `os.path.lexists` にする（8 パスのいずれかがあれば終了コード 1）
- `phase4/evaluate_overlay.py`: `format_eval_txt(stats, verdict, inputs_text) -> str` を追加し、`evaluate_and_write` の `lines = [...]` をこの関数に移す。`valid_ratio` の行の後に `valid_ratio_raw: {repr}`、`median` の行の後に `median_raw: {repr}` を加える
- `tests/test_feat041_intrinsics_loop.py`: テスト 13 に境界ケースを追加、テスト 19 を強化、テスト 20（置き換え）・21 を追加、テスト 7c の記述を変える

#### 修正コード

`phase4/estimate_intrinsics_loop.py`:

修正前:

```python
CONV_PX = 0.5
...
    if not converged:
        rep.append("終了状態: 未収束")
        print("エラー: 未収束", file=sys.stderr)
        return 3

    out["loop_toml"].write_text(format_loop_toml(camera, params), encoding="utf-8")
```

修正後:

```python
CONV_PX = 2.0
...
    if not converged:
        out["loop_toml"].write_text(format_loop_toml(camera, params), encoding="utf-8")
        rep.append("終了状態: 未収束（ループ結果 TOML は最後の反復の値）")
        print("エラー: 未収束（ループ結果 TOML は最後の反復の値で書き出した）", file=sys.stderr)
        return 3

    out["loop_toml"].write_text(format_loop_toml(camera, params), encoding="utf-8")
```

`phase4/evaluate_overlay.py`:

修正前:

```python
PASS_MEDIAN_PX = 1.0
PASS_P95_PX = 3.0
...
    if stats["median"] <= PASS_MEDIAN_PX and stats["p95"] <= PASS_P95_PX:
        return "合格"
...
        f"合格ライン: median <= {PASS_MEDIAN_PX:.3f} px かつ p95 <= {PASS_P95_PX:.3f} px"
        f"（valid_ratio >= {VALID_RATIO_MIN:.3f}）",
```

修正後:

```python
PASS_MEDIAN_PX = 1.0
...
    if stats["median"] <= PASS_MEDIAN_PX:
        return "合格"
...
        f"合格ライン: median <= {PASS_MEDIAN_PX:.3f} px"
        f"（valid_ratio >= {VALID_RATIO_MIN:.3f}）。p95 は記録のみ",
```

**修正が設計書に沿っているか**: §3.1、§3.2、§4.2.1、§4.2.5、§4.3.3、§4.3.4、§10 と矛盾するため、上記の design.md 変更案で更新する。

#### 今回の修正に含めないこと

- 3DGS の撮り直し（下側の撮影）。ユーザーの運用判断で、本案件のコードの範囲外
- 3DGS の再現が不確かな領域を評価から除く仕組み（ADR の却下案のとおり）
- 歪み係数の追加（k3 など）。p95 を押し上げる原因が未特定のため、本イテレーションでは扱わない
- 最終精緻化の一時ファイル（`refine_extrinsics.py` が書く `{camera}_final.toml.tmp`）と入力パスの衝突（codex-23 指摘1、高）: その一時パスに置いた TOML を `--toml` に指定すると、最終精緻化の出力時に入力が上書きされ得る。初版の設計から存在する性質で、イテレーション2 の変更が生んだものではない。BUGFIX_STANDARD.md §2.3 に従い本イテレーションでは扱わず、別案件 **bug-006**（`docs/issues/bug-006-intrinsics-loop-refine-tmp-input-collision/`）として起票した。既存の `{camera}_final.toml.tmp` が入力 TOML を指すシンボリックリンクの場合も、`--toml` が出力先の外にあっても上書きされ得る。運用上の回避: `--out-dir` には入力と分離した新規の出力ディレクトリ（既存ファイルの無いディレクトリ）を指定し、`--toml` には `--out-dir` 配下のファイルを指定しない。design.md §9 手順 6 の正式な実行はこの条件を満たしている（codex-24）
- 評価画像の書き込み失敗の検出（codex-23 指摘2）: `evaluate_and_write` は `cv2.imwrite` の戻り値を確認しない（初版から）。コードの修正は本イテレーションでは扱わず、正式判定では `experiments/e3_10F_it2/criteria.md` §2 の項目 6（最終 TOML と評価 4 ファイルが読めること）で検出する
- ループ結果 TOML の書き込み失敗（ディスク容量不足など）の特別な扱い（codex-31 指摘3、codex-32）: 一度は「OSError を捕まえて途中のファイルを消し、終了コード 1」を設計したが、削除の失敗が元のエラーを隠す、という新しい問題が出たため、行き詰まり検出に従って取り下げた。未収束時の書き込みは、収束時の書き込み（初版から、特別な扱いなし）と同じ扱いにそろえる。書き込みに失敗した場合は Python の例外で異常終了し（終了コード 1、レポートは書かれない）、途中まで書かれたファイルが残り得る。これは初版から収束時の書き込みに存在する性質と同じで、既知の制約とする
- 同じ `--out-dir`・カメラ名での同時実行の排他（codex-20 指摘）: 2 プロセスが同時に出力済み検査を通過すると、出力が混在し得る。これは初版の設計（codex-08 ゲート通過）から存在する性質で、イテレーション2 の変更が生んだものではない。運用は GPU 1 枚で 1 回ずつ実行する形であり、同時実行は想定していない。BUGFIX_STANDARD.md §2.3（スコープの限定）に従い本イテレーションでは扱わず、既知の制約として記録する。必要になった場合は別案件で扱う

### 2.4 影響範囲

- **他の機能への影響**: `evaluate_overlay.py` の判定は、単体 CLI としても使われる（比較用の測定、design.md §9 手順 2）。判定が変わるだけで、統計値と出力画像は変わらない。過去の評価ファイル（`feat041_out/`、`baseline/`）は旧基準のまま残る
- **リグレッションリスク**:
  1. `CONV_PX` を大きくしたので、収束までの反復は少なくなる。内部パラメータの最後の揺れ（Δ_intr で最大 2px、画像の隅での値）は残る。この影響が評価の中央値に表れるとは限らない（中央値がカメラパラメータの良否を表すかは未検証。§2.2）。ずれが一部の領域に留まる場合に備えて、ステップ7 で flowmag と重ね合わせ画像を目視で確認する（codex-36 指摘5）
  2. `--overwrite` を廃止するので、同じ `--out-dir` での再実行は終了コード 1 で拒否される。再実行は新しい `--out-dir` を指定するか、利用者が前回の出力を手で削除する。これまでの実データ検証の手順（design.md §9 手順 3・5・6）は毎回別の出力先を使っており、`--overwrite` を使っていないので影響しない
  3. テスト 13、18 の期待値を変える。それ以外の既存テストは変えない

### 2.5 確認方法

- **自動テスト**: design.md §8 のテスト 13、18 の変更と、テスト 19、20、21 の追加、テスト 7c の記述の変更。`uv run pytest -v` の全件実行結果を `tests/results/feat-041_test_result.txt` に上書き保存する
- **正式な判定の実行者**: design.md §9 手順 6 の実行は、CLAUDE.md ステップ6 の実データ検証として、実装担当（Sonnet サブエージェント）が行う。その 1 回の実行を `criteria.md` に基づく正式な判定の結果とする（codex-17 指摘6）
- **ステップ7（ユーザーの手動テスト）**: ユーザーは、ステップ6 の出力（`phase4/data/10F/feat041_out_it2/`）のうち次を確認する。(1) `s25_20261124_152436_eval.txt` の判定と値、(2) `s25_20261124_152436_eval_checker.png`・`_eval_blend.png`・`_eval_flowmag.png` の目視（局所的に大きなずれが集まる領域が無いか。特に画像の隅）、(3) `s25_20261124_152436_final.toml` を、ユーザーが使っている 3DGS ビューア（TOML をカメラとして取り込むプラグイン）に取り込み、3DGS に写り込んだ三脚上の S25 のレンズ位置と比べる（参考の確認で、真値ではない）。ユーザーが問題なしと判断した時点で手動テストを完了とする。問題があれば investigation.md に次のイテレーションを追記する（codex-18 未解消6）
- **判定基準の事前確定（criteria lock）**: 判定基準を `experiments/e3_10F_it2/criteria.md` に書いた（問い、終了状態、valid_ratio ≥ 0.10、median ≤ 1.0px、判定の記載、全体 60 分以内、成果物（最終 TOML と評価 4 ファイル）の読み取り。すべて満たせば合格）。この criteria.md を本イテレーションのドキュメントと一緒に Codex レビューにかけ、高・中ゼロに収束させる。収束前に実データを実行しない（codex-14 指摘1）
- **実データ検証（ステップ6。design.md §9 手順 6）**: 手順 1・2 は再実行しない。手順 3 を出力先 `phase4/data/10F/feat041_out_it2`、中間 `phase4/data/10F/feat041_tmp_it2` で、`/usr/bin/time -v` を付けて実行する
  - 合格ライン: `experiments/e3_10F_it2/criteria.md` §2 のすべてを満たすこと（変更後の requirements.md §7 の合格ラインと NFR-001 を含む）
  - この実行で確認できるのは、固定した新仕様を実装が満たすことである。新しい基準そのものの妥当性は検証しない
  - 事前確認（bug-006 の回避条件）: 出力先 `phase4/data/10F/feat041_out_it2` と中間ディレクトリ `phase4/data/10F/feat041_tmp_it2` がどちらも存在しないこと（中間ディレクトリの `iter00_render.png` などへの書き込みが、残っていたリンク経由で入力を上書きし得るため。codex-31 指摘1）。出力先（シンボリックリンクとしても存在しないこと。design.md §9 手順 6 の `test ! -e` と `test ! -L`）、入力ファイルがすべてその外にあること。満たさなければ正式な実行を開始しない（design.md §9 手順 6。codex-26 指摘1）
  - 記録: `exit_code=0` で初期化し、コマンドの末尾に `|| exit_code=$?` を付け、直後に `printf 'EXIT_CODE=%s\n' "$exit_code"` で表示した値を正式な終了コードとする（design.md §9 手順 6。errexit が有効でも表示まで到達させるため。codex-28）（パイプ・ログファイルは使わない。codex-27）。実行後、criteria.md §2 の全 6 項目について、実測値または確認結果（終了コードは `EXIT_CODE=` の値、所要時間は Elapsed (wall clock) time、`valid_ratio_raw`・`median_raw`・`n_valid` は eval.txt、成果物の読み取り結果）と総合判定（合格／不合格）を `experiments/e3_10F_it2/experiment_log.md` に記録する。`EXIT_CODE=` の行、`/usr/bin/time -v` の出力全体、`Command terminated by signal` の行の有無は、コマンドの出力からそのまま実験ログに書き写す（codex-26 指摘2、codex-27）
  - CLAUDE.md の実験プロトコルに従い、実行直前に予測値を、実行後に実測値を `experiments/e3_10F_it2/experiment_log.md` に記録して照合する。予測と照合の項目は `experiments/e3_10F_it2/criteria.md` §3 の一覧（収束する反復、最終 f・cx・cy・k1・k2、最終精緻化の受理の種類、評価の median・p95・valid_ratio、全体の所要時間）をそのまま使う
  - 途中で失敗した場合（codex-31 指摘4）: 到達しなかった項目（収束した反復、精緻化の受理の種類、評価値など）は、理由を付けて「未実行」または「取得不能」と実験ログに記録し、総合判定は「不合格」とする
  - ステップ7 への遷移（codex-32 で統一）: 総合判定が不合格なら、予測との乖離の有無にかかわらず、正式な実行の再試行やステップ7 の成果物確認には進まず、原因を調べて investigation.md に次のイテレーションを追記する。総合判定が合格の場合だけステップ7 に進む。ただし予測と実測が乖離した項目があれば、その原因を特定して実験ログに記録してから進む。原因を特定できない場合は、総合判定が合格でもステップ7 への遷移を保留し、ユーザーに報告する（CLAUDE.md 実験プロトコルの照合。codex-36 指摘2）
  - 参考実行（`feat041_out_it1_dump/conv2/`、median 0.671px、p95 4.085px）と比較した表を残す。正式な実行は参考実行と同じ処理なので、値はほぼ一致する見込みである。一致しない場合は原因を調べる

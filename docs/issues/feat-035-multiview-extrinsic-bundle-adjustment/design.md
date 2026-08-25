# feat-035 機能設計書: 8台多視点同時外部パラメータ調整

作成日: 2026-08-17 / ステータス: Draft（Codexレビュー前）

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 MASt3R CLI | §2.1 |
| FR-002 ブートストラップ | §2.3 |
| FR-009 各カメラの反復精緻化（Stage R）【廃止】 | §2.35（廃止記録） |
| FR-010 エピポーラ整合フィルタ | §2.45 |
| FR-003 アンカー対応点 | §2.4 |
| FR-004 クロス対応点とペア選定 | §2.2 |
| FR-005 バンドル調整 | §2.5 |
| FR-006 ホールドアウト評価とレポート | §2.6 |
| FR-007 出力TOML | §2.7 |
| FR-008 パラメータ決定実験 | §3 |

## 1.2 システム構成

```
phase4/adjust_extrinsics_multiview.py   # 本体（新規）。Stage P→B→A→D→E を順に実行
matcher_lab/mast3r_cli.py               # MASt3R ペアマッチング CLI（新規。subprocess で呼ばれる）
phase4/refine_extrinsics.py             # 流用元（変更しない）: render_depth_alpha_distorted,
                                        #   sample_depth_bilinear, depth_variance_map, gate_breakdown,
                                        #   call_matcher, _format_camera_section, pose_diff,
                                        #   GATE_* / NEAR_PLANE 定数, sched（ANCHOR_DISP_TAU_PX の出典）
                                        #   （write_output_toml は入力TOML全カメラを書くため流用しない）
phase4/render_keypoints.py              # 流用元（変更しない）: load_cameras_toml, select_camera, render_image
phase4/render.py                        # 流用元（変更しない）: load_ply
```

依存方向: `adjust_extrinsics_multiview.py` → `refine_extrinsics.py` / `render_keypoints.py` / `render.py`（一方向。逆参照なし）。`mast3r_cli.py` は独立プロセス（matcher_lab 環境）で、本体から subprocess 起動のみ。torch / gsplat の import は関数内遅延 import とし、ルート環境の pytest から torch 非依存で単体テスト可能に保つ（refine_extrinsics.py と同方針）。

処理ステージ（本体内の関数分割）:

```
Stage P: ペアマッチング     … 28ペア全てに mast3r_cli.py を実行、採用ペア選定 (FR-004)
Stage F: エピポーラフィルタ … 確定ポーズでクロス対応点の偽マッチを除去、採用ペア再判定 (FR-010)
Stage A: アンカー対応点     … 各ポーズ確定カメラで レンダ→LoFTR→深度リフト (FR-003)
Stage D: バンドル調整       … scipy least_squares による同時最適化 + 非悪化フォールバック (FR-005)
Stage E: 評価・出力         … ホールドアウト評価、レポート、TOML出力 (FR-006/007)
（Stage B〔ブートストラップ〕は案Aにより主経路外。コードは温存するが main() から呼ばない。Stage R は廃止）
```

## 1.3 技術スタック

- Python 3.10（phase4）/ Python 3.12（matcher_lab）。パッケージ管理は uv（両環境とも既存 pyproject.toml。**新規ライブラリ追加なし**）
- NumPy / SciPy `optimize.least_squares`（method='trf', loss='huber', jac_sparsity 指定。選定理由: LM は疎ヤコビアン非対応であり、本問題は未知数（8×6 + 3M 点）に対し残差ブロックが疎なため trf が必須）
- OpenCV（solvePnPRansac, triangulatePoints, projectPoints, undistortPoints, Rodrigues）
- gsplat 1.5.3（アンカー用レンダ。feat-026 と同一経路）
- MASt3R（`~/git/mast3r`、チェックポイント `~/data/models/mast3r/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth`。実装は m3b の matcher_cli.py の mast3r 経路を移植）
- kornia LoFTR（アンカー対応点。refine_extrinsics.call_matcher → matcher_lab/loftr_cli.py の既存経路をそのまま呼ぶ）

## 2. 各機能の詳細設計

### 2.0 共通データ構造・定数

カメラ辞書は `load_cameras_toml()` の戻り値（`K`(3,3) float64, `D`(4|5|8,) float64, `rvec`(3,), `tvec`(3,), OpenCV world-to-camera 規約）を基に作る。**マージ規則**: `--toml`（intrinsics_all.toml）からは K・D・画像サイズのみ採用し、同TOMLに書かれている rotation/translation は**読み捨てる**（別座標系の値である可能性があるため初期値に使わない）。`rvec`/`tvec` は「`--init-toml` に存在し、**かつ `--init-cameras` に指定された**カメラ」についてのみ `--init-toml` の値をコピーし、それ以外のカメラは必ず None とする（None のカメラが Stage B の対象）。`--init-cameras` 省略時は `--init-toml` の全カメラを指定したものとみなすが、**省略が許されるのは「全対象カメラ分のみを含む受理済みTOML」を使う場合に限る**（inuyama 主経路では明示指定必須。§2.7）。**注意**: feat-026 の出力TOML（`write_output_toml` 生成）は入力TOMLの全カメラのセクションを含み、精緻化されなかったカメラには無効なポーズが残るため、これを `--init-toml` に使う場合は `--init-cameras` の明示指定が必須（requirements FR-002）。`--init-cameras` に指定されたカメラが `--init-toml` に存在しない場合はエラーメッセージを出し終了コード1。

本体冒頭に定義する定数（FR-008 の実験対象は各コメントに「実験で決定」と明記。固定流用値は出典を併記）:

```python
N_PAIR_MIN = 50        # 採用ペアの最小マッチ数（初期値。実験で決定）
N_BOOT_MIN = 30        # ブートストラップPnPの最小 2D-3D 対応数（refine_extrinsics N_MIN と同値）
BOOT_RANSAC_PX = 8.0   # ブートストラップPnP RANSAC 閾値（refine_extrinsics sched(0) と同値）
BOOT_INLIER_MIN = 30   # ブートストラップPnP の最小 inlier 数
HUBER_PX = 2.0         # Huber f_scale [px]（p1_param_sweep で決定。当初 1.0 採用も反映後再検証で
                       # W_CROSS=2.0 との相互作用による BA 停滞が発覚し、既定値 2.0 に改訂。
                       # 2026-08-25 ユーザー承認済み。criteria §4b・experiment_log 参照）
W_ANCHOR = 1.0         # アンカー項重み（p1_param_sweep で既定値維持を確認）
W_CROSS = 2.0          # クロス項重み（p1_param_sweep で決定。2026-08-25 採用・ユーザー承認済み）
HOLDOUT_RATIO = 0.2    # ホールドアウト比率（初期値。実験で決定）
SEED_DEFAULT = 5000    # ホールドアウト分割の乱数シード（--seed で変更可）
ANCHOR_MAX_PTS = 3000  # カメラあたりアンカー対応点の上限（超過時は conf 降順で切詰め。BA規模の抑制）
MAX_NFEV = 1000        # least_squares の最大評価回数（改訂 2026-08-24: x_scale='jac' 前提でも
                       # inuyama 実測 nfev=270 で収束するため、収束余裕を持って 200→1000 に引き上げ）
TRI_MIN_ANGLE_DEG = 2.0  # 三角測量点の最小交会角[度]。未満の点は BA・評価から除外
Z_TRI_RANGE = (0.1, 10.0)  # 三角測量点の許容深度[m]（両カメラとも）。範囲外は除外
N_ANCHOR_MIN = 30      # BA参加に必要な最小アンカー対応点数（FR-005 のBA参加条件）
BA_DEGRADE_TOL_PX = 0.3  # 非悪化フォールバックの許容悪化幅[px]（FR-005）
THETA_OPT_DEG = 30.0   # 交会角重みの最適角[度]（VISAPP2026。交会角重み無効化採用により主経路では未使用。
                       # crossing_angle_weight 関数と単体テストは温存）
THETA_SIGMA_DEG = 15.0 # 交会角重みのガウス幅[度]（同上。主経路では未使用）
EPI_TOL_PX = 3.0       # エピポーラ整合フィルタの許容誤差[px]（初期値。実験で決定。FR-010）
ANCHOR_DISP_TAU_PX = sched(1)["tau_px"]  # =10.0。Stage A の gate_breakdown cond_disp 許容画素変位[px]
                       # （refine_extrinsics の収束後反復の値を援用。§2.4。FR-008 の実験対象外）
```

ゲート定数・NEAR_PLANE・`sched` は `refine_extrinsics` から import して使う（GATE_ALPHA=0.5, GATE_Z_RANGE=(0.5,10.0), GATE_VAR_WINDOW=5, GATE_VAR_REL=0.02, NEAR_PLANE=0.5。再定義しない。`sched` は ANCHOR_DISP_TAU_PX の出典として使用）。

### 2.1 MASt3R ペアマッチング CLI（FR-001）

- **ファイル**: `matcher_lab/mast3r_cli.py`
- **実行**: `uv run --project matcher_lab python matcher_lab/mast3r_cli.py <imgA> <imgB> --out <out.npz> [--resolution full|1280x720]`
- **入力**: PNG 2枚（常に原寸 1920x1080。それ以外のサイズはエラーメッセージを出し終了コード1）。`--resolution` 既定 full。`1280x720` 指定時は CLI 内部で 1280x720 に縮小してから推論し、マッチ座標を原寸へ逆写像する（呼び出し側は縮小画像を作らない）
- **出力 NPZ**: `u_a`, `u_b` (N,2) float64 原寸画素座標 / `conf` (N,) float32（MASt3R は信頼度を返さないため 1.0 埋め）/ `meta` JSON 文字列（元解像度、処理解像度、縮小スケール係数、逆写像 round-trip 最大誤差[px]、原寸範囲内率、処理時間[s]）
- **処理**: m3b `matcher_cli.py` の `run_mast3r` 相当を移植する。処理解像度（full または 1280x720）→ MASt3R 内部解像度（512）へのリサイズ→推論→原寸座標への逆写像。移植時に query/render の変数名を imgA/imgB に改名し `--resolution` を追加する以外のロジック変更をしない。マッチングは MASt3R 標準の相互最近傍（reciprocal NN）であり、先行研究（VISAPP2026）のサイクル整合性フィルタのペア版に相当する選別が組み込みで効いている（prior_work.md §2。追加の往復フィルタは実装しない）。
- **エラーハンドリング**: CUDA OOM を捕捉したら終了コード42で終了（stderr に "CUDA OOM" を出力）。チェックポイントファイル不在は終了コード1（stderr にパスを出力）。
- **境界条件**: マッチ0点でも N=0 の NPZ を出力し終了コード0（採否判定は本体側の責務）。

### 2.2 Stage P: ペアマッチングとペア選定（FR-004）

データフロー: 実写8枚（`<images_dir>/<camera_name>.png`）→ 28ペアの NPZ（`<tmp_dir>/pair_<camA>_<camB>.npz`。camA < camB の辞書順）→ 採用ペアリスト。

処理ロジック:
1. TOML 記載カメラのうち画像ファイルが存在するものを対象カメラとする（8台未満でも続行。1台以下ならエラー終了コード1）
2. 対象カメラの全ペア（辞書順）について `mast3r_cli.py` を subprocess 実行する。既に NPZ が存在する場合はスキップして再利用する（再実行の高速化。`--fresh-match` 指定時は無視して再実行）
3. 終了コード42（OOM）の場合、`--resolution 1280x720` を付けて1回だけ再試行する（画像は原寸のまま渡す。縮小・逆写像は CLI 内部の責務）。再度失敗したらそのペアはマッチ0点として扱い、レポートに "OOM" と記録する
4. 終了コード1の場合はそのペアをマッチ0点として扱い、レポートに "ERROR" と記録する（全体は継続）
5. マッチ数 N ≥ N_PAIR_MIN のペアを採用ペアとする。採用ペア数0ならエラーメッセージを出力し終了コード1

境界条件: 画像が1枚しかない場合はペアが作れないためエラー終了コード1。

### 2.3 Stage B: ブートストラップ（FR-002。**主経路外・コード温存**）

> **改訂（2026-08-18 案A）**: 対称環境の偽マッチによる誤収束（experiments/p0_bootstrap）のため、本ステージは main() の主経路から呼び出さない。実装済みの `gate_depth_quality` / `bootstrap_cameras` とその単体テストは温存する（非対称シーン向けの将来再接続は別案件）。main() は、対象カメラに `--init-cameras` 外のカメラが存在する場合エラーメッセージを出し終了コード1とする。以下は温存コードの仕様（参照用）。

データフロー: §2.0 のマージ後カメラ辞書 → 較正済み集合 S =「`rvec`/`tvec` が None でないカメラ」（＝ `--init-toml` に存在しかつ `--init-cameras` で信頼指定されたカメラ。1台以上必要、0台はエラー終了コード1）。未較正カメラ集合 U =「`rvec`/`tvec` が None のカメラ」。採用ペアのクロス対応点 → 各未較正カメラの初期ポーズ。

処理ロジック（貪欲反復。終了条件: U が空になるか、1周で1台も較正できなかったとき）:
1. S×U の採用ペアのうちマッチ数最大のペア (c∈S, u∈U) を選ぶ。存在しなければ終了
2. c のポーズで `render_depth_alpha_distorted(gaussians, cam_c, NEAR_PLANE)` を実行し深度・αマップを得る
3. ペアの c 側マッチ座標を、ブートストラップ専用ゲート `gate_depth_quality` で選別する。判定式は feat-026 実装（`refine_extrinsics.py` の `run_iteration` 内）と同一とする: `pixel_valid = isfinite(depth) & (alpha > GATE_ALPHA) & (depth > GATE_Z_RANGE[0]) & (depth < GATE_Z_RANGE[1])` を作り（`pixel_valid` が0件の場合は全点 reject とし、その (c, u) ペアは試行失敗「対応点数不足」として扱う）、`d_bilin, depth_ok = sample_depth_bilinear(depth, uv)`、`cond_alpha = alpha[iy, ix] > GATE_ALPHA`、`cond_z = GATE_Z_RANGE[0] < d_bilin < GATE_Z_RANGE[1]`、`var_map = depth_variance_map(depth, pixel_valid, GATE_VAR_WINDOW)`、`tau_d = GATE_VAR_REL * median(depth[pixel_valid])`、`cond_var = var_map[iy, ix] < tau_d ** 2`、採択 = `depth_ok & cond_alpha & cond_z & cond_var`（4条件のみ。画素変位条件は含めない）。`gate_breakdown` は流用しない（同関数は query↔render の画素変位ゲートを含み、視点が異なるカメラ間のマッチには適用できないため。`gate_breakdown` はアンカー取得〔§2.4〕専用とする）。採択された c 側画素を `cv2.undistortPoints`（c の K, D）→ 正規化座標 × 深度 → c のカメラ座標 → 3DGS座標系の3D点に変換する
4. 3D点と u 側マッチ座標で `cv2.solvePnPRansac`（u の K, D、reprojectionError=BOOT_RANSAC_PX, iterationsCount=2000, flags=SOLVEPNP_ITERATIVE, useExtrinsicGuess=False）を解く
5. 対応数 < N_BOOT_MIN（「対応点数不足」）または inlier 数 < BOOT_INLIER_MIN（「PnP inlier 不足」）の場合は**試行失敗**として、u ごとの試行済みペア集合に (c, u) と失敗理由を記録するのみとする（この時点では u を失敗リストに入れない）。u の**確定失敗**は「S×{u} の採用ペアのうち未試行のものが存在しなくなった時点」でのみ判定し、失敗リストには全試行の理由を添えて記録する（較正済み集合 S は反復中に増えるため、確定失敗の判定は毎周の選択時に行う）
6. 成功したら u を S に移す

境界条件: U が最初から空になるのは「対象全カメラが `--init-cameras` の信頼対象になった場合のみ」であり、その場合に限り Stage B をスキップする（init-toml に全カメラのセクションが存在するだけではスキップ理由にならない。U の定義は上記データフローの通りマージ後の None 判定による）。S が最後まで増えない場合、S のカメラのみで以降の Stage を続行し、レポートに未較正カメラを記録する（S が1台のみの場合、クロス項が作れないためエラー終了コード1）。

### 2.35 Stage R: 各カメラの反復精緻化（FR-009）【廃止】

案A（2026-08-18）により廃止。全カメラの初期ポーズが feat-026 精緻化済みの信頼初期カメラとなり、追加の反復精緻化の対象が存在しない。`refine_cameras`・関連CLI（`--stop-after-stage-r`, `--quality-accepted-cameras`）・2段階閾値確定手順は実装しない。経緯: requirements FR-009【廃止】および experiments/p0_bootstrap/experiment_log.md を参照。

### 2.4 Stage A: アンカー対応点（FR-003）

対象は**ポーズ確定カメラ**（= `--init-cameras` で信頼指定されたカメラ）のみ。各ポーズ確定カメラ i について:
1. `render_image(gaussians, cam_i, NEAR_PLANE, distort=True)` でレンダPNGを作り、`refine_extrinsics.call_matcher`（LoFTR）で実写とマッチする。`call_matcher` は raw マッチ（全confidence）を返すだけなので、`collect_anchor_points` 側で `conf >= refine_extrinsics.LOFTR_CONF_TH`（=0.2）の選別を明示的に適用する
2. `render_depth_alpha_distorted` の深度・αマップと `gate_breakdown` でレンダ側画素を選別・3D化する（3D化手順は §2.3-3 と同一）。ゲートの構成は feat-026 実装（`run_iteration` Stage 4）と同一とする: `pixel_valid`・`var_map`・`tau_d` を同手順で作り、`gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, ANCHOR_DISP_TAU_PX)` を呼ぶ。cond_disp の許容画素変位は `ANCHOR_DISP_TAU_PX = sched(1)["tau_px"]`（=10.0px）を援用する（根拠: Stage A のマッチは確定ポーズのレンダ↔実写であり、feat-026 の収束後反復〔sched(1)〕と同条件。新規チューニング定数を増やさない）
3. 対応点が ANCHOR_MAX_PTS を超える場合は LoFTR conf 降順に切り詰める
4. N_A < N_ANCHOR_MIN のカメラは BA に参加させない（FR-005 のBA参加条件。クロス項のみでは並進スケールが平坦方向になり得るため）。当該カメラは**信頼初期ポーズ**（Step 3M の feat-026 受理済みポーズ）を最終値とし、レポートに "アンカー不足（BA未参加）" と記録する（feat-026 品質は入力時点で保証されている）。当該カメラを含む採用ペアのクロス対応点は BA・評価の対象から除外する

出力: カメラ i ごとの (X_w (N_A,3) float64, u_i (N_A,2) float64)。

### 2.45 Stage F: クロス対応点のエピポーラ整合フィルタ（FR-010）

対象: Stage P の採用ペアのうち、両端がポーズ確定カメラのペア（片端でもポーズ確定でないペアは不採用に降格しレポートに記録）。

処理ロジック（ペア (i, j) ごと）:
1. 両側のマッチ座標を `cv2.undistortPoints(uv, K, D, P=None)` で正規化座標にする
2. 確定ポーズから相対ポーズ `R_rel = R_j @ R_i.T`、`t_rel = t_j - R_rel @ t_i` を作り、エッセンシャル行列 `E = [t_rel]_x @ R_rel` を計算する
3. 各対応の Sampson 距離を正規化座標系で計算する: `sd = (x_j^T E x_i)^2 / ((E x_i)_1^2 + (E x_i)_2^2 + (E^T x_j)_1^2 + (E^T x_j)_2^2)`（x は同次正規化座標、添字1/2はベクトル成分）。画素換算は `d_px = sqrt(sd) * f_mean`（`f_mean = (f_i + f_j) / 2`、f は各カメラの (fx+fy)/2）とし、`d_px <= EPI_TOL_PX` の対応のみ残す（sd は二乗量であり、平方根を取ってから焦点距離を掛けることに注意）
4. 生存数 < N_PAIR_MIN のペアは不採用に降格する。フィルタ後の採用ペアでグラフを構築し、**全ポーズ確定カメラが単一連結成分に入らない場合はエラーメッセージ（連結成分の内訳つき）を出し終了コード1**（全ペア不採用も同様）
5. ペアごとの（フィルタ前対応数、生存数、生存率、採用再判定）をレポート (h) に記録する

境界条件: 生存0点のペアも「生存数0・不採用」として記録して継続する（エラーになるのは「全ペア不採用時」または「フィルタ後採用ペアグラフで全ポーズ確定カメラが単一連結成分に入らない時」のみ）。ホールドアウト分割（§2.6）は**フィルタ後**の対応点に対して行う。

### 2.5 Stage D: バンドル調整（FR-005）

**未知数**: BA参加カメラ（**ポーズ確定カメラ**かつ N_A ≥ N_ANCHOR_MIN）の (rvec_i, tvec_i)（6×n_cam）と、クロス対応点の3D点 Y_k（3×M）。K・D は固定。

**非悪化フォールバック（FR-005）**: 最適化終了後、各 BA 参加カメラについて自カメラのアンカー対応点への再投影残差中央値を「BA前（信頼初期＝feat-026 精緻化済み）ポーズ」と「BA後ポーズ」で比較し、悪化幅が `BA_DEGRADE_TOL_PX = 0.3` px を超えたカメラは **BA前ポーズに戻す**（当該カメラの出力・以降の評価は BA 前ポーズを用いる。レポートに「BAフォールバック」とカメラ名・悪化幅を記録）。比較に使うアンカー対応点集合は Stage A で取得したものを固定して使う（BA前後で同一集合）。

**クロス3D点の初期化**: Stage F フィルタ後の採用ペアごとに、ホールドアウト分割（§2.6）後の学習側マッチを三角測量する。座標方式は正規化座標に統一する: `cv2.undistortPoints(uv, K, D, P=None)` で正規化座標を得て、`cv2.triangulatePoints` には K を掛けない射影行列 `P_i = [R_i | t_i]`（3x4）を渡す。ピクセル座標＋`K[R|t]` 方式は使わない（混在禁止。ホールドアウト評価〔§2.6〕の三角測量も同一方式とする）。交会角 < TRI_MIN_ANGLE_DEG または深度が Z_TRI_RANGE 外の点は除外する。3視点以上のトラック統合は行わない（ペア単位の2視点点として扱う。設計判断: トラック統合は対応点の同一性判定が必要になり複雑化するため却下）。

**残差ベクトル**（この順で連結）:
- アンカー項: 各カメラ i の各点 `W_ANCHOR * (projectPoints(X_w, rvec_i, tvec_i, K_i, D_i) - u_i)` … 2×ΣN_A 成分。X_w は定数（最適化しない。設計判断: アンカー3D点を動かすと3DGS座標系への固定が緩むため定数とする）
- クロス項: 各3D点 Y_k を観測する2カメラへの `W_CROSS * w_k * (projectPoints(Y_k, ...) - u)` … 4×M 成分。`w_k` は**1固定（交会角重みの無効化。p1_param_sweep 2026-08-25 の採用・ユーザー承認済み: VISAPP2026 由来のガウス重み `w_k = exp(-(θ_k - THETA_OPT_DEG)**2 / (2 * THETA_SIGMA_DEG**2))` は本データで効果がなく、w_k=1 が中央値最小だった）**。実装は `build_cross_points` が weight=1.0 を設定する（`crossing_angle_weight` 関数・θ_k の計算・関連単体テストは将来の再有効化に備えて温存する。三角測量ゲート TRI_MIN_ANGLE_DEG は従来どおり有効）

**最適化**: `scipy.optimize.least_squares(fun, x0, jac_sparsity=S, method='trf', loss='huber', f_scale=HUBER_PX, max_nfev=MAX_NFEV, x_scale='jac')`。jac_sparsity は lil_matrix で「各残差ブロック × 対応カメラの6パラメータ」「クロス残差ブロック × 対応3D点の3パラメータ」のみ 1 を立てる。`x_scale='jac'` は必須（改訂 2026-08-24。未知数に回転[rad]と並進・3D点[m]が混在しスケールが不均衡なため、既定の x_scale=1.0 では trf が収束しない。inuyama 実データでの実測: x_scale 既定では max_nfev=3000 でも status=0〔コスト 46390 で漸近〕、x_scale='jac' では nfev=270 で status=2 収束〔コスト 46337〕。ヤコビアン列ノルムによる自動スケーリングは疎 BA の標準手法）。MAX_NFEV は `x_scale='jac'` 前提でも実測 nfev=270 のため、収束余裕を持って 1000 とする（§2.0 定数一覧と同時改訂）。

**エラーハンドリング**: least_squares が status < 1（収束せず）の場合、警告をレポートに記録し、最終反復の解をそのまま出力する（中断しない）。残差に NaN が生じた場合（3D点がカメラ背面に回った場合に発生し得る）は、当該点の残差を 1e3 px に置換する定数バリアで扱う。

境界条件: M=0（クロス点が全て三角測量ゲートで落ちた）はエラー終了コード1。n_cam（ポーズ確定カメラ数）< 2 は §2.7 のフル実行前チェックでエラー終了コード1として事前に排除する（品質フィルタ後に1台へ減るケースを含む。§2.3 の較正済み集合チェックだけでは不十分）。

### 2.6 Stage E: ホールドアウト評価・レポート・TOML出力（FR-006/007）

**ホールドアウト分割**: 採用ペアごとに `numpy.random.default_rng(SEED_DEFAULT)` でマッチをシャッフルし、床関数 `floor(N*HOLDOUT_RATIO)` 点を評価専用にする（分割は三角測量の前に行う）。

**評価**: 最適化後ポーズでホールドアウトを三角測量（§2.5 と同一のゲート）し、両カメラへの再投影残差を全点分集計する。

**レポート**（`--out-report`、テキスト）: requirements FR-006 の (a)〜(i) をこの順のセクションで出力する。ポーズ変化量は `pose_diff` を使い cm・度で出す。

**TOML**（`--out-toml`）: 専用ライタ `write_multiview_toml(intrinsics_toml_path, out_path, poses)` を新設する（既存 `write_output_toml` は入力TOMLの全カメラを書き出すため流用しない）。poses（camera_name → (rvec, tvec)）に含まれるカメラのみを、入力TOMLの K・D・サイズと最終ポーズから `refine_extrinsics._format_camera_section` を使って書き出す。poses に入れるのは「ポーズ確定カメラ」のみ: (1) BA 参加カメラの BA 後ポーズ（非悪化フォールバックが発動したカメラはフォールバック後＝信頼初期ポーズ）、(2) アンカー不足で BA 未参加のポーズ確定カメラの信頼初期ポーズ。**ポーズ確定カメラ以外は入れない**。

### 2.7 CLI（本体）

```
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/adjust_extrinsics_multiview.py \
  --toml <intrinsics_all.toml> --ply <point_cloud.ply> --images-dir <dir> \
  --init-toml <cage_refined_all8.toml> \
  --init-cameras cam05520125 cam05520126 cam05520128 cam05520129 cam41520554 cam41520556 cam41520557 cam41520558 \
  --out-toml <out.toml> --out-report <report.txt> \
  [--cameras <name>...] [--overwrite] [--seed 5000] [--tmp-dir <dir>] [--fresh-match]
```

引数の型・既定値: `--cameras` 省略時は TOML×画像の積集合全て。`--init-cameras` nargs='+'。inuyama 主経路では8台全てを明示列挙する（上記コマンド例）。省略時の「`--init-toml` の全カメラを信頼」規則は、全対象カメラ分のみを含む受理済みTOMLを使う場合に限る（feat-026 出力TOMLのような全カメラ入りTOMLでの省略は 2026-08-17 の事故〔無効ポーズの全台信頼〕を再発させるため禁止。§2.0 マージ規則）。`--seed` int 既定 5000。ポーズ確定カメラ = `--init-cameras` で信頼指定されたカメラ。対象カメラ（`--cameras` または TOML×画像の積集合）に `--init-cameras` 外のカメラが含まれる場合はエラーメッセージを出し終了コード1（案A: 全対象カメラの初期ポーズ必須）。**ポーズ確定カメラが2台未満の場合は、クロス項が構成できないため Stage F に進まずエラーメッセージを出し終了コード1で終了する**（§2.5 境界条件）。`--tmp-dir` 省略時は `<out-toml の親>/tmp_feat035/`。`--overwrite` 無指定で出力先が存在する場合はエラー終了コード1（`_check_can_write` 流用）。

## 1.5 状態遷移

GUI・常駐状態なし（バッチ一方向処理のため該当なし）。

## 1.6 ファイル・ディレクトリ設計

- 入力: §2.7 の引数で全て指定（暗黙のパス規約なし）
- 中間: `<tmp_dir>/pair_<camA>_<camB>.npz`（クロスマッチ）、`<tmp_dir>/anchor_<cam>.npz`（アンカー対応点）、`<tmp_dir>/render_<cam>.png`（アンカー用レンダ）
- 出力: `--out-toml`（Calib_scene型）、`--out-report`（UTF-8 テキスト）

## 1.7 インターフェース定義（本体の公開関数）

```python
def run_pair_matching(camera_names: list[str], images_dir: Path, tmp_dir: Path,
                      fresh: bool) -> dict[tuple[str, str], dict]   # {"u_a","u_b","n","status"}
def bootstrap_cameras(cams: dict[str, dict], pairs: dict, gaussians: dict,
                      images_dir: Path) -> tuple[dict[str, dict], list[dict]]  # (較正済みcams, 失敗記録)
def collect_anchor_points(cam: dict, gaussians: dict, query_png: Path,
                          tmp_dir: Path) -> tuple[np.ndarray, np.ndarray]      # (Xw(N,3), uv(N,2))
def build_ba_problem(cams, anchor, cross_train) -> "BAProblem"   # dataclass: x0, sparsity, unpack
def run_bundle_adjustment(problem: "BAProblem") -> "BAResult"    # dataclass: poses, status, cost
def evaluate_holdout(cams_opt, cross_holdout) -> dict            # 残差統計
def gate_depth_quality(uv: np.ndarray, depth: np.ndarray,
                       alpha: np.ndarray) -> tuple[np.ndarray, np.ndarray]  # (keep_mask, depths)
def filter_cross_matches(cams: dict, pairs: dict) -> dict
                   # Stage F。ペア別にエピポーラ整合フィルタを適用し、生存対応点と診断
                   # （フィルタ前後の点数・生存率・採用再判定）を返す（§2.45）
def write_multiview_toml(intrinsics_toml_path: str, out_path: str,
                         poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None
def main(argv=None) -> int
```

呼び出し方向: main → 各関数（循環なし）。

## 1.8 ログ・デバッグ設計

- 標準出力（print、日本語）: Stage 開始/終了、ペアごとのマッチ数、ブートストラップの成否、BA の初期/最終コストと反復数。DEBUG レベル分けは導入しない（既存スクリプトと同方針）
- 失敗の詳細（subprocess の stderr 末尾2000文字）はレポートにのみ記録する

## 3. パラメータ決定実験（FR-008）の進め方

実装完了・自動テスト通過後、`experiments/p1_param_sweep/` に criteria 文書を作成する。criteria には「本実験は合格判定ではなく基準値の測定をゴールとする」ことを明記し、(1) N_PAIR_MIN・HUBER_PX・W_CROSS/W_ANCHOR・HOLDOUT_RATIO・THETA_SIGMA_DEG（交会角重みの無効化 `w_k=1` を含む）・EPI_TOL_PX（Stage F の許容誤差。フィルタ後生存率とホールドアウト残差の両方を記録）の走査範囲、(2) 各設定で記録する測定値（ホールドアウト残差の中央値・90%点、収束反復数）、(3) 採用値の決め方（ホールドアウト残差中央値が最小の設定を採用。同率は既定値優先。ただしホールドアウト集合の構成を変えるパラメータ〔EPI_TOL_PX・N_PAIR_MIN〕は、連結性成立かつ採用ペア数・ホールドアウト点数がベースライン以上の候補に限定して比較する coverage-aware 規則とし、HOLDOUT_RATIO は比較不能のため既定値を維持して感度記録のみとする。one-at-a-time 走査で決めた採用値の組み合わせは反映前に**結合検証**〔収束 status≥1 かつ中央値が単独最良値+0.01px 以内。不合格時は単独改善幅最小のパラメータから既定値に戻す〕を行う — p1_param_sweep criteria §4b の追補 2026-08-25）を事前定義し、Codex レビュー（criteria lock）後に実施する。また、アンカー対応点のマッチャーを LoFTR から MASt3R に替えた場合の比較（GS-CPR が MASt3R を採用している先例に基づく。prior_work.md §1）を任意の追加実験項目として criteria に含めてよい（採否は criteria 作成時に決める）。実験の実行・記録は CLAUDE.md「実験・検証の進め方」に従う。

## 4. テスト設計

- `tests/test_feat035_multiview.py`（ルート環境、torch 非依存）:
  - ペア列挙・採用判定（N_PAIR_MIN 境界: N=49 不採用 / N=50 採用）
  - `--init-cameras` によるマージ規則（全カメラを含む init-toml でも指定カメラのみポーズを持ち他は None / 指定カメラが init-toml に無い場合はエラー / 省略時は init-toml 全カメラを信頼〔省略の許容条件は §2.0: 全対象カメラ分のみを含む受理済みTOML使用時に限る〕）
  - ブートストラップの貪欲順序と確定fail条件（モック対応点で検証）
  - ホールドアウト分割の再現性（同一シードで同一分割）と比率（floor 挙動）
  - BAProblem の jac_sparsity 形状（残差数×未知数、非ゼロ位置）
  - 残差 NaN 置換バリア（背面点で 1e3 px になる）
  - 出力TOMLの往復（write_multiview_toml → load_cameras_toml で読み戻し値一致）
  - write_multiview_toml が poses に含まれるカメラのみ出力する（含まれないカメラのセクションが存在しない）
  - gate_depth_quality が画素変位に依存しない（視点の異なる対応でも深度品質のみで判定される）
  - 三角測量の座標方式（正規化座標 + `[R|t]`。既知の合成ポーズ・3D点で往復一致）
  - 交会角重みの計算（θ=THETA_OPT_DEG で w=1、θ=TRI_MIN_ANGLE_DEG と θ=90° で単調減衰）
  - Stage F エピポーラフィルタ（合成ポーズ・合成対応点で: エピポーラ整合対応が生存 / 対称偽マッチ相当の外れ対応が除去される / 換算誤差 3px 相当は通過・4px 相当は除去の境界 / 生存数 < N_PAIR_MIN でペア不採用降格 / 連結性不成立でエラー / 全ペア不採用でエラー）
  - `--init-cameras` 外の対象カメラが存在する場合のエラー、ポーズ確定カメラ2台未満のエラー
  - 非悪化フォールバック（悪化 0.31px → BA前ポーズに復帰 / 悪化 0.29px → BA後ポーズを維持）
  - BA参加条件の境界（N_A=29 は BA 除外・信頼初期ポーズ維持 / N_A=30 は BA 参加）
- 実行: `uv run pytest -v`、結果は `tests/results/feat-035_test_result.txt` に保存
- 動作確認（GPU、実データ）: inuyama 8台で本体を実行し、レポート (a)〜(i) の出力と `render_keypoints.py --no-keypoints --distort` による出力TOML読込レンダ成功を確認する

## 5. 設計判断の記録（ADR簡易版）

| 判断 | 採用 | 却下と理由 |
|---|---|---|
| クロスマッチャー | MASt3R（広基線） | LoFTR: 対面カメラの広基線で実績なし（feat-026 はレンダ↔実写の近視点のみ）|
| ゲージ固定 | アンカー項（3DGS座標系） | 1台固定: 3DGS座標系との整合が壊れ、最終用途（3DGS内へのキーポイント描画）に反する |
| アンカー3D点 | 定数（最適化しない） | 可変: 座標系固定が緩み、3DGSの局所誤差に引きずられる |
| 最適化手法 | trf + jac_sparsity + huber | lm: 疎行列非対応で点数がスケールしない。手動BA実装: 車輪の再発明 |
| トラック統合 | しない（ペア単位2視点） | 統合: マッチの同一性判定が必要になり複雑。効果はホールドアウト評価で不足が見えてから検討 |
| 手動基準点 | 本スクリプトの自動処理では不使用（拘束・評価・スクリプト内初期値生成）。例外: 外部生成・受理済みの TOML を --init-cameras 指定するのは可（案Aの主経路。Step 3M） | 全用途で完全禁止: 案Aの初期ポーズ確定手段（手動プロット→refine_extrinsics.py）を塞いでしまう |
| アンカー不足カメラ | BA除外（信頼初期ポーズ〔Step 3M の feat-026 受理済みポーズ〕を最終値に） | クロス項のみでBA参加: 2視点ペア点のみでは並進スケールが観測不能になり得る（Codexレビュー高1） |
| OOM時の縮小 | CLI 内部で `--resolution 1280x720`（縮小・逆写像はCLIの責務） | 呼び出し側で縮小PNG生成: CLI が原寸以外を拒否する仕様と矛盾（Codexレビュー高3） |
| 出力TOMLライタ | 専用 `write_multiview_toml` 新設 | `write_output_toml` 流用: 入力TOML全カメラを書くため失敗カメラ除外（FR-007）を満たせない（Codexレビュー高2） |
| クロス項の交会角重み | 【改訂 2026-08-25】w_k=1 固定（p1_param_sweep で無効化が中央値最小となり採用・ユーザー承認済み。`crossing_angle_weight` 関数と単体テストは将来再有効化用に温存） | 当初採用のガウス型スコア（最適30°。VISAPP2026 準拠）: 本データで効果なし。ハード閾値のみ: 三角測量ゲート TRI_MIN_ANGLE_DEG として維持 |
| Stage R の挿入（BA前の反復精緻化） | 【その後廃止】案A採用で対象消滅（下記） | 粗ブートストラップ→直接BA（旧計画）: 2026-08-17 実測で初期化ずれ 15〜43px |
| 初期ポーズの取得経路（案A。2026-08-18） | 全カメラを手動プロット + refine_extrinsics.py で確定し `--init-cameras` に与える | 自動ブートストラップ主経路: 対称環境で MASt3R が対称構造を偽マッチし全6台が誤収束（experiments/p0_bootstrap 追加診断）。検知強化（案B）も正解が得られる保証がなく却下 |
| クロス対応点の事前フィルタ | エピポーラ整合（Sampson距離、確定ポーズ基準）で偽マッチ除去 | フィルタなし: 対称偽マッチが BA のクロス項を汚染する（同診断で inlier 率 10〜37% を実測） |
| BA 非悪化フォールバック | アンカー残差中央値の悪化 >0.3px でカメラ単位で BA 前ポーズに復帰 | フォールバックなし: BA が精緻化結果を悪化させ得ると本機能を使う意味がない（ユーザー要求） |
| ブートストラップ順序 | マッチ数最大の貪欲選択 | 3-cycleスコアによる順序付け（VISAPP2026）: 3視点トラックの構築が必要になり「トラック統合しない」方針と矛盾。効果はFR-008後に不足が出たら検討 |
| グローバル初期化（MASt3R-SfM/VGGT） | 不採用（Won't 維持） | ブートストラップ連鎖失敗時の代替として prior_work.md に記録済み。本案件では貪欲ブートストラップで十分か実データで先に検証する |
| 実験フォルダのコード | 移植（mast3r_cli.py 新設） | 直接 import: 実験フォルダは本番の依存先にしない（プロジェクト運用） |
```

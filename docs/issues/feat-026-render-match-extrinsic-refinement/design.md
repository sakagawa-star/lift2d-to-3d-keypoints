# feat-026 機能設計書: レンダリング＋自動マッチングによるカメラ外部パラメータ精緻化

- 準拠: `docs/DESIGN_STANDARD.md`。要求は `requirements.md`（同フォルダ）
- 状態: draft（Codex 再帰レビュー収束 + 人レビュー前）
- 方針: 実験コード（`experiments/` 配下）は**流用せず本設計書から書き直す**（roadmap M5）。実験コードは実測根拠・参照実装として参照名を各所に付記する
- 本書の定数はすべて criteria lock 下の実験（M2〜M4-2）で実証された値であり、**実装者が値を選ぶ余地はない**（実装判断ゼロの原則）

## 1. 対応要求マッピング

| 要求ID | 対応する設計節 |
|---|---|
| FR-001 入力検証 | §4.1 |
| FR-002 6点PnP | §4.2 |
| FR-003 反復リファイン | §4.3（定数表 §3.1） |
| FR-004 サンプリング | §4.4 |
| FR-005 合意判定と受理 | §4.5 |
| FR-006 前提条件・診断 | §4.6 |
| FR-007 結果出力 | §4.7 |
| FR-008 バッチ処理 | §4.8 |
| FR-009 マッチャー CLI | §4.9 |
| FR-010 実行ログ | §7 |
| NFR-003 再現性 | §3.2（seed 規則） |
| NFR-005 オフライン | §4.9（重みのローカル読み） |

## 2. システム構成

```
[phase4 環境 (Python 3.10)]                     [matcher_lab 環境 (Python 3.12)]
 refine_extrinsics.py                            loftr_cli.py
  ├ Stage 0: 入力検証                              ├ kornia LoFTR(outdoor) 推論
  ├ Stage 1: 6点PnP 初期値                          ├ 1280x720 縮小 + 逆写像
  ├ チェーン A/B/C × サンプリング                    └ NPZ 出力(u_q, u_r, conf, meta)
  │   └ 反復1回 = レンダ(gsplat 方式A)
  │        → [subprocess] loftr_cli.py 呼び出し ──── PNG 2枚 / NPZ 受け渡し
  │        → ゲート → 深度リフト → PnP
  ├ pooled 合意判定(60サンプル) → 受理/仲裁
  └ 出力: 精緻化 TOML + 診断レポート
```

- 2環境の連携は subprocess + ファイル受け渡し（PNG/NPZ）のみ。共有メモリ・ソケットは使わない（M3b/M4-2 で実証済みの構成）
- 既存 phase4 モジュールの再利用: `render.py` の `load_ply`、`render_keypoints.py` の `load_cameras_toml` / `select_camera` / `render_image`。**これらは変更しない**

## 3. 技術スタック

| 環境 | 依存（バージョンは各 uv.lock 固定値） | 用途 |
|---|---|---|
| phase4 | torch 2.10+cu128 / gsplat 1.5.3 / opencv-python / numpy / tomli | レンダ・ゲート・PnP・クラスタ分析・TOML 入出力 |
| matcher_lab | torch 2.13+cu130 / kornia 0.8.3 / opencv-python / numpy | LoFTR 推論 |

- 新規ライブラリの追加はなし（両環境とも導入済み。`docs/TECH_STACK.md` 参照）
- gsplat 実行時は環境変数 `TORCH_CUDA_ARCH_LIST="9.0+PTX"` が必要（CLAUDE.md 既知制約）

### 3.1 パイプライン定数（全数値固定。出典付き）

| 定数 | 値 | 出典（実証） |
|---|---|---|
| レンダ near_plane | 0.5 | M2〜M4-2（`stage2_smoke_test.NEAR_PLANE`） |
| レンダ distort | True（歪み付き直接レンダ） | 同上 |
| 深度取得 | 方式A（3DGUT 2パス: z を色として流しαで正規化） | findings §2（gsplat RGB+ED 非対応のため） |
| 引き締めスケジュール | k=0: τ_px=250.0, ransac_px=8.0 / k≥1: τ_px=10.0, ransac_px=2.0 | M2（`pipeline_loop.SCHEDULE`） |
| GATE_ALPHA | 0.5 | M2 |
| GATE_Z_RANGE | (0.5, 10.0) m | M2 |
| GATE_VAR_WINDOW | 5（深度分散マップの窓） | M2 |
| GATE_VAR_REL | 0.02（τ_d = 0.02 × 有効画素深度中央値） | M2（M4-1 で再較正 No-Go=この値のまま） |
| 深度参照 | bilinear 補間。2×2近傍に NaN または深度広がり（min-max）が中心深度の 5% 超なら除外 | findings §2（`sample_depth_bilinear`） |
| N_MIN | 30 | M2/M3/M4 |
| LoFTR 重み | outdoor（`~/.cache/torch/hub/checkpoints/loftr_outdoor.ckpt`、sha256 `21f5bec5968178e8bc8b7633441836fe5de4f47d861dd2cd7dc38e271b0479ec`） | M3b criteria §3.1 |
| LoFTR 入力解像度 | 1280x720（原寸 1920x1080 から cv2.INTER_AREA 縮小。full は本構成で OOM 実測のため使わない） | M3b フェーズ0 実測 |
| 座標逆写像 | 画素中心規約 orig=(small+0.5)/scale−0.5、scale=1280/1920=2/3 | M3b `matcher_cli.py` |
| confidence 閾値 | 0.2（conf ≥ 0.2 を採用） | M3b/M4-2 フェーズ1 選択規則の採用値 |
| 逆写像検証 | round-trip 全点 ≤ 0.5px・原寸範囲内率 100%（違反は即エラー） | M3b criteria フェーズ0(c) |
| 6点PnP | solvePnPRansac(iterationsCount=1000, reprojectionError=8.0, SOLVEPNP_ITERATIVE) → inlier で solvePnP(ITERATIVE, useExtrinsicGuess=True) | M4 §5-1 |
| 反復内 PnP | solvePnPRansac(iterationsCount=1000, reprojectionError=sched(k).ransac_px, SOLVEPNP_ITERATIVE) → 同精密化 | M2 |
| チェーン | A=無摂動 / B, C=5cm・2° 摂動（numpy default_rng seed 11, 12。`perturb_pose` と同一の方向一様・大きさ固定摂動） | M3 §5.2 / M4 §5-2 |
| サンプリング回数 N_S | 20（チェーンあたり。k=0 の後、k=1 条件で20回） | M3-1 |
| クラスタ半径 | R_POS=0.02 m / R_ANG=0.30° | M3-1 |
| 受理（単峰） | f_c ≥ 0.7 かつ第2クラスタ占有率 < 0.2 | M3-2 |
| 受理（二峰仲裁） | 第1+第2 合計占有率 ≥ 0.9 かつ手動6点再投影中央値の峰間差 ≥ 1.0px | M4 criteria §6.2 |
| 前提条件 | 初期解→最終解の差 ≤ 0.50 m かつ ≤ 10° | M4 criteria §6.4（M2 実測収束域） |
| 警告閾値（記録のみ） | RMSE>2.0px / λ3/λ1<0.09 / 深度レンジ<1.4m / 凸包面積比<0.22 | M3 §5.5 |

### 3.2 乱数 seed 規則（NFR-003 再現性）

| 用途 | 規則 |
|---|---|
| チェーン B/C の摂動 | numpy `default_rng(11)` / `default_rng(12)`（固定） |
| RANSAC（cv2 グローバル RNG） | チェーン ci（0..2）・サンプル j（0..N_S）に対し `cv2.setRNGSeed(seed_base + ci*25 + j)`。seed_base の既定値は 5000（CLI で変更可） |
| 6点PnP | seed 設定なし（プロセス開始直後の cv2 既定 RNG 状態で実行。M4/M4-2 と同一条件。6点の RANSAC は inlier 集合が安定なため結果は決定的——M4-2 フェーズ0 で再現一致を実証） |

- 複数カメラのバッチでは、カメラ index mi を加えて `seed_base + mi*100 + ci*25 + j` とする（ci*25+j の最大値 70 < 100 のためカメラ間で重複しない）

## 4. 各機能の詳細設計

### 4.1 入力検証（FR-001）

- **データフロー**: CLI 引数（§6）→ 各ファイルの読み込み → 検証 → 失敗時は該当メッセージで exit 1
- **処理ロジック**:
  1. TOML を `load_cameras_toml` で読み、対象カメラ集合を決める（`--cameras` 指定があればその集合、なければ TOML 内の全カメラのうち 2D点 CSV に当該カメラ名の行がある全カメラ）
  2. カメラごとに: 画像 `{images_dir}/{camera_name}.png` を読み、shape が TOML の (height, width) と一致することを確認。原寸が (1080, 1920) 以外なら拒否（requirements 制約）
  3. 2D-3D 対応点を ObjectName で突合し、**一意性を明示検証**する（codex-66 指摘対応）:
     - 3D CSV: ObjectName の重複があればエラー（重複名を列挙）
     - 2D CSV: (camera_name, ObjectName) の重複があればエラー（重複行を列挙）
     - 突合結果（交差集合）が**一意な ObjectName ちょうど6個**でなければエラー（不足/過剰の名前を列挙）
     - 全座標が有限値（NaN/inf 拒否）、2D 座標が画像範囲 [0, W-1]×[0, H-1] 内、必須列（ObjectName, X, Y, Z / ObjectName, camera_name, X, Y）の欠損なし
- **エラーハンドリング**: 各失敗は「カメラ名: 理由（実測値と期待値）」の形式でレポートに記録。ファイル欠落・PLY 読込失敗・CSV の重複/欠損列は全体エラーで exit 1、カメラ単位の不整合（画像サイズ・点数・座標範囲）は当該カメラのみ失敗扱いで続行
- **境界条件**: 「重複1件+欠損1件で点数が6のまま」のケースは上記の一意 ObjectName 検証で検出される（点数カウントだけでは通ってしまうため一意性検証を必須とする）

### 4.2 6点PnP 初期値（FR-002）

- **処理ロジック**: §3.1 の「6点PnP」行のとおり2段で解く。inlier < 4 または RANSAC 失敗で当該カメラ失敗（失敗段「6点PnP」）
- **出力**: rvec0, tvec0（float64 flatten）、inlier/outlier 点名
- **参照実装**: `experiments/m4_cage/m4_run.py` の `pnp_6pt`

### 4.3 反復リファイン1回分（FR-003）

- **データフロー**: (rvec_cur, tvec_cur, k) → レンダ（BGR・深度・α）→ レンダ PNG 保存 → loftr_cli subprocess → NPZ 読込 → conf 選別 → ゲート → 深度リフト → PnP → (rvec_new, tvec_new, diag)
- **処理ロジック**（順序固定。参照実装: `experiments/m4_cage/m4_2_run.py` の `run_iteration_matcher_m4`）:
  1. **Stage 2 レンダ**: `render_image(gaussians, cam_cur, near_plane=0.5, distort=True)` で BGR、方式A で深度・α。方式A は以下の擬似コードに**固定**する（`render_image(return_depth=True)` や `render_mode="RGB+ED"` は 3DGUT 経路非対応のため**使用禁止**。codex-66 指摘対応。参照実装: `stage2_smoke_test.render_depth_alpha_distorted`）:

```python
# 方式A: 各ガウシアン中心のカメラ座標奥行き z_i を「色」として RGB モードで流す
z_cam = (means @ viewmat[:3, :3].T + viewmat[:3, 3])[:, 2]          # (N,)
depth_colors = z_cam.unsqueeze(-1).expand(-1, 3).contiguous()       # 3ch 複製
rendered, alphas, _ = rasterization(
    means, quats, scales, opacities, colors=depth_colors,
    viewmats=viewmat[None], Ks=K[None], width=W, height=H,
    near_plane=0.5, far_plane=1e10, camera_model="pinhole",
    with_ut=True, with_eval3d=True, packed=False,
    radial_coeffs=..., tangential_coeffs=...,   # render_image(distort=True) と同一の係数詰め替え
    render_mode="RGB")                          # sh_degree は渡さない（直接色）
accum = rendered[0, ..., 0]                     # Σ(w·z)
alpha = alphas[0, ..., 0]                       # Σw
depth = where(alpha > 1e-6, accum / clamp(alpha, min=1e-6), NaN)    # α≈0 は NaN
```
  2. **Stage 3 マッチング**: レンダ BGR を PNG 保存し、`uv run --project matcher_lab python matcher_lab/loftr_cli.py <query.png> <render.png> --out <npz>` を subprocess 実行。NPZ から u_q（実写側）, u_r（レンダ側）, conf, meta を読む。meta の round-trip・範囲内率を §3.1 の基準で検証（違反は exit 1）。conf ≥ 0.2 で選別。選別後 0 件なら失敗段「マッチ0件」
  3. **Stage 4 ゲート**（AND。除外内訳は depth_ok→α→z→分散の順の逐次分類で記録）:
     - 変位: ‖u_q − u_r‖ < sched(k).tau_px
     - 深度: bilinear 補間値 d_bilin。2×2近傍の NaN/広がり5%超は除外（depth_ok）
     - α: レンダα（u_r の最近傍画素）> 0.5
     - z範囲: 0.5 < d_bilin < 10.0
     - 分散: 深度分散マップ（窓5、有効画素のみで計算）の値 < (0.02 × 有効画素深度中央値)²
     - 通過数 N < 30 なら失敗段「ゲート生存 N=<N> < 30」
  4. **Stage 5 深度リフト**: u_r を undistortPoints(P=K) でピンホール化 → K⁻¹ で光線 → d_bilin を掛けてカメラ座標 → 現在ポーズで世界座標 X_w
  5. **Stage 6 PnP**: §3.1 の「反復内 PnP」行のとおり。RANSAC 失敗・inlier<4・精密化失敗はそれぞれ失敗段として記録
  6. **診断値**: inlier 数、RMSE（inlier 上の再投影残差 RMS）、λ3/λ1（X_w[inlier] の共分散固有値比）、深度レンジ、凸包面積比、準誤マッチ数（N − inlier）
- **エラーハンドリング**: subprocess の exit code 42（CUDA OOM）は失敗段「マッチャーOOM」として当該カメラ失敗（解像度は 1280x720 固定であり fallback 先はない）。exit code その他非0 は stderr 末尾を添えて exit 1（環境異常）
- **境界条件**: マッチ0件でも配列 shape (0,2) で後続へ進まない（選別直後に失敗を返す）。u_r の最近傍画素 index は画像範囲に clip

### 4.4 サンプリング（FR-004）

- **処理ロジック**（参照実装: `m4_2_run.py` の `run_sampling_chain_matcher`）:
  1. チェーン ci（A/B/C）の初期ポーズを §3.1 のとおり生成
  2. `cv2.setRNGSeed(seed(ci, 0))` → k=0 で反復1回。失敗ならチェーン失敗（失敗段を記録）
  3. j=1..20: `cv2.setRNGSeed(seed(ci, j))` → k=1 条件で反復1回、成功した解を採取。失敗ならチェーン失敗
  4. 収束判定（更新量 ε）は行わない（サンプリング型が停止判定を置き換える。findings §1.9〜1.10）
- **境界条件**: チェーン失敗時も、そのチェーンで取得済みの診断値はレポートに残す

### 4.5 合意判定と受理（FR-005）

- **処理ロジック**（参照実装: `m3_1_sampling.py` の `cluster_analysis`。アルゴリズムを以下に固定）:
  1. 60サンプルの各解をカメラ中心 C = −Rᵀt と回転行列 R に変換
  2. 全ペアの中心距離と回転角を計算し、`within = (dpos ≤ 0.02) AND (dang ≤ 0.30°)` の隣接行列を作る
  3. 隣接数最大の点を anchor とし、anchor の within 集合を第1クラスタとする。f_c = |第1クラスタ|/60
  4. 残り集合で同じ操作を行い第2クラスタ占有率 f2 を得る
  5. 中心解: 位置は第1クラスタの C 平均。回転は anchor 相対の回転ベクトル平均（クラスタ半径 0.30° 以下で線形化が成立）を anchor に合成。tvec = −R_final @ C_mean
  6. 受理判定は §3.1 の「受理（単峰）」「受理（二峰仲裁）」のとおり。二峰仲裁は第1・第2クラスタそれぞれの中心解で手動6点の再投影誤差中央値を計算し、差 ≥ 1.0px なら誤差の小さい峰を採用
- **エラーハンドリング**: 3チェーンのいずれかが失敗（60サンプル未満）の場合、クラスタ分析は行わず不受理（失敗段はチェーンのもの）
- **境界条件**: f2 ≥ 0.2 だが合計占有率 < 0.9（多峰）、または仲裁差 < 1.0px は不受理とし分類名をレポートに出す

### 4.6 前提条件チェックと診断（FR-006）

- 初期解と最終解の pose 差（カメラ中心距離・回転角。`pose_diff` と同一定義）を計算し、> 0.50m または > 10° で「前提条件違反」（受理を取り消して失敗扱い）
- 手動6点再投影中央値（初期解・最終解）、警告閾値の点灯一覧をレポートに記録（合否に使わない）

### 4.7 結果出力（FR-007）

- **出力 TOML の書式**: phase0 `estimate_camera_params._format_toml_section` 互換のセクション形式。**入力 TOML の全カメラを元の順序で含める**（`--cameras` で対象を絞っても非対象カメラを脱落させない。codex-66 指摘対応）。カメラごとに name / size / matrix / distortions / rotation / translation を出力する。値の由来:
  - 受理カメラ: rotation/translation は最終解、その他キーは入力 TOML の値をそのまま転記
  - 不受理・失敗・`--cameras` 非対象カメラ: 全キーを入力 TOML の値のまま転記
  - 入力 TOML に上記以外のキー（fisheye 等）がある場合はそのキーも値を保存して転記する。転記方法が定義できない型（テーブル内テーブル）が現れた場合はエラー終了（安全側）
  - 入力に `[metadata]` セクションがある場合、そのキーと値を転記し、`refined_by = "feat-026"` と実行日（YYYY-MM-DD）を追記する
- **診断レポート**（テキスト。`--out-report` パス）: 実行条件（入力パス・定数の要約・seed_base）、カメラごとの節（6点PnP 結果、チェーンごとの反復ログ行、f_c/f2、受理形態、前提条件、警告一覧、手動点再投影）、全体サマリ（受理（単峰）/受理（仲裁）/不受理/失敗の内訳）
- **エラーハンドリング**: `--out-toml` / `--out-report` の既存ファイルは `--overwrite` なしなら exit 1（FR-007-3）

### 4.8 バッチ処理（FR-008）

- カメラを TOML 記載順に逐次処理（並列化しない）。カメラ単位の try で失敗を捕捉し、失敗段を記録して次のカメラへ進む
- PLY・LoFTR の読み込みコストのため、PLY はプロセスで1回だけロードする。LoFTR 重みのロードは loftr_cli 呼び出しごと（subprocess のため）に発生する——これは NFR-001 の処理時間見積りに織り込み済み（M4-2 実測）

### 4.9 マッチャー CLI（FR-009）

- **ファイル**: `matcher_lab/loftr_cli.py`（新規。参照実装: `experiments/m3b_matcher/matcher_cli.py` の LoFTR 部分のみ。MASt3R 部分は移植しない）
- **処理**: 画像2枚（原寸 1920x1080 チェック）→ グレースケール化 → 1280x720 に INTER_AREA 縮小 → `kornia.feature.LoFTR(pretrained="outdoor")` 推論（`torch.inference_mode`）→ 係数 1.5 で原寸へ逆写像 → 検証値（round-trip・範囲内率）を計算 → NPZ 保存
- **重みのオフライン読み**: 実行前にローカルキャッシュの存在と sha256 一致を確認し、なければ「セットアップ手順（README 記載のダウンロードコマンド）を実行せよ」というメッセージで exit 1。`torch.hub` の自動ダウンロードに fallback しない（NFR-005）
- **インターフェース**: §6.2

## 5. 状態遷移

該当なし（バッチ CLI。GUI・常駐状態なし）

## 6. ファイル・インターフェース設計

### 6.1 ファイル配置

| ファイル | 新規/変更 | 内容 |
|---|---|---|
| `phase4/refine_extrinsics.py` | 新規 | メイン CLI（§4.1〜4.8。単一ファイル） |
| `matcher_lab/loftr_cli.py` | 新規 | LoFTR 推論 CLI（§4.9） |
| `tests/test_feat026_refine.py` | 新規 | §8 の単体テスト |
| 既存ファイル | 変更なし | render.py / render_keypoints.py / matcher_lab/pyproject.toml（依存追加なし） |

### 6.2 refine_extrinsics.py CLI

```
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/refine_extrinsics.py \
  --toml <入力TOML> --ply <PLY> --images-dir <画像ディレクトリ> \
  --points-3d <3D基準点CSV> --points-2d <2D点CSV> \
  --out-toml <出力TOML> --out-report <レポートtxt> \
  [--cameras cam1 cam2 ...] [--overwrite] [--seed-base 5000] [--tmp-dir <中間PNG/NPZ置き場>]
```

- `--tmp-dir` 省略時はシステム一時ディレクトリ（`tempfile.mkdtemp`）を使い、正常終了時に削除する
- 終了コード: 0=全カメラ処理完了（不受理カメラがあっても 0。結果はレポートで判別）/ 1=全体エラー（入力不備・環境異常・出力衝突）

### 6.3 loftr_cli.py CLI と NPZ スキーマ

```
uv run --project matcher_lab python matcher_lab/loftr_cli.py <query.png> <render.png> --out <matches.npz>
```

- 終了コード: 0=成功 / 42=CUDA OOM / 1=その他（画像不正・重み欠落）
- NPZ: `u_q` float64 (N,2) 実写側原寸座標 / `u_r` float64 (N,2) レンダ側原寸座標 / `conf` float32 (N,) / `meta` JSON 文字列（`resolution`, `scale`, `roundtrip_max_px`, `in_range_rate`, `weight_sha256`）

## 7. ログ・デバッグ設計（FR-010）

- 反復ごとに1行: `k=<k>: OK|失敗(<失敗段>) | マッチ<生> 選別後<n> N=<N> inl=<inl> RMSE=<x>px | 警告: <一覧>` を標準出力と診断レポートの両方に出す
- 除外内訳（depth_ok/α/z/分散）は k=0 の反復についてレポートに記録する（檻シーン型の失敗切り分け材料。M4 §6.3 の運用実績）
- デバッグ用の中間ファイル（レンダ PNG・マッチ NPZ）は `--tmp-dir` 指定時のみ残す

## 8. テスト設計

実データ（PLY・GPU）に依存しない部分を `uv run pytest -v` の単体テストにする:

1. クラスタ分析（§4.5）: 合成サンプル（単峰60・二峰42+18・多峰）で f_c/f2/受理分類が期待どおり
2. 逆写像（§4.9 の式）: 格子点 round-trip ≤ 0.5px、既知座標の順逆一致
3. ゲート逐次分類（§4.3-3）: 人工の深度・αマップで除外内訳の合計が一致
4. TOML 出力（§4.7）: 入力 TOML → 出力 TOML で K・歪みが不変、rotation/translation のみ置換、`load_cameras_toml` で再読込可能
5. pose_diff・摂動生成: 既知入力での数値一致
- GPU を要する結合動作（レンダ・LoFTR）は手動テスト（M7）で確認する。テスト結果は `tests/results/feat-026_test_result.txt` に保存する

## 9. 設計判断の記録

| 判断 | 採用 | 理由・却下案 |
|---|---|---|
| 環境構成 | 2環境 subprocess 連携 | ユーザー確定仕様。kornia 0.8.3 が Python≥3.11 必須で phase4（3.10）と同居不可。phase4 の 3.12 化は gsplat JIT 再検証のリスクがあり却下 |
| マッチャー | LoFTR outdoor@0.2 固定 | ユーザー確定仕様。M3b/M4-2 で実証済みの唯一の構成。MASt3R は同一解精度が基準僅差で不足（M3b）のため Won't |
| 停止判定 | サンプリング型（ε 収束判定なし） | 更新量 ε=5mm/0.05° は実写ジッタ床（5〜13mm）より下で偽陰性を出す（M3 実測）。サンプリング型は M3-1/M3-2/M4-2 で Go |
| Stage 1.5 | 保留リスト行き | ユーザー確定仕様。採用条件（救済成功1件以上）が全実験で未観測 |
| LoFTR 解像度 | 1280x720 固定（full なし） | full はレンダプロセスと GPU 同居の本構成で OOM（M3b フェーズ0 実測）。fallback 分岐を持たないことで挙動を単純化 |
| 出力 | 新規 TOML（入力不変） | ユーザー確定仕様。20点方式の結果を破壊しない |
| メイン実装の言語・場所 | phase4 に単一スクリプト | 開発方針（シンプルに小さく）。レンダ・PnP・クラスタ分析は同一プロセスで完結し、外部依存はマッチャー subprocess のみ |

## 改訂履歴

- 2026-08-02: 初版 draft（requirements.md 初版と同時作成）
- 2026-08-02: codex-66 指摘対応（高2・中2）: ①requirements の目的文を運用受理限定に訂正、②入力点の一意性検証（一意 ObjectName 6個・有限値・範囲内）を requirements/design 両方に固定、③方式A の擬似コードを設計書に固定（RGB+ED 使用禁止を明記）、④出力 TOML は入力の全カメラを元順で含める規則に固定

# feat-024 スパイク計画書（ステップ1: gsplat UT経路の画質検証）

要求と設計を兼ねる軽量文書（1文書構成はユーザー承認済み。2026-07-22）。
スパイクの成果物は使い捨てであり、リポジトリにはコミットしない。

## 1. 目的

gsplat 1.5.3 の 3DGUT 経路（`with_ut=True`）が、**正しい条件**（`with_eval3d=True`、
`near_plane=0.5`）で使えば歪みレンダリングに実用できるか（黒い靄・品質劣化が出ないか）を
実機で判定し、feat-024 本実装の方式A（gsplat ネイティブ）/ 方式B（後処理ワープ）の
選択根拠を得る。

### 背景（なぜ再検証するか）

feat-013 で「UT経路 = 黒い靄」と結論されたが、その試行は
(1) 靄の真因（`near_plane=0.01` によるカメラ至近 floater の描画。ピンホール経路でも
同症状。feat-014/015 で判明）が未診断の時期に、
(2) `with_eval3d=True` なし・`packed=False` の条件で行われた。
交絡の疑いが濃厚であり、現在の知見（near_plane=0.5）+ 3DGUT の推奨条件
（`with_ut=True` と `with_eval3d=True` の併用）での再測定が必要。

## 2. 要求

### FR-S1: 3条件の比較レンダリング

同一の PLY・カメラ・near_plane で、静止画3枚を出力する。

| 条件 | 経路 | 歪み係数 | 役割 |
|---|---|---|---|
| (a) | 古典ピンホール（`with_ut=False, packed=True`。現行 `render_image` と同一） | なし | 基準画像 |
| (b) | UT（`with_ut=True, with_eval3d=True, packed=False`） | **ゼロ**（radial/tangential とも0） | UT経路そのものの劣化を分離検出する対照 |
| (c) | UT（`with_ut=True, with_eval3d=True, packed=False`） | TOML の実係数 | 歪みレンダリング本命 |

注: gsplat 1.5.3 は `with_ut` / `with_eval3d` 使用時に `packed=False` を要求する
（`rendering.py:353` の assert で確認済み）。UT条件では必ず `packed=False` を指定する。

- (b) が (a) とほぼ一致すれば「UT経路自体は無害」、(c) の変形が歪みとして自然なら方式A採用の根拠になる。
- (b) の段階で靄・劣化が出れば「UT経路自体が原因」と確定し、方式Bへ。

### FR-S2: 判定基準

1. **目視**: 黒い靄・ガウシアンの破綻・明らかな品質劣化の有無。(c) は樽型歪み
   （k1<0）に応じて画像周辺が内側へ湾曲した自然な変形であること
2. **数値の補助指標**（判定の主は目視。数値は記録用）:
   - 各画像の平均輝度（feat-014 の記録では靄あり時に暗転。正常時 輝度〜138）
   - (a) と (b) の画素差の平均絶対値（ほぼ一致の確認。目安: 平均輝度差が (a) の5%以内）

### FR-S3: 結果の記録

判定結果（3枚の画像パス・輝度統計・目視所見・結論）を feat-024 README の
ロードマップ表 1d 欄および「スパイク結果」節（新設）に記録する。

## 3. 設計

### 3.1 入力データ（固定値）

- PLY: `phase4/data/Blender/handai-hosp2_20260418.ply`
- TOML: `phase4/data/Blender/Config_scene.toml`
- カメラ: `cam41520554`（feat-015 のGT比較で使用実績。
  distortions = [-0.04, 0.1073, 0.0004, -0.0017, -0.0535] = [k1, k2, p1, p2, k3]）
- near_plane: 0.5（feat-015 で鮮明を確認済みの値）
- 背景色: 黒 (0, 0, 0)

### 3.2 スクリプト仕様

- 配置: セッションの scratchpad ディレクトリ（`spike_ut_check.py`）。**リポジトリに置かない・
  既存コードを変更しない**
- 実装方針: `sys.path` に `phase4/` を追加し、`render.py` の `load_ply` と
  `render_keypoints.py` の `load_cameras_toml` / `select_camera` / `camera_to_viewmat` を
  import して再利用。gsplat 呼び出しのみ自前で3条件分を書く
- 歪み係数の詰め替え（条件c）: TOML `[k1, k2, p1, p2, k3]` →
  `radial_coeffs = [[k1, k2, k3, 0, 0, 0]]`（gsplat は k1-k6 の6要素）、
  `tangential_coeffs = [[p1, p2]]`。いずれも torch.float32・CUDA・バッチ次元付き
- 条件bのゼロ係数: 同形状のゼロテンソルを明示的に渡す（None ではなく、係数経路を
  通したうえでの対照とする）
- 出力: scratchpad に `spike_a_pinhole.png` / `spike_b_ut_zerodist.png` /
  `spike_c_ut_dist.png` の3枚と、輝度統計・画素差のテキスト（標準出力）
- 実行コマンド（phase4 の venv・CUDA JIT ビルドのため環境変数必須）:
  ```bash
  TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python <scratchpad>/spike_ut_check.py
  ```

### 3.3 エラーハンドリング・境界

- gsplat が UT 系引数の組み合わせを拒否した場合（例外・引数エラー）: メッセージを
  そのまま記録して終了し、内容で分類する:
  - **既知の仕様制約への違反**（例: `packed=True` のまま UT を使った際の
    "Packed mode is not supported" 系 assert）→ スクリプト仕様違反として 1c に戻り修正
  - **上記以外の拒否**（本計画どおりの引数で発生するエラー）→
    「方式Aは現バージョンでは不可」の判定材料とする（回避策の実装はしない）
- JIT ビルド失敗（nvcc エラー等）: ログを記録して終了。環境課題として報告
- 実行時間: JIT ビルド込みで最大15分を想定。Bash タイムアウトは 600000ms に設定し、
  超過時はバックグラウンド実行に切り替える

### 3.4 テスト（スパイクにおける検証）

スパイク自体の検証は FR-S2 の判定そのものである（pytest 対象外。使い捨てスクリプトの
ため `tests/` にテストを追加しない）。スクリプトの正しさは条件(a)が現行
`render_keypoints.py` のピンホール出力と同一構図・同等輝度になることで確認する。

## 4. スコープ外

- `render_keypoints.py` 本体の変更（ステップ5で実施）
- 方式B（`cv2.remap` 後処理ワープ）のスパイク（方式A不採用と判定された場合に
  ステップ2で要否を判断）
- E0085 データでの検証（本実装後のステップ6で実施。スパイクは既存実績データで行う）

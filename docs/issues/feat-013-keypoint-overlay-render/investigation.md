# 調査・検証計画: feat-013 3DGSレンダリング不具合

本ファイルは feat-013 の手動テスト（実機確認）で見つかった不具合の調査・検証方法・修正計画を、イテレーション番号付きで記録する（上書きせず追記）。

---

## Iteration 1（2026-06-13）: 3DGSレンダリングNGの検証方法

### 発見した不具合

実機で `render_keypoints.py --camera cam41520554 --mp4` を実行し、出力MP4を目視確認した結果:

- **キーポイント: OK** — Halpe26の点・ボーンは妥当な位置・姿勢・動きで描画されている
- **3DGSレンダリング: NG** — 背景に「全く違うもの」が映っている（本来そのカメラから見えるべき部屋の景色になっていない）

### 数値検証で判明した事実（Subagentレビュー時に実施）

gsplatを使わない座標照合により、以下を確認した:

- PLY点群の bounding box は原点中心・extent 約10m（部屋スケール）、mean ≈ (0.24, -0.16, 0.34)
- 5台のカメラ中心 `-Rᵀt` はすべて原点から1〜2.5m（部屋内の妥当な配置）
- cam41520554 のカメラ空間で **PLY点の98%が前方（Z>0）**、median深度1.9m
- Halpe26キーポイント（calib座標）**26点すべてが PLY の bounding box 内**に収まり、全点が画面内、Nose が RAnkle より上

→ **PLY・キャリブ・キーポイントは同一の整合したワールド座標系にある**。座標系が回転/並進/スケールでズレていればこの整合は成立しない。

### 核心の論理

- キーポイントは `cv2.projectPoints(X_calib, rvec, tvec, K, D)` でOK → **R, t, K, D は正しい**
- gsplat の `viewmats` 規約は world-to-camera（OpenCV）で、`[[R,t],[0,1]]` と一致（gsplatソース確認済み）→ **viewmat 規約は正しい**
- ただし「viewmat が正しい」ことはレンダリングが正しい**十分条件ではない**。means座標も正しい（上記数値検証）
- よって残る容疑は **gsplat の非投影経路**（UT / packed / colors / quats / 活性化関数）

### 原因仮説（優先度順。Subagentレビュー反映）

1. **【最有力】UT + 歪み + `packed=False` 経路の破綻**
   - `render_keypoints.py` の `render_background` だけが `with_ut=True` + 歪み係数 + `packed=False`（`fully_fused_projection_with_ut`）を使う。動作実績のある `render.py` の `render_frame` は古典経路（`packed=True`、歪みなし）。**両者の唯一の実質的差分がここ**
   - 歪み係数のマッピング（`[k1,k2,k3,0,0,0]`(1,6) / `[p1,p2]`(1,2)）自体は gsplatソースと整合。したがって疑いは「歪みの値」ではなく「**UT経路に乗せたこと自体**」または「**この LCC製 PLY が UT経路の前提と合わない**」
2. **【次点】LCC製 PLY の活性化規約ミスマッチ**
   - `load_ply`（render.py）は scales=exp、opacity=sigmoid、quats=wxyz正規化 を仮定。LCC製 PLY（`point_cloud.ply`、`f_rest_*` なし sh_degree=0、法線 `nx,ny,nz` あり）が別規約なら、巨大/極小ガウシアンや全透明/全不透明で画面が崩れ「全く違うもの」になりうる。`print_ply_summary` の値域で即確認できる
   - 経路（UT/古典）に依らず効くため、もし値域が異常なら最有力に昇格
3. **【棄却寄り】PLY native ≠ キャリブ world 座標**
   - Iteration 1 当初は最有力としたが、上記数値検証で整合が確認され棄却寄り

### ground truth（採用: Blenderカメラビュー）

ユーザー決定により、正解は **Blender 上でのカメラビュー** とする。

生成方法（ユーザー環境で実施）:
- Pose2Sim_Blender（修正版フォーク `fix/coordinate-transform-rotation-bug` ブランチ）で `Config_scene.toml` のカメラをインポート
- KIRI アドオンで `point_cloud.ply` を 3DGS として表示（LCC製のため「Rotate for Blender Axes」は押さない）
- `cam41520554` をアクティブカメラにし、そのカメラビューをレンダリング（1920×1080）

注意点（Subagentレビュー反映）:
- Blenderのビューポート/EEVEE表示と gsplat はラスタライズ方式が異なる → **ピクセル一致でなく「構図（特徴点が画面内の正しい象限に来るか）」で判定**
- Blenderビューは構図の正解は与えるが、**TOML→viewmat 規約の正否の証明にはならない**（規約は数値で別途確認）

### 切り分け手順（改訂・実行は承認後）

下記は原因を「gsplat経路 vs PLY規約 vs 座標系」に一意に切り分けるための順序。手順0は gsplat不要・数秒。

- **手順0【gsplat不要・数秒】**
  - (0a) PLY bbox・カメラ中心・キーポイントの空間照合 → 仮説③（座標系）の棄却確認（数値検証で実施済み。再現スクリプトを残す）
  - (0b) `print_ply_summary` の値域チェック → scales after exp が常識的な大きさ（数mm〜数十cm）か、opacities after sigmoid が0〜1で妥当に分布するか。仮説②（規約異常）の早期検出
- **手順1【決定的】**: `render.py` の `render_frame` を cam41520554 の viewmat/K でそのまま1枚レンダリング（UT・歪み・packed をすべて render.py 既定=古典経路に戻した最小再現）
  - 背景が**正しく**映る → 原因は **UT + 歪み + packed=False 経路**に確定（仮説①）
  - 背景が**それでもNG** → 原因は **PLY活性化規約 or quats順序**（仮説②。手順0bと合わせ `load_ply` の規約照合へ）
- **手順2【交絡解消】**: 手順1がUT経路を指した場合、歪みとUTを分離して主因を特定
  - (2a) 歪みあり・UTあり（=現状）
  - (2b) 歪みなし・`with_ut=False`・`packed=True`（=render.py完全一致経路）
  - (2b)で直れば原因は「歪み or UT経路」。(2a)との差分で歪み単体かUT単体かを切り分け

> 旧・手順3（viewmat数値照合）は削除。`render.py` の値は別カメラ（Blender由来の FPSCamera）・別シーンのため、cam41520554 との数値直接比較は不可能（Subagentレビュー L-1）。規約検証は手順1（render.py経路の最小再現）に統合した。

### 原因別の修正方針（仮。原因確定後に design/requirements へ反映してからコード修正）

- **UT + 歪み経路が主因** の場合 → 背景は古典pinhole経路（歪みなし・packed=True）でレンダリングし、**歪みは後処理 `cv2` remap で付与**する方式に切り替え（キーポイントは元々 `cv2.projectPoints` で歪みあり投影なので、背景remapと整合させる）。または UT経路の使い方（パラメータ・前提）を見直す
- **PLY活性化規約ミスマッチ** の場合 → `load_ply` の exp/sigmoid/quat順序を LCC製 PLY の規約に合わせて修正
- **（棄却寄りだが万一）座標系** の場合 → gsplat に渡す `means` に座標変換を適用

### 次のアクション

- 本 Iteration 1（改訂版）をユーザーがレビュー（CLAUDE.md ステップ4の「人」レビュー。Subagentレビューは反映済み）
- 承認後、**手順0 → 手順1 → 手順2** を実行して原因を一意に特定し、結果と確定した修正計画を **Iteration 2** として追記
- 修正が design.md / requirements.md に影響する場合は先に更新してからコード修正（docs-before-code）

---

## Iteration 2（2026-06-13）: 手動テスト結果と修正方針（歪み廃止＋オクルージョン追加）

### 切り分け結果（Iteration 1 の手順を実行して確定）

- **viewmat / K / D は完全に正しい**: GT病室画像（Blender+KIRIで cam41520554 をレンダリングした `0001_color.png`）にキーポイントを重ねると、人体スケルトンがベッド上に正確に乗る。座標系・カメラパラメータは問題なし
- **処理は render.py と等価**: 歪みなしなら rasterization 引数は同一、`load_ply` を共有。`render.py` の `render_frame` をこのデータ・このviewmatで呼んでも同じ結果（座標変換の経路だけ入力規約に応じて違うが各々整合）
- **PLY規約は同じ**: `project.ply`（正常描画）と `point_cloud.ply` は scale=log・opacity=logit・quat正規化wxyz が一致（両方LCC Studio製）

### 手動テストで判明した3つの問題

1. **PLYファイルが誤り（確定・解決）**: `render_keypoints.py` に渡していた PLY が想定と別データだった。正しいPLYに差し替え済み。Iteration 1 で延々と切り分けていた「壁のもや」の主因はこれ
2. **3DGSレンダリングが黒い靄 ＝ 歪み（UT経路）が原因**: `with_ut=True` ＋ 歪み係数（unscented transform）は投影の近似で、密な点群で靄・品質劣化を起こす。標準3DGSはピンホール専用。→ **歪みを廃止してピンホール統一**（`render.py` と同じ古典経路）。キーポイント投影も歪みなしに統一
3. **オクルージョンが無い ＝ 要求漏れ（中核要件の欠落）**: 現実装は2D重ね描きで深度を考慮せず、カメラ手前の障害物にキーポイントが隠れず貫通する。これは本機能の存在意義に関わる中核要件だったが、**要求仕様作成時のヒアリングで確認漏れ**だった（「重ね描き」を2D描画と解釈した）。→ gsplat深度マップ（`render_mode="RGB+ED"`）で前後関係を判定し隠蔽する機能を追加

### 確定した修正方針（requirements.md / design.md に反映済み）

- **歪み**: あり（UT）→ **なし（ピンホール統一）**。`render_background` を古典経路（`with_ut=False`, `packed=True`, 歪み係数なし）に、`project_keypoints` を `distCoeffs=None` に。旧 `distortion_to_gsplat` は削除
- **オクルージョン**: 新規追加。`render_background` が `(bgr, depth_map, alpha_map)` を返す。`compute_keypoint_depth`（5.4）/ `compute_visibility`（5.12）を新設。`draw_overlay` は点の隠蔽＋ボーンの部分隠蔽（線分サンプリング）。CLI に `--no-occlusion` / `--occlusion-margin` を追加

### 次のアクション

- 更新した requirements.md / design.md を Subagent＋ユーザーでレビュー（高・中ゼロまで）
- 承認後、実装を修正（歪み廃止＋オクルージョン）、テスト更新、手動テスト（→ 結果を Iteration 3 として追記）

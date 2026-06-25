# feat-018 調査・修正計画

## イテレーション1 (2026-06-25)

### 1.1 不具合の特定

- **対応する要求ID**: FR-005（Blender io_anim_c3d での取り込み確認 / 人体が正立して見えること）
- **対応する設計セクション**: design.md「Blender 取り込み時の座標変換（検証根拠、FR-005）」、
  ADR-2、`write_c3d` の `set_screen_axis(X_SCREEN, Y_SCREEN)`、定数 `X_SCREEN/Y_SCREEN`
- **現在の動作**: 生成した `session001_f145749_world300.c3d` を io_anim_c3d で取り込むと、
  人体が**上下逆さま**に表示される。
  - 実測: 現行設定 `X_SCREEN='+Z', Y_SCREEN='-Y'` では、Blender の上方向(+Z)成分で
    Head − Ankle = **−0.257**（頭が下）。ユーザー報告（上下逆）と一致。
- **期待する動作**: requirements.md FR-005「人体の垂直方向が Blender の上方向(+Z)に一致して
  表示される（横倒し・上下逆さまでない）」。

### 1.2 原因分析

- **原因箇所**: `phase4/npz_to_c3d.py` の定数 `Y_SCREEN = "-Y"`（および design.md / requirements.md の
  軸設計の前提）。
- **原因の説明**: 軸設計が「world の鉛直（上）軸は **Y**」という前提（spec §4 本文「y=垂直」）に
  基づいていた。しかし spec §4 には「← キャリブ依存」の注記があり、**実データの鉛直軸は Z** だった。
  - 実測（先頭フレーム、Head − 両Ankle平均）: world 各軸差 = `[X:+0.106, Y:-0.257, Z:+0.784]`。
    絶対値最大は **Z軸（+0.784）** で、頭が +Z 側 = world の上方向は **+Z**。
  - 現行 `X_SCREEN='+Z', Y_SCREEN='-Y'` は、io_anim_c3d の `global_orient`（`O_sys=I`）により
    raw `(Yw,Zw,Xw)` を `p_blender = (Xw, -Zw, Yw)` に写す。Blender の上(+Z)成分には world の
    `Yw`（実際は非鉛直）が入り、真の鉛直 `Zw` は Blender の Y(横)へ回る。結果、上下が崩れる。
- **根本原因 or 表面的原因**: **根本原因**は「world の鉛直軸を Y と誤認したこと」。
  座標規約（ADR-1: raw=(Y,Z,X)mm、render_keypoints 互換）は正しく、変更不要。
  X_SCREEN/Y_SCREEN は Blender 表示専用で render_keypoints の挙動に影響しない（ADR-2 の独立性）ため、
  ここだけを正す。

### 1.3 修正内容

- **変更対象ファイル**:
  1. `phase4/npz_to_c3d.py`: 定数 `Y_SCREEN = "-Y"` → `Y_SCREEN = "+X"`（`X_SCREEN='+Z'` は維持）。
     - 結果: `p_blender = global_orient @ raw = (Xw, Yw, Zw)`（Blender = world そのまま、Blender Z = world Z = 上）。
       右手系（det=+1）。実測で Head − Ankle の Blender z = **+0.784（頭が上＝正立）** を確認済み。
  2. `docs/issues/feat-018-npz-to-c3d/requirements.md`:
     - FR-003 受け入れ基準: `Y_SCREEN='-Y'` → `Y_SCREEN='+X'`。
     - FR-005 受け入れ基準: 「人体の垂直方向（world Y）」→「人体の垂直方向（world の鉛直軸 = Z）」。
  3. `docs/issues/feat-018-npz-to-c3d/design.md`:
     - 定数記載・「Blender 取り込み時の座標変換」節・ADR-2 を `Y_SCREEN='+X'`／`p_blender=(Xw,Yw,Zw)`／
       world 鉛直=Z 前提に更新。テスト T-3 の期待値を `(Xw, Yw, Zw)` に更新。
  4. `tests/test_feat018_npz_to_c3d.py`: T-3 の期待値を `(Xw, Yw, Zw)` に更新
     （旧 `(Xw, -Zw, Yw)`）。
- **変更しないファイル**:
  - `world_to_c3d_raw`／座標規約（ADR-1）: render_keypoints 互換のため変更不要。X_SCREEN/Y_SCREEN は
    生レイアウトに影響しない。
  - `render_keypoints.py`: X_SCREEN/Y_SCREEN を無視して生 mm を直読みするため無影響。
- **修正が設計書に沿っているか**: design.md の注記「万一ずれた場合は X/Y_SCREEN の符号・軸の組合せ
  のみを調整する（生レイアウトと render_keypoints 互換は変更しない）」に沿う。前提（world 鉛直軸）の
  誤りを正すため設計書本文も併せて更新する（BUGFIX_STANDARD 2.2）。

### 1.4 影響範囲

- **他の機能への影響**: なし。X_SCREEN/Y_SCREEN は Blender io_anim_c3d 取り込み時の表示向きのみに作用。
  render_keypoints・座標ラウンドトリップ（c3d_to_calib）には無影響。
- **リグレッションリスク**: 低。座標値・ラベル・フレーム数・UNITS・residual・1始まりフレームは不変。
  変わるのは C3D の `POINT:Y_SCREEN` 文字列1個のみ。

### 1.5 確認方法

- **自動テスト（pytest）**:
  - T-3 を更新: `global_orient @ raw == (Xw, Yw, Zw)`、det=+1。
  - 追加: 実データ NPZ がある場合のみ、変換後 C3D を読み戻し、`global_orient` 適用後の
    Head の Blender z > 両Ankle の Blender z（正立）を確認（ファイル非存在時 skip）。
  - 既存 T-1/T-2/T-2b/T-4 が引き続き合格すること（座標・メタデータは不変）。
- **手動テスト**: 修正後 C3D を再生成し、io_anim_c3d で取り込んで人体が正立すること、
  全22ラベルがボーン化、スケールがメートル相当であることをユーザーが確認する。

## イテレーション2 (2026-06-25)

### 1.1 不具合の特定

- **対応する要求ID**: FR-005（人体が正立して見えること）
- **対応する設計セクション**: design.md「Blender 取り込み時の座標変換（検証根拠、FR-005）」、ADR-2
- **現在の動作**: イテレーション1で `Y_SCREEN='+X'` に修正したが、**まだ上下逆**（手動テスト2回目）。
  - 確認済み: Manual Orientation = OFF（デフォルト axis_interpretation 経路）、新規シーンで再取込、
    アドオンは `/home/sakagawa/git/io_anim_c3d/` のコードそのもの。→ イテレーション1の周辺仮説
    （手動向き／旧ファイル／別バージョン）はすべて否定された。
- **期待する動作**: 取り込み後に人体が正立（world 鉛直 +Z が Blender 上 +Z）。

### 1.2 原因分析

- **原因箇所**: イテレーション1の軸解析（design.md「Blender 取り込み時の座標変換」）が
  **不完全だった**。`p_blender = global_orient @ p_raw` までしか考慮せず、その後段を無視していた。
- **原因の説明（真の根本原因）**: io_anim_c3d は各マーカーを **armature の pose bone の
  `location` F-Curve** として書き込む（c3d_importer.py:118, 228 `pose.bones["%s"].location`）。
  pose bone の `location` は **ボーンのローカル座標**であり、ボーンは rest 状態で
  head=(0,0,0)→tail=(0,0,bone_size)（**+Z 向き**, c3d_importer.py:337-338）に作られる。
  +Z 向きボーンの rest 行列（local→world）は Blender の `vec_roll_to_mat3`（roll=0）で `Rx(+90)`、
  すなわち `(lx,ly,lz) → (lx, -lz, ly)` となる。
  したがって**画面に表示されるワールド座標は** `displayed = B @ (global_orient @ p_raw)`、
  `B = [[1,0,0],[0,0,-1],[0,1,0]]`。
  - これにより表示ワールドの鉛直成分は `displayed.z = (global_orient @ p_raw).y = axis_y · p_raw`
    となり、**`Y_SCREEN`（axis_y）だけで鉛直の上下が決まる**（`X_SCREEN` は鉛直に無関係）。
  - `p_raw = (Y_w, Z_w, X_w)` に対し:
    - `Y_SCREEN='-Y'` → axis_y·p_raw = −Z_w（頭が下＝逆）。実測1と一致。
    - `Y_SCREEN='+X'` → axis_y·p_raw = +Y_w（頭が下＝逆。Y_w は非鉛直）。実測2と一致。
    - `Y_SCREEN='+Y'` → axis_y·p_raw = +Z_w（頭が上＝**正立**）。
- **モデル検証**: 上記モデル（ボーン rest 行列 B を含む）を**実物の `axis_interpretation`** に通して
  数値計算したところ、表示ワールド Z（頭−足）は `-Y: −0.784m`, `+X: −0.257m`, `+Y: +0.784m`。
  **過去2回の実測（-Y, +X が逆）を両方とも再現**したため、モデルは妥当。
- **イテレーション1が誤った理由**: `axis_interpretation` を手計算で再現し、ボーン rest 行列の段を
  欠いた状態で「`+X` で正立」と誤判定した。今回は実物コード + ボーン行列で検証し、実測2点で裏付けた。

### 1.3 修正内容

- **変更対象ファイル**:
  1. `phase4/npz_to_c3d.py`: 定数 `Y_SCREEN = "+X"` → `Y_SCREEN = "+Y"`（`X_SCREEN='+Z'` 維持）。
  2. `requirements.md` FR-003: 受け入れ基準の `Y_SCREEN` を `'+Y'` に。
  3. `design.md`: 「Blender 取り込み時の座標変換」節を **ボーン rest 行列 B を含む**正しい導出に書き換え、
     ADR-2 と `set_screen_axis` を `'+Y'` に更新。T-3 の検証方法を「実物 axis_interpretation + B」に更新。
  4. `tests/test_feat018_npz_to_c3d.py`: T-3 を **ボーン rest 行列 B 込み**の判定に修正
     （手計算モデル単体では実機を再現できない）。実データの正立チェックも B 込みにする。
- **座標値変換は変更しない**: `world_to_c3d_raw`（world→raw mm）は正しい（読み戻し 6e-8m・
  render_keypoints 互換）。本不具合は **Blender 表示向きメタデータ（Y_SCREEN）のみ**の問題。
- **変更しないファイル**: `render_keypoints.py`（X/Y_SCREEN を無視して生 mm を直読み。無影響）。

### 1.4 影響範囲

- **他の機能への影響**: なし。Y_SCREEN は Blender 表示向きのみ。座標値・ラベル・フレーム数・
  residual・UNITS・1始まりフレームは不変。render_keypoints のラウンドトリップにも無影響。
- **リグレッションリスク**: 低。変わるのは C3D の `POINT:Y_SCREEN` 文字列1個のみ。

### 1.5 確認方法

- **自動テスト（pytest）**: T-3 を「実物 `axis_interpretation` から得た global_orient に
  ボーン rest 行列 B を適用 → Head の表示ワールド Z > 両Ankle」で判定（実データ NPZ がある時のみ、
  非存在時 skip）。手計算モデル単体では実機を再現できないので `B` を必ず含める。
- **手動テスト**: `Y_SCREEN='+Y'` で C3D 再生成 → io_anim_c3d（Manual Orientation OFF）で取り込み、
  人体が正立することをユーザーが確認する。

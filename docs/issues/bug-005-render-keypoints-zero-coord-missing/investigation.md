# bug-005 調査・修正計画: render_keypoints.py が座標 (0,0,0) の欠損マーカーを有効点として描画する

基準: `docs/BUGFIX_STANDARD.md`

## イテレーション1 (2026-10-18)

### 前提（修正のベースとなるコード）

本修正は、作業ツリー上の `phase4/render_keypoints.py`（feat-037 の実装を含む、未コミットの状態）に対して行う。feat-037 は手動テストの途中で、ユーザー判断により本案件の完了まで完了確認を保留している（feat-037 のステータスは Review のまま変えない）。修正対象の `extract_keypoints` は feat-037 では変更されておらず、feat-021 時点の実装のままである。

### 1.1 不具合の特定

- **対応する要求ID**: feat-021 FR-001（欠損マーカー許容）。feat-021 要求仕様書の用語「無効サンプル」の定義（「C3D 上でマーカーは存在するが、当該フレームで residual < 0 または座標に NaN を含むサンプル」）が、Pose2Sim 出力 C3D の欠損の表し方（座標 (0,0,0)・residual 0）を含んでいない
- **対応する設計セクション**: feat-021 `design.md` 1.4.2（`extract_keypoints`。`valid[i] = residual[idx] >= 0 and not isnan(data[idx]).any()`）
- **現在の動作**: Pose2Sim 出力 C3D の欠損マーカー（座標が厳密に (0,0,0)、residual 0）が valid=True になり、点がワールド原点の投影位置に描かれ、その点につながるボーンが原点の投影位置まで引かれる。再現手順は `README.md` を参照
- **期待する動作**: 欠損したマーカーは点・ボーンとも描かない（feat-021 FR-001「欠損マーカーは全フレームで点・ボーンとも描画されない」、および無効サンプルは描画スキップ）
- **エラーメッセージ**: なし（例外は発生せず、描画結果だけが崩れる）

#### 実データの調査結果（2026-10-18）

`load_c3d_all_frames` で全フレームを読み、集計した。

| ファイル | 座標が厳密に (0,0,0) の点 | (0,0,0) の点を含むフレーム | NaN の点 | residual < 0 の点 | (0,0,0) の点の residual |
|---|---|---|---|---|---|
| `m6_full_person1_0-54006.c3d` | 163,275 / 1,404,182 | 21,920 / 54,007 | 0 | 0 | すべて 0.0 |
| `m6_full_person2_0-54006.c3d` | 230,196 / 1,404,182 | 21,730 / 54,007 | 0 | 0 | すべて 0.0 |

P1 フレーム1では `RHeel`, `LAnkle`, `LBigToe`, `LSmallToe`, `LHeel` の5点が (0,0,0) で、カメラ `cam05520126` への投影位置はすべて (1225, 468)（ワールド原点の投影位置）だった。それ以外の点は画面左外（u < 0）にあり、位置は正しい（画面外の人物の描画自体は不具合ではない）。

### 1.2 原因分析

- **原因箇所**: `phase4/render_keypoints.py` の `extract_keypoints`

  ```python
  valid[i] = bool(residual[idx] >= 0) and not bool(np.isnan(coord).any())
  ```

- **原因の説明**: Pose2Sim の C3D 書き出し（`/home/sakagawa/git/pose2sim/Pose2Sim/common.py` の `create_c3d_file`）は、全マーカー・全フレームで residual を 0.0 に固定して書く（`residuals = np.full((len(marker_names), 1), 0.0)`）。欠損したマーカーの座標は 0 として書かれる（実データで NaN は0件、欠損位置は厳密に (0,0,0)）。そのため residual でも NaN でも欠損を判別できず、`extract_keypoints` は valid=True を返す
- **根本原因 or 表面的原因**: 根本原因。有効判定の定義（feat-021「無効サンプル」）が、入力に来る欠損の表し方の1つ（座標 (0,0,0)）を含んでいないこと。Pose2Sim 側の書き出し形式は本プロジェクトの管理外なので、読み込み側の有効判定で吸収する

### 1.3 修正内容

#### 変更対象ファイル

1. `phase4/render_keypoints.py` の `extract_keypoints`: 有効判定に「生座標が厳密に (0,0,0) でない」を追加する。docstring の有効の定義も同じ内容に更新する
2. `tests/test_feat016_keypoints.py` の `TestExtractKeypoints.test_missing_marker_invalid`: テストデータを `np.zeros((26, 3))` から `np.ones((26, 3))` に変える（後述「既存テストへの影響」）
3. `tests/test_bug005_zero_coord_missing.py`（新規）: 1.5 のテスト

#### 修正コード

修正前:

```python
    """ラベルから既知マーカー28点を抽出し、(28,3) 座標と (28,) 有効フラグを返す。

    有効 = マーカーがC3Dに存在し、residual >= 0 かつ 座標にNaNを含まない。
    C3Dに存在しない既知マーカーは valid=False・座標 0.0 とする（欠損許容。例外は投げない）。
    """
    ...
        valid[i] = bool(residual[idx] >= 0) and not bool(np.isnan(coord).any())
```

修正後:

```python
    """ラベルから既知マーカー28点を抽出し、(28,3) 座標と (28,) 有効フラグを返す。

    有効 = マーカーがC3Dに存在し、residual >= 0 かつ 座標にNaNを含まず、
    かつ 座標が厳密に (0,0,0) でない。Pose2Sim 出力C3Dは欠損を residual 0・座標 (0,0,0)
    で書くため、(0,0,0) も欠損として扱う（bug-005）。
    C3Dに存在しない既知マーカーは valid=False・座標 0.0 とする（欠損許容。例外は投げない）。
    """
    ...
        valid[i] = (bool(residual[idx] >= 0) and not bool(np.isnan(coord).any())
                    and not bool(np.all(coord == 0.0)))
```

- 判定は C3D の生座標（mm、`c3d_to_calib` で変換する前）に対して行う。`extract_keypoints` が受け取る `data` が生座標なので、関数内の判定位置は上記の1行のみ
- 「厳密に 0」は `coord == 0.0` の要素ごとの完全一致とし、許容誤差は設けない（欠損は書き出し時に 0 が代入されたもので、計算誤差を含まないため）。`-0.0 == 0.0` は True なので、符号付きゼロも欠損として扱われる
- 3成分すべてが 0 のときだけ欠損とする（1成分や2成分だけが 0 の点は有効のまま）

#### 変更しないファイル・関数

- `load_c3d_all_frames` / `load_npz_all_frames`: 読み込み結果（座標・residual）は変えない。判定は `extract_keypoints` に一元化する（feat-034 D-4 と同じ考え方）
- `collect_head_points`（FPS視錐台）、feat-037 の `compute_person_depth` / `compute_label_anchor` / フレームループ: すべて `extract_keypoints` の valid を使っているため、変更なしで (0,0,0) が欠損として扱われる
- `phase4/filter_c3d.py`: 平滑化の入力として Pose2Sim 出力 C3D を渡すと (0,0,0) が有効値として扱われる可能性がある。本案件のスコープ外（BUGFIX_STANDARD 2.3）。必要なら別案件として扱う
- `phase4/npz_to_c3d.py` / `phase4/render_fps_video.py`: NPZ の欠損は NaN で表されるため対象外

#### 修正が設計書に沿っているか

feat-021 の「無効サンプル」の定義を広げる修正であり、feat-021 の要求仕様書・機能設計書の記述と食い違う。そのため次の変更を feat-021 の文書に適用した（2026-10-18、本計画と同時に保存済み。3 は codex-01、4 は codex-02 の指摘で追加。Closed 案件の文書なので、bug-005 での変更であることを本文に明記している）。

1. `docs/issues/feat-021-render-keypoints-flexible-markers/requirements.md` の 1.2 用語定義「無効サンプル」の行:
   - 変更前: `C3D 上でマーカーは存在するが、当該フレームで residual < 0 または座標に NaN を含むサンプル`
   - 変更後: `C3D 上でマーカーは存在するが、当該フレームで residual < 0、座標に NaN を含む、または座標が厳密に (0,0,0) のいずれかに当たるサンプル（(0,0,0) は bug-005 で追加。Pose2Sim 出力C3Dの欠損表現）`
2. `docs/issues/feat-021-render-keypoints-flexible-markers/design.md` 1.4.2 の擬似コードの判定式の行:
   - 変更前: `    valid[i] = residual[idx] >= 0 and not isnan(data[idx]).any()`
   - 変更後: `    valid[i] = residual[idx] >= 0 and not isnan(data[idx]).any() and not all(data[idx] == 0)   # (0,0,0) 判定は bug-005 で追加`
3. `docs/issues/feat-021-render-keypoints-flexible-markers/design.md` 1.4.2 の戻り値 `valid` の説明:
   - 変更前: `  既存と同じ規則（\`residual >= 0\` かつ座標に NaN なし）`
   - 変更後: `  既存と同じ規則（\`residual >= 0\` かつ座標に NaN なし）に加え、座標が厳密に (0,0,0) でないこと` ＋ 次行 `  （(0,0,0) の条件は bug-005 で追加）`
4. `docs/issues/feat-021-render-keypoints-flexible-markers/requirements.md` と `design.md` の両方の冒頭（1行目のタイトルの直後）に、「bug-005 による変更（2026-10-18）」の注記ブロックを追加した。内容: (1) 無効サンプルの定義に (0,0,0) を追加したこと、(2) 本文中の「変更前と同一・同値・バイト一致」を求める要件と検証（要求仕様書 FR-001 受け入れ基準2 の (a)(b) と実データPNGのバイト一致、非機能要求の性能比較、機能設計書「2. テスト設計」の同値・一致の確認）は、座標が厳密に (0,0,0) のサンプルを含まない入力に限って適用し、(0,0,0) を含む入力の期待動作は本 investigation.md 1.5 で検証すること
   - 方式の経緯: codex-01 の指摘を受けて当初は受け入れ基準 2(b) の該当行だけに例外を書き足したが、codex-02 で同じ種類の指摘（描画内容・PNG一致の要件、feat-021 design.md「2. テスト設計」の実データPNGバイト一致の手動テスト）が別の箇所で再発した。1行ずつの追記では「変更前と同一」を求める記述をすべて拾いきれないと判断し（CLAUDE.md「行き詰まり検出」）、冒頭の注記ブロックで互換性要件全体に一括で例外を適用する方式に切り替えた。2(b) への行内追記は注記ブロックと重複するため取り消した
   - feat-021 のテスト（`tests/test_feat021_flexible_markers.py`）は、上記のリグレッション調査（判定を一時的に差し込んだ全件実行）で失敗しておらず、変更は不要

#### 過去の設計判断との関係（「全成分0」を判定に使わない、という feat-013 の判断）

`docs/issues/feat-013-keypoint-overlay-render/requirements.md` 60行目と同 `design.md` 146行目には、「『全成分0』は判定基準にしない（原点近傍の正当な点を誤って無効化するため）」という判断が記録されている（feat-013 は Cancelled。この判断は feat-016・feat-021 の residual ベースの判定に引き継がれた）。本修正はこの判断を次の理由で見直す。

- feat-013 の懸念は「原点**近傍**」の正当な点を消すこと。本修正は許容誤差を設けず、3成分が**厳密に** 0.0 の点だけを欠損とする。原点の近くにある点（例: (0.3, −1.2, 0.0) mm）は有効のまま残る
- 正当な点の3成分が浮動小数点で厳密に 0.0 になるのは、ワールド原点（=キャリブレーションの基準点）にマーカーがミリ未満まで一致して乗った場合だけで、実データ上は起こらない（1.1 の調査で、(0,0,0) の点はすべて欠損の位置に現れている）
- feat-013 の時点では、residual で欠損を表さない入力（Pose2Sim の C3D 書き出し）を想定していなかった。そのような入力では residual による判定だけでは欠損を区別できない

#### 変更しない既存文書

`extract_keypoints` の有効判定の現行の定義は feat-021 の文書（上記 1〜4 で更新）とする。次の文書は、その時点の判定を記録した履歴として扱い、本案件では変更しない（Closed / Cancelled 案件の記録を書き換えず、定義の更新は feat-021 と本 investigation.md に集約する）。

| 文書 | 該当箇所 | 変更しない理由 |
|---|---|---|
| feat-013 `requirements.md` 60行目 / `design.md` 100・134・146・276・293行目 | 「全成分0」を判定に使わない判断、residual ベースの判定 | Cancelled 案件の記録。見直しの理由は本節に記録した |
| feat-016 `requirements.md` 43・116行目 / `design.md` 109・203・228行目 | 「residual >= 0 かつ NaN なし」 | feat-021 で `extract_keypoints` に置き換わる前の定義の記録 |
| feat-017 `requirements.md` 29行目、feat-032 `requirements.md` 21行目 / `design.md` 135行目、feat-034 `requirements.md` 27・52行目 / `design.md` 221行目 | 欠損・無効の例として residual < 0・NaN を挙げている | 判定そのものは `extract_keypoints` に委ねており、(0,0,0) の追加で記述内容が誤りにはならない（例示が網羅でないだけ） |
| feat-018 `design.md` 175・261行目 | `npz_to_c3d.py` が無効点を residual < 0 で書くことの説明 | `npz_to_c3d.py` の書き出しは変えないため、記述は正しいまま |
| feat-020 `requirements.md` 23行目 | `filter_c3d.py` の「有効サンプル」の定義 | `filter_c3d.py` は本案件の対象外（1.3「変更しないファイル・関数」） |

### 1.4 影響範囲

- **他の機能への影響**:
  - `render_keypoints.py` の動画モード全般（1人モード・複数人モード、C3D・NPZ 入力とも）。(0,0,0) の点が描かれなくなり、その点につながるボーンも描かれなくなる
  - FPS視錐台（`--fps-frustum`）: 頭部7点のどれかが (0,0,0) のフレームは視錐台を描かなくなる（従来は原点を頭部点として計算していた）。終了時サマリの非描画数が増える
  - feat-037 の人物深度・ラベル基準点: (0,0,0) の点が平均・候補から外れる
- **リグレッションリスク**:
  - 実際の座標が厳密に (0,0,0) の正しい点は、欠損として描かれなくなる。ワールド原点に人体のマーカーがミリ単位の浮動小数点で完全一致する確率は実質ゼロのため、許容する
  - (0,0,0) を有効な点として使っている既存テスト。調査（2026-10-18）として、この判定を一時的に差し込んだ状態で `uv run pytest` を全件実行したところ、失敗は `tests/test_feat016_keypoints.py::TestExtractKeypoints::test_missing_marker_invalid` の1件のみだった（他は 554 passed / 1 skipped）。このテストは座標の値に意味がなく、「既知マーカーが欠けても例外にならない」ことを確かめるためにゼロ配列を使っているだけなので、テストデータを `np.ones((26, 3))` に変える。期待値（Hip だけ無効、残り25点は有効）は変えない
  - 既存の実データ（`npz_to_c3d.py` 製 C3D、NPZ）は (0,0,0) の点を含まないため、出力は変わらない（1.5 の手動確認で確かめる）

### 1.5 確認方法

#### 自動テスト（`tests/test_bug005_zero_coord_missing.py`）

| # | テスト | 期待値 |
|---|---|---|
| T-01 | `extract_keypoints`: ある点の座標を (0,0,0)、residual 0 にする | その点は valid=False、他の点は valid=True |
| T-02 | 同: 座標を (−0.0, 0.0, −0.0) にする | valid=False |
| T-03 | 同: 座標を (0, 0, 5.0) / (0, 3.0, 0) にする（1成分でも非ゼロ） | valid=True |
| T-04 | 同: residual < 0 の点・NaN の点 | 従来どおり valid=False（既存判定が残っていること） |
| T-05 | 同: 全点が (0,0,0) のフレーム | 全点 valid=False |
| T-06 | `collect_head_points`: 頭部7点の1点が (0,0,0) のフレーム | その点が NaN になり、`compute_fps_poses` で当該フレームが無効になる |

- 既存テスト全件: `uv run pytest -v` 全件パス（`test_missing_marker_invalid` のテストデータ変更後）。結果を `tests/results/bug-005_test_result.txt` に保存する

#### 動作確認（実装ステップ内でサブエージェントが実施）

プロジェクトルートで実行する。出力先はスクラッチディレクトリ（`/tmp/claude-1000/-home-sakagawa-git-lift2d-to-3d-keypoints/51e44b4e-f838-432c-beae-1a4116cbcde5/scratchpad/` 配下）にする。`TORCH_CUDA_ARCH_LIST="9.0+PTX"` は必須。

1. **不具合の解消**: `README.md` の再現手順（`--config phase4/data/run_keypoints_tokyo1.yaml --start-frame 1 --end-frame 1`、出力先を `--output-dir` でスクラッチディレクトリに変える）を実行し、P1 のボーンが (1225, 468) 付近に集まっていないことを目視で確認する。あわせて、修正後の `extract_keypoints` が P1 フレーム1の `RHeel`, `LAnkle`, `LBigToe`, `LSmallToe`, `LHeel` を valid=False にすることを確認する
2. **既存データで出力が変わらないこと**: feat-037 の動作確認と同じ条件（PLY `phase4/data/Blender/handai-hosp1_20251024.ply`、TOML `phase0/data/E0085/E0085_calib.toml`、カメラ `E0085-01`、`--near-plane 0.5`、C3D `phase4/data/session001_f145749_world300.c3d`、`--start-frame 100 --end-frame 102`）で、修正前（作業ツリーの修正前の `phase4/render_keypoints.py` をスクラッチディレクトリにコピーして実行）と修正後の PNG 3枚がバイト同一であること。修正前のコピーは、編集を始める前に取っておくこと

#### 手動テスト（ユーザー）

`phase4/data/run_keypoints_tokyo1.yaml` で区間を指定してレンダリングし、P1・P2 とも欠損点が原点に引っ張られていないことを確認する。完了後に feat-037 の手動テスト（完了確認）を再開する。

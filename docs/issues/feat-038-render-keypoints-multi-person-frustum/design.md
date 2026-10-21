# feat-038 機能設計書: render_keypoints.py 複数人全員のFPS視錐台の同時描画

作成日: 2026-10-20
基準: `docs/DESIGN_STANDARD.md`
対応要求: `docs/issues/feat-038-render-keypoints-multi-person-frustum/requirements.md`

## 1. 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 `--fps-person all` の受け付け | 4.1, 4.2 |
| FR-002 全人物の視錐台を描く | 4.2, 4.4, 4.5 |
| FR-003 全員指定時の頭部マーカー構成チェック | 4.3 |
| FR-004 1人モードでの全員指定 | 4.2, 4.6 |
| FR-005 設定YAMLでの全員指定 | 4.1 |
| FR-006 実行時の表示 | 4.4, 4.5 |
| FR-007 番号指定の後方互換 | 4.2〜4.6, 6, 7(D-2, D-3) |

## 2. システム構成

### 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `phase4/render_keypoints.py` | 変換関数 `_fps_person_value` の追加、`_build_parser` の `--fps-person` 定義・`CONFIG_CONVERTERS` の `fps_person`・`main` の変更、モジュール docstring の追記 |
| `tests/test_feat038_multi_frustum.py` | 新規テスト |

`render_fps_video.py` / `npz_to_c3d.py` / `render.py` は変更しない。`collect_head_points` / `compute_fps_poses` / `compute_frustum_vertices` / `draw_frustum` は変更しない。既存テストは変更しない。

### 依存関係

既存と同じ（`render_keypoints.py` → `render_fps_video.py`〔YAML 機構・`compute_fps_poses`〕）。新しい import は無い。

## 3. 技術スタック

既存と同一（Python 3.10 / NumPy / OpenCV / tomli / c3d / gsplat 経路。バージョンは `docs/TECH_STACK.md` のとおり）。新規ライブラリなし。`docs/TECH_STACK.md` の更新は不要。

## 4. 詳細設計

### 4.1 値の変換関数と CLI・設定YAML（FR-001, FR-005）

#### 定数（既存の `FRUSTUM_EDGES` の直後に追加）

```python
FPS_PERSON_ALL = "all"   # --fps-person の全員指定（feat-038）
```

#### `_fps_person_value(value: str) -> int | str`

`_yaml_paths` の直後に追加する。CLI の `type` と設定YAMLの変換関数の両方に使う。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def _fps_person_value(value: str) -> int | str:
    """--fps-person / 設定YAML fps_person の値（1以上の整数 または all）を変換する（feat-038）。

    "all"（小文字の完全一致）は FPS_PERSON_ALL を返す。それ以外は int() で変換する。
    値域（1〜人物数）の検査は main() で行う（人物数がここでは分からないため）。
    """
    if value == FPS_PERSON_ALL:
        return FPS_PERSON_ALL
    try:
        return int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"1 以上の整数または {FPS_PERSON_ALL} を指定してください（指定値: {value!r}）")
```

- `int()` の解釈は従来の `type=int` と同じ（`" 2"`・`"+2"`・`"-1"` は整数として通る。`0`・負数は main の値域検査で終了コード 2）
- `ALL`・`All`・`" all"` は `int()` に失敗して ArgumentTypeError になる

#### `_build_parser()` の変更

`--fps-person` の定義を次に置き換える（位置は変えない）:

```python
parser.add_argument("--fps-person", type=_fps_person_value, default=None,
                    help="視錐台を描く人物番号（1始まり、キーポイントファイルの指定順）。"
                         "all で全人物の視錐台を描く（--fps-frustum 専用、default: 1）")
```

- argparse は `type` が ArgumentTypeError を送出すると `argument --fps-person: <メッセージ>` を表示して終了コード 2 で終了する

#### `CONFIG_CONVERTERS` の変更

`"fps_person": int` を `"fps_person": _fps_person_value` に変える。

- 解釈できない値は既存の `parse_config_yaml`（`render_fps_video.py`）が `ArgumentTypeError` を捕捉し、`設定YAML キー 'fps_person': <メッセージ>` として終了コード 2 にする（追加実装なし）
- 設定YAMLから `"all"`（文字列）が `parser.set_defaults` に入った場合、CLI で `--fps-person` を指定しなければ argparse は文字列の既定値に `type` を再適用する。`_fps_person_value("all")` は `"all"` を返すので結果は変わらない。整数の既定値には再適用されない（従来どおり）

| 値（CLI / 設定YAML） | `args.fps_person` |
|---|---|
| （省略） | `None`（main で 1 に解決） |
| `2` | `2` |
| `all` | `"all"` |
| `ALL` / `All` / `x` / `1,2` | 終了コード 2 |

### 4.2 視錐台対象人物の決定（FR-001, FR-002, FR-004, FR-007）

`main()` の既存の `--fps-person` 値域検査を、全員指定を除外するように変える（4.2-1）。その後、既定値の解決の直後で視錐台対象人物の番号リスト `fps_targets` を作る（4.2-2）。

#### 4.2-1 値域検査（既存の検査の置き換え）

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if (args.fps_person is not None and args.fps_person != FPS_PERSON_ALL
        and not (1 <= args.fps_person <= n_persons)):
    parser.error(f"--fps-person は 1〜{n_persons} の整数を指定してください"
                 f"（指定値: {args.fps_person}）")
```

- メッセージ・位置は既存のまま。全員指定（`"all"`）は値域検査の対象外
- `--fps-frustum` なし・`--no-keypoints` との併用の検査（既存の `args.fps_person is not None` による判定）は変更しない。`"all"` も `None` ではないため、整数と同じく終了コード 2 になる

#### 4.2-2 `fps_targets` の作成（既存の `if args.fps_person is None: args.fps_person = 1` の直後に追加）

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if args.fps_person == FPS_PERSON_ALL:
    fps_targets = list(range(1, n_persons + 1))   # 人物番号の昇順
else:
    fps_targets = [args.fps_person]
```

- 1人モードで全員指定のとき `fps_targets == [1]` となり、以降の処理は `--fps-person 1` と同一になる（FR-004）
- `--fps-frustum` なしのときも `fps_targets` は作るが、以降の視錐台処理は `args.fps_frustum` が False のため実行されない
- 静止画モード（`--no-keypoints`）では `n_persons == 0` だが、`--fps-person` は併用できないため `fps_targets == [1]` になる。静止画モードは `fps_targets` を使わない

### 4.3 頭部マーカー構成チェック（FR-003, FR-007）

#### 1人モード

既存の頭部マーカー存在チェック（`HEAD_JOINT_NAMES` のうち `labels` に無いものを列挙し、`頭部マーカーが入力にありません（--fps-frustum に必須）: {名前列挙}` を stderr に出して return 1）を変更しない。1人モードの `fps_targets` は常に `[1]` のため、全員指定でも同じ検査になる。

#### 複数人モード

既存の「`--fps-frustum` 用の頭部マーカー構成チェック（対象人物のファイルのみ）」を、`fps_targets` の全人物を検査する処理に置き換える。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if args.fps_frustum:
    has_error = False
    for fp_k in fps_targets:                       # 人物番号の昇順
        fp = persons[fp_k - 1]
        missing_head = [name for name in HEAD_JOINT_NAMES if name not in fp["labels"]]
        if missing_head:
            print(
                f"頭部マーカーが入力にありません（--fps-frustum に必須）: "
                f"P{fp_k}: {fp['path']}: {', '.join(missing_head)}", file=sys.stderr,
            )
            has_error = True
    if has_error:
        return 1
```

- 番号指定では `fps_targets` が1要素なので、表示・戻り値は既存と同一（FR-007）
- 全員指定では、欠けている人物をすべて1行ずつ表示してから return 1 する
- 位置（人物ごとの読み込みループの後、point rate 一致検査の前）は既存のまま
- 検査の順序（FR-003）: 人物ごとの読み込みループ内の既存の検査（`load_c3d_all_frames` の失敗、フレーム番号重複、既知マーカー0個）は、最初に見つかった1件で即時 return 1 する既存動作のまま変えない。したがって、ある人物が既知マーカー0個のときは、他の人物の頭部マーカー不足は表示されない（例: P1 が既知マーカー0個、P2 が `Head` 不足 → 既知マーカー0個のエラーだけを表示して終了コード 1）。集約して表示するのは、読み込みループを全員分通過した後の頭部マーカー不足だけ

### 4.4 視錐台の事前計算（FR-002, FR-006）

既存の事前計算（`frustum_verts` / `frustum_valid` を1人分作る処理）を、`fps_targets` の人物ごとに作るリスト `frustums` に置き換える。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
frustums = []      # 要素: {"index": 人物番号, "verts": (F,5,3), "valid": (F,) bool}
if args.fps_frustum:
    print(f"FPS視錐台: カメラ {fps_cam['name']} ({fps_cam['width']}x{fps_cam['height']}), "
          f"奥行き {args.frustum_depth} m")                              # 既存のまま
    if len(fps_targets) >= 2:
        print(f"FPS視錐台: 対象人物 {', '.join(f'P{k}' for k in fps_targets)}")
    head_idx = {name: i for i, name in enumerate(HEAD_JOINT_NAMES)}
    for fp_k in fps_targets:
        fp = persons[fp_k - 1]
        fps_frames = [fr if fr is not None else make_absent_frame(f, len(fp["labels"]))
                      for f, fr in zip(frame_nos, fp["frames"])]
        head_arr = collect_head_points(fp["labels"], fps_frames)       # (F,7,3)
        c2w_b, valid, _ = compute_fps_poses(head_arr, head_idx)
        verts = compute_frustum_vertices(
            c2w_b, fps_cam["K"], fps_cam["width"], fps_cam["height"], args.frustum_depth)
        frustums.append({"index": fp_k, "verts": verts, "valid": valid})
```

- 1人分の計算（`make_absent_frame` による欠席フレームの補完、`collect_head_points` → `compute_fps_poses` → `compute_frustum_vertices`）は既存と同じ関数を同じ引数で呼ぶ
- `frustums` の並びは `fps_targets` と同じ（人物番号の昇順）
- 変数 `frustum_verts` / `frustum_valid` は `frustums` に置き換えて削除する（既存コードの参照箇所はフレームループと終了時サマリの2か所。4.5 で置き換える）

### 4.5 フレームループと終了時サマリ（FR-002, FR-006, FR-007）

#### フレームループ

既存の視錐台描画（`base = bgr_bg` の後、`frustum_verts is not None and frustum_valid[i]` のとき `base = draw_frustum(bgr_bg, frustum_verts[i], ...)`）を次に置き換える。スケルトン・人物ラベルの描画以降は変更しない。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
base = bgr_bg
for fr_info in frustums:                         # 人物番号の昇順
    if fr_info["valid"][i]:
        base = draw_frustum(
            base, fr_info["verts"][i], cam, depth_map, alpha_map,
            args.occlusion_margin, args.near_plane, occlusion=occlusion,
        )
```

- 1人目の `draw_frustum` は `bgr_bg` を入力に受け取り、2人目以降は前の人物の描画結果を受け取る。`draw_frustum` は入力をコピーして描くため `bgr_bg` は変更されない
- 視錐台対象人物が1人のときは、既存と同じ引数（入力 `bgr_bg`）で1回だけ呼ぶので出力は既存と同一（FR-007）
- `--fps-frustum` なしでは `frustums` が空のため、`base` は `bgr_bg` のまま（既存と同一）
- 視錐台をすべて描いた `base` の上にスケルトンと人物ラベルを描く処理は既存のまま（視錐台が一番下）

#### 終了時サマリ

既存の終了時サマリ（`if args.fps_frustum:` ブロック）を次に置き換える。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
if args.fps_frustum:
    n_total = len(frame_nos)
    for fr_info in frustums:
        prefix = "FPS視錐台" if len(frustums) == 1 else f"FPS視錐台 P{fr_info['index']}"
        n_valid = int(np.count_nonzero(fr_info["valid"]))
        if n_valid == n_total:
            print(f"{prefix}: {n_valid}/{n_total} フレームに描画")
        else:
            n_invalid = n_total - n_valid
            print(f"{prefix}: {n_valid}/{n_total} フレームに描画"
                  f"（頭部キーポイント欠損・縮退により {n_invalid} フレームは非描画）")
```

- 視錐台対象人物が1人のときの文言は既存と同一（FR-007）

### 4.6 1人モードの全員指定（FR-004）

4.2-2 で `fps_targets == [1]` となるため、4.3〜4.5 はすべて `--fps-person 1` と同じ処理を通る。`args.fps_person` の値（`"all"` か `1` か）を 4.2-2 以降で参照する箇所は作らない（`fps_targets` だけを参照する）。

### 4.7 モジュール docstring

既存の feat-037 の段落の「`--fps-frustum の視錐台は --fps-person で指定した1人（既定 P1）だけ描く。`」を次に置き換える:

`--fps-frustum の視錐台は --fps-person で指定した1人（既定 P1）を描く。--fps-person all で全人物の視錐台を同時に描く（feat-038）。`

### 4.8 エラーハンドリング一覧

| エラー | 検出 | 処理 | 終了コード |
|---|---|---|---|
| `--fps-person` が整数でも `all` でもない（CLI） | 4.1 `_fps_person_value` | argparse が `argument --fps-person: ...` を表示 | 2 |
| `fps_person` が整数でも `all` でもない（設定YAML） | 4.1 | `parse_config_yaml` の既存処理（`設定YAML キー 'fps_person': ...`） | 2 |
| `--fps-person` の整数が範囲外 | 4.2-1（既存） | `parser.error`（既存メッセージ） | 2 |
| `--fps-frustum` なしの `--fps-person`（`all` を含む） | 既存 | `parser.error`（既存メッセージ） | 2 |
| `--no-keypoints` と `--fps-person`（`all` を含む） | 既存 | `parser.error`（既存メッセージ） | 2 |
| 視錐台対象人物に頭部マーカーが無い（複数人モード） | 4.3 | 欠けている人物ごとに1行 stderr 表示 | 1 |
| 頭部マーカーが無い（1人モード） | 4.3（既存） | 既存メッセージ | 1 |

### 4.9 境界条件

- **全員指定で人物1人（1人モード）**: `--fps-person 1` と同一（4.6）
- **全員指定で全人物が同じフレームで視錐台を計算できない**: そのフレームは視錐台なし（スケルトンのみ）
- **全員指定で全人物の視錐台が全フレームで計算できない**: 処理は中断せず、各人物のサマリに `0/N フレームに描画` と表示する
- **同じパスを2回指定して全員指定**: 2人の視錐台が完全に重なって描かれる（同じ色のため1つに見える）
- **視錐台同士・視錐台とスケルトンの重なり**: 補正しない（視錐台はスケルトンより下、視錐台同士は人物番号の小さい順に下から描く）

## 5. ログ・デバッグ設計

- print ベース（既存と同じ。ログレベル機構は導入しない）
- 追加する標準出力は、視錐台対象人物が2人以上のときの「対象人物」1行（4.4）と、終了時サマリの人物ごとの行（4.5）のみ
- 番号指定時の標準出力は変えない

## 6. テスト設計

`tests/test_feat038_multi_frustum.py` を新規作成する。gsplat / torch を import しない範囲でテストする。main を通すテストは `tests/test_feat037_multi_person.py` と同じ方式（`render` モジュール・`render_image`・`load_c3d_all_frames` のスタブ化、合成TOML、`_make_person` 相当の合成人物データ）で行う。`tests/test_feat037_multi_person.py` のヘルパー関数は import せず、新規テストファイル内に必要なものを複製する（既存テストファイルを変更しないため）。

| # | テスト | 期待値 |
|---|---|---|
| T-01 | `_fps_person_value`: `"all"` / `"2"` / `"-1"` | `"all"` / `2` / `-1` |
| T-02 | `_fps_person_value`: `"ALL"` / `"All"` / `"x"` / `"1,2"` / `" all"` | `argparse.ArgumentTypeError` |
| T-03 | パーサー: `--fps-person all` | `args.fps_person == "all"` |
| T-04 | main: `--fps-person ALL`（`--fps-frustum` あり） | SystemExit(2)、stderr に `--fps-person` |
| T-05 | main: `--fps-person all` を `--fps-frustum` なしで指定 | SystemExit(2)、stderr に `--fps-frustum` |
| T-06 | main: `--no-keypoints --fps-person all` | SystemExit(2) |
| T-07 | main: 2人で `--fps-person all`。`collect_head_points` を monkeypatch で記録（元の関数に委譲） | 呼び出し2回、渡る labels が P1 → P2 の順 |
| T-08 | main: 2人で `--fps-person all`。`draw_frustum` を monkeypatch で記録（元の関数に委譲） | 両人物が全フレームに出席するとき、フレームあたり2回呼ばれる。各フレームの1回目の入力画像が背景画像（`render_image` スタブの戻り値）と同じ値、2回目の入力画像が1回目の戻り値と同一オブジェクト |
| T-09 | main: 2人で `--fps-person all`、P1=フレーム1〜4、P2=フレーム1〜2 | フレーム3・4 では `draw_frustum` が1回だけ呼ばれる。stdout に `FPS視錐台 P1: 4/4 フレームに描画` と `FPS視錐台 P2: 2/4 フレームに描画` |
| T-10 | main: 2人で `--fps-person all` の stdout | `FPS視錐台: 対象人物 P1, P2` を含む |
| T-11 | main: 2人で P2 にだけ `Head` が無い。`--fps-person all` | 戻り値 1、stderr に `P2` を含み `P1:` を含まない |
| T-12 | main: 2人とも `Head` が無い。`--fps-person all` | 戻り値 1、stderr に `P1` と `P2` の2行 |
| T-13 | main: T-11 と同じ入力で `--fps-person 1` | 戻り値 0 |
| T-13a | main: P1 が既知マーカー0個（ラベルが既知28マーカー以外のみ）、P2 が `Head` 不足。`--fps-person all` | 戻り値 1、stderr に `既知マーカーが1つもありません` と `P1` を含み、`頭部マーカーが入力にありません` を含まない |
| T-13b | main: 1人モードで `Head` の無いファイル1個。`--fps-person all` | 戻り値 1、stderr が `--fps-person 1` 指定時と一致（既存形式: `頭部マーカーが入力にありません（--fps-frustum に必須）: Head`。人物番号を含まない） |
| T-14 | main: 1人モードで `--fps-person all` と `--fps-person 1` | 出力PNGが `np.array_equal` で一致、stdout（出力ディレクトリ名を揃えて比較）が一致 |
| T-15 | main: 2人で `--fps-person 2`（番号指定） | `draw_frustum` がフレームあたり1回、stdout に `FPS視錐台 P` を含まず `FPS視錐台: ` のサマリ1行。`対象人物` を含まない |
| T-16 | main: 2人で `--fps-person all` の出力PNGと、P1・P2 の視錐台を `draw_frustum` で順に直接描いた上に P2 → P1 の順（人物深度で奥から）でスケルトンを `draw_overlay` で描き `draw_person_labels` でラベルを描いた画像 | `np.array_equal` |
| T-17 | 設定YAML: `fps_person: all` | `args.fps_person == "all"` |
| T-18 | 設定YAML: `fps_person: x` | SystemExit(2)、stderr に `fps_person` |
| T-19 | 設定YAML: `fps_person: all` を CLI の `--fps-person 2` で上書き | `args.fps_person == 2` |
| T-20 | 回帰: 既存テスト全件 | `uv run pytest -v` 全件パス（変更前 561 passed / 1 skipped + 本案件追加分）。既存テストの期待値修正は行わない |

T-16 の比較画像は、テスト内で main と同じ入力（スタブの背景画像・深度マップ・αマップ・合成人物データ・合成TOMLのカメラ）から、`compute_fps_poses` / `compute_frustum_vertices` / `extract_keypoints` / `c3d_to_calib` / `project_keypoints` / `compute_keypoint_depth` / `compute_visibility` / `compute_label_anchor` を直接呼んで作る。人物深度は P1=2m、P2=3m とし、描画順が P2 → P1 になるようにする。

既存テストが失敗した場合は、実装を中断して報告する（CLAUDE.md「実装の実行方法」の想定外事象の扱い）。

テスト結果は `tests/results/feat-038_test_result.txt` に `-v` 出力を保存する。

### 実データでの動作確認（実装ステップ内で実施）

プロジェクトルートで実行する。入力は `phase4/data/run_keypoints_tokyo1.yaml` の `ply_path` / `toml_path` / `keypoints_path`（tokyo 20261006 の person1・person2 の C3D 2ファイル。bug-005 の手動テストで使ったもの）/ `camera`（`cam05520126`）/ `near_plane`（0.5）と同じ値を CLI で渡す（設定YAMLは使わない）。同 YAML には視錐台関連のキーが無いため、`--fps-camera` には観察カメラと同じ `cam05520126` を指定し、`--fps-toml` は指定しない（FOV の確認が目的ではなく、描画の有無と同一性の確認が目的のため。モジュール docstring の feat-034 の例と同じ指定方法）。`--output-dir` はスクラッチディレクトリ配下にし、同 YAML の `output_dir` には出力しない。

1. **番号指定のバイト同一性**: 変更前の `render_keypoints.py`（`git show HEAD:phase4/render_keypoints.py` をスクラッチディレクトリに保存し、`PYTHONPATH=<絶対パス>/phase4` を通して実行）と変更後で、同じ2ファイル・`--fps-frustum --fps-person 1`・`--start-frame` / `--end-frame` で3フレームに絞って PNG を出力し、全 PNG がバイト同一であること。`--fps-frustum` なしでも同様に3フレームがバイト同一であること
2. **全員指定**: 同じ3フレームを `--fps-person all` で出力し、終了コード 0、終了時サマリが `FPS視錐台 P1:` / `FPS視錐台 P2:` の2行であること、出力 PNG に両人物の頭部から伸びるマゼンタの視錐台が描かれていることを目視確認する（両人物が出席し、頭部が欠損していないフレームを選ぶ。選んだフレーム番号を報告に記載する）
3. 長い区間での確認は手動テスト（ステップ7）でユーザーが行う

## 7. 設計判断（ADR 簡易版）

- **D-1 全員指定は `--fps-person all`**: ユーザー選択（2026-10-20）。「複数人なら常に全員」案は従来の番号指定の出力を変えるため、「番号の複数指定（`1,2`）」案は値の書式と検査が増えるため却下
- **D-2 内部では `fps_targets`（人物番号のリスト）に正規化する**: 全員指定と番号指定で処理を分岐させず、1要素のときに既存と同一の呼び出しになるようにする。`args.fps_person` を 4.2-2 以降で参照しないことで、1人モードの全員指定が `--fps-person 1` と同一になることを保証する
- **D-3 視錐台は人物ごとに `draw_frustum` を連鎖して描く**: 全人物の頂点をまとめて1回で描く新関数を作る案は、`draw_frustum` の変更または重複実装が必要になり、1人のときの出力同一性（FR-007）の保証が難しくなるため却下
- **D-4 色は全員マゼンタ・視錐台へのラベルなし**: ユーザー選択（2026-10-20）。どの人物の視錐台かは頭の位置と既存の人物ラベルで判断する
- **D-5 FOV・奥行きは全員共通**: ユーザー選択（2026-10-20）
- **D-6 頭部マーカーが無い人物がいたらエラー終了**: ユーザー選択（2026-10-20）。欠けている人物をすべて表示してから終了する（1人ずつ直して再実行する手間を避けるため）
- **D-7 `all` は小文字の完全一致のみ**: 大文字小文字を無視する案は、受け付ける表記を増やす根拠が無いため却下（エラーメッセージに `all` を示すので誤記は直せる）
- **D-8 終了時サマリは対象人物が2人以上のときだけ人物番号を付ける**: 常に番号を付ける案は、番号指定の標準出力を変えて後方互換（FR-007）を壊すため却下

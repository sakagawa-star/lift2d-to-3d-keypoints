# feat-032 機能設計書: render_keypoints.py の NPZ 入力対応

## 1. 対応要求マッピング

要求仕様書: `docs/issues/feat-032-render-keypoints-npz-input/requirements.md`

| 要求ID | 設計セクション |
|---|---|
| FR-001（拡張子判別） | 4.1（ディスパッチ） |
| FR-002（NPZ読み込み・検証） | 4.2（`load_npz_all_frames`）、4.4（エラーハンドリング） |
| FR-003（座標系の整合） | 4.2（座標変換） |
| FR-004（residual 合成） | 4.2（residual 合成） |
| FR-005（フレーム番号） | 4.2（frames_data 構築）、4.3（変更しない既存処理） |
| FR-006（MP4 fps） | 4.5（fps 決定・警告文言） |
| FR-007（CLI表記更新） | 5（CLI・メッセージ変更一覧） |

## 2. システム構成

### モジュール構成

```
phase4/
├── render_keypoints.py   # 変更対象。load_npz_all_frames を新設、main のディスパッチ変更
├── npz_to_c3d.py         # 変更なし。load_npz / world_to_c3d_raw を import 元として再利用
└── render.py             # 変更なし
tests/
├── test_feat032_npz_input.py       # 新規テスト
└── test_feat024_distort_render.py  # 1箇所変更（assert 文言の追随、5.3参照）
```

### 依存関係

- `render_keypoints.load_npz_all_frames` → `npz_to_c3d.load_npz`, `npz_to_c3d.world_to_c3d_raw`（**関数内 import**。npz_to_c3d のトップレベル import は argparse/os/sys/numpy のみで軽量だが、既存流儀〔c3d・torch は関数内 import〕に合わせる）
- 逆方向（npz_to_c3d → render_keypoints）の import は存在しない（循環依存なし）

## 3. 技術スタック

- Python 3.10（phase4 venv、uv 管理）。テストは root venv の `uv run pytest` で実行（既存方式: `sys.path.insert` で `phase4/` を追加）
- numpy（既存）。新規ライブラリ追加なし → `docs/TECH_STACK.md` の更新は不要

## 4. 詳細設計

### 4.1 入力形式ディスパッチ（FR-001）

`main()` の C3D ロード部（現行 680〜686 行付近）を置き換える。

```python
# 意図伝達用の擬似コード（そのままコピーしない）
is_npz = args.keypoints_path.lower().endswith(".npz")
fmt = "NPZ" if is_npz else "C3D"
print(f"キーポイント読み込み中（{fmt}, 全フレーム）: {args.keypoints_path}")
try:
    if is_npz:
        labels, frames_data, point_rate = load_npz_all_frames(args.keypoints_path)
    else:
        labels, frames_data, point_rate = load_c3d_all_frames(args.keypoints_path)
except (ValueError, FileNotFoundError) as e:
    print(e, file=sys.stderr)
    return 1
```

- 判別は「パス文字列を `str.lower()` した結果が `".npz"` で終わるか」のみ。ファイル内容の magic 判定はしない
- **`FileNotFoundError` を捕捉対象に追加する**（現行は ValueError のみ）。C3D 経路もファイル不存在時にトレースバックではなくメッセージ + 終了コード 1 になる（意図的な同時改善。4.4 参照）

### 4.2 `load_npz_all_frames`（FR-002/003/004/005）

`load_c3d_all_frames` の直後に新設する。

#### インターフェース

```python
def load_npz_all_frames(npz_path: str) -> tuple[list[str], list[dict], float]:
```

戻り値は `load_c3d_all_frames` と同一形式:

| 要素 | 型・内容 |
|---|---|
| labels | `list[str]`。NPZ の `joint_names` |
| frames_data | フレームごとの dict のリスト。`{"frame_no": int(frame_ids[i]), "data": (J,3) float64 mm, "residual": (J,) float64}` |
| point_rate | `0.0` 固定（NPZ にレート情報が無いことを表す。既存の「rate 取得不能」と同じ値） |

#### 処理ロジック

```python
# 意図伝達用の擬似コード（そのままコピーしない）
def load_npz_all_frames(npz_path):
    from npz_to_c3d import load_npz, world_to_c3d_raw  # 関数内 import（2章参照）

    try:
        x3d_world, frame_ids, joint_names = load_npz(npz_path)  # 正規化・検証込み
    except (ValueError, FileNotFoundError):
        raise                              # そのまま main で捕捉（メッセージ流用）
    except Exception as e:                 # zip破損・EOFError・pickle系等を正規化
        raise ValueError(f"NPZファイルを読み込めません: {npz_path}: {e}") from e
    if x3d_world.shape[0] == 0:
        raise ValueError(f"NPZにフレームがありません: {npz_path}")

    raw = world_to_c3d_raw(x3d_world)              # (F, J, 3) float64 mm
    finite = np.isfinite(raw).all(axis=-1)         # (F, J) bool
    residuals = np.where(finite, 0.0, -1.0)        # (F, J) float64

    frames_data = [
        {"frame_no": int(fid), "data": raw[i], "residual": residuals[i]}
        for i, fid in enumerate(frame_ids)
    ]
    return joint_names, frames_data, 0.0
```

#### 入力の正規化と検証（FR-002、feat-018 準拠）

正規化・検証はすべて `npz_to_c3d.load_npz` の既存実装をそのまま使う（**強化・変更しない**。7章 D-7 参照）:

- 正規化: `x3d_world` float64 化 / `frame_ids` reshape(-1) + int64 化 / `joint_names` reshape(-1) + str 化。`(F,1)` 形状や float dtype の `frame_ids` はエラーにせず正規化して受け付ける
- 検証（ValueError、日本語メッセージは feat-018 実装済み）: 必須キー存在、`x3d_world` 3次元かつ第3軸=3、`frame_ids`/`joint_names` 長さ一致、`frame_ids` 昇順連番（F≥2 時）
- F=0 のみ `load_npz` は検出しない（長さ一致・連番チェックが自明に通る）ため、`load_npz_all_frames` で明示チェックする
- `np.load(allow_pickle=True)` も feat-018 の既存仕様を踏襲する（入力 NPZ は自プロジェクトのリフトアップパイプラインが生成するローカルファイルに限られ、信頼境界を越えないため脅威にならない。7章 D-7 参照）
- `load_npz` が送出する `ValueError` / `FileNotFoundError` 以外のすべての例外（zip 破損の `zipfile.BadZipFile`、切り詰めファイルの `EOFError`、pickle 系例外を含む）は、上記擬似コードのとおり `ValueError("NPZファイルを読み込めません: <path>: <原因>")` に正規化して main の捕捉に乗せる（トレースバックで異常終了させない）

#### データフロー（座標変換の等価性、FR-003）

```
NPZ x3d_world (F,J,3) float32/float64, world座標[m]
  → npz_to_c3d.load_npz で float64 化・検証
  → world_to_c3d_raw: (X,Y,Z)m → (Y,Z,X)×1000 mm     … C3D raw 形式に変換
  → （既存パイプライン）extract_keypoints → c3d_to_calib: (px,py,pz)mm → (pz,px,py)×0.001 m
  → 結果はもとの world座標[m] と一致（feat-018 で確立した恒等性。float64 演算で誤差は 1e-9 m 未満）
```

world→raw→calib の往復変換を挟む理由は 7 章（設計判断）を参照。

#### residual 合成（FR-004）

- 有効: 関節の座標3成分すべてが有限（`np.isfinite` が True）→ `0.0`
- 無効: 1成分でも NaN/Inf → `-1.0`（既存の `extract_keypoints` が `residual >= 0` かつ NaN なしを有効とするため、`-1.0` で確実に描画スキップになる）
- `pnp_ok` はキーとして存在しても**参照しない**（全フレーム描画。要求仕様の用語定義およびユーザー決定 2026-08-13）

#### 境界条件

| 条件 | 振る舞い |
|---|---|
| F=0（`x3d_world.shape[0] == 0`） | `ValueError("NPZにフレームがありません: <path>")` → main で捕捉、終了コード 1。`npz_to_c3d.load_npz` は F=0 を検出しない（長さ一致・連番チェックが自明に通る）ため本関数で明示チェックする |
| F=1 | 正常動作（連番チェックは F≥2 のみ。既存 load_npz の仕様） |
| J=0（`joint_names` が空） | `load_npz` は通す。main の既存チェック「既知マーカーが1つもありません」で終了コード 1（4.3 参照） |
| 全関節 NaN のフレーム | residual 全て -1.0 → そのフレームは背景のみの出力（既存の欠損許容動作） |
| frame_ids が 6 桁超（例: 1145599） | PNG 名は `frame_{fid:06d}.png` の書式のまま桁あふれせず全桁出力（Python の `{:06d}` は最小幅指定）。既存 C3D 経路と同一書式で変更なし |
| `.NPZ`（大文字拡張子） | `lower()` により NPZ として読む |

### 4.3 変更しない既存処理（FR-005 ほか）

以下は frames_data 構築後の共通処理であり、**一切変更しない**:

- 既知マーカー存在チェック（`present` 構築、0個で終了コード 1）。NPZ テストデータは 28 点中 22 点（足先 6 点欠損）→ feat-021 の欠損許容がそのまま働く
- フレーム範囲フィルタ（`--start-frame`/`--end-frame`、`frame_no` 基準・両端含む）。NPZ では `frame_no` = 絶対 frame_ids なので、絶対フレーム番号で指定する（FR-005 の受け入れ基準）
- 投影（`c3d_to_calib` → `project_keypoints`）、オクルージョン判定、描画、連番 PNG 名 `frame_{frame_no:06d}.png`、MP4 書き出し

### 4.4 エラーハンドリング

| エラー | 検出方法 | 処理 | 出力（stderr） |
|---|---|---|---|
| NPZ ファイル不存在 | `np.load` の `FileNotFoundError` | main で捕捉 → 終了コード 1 | 例外メッセージそのまま |
| C3D ファイル不存在 | `open` の `FileNotFoundError` | 同上（今回捕捉を追加） | 例外メッセージそのまま |
| 必須キー欠落 / 形状不正 / 長さ不一致 / frame_ids 非連番 | `npz_to_c3d.load_npz` の `ValueError`（日本語メッセージは feat-018 実装済み） | main で捕捉 → 終了コード 1 | load_npz のメッセージそのまま |
| F=0 | `load_npz_all_frames` 内の明示チェック | 同上 | `NPZにフレームがありません: <path>` |
| NPZ が壊れている（zip 破損・非 NPZ バイナリ・切り詰め・pickle 系） | `load_npz_all_frames` が `ValueError` / `FileNotFoundError` 以外のすべての読み込み例外を `ValueError` に正規化（4.2 参照） | main で捕捉 → 終了コード 1 | `NPZファイルを読み込めません: <path>: <原因>` |

リトライは行わない（ローカルファイル処理のため）。ログは print（stdout=進捗、stderr=エラー・警告）の既存方式を踏襲し、logging モジュールは導入しない。

### 4.5 MP4 fps の決定と警告文言（FR-006）

既存ロジック（`--mp4-fps` 指定 > `point_rate > 0` > フォールバック 30.0）は変更しない。`load_npz_all_frames` が `point_rate=0.0` を返すことで自動的にフォールバックに入る。

変更は警告文言のみ（現行 750 行付近）:

- 変更前: `"警告: C3D rate を取得できないため MP4 fps=30 を使用"`
- 変更後: `"警告: 入力にフレームレート情報がないため MP4 fps=30 を使用"`

（この文言を assert している既存テストは無いことを確認済み）

## 5. CLI・メッセージ変更一覧（FR-007）

### 5.1 位置引数の改名

`_build_parser` の位置引数 `c3d_path` → `keypoints_path`（`args.c3d_path` 参照は全て `args.keypoints_path` に置換。現行 main 内 6 箇所 + `_build_parser` 内 1 箇所）。

help 文言:

- 変更前: `"人体キーポイントC3Dパス（Halpe26 + Spine/Thorax の既知28マーカー。欠損許容、全フレームを使用。--no-keypoints 時は省略）"`
- 変更後: `"人体キーポイントファイルパス（C3D または NPZ。拡張子 .npz で NPZ と判別。Halpe26 + Spine/Thorax の既知28マーカー、欠損許容、全フレームを使用。--no-keypoints 時は省略）"`

### 5.2 parser.error / print 文言の変更

| 箇所（現行行付近） | 変更前 | 変更後 |
|---|---|---|
| 644-645 | `--no-keypoints 指定時は c3d_path を渡せません（静止画モードは C3D 不要）` | `--no-keypoints 指定時は keypoints_path を渡せません（静止画モードはキーポイント入力不要）` |
| 656-657 | `c3d_path を省略する場合は --no-keypoints を指定してください` | `keypoints_path を省略する場合は --no-keypoints を指定してください` |
| 681 | `C3D読み込み中（全フレーム）: {path}` | `キーポイント読み込み中（{fmt}, 全フレーム）: {path}`（fmt は `"NPZ"` または `"C3D"`） |
| 690 | `C3Dに既知マーカーが1つもありません（labels: ...）` | `{fmt}に既知マーカーが1つもありません（labels: ...）` |
| 699 | `C3D: {n} フレーム, rate={r} Hz` | `{fmt}: {n} フレーム, rate={r} Hz` |
| 750 | 4.5 参照 | 4.5 参照 |
| 565-567 help | 5.1 参照 | 5.1 参照 |
| 579-582 help（--start-frame/--end-frame） | `描画開始C3Dフレーム番号…` / `描画終了C3Dフレーム番号…` | `描画開始フレーム番号（C3D frame_no / NPZ frame_ids。この番号を含む、省略時は最小フレーム）` / `描画終了フレーム番号（同、省略時は最大フレーム）` |
| 586 help（--mp4-fps） | `MP4フレームレート（小数可、default: C3D rate）` | `MP4フレームレート（小数可、default: C3D rate。NPZ はレート情報が無いため 30）` |
| モジュール docstring（1-24行） | C3D のみの記述 | 入力に「C3D または NPZ（`x3d_world` world座標[m]、拡張子で判別）」を追記。実行例は現行のまま維持し、NPZ を渡す例を 1 行追加 |

### 5.3 既存テストの追随変更

`tests/test_feat024_distort_render.py` の `test_c3d_given_with_no_keypoints_exits_2`:

- 変更前: `assert "c3d_path" in err`
- 変更後: `assert "keypoints_path" in err`

これ以外の既存テストは変更しない（`uv run pytest -v` 全件パスが受け入れ条件）。

## 6. テスト設計

新規 `tests/test_feat032_npz_input.py`。既存 `test_feat017_all_frames.py` のパターンを踏襲する（root venv で実行、`sys.path.insert` で `phase4/` を追加、`render` モジュールを `types.ModuleType` でスタブ化、`render_keypoints.render_image` を monkeypatch）。合成 NPZ は `np.savez` で tmp_path に生成する。

| # | テスト | 対応FR | 概要 |
|---|---|---|---|
| 1 | `test_load_npz_all_frames_basic` | FR-002/005 | 合成 NPZ（F=3, J=22, frame_ids=145599..145601）→ labels/frame_no/point_rate=0.0 を検証。`frames_data[i]["frame_no"] == frame_ids[i]` |
| 2 | `test_world_roundtrip_equivalence` | FR-003 | `c3d_to_calib(frames_data[i]["data"])` が元の world 座標と `atol=1e-9` で一致 |
| 3 | `test_nan_joint_residual` | FR-004 | 1関節に NaN を仕込む → その residual が -1.0、他は 0.0。`extract_keypoints` を通すと valid=False |
| 4 | `test_pnp_ok_ignored` | FR-004 | `pnp_ok` 全 False の NPZ でも全フレームの residual が 0.0（座標有限なら有効） |
| 5 | `test_empty_frames_error` | FR-002 | F=0 の NPZ → `load_npz_all_frames` が `ValueError`（メッセージに「フレームがありません」を含む） |
| 6 | `test_invalid_npz_exit_code`（parametrize） | FR-002 | 異常 NPZ を main に渡す（合成 TOML 使用）→ 戻り値 1、stderr にメッセージ。ケース: (a) 必須キー欠落 (b) `x3d_world` 形状不正（2次元） (c) `frame_ids` 長さ不一致 (d) `joint_names` 長さ不一致 (e) `frame_ids` 非連番 (f) F=0 (g) 壊れた NPZ（`.npz` 拡張子の非 zip ゴミバイトファイル） |
| 7 | `test_dispatch_by_extension` | FR-001 | `.npz`（大文字 `.NPZ` 含む）→ NPZ ローダー、`.c3d` → C3D ローダーが呼ばれることを monkeypatch で記録して検証 |
| 8 | `test_main_npz_end_to_end` | FR-001/005 | render スタブ + 実 `load_npz_all_frames` で main を完走させ、PNG 名が `frame_145599.png` 形式（絶対フレーム番号）になること、`--start-frame/--end-frame`（絶対番号指定）でフレーム数が絞られることを検証 |
| 9 | `test_mp4_fps_fallback_and_override` | FR-006 | ffmpeg を monkeypatch し、`--mp4-fps 25` → fps=25、未指定 → fps=30 + stderr に「フレームレート情報がない」警告 |
| 10 | `test_missing_file_returns_1` | FR-002 | 存在しない `.npz` / `.c3d` パス → main 戻り値 1（トレースバックにならない） |
| 11 | `test_help_shows_keypoints_path` | FR-007 | `_build_parser().format_help()` に `keypoints_path` が含まれる |

- テスト結果は `tests/results/feat-032_test_result.txt` に `uv run pytest -v` の出力をそのまま保存する

### 実データでの動作確認（実装ステップ内で実施）

phase4 venv で以下を実行し、終了コード 0・出力 MP4 の生成を確認する（画質・重なりの妥当性判断は手動テストでユーザーが行う）:

```bash
# phase4/ ディレクトリで実行
TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run python render_keypoints.py \
    /home/sakagawa/data/PLY/HandaiHospital-20251024-01/point_cloud/iteration_100/point_cloud.ply \
    data/Blender/handai-hosp1_20251024.toml \
    data/session001_f145749_world300_filtered.npz \
    --camera int_cam01_img --near-plane 0.5 \
    --output-dir data/feat032_check --no-png --mp4
```

期待値: 「キーポイント読み込み中（NPZ, 全フレーム）」表示、22/28 マーカー描画対象・欠損 6 点表示、300 フレーム処理、fps=30 警告つきで MP4 生成。

## 7. 設計判断（ADR 簡易版）

### D-1: world座標 → C3D raw(mm) に変換して既存パイプラインへ合流（採用）

- **採用**: NPZ の world座標[m]を `world_to_c3d_raw` で C3D raw(mm) に変換し、frames_data 以降の処理（`extract_keypoints` → `c3d_to_calib` → 投影）を無変更で共用する
- **却下案**: パイプラインをキャリブ座標[m]ベースに再構成し、C3D 経路側で変換する
- **理由**: 却下案は `extract_keypoints` 以降の全関数と既存テスト（feat-016/017/021/022/024）に波及する。採用案の往復変換 `c3d_to_calib(world_to_c3d_raw(w)) == w` は feat-018 で数学的恒等（軸入替と ×1000/×0.001 のみ）と確認・テスト済みで、float64 演算での誤差は問題にならない

### D-2: `npz_to_c3d.py` の `load_npz` / `world_to_c3d_raw` を import 再利用（採用）

- **採用**: 検証ロジックと座標変換を feat-018 実装から import する
- **却下案**: `render_keypoints.py` 内に同等ロジックを再実装する
- **理由**: プロジェクトには読み込み関数の重複実装という既知の技術的負債があり（CLAUDE.md ドメイン知識）、新規の重複を増やさない。npz_to_c3d のトップレベル import は軽量（c3d は関数内 import 済み）で、テスト実行環境（root venv）でも import 可能

### D-3: 拡張子による形式判別（採用）

- **採用**: パス末尾 `.npz`（大小無視）で判別。それ以外は C3D
- **却下案**: `--input-format {c3d,npz}` オプションの新設、またはファイル magic バイト判定
- **理由**: 本プロジェクトの NPZ/C3D はすべて拡張子つきで運用されており、オプション追加は CLI を複雑にする。magic 判定は c3d ライブラリの読み込みを試みる必要があり早期検証の順序（重い処理前）を崩す

### D-4: 位置引数 `c3d_path` を `keypoints_path` に改名（採用）

- **採用**: 改名し、既存テスト 1 箇所（5.3）を追随させる
- **却下案**: `c3d_path` のまま help のみ更新（テスト無変更）
- **理由**: NPZ パスを `c3d_path` に格納するのは誤読を招く。CLI 呼び出し（位置引数）に互換性影響はなく、追随変更は assert 1 行のみで影響範囲が明確

### D-5: `pnp_ok` を無視して全フレーム描画（採用）

- **採用**: `pnp_ok` を参照しない。有効性は座標の有限性のみで決める
- **却下案**: `pnp_ok=False` フレームを背景のみにする／オプションで切り替え
- **理由**: ユーザー決定（2026-08-13）。テストデータでは pnp_ok=True が 9/300 しかなく、False を弾くと検証目的（キャリブの時系列目視検証）に使えない。将来必要になればオプション追加の後続案件とする

### D-6: `FileNotFoundError` の捕捉追加（採用）

- **採用**: main のロード部の except に `FileNotFoundError` を追加（C3D 経路にも適用）
- **却下案**: 現行どおり ValueError のみ捕捉（ファイル不存在はトレースバック）
- **理由**: NPZ 対応でロード部に手を入れるため、同一 try 節の 1 行変更で両形式のエラー表示が揃う。挙動変化は「トレースバック → 1 行メッセージ + 終了コード 1」の改善方向のみ

### D-7: `load_npz` の正規化仕様を要求の正とし、既存関数を変更しない（採用）

- **採用**: `npz_to_c3d.load_npz` の既存の正規化（`frame_ids` reshape(-1)+int64化、`joint_names` str化）と `allow_pickle=True` をそのまま踏襲し、要求仕様（FR-002）をこの動作に合わせて定義する。読み込み例外の正規化（`ValueError`/`FileNotFoundError` 以外 → `ValueError`）は `load_npz_all_frames` 側で行う
- **却下案**: `load_npz` に厳格検証（`frame_ids.ndim == 1`・整数 dtype・文字列 dtype の明示チェック）や `allow_pickle=False` 化を追加する
- **理由**: `load_npz` は Closed 済みの feat-018（`npz_to_c3d.py`）の共有関数であり、変更は既存機能への回帰リスクになる。入力 NPZ は自プロジェクトのリフトアップパイプラインが生成するローカルファイルに限られ（信頼境界を越えない）、正規化で解釈できる形状揺れを弾く実益がない。pickle 由来のコード実行リスクも同じ理由で脅威にならない

## 8. 完了時の更新対象（ステップ8で本体が実施）

- `docs/BACKLOG.md`: feat-032 を Closed に
- `docs/CHANGELOG.md`: 完了内容を記録
- `CLAUDE.md`: ディレクトリ構成の `render_keypoints.py` 説明に NPZ 対応を追記（ファイル追加・削除はなし）
- `README.md`: `render_keypoints.py` の使用方法（入力形式・コマンド例）に NPZ を追記

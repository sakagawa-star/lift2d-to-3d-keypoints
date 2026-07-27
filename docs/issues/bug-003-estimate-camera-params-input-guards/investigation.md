# bug-003 調査・修正計画

## イテレーション1 (2026-07-27)

対象: `phase0/estimate_camera_params.py`
発端: Claude Code によるコードレビュー（`docs/REVIEW_CRITERIA.md` 準拠）で重要度「高」1件・「中」3件を検出。

改訂履歴（イテレーション1内）:
- 初版 → Codex レビュー（codex-01）の指摘4件を反映して改訂（要求への紐付け明確化、TOMLキー欠落時の終了コード1保証、入力ガード範囲の拡大、テスト計画の修正）
- Codex 再レビュー（codex-02）の指摘2件を反映して改訂（CSV 検証を `is_file()` に変更、K既知経路の config 検証テストを追加）

---

## 1. 不具合の特定

### 1.1 対応する要求ID・設計セクション

**feat-002（`docs/issues/feat-002-wide-lens-support/`）の以下に紐付ける:**

- **requirements.md FR-6**: 「`run_estimation` 関数は正常終了時に `return 0`、エラー時に `return 1` を返す」「エラーメッセージを出力して終了コード1で終了する」— 本スクリプトの異常系の基本仕様。H-1・M-3 の「トレースバックで異常終了する」動作はこの仕様に違反する（エラーメッセージなし・Python 例外による終了は「エラーメッセージを出力して終了コード1」ではない）
- **requirements.md FR-3 / design.md「定数追加」節**: 最小点数の制御。`MIN_POINTS_*` 定数群による点数制御が仕様化されているが、「歪みなしモデル自体が実行可能な最小点数」のチェックは未定義のまま実装漏れとなった（M-1: `MIN_POINTS_NO_DIST` 未使用）
- **design.md:296**: 「マッチング点数が0点の場合のエラーハンドリングは既存コードにも存在しない問題であり、本案件のスコープ外とする」— H-1 は feat-002 時点で**既知の未処理問題としてスコープ外宣言されたまま放置された不具合**であり、本案件で解消する

**K既知モード（feat-001/003）**: `estimate_extrinsic` は点数不足時に日本語エラーメッセージ＋戻り値 `None`（呼び出し側でスキップ／終了コード1）とする異常系が実装済み。M-3 のうち TOML キー欠落は、この確立済みの異常系方針（FR-6 と同じ「メッセージ＋終了コード1」）に対する処理漏れである。

M-2（収束判定なし）は feat-002 FR-2「`scipy.optimize.least_squares`（method='lm'）で最適化」の実装において、最適化が収束しなかった場合の扱いが未定義のまま成功時と同一の出力を行うもので、REVIEW_CRITERIA.md 2.1 の「潜在的バグ（中）」に該当する。本書セクション3.1(b) をその仕様定義とする（BUGFIX_STANDARD.md 2.2 に基づく設計変更案の提示。独立した design.md が存在しないため、本 investigation.md が仕様の典拠となる）。

### 1.2 現在の動作（再現手順）

**H-1: 点数不足でトレースバック終了（重要度: 高）**

1. 対象カメラのマッチ点数が5点以下になる入力を用意する
2. `uv run python estimate_camera_params.py config.yaml` を phase0/ で実行する
3. 結果:
   - 点数4〜5: `scipy.optimize.least_squares(method='lm')` が「残差数(2N) < 変数数(≥8)」で `ValueError` を送出（例: 4点・主点推定・歪みなし = 残差8 < 変数10）
   - 点数0〜3: `cv2.solvePnP(SOLVEPNP_ITERATIVE)` が `cv2.error` を送出（0点は空配列で即例外）
   - いずれも Python トレースバックで異常終了する（FR-6 違反）

**M-1: `MIN_POINTS_NO_DIST` 未使用（重要度: 中）**

`estimate_camera_params.py:37` で定義されているが、全コード中で参照箇所がゼロ。H-1 のガードに使われるべき定数（FR-3 の点数制御の実装漏れ）。

**M-2: `least_squares` の収束判定なし（重要度: 中）**

通常モードの全10箇所の `least_squares(...)` 呼び出しで、戻り値 `result.status` を確認せず `result.x` を採用している。収束失敗（`status = 0`: 反復回数上限到達）でも通常の推定結果として表示され、ユーザーが失敗に気づけない。

**M-3: 必須キー・入力ファイルの検証漏れで生の例外（重要度: 中）**

以下の入力不正で、日本語エラーメッセージなしにトレースバックで終了する（FR-6 違反）:

1. **config.yaml の必須キー欠落**（`KeyError`）:
   - `main()` 複数カメラチェック（`:1221`）: `config['target_camera']`
   - `run_estimation`（`:634-636`）: `config['target_camera']`, `config['points_3d']`, `config['points_2d']`
   - `_run_extrinsic_estimation`（`:417-422`）: 同上
2. **points_3d / points_2d の CSV ファイル不在**（`FileNotFoundError`）: `common.py` の `load_points_3d`（`:22`）/ `load_points_2d`（`:33`）は `open()` を直接呼ぶため、config に書かれたパスが存在しないと生例外になる。既存のファイル存在チェックは `main()` の config 自体と `--intrinsic-toml` のみで、CSV は未チェック
3. **内部パラメータ TOML のセクション内キー欠落**（`KeyError`）: `load_intrinsic_toml`（`:265-267`）の `cam_data['matrix']`, `cam_data['distortions']`, `cam_data['size']`

### 1.3 期待する動作

- **H-1/M-1**: 通常モードでマッチ点数が `MIN_POINTS_NO_DIST`（6点）未満の場合、「エラー: 基準点が{N}点しかありません。最低6点必要です。」と表示して終了コード1で終わる（FR-6 準拠。K既知モードの `estimate_extrinsic` の既存メッセージ形式に合わせる）
- **M-2**: 収束失敗時（`result.status <= 0`）は警告メッセージを表示する。処理は継続し、結果表示・TOML/CSV 出力は行う（feat-025 の範囲チェックと同じ「警告」の位置づけ。終了コードは0のまま）
- **M-3**: 上記 1.2 の入力不正 1〜3 のすべてで、日本語エラーメッセージを表示して**終了コード1**で終わる。特に TOML セクション内キー欠落（3.）は、複数カメラ処理中の1台であっても「壊れた入力」であるため**スキップ扱いにせず即終了コード1とする**（「カメラセクション自体の不在」は従来どおり警告＋スキップ。欠落と破損を区別する）

---

## 2. 原因分析

- **原因箇所**: 上記 1.2 の各該当箇所
- **原因の説明**: 初期実装時に正常系のみ実装され、異常系ガードが後続の feat 案件（FR-6 の solvePnP 失敗、K既知モードの点数チェック等）で部分的にのみ整備された。通常モードの点数ガード・入力検証・収束判定は最初から存在せず、feat-002 design.md:296 で明示的に先送りされたまま未解消
- **根本原因 or 表面的原因**: 根本原因（チェック処理自体の欠落）。例外の握りつぶし等の対症療法ではなく、入口での妥当性検証を追加する

---

## 3. 修正内容

### 3.1 変更対象ファイル

**`phase0/estimate_camera_params.py`** のみ。

**(a) H-1/M-1: 最小点数ガードの追加**

`run_estimation` のマッチング直後（`num_points` 算出・点名一覧表示の後、`min_points` 決定の前）に追加:

```python
if num_points < MIN_POINTS_NO_DIST:
    print(f"\nエラー: 基準点が{num_points}点しかありません。最低{MIN_POINTS_NO_DIST}点必要です。")
    return 1
```

根拠: 6点あれば (1) `cv2.solvePnP(ITERATIVE)` の点数要件を満たし、(2) 歪みなしモデルの最大変数数10（主点推定モード）に対し残差12 ≥ 10 で `method='lm'` の要件を満たす。歪み係数を推定する分岐は既存の `min_points`（最小12）判定で保護済み。

**(b) M-2: 収束判定の追加**

各分岐の `result = least_squares(...)` は共通化せず、結果表示部の直前（`reproj_error` 算出後、「推定結果」ヘッダー表示の前）に1箇所追加:

```python
if result.status <= 0:
    print(f"\n⚠ 警告: 最適化が収束しませんでした（status={result.status}: {result.message}）")
    print("→ 推定結果が信頼できない可能性があります。点対応・点数を確認してください。")
```

`least_squares` の `status` は正値が収束、0 が反復上限到達（`method='lm'` では負値は発生しないが、防御的に `<= 0` とする）。処理は継続する。

**(c) M-3-1/M-3-2: config 必須キーと CSV ファイル存在の検証**

`run_estimation`（通常モード）と `_run_extrinsic_estimation` のそれぞれで、`load_yaml_simple` 直後・CSV ロード前に追加（2箇所同型）:

```python
for key in ('target_camera', 'points_3d', 'points_2d'):
    if key not in config or not str(config[key]).strip():
        print(f"エラー: 設定ファイルに {key} がありません: {config_path}")
        return 1

points_3d_path = config_dir / config['points_3d']
points_2d_path = config_dir / config['points_2d']
for path in (points_3d_path, points_2d_path):
    if not path.is_file():
        print(f"エラー: 入力ファイルが見つかりません（またはファイルではありません）: {path}")
        return 1
```

`exists()` ではなく `is_file()` を使う。存在するディレクトリ（例: `points_2d: data/`）を指定した場合に `open()` の `IsADirectoryError` になることを防ぐため。

- キー欠落と空文字列値を同一扱いとする（`not str(config[key]).strip()`）
- `main()` の複数カメラチェック（`:1219-1225`）は `config.get('target_camera')` を使い、欠落・空なら「エラー: 設定ファイルに target_camera がありません: {config_path}」を表示して `return 1`（`run_estimation` 側と重複するが、`main()` 側が先に参照するため両方必要）
- `common.py` は変更しない（3.2 参照）

**(d) M-3-3: TOML セクション内キー欠落は即終了コード1**

「セクション不在（=そのカメラのデータがない）」と「セクションはあるが壊れている」を区別する:

- `load_intrinsic_toml` は、セクション不在時は従来どおりエラーメッセージ表示＋`None` 返却（呼び出し側は警告＋スキップ継続）
- セクション内の必須キー `matrix` / `distortions` / `size` の欠落時は、「エラー: TOMLの [{camera_name}] に {キー名} がありません: {toml_path}」を表示して `ValueError(メッセージ)` を送出する
- `_run_extrinsic_estimation` は `load_intrinsic_toml` の呼び出しを `try/except ValueError` で囲み、捕捉したら（他カメラの成否にかかわらず）**即 `return 1`** する。ファイル出力（`--output`）も行わない

これにより「複数カメラ中1台だけ TOML が壊れている場合に全体が成功扱い（終了コード0）になる」ことを防ぐ。

### 3.2 変更しないファイル

- `phase0/common.py`: `load_points_3d` / `load_points_2d` / `load_yaml_simple` 自体は変更しない。キーの要否・ファイル存在の検証は呼び出し側の文脈に依存するため（例: `camera_params` キーは検証スクリプトのみ必須）、チェックは呼び出し側に置く。ファイル存在チェックも 3.1(c) でロード前に行うため関数内の変更は不要
- K既知モードの `estimate_extrinsic`: 点数ガードは実装済みのため変更しない
- `phase0_verification.py` / `verify_triangulation.py` / `convert_toml_to_csv.py` / `visualize_points_2d.py`: 本スクリプトを import していないため変更不要

### 3.3 設計書との整合性

- H-1/M-1/M-3 は feat-002 FR-6 が定めた異常系方針（メッセージ＋終了コード1）を、feat-002 design.md:296 で先送りされた箇所および検証漏れ箇所に適用するもので、既存設計書と矛盾しない
- M-2 の収束警告と M-3-3 の「破損セクションは即終了」は既存設計書に記述のない新規仕様であり、BUGFIX_STANDARD.md 2.2 に基づき本書 1.3 / 3.1 をその仕様定義（設計変更案）とする
- feat-025 design.md（範囲チェック）とは独立の表示ブロックであり矛盾しない

---

## 4. 影響範囲

- **他の機能への影響**:
  - 通常モードの正常系（6点以上・キー完備・ファイル存在・収束成功）は出力が一切変わらない
  - K既知モードで従来「`KeyError` でクラッシュしていた」壊れた TOML 入力が、メッセージ付き終了コード1に変わる。正常な TOML では挙動不変
  - K既知モードの複数カメラ処理で、壊れたセクションがあると従来スキップ相当（実際はクラッシュ）だったものが全体エラー終了になる — これは 1.3 で定義した期待動作
- **リグレッションリスク**:
  - `load_yaml_simple` の戻り値のキー名は YAML キーそのものであり、キーチェックの偽陽性リスクは低い
  - 既存テスト・既存の実データ config が相対パスで CSV を指定している場合、`config_dir` 基準のパス解決は既存コードと同一式を使うため挙動は変わらない
  - 既存テストが5点以下の入力やキー欠落 config で通常モードを実行している場合は失敗するようになる（→ 5. の pytest 全件実行で検出する）

---

## 5. 確認方法

### 5.1 自動テスト（pytest、tests/ に追加）

`tests/test_bug003_input_guards.py` を新規作成する。テストデータは `tmp_path` 上に最小の CSV/YAML/TOML を生成する（既存テストのスタイルに合わせる）。検証項目:

1. **H-1**: マッチ点数5点の入力で `run_estimation` が 1 を返し、標準出力（capsys）に「最低6点必要」を含む
2. **H-1 境界**: マッチ点数6点（歪みなしモード）で 0 を返して正常終了する
3. **M-3-1（関数レベル）**: `target_camera` / `points_3d` / `points_2d` の各キー欠落および空文字列の config で `run_estimation` が 1 を返し、「がありません」を含む
4. **M-3-1（CLI レベル）**: `target_camera` 欠落 config に対し `main()` を実行（`monkeypatch.setattr(sys, 'argv', [...])`）して戻り値 1 を確認する（`main()` 側の参照経路 `:1221` を通ることの検証）
5. **M-3-2**: config の `points_2d` が存在しないファイルを指す場合、および `points_3d` が存在する**ディレクトリ**を指す場合に、`run_estimation` が 1 を返し、「入力ファイルが見つかりません」を含む
5b. **M-3-1/M-3-2（K既知経路）**: `run_estimation(..., intrinsic_toml=...)` 経由（通常モードの検証をバイパスする経路）で、`target_camera` / `points_3d` / `points_2d` の欠落・空文字列、および CSV 不在の各ケースで 1 が返ることを検証する
6. **M-3-3**: `matrix` キーを欠いたセクションを含む**2カメラ構成**の TOML で `_run_extrinsic_estimation` が 1 を返す（もう1台が正常でも成功扱いにならないこと）。セクション自体の不在は従来どおりスキップされ、正常カメラのみで 0 を返すことも併せて検証する
7. **M-2**: モジュール内シンボル `estimate_camera_params.least_squares` を monkeypatch し、`status=0` のダミー結果（`x` は実寸の配列、`message` 付き）を返させて「収束しませんでした」の警告出力と戻り値 0 を検証する

実行: `uv run pytest -v` 全件。結果を `tests/results/bug-003_test_result.txt` に保存する。

### 5.2 手動テスト

1. 実データ config で通常モード（オプションなし、`--fix-center`、`--wide`）を実行し、従来と同一の推定結果が出ることを確認する
2. 基準点を4点に減らした points_2d.csv で実行し、日本語エラーメッセージ＋終了コード1（`echo $?`）で終わることを確認する
3. K既知モードで正常 TOML を使った複数カメラ一括推定が従来どおり動くことを確認する

---

## 6. スコープ外（本案件では修正しない）

レビューで検出した重要度「低」の指摘は本案件のスコープ外とする（BUGFIX_STANDARD.md 2.3 スコープの限定）:

- L-1: 推定分岐10ブロックの重複（リファクタリングは別案件）
- L-2: 未使用 `import sys` / `exit()` → `sys.exit()`
- L-3: RANSAC inlier の set 順序の非決定性（実害なし）
- L-4: `estimate_extrinsic` docstring の補足

また、CSV の内容不正（列欠落・数値でない値等）のハンドリングは本案件の対象外とする（レビューで指摘された範囲＝キー欠落・ファイル不在・点数不足・収束失敗に限定する）。

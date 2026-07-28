# bug-004 調査・修正計画

## イテレーション1 (2026-07-27)

対象: `phase0/estimate_camera_params.py`（bug-003 修正後、コミット `7014e10` 時点）
発端: Codex による第三者コードレビュー（`reviews/codex-01.result.md`、REVIEW_CRITERIA.md セクション2準拠）で重要度「高」3件・「中」2件を検出。全件を Claude Code が実コードで確認済み。

改訂履歴（イテレーション1内）:
- 初版 → Codex レビュー（codex-02）の指摘2件を反映して改訂（H-1/H-2 のガードを関数レベルにも配置、M-2 の feat-001 要求仕様書への正式追記を計画に追加）
- Codex 再レビュー（codex-03）の指摘2件を反映して改訂（`--output` 親ディレクトリ検証の集約、feat-001/feat-003 設計書への変更注記を計画に追加）

---

## 1. 不具合の特定

### 1.1 対応する要求ID・設計セクション

- **H-1（入力TOML破壊）/ H-2（パス種別未検証）**: feat-008（複数カメラ一括推定）で導入された `--output` と、feat-002 FR-6 の異常系方針（エラーメッセージ＋終了コード1）に紐付ける。入力ファイルの破壊は REVIEW_CRITERIA.md 2.1 の「データ破損（高）」に該当し、feat-008 の要求（推定結果を出力する）は「入力の内部パラメータTOMLを破壊しない」ことを暗黙の前提とする
- **H-3（TOML値の形状未検証）**: bug-003 (d) で導入したキー存在チェックの検証不足（存在はするが形状・長さが不正な値で後段の OpenCV 呼び出しがクラッシュ）。bug-003 の期待動作「壊れたTOML入力はメッセージ＋終了コード1」の適用範囲拡大
- **M-1（オプション排他チェックの順序）**: feat-023 の requirements「`--intrinsic-toml` 指定時は既存フラグ同様に警告して無視」に違反する（無視されるべき組み合わせが先にエラー終了する）
- **M-2（fisheye フラグ無視）**: feat-001 requirements.md:97 に「`rotation`, `translation`, `fisheye` は読み込まない」と明記されており、現行動作は仕様どおり。ただしこの仕様は `fisheye = true` の TOML（魚眼キャリブレーション）を通常歪みモデルで誤処理して無警告の誤結果を出すことを許すため、REVIEW_CRITERIA.md 2.1 の「潜在的バグ（中）」として仕様自体を変更する。BUGFIX_STANDARD.md 2.2「設計書の変更案も併せて提示する」に従い、feat-001 要求仕様書への変更注記を本計画に含める（3.1(f)。bug-002 で feat-019 ドキュメントに変更注記を追記した前例に倣う）。新仕様: `fisheye = true` は明示的な未対応エラー（docstring「魚眼レンズ（>120°）: 別途fisheyeモデルが必要」と整合）

### 1.2 現在の動作（再現手順）

**H-1: `--output` が入力TOMLを破壊できる（重要度: 高）**

1. `uv run python estimate_camera_params.py config.yaml --intrinsic-toml Calib_scene.toml --output Calib_scene.toml` を phase0/ で実行する
2. `_write_toml_output`（`:404` 相当）が `open(output_path, 'w')` で入力TOMLを上書きする。書き込まれるのは今回成功したカメラのセクションのみのため、対象外カメラ・失敗カメラ・`metadata` セクションが**恒久的に失われる**（データ破損）

**H-2: config / `--intrinsic-toml` / `--output` のパス種別未検証（重要度: 高）**

- `main()` の config チェック（`:1214`）と `--intrinsic-toml` チェック（`:1237`）は `exists()` のみのため、ディレクトリを渡すと後段の `open()` で `IsADirectoryError` のトレースバック終了（FR-6 違反）
- `--output` は親ディレクトリの存在（`:1246-1250`）しか見ておらず、`--output` 自体が既存ディレクトリの場合に書き込み時 `IsADirectoryError` でクラッシュ

**H-3: TOML 値の形状・長さ未検証（重要度: 高）**

`load_intrinsic_toml`（`:265-273`）はキーの存在のみ確認し、値を無検証で `np.array` 化する:
- `distortions = [0.0, 0.0]`（長さ2）→ `cv2.solvePnPRansac` が `cv2.error` でトレースバック終了
- `matrix` が 3x3 でない → `K[0, 2]` 参照や OpenCV 呼び出しでクラッシュ
- `size` が2要素未満・非数値 → `int(size[0])` 等で例外

**M-1: K既知モードで `--wide --zero-tangent` 併用が警告でなくエラー終了（重要度: 中）**

1. `uv run python estimate_camera_params.py config.yaml --intrinsic-toml intrinsic.toml --wide --zero-tangent` を実行する
2. 期待（feat-023 要求）: 「警告: --intrinsic-toml 指定時は --wide, --zero-tangent は無視されます」と表示して K既知モードを実行する
3. 実際: 排他チェック（`:1219`）が K既知モード判定（`:1224`）より先に評価され、「エラー: --zero-tangent と --wide は併用できません」で終了コード1になる

**M-2: TOML の `fisheye = true` を無視して通常歪みモデルで処理（重要度: 中）**

`load_intrinsic_toml` は `fisheye` キーを読み取らず、魚眼キャリブレーションの TOML を通常の OpenCV 歪みモデル（`cv2.solvePnPRansac` / `cv2.projectPoints`）で処理する。魚眼の歪み係数は意味が異なるため、**クラッシュせずに誤った外部パラメータを出力し得る**（無警告の誤結果）。

### 1.3 期待する動作

- **H-1**: `--output` が `--intrinsic-toml` と同一ファイルを指す場合（`Path.resolve()` で比較）、「エラー: --output に入力TOMLと同じパスは指定できません: {path}」を表示して終了コード1。さらに書き込みは一時ファイル経由の原子的書き込みとし、途中失敗で出力先が中途半端な内容にならないようにする
- **H-2**: config / `--intrinsic-toml` は `is_file()` で検証し、不合格なら既存メッセージ形式でエラー終了。`--output` が既存ディレクトリなら「エラー: --output はディレクトリです: {path}」で終了コード1
- **H-3**: `matrix` が 3x3、`distortions` が長さ 4/5/8/12/14 のいずれか、`size` が2要素かつ両方とも正の有限数、を検証し、不正なら「エラー: TOMLの [{camera_name}] の {キー名} が不正です（詳細）: {toml_path}」を表示して `ValueError` 送出（bug-003 (d) で確立した既存のエラー終了経路に乗せ、終了コード1）
- **M-1**: `--intrinsic-toml` 指定時は排他チェックをスキップし、既存の「無視されます」警告のみ表示して K既知モードを実行する（K未知モードでの排他エラーは従来どおり）
- **M-2**: セクションに `fisheye = true` がある場合、「エラー: [{camera_name}] は fisheye = true です。魚眼モデルは未対応です: {toml_path}」を表示して `ValueError` 送出（終了コード1）。`fisheye` キーがない場合と `false` の場合は従来どおり処理する

---

## 2. 原因分析

- **原因箇所**: 上記 1.2 の各該当箇所（`main()` のオプション検証部、`_write_toml_output`、`load_intrinsic_toml`）
- **原因の説明**:
  - H-1/H-2: feat-008 で `--output` を導入した際、出力先の検証は「親ディレクトリの存在」のみ設計され、入力との衝突・パス種別が考慮されなかった
  - H-3: bug-003 (d) はレビュー指摘の範囲（キー欠落）のみを検証対象とし、値の形状は対象外だった
  - M-1: feat-023 で排他チェックを「重い処理の前に判定」として `main()` 冒頭寄りに置いた際、K既知モードの「無視して警告」判定（それより後ろ）との順序関係が考慮されなかった
  - M-2: `fisheye` キーは Calib_scene.toml 形式の出力互換のために書き出されているだけで、読み込み側の扱いが未定義だった
- **根本原因 or 表面的原因**: いずれも根本原因（検証・分岐の欠落）。対症療法ではなく入口での検証追加・分岐条件の修正を行う

---

## 3. 修正内容

### 3.1 変更対象ファイル

- **`phase0/estimate_camera_params.py`**（コード修正 (a)〜(e)）
- **`docs/issues/feat-001-estimate-extrinsic/requirements.md` / `design.md`、`docs/issues/feat-003-integrate-extrinsic/design.md`**（M-2/H-3 に伴う変更注記の追記 (f)）

**(a) H-1: 同一パス防止と原子的書き込み**

同一パス防止は `main()` ではなく **`_run_extrinsic_estimation` の入口**（config キー検証の後）に置く。これにより `run_estimation(..., intrinsic_toml=..., output_path=...)` の関数直呼び経路（テストや将来の再利用）でも防止が効く:

```python
if output_path and Path(output_path).resolve() == Path(toml_path).resolve():
    print(f"エラー: --output に入力TOMLと同じパスは指定できません: {output_path}")
    return 1
```

`_write_toml_output` を原子的書き込みに変更:

```python
tmp_path = Path(output_path).with_suffix(Path(output_path).suffix + '.tmp')
with open(tmp_path, 'w', encoding='utf-8') as f:
    ...  # 既存の書き込み処理
tmp_path.replace(output_path)
```

**(b) H-2: パス種別の検証（main と関数入口の両方）**

CLI 経路（早期終了のため）と関数直呼び経路の両方を守る:

- `main()`: config チェック（`:1214`）と `--intrinsic-toml` チェック（`:1237`）の `exists()` を `is_file()` に変更（メッセージは既存のまま「見つかりません」）
- `run_estimation` の先頭（K既知/K未知の分岐前）: `Path(config_path).is_file()` を検証し、不合格なら「エラー: 設定ファイルが見つかりません: {config_path}」で `return 1`
- `_run_extrinsic_estimation` の入口: `Path(toml_path).is_file()` を検証し、不合格なら「エラー: TOMLファイルが見つかりません: {toml_path}」で `return 1`。`output_path` については以下を同一ブロックで検証する（(a) の同一パス防止も同ブロック）:
  - `Path(output_path).is_dir()` → 「エラー: --output はディレクトリです: {output_path}」で `return 1`
  - `Path(output_path).parent.is_dir()` でない → 「エラー: 出力先ディレクトリが存在しません: {parent}」で `return 1`（親が存在しない場合の `FileNotFoundError` と、親が既存ファイルの場合の `NotADirectoryError` の両方を防ぐ。`is_dir()` はどちらも False になる）
- `main()` 側の既存の親ディレクトリチェック（`:1246-1250`, `exists()`）は**削除し、`_run_extrinsic_estimation` に集約する**（`exists()` のままだと親が既存ファイルの場合を通してしまうため。CLI 経路も同関数を必ず通るので検証は失われない）

**(c) H-3: TOML 値の形状・長さ検証**

`load_intrinsic_toml` のキー存在チェック直後、`return` 前に追加。不正時は「エラー: TOMLの [{camera_name}] の {キー名} が不正です（{理由}）: {toml_path}」を表示して `ValueError(メッセージ)` を送出:

- `K = np.array(...)` が例外を出すケース（非数値・不揃いのネスト）も `try/except (ValueError, TypeError)` で同メッセージに変換する
- `K.shape == (3, 3)` でなければ不正（理由: 「3x3ではありません」）
- `dist.ndim == 1 and dist.size in (4, 5, 8, 12, 14)` でなければ不正（理由: 「長さが4/5/8/12/14のいずれでもありません」）
- `size` は長さ2、かつ各要素が `float()` 変換可能で正の有限値（`math.isfinite` かつ > 0）でなければ不正

**(d) M-1: 排他チェックの条件限定**

`main()` の排他チェック（`:1219`）を以下に変更（位置は移動しない）:

```python
if not args.intrinsic_toml and args.zero_tangent and args.wide:
```

これにより K既知モードでは既存の「無視されます」警告（`:1224-1235`）のみが表示される。

**(e) M-2: fisheye 未対応エラー**

`load_intrinsic_toml` のセクション取得後（キー存在チェックと同じブロック）に追加:

```python
if cam_data.get('fisheye', False):
    message = f"エラー: [{camera_name}] は fisheye = true です。魚眼モデルは未対応です: {toml_path}"
    print(message)
    raise ValueError(message)
```

呼び出し側（`_run_extrinsic_estimation`）は bug-003 (d) の `try/except ValueError` で捕捉済みのため変更不要（即 `return 1`、ファイル出力なし）。

**(f) M-2 に伴う要求仕様書・設計書の変更（feat-001 / feat-003）**

以下の3ファイルに変更注記を追記する:

1. `docs/issues/feat-001-estimate-extrinsic/requirements.md` の「`rotation`, `translation`, `fisheye` は読み込まない」（:97）の直後:

> **変更注記（2026-07-27, bug-004）**: `fisheye` は読み込み、`true` の場合は「未対応エラー」として終了コード1で終了するよう仕様変更した。魚眼キャリブレーションの TOML を通常の OpenCV 歪みモデルで誤処理し、無警告で誤った外部パラメータを出力することを防ぐため。`fisheye = false` およびキーなしの動作は従来どおり。

2. `docs/issues/feat-001-estimate-extrinsic/design.md` の `load_intrinsic_toml` を定義している節に:

> **変更注記（2026-07-27, bug-004）**: `load_intrinsic_toml` は `fisheye = true` のセクションに対し、エラーメッセージを表示して `ValueError` を送出する（呼び出し側で捕捉し終了コード1）。`false` / キーなしは従来どおり。あわせて `matrix`（3x3）・`distortions`（長さ4/5/8/12/14）・`size`（2要素の正の有限数）の形状検証を追加した。

3. `docs/issues/feat-003-integrate-extrinsic/design.md`（統合後の現行実装を定義する設計書）の該当節にも 2. と同一の変更注記を追記する。

### 3.2 変更しないファイル

- `phase0/common.py`: 対象外（本案件の指摘はすべて estimate_camera_params.py 内）
- `convert_toml_to_csv.py` / その他スクリプト: 本スクリプトを import しておらず影響なし。`convert_toml_to_csv.py` 自体の fisheye / 形状検証は本案件のスコープ外（必要なら別案件）

### 3.3 設計書との整合性

- H-1/H-2 は feat-008 の `--output` 検証の強化であり、既存要求（結果をTOML出力する）と矛盾しない。原子的書き込みは出力内容を変えない（書き込み方式のみの変更）
- H-3/M-2 は bug-003 investigation.md で確立した「壊れたTOMLは ValueError → 即終了コード1」経路の適用範囲拡大であり、同設計と整合する
- M-1 は feat-023 requirements の「`--intrinsic-toml` 指定時は警告して無視」への準拠回復であり、feat-023 の設計意図どおりに戻す修正

---

## 4. 影響範囲

- **他の機能への影響**:
  - 正常系（正しいパス・正しいTOML・fisheye なし/false）は出力・挙動とも一切変わらない（(a) の一時ファイル経由書き込みは最終結果同一）
  - K既知モードで `--wide --zero-tangent` を併用した場合の挙動が「エラー終了」→「警告して無視」に変わる（feat-023 要求への準拠回復。1.3 で定義した期待動作）
  - fisheye = true の TOML は従来「無警告で誤処理」だったものがエラー終了になる（明示的な非互換だが、従来の結果は信頼できないため許容する）
- **リグレッションリスク**:
  - `is_file()` 化により、シンボリックリンク経由の指定は `is_file()` がリンク先を解決するため従来どおり通る
  - `tests/test_bug003_input_guards.py` を含む既存テストへの影響は pytest 全件実行で確認する（`--wide --zero-tangent` 排他のテストが feat-023 で存在する場合、K未知モードのテストであれば影響なし）

---

## 5. 確認方法

### 5.1 自動テスト（pytest、tests/ に追加）

`tests/test_bug004_toml_output_guards.py` を新規作成する。テストデータは `tmp_path` 上に生成。検証項目:

1. **H-1**: `--output` = `--intrinsic-toml`（相対/絶対の表記違いを含む）で `main()` 経由が 1 を返し、「同じパスは指定できません」を含み、**入力TOMLの内容が変化していない**こと。同じ検証を `run_estimation(..., intrinsic_toml=..., output_path=...)` **関数直呼び**でも行う
2. **H-1（原子性）**: `_write_toml_output` 実行後に `.tmp` ファイルが残っていないこと、出力内容が従来形式と同一であること
3. **H-2**: config / `--intrinsic-toml` にディレクトリを渡すと `main()` 経由・`run_estimation` 直呼びの両方で 1 が返る（K既知/K未知両モードの config を含む）。`--output` について以下の各ケースを `main()` 経由・`run_estimation` 直呼びの両方で検証し、いずれも 1 が返る: 既存ディレクトリを指定（「ディレクトリです」）、親ディレクトリが存在しない（「出力先ディレクトリが存在しません」）、親パスが既存ファイル（同メッセージ）
4. **H-3**: `matrix` 非3x3、`distortions` 長さ2、`size` 1要素・負値・非数値の各TOMLで `load_intrinsic_toml` が `ValueError` を送出し、`_run_extrinsic_estimation` 経由で 1 が返る
5. **M-1**: `--intrinsic-toml --wide --zero-tangent` で排他エラーにならず「無視されます」警告が出て K既知モードが実行される（正常TOMLで戻り値 0）。K未知モードの `--wide --zero-tangent` は従来どおりエラー（戻り値 1）
6. **M-2**: `fisheye = true` のセクションで `ValueError` → 1。`fisheye = false` およびキーなしは従来どおり処理される

実行: `uv run pytest -v` 全件。結果を `tests/results/bug-004_test_result.txt` に保存する。

### 5.2 手動テスト

1. 実データの K既知モード（正常TOML・`--output` 指定）で従来と同一の出力TOMLが生成されることを確認する（diff で比較）
2. `--output` に入力TOMLと同じパスを指定してエラー終了し、入力TOMLが無傷であることを確認する
3. K未知モードの通常実行（オプションなし）が従来どおり動くことを確認する

---

## 6. スコープ外（本案件では修正しない）

- 魚眼モデルの実処理対応（`cv2.fisheye` 系の別経路実装）: 明示エラー化のみ行う。必要になれば feat 案件として要求定義から行う
- `convert_toml_to_csv.py` 等、他スクリプトの同種検証
- 方式1で既知の重要度「低」4件（bug-003 investigation.md セクション6参照）

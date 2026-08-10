# feat-029 機能設計書: render_fps_video.py の YAML 設定ファイル読み込み

作成日: 2026-08-09
基準: `docs/DESIGN_STANDARD.md`
対応要求: `requirements.md`（feat-029）

---

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001 `--config` と読み込み | §4.1, §4.3 |
| FR-002 設定キーと型変換・検証 | §4.2 |
| FR-003 優先順位（CLI > YAML） | §4.3 |
| FR-004 必須項目の統合後検証 | §4.4 |
| FR-005 エラー検出 | §4.2, §4.4 |
| NFR-001 後方互換 | §4.5, §7 |

## 1.2 システム構成

変更対象は `phase4/render_fps_video.py` のみ（既存関数は変更せず、引数解析部のみ改修・追加）。

```
render_fps_video.py
  ├─ _build_parser()            # 変更（§4.5: 位置引数を任意化、required 除去、--config 追加）
  ├─ load_yaml_flat()           # 新規（簡易フラットパーサー。phase0 load_yaml_simple と同仕様）
  ├─ parse_config_yaml()        # 新規（キー検証・型変換）
  ├─ parse_args_with_config()   # 新規（プレパース → set_defaults → 本パース → 統合後必須検証）
  └─ main()                     # 変更（冒頭の parse_args 呼び出しを parse_args_with_config に差し替えるのみ）
```

## 1.3 技術スタック

- 新規ライブラリなし（標準ライブラリ + 既存依存のみ。PyYAML 不採用は要求で確定済み）
- `docs/TECH_STACK.md` の更新: 不要

## 4. 詳細設計

### 4.1 簡易フラットパーサー（FR-001）

```python
def load_yaml_flat(yaml_path: str) -> dict[str, str]:
    # phase0/common.py load_yaml_simple と同仕様:
    # - utf-8 で読み、行ごとに strip
    # - 空行と '#' 始まりの行はスキップ
    # - ':' を含む行を最初の ':' で分割し、key.strip(): value.strip() を返す（値は常に文字列）
    # - ':' を含まない非空行はスキップ（load_yaml_simple と同挙動）
    # - ファイル不在・読み取り不能は OSError をそのまま送出（呼び出し側で捕捉）
```

phase0/common.py は import しない（phase4 は独立 uv 環境。重複実装はプロジェクト慣例。ADR-3）。

### 4.2 設定キーの検証・型変換（FR-002, FR-005）

モジュール定数として全設定キーと変換関数を定義する:

```python
CONFIG_CONVERTERS: dict[str, Callable[[str], object]] = {
    "ply_path": str, "npz_path": str, "toml": str, "camera": str,
    "output": str, "preset": str, "still_dir": str, "dump_poses": str,
    "fps": _positive_float, "gpu": _nonnegative_int,
    "chunk_size": _positive_int, "crf": _crf_int,
    "overwrite": _yaml_bool, "keep_chunks": _yaml_bool,
    "still_range": _yaml_still_range,
}
```

- `_yaml_bool(s)`: `"true"→True`, `"false"→False`、それ以外は `argparse.ArgumentTypeError`（小文字のみ許可）
- `_yaml_still_range(s)`: `s.split()` が2要素で各要素が `int()` 変換できれば `[int, int]` を返す。それ以外は `ArgumentTypeError`
- 数値系は CLI と同一の既存検証関数（`_positive_float` 等）をそのまま使う（検証の単一ソース化）

```python
def parse_config_yaml(yaml_path: str, parser: argparse.ArgumentParser) -> dict:
    # 1. load_yaml_flat を呼ぶ。OSError → parser.error(f"--config が読み込めません: {yaml_path}: {e}")
    # 2. "config" キーがあれば parser.error("設定YAMLに config キーは書けません（連鎖読み込み不可）")
    # 3. CONFIG_CONVERTERS にないキーがあれば
    #    parser.error(f"設定YAMLに未知のキーがあります: {キー名}。有効なキー: {sorted(CONFIG_CONVERTERS)}")
    # 4. 各値を変換。argparse.ArgumentTypeError / ValueError / TypeError のいずれも捕捉して
    #    parser.error(f"設定YAML キー '{k}': {メッセージ}") に変換する
    #    （CLI 経由では argparse が type= 呼び出しの ValueError を吸収するが、本関数は検証関数を
    #    直接呼ぶため、`fps: abc` のような int()/float() 由来の ValueError も自前で捕捉する必要がある）
    # 5. {dest名: 変換済み値} を返す
```

`parser.error` は argparse 標準どおり usage とメッセージを stderr に出して終了コード2（FR-005 の要求と一致）。

### 4.3 引数統合（FR-001, FR-003）

```python
def parse_args_with_config(argv=None, parser=None) -> argparse.Namespace:
    # 1. プレパース: argparse.ArgumentParser(add_help=False) に --config のみ登録し
    #    parse_known_args(argv) で config パスを取得（未知引数は無視）
    # 2. parser が None なら parser = _build_parser()（main は自作の parser を渡す。§1.7）
    # 3. config 指定時: defaults = parse_config_yaml(path, parser); parser.set_defaults(**defaults)
    # 4. args = parser.parse_args(argv)
    # 5. 統合後必須検証（§4.4）
    # 6. return args
```

- **優先順位の実現**: `set_defaults` は「デフォルト値の差し替え」なので、CLI で明示指定された値は argparse の通常動作で YAML 値を上書きする。明示指定がなければ YAML 値（= 差し替え後デフォルト）、YAML にもなければ組み込みデフォルトが残る（ADR-1）
- **位置引数**: `nargs="?"` 化（§4.5）により、CLI に書いた個数分だけ先頭から上書きされる（1個なら `ply_path`。argparse の仕様どおり。requirements FR-003 に明記済み）
- **フラグの非対称性**: `overwrite` / `keep_chunks` は `store_true`（デフォルト False）。YAML `true` → デフォルト True に差し替え。CLI からは true 方向のみ（requirements FR-003 のとおり。`--config` の help に明記）
- **still_range**: YAML 由来は `[int, int]` のリストでデフォルトに入り、既存コードの `args.still_range is not None` / `start, end = args.still_range` がそのまま動く

### 4.4 統合後の必須検証（FR-004）

`parse_args_with_config` のステップ5:

```python
REQUIRED_DESTS = [("ply_path", "ply_path"), ("npz_path", "npz_path"),
                  ("toml", "--toml"), ("camera", "--camera"), ("fps", "--fps")]
# getattr(args, dest) is None のものを集め、あれば
# parser.error(f"必須項目が未指定です（CLIまたは--configで指定）: {表示名をカンマ区切り}")
```

- 排他制約・still-dir 必須・ファイル存在などの既存検証（現行 `main()` 冒頭の検証群）は**統合後の args に対して無変更のまま適用される**（`main()` は parse 呼び出しの差し替えのみ）
- マニフェスト（FR-007 再開）への影響なし: マニフェストは統合後の実行パラメータから構築されるため、YAML の内容変更はパラメータ不一致として従来どおり検出される。`--config` のパス自体はマニフェストに含めない

### 4.5 `_build_parser` の変更（FR-001, NFR-001）

| 引数 | 変更前 | 変更後 |
|---|---|---|
| `ply_path` / `npz_path` | 必須位置引数 | `nargs="?"`, `default=None`（help に「--config でも指定可」を追記） |
| `--toml` / `--camera` / `--fps` | `required=True` | `default=None`（required 除去。help は変更なし。欠落検出は §4.4 の統合後検証に移す） |
| `--config` | — | 新規追加。help: 「設定YAML（フラット `key: value`）。優先順位は CLI > YAML > デフォルト。--overwrite/--keep-chunks は CLI から true 方向のみ上書き可」 |

- **後方互換の担保**: `--config` なしの場合、YAML 由来の `set_defaults` が呼ばれないため、従来と同じデフォルト・同じ値になる。必須欠落のエラーが「argparse の required エラー（メッセージ: the following arguments are required）」から「§4.4 の統合後エラー（日本語メッセージ・同じ終了コード2）」に変わる点のみが差分であり、正常系の挙動・値は完全に同一（requirements NFR-001 の「挙動を変えない」はエラーメッセージ文字列を含まない。受け入れは既存テスト全件成功で判定）
- 既存テスト T10（feat-027）は SystemExit(2) を検証しており、メッセージ文字列には依存しない（実装時に確認し、依存があればテスト報告で中断する）

### エラーハンドリング一覧

| エラー | 検出 | 動作 |
|---|---|---|
| `--config` のファイル不在・読み取り不能 | `load_yaml_flat` の OSError | parser.error（終了コード2、パス表示） |
| 未知キー | CONFIG_CONVERTERS との照合 | parser.error（キー名 + 有効キー一覧） |
| `config` キーの連鎖 | キー名チェック | parser.error |
| bool 不正値 / still_range 不正 / 数値範囲違反 / 数値の型変換失敗（`fps: abc` 等） | 変換関数の ArgumentTypeError / ValueError / TypeError（すべて捕捉） | parser.error（キー名 + 理由） |
| 必須欠落（統合後） | §4.4 | parser.error（欠落項目列挙） |

### 境界条件

- 設定YAMLが空（有効行ゼロ）: エラーにしない（`set_defaults({})` 相当。必須欠落は §4.4 が検出）
- 同一キーが設定YAML内に重複: 後の行が勝つ（`load_yaml_flat` の dict 上書き。phase0 と同挙動。仕様として明記）
- 値が空文字列（`toml:` のような行）: 文字列キーは空文字列として通り、後段のファイル存在検証で弾かれる。数値・bool キーは変換エラーになる

## 1.5 状態遷移

対象外（バッチ処理。モードは起動時に確定）。

## 1.6 ファイル・ディレクトリ設計

- 設定YAMLの置き場所・ファイル名は任意（ユーザー管理。git 管理外の `phase4/data/` 配下を想定するが強制しない）
- 書式はフラット `key: value` のみ。キー一覧・型は requirements FR-002 の表が正本

## 1.7 インターフェース定義（追加分）

```python
CONFIG_CONVERTERS: dict[str, Callable]
def load_yaml_flat(yaml_path: str) -> dict[str, str]
def parse_config_yaml(yaml_path: str, parser: argparse.ArgumentParser) -> dict
def parse_args_with_config(argv=None, parser: argparse.ArgumentParser | None = None
                           ) -> argparse.Namespace   # main が呼ぶ唯一の入口
```

- `parse_args_with_config` の戻り値は **`argparse.Namespace` のみに固定する**（tuple 返却は禁止）
- `parser` 引数: None なら内部で `_build_parser()` を呼ぶ（テストからは引数なしで呼べる）。`main()` は自身で `parser = _build_parser()` を作って渡し、以降の既存 `parser.error` 呼び出し（排他検証等）で**同一の parser オブジェクト**を使い続ける（現行 main の構造・エラー文言を維持）
- 既存関数のシグネチャは変更しない

## 6. 設計判断の記録（ADR）

| ID | 採用 | 却下と理由 |
|---|---|---|
| ADR-1 | `parser.set_defaults(**yaml値)` 方式 | (a) YAML→argv トークン展開して前置: 位置引数が CLI と二重になり argparse エラー。(b) SUPPRESS デフォルトの二重パースで明示指定を検出: 実装が複雑で得るものがない |
| ADR-2 | `required=True` を外し統合後検証に移す | `parser._actions` の required 属性を実行時に書き換える案: private API 依存で argparse バージョン変化に脆い |
| ADR-3 | 簡易パーサーを `render_fps_video.py` 内に新規実装 | `phase0/common.py` の import: phase0/phase4 は独立 uv 環境で、phase 間 import の前例がない。重複実装はプロジェクトの既存慣例（CLAUDE.md ドメイン知識） |
| ADR-4 | フラグは CLI から true 方向のみ上書き | `--no-overwrite` 等の否定オプション追加: 本案件のスコープ外の CLI 拡張になる。必要になれば別案件 |

## 7. テスト設計

- テストファイル: `tests/test_feat029_config_yaml.py`（新規。既存 `tests/test_feat027_render_fps_video.py` は変更しない）
- 実行: `uv run --project phase4 --with pytest pytest tests/test_feat029_config_yaml.py -v`。全体回帰 `uv run pytest -v`
- テスト結果保存: `tests/results/feat-029_test_result.txt`
- テストは `parse_args_with_config` の戻り値（Namespace）検証を基本とし、GPU・ffmpeg 不要。E2E は dump-poses モード1件のみ

| # | 対象 | 内容 |
|---|---|---|
| T1 | E2E | 全必須+`dump_poses` を書いた設定YAMLで `main(["--config", path])` が戻り値0、JSON生成（GPU不要） |
| T2 | 優先順位 | YAML `crf: 20` + CLI `--crf 23` → 23。CLI なし → 20。YAML にもなし → 18 |
| T3 | 未知キー | `crff: 18` → SystemExit(2)、メッセージに有効キー一覧 |
| T4 | bool | `overwrite: true` → True、`false` → False、`True`（大文字）→ SystemExit(2) |
| T5 | still_range | `still_range: 100 200` → [100, 200]。要素1個・非整数 → SystemExit(2) |
| T6 | 必須欠落 | fps を CLI にも YAML にも与えない → SystemExit(2)、メッセージに fps |
| T7 | 後方互換 | `--config` なしの従来 argv で Namespace の全値が従来どおり（+ 全体回帰で既存テスト成功） |
| T8 | config 不在 | 存在しないパス → SystemExit(2) |
| T9 | 連鎖禁止 | YAML に `config: other.yaml` → SystemExit(2) |
| T10 | 型検証 | 範囲違反: `crf: 99` / `fps: 0` / `gpu: -1`、型変換失敗: `crf: abc` / `fps: abc` / `gpu: abc` / `chunk_size: abc` → いずれも SystemExit(2)、キー名を含む（ValueError で未捕捉クラッシュしないこと） |
| T11 | フラグ非対称 | YAML `overwrite: true` + CLI フラグなし → True（false に戻す CLI 手段はない仕様の確認） |
| T12 | 排他が統合後に効く | YAML に `dump_poses` と `still_range` の両方 → SystemExit(2)（排他エラー） |
| T13 | 位置引数の上書き | YAML に ply/npz + CLI 位置引数1個 → ply_path のみ上書き |
| T14 | 空YAML・重複キー | 有効行ゼロ → 必須欠落エラーのみ。同一キー重複 → 後の行が勝つ |

## 8. 実装メモ（Sonnet サブエージェント向け注意）

- 環境操作は uv のみ（pip / venv 直接操作は禁止）
- 変更は `phase4/render_fps_video.py` と新規テストファイルのみ。既存の他スクリプト・既存テストファイルは変更しない
- 既存の feat-027 テスト（T10 など）が required エラーの**メッセージ文字列**に依存していないことを実装前に確認する。依存していた場合は実装を中断して報告する（design §4.5 の前提が崩れるため）
- 本設計書のコードスニペットは意図の伝達が目的であり、そのままコピーして使うものではない

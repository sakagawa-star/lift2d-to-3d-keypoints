# bug-006 調査・修正計画

本書は `docs/BUGFIX_STANDARD.md` に従う。対象機能の要求仕様書・機能設計書は feat-041 のもの（`docs/issues/feat-041-intrinsic-loop-render-match/requirements.md`、`design.md`）で、修正に伴う変更案は本書 §1.3 に示す。

---

## イテレーション1 (2026-11-25)

### 1.1 不具合の特定

- **対応する要求ID**: feat-041 requirements.md NFR-004「原本・入力ファイルは変更しない」
- **対応する設計セクション**: feat-041 design.md §4.2.1（出力済み検査と、bug-006 の修正までの運用条件）、§4.2.6（最終精緻化の subprocess 呼び出し）、§5（中間ファイル）、§9 手順 6（事前確認）
- **現在の動作**: `phase4/estimate_intrinsics_loop.py` は、処理を始める前に出力 8 ファイル（`{camera}_loop.toml`、`_final.toml`、`_loop_report.txt`、`_refine_report.txt`、評価の 4 ファイル）だけを `os.path.lexists` で検査する。次の 2 種類のパスは検査しないまま書き込む。
  1. **最終精緻化の一時ファイル** `{out-dir}/{camera}_final.toml.tmp`
     - `refine_extrinsics.py` の `write_output_toml` がここに書く。`open(tmp_path, "w")` で開き、書き込み後に `tmp_path.replace(out_path)` で移動する
     - このパスが入力ファイルへのシンボリックリンクの場合: `open(..., "w")` がリンク先の入力を切り詰めて上書きする
     - このパスが入力ファイルそのものの場合（`--toml {out-dir}/{camera}_final.toml.tmp`）: 入力が上書きされたうえで、`_final.toml` に移動されて元の名前から消える
  2. **`--tmp-dir` 配下の中間ファイル**
     - 次のファイルを、既存の有無を確かめずに書く:
       - `iter{i:02d}_render.png`（`cv2.imwrite`）
       - `iter{i:02d}_matches.npz`（マッチャー CLI）
       - `refine/` 配下（`refine_extrinsics.py` の中間）
     - `--tmp-dir` に既存のディレクトリを指定し、その中に入力を指すリンクや入力そのものが同じ名前で置かれていれば、同じく上書きされる（feat-041 codex-31 指摘1）
- **再現手順**（机上。破壊的なので実行はしない）:
  1. 有効な初期 TOML を `{out_dir}/{camera}_final.toml.tmp` に置く（または、そのパスに入力 TOML へのシンボリックリンクを作る）
  2. `estimate_intrinsics_loop.py --toml <その TOML または元の TOML> --out-dir {out_dir} ...` を実行する
  3. ループが収束して最終精緻化まで進むと、`refine_extrinsics.py` がその一時パスに書き込み、入力 TOML が上書きされる
- **期待する動作**: NFR-004「原本・入力ファイルは変更しない」。出力・中間ファイルの書き込み先が既に存在する場合は、処理を始める前にエラー終了する（終了コード 1）
- **エラーメッセージ**: なし（エラーにならずに入力が上書きされる）

### 1.2 原因分析

- **原因箇所**: `phase4/estimate_intrinsics_loop.py` の `main`（出力済み検査の対象が出力 8 パスだけ）
- **原因の説明**: 出力済み検査は「最終成果物のパス」だけを対象にしていて、「処理の途中で書き込むパス」が対象になっていない。途中で書くパスは次の 2 つ。
  - `refine_extrinsics.py` の一時ファイル。`out_toml + ".tmp"` という固定の名前
  - `--tmp-dir` 配下。名前は反復番号から決まる
  
  `refine_extrinsics.py` 側の `_check_can_write` は `--out-toml` と `--out-report` だけを `exists()` で見ており、一時パスは見ない。しかも `estimate_intrinsics_loop.py` は常に `--overwrite` を付けて呼ぶ（design.md §4.2.6）。
- **根本原因 or 表面的原因**: 根本原因。書き込み先のディレクトリ（`--out-dir`、`--tmp-dir`）として、既存のディレクトリを使えてしまうこと。既存のディレクトリには、入力を指すリンクや入力そのものが、途中で書く名前で置かれ得る。「書き込む前にパスを検査する」方式では、検査と実際の書き込み先がずれる抜け道が残る（下記「取り下げた案」）

### 1.3 修正内容

**方針**: `refine_extrinsics.py` は変更しない（feat-041 requirements.md §5 の既存コード変更禁止。feat-026 の動作を変えない）。`estimate_intrinsics_loop.py` が書き込むディレクトリ（`--out-dir` と、指定時の `--tmp-dir`）を、**存在しないディレクトリとして自分で新しく作る**。作成は `Path(...).mkdir(parents=True, exist_ok=False)` で行う。最後の成分が既に存在すれば `FileExistsError` になる。存在の判定はパスの書き方や途中のシンボリックリンクの解決によらず、OS が作成時に行う。リンク先の無いシンボリックリンクも存在するものとして扱われる。こうして新しく作ったディレクトリの中身は、すべてこのプロセス（と、その子プロセスの `refine_extrinsics.py`・マッチャー CLI）が書いたファイルになる。そのため、途中で書く一時ファイル `{camera}_final.toml.tmp` や中間ファイルが、前から置かれていたリンクや入力に当たることがない。

1. **`--out-dir`**: 存在しないパスを指定する。`mkdir(parents=True, exist_ok=False)` で作り、`FileExistsError` なら終了コード 1（「エラー: 出力ディレクトリが既に存在します（存在しない新しいパスを指定してください）: {パス}」）。これまでの「出力 8 ファイルの `os.path.lexists` による出力済み検査」は、この作成で包含されるので置き換える
2. **`--tmp-dir`**（指定時）: 存在しないパスを指定する。`--out-dir` を作った後に、同じく `mkdir(parents=True, exist_ok=False)` で作る。`FileExistsError` なら終了コード 1（「エラー: 中間ファイル置き場が既に存在します（存在しない新しいパスを指定してください）: {パス}」）。このとき、直前に作った `--out-dir` は空なので `rmdir` で消してから終了する（`rmdir` は空でなければ失敗するので、他のファイルを消すことはない）。`--tmp-dir` 省略時（`tempfile.TemporaryDirectory`）は、毎回新しいディレクトリなので従来どおり
3. **その他の OSError**: 作成時のそれ以外の `OSError`（親が通常ファイル、権限不足など）も終了コード 1 にし、理由を標準エラー出力に出す。`--tmp-dir` の作成で失敗した場合は、2 と同じく作った `--out-dir` を `rmdir` で消す
4. **拒否したときの書き込み**: 1〜3 で拒否した場合は、レポートを含めて何も書かない（従来の出力済み検査と同じ。`--out-dir` を作っていたら 2・3 のとおり消す）
5. **カメラ名の制限**（codex-04 指摘1）: `--camera` は、出力ファイル名（`{camera}_loop.toml` など）にそのまま連結される。そのため、`/` を含む名前（例: `--camera /work/victim`）では、出力が新しく作った `--out-dir` の外に出て、既存のファイルを上書きし得る。これを防ぐため、カメラ名は単一のファイル名成分に限る。`camera == "" or camera in (".", "..") or Path(camera).name != camera or "/" in camera or "\\" in camera` なら終了コード 1（「エラー: カメラ名にはパス区切りを含まない名前を指定してください: {camera}」）。この検査は `--out-dir` の作成より前に行う
6. **後始末の失敗**（codex-04 指摘2）: `--tmp-dir` の作成は、親ディレクトリを途中まで作ってから失敗することがある（例: `--out-dir /work/new --tmp-dir /work/new/sub/..` では `sub` を作った後に `new` で `FileExistsError`）。この場合、`--out-dir` は空でないので `rmdir` は失敗する。後始末の `out_dir.rmdir()` が `OSError` で失敗しても、元のエラーを表示して終了コード 1 で終わる。残ったディレクトリは「エラー: 作成したディレクトリが残っています（手で削除してください）: {out_dir}」と表示する。途中まで作られた親ディレクトリも残り得る（入力は上書きしない。新しく作ったディレクトリしか残らないため）
7. 検査の順序: 引数解析 → カメラ名の制限 → `--out-dir` の作成 → `--tmp-dir` の作成（指定時）→ 以降の処理（従来どおり）

**取り下げた案（行き詰まり検出。codex-01、codex-03）**: 最初は「書き込む前にパスを検査する」方式を採っていた。内容は、出力 8 パスに一時ファイル `{camera}_final.toml.tmp` を加えて `os.path.lexists` で検査し、`--tmp-dir` が存在すれば拒否するもの。しかし、この方式には抜け道が続けて見つかった。

- codex-01: 未作成の親と `..` を含むパス（`--tmp-dir /work/new/../existing`）では、検査時は「存在しない」と判定されるが、`mkdir(parents=True, exist_ok=True)` が既存の `existing` を再利用する。これに対して `..` を拒否する規則を足した
- codex-03: 親成分のシンボリックリンク（`/work/link -> new/../existing`、`--out-dir /work/new --tmp-dir /work/link/sub`）では、`--out-dir` を作った時点でリンクが解決できるようになり、既存の `existing/sub` が再利用される

どちらも、検査した時点と書き込む時点で、パスの指す先が変わることによる。CLAUDE.md の行き詰まり検出に従い、検査を足す方向をやめた。代わりに、書き込み先のディレクトリを `exist_ok=False` で新しく作る方式に切り替えた（codex-03 の修正案の後半と同じ方向）。この方式では `..` の拒否と、出力 8 パスの個別検査は不要になる。

#### 変更対象ファイル

| ファイル | 変更内容 |
|---|---|
| `phase4/estimate_intrinsics_loop.py` | `main` の出力済み検査（出力 8 パスの `os.path.lexists`）と `out_dir.mkdir(parents=True, exist_ok=True)`・`tmp_dir.mkdir(parents=True, exist_ok=True)` を、`exist_ok=False` での新規作成と失敗時の終了コード 1 に置き換える |
| `tests/test_feat041_intrinsics_loop.py` | テスト 23〜27 を追加（下記）。既存テスト 7c・20 は期待値を変えずに通る（既存の出力ディレクトリを使うので終了コード 1、中身は変わらない） |
| `docs/issues/feat-041-intrinsic-loop-render-match/requirements.md` | NFR-004 を、修正後の仕様に置き換える（下記） |
| `docs/issues/feat-041-intrinsic-loop-render-match/design.md` | §4.2.1（CLI・擬似コード）、§4.2.7、§5、§8（テスト）、§9 手順 6 の注記（下記） |
| `README.md` | feat-041 の節の運用条件の行を、修正後の仕様に置き換える（完了処理で行う） |

#### 変更しないファイル

| ファイル | 理由 |
|---|---|
| `phase4/refine_extrinsics.py` | feat-041 requirements.md §5 の既存コード変更禁止。`refine_extrinsics.py` を単体で使う場合（feat-026）は本不具合の対象外。`estimate_intrinsics_loop.py` が新しく作った `--out-dir` の中で呼ぶので、一時ファイルのパスは必ず新しい |
| `phase4/evaluate_overlay.py` | 単体 CLI の出力は `--out-prefix` の 4 パスだけで、一時ファイルや中間ディレクトリを使わない |

#### 修正コード

修正前（`main`）:

```python
    all_paths = [out["loop_toml"], out["final_toml"], out["loop_report"],
                 out["refine_report"]] + [Path(p) for p in eval_output_paths(out["eval_prefix"]).values()]
    for pth in all_paths:
        if os.path.lexists(pth):
            print(f"エラー: 出力ファイルが既に存在します: {pth}", file=sys.stderr)
            return 1

    out_dir.mkdir(parents=True, exist_ok=True)
    tmp_ctx = None
    if args.tmp_dir:
        tmp_dir = Path(args.tmp_dir)
        tmp_dir.mkdir(parents=True, exist_ok=True)
    else:
        ...
```

修正後:

```python
    if (camera in ("", ".", "..") or Path(camera).name != camera
            or "/" in camera or "\\" in camera):
        print(f"エラー: カメラ名にはパス区切りを含まない名前を指定してください: {camera}", file=sys.stderr)
        return 1
    # bug-006: 書き込み先のディレクトリは存在しないパスとして新しく作る（既存ディレクトリ内の
    # リンクや入力を、一時ファイル {camera}_final.toml.tmp や中間ファイルで上書きしないため）
    try:
        out_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"エラー: 出力ディレクトリが既に存在します（存在しない新しいパスを指定してください）: {out_dir}",
              file=sys.stderr)
        return 1
    except OSError as e:
        print(f"エラー: 出力ディレクトリを作成できません: {out_dir}（{e}）", file=sys.stderr)
        return 1
    tmp_ctx = None
    if args.tmp_dir:
        tmp_dir = Path(args.tmp_dir)
        try:
            tmp_dir.mkdir(parents=True, exist_ok=False)
        except OSError as e:
            try:
                out_dir.rmdir()   # 直前に作った出力ディレクトリを消す（空でなければ失敗する）
            except OSError:
                print(f"エラー: 作成したディレクトリが残っています（手で削除してください）: {out_dir}",
                      file=sys.stderr)
            if isinstance(e, FileExistsError):
                print(f"エラー: 中間ファイル置き場が既に存在します（存在しない新しいパスを指定してください）: {tmp_dir}",
                      file=sys.stderr)
            else:
                print(f"エラー: 中間ファイル置き場を作成できません: {tmp_dir}（{e}）", file=sys.stderr)
            return 1
    else:
        ...（従来どおり）
```

（`os` の import が他で不要になれば削除してよい）

#### requirements.md の変更案（feat-041）

NFR-004 の「ただし `estimate_intrinsics_loop.py` は `--overwrite` を持たない。」から「…終了コード 1。」までの記述（出力ファイル一式の存在検査、運用条件、bug-006 の検査と `..` の禁止）を、次に置き換える:

> ただし `estimate_intrinsics_loop.py` は `--overwrite` を持たない。`--out-dir` と、指定した場合の `--tmp-dir` は、存在しないパスでなければならず、処理を始める前に新しく作る（既に存在すれば終了コード 1。リンク先の無いシンボリックリンクも存在するとみなす）。再実行するときは、新しいパスを指定するか、利用者が前回のディレクトリを手で削除する。今回の実行が途中で終わったときに前回の結果と混在しないようにし、途中で書き込む一時ファイル・中間ファイルが入力を上書きしないようにするため（investigation.md イテレーション2、bug-006）。

#### design.md の変更案（feat-041）

1. §4.2.1 の CLI: `--out-dir <出力ディレクトリ（存在しない新しいパス）>`、`[--tmp-dir <中間ファイル置き場（存在しない新しいパス）>]` とする
2. §4.2.1 の擬似コード: 「out = §1.3 の 8 ファイルのパス」以下の出力済み検査、`..` の検査、`--tmp-dir` の存在検査、「--out-dir を作る（exist_ok）」「tmp_dir = --tmp-dir（作る）」を、本書 §1.3 の 1〜5 に置き換える
3. §4.2.7: 「出力済み検査（…）を通過した後に終了する場合だけ」を「`--out-dir`・`--tmp-dir` の新規作成（§4.2.1）に成功した後に終了する場合だけ」に変える。「出力済み検査で拒否した場合」を「新規作成で拒否した場合」に変える
4. §5: 「`--tmp-dir` には存在しない新しいパスを指定する（既に存在すれば終了コード 1。bug-006）」はそのまま残す。「`--out-dir` も存在しない新しいパスを指定する」を加える
5. §8 テスト: 23〜25 を次に置き換え、26・27 を加える

   | # | 対象 | 内容 | 期待 |
   |---|---|---|---|
   | 23 | `--out-dir` の新規作成（bug-006） | テスト 19 と同じモックで、`--out-dir` に (a) 既存の空ディレクトリ、(b) リンク先の無いシンボリックリンク、(c) 未作成の親と `..` を含み既存ディレクトリを指すパス（`{tmp_path}/new2/../outx`。`outx` は既存で、中に `{camera}_final.toml.tmp`（内容 "old"）を置く）を指定して `main` を呼ぶ | どれも終了コード 1。`run_loop_iteration` は呼ばれない。(a) は空のまま。(b) のリンクはそのまま。(c) の `{camera}_final.toml.tmp` は内容 "old" のまま |
   | 24 | `--tmp-dir` の新規作成（bug-006） | テスト 19 と同じモックで、`--out-dir` は存在しないパス、`--tmp-dir` に (a) 既存の空ディレクトリ、(b) リンク先の無いシンボリックリンクを指定して `main` を呼ぶ。対照として (c) 存在しないパス | (a)(b) 終了コード 1、`run_loop_iteration` は呼ばれず、`--out-dir` は残っていない（作ってから消される）。(c) 従来どおり進む（テスト 19 と同じ終了コード 3） |
   | 25 | 親成分のシンボリックリンク（bug-006、codex-03 の配置） | `{tmp_path}/existing/sub/iter00_render.png`（内容 "old"）を置き、`{tmp_path}/link -> new/../existing` のシンボリックリンクを作る（`new` は未作成）。`--out-dir {tmp_path}/new --tmp-dir {tmp_path}/link/sub` で `main` を呼ぶ（テスト 19 と同じモック） | 終了コード 1。`iter00_render.png` は内容 "old" のまま。`run_loop_iteration` は呼ばれない。`{tmp_path}/new` は残っていない |
   | 26 | カメラ名の制限（bug-006、codex-04 指摘1） | テスト 19 と同じモックで、`--camera` に (a) `{tmp_path}/victim`（絶対パス。`{tmp_path}/victim_loop_report.txt` に内容 "old" を置く）、(b) `sub/cam`、(c) `..` を指定し、`--out-dir` は存在しないパスで `main` を呼ぶ | どれも終了コード 1。`--out-dir` は作られない。(a) の `victim_loop_report.txt` は内容 "old" のまま |
   | 27 | `--tmp-dir` 作成の途中失敗（bug-006、codex-04 指摘2） | テスト 19 と同じモックで、`--out-dir {tmp_path}/new --tmp-dir {tmp_path}/new/sub/..` で `main` を呼ぶ | 終了コード 1（例外で落ちない）。標準エラー出力に「中間ファイル置き場が既に存在します」と「作成したディレクトリが残っています」の両方が出る。`run_loop_iteration` は呼ばれない |

6. §9 手順 6 の注記「（bug-006 の修正後は、(1) の一時ファイル・`--tmp-dir` の存在はツール側でも検査される）」を「（bug-006 の修正後は、`--out-dir`・`--tmp-dir` が存在しないことをツール側でも検査する）」に変える

### 1.4 影響範囲

- **他の機能への影響**: `estimate_intrinsics_loop.py` の起動時の検査だけ。ループ、最終精緻化、評価の動作は変わらない
- **リグレッションリスク**:
  1. `--out-dir`・`--tmp-dir` に既存のディレクトリ（空でも）を指定して使う使い方ができなくなる。これまでの実データの手順（design.md §9 手順 3・5・6）は、毎回新しいパス（`feat041_out`、`_it1`、`_it2`、`feat041_tmp`、`_it1`、`_it2`）を使っているので影響しない
  2. 既存テスト 7c・20(b) は既存の出力ディレクトリを使い、終了コード 1 と中身が変わらないことを確かめている。修正後も同じ結果になる（拒否の理由が「出力ファイルが既に存在」から「出力ディレクトリが既に存在」に変わるだけ）。テスト 19・20(a)・21 はそのまま通る
  3. `refine_extrinsics.py` の一時ファイルの名前規則が将来変わっても、新しく作ったディレクトリの中なので影響しない
- **今回の修正に含めないこと**:
  - 検査と書き込みの間に、別のプロセスがリンクを作る競合（feat-041 の同時実行の制約と同じく対象外）
  - `refine_extrinsics.py` を単体で使う場合の一時ファイルの検査（feat-026 の動作。本不具合の対象外）

### 1.5 確認方法

- **自動テスト**: テスト 23〜27 を追加し、`uv run pytest -v` の全件実行結果を `tests/results/bug-006_test_result.txt` に保存する。既存テストがすべて通ること
- **手動テスト**: 実データの再実行は不要。修正は起動時の検査だけで、ループ以降の処理は変わらないため。ユーザーは、起動時の検査で拒否される様子を、次の 3 つのコマンドで確認する
  1. 存在する `--out-dir`（空のディレクトリ）を指定すると、終了コード 1 で拒否される
  2. 存在する `--tmp-dir` を指定すると、終了コード 1 で拒否され、`--out-dir` が残らない
  3. どちらもない場合は、従来どおり開始する（開始を確認したら Ctrl+C で止めてよい）

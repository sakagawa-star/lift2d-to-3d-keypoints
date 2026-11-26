# bug-007 調査・修正計画（investigation.md）

## イテレーション 0（初版、2026-11-26）

### 1.1 不具合の特定

- **対応する要求ID**: feat-041 requirements.md FR-006（収束後のパラメータで `refine_extrinsics.py` を呼び、外部パラメータを最終精緻化する）。利用者が指定したパスは、実行したカレントディレクトリ基準で解釈される前提（feat-042 requirements.md「相対パスは実行したカレントディレクトリが基準」、README の各ツールの節も同じ）
- **対応する設計セクション**: feat-041 design.md §4.2.6「最終精緻化は次のコマンドを subprocess で実行する（cwd はプロジェクトルート、環境変数は継承）」
- **現在の動作**: プロジェクトルート以外のディレクトリから相対パスで `estimate_intrinsics_loop.py` を実行すると、ループは収束するが、最終精緻化の `refine_extrinsics.py` が入力 TOML を見つけられず異常終了する（終了コード 1）
- **期待する動作**: 実行したディレクトリがどこでも、相対パスは実行したカレントディレクトリ基準で解釈され、プロジェクトルートで実行した場合と同じ処理になる
- **再現手順（2026-11-26 に実行して確認）**:
  1. 前処理済みの入力（feat-043 の手動テストで作った `phase4/data/10F/job_s25_*`）を用意する
  2. リポジトリのルートで次を実行する。`uv run --directory phase4` により、推定は `phase4/` をカレントディレクトリにして動き、パスはすべて `phase4/` からの相対パスになる

     ```bash
     TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --directory phase4 python estimate_intrinsics_loop.py \
       --init-toml data/10F/job_s25_init.toml --camera s25_20261124_152436 \
       --ply data/10F/point_cloud.ply --images-dir data/10F/job_s25_images \
       --points-3d ../phase0/data/kijunten_locations_10F.csv \
       --points-2d data/10F/job_s25_points_2d.csv --out-dir data/10F/bug007_repro_out
     ```

     - `--out-dir` は実行のたびに**存在しない新しい相対パス**にする（推定は既存の `--out-dir` を処理開始前に拒否して終了コード 1 を返し、不具合と同じ終了コードになるため。2026-11-26 の実行で `phase4/data/10F/bug007_repro_out/` は作成済みなので、再度再現するときは `data/10F/bug007_repro2_out` などに変える。codex-06）
     - **再現成立の条件**（終了コードだけで判定しない）: ループレポート（`{out_dir}/{camera}_loop_report.txt`）に反復の収束と「終了状態: refine_extrinsics.py が異常終了（終了コード 1）」が記録され、stderr（または子プロセスを同条件で直接実行したときの stderr）に「入力TOMLを読み込めません」が出ること
  3. 結果: 反復 i=0〜4 は正常に進み i=4 で収束（f=1279.99、プロジェクトルートで実行した場合と同じ値）。最終精緻化で終了コード 1。ループレポートの末尾は「refine_extrinsics.py 所要時間: 0.2s」「終了状態: refine_extrinsics.py が異常終了（終了コード 1）」。`--out-dir` には `_loop.toml` と `_loop_report.txt` だけが残る
- **エラーメッセージ**: 子プロセスと同じ条件（プロジェクトルートを cwd にして、同じ相対パスを渡す）で `refine_extrinsics.py` を直接実行して取得した stderr:

  ```
  エラー: 入力TOMLを読み込めません: data/10F/bug007_repro_out/s25_20261124_152436_loop.toml（[Errno 2] No such file or directory: 'data/10F/bug007_repro_out/s25_20261124_152436_loop.toml'）
  ```

  （トレースバックは出ない。`refine_extrinsics.py` が例外を捕まえてこのメッセージで終了コード 1 を返す）

### 1.2 原因分析

- **原因箇所**: `phase4/estimate_intrinsics_loop.py` の `_run()`、最終精緻化の呼び出し

  ```python
  r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
  ```

- **原因の説明**: `cmd` には、利用者が指定した相対パス（`args.ply`、`args.images_dir`、`args.points_3d`、`args.points_2d`）と、相対の `--out-dir` / `--tmp-dir` から作ったパス（`out["loop_toml"]`、`out["final_toml"]`、`out["refine_report"]`、`tmp_dir / "refine"`）をそのまま入れている。これらは親プロセスのカレントディレクトリ基準の相対パスだが、子プロセスは `cwd=ROOT`（プロジェクトルート）で起動するため、子プロセスではプロジェクトルート基準で解釈される。親のカレントディレクトリがプロジェクトルートと異なると、別の場所を指す
- **根本原因 or 表面的原因**: 根本原因。子プロセスのカレントディレクトリを親と変えていることが、相対パスの基準のずれの唯一の原因
  - `refine_extrinsics.py` 自身はカレントディレクトリに依存しない: 自分のファイル位置から `ROOT = Path(__file__).resolve().parents[1]` を求め、`matcher_lab` の呼び出し（`uv run --project {ROOT}/matcher_lab ...`）も絶対パスで行う。スクリプトとして起動されるため、`render_keypoints` などの import は自分のディレクトリ（`phase4/`）から解決される。したがって `cwd=ROOT` は不要
  - 推定の他の subprocess（MASt3R の `mast3r_cli.py`）は `cwd` を指定しておらず、親のカレントディレクトリを継承する。再現手順でも i=0 の MASt3R は `phase4/` から正常に動いた
- **付随する危険（同じ原因）**: 子プロセスには `--overwrite` を常に付けている。入力がプロジェクトルート基準でもたまたま存在する場合は、`--out-toml`・`--out-report`・`--tmp-dir` がプロジェクトルート基準の別の場所を指したまま、そこに上書きで書き込む（bug-006 で守った「新しく作った `--out-dir` の中だけに書く」が崩れる）。本修正でこれも解消する

### 1.3 修正内容

- **変更対象ファイル**（4・5 の設計書は本計画の保存時に反映済み。1〜3 のコードとテストは実装で変更する。利用者向けの「修正済み」の案内は、実装と手動テストの後に完了処理で書く（1.5 の「完了処理で更新する文書」。codex-02））:
  1. `phase4/estimate_intrinsics_loop.py`: 最終精緻化の `subprocess.run` から `cwd=str(ROOT)` を外す（子プロセスは親のカレントディレクトリを継承する）。`cmd` の中身は変えない
  2. `tests/test_feat042_config_yaml.py`: テスト 16（`test_16_subprocess_command_unchanged`）の最後の `assert kwargs["cwd"] == str(ROOT)` を `assert "cwd" not in kwargs` に変える（コマンド本体の比較 `assert cmd == expected` は変えない）
  3. `tests/test_feat041_intrinsics_loop.py`: テスト 28 を追加する（後述 1.5）
  4. `docs/issues/feat-041-intrinsic-loop-render-match/design.md` §4.2.6: 「最終精緻化は次のコマンドを subprocess で実行する（cwd はプロジェクトルート、環境変数は継承）:」を「最終精緻化は次のコマンドを subprocess で実行する（cwd は指定せず呼び出し元のカレントディレクトリを継承、環境変数も継承。相対パスは呼び出し元と同じ基準で解釈される。bug-007）:」に変える
  5. `docs/issues/feat-042-config-yaml-refine-tools/design.md` §6 テスト 16 の合格条件: 「`cwd` が `str(ROOT)`。」を「`cwd` キーワードを渡さない（親のカレントディレクトリを継承する。bug-007 で変更）。」に変える（コマンドの引数列の完全一致の条件は変えない。codex-01 指摘1）
- **修正コード**:

  修正前:

  ```python
  r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
  ```

  修正後:

  ```python
  r = subprocess.run(cmd, capture_output=True, text=True)
  ```

- **変更しないファイル**:
  - `phase4/refine_extrinsics.py`: カレントディレクトリに依存しない（1.2）。受け取った相対パスを自分のカレントディレクトリ基準で解釈するのは正しい動作
  - `phase4/prepare_phone_photo.py`: subprocess を使わない
  - `ROOT` の import（`estimate_intrinsics_loop.py`）: `cmd` の先頭のスクリプトパス `ROOT / "phase4" / "refine_extrinsics.py"` で引き続き使うため残す
  - MASt3R 呼び出し（`call_mast3r`）: もともと `cwd` を指定していない
- **修正が設計書に沿っているか**: feat-041 design.md §4.2.6 の「cwd はプロジェクトルート」と、feat-042 design.md §6 テスト 16 の合格条件「`cwd` が `str(ROOT)`」と矛盾するため、上記 4・5 のとおり設計書も変える（`cwd` を記述している文書をリポジトリ全体で検索し、該当はこの 2 か所と、feat-042 requirements.md §5.1 の bug-007 の制約の記述だけであることを確認済み。後者は完了処理で更新する）。他の箇所（README、feat-042 の文書）の「相対パスは実行したカレントディレクトリが基準」とは、修正後に一致する
- **採らなかった案**: 子プロセスに渡すパスを `os.path.abspath` で絶対パスにして `cwd=ROOT` を残す案（feat-042 codex-01 の修正案）。変更箇所が 8 つのパスに広がるうえ、`abspath` は `..` を文字列として畳むため、シンボリックリンクを含むパスでは OS の解釈（親プロセスが読んだ場所）とずれうる。`cwd` を外せば、親が読んだのと同じ文字列を同じ基準で子が解釈するので、このずれが起きない

### 1.4 影響範囲

- **他の機能への影響**: `estimate_intrinsics_loop.py` の最終精緻化だけ。プロジェクトルートで実行した場合は、親のカレントディレクトリ＝プロジェクトルートなので、修正前と同じ動作になる。絶対パスで指定した場合も同じ
- **リグレッションリスク**: 子プロセスがカレントディレクトリ基準の何かに依存していた場合に壊れるが、1.2 のとおり `refine_extrinsics.py` はカレントディレクトリに依存しない。テスト 16 の変更以外に既存テストへの影響はない（`cwd` を参照するテストは他に `tests/test_feat035_multiview.py` の 1 か所があるが、これはテスト自身が別のスクリプトを起動する箇所で本修正と無関係）

### 1.5 確認方法

- **自動テスト**（`uv run pytest -v` 全件、結果は `tests/results/bug-007_test_result.txt`）:
  - テスト 16 の変更（`cwd` を渡していないこと）
  - テスト 28（新規、`tests/test_feat041_intrinsics_loop.py`）: プロジェクトルートと異なるディレクトリ（`tmp_path / "work"`）に `monkeypatch.chdir` し、入力・`--out-dir`・`--tmp-dir` をすべてそこからの相対パスで指定して `main` を呼ぶ。ループ部分と評価は既存テストと同じ方法で差し替える（GPU・実データ不要）。具体的には、同じファイルの `_bug006_setup(monkeypatch, tmp_path)` を呼んだ後、`eil.run_loop_iteration` を、毎回同じパラメータ `Params(1280.0, 960.0, 540.0, 0.0, 0.0, np.zeros(3), np.zeros(3))` を返す関数で差し替え直して i=1 で収束させる（feat-042 テスト 16 の `_loop_setup(..., constant_params=True)` と同じ挙動。戻り値の `diag` は `_bug006_setup` のものと同じ）。`eil.evaluate_and_write` はテスト 16 と同じ固定の統計を返す関数で差し替える。引数は `--init-toml x.toml --camera camA --ply x.ply --images-dir in_images --points-3d a.csv --points-2d b.csv --out-dir out --tmp-dir wtmp`（すべて相対パス）とする。`subprocess.run` を差し替えた関数で次を確認する:
    1. `cwd` キーワードが渡されていない
    2. `cmd` の `--toml` の値（相対パス）を、その時点のカレントディレクトリ基準で読めて、`tmp_path / "work" / {out_dir} / {camera}_loop.toml` と同じファイルを指す
    3. `--out-toml` と `--out-report` の値が、カレントディレクトリ基準で `tmp_path / "work" / "out"` の配下（それぞれ `camA_final.toml`、`camA_refine_report.txt`）を指し、`--tmp-dir` の値が、カレントディレクトリ基準で `tmp_path / "work" / "wtmp" / "refine"` を指す（codex-04 指摘1）
    - 差し替えた関数は、テスト 16 と同じく `--out-report` に受理行を、`--out-toml` に入力 TOML の写しを書いて終了コード 0 を返す。`main` の戻り値が 0 であることも確認する
- **手動テスト**（ユーザー）: **リポジトリのルートで**、再現手順 2 のコマンドを `--out-dir` だけ新しいパス `data/10F/bug007_fix_out` に変えて実行する（コマンドの `uv run --directory phase4` が、推定の実行時のカレントディレクトリを `phase4/` にする。シェルで `phase4/` に移動してから実行すると `phase4/phase4` を指して uv が終了コード 2 になる。codex-04 指摘2）。期待: 終了コード 0、評価は中央値 0.671px・「合格」（プロジェクトルートで実行した場合と同じ値）、出力 8 ファイルがすべて `phase4/data/10F/bug007_fix_out/` に揃い、ループレポートの末尾が「終了状態: 成功」。`--out-dir` は存在しない新しいパスでなければならない（`bug007_fix_out` が既にあれば `bug007_fix2_out` などに変える。既存だと開始前に終了コード 1 になり、修正の確認にならない。codex-06）
- **完了処理で更新する文書**（手動テスト合格後に行う。実装前に「修正済み」と案内しない。codex-02）: README の `estimate_intrinsics_loop.py ほか` の節の「相対パスの注意（bug-007）」の箇条を削除する（制約がなくなるため）。`docs/issues/feat-042-config-yaml-refine-tools/requirements.md` §5.1 の bug-007 の制約の箇条（「`estimate_intrinsics_loop.py` は最終精緻化の `refine_extrinsics.py` を `cwd=ROOT`（プロジェクトルート）で起動し、…」）の末尾に「（注記: bug-007 で修正済み。子プロセスは親のカレントディレクトリを継承するため、どのディレクトリからでも相対パスで実行できる。{完了日}）」を足す。BACKLOG・CHANGELOG・本案件 README のステータス
- 再現で作った `phase4/data/10F/bug007_repro_out/`（git 管理外）は、手動テスト合格後に削除してよい

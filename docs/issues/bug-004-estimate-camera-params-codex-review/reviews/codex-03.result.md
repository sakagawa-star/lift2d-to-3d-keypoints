**指摘**

**高**
- H-2 の `--output` 親ディレクトリ検証がまだ不十分です。  
  該当: [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:105), [同:106](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:106), [同:179](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:179)  
  `_run_extrinsic_estimation` 側は `output_path` 自体が既存ディレクトリかしか見ない計画です。`run_estimation(..., output_path="存在しないdir/out.toml")` の直呼びでは、親ディレクトリ未存在のまま `_write_toml_output` に進み `FileNotFoundError` でクラッシュします。また `main()` 側の既存親チェックは `exists()` のまま残す計画なので、親パスが既存ファイルの場合も `NotADirectoryError` が残ります。  
  **修正提案**: `_run_extrinsic_estimation` で `Path(output_path).parent.is_dir()` を検証し、不合格なら終了コード1にする。`main()` 側の親チェックも `exists()` ではなく `is_dir()` にするか、親チェックを `_run_extrinsic_estimation` に集約する。テストに「親ディレクトリ未存在」と「親がファイル」を `main()` / `run_estimation()` 両方で追加してください。

**中**
- M-2 の正式化は要求仕様書だけで、設計書側の変更案がまだありません。  
  該当: [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:140), [BUGFIX_STANDARD.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/BUGFIX_STANDARD.md:59), [feat-001 design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-001-estimate-extrinsic/design.md:32)  
  `BUGFIX_STANDARD.md` は設計書変更が必要な場合に設計書の変更案も提示することを求めています。`fisheye=true` を未対応エラーにするなら、`load_intrinsic_toml` の入力マッピング/エラー処理を定義している design 側にも追記が必要です。  
  **修正提案**: `feat-001-estimate-extrinsic/design.md` の `load_intrinsic_toml` 節に、`fisheye=true` は `ValueError`、`false`/キーなしは従来どおり、を変更注記として追加する計画を入れてください。feat-003/feat-008 の design を現行実装設計として参照する運用なら、そこにも追記または bug-004 で上書きされる旨を明記してください。

**低**
- なし。

前回指摘の「関数直呼び経路のガード」は、同一パス・config/toml ディレクトリについては反映済みです。上記の `--output` 親ディレクトリだけが残っています。
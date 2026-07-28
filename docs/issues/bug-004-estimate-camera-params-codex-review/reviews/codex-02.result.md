**指摘**

**高**
- H-1/H-2 のガードが `main()` 前提で、`run_estimation()` / `_run_extrinsic_estimation()` 直呼び経路が残ります。  
  該当: [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:78), [同:97](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:97), [同:163](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:163)  
  `run_estimation(config_path, ..., intrinsic_toml=toml, output_path=toml)` は現在の計画だと同一パス防止を通らず、入力TOML破壊が残ります。config / intrinsic のディレクトリ指定も関数直呼びではまだ `open()` クラッシュになります。  
  **修正提案**: `main()` だけでなく、`run_estimation()` または `_run_extrinsic_estimation()` の入口でも `config_path.is_file()`、`toml_path.is_file()`、`output_path.is_dir()`、`output_path.resolve() != toml_path.resolve()` を検証する。テストも `main()` 経由だけでなく `run_estimation(..., intrinsic_toml=..., output_path=...)` 直呼びを追加してください。

**中**
- M-2 は「仕様未定義」と明記しているのに、`investigation.md` 内だけで仕様定義して bug 修正に含めています。  
  該当: [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-004-estimate-camera-params-codex-review/investigation.md:17), [BUGFIX_STANDARD.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/BUGFIX_STANDARD.md:15)  
  `BUGFIX_STANDARD.md` は、要求仕様に該当記述がない場合は feat として要求仕様書・機能設計書を作る、と定めています。`fisheye=true` の扱いを変えるなら、この計画だけでは基準未達です。  
  **修正提案**: M-2 を別 feat に切り出すか、少なくとも既存の要求/設計ドキュメントに「fisheye=true は未対応エラー」を正式に追加する計画を入れてから実装してください。

**低**
- なし。

前回の高3件・中2件のうち、H-3 と M-1 は計画どおり実装すれば概ね解消されます。H-1/H-2 は上記の関数直呼び経路を塞がないと未解消です。
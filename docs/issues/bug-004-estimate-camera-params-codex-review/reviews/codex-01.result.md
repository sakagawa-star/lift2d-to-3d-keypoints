**指摘**

**高**
- `main` / `_write_toml_output` 行: [1246](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1246), [404](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:404)  
  `--output` に `--intrinsic-toml` と同じパスを指定でき、内部パラメータ入力TOMLを `open(..., 'w')` で破壊的に上書きできます。成功カメラ分だけを書き直すため、未対象/失敗/metadata セクションも失われます。  
  **修正提案**: `Path(args.output).resolve() == Path(args.intrinsic_toml).resolve()` を明示的にエラーにする。併せて一時ファイルへ書いてから rename する原子的書き込みにすると安全です。

- `main` / `_write_toml_output` 行: [1214](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1214), [1237](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1237), [404](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:404)  
  `config` と `--intrinsic-toml` は `exists()` だけなので、ディレクトリを渡すと後段の `open()` でクラッシュします。`--output` も出力先自体がディレクトリでも通り、書き込み時にクラッシュします。  
  **修正提案**: 入力は `is_file()` で検証し、`--output` は `Path(args.output).is_dir()` をエラーにする。`run_estimation()` 直呼び経路も同等に守るとよいです。

- `load_intrinsic_toml` 行: [271](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:271)  
  TOML の `matrix` / `distortions` / `size` は存在確認だけで、形状・長さを検証していません。例: `distortions = [0, 0]` は `cv2.solvePnPRansac` 側で `cv2.error`、`matrix` が 3x3 でない場合は `K[0,0]` 参照や OpenCV 呼び出しでクラッシュします。  
  **修正提案**: `K.shape == (3, 3)`、`dist.size in {0,4,5,8,12,14}`、`size` が2要素かつ正の有限値であることを検証し、不正なら `ValueError` で既存のエラー終了経路に乗せる。

**中**
- `main` 行: [1218](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1218)  
  `--intrinsic-toml --wide --zero-tangent` の場合、本来 K既知モードでは `--wide` / `--zero-tangent` は無視される設計なのに、先に排他チェックで終了します。実際に該当コマンドは `エラー: --zero-tangent と --wide は併用できません` で終了しました。  
  **修正提案**: `--intrinsic-toml` 指定時の ignored 警告処理を排他チェックより先にする、または排他チェックを `if not args.intrinsic_toml and args.zero_tangent and args.wide:` に限定する。

- `load_intrinsic_toml` / `estimate_extrinsic` 行: [263](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:263), [295](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:295)  
  TOML の `fisheye = true` を無視して通常の OpenCV 歪みモデルとして処理します。魚眼係数を通常モデルに渡すと、クラッシュしないまま誤った外部パラメータを出力する可能性があります。  
  **修正提案**: `fisheye=true` は明示的に未対応エラーにするか、魚眼用に `cv2.fisheye.undistortPoints` 等で正規化してから PnP する別経路を実装する。

**低**
- なし。既知のスコープ外項目は除外しました。

確認として、関連テストは `UV_CACHE_DIR=/tmp/uv-cache uv run pytest ...` で 73件すべて成功しています。
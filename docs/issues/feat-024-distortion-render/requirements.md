# feat-024 要求仕様書: render_keypoints.py 歪みモデル対応レンダリング（GT比較用）

## 1.1 プロジェクト概要

- **何を作るのか**: `phase4/render_keypoints.py` に、(1) キーポイント描画をスキップして
  3DGS 背景のみの静止画1枚を出力する `--no-keypoints` オプションと、(2) キャリブTOMLの
  歪み係数を使って歪みモデルでレンダリングする `--distort` オプションを追加する。
- **なぜ作るのか**: `estimate_camera_params.py` の推定結果（K・歪み係数・R, t）を
  Ground Truth（実写画像・基準点）と視覚比較して検証するため。現状のピンホール
  レンダリングでは、実レンズの歪みで写っている画像端の基準点（例: E0085-01 の
  基準_018/051）が画角外になり、推定の妥当性を画像で確認できない。
- **誰が使うのか**: 本プロジェクトでカメラキャリブレーションの検証を行う開発者・研究者。
- **どこで使うのか**: Linux + CUDA GPU（gtune2: RTX 5060 Ti）。`phase4/` の uv 環境で
  CLI として実行（`TORCH_CUDA_ARCH_LIST="9.0+PTX"` 必須）。

## 1.2 用語定義

本ドキュメントの用語は機能設計書・コード内でも同じ意味で使う。

| 用語 | 定義 |
|---|---|
| 静止画モード | `--no-keypoints` を指定し `c3d_path` を省略した実行形態。3DGS 背景のみの PNG を1枚出力する |
| 動画モード | 従来どおり `c3d_path` を渡す実行形態（連番PNG/MP4、キーポイント描画あり）。本案件では動作を一切変更しない |
| 歪みレンダリング | gsplat 1.5.3 の 3DGUT 経路（`with_ut=True, with_eval3d=True, packed=False` + `radial_coeffs`/`tangential_coeffs`）による OpenCV 歪みモデル込みのレンダリング（方式A。スパイクで画質を検証済み） |
| ピンホールレンダリング | 現行の古典経路（`with_ut=False, packed=True`、歪みなし） |
| TOML歪み係数 | Calib_scene.toml / Config_scene.toml の `distortions` 配列。OpenCV 並びで長さ4 `[k1,k2,p1,p2]`、5 `[k1,k2,p1,p2,k3]`、8 `[k1,k2,p1,p2,k3,k4,k5,k6]` のいずれか |

## 1.3 機能要求一覧

### FR-001: --no-keypoints オプション（静止画モード）

- **機能名**: キーポイント描画スキップ（静止画モード）
- **概要**: `--no-keypoints` フラグを追加する。指定時は C3D 読み込み・キーポイント抽出・
  投影・オクルージョン計算・重ね描きをすべてスキップし、3DGS 背景レンダリング結果のみを
  PNG 1枚として保存する。
- **入力**: `--no-keypoints`（フラグ）。位置引数は `ply_path`, `toml_path` の2つ
  （`c3d_path` は省略。FR-002）
- **出力**: `<output-dir>/still_<カメラ名>.png` の1枚（8bit BGR、画像サイズはTOMLの size）
- **受け入れ基準**: 静止画モードの実行で PNG が1枚だけ生成され、その画素内容が
  同条件の動画モードの背景（キーポイント非描画部分）と一致すること。

### FR-002: c3d_path の省略（案b）

- **機能名**: C3Dパス省略
- **概要**: 位置引数 `c3d_path` を省略可能にする。省略の可否は `--no-keypoints` と
  **双方向に連動**する: `c3d_path` 省略は `--no-keypoints` 指定時のみ可。
  `--no-keypoints` 指定時は `c3d_path` を渡してはならない。
- **入力**: コマンドライン引数の組み合わせ
- **出力**: 不整合時は `parser.error()`（終了コード2）。メッセージは
  - `c3d_path` 省略 + `--no-keypoints` なし → 「c3d_path を省略する場合は --no-keypoints を指定してください」
  - `c3d_path` 指定 + `--no-keypoints` あり → 「--no-keypoints 指定時は c3d_path を渡せません（静止画モードは C3D 不要）」
- **受け入れ基準**: 上記2パターンが重い処理（C3D/PLY/torch ロード）前に終了コード2で
  拒否されること。従来の3引数指定（動画モード）は無変更で動作すること。

### FR-003: 静止画モードで無意味なオプションの拒否

- **機能名**: オプション組み合わせバリデーション
- **概要**: 静止画モードでは以下のオプションは機能しないため、指定されたら
  `parser.error()`（終了コード2）で拒否する（警告して無視はしない）:
  `--mp4`, `--mp4-fps`, `--no-png`, `--start-frame`, `--end-frame`,
  `--no-occlusion`, `--occlusion-margin`
- **入力**: 静止画モード + 上記オプション
- **出力**: エラーメッセージ「--no-keypoints 指定時は <オプション名> は使用できません」+ 終了コード2
- **受け入れ基準**: 上記7オプションそれぞれで拒否されること。動画モードでの各オプションの
  動作は無変更であること。

### FR-004: --distort オプション（歪みレンダリング）

- **機能名**: 歪みモデルレンダリング
- **概要**: `--distort` フラグを追加する。指定時、TOML歪み係数を gsplat の
  `radial_coeffs`/`tangential_coeffs` に詰め替え、3DGUT 経路でレンダリングする。
  本案件では `--distort` は **`--no-keypoints`（静止画モード）専用**とし、動画モード
  との併用は `parser.error()` で拒否する（キーポイント投影・オクルージョンとの歪み整合は
  将来の別案件）。
- **入力**: `--distort --no-keypoints`（+ TOML の `distortions`）
- **出力**: 歪みモデル込みの静止画 PNG 1枚
- **受け入れ基準**:
  - `--distort` なしの静止画モードはピンホールレンダリング（現行 `render_image` と同一条件）であること
  - `--distort` ありで、歪み係数に応じた変形が周辺部に現れること（ステップ6で E0085 実データにより確認）
  - `--distort` を `--no-keypoints` なしで指定すると終了コード2で拒否されること

### FR-005: TOML歪み係数の詰め替え

- **機能名**: 係数変換
- **概要**: TOML歪み係数（長さ4/5/8）を gsplat の引数形式に変換する:
  - 長さ4 `[k1,k2,p1,p2]` → radial `[k1,k2,0,0,0,0]`, tangential `[p1,p2]`
  - 長さ5 `[k1,k2,p1,p2,k3]` → radial `[k1,k2,k3,0,0,0]`, tangential `[p1,p2]`
  - 長さ8 `[k1,k2,p1,p2,k3,k4,k5,k6]` → radial `[k1,k2,k3,k4,k5,k6]`, tangential `[p1,p2]`
  - 上記以外の長さ → ValueError（メッセージに実際の長さを含める）
- **入力**: TOML の `distortions` 配列
- **出力**: gsplat 用 radial（6要素）/ tangential（2要素）
- **受け入れ基準**: 3パターンの変換と不正長のエラーが単体テストで確認できること。

### FR-006: 既存動作の完全維持

- **機能名**: 後方互換
- **概要**: `--no-keypoints` / `--distort` を指定しない動画モードの動作・出力
  （レンダリング結果・進捗表示・エラー処理）を変更前と完全に同一とする。
- **受け入れ基準**: 既存テスト（feat-015/016/017/021/022 関連）が無変更で全件パスすること。

## 1.4 非機能要求

- **パフォーマンス**: 静止画モードは1枚のみのため既存の背景レンダリングと同等
  （スパイク実績: JITビルド済み環境で1分未満）。UT経路の速度低下は静止画1枚の用途では
  許容とし、数値目標は設けない。
- **対応環境**: Linux、Python 3.10、phase4 uv 環境、CUDA GPU 必須、
  `TORCH_CUDA_ARCH_LIST="9.0+PTX"` 必須。UT経路の初回実行で CUDA JIT ビルドが発生し得る
  （最大15分程度を許容）。
- **信頼性**: 入力ファイルは読み取りのみ。出力の扱いは次のとおり:
  過去の実行で出力された連番 `frame_*.png` は削除・上書きしない（feat-022 の非破壊方針を維持）。
  静止画モードの `still_<カメラ名>.png` は**再実行時に上書きする**（GT比較で条件を変えて
  撮り直す用途のため。上書きは本ファイル1つに限定される）。
- **セキュリティ**: 該当なし。

## 1.5 制約条件

- 使用ライブラリは既存のもの（gsplat 1.5.3, torch, OpenCV, NumPy, tomli）に限定し、
  新規ライブラリを追加しない。gsplat のバージョン変更も行わない。
- gsplat 1.5.3 の制約: `with_ut=True` / `with_eval3d=True` 使用時は `packed=False` 必須
  （`rendering.py:353` の assert。スパイクで動作確認済み）。
- 変更対象ファイルは `phase4/render_keypoints.py` と `tests/`、ドキュメント
  （CLAUDE.md, BACKLOG.md, CHANGELOG.md）に限定する。`phase4/render.py` は変更しない。
- スパイクで検証済みの条件（`with_eval3d=True`, `near_plane` はCLI値, `packed=False`）を
  そのまま使う。

## 1.6 優先順位

| 要求ID | MoSCoW |
|---|---|
| FR-001 | Must |
| FR-002 | Must |
| FR-003 | Must |
| FR-004 | Must |
| FR-005 | Must |
| FR-006 | Must |

MVP は FR-001〜FR-006 のすべて（本案件は分割せず一括で完了させる）。

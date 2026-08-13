レビュー結果です。瑣末な表記は除外し、実装結果を壊し得る点だけ挙げます。

**高**
1. FR-002 の検証要件を、設計どおりでは満たせません。  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/requirements.md:43) は `frame_ids (F,) 整数配列`、`joint_names (J,) 文字列配列` を要求していますが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/design.md:90) は既存 `load_npz()` を「検証込み」として再利用するだけです。実コードでは [npz_to_c3d.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/npz_to_c3d.py:65) が `frame_ids` を `reshape(-1).astype(np.int64)` するため、形状 `(F,1)` や float 値の切り捨てを許容します。また [npz_to_c3d.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/npz_to_c3d.py:76) は `joint_names` を `str()` 化するため、文字列配列でない入力も通ります。  
修正提案: `npz_to_c3d.load_npz()` 自体を FR-002 の正本に合わせて強化し、`frame_ids.ndim == 1`、`shape == (F,)`、整数 dtype、`joint_names.ndim == 1`、`shape == (J,)`、文字列 dtype を明示検証してください。既存関数を変えない方針なら、requirements の「既存 `load_npz` を再利用」に例外を設け、共通 validator を新設する必要があります。

2. 壊れた NPZ / 非 NPZ の例外設計が甘く、要求どおりの終了にならない可能性があります。  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/requirements.md:44) は検証失敗時に日本語 stderr + 終了コード 1 を要求しています。一方 [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/design.md:150) は「壊れた NPZ は `np.load` が `ValueError`」と断定し、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/design.md:57) では `ValueError, FileNotFoundError` しか捕捉しません。`zipfile.BadZipFile`、`EOFError`、pickle 系例外などは漏れてトレースバック経路になり得ます。  
修正提案: `load_npz()` または `load_npz_all_frames()` 内で NPZ 読み込み例外を捕捉し、`ValueError("NPZファイルを読み込めません: ...")` に正規化してください。あわせて `allow_pickle=False` にできるなら変更し、object 配列が必要なら脅威モデルと理由を文書化してください。

**中**
1. FR-002 の受け入れ基準に対してテスト設計が不足しています。  
[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/requirements.md:45) は各異常系ごとに終了コード 1 を確認するとしていますが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/design.md:210) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-032-render-keypoints-npz-input/design.md:211) では F=0 は loader の `ValueError` のみ、main 経由の exit code は必須キー欠落だけです。形状不正、長さ不一致、非連番、F=0 の main 経由確認が抜けています。  
修正提案: `main()` に渡す異常 NPZ を parameterize し、必須キー欠落、`x3d_world` 形状不正、`frame_ids` 長さ不一致、`joint_names` 長さ不一致、非連番、F=0、壊れた NPZ のすべてで `return 1` と stderr を検証してください。

**低**
なし。
---

## Claude Code の対応方針（2026-08-13）

- **高1（検証要件の不一致）**: 既存 `load_npz` の変更は feat-018 への回帰リスクがあるため、要求仕様（FR-002）側を既存の正規化仕様（reshape・int64化・str化）に合わせて明文化した。design.md に「入力の正規化と検証（feat-018 準拠）」節と ADR D-7 を追加
- **高2（読み込み例外の漏れ）**: `load_npz_all_frames` で `ValueError`/`FileNotFoundError` 以外のすべての読み込み例外を `ValueError("NPZファイルを読み込めません: ...")` に正規化する設計に変更（design.md 4.2/4.4）。`allow_pickle=True` は入力が自パイプライン生成のローカルファイルに限られる（信頼境界を越えない）ため踏襲し、理由を D-7 に文書化
- **中1（異常系テスト不足）**: テスト #6 を parametrize 化し、必須キー欠落 / 形状不正 / 長さ不一致（frame_ids・joint_names）/ 非連番 / F=0 / 壊れた NPZ の全ケースで main 戻り値 1 + stderr を検証するよう design.md 6章と FR-002 受け入れ基準を更新

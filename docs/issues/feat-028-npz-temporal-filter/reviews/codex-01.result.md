**指摘**

- **高**: `joint_names` の扱いが設計内で矛盾しています。  
  [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:54) では `load_npz_keypoints()` が `list[str]` を返す設計ですが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:140) では `joint_names` を入力配列オブジェクトのまま書く必要があるとしています。これは [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/requirements.md:90) の「無加工コピー」と衝突します。  
  **修正提案**: `load_npz_keypoints()` は `joint_names_arr: np.ndarray` を返す契約に統一し、表示用の `list[str]` は呼び出し側で派生させる、と明記してください。

- **高**: `_verify_npz()` が入力NPZとの非有限マスク一致を検証できない設計になっています。  
  要求では出力の非有限マスクが入力と一致することが必須ですが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:271) では `_verify_npz()` に「書き出した `x3d_out`」を渡すと書かれており、これでは出力同士の自己比較になります。  
  **修正提案**: `x3d_original` または `invalid_mask_in = ~np.isfinite(x3d_original)` を `write_npz()` / `_verify_npz()` に渡し、読み戻した `x3d_world` と入力由来マスクを直接比較してください。テストにも NPZ roundtrip で非有限マスク一致を追加すべきです。

- **中**: NaN ギャップの扱いが README / 背景資料と要求・設計で食い違っています。  
  README と hearing-notes は「NaNで区切られた有効区間ごとに平滑化」と読めますが、要求・設計は `--max-gap` 以下の NaN ギャップを補間して同一セグメント内でフィルタします。該当: [README.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/README.md:16), [hearing-notes.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-027-fps-video-from-npz/hearing-notes.md:44), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/requirements.md:68)。  
  **修正提案**: feat-020 踏襲が正なら README / hearing-notes 側を「`max_gap` 超のギャップで分割、`max_gap` 以下は補間して計算」に更新してください。任意の NaN で必ず分割が正なら、要求・設計から補間方針を外す必要があります。

- **中**: `frame_ids` の dtype / 正規化方針が未確定です。  
  正本の `npz_to_c3d.load_npz()` は `frame_ids` を `int64` に変換してから連番判定していますが、feat-028 設計は dtype 変換しないまま `np.diff(frame_ids) == 1` としています。該当: [npz_to_c3d.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/npz_to_c3d.py:65), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:70)。文字列/object 配列などで日本語エラー終了ではなく例外漏れになる余地があります。  
  **修正提案**: 「整数 dtype の1次元配列のみ許可」または「`int64` へ損失なく変換できるもののみ許可」のどちらかに固定し、不正 dtype は `ValueError` にしてください。

低重要度の表記ゆれは報告対象から外しました。
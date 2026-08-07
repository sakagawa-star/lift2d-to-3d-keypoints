**指摘**

- **高**: なし。

- **中**: `load_npz_keypoints()` の型シグネチャだけ古い契約が残っています。  
  [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:54) がまだ `list[str]` を返す型になっていますが、同じ節の出力説明と 1.7 は `joint_names_arr: np.ndarray` に更新済みです。実装者が冒頭の関数定義だけを正と見ると、前回の `joint_names` 無加工コピー問題が再発します。  
  **修正提案**: line 54 を `tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray], np.dtype]` に直してください。

- **中**: `frame_ids` / `joint_names` の形状検証が、規約の `(F,)` / `(J,)` とまだズレています。  
  要求では規約NPZを `frame_ids (F,)`, `joint_names (J,)` と定義していますが、設計は [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:74) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-028-npz-temporal-filter/design.md:78) で「1次元化した長さ」だけを見ます。さらに配列は入力のまま書き戻すため、例えば `(F,1)` が通ると、出力も規約外形状のままになります。  
  **修正提案**: 無加工コピー方針を維持するなら、`frame_ids.ndim == 1 and frame_ids.shape == (F,)`、`joint_names_arr.ndim == 1 and joint_names_arr.shape == (J,)` を必須検証にしてください。

**前回指摘の確認**

- (1) `joint_names` の配列返却方針は本文・1.7では解消済み。ただし上記の型シグネチャ残りあり。
- (2) `_verify_npz` の自己比較問題は解消済み。
- (3) NaN 分割表現は README / hearing-notes ともに max-gap 補間方式へ統一済み。
- (4) `frame_ids` 非整数型の扱いは解消済み。残るのは形状検証の問題だけです。

低重要度の表記ゆれは除外しました。
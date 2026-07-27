再レビュー結果です。前回の高指摘2件は文面上ほぼ解消されています。残っている致命寄りの指摘は中のみです。

**中**

- CSV入力ファイル検証が `exists()` だけなので、ディレクトリ指定で生例外が残ります。  
  [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:115) の案は `path.exists()` だけを見ますが、`points_3d: .` や `points_2d: data/` のように存在するディレクトリを指定すると通過し、[common.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/common.py:25) / [common.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/common.py:36) の `open()` で `IsADirectoryError` になります。  
  **修正提案**: `exists()` ではなく `is_file()` を使い、メッセージも「入力ファイルが見つかりません」または「入力ファイルではありません」に統一してください。テストに `points_2d` または `points_3d` がディレクトリを指すケースを追加してください。

- K既知モード側の config 必須キー検証がテスト計画で十分に押さえられていません。  
  [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:105) は `_run_extrinsic_estimation` にも同型チェックを入れる計画ですが、確認項目は通常モードの `run_estimation` と `main()` が中心です。[run_estimation](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:628) は `intrinsic_toml` 指定時に通常モードの検証を完全にバイパスするため、K既知経路の `target_camera` / `points_3d` / `points_2d` 欠落が残っても検出できません。  
  **修正提案**: `run_estimation(..., intrinsic_toml=...)` または `_run_extrinsic_estimation` 直接で、3キーの欠落・空文字・CSV不在を検証するテストを追加してください。

**前回指摘の確認**

- 要求・設計への紐付け不足: 解消済み。
- TOMLキー欠落時に成功扱いになり得る問題: 解消済み。
- CSVファイル不在ガード不足: 概ね解消、ただし `is_file()` 不足が残存。
- CLI経路・monkeypatch対象のテスト不備: 通常モード側は解消、K既知側の検証が残存。

**高 / 低**

高: なし。  
低: 致命的な指摘なし。
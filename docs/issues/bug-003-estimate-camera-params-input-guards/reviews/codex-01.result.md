**指摘**

- **高**: bugfix標準に対する前提が崩れています。  
  [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:14) で「通常モードの点数不足時仕様は存在しない」と書いている一方、[BUGFIX_STANDARD.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/BUGFIX_STANDARD.md:15) は仕様未定義なら bug ではなく feat として要求・設計を作る、としています。さらに [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:116) は設計書なしでコード変更するとしており、[BUGFIX_STANDARD.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/BUGFIX_STANDARD.md:59) に反します。  
  **修正提案**: H-1 は既存の feat-002 FR-6 / design §5 に紐付けるか、紐付けられない M-2 などは feat として要求仕様・設計を追加してください。`investigation.md` には対応要求ID、設計セクション、引用を明記する必要があります。

- **高**: TOML必須キー欠落の修正案が、期待動作「終了コード1」と矛盾しています。  
  [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:51) は必須キー欠落時に終了コード1と定義していますが、[同:98](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:98) の案では `load_intrinsic_toml` が `None` を返します。実コードでは `None` は「スキップ」扱いで、他カメラが成功すれば `_run_extrinsic_estimation` は 0 を返します。[estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:429), [同:557](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:557)  
  **修正提案**: 「カメラセクション不在」と「セクションはあるが `matrix/distortions/size` 欠落」を区別し、後者は即 `return 1` する設計にしてください。複数カメラで1台だけ壊れたTOMLでも成功扱いにしないテストを追加してください。

- **中**: 入力ガードの範囲が不足しています。  
  [investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:95) の案は config キー存在だけを見ますが、`points_3d` / `points_2d` の値が空、またはファイル不在なら [common.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/common.py:25) / [common.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/common.py:36) で生例外になります。[investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:45) の「ファイル不在は処理済み」も事実と違います。  
  **修正提案**: 本件に含めるなら、空文字とCSVファイル存在を `run_estimation` / `_run_extrinsic_estimation` のロード前に検証し、終了コード1のテストを追加してください。含めないなら、処理済みという記述を削除し別bugとして切り出してください。

- **中**: テスト計画が実際の再現経路を十分に検証できません。  
  `target_camera` 欠落のCLI再現は [main()](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1220) で落ちますが、[investigation.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:140) は `run_estimation` だけを検証対象にしています。また [同:143](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/bug-003-estimate-camera-params-input-guards/investigation.md:143) の monkeypatch 対象は `scipy.optimize.least_squares` ではなく、実コードが直接 import している `phase0.estimate_camera_params.least_squares` にする必要があります。  
  **修正提案**: `main()` または subprocess のCLIテストを追加し、M-2 はモジュール内シンボルを monkeypatch してください。

**低**: 致命的な低重要度指摘はありません。
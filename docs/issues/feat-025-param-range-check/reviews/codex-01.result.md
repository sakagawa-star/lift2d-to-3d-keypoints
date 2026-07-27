**発見事項**

- **高**: FR-002/FR-003 の受け入れ基準を、設計書のテスト方針が満たしていません。  
  要求仕様では警告表示、正常表示、終了コード0、TOML/CSV出力維持を「テストで確認」としていますが、設計書では CLI 統合確認を手動確認に落とし、`check_param_ranges` の戻り値検証だけで済ませる方針になっています。これだと、実装が標準出力にセクションを出さなくても、終了コードや TOML/CSV 出力を壊してもテストで検出できません。  
  参照: [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/requirements.md:60), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/requirements.md:68), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/design.md:196)  
  **修正提案**: `run_estimation` の stdout を捕捉する統合テストを追加し、`[パラメータ範囲チェック]`、警告行、ヒント行、正常行、`Calib_scene.toml 形式`、`camera_params.csv 形式`、戻り値0を確認する。警告ケースは `check_param_ranges` を monkeypatch して表示経路を確実に通すのが実データ依存を避けられます。

- **中**: `--wide` 時の警告ヒントが、既存 CLI 制約と矛盾しています。  
  要求・設計では `--wide` でも p1/p2 をチェックし、警告時に常に `--zero-tangent / --fix-center` を検討せよと出します。しかし既存 CLI は `--wide` と `--zero-tangent` の併用をエラーにします。つまり `--wide` で p1/p2 警告が出た場合、表示される対処策の一部が実行不能です。  
  参照: [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/requirements.md:47), [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/requirements.md:58), [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/design.md:127), [estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1088)  
  **修正提案**: ヒント文をモード別にする。`use_wide=True` の場合は `--zero-tangent` を勧めず、`--fix-center` や標準モデルへの切り替え検討など、実行可能な対処だけを表示する設計に変更してください。

**低**: 致命的なものだけに絞ったため、低重要度の指摘はありません。
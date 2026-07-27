**前回指摘の確認**

前回の **高**（FR-002/FR-003 が自動テストで検証されない問題）は解消されています。要求仕様で stdout 捕捉テストが明記され、設計書にも `run_estimation` 統合テストが追加されています。

前回の **中**（`--wide` 時に実行不能な `--zero-tangent` を勧める問題）も、ヒント文のモード分岐として解消されています。

**残課題**

- **中**: 設計書に、まだ `--wide` と `--zero-tangent` の併用を含むように読める記述が残っています。  
  `--wide` と `--zero-tangent` は既存 CLI で併用不可ですが、設計書では「全モード（... `--wide` / `--zero-tangent` およびその併用 ...）」と書かれており、実装者・テスト作成者が `run_estimation(..., use_wide=True, zero_tangent=True)` を対象に含める余地があります。  
  参照: [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-025-param-range-check/design.md:146), [estimate_camera_params.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase0/estimate_camera_params.py:1088)  
  **修正提案**: 「およびその併用」を「併用可能な組み合わせ」に変更し、`--wide` と `--zero-tangent` は既存 CLI 制約により対象外、と明記してください。要求仕様の制約条件にも同じ一文を追加すると整合します。

**高**: なし  
**低**: 致命的なものに絞ったため、指摘なし。
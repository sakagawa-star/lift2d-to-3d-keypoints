**指摘事項**

- **高**: 後方互換要件が文字どおりには実装不能です。  
  [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:146) は `--fps-frustum` 未指定時に「エラーメッセージも完全同一」としていますが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:175) で同じ argparse parser に新規CLIを追加するため、既存エラー時の usage/help には新オプションが混ざります。  
  **修正提案**: FR-009 を「正常系の出力画像/PNG/MP4と既存オプションの挙動を不変」に絞り、argparse の usage/help/未知キー一覧は対象外と明記する。完全同一が必須なら、専用エラーフォーマッタまで設計に含める。

- **中**: `--frustum-depth` の非有限値が未定義です。  
  [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:80) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:191) は `0 以下` だけを拒否しますが、`float` は `nan` / `inf` を受け付けます。設計上は頂点計算で `depth` をそのまま掛けるため、非描画・誤描画・OpenCV描画時エラーにつながります。  
  **修正提案**: 値域を「有限な実数かつ `> 0`」に変更し、`np.isfinite(depth)` を検証条件に追加する。テストに `nan` / `inf` / `-inf` を追加する。

- **中**: `--fps-camera` のK/解像度を本当に使うことを検出するテストが不足しています。  
  要求は [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/requirements.md:61) で FPSカメラ側のK/解像度取得をMustにしていますが、テスト設計は直接Kを渡す幾何テストと存在しないカメラ名の検証に寄っています（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-034-render-keypoints-fps-frustum/design.md:238)）。観察カメラ `--camera` のKを誤って使う実装でも、現状の例とテストをすり抜けやすいです。  
  **修正提案**: TOMLにK/解像度が異なる2カメラを置き、`--camera camA --fps-camera camB` で frustum 頂点または開始サマリが camB のK/解像度に一致するテストを追加する。

**低**: 致命的な点のみ確認したため、低重要度の指摘はありません。
---

## Claude Code の対応方針（2026-08-13）

- メタ: 初回レビュー / 対象 requirements.md, design.md / session id: 019ffa10-98e4-7da0-8021-0c7bd958e638
- **高（FR-009 後方互換の実装不能）**: 採用。FR-009 を「既存機能の動作（出力画像・PNG・MP4・進捗表示・終了コード・既存オプションの挙動）の不変」に絞り、argparse の usage/help・設定YAML未知キーエラーのキー一覧は対象外と明記（requirements FR-009、design 4.5）
- **中（--frustum-depth の非有限値）**: 採用。値域を「有限かつ > 0」に変更し、`np.isfinite` 判定を design 4.6 検証3 に追加。テスト T-08 に nan/inf/-inf を追加（requirements FR-004/FR-007、design 4.2/4.6/6）
- **中（FPSカメラのK使用を検出するテスト不足）**: 採用。K・解像度が異なる2カメラTOMLで `--camera camA --fps-camera camB` とし、`compute_frustum_vertices` に渡る K・解像度が camB のものであることを monkeypatch で捕捉する T-19 を追加（requirements FR-003 受け入れ基準、design 6）

# bug-002: fps_camera_pose.py のデフォルトアーマチュア名変更にテスト・ドキュメントが未追随

## ステータス

Closed（2026-07-22完了）

## 概要

`phase4/fps_camera_pose.py` の `DEFAULT_ARMATURE_NAME` がユーザーの手動編集により
`"session001_f145749_world300"` から `"E00000"` に変更された（意図的な変更。新しい .blend
データセットのアーマチュア名に合わせたもの）。しかしテスト・設計書・CLAUDE.md が旧値の
ままであり、`uv run pytest -v` が常に1件失敗する。

## 再現手順

1. リポジトリルートで `uv run pytest -v` を実行する
2. `tests/test_feat019_fps_camera_pose_args.py::TestConstants::test_default_names` が
   `AssertionError: assert 'E00000' == 'session001_f145749_world300'` で失敗する

## 発見の経緯

feat-023 の実装時（2026-07-22）、全体テスト実行で本失敗を検出。feat-023 とは無関係の
既存事象と確認した（`tests/results/feat-023_test_result.txt` に注記あり）。

## 関連ドキュメント

- [investigation.md](investigation.md) — 調査・修正計画

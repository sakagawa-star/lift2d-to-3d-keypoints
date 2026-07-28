# bug-004: estimate_camera_params.py Codex コードレビュー（方式2）と指摘対応

## ステータス

Closed（2026-07-27 発番、2026-07-28 修正完了。手動テストで同一パス指定時のエラー終了・終了コード1・入力TOML無傷を確認済み）

## 概要

`phase0/estimate_camera_params.py` に対する第三者視点のコードレビューを Codex で実施し、重要度「高・中」の指摘があれば本案件で修正する。

背景: bug-003 で Claude Code 直接レビュー（方式1）の指摘（高1・中3）を修正済み（コミット `7014e10`）。本案件はその後のコードに対して Codex による独立レビュー（方式2）を行い、方式1で見落とした問題がないかを確認するもの。

方式1で検出済み・スコープ外とした重要度「低」4件（分岐重複、未使用 import sys、RANSAC inlier の set 順序、docstring 補足）は既知であり、本案件の修正対象としない。

## 再現手順

該当なし（レビュー実施案件。指摘が確定した時点で investigation.md に不具合ごとの再現手順を記録する）。

## 関連ドキュメント

- レビュー結果: `reviews/`
- 修正計画（指摘が出た場合）: `investigation.md`
- 先行案件: `docs/issues/bug-003-estimate-camera-params-input-guards/`

# feat-003: estimate_extrinsic.py を estimate_camera_params.py に統合（Stage 2）

## Status: Closed

## 概要

feat-001（Stage 1）で作成した `estimate_extrinsic.py`（K既知での外部パラメータ推定）の機能を `estimate_camera_params.py` に統合する。統合完了後、`estimate_extrinsic.py` の廃止を判断する。

## 背景

- Stage 1 で `estimate_extrinsic.py` をスタンドアロンツールとして作成・テスト済み
- 本案件で `estimate_camera_params.py` に `--intrinsic-toml` オプションを追加し、K既知モードを統合する
- 重複関数（load_yaml_simple, load_points_3d, load_points_2d, match_points）の共通モジュール化も行う

## 参考

- ロードマップ: `docs/issues/feat-001-estimate-extrinsic/feat-001-roadmap.md`（Stage 2 セクション）
- Stage 1 実装: `phase0/estimate_extrinsic.py`
- 統合先: `phase0/estimate_camera_params.py`

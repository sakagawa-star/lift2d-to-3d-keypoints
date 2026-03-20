# feat-002: estimate_camera_params.py 広角レンズ対応

## Status: Closed

## 概要

既存の `estimate_camera_params.py` に広角レンズ用8係数歪みモデル（k1, k2, p1, p2, k3, k4, k5, k6）を追加する。`estimate_camera_params_wide.py` の実装を参考に移植する。

## 背景

広角レンズ（画角60〜100°）では標準の4係数歪みモデルでは歪み補正が不十分。`estimate_camera_params_wide.py` として別ツールで実装済みだが、本体（`estimate_camera_params.py`）に統合して一元管理する。

## 参考

- `phase0/estimate_camera_params_wide.py` — 広角レンズ対応の独立実装
- `phase0/estimate_camera_params.py` — 移植先（既存コード）

# feat-033: render_keypoints.py の YAML 設定ファイル読み込み

## ステータス

Closed（2026-08-13 手動テスト合格・完了処理済み）

## 概要

`phase4/render_keypoints.py` に `--config <YAMLパス>` オプションを追加し、CLI オプション（位置引数を含む）の内容を YAML ファイルから読み込めるようにする。feat-029（`render_fps_video.py` の同機能）と同じ方式（簡易フラットパーサー、優先順位 CLI > 設定YAML > デフォルト）を踏襲する。

## 背景

- `render_keypoints.py` は位置引数3つ（PLY・TOML・キーポイント）+ `--camera` が毎回必要で、コマンドが長い。feat-032 で NPZ 直読みに対応し利用頻度が上がるため、定型部分をファイル化したい
- feat-029 で `render_fps_video.py` に同機能を導入済み（方式・優先順位・パーサーはヒアリング確定済みの前例がある）。feat-029 の要求仕様では他スクリプトへの展開はスコープ外（Won't）とされており、本案件はその展開にあたる

## 関連ドキュメント

- `requirements.md` — 要求仕様書
- `design.md` — 機能設計書
- 関連案件: feat-029（render_fps_video.py の YAML 設定読み込み。方式の確立元）、feat-032（NPZ 入力対応。位置引数 `keypoints_path` の現仕様）

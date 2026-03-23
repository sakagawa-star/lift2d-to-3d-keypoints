# feat-008: estimate_camera_params.py 複数カメラ一括推定

## Status: Open

## 概要

config.yaml で複数カメラを指定し、`--intrinsic-toml` モードで一括推定できるようにする。
推定結果を1つのTOMLファイルにまとめて出力する。

現状は1台ごとに config.yaml の `target_camera` を書き換えて実行する必要があり、6台のカメラを扱う際に非常に手間がかかる。

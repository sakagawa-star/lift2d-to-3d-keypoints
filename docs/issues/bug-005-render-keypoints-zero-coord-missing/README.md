# bug-005: render_keypoints.py が座標 (0,0,0) の欠損マーカーを有効点として描画する

## ステータス

Review（2026-10-18 Codex レビュー収束〔codex-01〜03 反復、codex-04 全文ゲートで高・中・低ゼロ〕。人レビュー待ち）

## 概要

Pose2Sim が書き出した C3D では、欠損したマーカーが「座標 (0, 0, 0)・residual 0」で記録されている。`phase4/render_keypoints.py` の `extract_keypoints` は「residual < 0 または NaN」だけを無効と判定するため、この点を有効点として扱う。その結果、ワールド原点を投影した画素に向かってボーンが引かれ、スケルトンが崩れる。

## 発見の経緯

- feat-037（複数人対応）の手動テスト中に発見した（2026-10-18）
- feat-037 で入った不具合ではない。1人モード（feat-036 時点の動作）でも同じように崩れる、以前からの不具合
- これまでの入力（`npz_to_c3d.py` 製の C3D、NPZ）は欠損を residual = −1 または NaN で表していたため、表に出ていなかった

## 再現手順

1. 設定YAML `phase4/data/run_keypoints_tokyo1.yaml`（git 管理外）を用意する。内容:
   - PLY: `/home/sakagawa/data/PLY/tokyo-20261006-01/point_cloud/iteration_100/point_cloud.ply`
   - TOML: `/home/sakagawa/git/pose2sim/Pose2Sim/20261006-tokyo/calibration/Config_scene.toml`、カメラ `cam05520126`、`near_plane: 0.5`
   - キーポイント: `m6_full_person1_0-54006.c3d` と `m6_full_person2_0-54006.c3d`（`/home/sakagawa/git/pose2sim/Pose2Sim/20261006-tokyo/m6_full_person{1,2}/pose-3d/`。Pose2Sim 出力）
2. プロジェクトルートで実行する:
   ```bash
   TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python phase4/render_keypoints.py \
       --config phase4/data/run_keypoints_tokyo1.yaml --start-frame 1 --end-frame 1
   ```
3. 出力 `frame_000001.png` で、P1 の赤・青のボーンが画像上の1点（u≈1225, v≈468）に集まっている

1人モードでも再現する（キーポイントファイルを `m6_full_person1_0-54006.c3d` 1個にしても同じ崩れ方になる）。

## 関連ドキュメント

- `investigation.md` — 調査・修正計画
- 関連案件: feat-016（`extract_keypoints` の有効判定の導入）、feat-021（欠損マーカー許容。「無効サンプル」の定義）、feat-037（発見元。完了確認は本案件の完了後に行う）

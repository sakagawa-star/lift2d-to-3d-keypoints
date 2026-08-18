再レビューしました。前回指摘5件は、文書上はすべて解消されています。

**前回指摘の解消状況**
- アンカー不足カメラのBA参加問題: 解消。`N_A >= 30` のカメラのみBA参加に変更済み。
- TOML出力で失敗カメラが残る問題: 解消。専用 `write_multiview_toml` 新設方針に変更済み。
- MASt3R OOM fallback不能問題: 解消。CLI内 `--resolution 1280x720` に整理済み。
- ブートストラップで `gate_breakdown` を使う問題: 解消。専用 `gate_depth_quality` に分離済み。
- 三角測量の座標系曖昧さ: 解消。正規化座標 + `[R|t]` に固定済み。

**高**
なし。

**中**
1. **`--toml` と `--init-toml` のマージ規則がまだ実装者依存**
   [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:43) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:105) では初期ポーズTOMLで較正済み集合を作る方針ですが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:57) は `load_cameras_toml()` の戻り値を使うとだけあります。既存 `load_cameras_toml()` はTOML内の `rotation/translation` を常に読むため、`--toml` 側の pose を誤って全カメラの初期値扱いする実装になり得ます。
   修正提案: `--toml` からは `K/D/size` のみ採用し、`rvec/tvec` は `--init-toml` に存在するカメラだけコピー、その他は必ず `None` にする、と明記してください。

2. **アンカー取得の LoFTR confidence filter の所在が実コードと違う**
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:120) は `call_matcher` が `conf >= 0.2` 選別を含むように読めますが、実際の `call_matcher` は raw `conf` を返すだけで、選別は呼び出し側の `run_iteration` で行っています（[refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:456), [refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:511)）。
   修正提案: `collect_anchor_points` 側で `conf >= LOFTR_CONF_TH` を明示的に適用すると設計に書いてください。`call_matcher` は「raw matchを返す」と定義するのが安全です。

**低**
なし。
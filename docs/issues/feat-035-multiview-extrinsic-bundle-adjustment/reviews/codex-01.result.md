レビュー基準に照らし、致命的または誤実装に直結する点だけ挙げます。低重要度の指摘はありません。

**高**
1. **アンカー不足カメラの外部パラメータが metric に定まらない**
   [requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:64) は3DGS座標系・スケール固定を要求していますが、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:120) ではアンカー不足カメラをクロス項のみで最適化対象にしています。2視点対応＋自由3D点だけでは、アンカーを持たないカメラの並進スケールは観測不能です。
   修正提案: 最適化対象カメラ全台に `N_A >= 30` を必須化するか、アンカーなしカメラには PnP 初期姿勢への pose prior、カメラ中心距離 prior、または別の metric 拘束を追加してください。少なくともアンカー/gaugeを持たない連結成分はエラー終了にする必要があります。

2. **FR-007 の「失敗カメラをTOMLに含めない」が既存 writer 流用では満たせない**
   要求は失敗カメラ除外です（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:80)）。設計は `write_output_toml` 流用としていますが（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:148)）、既存実装は入力TOMLの全カメラを書き、accepted のみ pose 置換します（[refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:873)）。
   修正提案: feat-035専用 writer を作り、内部パラメータTOMLから `matrix/distortions/size` を取り、較正成功カメラだけに最終 `rotation/translation` を注入して出力してください。テストで「成功カメラは含む、失敗カメラは含まない」を固定してください。

3. **MASt3R OOM fallback が設計上実行不能**
   要求は OOM 時に 1280x720 再試行です（[requirements.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:105)）。一方で CLI は 1920x1080 以外を終了コード1にすると定義されています（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:81)）。Stage P の 1280x720 再試行（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:94)）が常に失敗します。
   修正提案: CLI に `--resize 1280x720` 等を持たせ、原寸画像を渡したまま内部で縮小・逆写像する仕様にしてください。少なくとも 1280x720 入力を明示的に許可し、`meta` に元解像度・処理解像度・scale・roundtrip を記録してください。

**中**
1. **ブートストラップで `gate_breakdown` をそのまま使うと広基線マッチを誤って落とす**
   設計は cross 対応点に `gate_breakdown` と同一ゲートを使うとしています（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:107)）。しかし既存 `gate_breakdown` は `u_q` と `u_r` の画素変位閾値も含みます（[refine_extrinsics.py](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/refine_extrinsics.py:420)）。別カメラ間では画素変位比較に意味がなく、 valid な広基線対応を落とします。
   修正提案: ブートストラップ用に `depth/alpha/z/local variance` のみを見る別ゲートを定義してください。`gate_breakdown` はアンカー取得専用に限定するのが安全です。

2. **三角測量の座標系仕様が不足しており、実装が分岐し得る**
   `undistortPoints → triangulatePoints` とありますが、射影行列を `[R|t]` にするのか `K[R|t]` にするのかが明記されていません（[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:128)）。ここを間違えると BA 初期点とホールドアウト評価が壊れます。
   修正提案: `cv2.undistortPoints(..., P=None)` で正規化座標を使い、`P_i = [R_i | t_i]` を渡す、と明記してください。ピクセル座標を使う方式なら `P_i = K_i [R_i | t_i]` とし、両方式を混在させないでください。
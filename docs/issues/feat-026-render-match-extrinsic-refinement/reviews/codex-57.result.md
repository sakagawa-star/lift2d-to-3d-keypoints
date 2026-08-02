**中**

[criteria.md:29](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/criteria.md:29>) / [TECH_STACK.md:129](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/TECH_STACK.md:129>)  
LoFTR の評価対象に `indoor_new` が追加されていますが、`docs/TECH_STACK.md` の外部アセットには `loftr_outdoor.ckpt` しか固定されていません。現ローカルキャッシュにも `loftr_outdoor.ckpt` だけが存在します。`indoor_new` を評価対象にするなら、実行時にネットワーク取得へ依存する可能性があり、criteria lock 後の実行不能要因になります。

修正提案: `indoor_new` の checkpoint パス、入手元、サイズ、可能なら sha256 を criteria または TECH_STACK に固定してください。フェーズ0の前提確認にも「LoFTR outdoor / indoor_new の両重みがオフラインでロード可能」を入れるとよいです。

**高**

該当なし。

**低**

該当なし。

前回指摘の解消確認:

- TOML パス不一致: 解消済み。M3 と同一の pose2sim 側 TOML に修正され、sha256 も固定されています。
- LoFTR 選択規則のタイブレーク: 解消済み。最後の同点条件で `indoor_new` 優先が固定されています。
- 逆写像の数値固定: 解消済み。round-trip `≤0.5px`、範囲内率 `100%`、MASt3R 変換式の記録が追加されています。
- OOM fallback 適用単位: 解消済み。1ペアでも OOM なら LoFTR 全設定・全カメラを `1280x720` に統一する規則になっています。
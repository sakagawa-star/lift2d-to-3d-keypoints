レビュー結果: **高 2件、中・低 0件**。criteria lock 前に直すべき致命点だけです。

**高**

1. [criteria.md:53](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/criteria.md:53) のタイルNPZ生成が固定されていません。  
`準備スクリプトで生成する` とありますが、実験フォルダには `criteria.md` しかなく、生成スクリプトの実体・実行コマンド・検証コマンドが lock 対象になっていません。入力データ自体が実行者依存になり、事後解釈の余地があります。  
修正提案: `make_tiled_npz.py` などを実験フォルダに追加し、criteria に完全な生成コマンドと検証条件を固定してください。最低限、出力パス、tile回数=10、`frame_ids=145599..148598`、shape `(3000,22,3)`、NaN数 `0`、`compute_fps_poses` の有効フレーム `3000/3000` を Phase 0 合格条件として数値化するべきです。

2. [criteria.md:71](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/criteria.md:71) / [criteria.md:76](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/criteria.md:76) / [criteria.md:78](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-030-render-fps-video-parallel/experiments/e1_single_gpu_concurrency/criteria.md:78) の測定ログ定義が、実装ログと一致せず、同時性判定も再現できません。  
実装はチャンクごとに `秒/フレーム` を出さず、[render_fps_video.py:953](/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/render_fps_video.py:953) で `描画 n/N フレーム, sec秒` を出すだけです。また stdout / `nvidia-smi` コマンドにタイムスタンプがないため、「チャンク1以降の実時間ウィンドウの重なり8割」や「Phase 1 定常区間のGPU使用率中央値」を後から一意に算出できません。  
修正提案: criteria に実ログ形式へ合わせた抽出式を明記してください。例: `チャンク ([0-9]+).*描画 ([0-9]+)/[0-9]+ フレーム, ([0-9.]+)秒` から `chunk_index, n_rendered, sec` を取る。さらに `PYTHONUNBUFFERED=1` / `python -u` と timestamp 付き保存コマンドを固定し、`nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used ...` のように時刻列を含める。重なり率も `intersection_duration / union_duration >= 0.80` など式で固定してください。

補足: 元NPZ、PLY、TOMLの実在は確認しました。元NPZは `(300,22,3)`, `frame_ids 145599..145898`, NaN 0、有効ポーズ `300/300` です。
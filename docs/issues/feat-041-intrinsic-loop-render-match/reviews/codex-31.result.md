# codex-31 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md（関連: m1 ログ、bug-006 README）
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート 4 回目（`/new` で会話をクリアしてから依頼）
- 指摘数: 高 1 / 中 3 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1118979,"cached_input_tokens":1016960,"cache_write_input_tokens":0,"output_tokens":4282,"reasoning_output_tokens":794,"total_tokens":1123261}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-47-00-01a2113e-3adb-7923-99a7-be58d79bd5b5.jsonl

## 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| 1（高） | 正式実行の事前確認は --out-dir だけで、固定の --tmp-dir（iter00_render.png 等を直接書く）に入力を指すリンクが残っていれば入力を上書きし得る | 事前確認の対象に中間ディレクトリ feat041_tmp_it2 も加えた（test ! -e && test ! -L）。design.md §9 手順 6、investigation.md §2.5、criteria.md |
| 2（中） | FR-008 の基準 2（10% 未満は判定不能）と改訂した基準 3 が競合 | 基準 3 に「集計対象画素率が 10% 以上の場合」を明記。§7 の合格ラインにも同じ条件を入れた |
| 3（中） | 未収束時の TOML 書き込み失敗時の終了状態が未定義 | OSError を捕まえ、途中のファイルを unlink(missing_ok=True) で消し、終了状態「エラー（ループ結果 TOML の書き込みに失敗: …）」、終了コード 1。修正コード（investigation.md）、終了状態表（design.md §4.2.1）に追加。テスト 22 を追加 |
| 4（中） | 正式実行が途中で失敗した場合の記録と、手動テストへの遷移が未定義 | 到達しなかった項目は理由付きで「未実行／取得不能」と記録し総合不合格。不合格なら再試行やステップ7 に進まず次のイテレーションへ。予測と実測の乖離は原因を記録してからステップ7 へ（investigation.md §2.5、design.md §9、criteria.md） |

# codex-23 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md（関連: experiments/m1_p95_tail/experiment_log.md）
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート 2 回目（`/new` で会話をクリアしてから依頼）
- 指摘数: 高 1 / 中 3 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":922294,"cached_input_tokens":810624,"cache_write_input_tokens":0,"output_tokens":4659,"reasoning_output_tokens":938,"total_tokens":926953}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-35-32-01a21133-bba5-74f1-815b-dbd1791f8ba5.jsonl

## 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| 1（高） | 最終精緻化の一時ファイル（refine_extrinsics.py が書く {camera}_final.toml.tmp）が入力 TOML を上書きし得る。事前検査に含めるべき | 初版の設計から存在する性質で、イテレーション2 が生んだものではないため、BUGFIX_STANDARD.md §2.3 に従いスコープ外とし、別案件（bug）として起票する。investigation.md §2.3「今回の修正に含めないこと」に記録（運用上の回避: --toml に --out-dir 配下のファイルを指定しない） |
| 2（中） | 評価画像の書き込み失敗（cv2.imwrite の戻り値未確認）でも正式判定が合格し得る | コード修正はスコープ外（初版から）。criteria.md §2 に項目 6（最終 TOML と評価 4 ファイルが読めること）を追加して正式判定で検出。investigation.md §2.3・§2.5、design.md §9 手順 6 を更新 |
| 3（中） | ループ失敗時に TOML を残す条件が設計内で矛盾（逆歪み検査の不合格は TOML を書いた後） | design.md §4.2.1 に「終了状態とループ結果 TOML の有無」の表を追加し、終了の場面ごとに終了コード・終了状態・TOML の有無を定義。investigation.md §2.3 も合わせた |
| 4（中） | p95 の採否理由に、m1b で裏付けられない原因の除外が残る（主点の一様なずれでも放射成分の符号は左右で変わり得る） | requirements.md FR-008、design.md ADR、investigation.md §2.2・§2.3 の根拠を観測事実（最下段に集中、C 中央値 0.81、Rr 正 4・負 4、隣接セルで反転）に限定し、原因未特定と記載。「歪み係数の追加」の除外理由も「原因未特定のため」に変更。m1 ログに注記2を追記 |

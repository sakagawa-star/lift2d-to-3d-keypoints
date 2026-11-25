# codex-17 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md（関連: experiments/m1_p95_tail/experiment_log.md）
- ストリーム: rev-lift2d-feat-041
- フェーズ: 全文ゲート（`/new` で会話をクリアしてから依頼）
- 指摘数: 高 0 / 中 6 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":739786,"cached_input_tokens":654336,"cache_write_input_tokens":0,"output_tokens":4679,"reasoning_output_tokens":1006,"total_tokens":744465}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-25-55-01a2112a-eb49-7c70-a929-798e4dfa18fa.jsonl

## 指摘と対応

| # | 指摘（中） | 対応 |
|---|---|---|
| 1 | 未収束時の TOML が最後の反復の結果か、テスト 19 で確かめられない（f だけ照合、交互の値で i=5 でも一致） | テスト 19 を、反復ごとに固有の値を返すモックにし、呼び出し 8 回と K・D・rvec・tvec の i=7 との一致を確かめる形に強化 |
| 2 | 同じ出力先で `--overwrite` 再実行して未収束になると、前回の _final.toml・評価ファイルが残り新旧が混在する | `--overwrite` 時は出力 8 ファイルを処理開始前に削除（NFR-004、design §4.2.1）。テスト 20 を追加 |
| 3 | eval.txt は小数 3 桁で、criteria の数値条件を境界値で検証できない | eval.txt に valid_ratio_raw・median_raw（repr）を追加し、format_eval_txt に分離（design §4.3.4、§6.3）。テスト 21 追加、テスト 13 に表示上同じ値になる境界の両側を追加。criteria は _raw で判定 |
| 4 | m1b のログで原因側を数値ルールで判定しているが、条件が排他的でなく criteria lock の記録も無い | investigation §2.2 の根拠を実測値（C 中央値 0.81、Rr 正 4・負 4）に置き換え、分類を根拠に使わないと明記。m1 ログに注記を追記 |
| 5 | CONV_PX = 2.0 の選定理由が判定式と逆（「上限を超えない値」） | 「観測した揺れの上限 1.872px を上回る値。根拠は 10F の 1 回の観測」に修正（investigation §2.3、design §3.1・ADR） |
| 6 | 正式な手動テストの実行者が不一致（design §9 は実装担当、investigation はステップ7） | ステップ6 の実装担当による手順 6 の 1 回の実行を正式な結果とし、ユーザーはステップ7 で出力を確認、と investigation §2.5・design §9・criteria §2 に明記 |

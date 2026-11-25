# codex-20 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 解消確認（codex-19 と同じ会話。`/new` は実施せず）
- 指摘数: 高 0 / 中 1（新規） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":2064709,"cached_input_tokens":1917184,"cache_write_input_tokens":0,"output_tokens":10840,"reasoning_output_tokens":2050,"total_tokens":2075549}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T14-25-55-01a2112a-eb49-7c70-a929-798e4dfa18fa.jsonl

## 解消確認

| 指摘 | 判定 |
|---|---|
| codex-19 未解消: 削除途中の失敗・入力検証前の喪失 | 解消（削除処理そのものが無くなった） |
| codex-19 新規: リンク先の無いシンボリックリンク | 解消（os.path.lexists、テスト 20） |
| codex-18 新規1: 入力 TOML の削除／新旧混在 | 解消維持 |

## 新規指摘

- **中 — 同じ出力先への同時実行で、両方が存在検査を通過できる**: 同じ --out-dir とカメラ名で 2 プロセスを起動すると、出力が混在し得る。存在検査の前に排他し、後続を終了コード 1 で拒否するよう定義すること

## 対応

スコープ外として扱う（BUGFIX_STANDARD.md §2.3）。初版の設計（codex-08 ゲート通過）から存在する性質で、イテレーション2 の変更が生んだものではない。運用は GPU 1 枚で 1 回ずつの実行で、同時実行は想定していない。investigation.md §2.3「今回の修正に含めないこと」に既知の制約として記録した。Codex に再判定を依頼する。

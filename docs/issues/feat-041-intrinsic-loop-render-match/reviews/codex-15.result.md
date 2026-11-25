# codex-15 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md、experiments/e3_10F_it2/criteria.md（新規）
- ストリーム: rev-lift2d-feat-041
- フェーズ: 反復（codex-14 の解消確認。`/new` は実施せず）
- 指摘数: 高 0 / 中 2（未解消 1、新規 1） / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":1168214,"cached_input_tokens":984704,"cache_write_input_tokens":0,"output_tokens":4495,"reasoning_output_tokens":501,"total_tokens":1172709}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T12-16-17-01a210b4-3de5-7093-b2e6-3e852789c4f0.jsonl

## 解消確認

| 前回の指摘 | 判定 | 確認内容（Codex） |
|---|---|---|
| codex-14 指摘1（中）: criteria lock の欠落 | 解消 | criteria.md に問い・終了状態・閾値・60 分上限が定義され、investigation.md・design.md に高・中ゼロ後に正式実行すると明記 |
| codex-14 指摘2（中）: p95 の原因の断定 | 未解消（中） | §2.1 問題C の見出しが「3DGS の再現品質で決まっている」と断定したまま。FR-008 と §2.3 の「カメラパラメータの誤差と合わなかった」も、測定で比較した放射方向のパターンに範囲を絞るべき |

## 新規指摘

### 1. 中 — 正式実行前に記録する予測項目が文書間で一致していない

- 対象: criteria.md §3、investigation.md §2.5、design.md §9 手順 6
- 内容: criteria.md §3 は最終 cx・cy、精緻化の受理結果、全体の所要時間も予測対象にしているが、investigation.md と design.md の一覧に無い
- 修正案: 両手順から criteria.md §3 を参照し、同じ項目を使うよう統一する

## 対応

- 未解消分: investigation.md の問題C の見出しを「p95 が旧合格ライン（3.0px）を超えた。p95 を押し上げる原因は未特定」に変更。investigation.md（2 か所）と requirements.md FR-008 の「カメラパラメータの誤差と合わなかった」を「歪み・主点の誤差の放射方向のパターンと合わなかった（姿勢誤差の寄与は未分離）」に変更
- 新規指摘1: investigation.md §2.5 と design.md §9 手順 6 の予測項目を criteria.md §3 の参照に統一（項目も列挙）

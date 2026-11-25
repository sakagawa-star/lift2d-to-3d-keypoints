# codex-14 レビュー結果

- 日付: 2026-11-25
- 対象: investigation.md（イテレーション2）、requirements.md、design.md
- ストリーム: rev-lift2d-feat-041
- フェーズ: 反復（イテレーション2 の初回。`/new` は実施せず、codex-13 の会話を継続）
- 指摘数: 高 0 / 中 2 / 低 0
- トークン実測（累積）: "total_token_usage":{"input_tokens":814696,"cached_input_tokens":644224,"cache_write_input_tokens":0,"output_tokens":3058,"reasoning_output_tokens":421,"total_tokens":817754}
- rollout: /home/sakagawa/.codex/sessions/2026/11/25/rollout-2026-11-25T12-16-17-01a210b4-3de5-7093-b2e6-3e852789c4f0.jsonl

## 総評（Codex）

収束しきい値 2.0px、未収束時の最終反復 TOML 出力と終了コード 3、p95 の記録継続と合否判定からの除外は、要求・設計に反映されている。テスト 13・18・19 も変更案と一致。

## 指摘

### 1. 中 — 次回の正式実行に必要な criteria lock が手順に無い

- 対象: CLAUDE.md、investigation.md §2.5、design.md §9 手順 6
- 内容: 基準変更の経緯の明記と、次回の実行で判定する方針は適切。一方、確認手順は直前予測と実測の照合までで、CLAUDE.md が求める criteria 文書の事前レビュー（高・中ゼロの確定）が抜けている。同じ 10F 入力での再実行で確認できるのは、固定した新仕様を実装が満たすこと
- 修正案: experiments/e3_10F_it2/criteria.md に問い、valid_ratio ≥ 0.10、median ≤ 1.0px、全体 60 分以内を記載し、レビューで高・中ゼロになってから実行する手順を追加する
- 対応: criteria.md を新規作成（問い・確認できる範囲・合格条件 5 項目・予測の枠・参考値）。investigation.md §2.5 と design.md §9 手順 6 に criteria lock の手順を追加

### 2. 中 — p95 の原因を、追加測定で確かめた範囲より強く断定している

- 対象: 追加測定ログ、investigation.md §2.2、requirements.md FR-008、design.md ADR
- 内容: m1b が示したのは、最下段で放射成分の符号がセル間でそろわず、事前の分類で「3DGS 側」に該当したこと。3DGS の形状誤差・DIS の誤差・姿勢誤差の寄与は分離できておらず、中央値がカメラパラメータの良否を表すことも検証していない
- 修正案: BUGFIX_STANDARD.md §2.1 に従い、p95 を押し上げた原因は未特定とし、下側の撮影不足による局所的な再現誤差を仮説として記載する。ユーザー決定は維持し、FR-008 と ADR の根拠を実測と仮説に合わせる
- 対応: investigation.md §2.1（期待する動作）、§2.2（確かめた事実／原因未特定・仮説／未分離・未検証の明記）、§2.3（根拠）、requirements.md FR-008 受け入れ基準 3、design.md ADR「合否基準」を修正

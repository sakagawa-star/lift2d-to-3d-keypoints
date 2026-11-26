# codex-01（feat-044 初回レビュー）

- 日付: 2026-11-26
- 対象: docs/issues/feat-044-remove-expect-exif-options/requirements.md、design.md（関連: README.md、prepare_phone_photo.py、estimate_intrinsics_loop.py、テスト、feat-041/043 の文書）
- ストリーム: rev-lift2d-feat-044
- フェーズ: 反復（初回。依頼文に「致命的な点のみ指摘」の前提）
- 指摘数: 高 0 / 中 1 / 低 0
- トークン: total_tokens 358,325（input 355,711 うち cached 308,480、output 2,614、reasoning 754）
- rollout: ~/.codex/sessions/2026/11/26/rollout-2026-11-26T10-02-43-01a21560-4fb0-7140-bbcb-15ac5656ea07.jsonl

## 指摘

### 指摘1（中）実データ検証で既存の設定 YAML を上書きし得る

設計 §6.4 は固定パス `phase4/data/10F/feat044_job.yaml` に検証用 YAML を作るが、そのパスが使用済みの場合の扱いを定めていない。前処理の上書き防止が働くより先に、研究者が編集した設定を失う可能性がある。検証ごとに新規ディレクトリを作り、YAML と出力をその配下に置く手順に修正する。

## 総評

FR-001・FR-002 の削除範囲と終了コードについて、実装を妨げる矛盾は見つからない。テストは未実行。

## 対応

- 指摘1: §6.4 を「`mkdir phase4/data/10F/feat044_check`（`-p` なし。既存なら中断して報告）で新しいディレクトリを作り、検証用 YAML（`job.yaml`）と出力をすべてその配下に置く。`job_s25.yaml` は編集しない」に修正

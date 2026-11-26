# codex-10 レビュー結果

- 日付: 2026-11-26
- 対象: docs/issues/feat-045-estimate-wizard/investigation.md（イテレーション1）、requirements.md、design.md
- ストリーム: rev-lift2d-feat-045
- フェーズ: 全文ゲート（/new 実施後。依頼文に「致命的な点のみ」とリンク検査の指摘は不要の前提）
- 指摘数: 高 1 / 中 2 / 低 0
- トークン: total_tokens 728,688（input 724,363、cached 637,824、output 4,325）。~/.codex/sessions/2026/11/26/rollout-2026-11-26T16-17-28-01a216b7-683f-75f1-9a56-c989cf24d93c.jsonl

## 指摘

1. **高** CSV の文字コードを間違える（例 CP932）と、既存の読込関数が UnicodeDecodeError を送出し、設計の捕捉対象から漏れて対話が落ちる。修正案: FR-003 に UTF-8 を明記し、文字コードエラーを理由付きで表示して CSV 入力へ戻す
2. **中** 修正計画が求める「フォルダ作成前に停止」をテスト No.29（補助関数の戻り値だけ）では判定できない。修正案: 改行を含む親パスに JOBS_DIR を差し替えて main を通し、終了コード 1 と作業フォルダが作られないことを確認
3. **中** 3 ツールの実行中に中断すると、作業フォルダのパスだけが表示され、通常のコマンドでは既存出力の検査で止まる。修正案: YAML 作成前は新規フォルダでやり直し、作成後は §4.7 の復旧コマンドで続ける条件を FR-006 と §4.8 に揃える

## 対応

1. requirements FR-003 に UTF-8 を明記。design §4.3 手順 5.2 で UnicodeDecodeError を捕まえ、理由を表示して CSV のパスを聞き直す。テスト No.31
2. テスト No.29b（main を通し、終了コード 1、作業フォルダが作られないこと）を追加。investigation.md 1.4 に反映
3. requirements FR-006 に中断時の続け方（YAML 前は新しい作業フォルダ名で最初から、実行中は失敗時と同じ続きのコマンド）を明記。design §4.8 で run_tools が KeyboardInterrupt を捕まえて recovery_commands を表示、YAML 前の中断では「新しい作業フォルダ名で最初からやり直してください」を表示。テスト No.32・33

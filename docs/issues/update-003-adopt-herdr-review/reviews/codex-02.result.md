# codex-02: 再レビュー結果（解消確認）

- 日付: 2026-08-31
- 対象: `docs/issues/update-003-adopt-herdr-review/README.md`、`docs/issues/update-003-adopt-herdr-review/design.md`
- ストリーム名: `rev-lift2d-update-003`（同一会話を継続）
- フェーズ: 反復（2回目。`/new` なし）
- 指摘数: 高0 / 中1（新規）/ 低0。前回指摘（高1・中1）は全件解消
- トークン実測: total 363,503（累積。input 359,178〔うち cached 297,984〕、output 4,325）
- rollout jsonl: `~/.codex/sessions/2026/08/31/rollout-2026-08-31T09-08-16-01a05525-33b0-77c2-84cf-d1f2a4aef06a.jsonl`

## 前回指摘の判定

- **高（D2 の自己完結性）: 解消**。design.md 冒頭で D2 のみ外部テンプレートを必須前提と明記し、コピー元・コミット・SHA-256・検証方法を記載。SHA-256 は指定コミットの実体と一致することを Codex 側でも確認
- **中（full.log 検証の自己矛盾）: 解消**。検証手順は D3-4 の説明中の1箇所だけを許容し、旧方式の運用記述が残っていないことを検証する内容になった

## 新規指摘

### 中: D2 のコピーコマンドが、固定したはずのコミットを実際には参照していない

D2 は `cp` で テンプレートの**作業ツリー**をコピーするが、design.md で指定したコミット `19e4977...` をコマンド上で保証していない。テンプレート作業ツリーが別コミットへ移動または未コミット変更された場合、異なる内容をコピーして検証で失敗する。

修正案: コミット固定のコピーに変更する:

```bash
git -C /home/sakagawa/git/DEV_TEMPLATE show \
  19e4977449eb534a328b6446cfcf2ec947942fb8:template/docs/HERDR_SETUP.md \
  > docs/HERDR_SETUP.md
```

これなら SHA-256 検証は「検出」だけでなく、正しい転記の保証にもなる。

## Claude Code の対応方針

- 修正案どおり、D2 のコピーコマンドを `git show {コミット}:{パス}` によるコミット固定方式に差し替える

→ design.md を修正のうえ、同一会話で再レビュー（解消確認）を依頼する（codex-03）。

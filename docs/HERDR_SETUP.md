# Herdr によるエージェント連携セットアップ手順

Claude Code が codex にレビューを依頼し、結果を受け取る構成を新しい PC で再現するための手順。
2026-08-26 に Linux（Ubuntu）上で実機検証済み。

## 前提条件

以下が満たされていること。

- `claude`（Claude Code）がインストール済み・認証済み
- `codex`（Codex CLI）がインストール済み・認証済み
- `~/.claude` ディレクトリが存在する（Claude Code を一度起動すれば作られる）
- `npx` が使える（Node.js が入っている）。無い場合は手順3の代替手順を使う

`CLAUDE_CONFIG_DIR` を設定している場合は、以下の `~/.claude` をそのパスに読み替える。

## 手順1：Herdr のインストール

```
curl -fsSL https://herdr.dev/install.sh | sh
```

Homebrew を使う場合は `brew install herdr`。

確認：

```
herdr --version
```

## 手順2：Claude Code 連携フックの導入

目的：Claude Code がセッション開始時に自分の識別情報を Herdr のローカルソケットへ報告するようにする。これがないと Herdr が「どのペインでどの Claude Code セッションが動いているか」を対応付けられず、`herdr agent` 系コマンドの宛先解決ができない。

```
herdr integration install claude
```

書き込まれるもの（2つのみ）：

- `~/.claude/hooks/herdr-agent-state.sh`（新規作成）
- `~/.claude/settings.json` に `hooks` キーを追加

追加される内容：

```json
"hooks": {"SessionStart":[{"matcher":"*","hooks":[{"type":"command","command":"bash '<HOME>/.claude/hooks/herdr-agent-state.sh' session","timeout":10}]}]}
```

確認：

```
grep -c herdr ~/.claude/settings.json
```

1 以上なら導入済み。

## 手順3：Herdr スキルの導入

目的：Claude Code に `herdr` コマンドの使い方を教える。これがないと Claude Code はペイン操作もエージェント起動もできない。

```
npx skills add herdrdev/herdr --skill herdr -g
```

対話画面が出る。

1. 「Additional agents」の `Search:` に `claude` と入力
2. `Claude Code (<HOME>/.claude/skills)` を `↑↓` で選び `space` で選択
3. `enter` で確定

画面上部の「Universal (.agents/skills)」は常時対象だが、Claude Code はそこを読まないので、Claude Code を明示的に選ぶ必要がある。

確認：

```
ls -l ~/.claude/skills/herdr
```

`~/.agents/skills/herdr/` へのシンボリックリンクが表示される。Claude Code はシンボリックリンクをたどってリンク先の `SKILL.md` を読むため、これで正常。

### npx が無い環境の場合

Herdr のドキュメントは、リポジトリ内の `SKILL.md` を「手動フォールバックかつ正典」とし、スキル機構を持つエージェントには「そのファイルを `herdr` という名前のスキルとして導入せよ」と記載している。`skills/herdr/` の中身は `SKILL.md` 1ファイルのみ（10,553バイト、GitHub API で確認）であり、補助ファイルは無いため、このファイルを1つ置けば導入は完結する。

手順1で Herdr を導入済みなら、バイナリ同梱のリリース対応版を出力できる。

```
mkdir -p ~/.claude/skills/herdr
herdr --skill > ~/.claude/skills/herdr/SKILL.md
```

`herdr --skill` はドキュメントに "print the release-matched copy bundled with that binary"（そのバイナリに同梱されたリリース対応版を出力する）と記載されており、導入済み Herdr のバージョンに対応した内容が得られる。

Herdr がまだ無い、あるいは別マシン向けにファイルだけ用意する場合はリポジトリから取得する。ただしこちらは最新版であり、導入する Herdr のバージョンと一致する保証はない。

```
mkdir -p ~/.claude/skills/herdr
curl -fsSL https://raw.githubusercontent.com/herdrdev/herdr/master/skills/herdr/SKILL.md \
  -o ~/.claude/skills/herdr/SKILL.md
```

確認：

```
head -3 ~/.claude/skills/herdr/SKILL.md
```

`---` に続いて `name: herdr` が表示されれば正しい形式。

## 手順4：動作確認

1. 対象プロジェクトのディレクトリで `herdr` を起動
2. そのペインで `claude` を起動
3. 以下を入力（`<ファイルパス>` は既存のテキストファイル）

```
herdr で右に新しいペインを作り、そこに reviewer という名前で codex を起動してください。
codex には --sandbox read-only --ask-for-approval never を渡すこと。
起動できたら reviewer に <ファイルパス> のレビューを依頼してください。
依頼内容は「<ファイルパス> を読み、記述の矛盾・不足・曖昧な箇所のみを指摘せよ」。
完了を待って結果を読み取り、指摘を一覧にして見せてください。
```

Claude Code の出力に `Skill(herdr)` と表示され、右ペインに codex が起動し、指摘が一覧で返れば成功。

## 日常の使い方

毎回必要なのは以下だけ。手順1〜3は最初の1回のみ。

1. プロジェクトディレクトリで `herdr`
2. `claude`
3. 依頼文に「herdr」という語を含めて依頼する

依頼文の型は手順4のものをそのまま使う。

## 注意点

- **依頼文に「herdr」を含める**。スキルの description に「ユーザーが明示的に Herdr に言及したときのみ使う」と明記されているため、含めないと発動しない可能性がある。
- **エージェント名は稼働中のもの同士で一意**。前回の `reviewer` が残っていると同名で起動できない。終了させるか別名を使う。
- **`--sandbox read-only` を外すと codex がファイルを書き換えられる**。レビュー用途では読み取りのみで足りるため付ける。
- **`--ask-for-approval never` を外すと承認ダイアログで止まる**。その状態で `herdr agent prompt` を送ると `agent_blocked` が返り、入力は届かない。
- **Herdr の外で `claude` を起動しても連携は動かない**。スキルは `HERDR_ENV=1` を検査し、満たさなければ処理を中止する。

## Claude Code が内部で実行するコマンド

不具合の切り分け用。人間が手で打っても同じ結果になる。

```
herdr pane split --current --direction right --cwd "$PWD" --no-focus
herdr agent start reviewer --kind codex --pane <pane_id> -- --sandbox read-only --ask-for-approval never
herdr agent prompt reviewer "<依頼文>" --wait --timeout 120000
herdr agent read reviewer --source recent-unwrapped --lines 200
```

- `pane split` の戻り値 JSON の `.result.pane.pane_id` が新しいペインの ID
- `agent start` は Herdr がエージェントを検出して入力可能と判断するまで戻らない（既定30秒）。起動時に停止していると `agent_not_ready` を即座に返す
- `agent prompt --wait` は `idle` / `done` / `blocked` のいずれかに落ち着くまで待つ。`--until` で既定状態を重ねて書かないこと
- `agent read` の `--source` は `visible` / `recent` / `recent-unwrapped` / `detection`（既定は `recent`）

`--kind` の指定可能値は `herdr agent start --help` で確認できる。

## アンインストール

```
rm ~/.claude/hooks/herdr-agent-state.sh
rm -rf ~/.claude/skills/herdr ~/.agents/skills/herdr
```

加えて `~/.claude/settings.json` から `hooks` キーを削除する（`herdr integration install claude` 実行前に他のフックを使っていた場合は、その分は残すこと）。

## 出典

- https://herdr.dev/docs/agent-skill/
- https://herdr.dev/docs/cli-reference/
- https://herdr.dev/docs/integrations/
- https://code.claude.com/docs/en/skills
- https://learn.chatgpt.com/docs/developer-commands?surface=cli
- 各コマンドの `--help` 出力および実機での実行結果

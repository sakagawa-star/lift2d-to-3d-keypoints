**中**: `.claude/handovers/` の追跡解除で、現行リポジトリの HEAD から固有情報が落ちます。  
前回の「未追跡前提」は [README.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/README.md:83>) と [design.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/design.md:288>) で修正済みですが、今度は tracked handovers 14件・計806行を削除対象にしています。削除対象には、たとえば「引き継ぎノートを鵜呑みにせずディスク実在を確認する」という運用上の注意が含まれています [2026-06-14_1530.md](</home/sakagawa/git/lift2d-to-3d-keypoints/.claude/handovers/2026-06-14_1530.md:35>)。同文脈は `docs/` / `CLAUDE.md` 側には見つかりませんでした。

**修正提案**: G1 の前処理として、tracked handovers 14件の棚卸しを明記してください。恒久化する知見は `CLAUDE.md` または適切な `docs/` に移してから `git rm -r --cached .claude/handovers` を実行する。移さず捨てる判断なら、「内容を確認し、現行 docs に残さないことを明示的に決定済み」と README/design に書くべきです。

前回指摘の C9 追加と REVIEW_CRITERIA のコミット固定取得は解消済みです。高・低の新規指摘はありません。

---

## Claude Code の対応方針（2026-08-03）

- **中（handovers 棚卸し）**: 14件（計806行）を全文確認して棚卸しを実施し、README に14件のファイル別一覧（内容と恒久情報の所在）を明記した。恒久化すべき情報は案件ドキュメント・CHANGELOG・git 履歴・Claude Code メモリに収載済みで、docs への転記対象なしと明示的に決定。Codex が例示した「引き継ぎノートを鵜呑みにせずディスク実在を確認」の教訓は唯一の未収載項目だったため、Claude Code メモリ `feedback_handover_verify_disk_state` に保存した（リポジトリ docs ではなく Claude Code の運用知のため）。所在が不確かだった2点（feat-019 の Blender 4.5.5、feat-020 の素通しバイト比較）は grep で案件ドキュメント・ルート README への収載を確認済み
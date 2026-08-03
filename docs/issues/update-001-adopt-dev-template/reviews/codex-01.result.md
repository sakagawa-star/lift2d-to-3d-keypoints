**指摘事項**

- **高**: `.claude/handovers/` の追跡状態の前提が誤っています。  
  [README.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/README.md:82>) と [design.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/design.md:272>) は「handovers は未追跡なので ignore 追加のみ」としていますが、`git ls-files .claude/handovers` で14件が tracked でした。このままだと `.gitignore` 追加後も既存 handover は追跡され続け、[CLAUDE.md](</home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:350>) の「含めない」ルールとも矛盾します。  
  **修正提案**: G1 に tracked handovers の扱いを追加してください。基本は、永続化すべき情報を docs 側へ移したうえで `git rm --cached $(git ls-files .claude/handovers)` を実施し、検証項目も `.claude/settings.local.json` だけでなく handovers の staged deletion / ignored 状態まで確認する形に直すべきです。

- **高**: bug で `requirements.md` / `design.md` を変更した場合の「実装時に読む文書」が更新計画から漏れています。  
  設計では保存対象化 [design.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/design.md:45>) とレビュー対象化 [design.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/design.md:169>) までは入っていますが、実装サブエージェントの必読順は現行のまま「不具合修正は `investigation.md`」だけです [CLAUDE.md](</home/sakagawa/git/lift2d-to-3d-keypoints/CLAUDE.md:310>)。このままだと、変更済みの要求・設計を読まずに実装する運用が残ります。  
  **修正提案**: C9 として CLAUDE.md の「必読ドキュメントと読む順序」を、`不具合修正は investigation.md と、変更した場合は関連する requirements.md / design.md も必読` に更新する計画を README/design に追加してください。

- **中**: `docs/REVIEW_CRITERIA.md` の全置換が外部作業ツリー依存で、design.md 単体の自己完結性が弱いです。  
  [design.md](</home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/update-001-adopt-dev-template/design.md:247>) は `cp` で外部パスから全置換するとしていますが、置換後全文は design.md にありません。現在は外部 repo が指定コミットでクリーンでしたが、設計書だけで再現可能という条件からは外れます。  
  **修正提案**: 置換後の `REVIEW_CRITERIA.md` 全文を design.md に埋め込むか、少なくとも `git -C /home/sakagawa/git/DEV_TEMPLATE show 715db11e1e9025a2818ce687a86552f76d25ab54:template/docs/REVIEW_CRITERIA.md > docs/REVIEW_CRITERIA.md` のようにコミット固定の取得手順へ変更してください。

低重要度の指摘はありません。

---

## Claude Code の対応方針（2026-08-03）

- **高-1（handovers 追跡状態）**: 事実確認の結果、指摘どおり14件（2026-07-02 以前）が追跡済みだった。G1 に `git rm -r --cached .claude/handovers` と検証項目を追加。docs への転記は行わない（handovers はセッション引き継ぎ用の一時ノートであり、追跡解除後もワークツリーと git 履歴に残るため情報喪失はない）——README に理由を明記
- **高-2（必読ドキュメント）**: C9 として採用。テンプレート側に同一の改訂が存在しており（差分選別時の見落とし）、README の選別表と design.md に追加
- **中（REVIEW_CRITERIA 全置換の自己完結性）**: 提案どおりコミット固定の `git show 715db11...:template/docs/REVIEW_CRITERIA.md` による取得に変更。検証コマンドも同形式に変更
<!-- メタ: 2026-08-09 / 対象: feat-029 requirements.md + design.md / session id: 019fe4ae-1cb8-7881-96dc-45c25e014ae9 / 再レビュー(2回目) -->

前回指摘した YAML 数値キーの `ValueError` 未捕捉問題は解消済みです。  
[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:79) で `ArgumentTypeError / ValueError / TypeError` 捕捉が明記され、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:194) の T10 に `fps: abc` 等の未捕捉クラッシュ防止テストも追加されています。

**中**

1. `parse_args_with_config` の戻り値と責務が設計内でまだ揺れています。  
   [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:91) と [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:162) は `Namespace` 返却、[design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:181) も戻り値 `Namespace` 検証を前提にしています。一方で [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:165) は `(args, parser)` 返却を許容しており、さらに [design.md](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-029-render-fps-yaml-config/design.md:117) は既存 `main()` の `parser.error` 検証を維持するとしています。実装者が tuple 返却を選ぶと、テスト設計と公開インターフェースが割れます。  
   修正提案: `parse_args_with_config` は `argparse.Namespace` のみ返す、と固定してください。`main()` 側で同じ `parser` を使いたいなら、設計を `parse_args_with_config(argv=None, parser: argparse.ArgumentParser | None = None) -> argparse.Namespace` に統一し、tuple 返却許容の記述を削除するのが安全です。

**高・低**

高はありません。低は今回の基準では指摘しません。

---

**Claude Code の対応方針**（再レビュー(2回目)、指摘: 高0・中1・低0（前回の高1は解消確認））: parse_args_with_config の戻り値を Namespace のみに固定、parser 注入のシグネチャに統一

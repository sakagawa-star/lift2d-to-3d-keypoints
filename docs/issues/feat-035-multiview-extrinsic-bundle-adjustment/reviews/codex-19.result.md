再レビューしました。前回の2件は主修正としては解消しています。FR-005入力、design §2.6 TOML対象、MVPの「8台必須でない」方針は整合しました。

**高**
- 追加された手動プロット代替経路が、`--init-cameras` に昇格してよい条件を閉じていません。[requirements.md:53](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:53) と [roadmap.md:60](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:60) は「`refine_extrinsics.py` の結果を追加」とだけ書いており、`refine_extrinsics.py` 側で受理された結果だけを許すのか、最終レンダvs実写ずれ≤5pxを確認するのかが未定義です。これだと Stage R 不受理を手動経路で迂回して、品質未確認ポーズを信頼初期カメラとしてBA/TOMLに入れられます。
  - 修正提案: 代替経路は「`refine_extrinsics.py` が accepted を返したカメラのみ」「同じ測定法でレンダvs実写ずれ中央値≤5pxを確認済み」「不受理/失敗/未確認のTOMLセクションは `--init-cameras` に追加禁止」と明記する。

**中**
- 手動プロット許容の追記と、既存の手動基準点禁止が衝突しています。[requirements.md:114](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/requirements.md:114) と [design.md:259](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/design.md:259) は「全用途で不使用」と読めますが、FR-009代替経路では手動2D/3D点を使った外部精緻化結果を初期ポーズとして使います。
  - 修正提案: 禁止範囲を「本スクリプトの自動経路・BA拘束・評価では手動点を使わない」に限定し、FR-009代替経路で外部生成された受理済みTOMLを `--init-cameras` として使う例外を明記する。

- MVPを部分TOML許容に変えた一方で、手動確認がまだ「8台分」前提です。[roadmap.md:81](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:81) と [roadmap.md:90](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-035-multiview-extrinsic-bundle-adjustment/roadmap.md:90) は、Stage R不受理カメラを除外した合法なMVPでも8台比較を要求してしまいます。
  - 修正提案: 「出力TOMLに含まれるポーズ確定カメラ分のレンダvs実写比較 + 除外カメラ一覧」に変更する。8台比較は、代替経路で全台確定した場合のみ実施とする。

**低**
- なし。
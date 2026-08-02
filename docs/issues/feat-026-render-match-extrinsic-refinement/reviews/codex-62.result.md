**高**
- フェーズ3の実施条件が、§5 の見出し文と §5.3 で矛盾しています。§5 冒頭では「前フェーズ不成立なら打ち切り」とありますが、§5.3 では「フェーズ2で LoFTR が M4 判定を満たさない」場合に MASt3R のフェーズ3を実施するとしています。ここは実装者の読み方次第で、LoFTR 不成立時に即 No-Go にするか、MASt3R を試すかが分かれ、最終 Go/No-Go が変わり得ます。  
  根拠: [criteria_m4_2.md:44](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria_m4_2.md:44), [criteria_m4_2.md:67](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m4_cage/criteria_m4_2.md:67)。  
  修正提案: §5 に明示的な遷移表を追加し、「前フェーズ不成立なら打ち切り」は phase0/phase1 の前提不成立に限定する。例: phase0 不成立→中断、phase1 で LoFTR 不適格かつ MASt3R 不適格→No-Go、LoFTR 不適格かつ MASt3R 適格→phase3、LoFTR phase2 Go→Goで終了、LoFTR phase2 未達かつ MASt3R 適格→phase3、phase3 Go/No-Goで最終判定。

**中/低**
- 致命的な追加指摘なし。

M4 の §6.1/§6.2/§6.4/§7 の数値継承、M3b の matcher 仕様・1280x720 固定・LoFTR 採用規則の3チェーン版については、上記以外に合否を動かす漏れは見つけていません。
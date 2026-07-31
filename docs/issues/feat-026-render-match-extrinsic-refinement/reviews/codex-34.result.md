**高:** なし。

**中:** 条件3と条件5の個別判定が、非受理系列で正しく落ちない場合があります。  
[m3_1_sampling.py:227](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_1_sampling.py:227) で `not s["accepted"]` の場合に `continue` しているため、その系列が M3 成立済みでも [m3_1_sampling.py:236](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_1_sampling.py:236) 以降の無退行チェックに到達しません。criteria §3 条件3は「M3で成立していた12系列が全て受理される」なので、M3成立系列が非受理なら `reg_ok=False` にすべきです。  
同じ `continue` により、非受理だが `f2>=0.2` の系列も単峰性チェックから漏れます。総合 Go は `acc_ok`/`same_ok` で No-Go になる可能性が高いですが、5条件の個別判定と失敗モード分類が criteria とズレます。

修正提案: `m3_established` と `f2` 判定を `continue` より前に移動し、少なくとも以下を入れてください。

```python
m3s = m3_results[name]["series"].get(sname)
m3_established = m3s is not None and m3s.get("established")

if "f2" in s and s["f2"] >= F2_BIMODAL:
    uni_ok = False

if not s["accepted"]:
    acc_ok = False
    if m3_established:
        reg_ok = False
    continue
```

**低:** なし。

それ以外は、`N_S=20`、`r_pos=2cm`、`r_ang=0.3°`、`f_c>=0.7`、全15系列の p95 照合、seed の `seed_series + j`、`m3_run.run_iteration_real` の流用はいずれも criteria と整合しています。構文チェックも通っています。
---

## Claude Code の対応方針（2026-07-31）

- 対象: m3_1_sampling.py 実行前レビュー / session id: 019fb6c9-967d-7020-b17f-c648607c8892
- 中1（非受理系列が無退行・単峰性チェックから漏れる）: 提案どおり m3_established と f2 判定を continue の前に移動し、M3 成立系列の非受理を reg_ok=False に反映

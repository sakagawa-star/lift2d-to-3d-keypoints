# feat-028 検証記録: filter_npz.py と filter_c3d.py（feat-020）の数値等価性

実施日: 2026-08-07
種別: 測定・検証記録（事前 criteria なしのアドホック比較。Go/No-Go 判定ではない）
発端: ユーザーが2経路の出力 C3D を比較し「内容が異なるのは何故か」と質問したことによる調査

## 目的

新規実装 `phase4/filter_npz.py`（feat-028、NPZ→NPZ 平滑化）が、実績のある
`phase4/filter_c3d.py`（feat-020、C3D→C3D 平滑化）と数値的に等価であることを、
同一入力データからの2経路の最終出力（C3D）の直接比較で確認する。

## 手順

入力: `phase4/data/session001_world_22pt.npz`（194,279フレーム、22関節、float32、
NaNフレーム340）。実行はすべて phase4 ディレクトリで uv 経由（ユーザー実施）。

```
# 経路A（旧: C3D経由で平滑化）
uv run python npz_to_c3d.py data/session001_world_22pt.npz
uv run python filter_c3d.py data/session001_world_22pt.c3d
# → data/session001_world_22pt_filtered.c3d（.bak に退避）

# 経路B（新: NPZ段階で平滑化してからC3D変換）
uv run python filter_npz.py data/session001_world_22pt.npz --fps 30
uv run python npz_to_c3d.py data/session001_world_22pt_filtered.npz
# → data/session001_world_22pt_filtered.c3d
```

パラメータは両経路とも既定値（カットオフ 6.0 Hz、max-gap 10、レート/fps 30 Hz）。

比較は同梱の `compare_c3d.py` で実施（リポジトリルートから
`uv run --project phase4 python docs/issues/feat-028-npz-temporal-filter/experiments/c3d-npz-equivalence/compare_c3d.py`。
スクリプト内のパスは経路Aの出力を `session001_world_22pt_filtered.c3d.bak`、
経路Bの出力を `session001_world_22pt_filtered.c3d` として参照している）。

## 結果

| 項目 | 結果 |
|---|---|
| フレーム数・point rate・ラベル | 完全一致（194,279 / 30.0 Hz / 22関節） |
| 無効サンプルの位置（residual<0 マスク） | 完全一致（有効 4,266,658 = 194,279×22 − 340×22） |
| 有効サンプルの座標差 | 平均 0.000034 mm、中央値 0.000008 mm、最大 0.000173 mm |
| 0.1 mm 超の差があるサンプル | 0 件 / 4,266,658 |
| 最大差の位置 | フレーム 145469・LShoulder（差 0.17 µm） |

## 解釈

- Butterworth filtfilt は線形変換であり、軸置換（world→C3D raw）と mm 換算はフィルタと
  可換。したがって2経路は数学的には同一で、差が出る要因は **float32 への丸めが入る場所の
  違いのみ**（経路Aは mm 値を float32 保存してからフィルタ、経路Bは m でフィルタしてから
  float32 保存→mm換算）。
- float32 の相対精度 ≈1e-7 × 座標値スケール（数百〜数千 mm）= 丸め1回あたり ≈1e-4 mm。
  観測された最大差 0.00017 mm は理論値と整合する。
- 入力データのジッター（フレーム間変位 平均 11.8 mm）に対し差は5桁以上小さく、
  下流（レンダリング・C3D 取り込み）への影響は無視できる。

## 結論

`filter_npz.py` は `filter_c3d.py` と数値的に等価（差は float32 丸め誤差のみ、最大 0.17 µm）。
本記録は feat-028 手動テスト（フロー ステップ7）の合格エビデンスの一部として扱える。

## 再現に必要なファイル

- 入力 NPZ: `phase4/data/session001_world_22pt.npz`（git 管理外・ローカル実データ）
- 比較スクリプト: 本フォルダの `compare_c3d.py`

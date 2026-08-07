"""2経路のC3D出力を数値比較する"""
import numpy as np
import c3d

def load(path):
    with open(path, "rb") as f:
        r = c3d.Reader(f)
        labels = [l.strip() for l in r.point_labels]
        rate = float(r.point_rate)
        pts, res = [], []
        for _n, p, _a in r.read_frames():
            pts.append(p[:, :3].astype(np.float64))
            res.append(p[:, 3].astype(np.float64))
    return np.stack(pts), np.stack(res), labels, rate

pa, ra, la, ratea = load("phase4/data/session001_world_22pt_filtered.c3d.bak")  # 旧: C3D経由
pb, rb, lb, rateb = load("phase4/data/session001_world_22pt_filtered.c3d")      # 新: NPZ経由
print("旧(A): shape", pa.shape, "rate", ratea, "| 新(B): shape", pb.shape, "rate", rateb)
print("ラベル一致:", la == lb)
va = ra >= 0; vb = rb >= 0
print("有効サンプル数 A:", int(va.sum()), " B:", int(vb.sum()), " 無効マスク一致:", bool(np.array_equal(va, vb)))
both = va & vb
d = np.linalg.norm(pa - pb, axis=2)[both]  # mm
print(f"有効サンプルの座標差 [mm]: 平均={d.mean():.6f} 最大={d.max():.6f} 中央値={np.median(d):.6f}")
print(f"差が 0.1mm 超のサンプル: {int((d > 0.1).sum())} / {d.size}")
print(f"差が 1mm 超のサンプル: {int((d > 1.0).sum())} / {d.size}")
# 最大差の場所
idx = np.unravel_index(np.argmax(np.where(both, np.linalg.norm(pa - pb, axis=2), -1)), both.shape)
print("最大差の位置: フレーム", idx[0], "関節", la[idx[1]], " A=", pa[idx], " B=", pb[idx])

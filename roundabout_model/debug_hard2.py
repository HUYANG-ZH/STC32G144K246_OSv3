# -*- coding: utf-8 -*-
"""Examine hard val non samples vs val rb in full feature space."""
import numpy as np
import train_model as tm

X, y, f = tm.load_all()
rb = ["1.txt", "2.txt", "3.txt", "4.txt", "5.txt", "6.txt", "7.txt", "8.txt"]
non = ["1.txt", "2.txt", "3.txt"]
tr_keys = set(rb[:5] + ["non_" + v for v in non[:1]])
va_keys = set([rb[5]] + ["non_" + non[1]])
te_keys = set(rb[6:] + ["non_" + non[2]])
fkey = np.array([("non_" + v if y[i] == 0 else v) for i, v in enumerate(f)])
m_va = np.isin(fkey, list(va_keys))
m_te = np.isin(fkey, list(te_keys))
Xva, yva = X[m_va], y[m_va]
Xte, yte = X[m_te], y[m_te]

Xv0 = Xva[yva == 0]
hard = Xv0[(Xv0[:, 4] == 7777) & (Xv0[:, 0] >= 45)]  # TOF=7777 with high CH1
print(f"val non TOF=7777 & CH1>=45: n={len(hard)}")
print("feature stats (CH1 CH2 CH3 CH4 TOF):")
for i, c in enumerate(["CH1", "CH2", "CH3", "CH4", "TOF"]):
    print(f"  {c}: min={hard[:, i].min():6.1f} p25={np.percentile(hard[:, i], 25):6.1f} "
          f"p50={np.percentile(hard[:, i], 50):6.1f} p75={np.percentile(hard[:, i], 75):6.1f} max={hard[:, i].max():6.1f}")

Xv1 = Xva[yva == 1]
print(f"\nval rb: n={len(Xv1)}")
for i, c in enumerate(["CH1", "CH2", "CH3", "CH4", "TOF"]):
    print(f"  {c}: min={Xv1[:, i].min():6.1f} p25={np.percentile(Xv1[:, i], 25):6.1f} "
          f"p50={np.percentile(Xv1[:, i], 50):6.1f} p75={np.percentile(Xv1[:, i], 75):6.1f} max={Xv1[:, i].max():6.1f}")

# 找区分度: CH2+CH3, CH1+CH4, CH1-CH4
print("\n=== derived features: hard-non vs rb ===")
for nm, s in [("hard-non", hard), ("rb", Xv1)]:
    print(f"  {nm:9s}: n={len(s)} "
          f"CH1+CH4 p05={np.percentile(s[:,0]+s[:,3],5):6.1f} p50={np.percentile(s[:,0]+s[:,3],50):6.1f} "
          f"CH2+CH3 p05={np.percentile(s[:,1]+s[:,2],5):6.1f} p50={np.percentile(s[:,1]+s[:,2],50):6.1f} "
          f"CH2 p50={np.percentile(s[:,1],50):6.1f} CH3 p50={np.percentile(s[:,2],50):6.1f}")

# 检查这些 hard 样本在文件中的位置(时间连续段?)
# 先重新加载 non file2 带行号
rows = []
with open("data/non_roundabout/2.txt") as fh:
    for ln, line in enumerate(fh):
        p = line.strip().split(",")
        if len(p) == 5:
            try:
                v = [float(x) for x in p]
            except ValueError:
                continue
            rows.append((ln, v))
arr = np.array([r[1] for r in rows])
is_hard = (arr[:, 4] == 7777) & (arr[:, 0] >= 45)
idxs = np.where(is_hard)[0]
print(f"\nnon file2 hard row indices: {len(idxs)}, first={idxs[:10]}, last={idxs[-10:]}")
# 连续段统计
seg = np.split(idxs, np.where(np.diff(idxs) > 1)[0] + 1)
print("segments (len>=10):", [len(s) for s in seg if len(s) >= 10])
print("segment bounds:", [(s[0], s[-1]) for s in seg if len(s) >= 10][:10])

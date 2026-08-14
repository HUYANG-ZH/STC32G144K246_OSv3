# -*- coding: utf-8 -*-
"""Per-file TOF structure analysis: files 7/8 rb have real TOF readings?"""
import numpy as np
import train_model as tm

X, y, f = tm.load_all()
print("per-file TOF stats:")
for sub, lab in [("roundabout", 1), ("non_roundabout", 0)]:
    for fn in sorted(set(f[y == lab])):
        m = (y == lab) & (f == fn)
        Xs = X[m]
        tof = Xs[:, 4]
        n7777 = int(np.sum(tof == 7777))
        nlt500 = int(np.sum(tof < 500))
        n500_2000 = int(np.sum((tof >= 500) & (tof <= 2000)))
        ngt2000 = int(np.sum(tof > 2000))
        print(f"  {sub:14s} {fn:6s}: n={len(Xs):5d} 7777={n7777:5d} <500={nlt500:4d} "
              f"500-2000={n500_2000:4d} >2000={ngt2000:4d} tof_min={tof.min():.0f}")

# rb 文件 7/8 的 TOF<1000 样本长什么样
print("\nrb files 7/8 with TOF<1000:")
m = (y == 1) & np.isin(f, ["7.txt", "8.txt"]) & (X[:, 4] < 1000)
print(X[m][:15])
print("count:", m.sum())

# rb 文件 1-6 TOF 分布
print("\nrb files 1-6 TOF unique:", np.unique(X[(y == 1) & np.isin(f, ["1.txt", "2.txt", "3.txt", "4.txt", "5.txt", "6.txt"]), 4]))

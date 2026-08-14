# -*- coding: utf-8 -*-
"""Analyze class overlap in feature space, esp. the hard val non samples."""
import numpy as np
import train_model as tm

X, y, f = tm.load_all()
rb = ["1.txt", "2.txt", "3.txt", "4.txt", "5.txt", "6.txt", "7.txt", "8.txt"]
non = ["1.txt", "2.txt", "3.txt"]
tr_keys = set(rb[:5] + ["non_" + v for v in non[:1]])
va_keys = set([rb[5]] + ["non_" + non[1]])
te_keys = set(rb[6:] + ["non_" + non[2]])
fkey = np.array([("non_" + v if y[i] == 0 else v) for i, v in enumerate(f)])
m_tr = np.isin(fkey, list(tr_keys))
m_va = np.isin(fkey, list(va_keys))
m_te = np.isin(fkey, list(te_keys))
Xtr, ytr = X[m_tr], y[m_tr]
Xva, yva = X[m_va], y[m_va]
Xte, yte = X[m_te], y[m_te]

print("=== val non file2 structure ===")
Xv0 = Xva[yva == 0]
# easy vs hard: TOF=7777 & CH1<45 => easy
easy = (Xv0[:, 4] == 7777) & (Xv0[:, 0] < 45)
hard = ~easy
print(f"easy(TOF7777&CH1<45): {easy.sum()}, hard: {hard.sum()}")
for nm, m in [("easy", easy), ("hard", hard)]:
    s = Xv0[m]
    print(f"  {nm}: n={len(s)} CH1={s[:,0].mean():.1f} CH2={s[:,1].mean():.1f} "
          f"CH3={s[:,2].mean():.1f} CH4={s[:,3].mean():.1f} TOF_uniq={np.unique(s[:,4])[:8]}")

print("\n=== val roundabout (file6) ===")
Xv1 = Xva[yva == 1]
print(f"n={len(Xv1)} CH1={Xv1[:,0].mean():.1f} CH2={Xv1[:,1].mean():.1f} "
      f"CH3={Xv1[:,2].mean():.1f} CH4={Xv1[:,3].mean():.1f} TOF_uniq={np.unique(Xv1[:,4])}")

print("\n=== train non file1 ===")
Xt0 = Xtr[ytr == 0]
print(f"n={len(Xt0)} CH1={Xt0[:,0].mean():.1f} CH2={Xt0[:,1].mean():.1f} "
      f"CH3={Xt0[:,2].mean():.1f} CH4={Xt0[:,3].mean():.1f} TOF_uniq={np.unique(Xt0[:,4])[:10]}")
print(f"  TOF7777 count: {(Xt0[:,4]==7777).sum()}, TOF<500: {(Xt0[:,4]<500).sum()}")

print("\n=== test non file3 ===")
Xe0 = Xte[yte == 0]
print(f"n={len(Xe0)} CH1={Xe0[:,0].mean():.1f} CH2={Xe0[:,1].mean():.1f} "
      f"CH3={Xe0[:,2].mean():.1f} CH4={Xe0[:,3].mean():.1f} TOF_uniq={np.unique(Xe0[:,4])[:10]}")
print(f"  TOF7777 count: {(Xe0[:,4]==7777).sum()}, TOF<500: {(Xe0[:,4]<500).sum()}")
print("=== test roundabout (files 7,8) ===")
Xe1 = Xte[yte == 1]
print(f"n={len(Xe1)} CH1={Xe1[:,0].mean():.1f} CH2={Xe1[:,1].mean():.1f} "
      f"CH3={Xe1[:,2].mean():.1f} CH4={Xe1[:,3].mean():.1f} TOF_uniq={np.unique(Xe1[:,4])}")

# hard non vs rb boundary: CH1+CH4
print("\n=== boundary analysis CH1+CH4 (TOF=7777) ===")
for nm, s in [("val-hard-non", Xv0[hard]), ("val-easy-non", Xv0[easy]), ("val-rb", Xv1), ("test-non", Xe0), ("test-rb", Xe1)]:
    ssum = s[:, 0] + s[:, 3]
    print(f"  {nm:14s}: n={len(s):5d} sum_min={ssum.min():6.1f} p01={np.percentile(ssum,1):6.1f} "
          f"p05={np.percentile(ssum,5):6.1f} p50={np.percentile(ssum,50):6.1f} max={ssum.max():6.1f}")

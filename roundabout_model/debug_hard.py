# -*- coding: utf-8 -*-
"""Debug: why does GBC underperform LR? Check train AUC, and the hard non-roundabout samples."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.metrics import roc_auc_score
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

gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=60,
                                 max_depth=5, min_samples_leaf=15, subsample=0.9, random_state=42)
gbc.fit(Xtr, ytr)
for name, Xs, ys in [("train", Xtr, ytr), ("val", Xva, yva), ("test", Xte, yte)]:
    s = gbc.decision_function(Xs)
    print(f"GBC {name}: AUC={roc_auc_score(ys, s):.4f}")

# 哪些 val 非环岛样本分数高?
sv = gbc.decision_function(Xva)
hard = (yva == 0) & (sv > 1.0)
print(f"\nval non-roundabout hard samples: {hard.sum()} / {(yva==0).sum()}")
print("their TOF distribution:", np.unique(Xva[hard, 4], return_counts=True))
print("their CH1/CH4:")
print(Xva[hard][:10])
print("\nval non-roundabout easy samples TOF:", np.unique(Xva[(yva==0)&(~hard), 4], return_counts=True))

# 检查: TOF 在非环岛 val 的分布
print("\nval non TOF stats: min", Xva[yva==0, 4].min(), "unique:", np.unique(Xva[yva==0, 4])[:20])
# 环岛 val 的 TOF
print("val rb TOF unique:", np.unique(Xva[yva==1, 4])[:20])

# LR 在这些 hard 样本上?
from sklearn.linear_model import LogisticRegression
lr = LogisticRegression(max_iter=2000)
lr.fit(Xtr, ytr)
slr = lr.decision_function(Xva)
print("\nLR val score for hard samples:", slr[hard].min(), slr[hard].max())
print("LR val score for easy non:", slr[(yva==0)&(~hard)].min(), slr[(yva==0)&(~hard)].max())
print("LR val score for rb:", slr[yva==1].min(), slr[yva==1].max())

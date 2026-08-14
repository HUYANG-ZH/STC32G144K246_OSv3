# -*- coding: utf-8 -*-
"""Debug: val score distribution of sklearn GBC on the real split."""
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
sv = gbc.decision_function(Xva)
print("val score range:", sv.min(), sv.max())
for lab in [0, 1]:
    s = sv[yva == lab]
    print(f"  class {lab}: n={len(s)} mean={s.mean():.4f} min={s.min():.4f} max={s.max():.4f} "
          f"p50={np.percentile(s, 50):.4f} p01={np.percentile(s, 1):.4f}")
print("val AUC:", roc_auc_score(yva, sv))
for tau in [0.5, 1.0, 1.5, 2.0, 2.5]:
    pred = (sv >= tau).astype(int)
    mm = tm.metrics(yva, pred)
    print(f"  tau={tau}: recall={mm['recall']*100:.2f}% fpr={mm['fpr']*100:.4f}%")

# -*- coding: utf-8 -*-
"""Check n=200 GBC: score distributions, threshold search, recall/FPR on all sets."""
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

gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=200,
                                 max_depth=5, min_samples_leaf=15, random_state=42)
gbc.fit(Xtr, ytr)

for name, Xs, ys in [("train", Xtr, ytr), ("val", Xva, yva), ("test", Xte, yte)]:
    s = gbc.decision_function(Xs)
    print(f"{name}: AUC={roc_auc_score(ys, s):.4f}")
    for lab in [0, 1]:
        ss = s[ys == lab]
        print(f"   class {lab}: n={len(ss)} min={ss.min():.3f} max={ss.max():.3f} "
              f"p01={np.percentile(ss, 1):.3f} p05={np.percentile(ss, 5):.3f} p50={np.percentile(ss, 50):.3f}")

# threshold scan on val
sv = gbc.decision_function(Xva)
print("\nval threshold scan (recall/fpr):")
for tau in np.arange(-1.0, 4.5, 0.25):
    pred = (sv >= tau).astype(int)
    mm = tm.metrics(yva, pred)
    print(f"  tau={tau:5.2f}: recall={mm['recall']*100:6.2f}%  fpr={mm['fpr']*100:6.4f}%  (fp={mm['fp']})")

st = gbc.decision_function(Xte)
print("\ntest threshold scan:")
for tau in np.arange(-1.0, 4.5, 0.25):
    pred = (st >= tau).astype(int)
    mm = tm.metrics(yte, pred)
    print(f"  tau={tau:5.2f}: recall={mm['recall']*100:6.2f}%  fpr={mm['fpr']*100:6.4f}%  (fp={mm['fp']})")

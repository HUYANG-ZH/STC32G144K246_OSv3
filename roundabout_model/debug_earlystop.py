# -*- coding: utf-8 -*-
"""Debug early stopping: val logloss vs n_estimators for GBC."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
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


def val_loss(gbc, Xva, yva):
    raw = gbc.decision_function(Xva)
    p = 1.0 / (1.0 + np.exp(-np.clip(raw, -30, 30)))
    eps = 1e-12
    return -np.mean(yva * np.log(p + eps) + (1 - yva) * np.log(1 - p + eps))


# fixed fits at various n_estimators
for ne in [29, 60, 100, 150, 200, 300]:
    gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=ne,
                                     max_depth=5, min_samples_leaf=15, random_state=42)
    gbc.fit(Xtr, ytr)
    print(f"n={ne}: val_logloss={val_loss(gbc, Xva, yva):.6f}")

# warm-start incremental (like train_sklearn)
print("\n-- warm start incremental --")
gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=1,
                                 max_depth=5, min_samples_leaf=15, random_state=42)
for i in range(80):
    gbc.n_estimators = i + 1
    gbc.fit(Xtr, ytr)
    if i + 1 in [5, 10, 15, 20, 25, 29, 30, 40, 50, 60, 80]:
        print(f"n={i + 1}: val_logloss={val_loss(gbc, Xva, yva):.6f}")

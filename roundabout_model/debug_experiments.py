# -*- coding: utf-8 -*-
"""Experiment: model config vs val/test AUC. Check GBC variants and HistGB."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier, HistGradientBoostingClassifier
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

cfgs = [
    ("GBC d5 n200 lr0.05", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=200, max_depth=5, min_samples_leaf=15, random_state=42)),
    ("GBC d5 n200 lr0.05 sub0.9", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=200, max_depth=5, min_samples_leaf=15, subsample=0.9, random_state=42)),
    ("GBC d6 n300 lr0.03", GradientBoostingClassifier(loss="log_loss", learning_rate=0.03, n_estimators=300, max_depth=6, min_samples_leaf=10, random_state=42)),
    ("HistGB d=None n300 lr0.05", HistGradientBoostingClassifier(max_depth=None, max_iter=300, learning_rate=0.05, min_samples_leaf=15, random_state=42)),
    ("HistGB d5 n300 lr0.05", HistGradientBoostingClassifier(max_depth=5, max_iter=300, learning_rate=0.05, min_samples_leaf=15, random_state=42)),
]
for name, clf in cfgs:
    clf.fit(Xtr, ytr)
    line = [name]
    for tag, Xs, ys in [("tr", Xtr, ytr), ("va", Xva, yva), ("te", Xte, yte)]:
        s = clf.decision_function(Xs)
        line.append(f"{tag}={roc_auc_score(ys, s):.4f}")
    print(" ".join(line))

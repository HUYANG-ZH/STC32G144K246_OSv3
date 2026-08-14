# -*- coding: utf-8 -*-
"""Operating-point experiment: find model + threshold meeting recall>=99.5% & FPR<=0.2% on val."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
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


def scan(scores, y, taus):
    out = []
    for tau in taus:
        pred = (scores >= tau).astype(int)
        mm = tm.metrics(y, pred)
        out.append((tau, mm))
    return out


# best operating point on val: FPR<=0.15% (margin) and max recall
def best_op(scores, y, fpr_max=0.0015):
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


models = [
    ("LR", LogisticRegression(max_iter=5000, C=1.0)),
    ("LR C=0.1", LogisticRegression(max_iter=5000, C=0.1)),
    ("GBC d3 n500 lr0.05", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=500, max_depth=3, min_samples_leaf=20, random_state=42)),
    ("GBC d4 n300 lr0.05", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=300, max_depth=4, min_samples_leaf=20, random_state=42)),
    ("GBC d5 n200 lr0.05", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=200, max_depth=5, min_samples_leaf=15, random_state=42)),
    ("GBC d6 n150 lr0.05", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=150, max_depth=6, min_samples_leaf=15, random_state=42)),
]
for name, clf in models:
    clf.fit(Xtr, ytr)
    sv = clf.decision_function(Xva)
    st = clf.decision_function(Xte)
    opv = best_op(sv, yva)
    if opv is None:
        print(f"{name}: NO operating point with FPR<=0.15% on val")
        continue
    tau, mmv = opv
    mmt = tm.metrics(yte, (st >= tau).astype(int))
    print(f"{name}: val AUC={roc_auc_score(yva, sv):.4f} test AUC={roc_auc_score(yte, st):.4f}")
    print(f"   tau={tau:7.3f}  val: recall={mmv['recall']*100:.4f}% fpr={mmv['fpr']*100:.4f}% "
          f"(fp={mmv['fp']})  test: recall={mmt['recall']*100:.4f}% fpr={mmt['fpr']*100:.4f}% (fp={mmt['fp']})")

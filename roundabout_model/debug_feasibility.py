# -*- coding: utf-8 -*-
"""Feasibility check: can ANY model hit recall>=99.5% & FPR<=0.2% on the file-grouped test set?"""
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


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    f14 = c1 + c4
    f23 = c2 + c3
    f124 = c1 + c2 + c4
    f1234 = c1 + c2 + c3 + c4
    f1_4 = c1 * c4
    tof7777 = (tof > 7000).astype(float)
    tof500 = (tof > 500).astype(float)
    tofclip = np.minimum(tof, 2000.0)
    return np.column_stack([X, f14, f23, f124, f1234, f1_4, tof7777, tof500, tofclip])


Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)

gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=300,
                                 max_depth=3, min_samples_leaf=20, random_state=42)
gbc.fit(Ztr, ytr)
sv = gbc.decision_function(Zva)
st = gbc.decision_function(Zte)

# ROC on test
print("test AUC:", roc_auc_score(yte, st))
# 扫描 test 操作点
print("\ntest operating scan (recall / fpr):")
for tau in np.linspace(st.min(), st.max(), 400):
    mm = tm.metrics(yte, (st >= tau).astype(int))
    if 0.99 <= mm["recall"] <= 0.999 or mm["fpr"] <= 0.005:
        if mm["fpr"] <= 0.005 or mm["recall"] >= 0.99:
            print(f"  tau={tau:8.4f}: recall={mm['recall']*100:6.3f}% fpr={mm['fpr']*100:6.4f}% "
                  f"(fp={mm['fp']} fn={mm['fn']})")

# val 操作点
print("\nval operating scan:")
for tau in np.linspace(sv.min(), sv.max(), 400):
    mm = tm.metrics(yva, (sv >= tau).astype(int))
    if mm["fpr"] <= 0.005 or mm["recall"] >= 0.99:
        print(f"  tau={tau:8.4f}: recall={mm['recall']*100:6.3f}% fpr={mm['fpr']*100:6.4f}% "
              f"(fp={mm['fp']} fn={mm['fn']})")

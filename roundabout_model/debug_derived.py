# -*- coding: utf-8 -*-
"""Test derived features (CH1+CH4, CH2+CH3, etc.) for operating-point feasibility."""
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import GradientBoostingClassifier, RandomForestClassifier
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
    """内部派生特征: 输入仍是 5 路, 内部扩展"""
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    f14 = c1 + c4
    f23 = c2 + c3
    f14m = c1 - c4
    f23m = c2 - c3
    f13 = c1 + c3
    f24 = c2 + c4
    f124 = c1 + c2 + c4
    f1234 = c1 + c2 + c3 + c4
    f1_4 = c1 * c4
    f2_3 = c2 * c3
    f1_4_ratio = np.where(c4 > 0.1, c1 / np.maximum(c4, 0.1), 0.0)
    f2_3_ratio = np.where(c3 > 0.1, c2 / np.maximum(c3, 0.1), 0.0)
    tof7777 = (tof > 7000).astype(float)
    tof500 = (tof > 500).astype(float)
    tofclip = np.minimum(tof, 2000.0)
    return np.column_stack([X, f14, f23, f14m, f23m, f13, f24, f124, f1234,
                            f1_4, f2_3, f1_4_ratio, f2_3_ratio, tof7777, tof500, tofclip])


def best_op(scores, y, fpr_max=0.0015):
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)
print("feature count:", Ztr.shape[1])

models = [
    ("LR-raw", LogisticRegression(max_iter=5000, C=1.0), Xtr, Xva, Xte),
    ("LR-deriv", LogisticRegression(max_iter=5000, C=1.0), Ztr, Zva, Zte),
    ("LR-deriv-C01", LogisticRegression(max_iter=5000, C=0.1), Ztr, Zva, Zte),
    ("GBC-d3-deriv", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=300,
                                                max_depth=3, min_samples_leaf=20, random_state=42), Ztr, Zva, Zte),
    ("RF-deriv", RandomForestClassifier(n_estimators=300, max_depth=None, min_samples_leaf=5,
                                        random_state=42, n_jobs=1), Ztr, Zva, Zte),
]
for name, clf, Atr, Ava, Ate in models:
    clf.fit(Atr, ytr)
    sv = clf.decision_function(Ava) if hasattr(clf, "decision_function") else clf.predict_proba(Ava)[:, 1]
    st = clf.decision_function(Ate) if hasattr(clf, "decision_function") else clf.predict_proba(Ate)[:, 1]
    opv = best_op(sv, yva)
    if opv is None:
        print(f"{name}: NO op point FPR<=0.15% on val")
        continue
    tau, mmv = opv
    mmt = tm.metrics(yte, (st >= tau).astype(int))
    print(f"{name}: val AUC={roc_auc_score(yva, sv):.4f} test AUC={roc_auc_score(yte, st):.4f}")
    print(f"   tau={tau:8.4f}  val: recall={mmv['recall']*100:.4f}% fpr={mmv['fpr']*100:.4f}% (fp={mmv['fp']})"
          f"  test: recall={mmt['recall']*100:.4f}% fpr={mmt['fpr']*100:.4f}% (fp={mmt['fp']})")

# -*- coding: utf-8 -*-
"""Sweep on DEDUP data with 13-feature set, larger tree counts."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.metrics import roc_auc_score
import train_model as tm

X, y, f = tm.load_all()
Xr = np.round(X, 1)
ux, idx = np.unique(Xr, axis=0, return_index=True)
Xu, yu = ux, y[idx]
print(f"dedup {len(y)} -> {len(yu)} (rb {yu.sum()}, non {(yu==0).sum()})")


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4 / 100.0, c1 / np.maximum(c4, 0.1), c2 / np.maximum(c3, 0.1),
                            (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


Z = add_feats(Xu)
print("features:", Z.shape[1])


def find_op(scores, y, fpr_max=0.0015, recall_min=0.995):
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and mm["recall"] >= recall_min:
            return tau, mm
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


for name, d, n, lr in [
    ("d4-200", 4, 200, 0.05), ("d4-300", 4, 300, 0.05), ("d4-500", 4, 500, 0.04),
    ("d5-200", 5, 200, 0.05), ("d5-300", 5, 300, 0.04), ("d5-500", 5, 500, 0.03),
    ("d6-200", 6, 200, 0.04), ("d3-500", 3, 500, 0.04),
]:
    gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=lr, n_estimators=n,
                                     max_depth=d, min_samples_leaf=15, random_state=42)
    gbc.fit(Z, yu)
    s = gbc.decision_function(Z)
    auc = roc_auc_score(yu, s)
    op = find_op(s, yu)
    n_nodes = sum(est[0].tree_.node_count for est in gbc.estimators_)
    if op:
        tau, mm = op
        print(f"{name}: AUC={auc:.5f} OP recall={mm['recall']*100:.4f}% fpr={mm['fpr']*100:.4f}% "
              f"(fp={mm['fp']} fn={mm['fn']}) nodes={n_nodes} int8={n_nodes*5}B")
    else:
        print(f"{name}: AUC={auc:.5f} NO OP (best recall at fpr<=0.15% below 99.5%)")

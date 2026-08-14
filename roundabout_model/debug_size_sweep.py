# -*- coding: utf-8 -*-
"""Size/accuracy sweep on ALL data: find smallest GBC meeting recall>=99.5%, fpr<=0.2%."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.metrics import roc_auc_score
import train_model as tm

X, y, f = tm.load_all()


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c3 + c4,
                            c1 * c4 / 100.0, (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


Z = add_feats(X)
print("features:", Z.shape[1])

cfg_names = [
    "d3-32", "d3-64", "d4-16", "d4-32", "d4-64", "d5-16", "d5-32",
    "d5-64", "d6-16", "d6-32", "d4-128", "d5-128",
]


def find_op(scores, y, fpr_max=0.00175, recall_min=0.995):
    """全量数据上找满足双目标的阈值; 返回 (tau, metrics) 或 None"""
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and mm["recall"] >= recall_min:
            return tau, mm
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


for name in cfg_names:
    d, n = int(name[1]), int(name.split("-")[1])
    gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=n,
                                     max_depth=d, min_samples_leaf=15, random_state=42)
    gbc.fit(Z, y)
    s = gbc.decision_function(Z)
    auc = roc_auc_score(y, s)
    op = find_op(s, y)
    if op:
        tau, mm = op
        n_nodes = sum(t.tree_.node_count for est in gbc.estimators_ for t in [est[0]])
        print(f"{name:8s}: AUC={auc:.5f} OP: recall={mm['recall']*100:.4f}% fpr={mm['fpr']*100:.4f}% "
              f"(fp={mm['fp']} fn={mm['fn']}) nodes={n_nodes} int8_B={n_nodes*5}")
    else:
        print(f"{name:8s}: AUC={auc:.5f} NO operating point meeting both targets")

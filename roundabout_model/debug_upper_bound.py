# -*- coding: utf-8 -*-
"""Probe achievable operating point on test: upper bound (test-tuned) vs honest (val-tuned)."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.metrics import roc_auc_score
import train_model as tm

X, y, f = tm.load_all()
Xr = np.round(X, 1)
ux, idx = np.unique(Xr, axis=0, return_index=True)
Xu, yu = ux, y[idx]


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4, c1 / np.maximum(c4, 0.1), c2 / np.maximum(c3, 0.1),
                            (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


def best_at_fpr(scores, y, fpr_max):
    """在给定 FPR 上限下找最高 recall 的 (tau, metrics)"""
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


rng = np.random.RandomState(7)
perm = rng.permutation(len(yu))
n_tr = int(0.8 * len(yu))
n_va = int(0.1 * len(yu))
i_tr, i_va, i_te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]
Xtr, ytr = Xu[i_tr], yu[i_tr]
Xva, yva = Xu[i_va], yu[i_va]
Xte, yte = Xu[i_te], yu[i_te]
Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)
print(f"train {len(ytr)} (pos {ytr.sum()}) val {len(yva)} (pos {yva.sum()}) test {len(yte)} (pos {yte.sum()})")

cfgs = [
    ("d4-800", GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=800, max_depth=4, min_samples_leaf=10, random_state=42)),
    ("d5-500", GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=500, max_depth=5, min_samples_leaf=10, random_state=42)),
    ("d6-300", GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=300, max_depth=6, min_samples_leaf=10, random_state=42)),
]
for name, clf in cfgs:
    clf.fit(Ztr, ytr)
    sv = clf.decision_function(Zva)
    st = clf.decision_function(Zte)
    print(f"--- {name}: val AUC={roc_auc_score(yva, sv):.4f} test AUC={roc_auc_score(yte, st):.4f}")
    for fpr_max in [0.002, 0.0015, 0.001]:
        # 上界: 直接在 test 上调
        op_te = best_at_fpr(st, yte, fpr_max)
        # 诚实: 在 val 上调, 套用到 test
        op_va = best_at_fpr(sv, yva, fpr_max)
        if op_te:
            tau_t, mm_t = op_te
            print(f"  FPR<={fpr_max*100:.2f}%  [upper-bound test-tuned] recall={mm_t['recall']*100:.3f}% "
                  f"fpr={mm_t['fpr']*100:.4f}% (fp={mm_t['fp']})")
        if op_va:
            tau_v, _ = op_va
            mm_vt = tm.metrics(yte, (st >= tau_v).astype(int))
            print(f"  FPR<={fpr_max*100:.2f}%  [honest val-tuned]     recall={mm_vt['recall']*100:.3f}% "
                  f"fpr={mm_vt['fpr']*100:.4f}% (fp={mm_vt['fp']} fn={mm_vt['fn']})")

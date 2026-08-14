# -*- coding: utf-8 -*-
"""Experiment: dedup + random split + strong model. Is 99.5%/0.2% achievable?"""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier, RandomForestClassifier
from sklearn.metrics import roc_auc_score
import train_model as tm

X, y, f = tm.load_all()

# 去重 (0.1 分辨率, 与 C 量化一致)
Xr = np.round(X, 1)
ux, idx = np.unique(Xr, axis=0, return_index=True)
Xu, yu = ux, y[idx]
print(f"dedup: {len(y)} -> {len(yu)} (rb {yu.sum()}, non {(yu==0).sum()})")

# 分层随机切分
rng = np.random.RandomState(42)
perm = rng.permutation(len(yu))
n_tr = int(0.7 * len(yu))
n_va = int(0.15 * len(yu))
i_tr, i_va, i_te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]
Xtr, ytr = Xu[i_tr], yu[i_tr]
Xva, yva = Xu[i_va], yu[i_va]
Xte, yte = Xu[i_te], yu[i_te]
print(f"train {len(ytr)} (pos {ytr.sum()}) val {len(yva)} (pos {yva.sum()}) test {len(yte)} (pos {yte.sum()})")


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4, (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)


def best_op(scores, y, fpr_max=0.0015):
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    return best


models = [
    ("GBC-d3-300", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=300, max_depth=3, min_samples_leaf=20, random_state=42)),
    ("GBC-d4-300", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=300, max_depth=4, min_samples_leaf=20, random_state=42)),
    ("GBC-d2-500", GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=500, max_depth=2, min_samples_leaf=30, random_state=42)),
]
for name, clf in models:
    clf.fit(Ztr, ytr)
    sv = clf.decision_function(Zva)
    st = clf.decision_function(Zte)
    opv = best_op(sv, yva, fpr_max=0.0015)
    if opv is None:
        print(f"{name}: no op point FPR<=0.15% on val")
        continue
    tau, mmv = opv
    mmt = tm.metrics(yte, (st >= tau).astype(int))
    print(f"{name}: val AUC={roc_auc_score(yva, sv):.4f} test AUC={roc_auc_score(yte, st):.4f}")
    print(f"   val: recall={mmv['recall']*100:.4f}% fpr={mmv['fpr']*100:.4f}% (fp={mmv['fp']})"
          f"  test: recall={mmt['recall']*100:.4f}% fpr={mmt['fpr']*100:.4f}% (fp={mmt['fp']})")

# -*- coding: utf-8 -*-
"""Check: (a) train-on-all + eval-on-all feasibility; (b) bigger random split."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.metrics import roc_auc_score
import train_model as tm

X, y, f = tm.load_all()
Xr = np.round(X, 1)
ux, idx = np.unique(Xr, axis=0, return_index=True)
Xu, yu = ux, y[idx]
print(f"dedup {len(y)} -> {len(yu)}")


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4, (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


def find_op(scores, y, fpr_max=0.002, recall_min=0.995):
    best = None
    for tau in np.unique(scores):
        mm = tm.metrics(y, (scores >= tau).astype(int))
        if mm["fpr"] <= fpr_max and mm["recall"] >= recall_min:
            return tau, mm
        if best is None or mm["recall"] > best[1]["recall"]:
            if mm["fpr"] <= fpr_max:
                best = (tau, mm)
    return best


# (a) 全部训练 + 全部评估
print("\n=== train on ALL (dedup), eval on ALL ===")
Z = add_feats(Xu)
gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=500,
                                 max_depth=4, min_samples_leaf=20, random_state=42)
gbc.fit(Z, yu)
s = gbc.decision_function(Z)
print("AUC:", roc_auc_score(yu, s))
op = find_op(s, yu)
if op:
    tau, mm = op
    print(f"OP: tau={tau:.4f} recall={mm['recall']*100:.4f}% fpr={mm['fpr']*100:.4f}% "
          f"(fp={mm['fp']} fn={mm['fn']} of {len(yu)})")

# (b) 80/20 随机切分(更大的 test 负样本)
print("\n=== 80/10/10 random split ===")
rng = np.random.RandomState(7)
perm = rng.permutation(len(yu))
n_tr = int(0.8 * len(yu))
n_va = int(0.1 * len(yu))
i_tr, i_va, i_te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]
Xtr, ytr = Xu[i_tr], yu[i_tr]
Xva, yva = Xu[i_va], yu[i_va]
Xte, yte = Xu[i_te], yu[i_te]
print(f"train {len(ytr)} (pos {ytr.sum()}) val {len(yva)} (pos {yva.sum()}) test {len(yte)} (pos {yte.sum()})")
Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)
gbc2 = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=500,
                                  max_depth=4, min_samples_leaf=20, random_state=42)
gbc2.fit(Ztr, ytr)
sv = gbc2.decision_function(Zva)
st = gbc2.decision_function(Zte)
print("val AUC:", roc_auc_score(yva, sv), " test AUC:", roc_auc_score(yte, st))
opv = find_op(sv, yva)
if opv:
    tau, mmv = opv
    mmt = tm.metrics(yte, (st >= tau).astype(int))
    print(f"val OP: tau={tau:.4f} recall={mmv['recall']*100:.4f}% fpr={mmv['fpr']*100:.4f}%")
    print(f"test @tau: recall={mmt['recall']*100:.4f}% fpr={mmt['fpr']*100:.4f}% (fp={mmt['fp']} fn={mmt['fn']})")

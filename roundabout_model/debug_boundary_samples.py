# -*- coding: utf-8 -*-
"""Examine hard test samples: which non-rb score high, which rb score low."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
import train_model as tm

X, y, f = tm.load_all()
Xr = np.round(X, 1)
ux, idx = np.unique(Xr, axis=0, return_index=True)
Xu, yu = ux, y[idx]
rng = np.random.RandomState(42)
perm = rng.permutation(len(yu))
n_tr = int(0.7 * len(yu))
n_va = int(0.15 * len(yu))
i_tr, i_va, i_te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]
Xtr, ytr = Xu[i_tr], yu[i_tr]
Xva, yva = Xu[i_va], yu[i_va]
Xte, yte = Xu[i_te], yu[i_te]


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4, (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)
gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.05, n_estimators=500,
                                 max_depth=4, min_samples_leaf=20, random_state=42)
gbc.fit(Ztr, ytr)
st = gbc.decision_function(Zte)

# test 非环岛高分的
hi = np.where((yte == 0) & (st > 3.0))[0]
print(f"test non-rb with score>3.0: {len(hi)}")
for i in hi[:20]:
    print(f"  score={st[i]:7.3f}  X={Xte[i]}")

lo = np.where((yte == 1) & (st < 4.0))[0]
print(f"\ntest rb with score<4.0: {len(lo)}")
for i in lo[:20]:
    print(f"  score={st[i]:7.3f}  X={Xte[i]}")

# 高分非环岛的 TOF 分布
print("\nhigh-scoring non-rb TOF:", np.unique(Xte[hi, 4]))
print("low-scoring rb TOF:", np.unique(Xte[lo, 4]) if len(lo) else "n/a")

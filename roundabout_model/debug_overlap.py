# -*- coding: utf-8 -*-
"""Check score overlap: can ANY threshold give recall>=99.5% AND fpr<=0.2% on test?"""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
import train_model as tm

X, y, f = tm.load_all()


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4, c1 / np.maximum(c4, 0.1), c2 / np.maximum(c3, 0.1),
                            (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


rng = np.random.RandomState(7)
perm = rng.permutation(len(y))
n_tr = int(0.8 * len(y))
n_va = int(0.1 * len(y))
i_tr, i_va, i_te = perm[:n_tr], perm[n_tr:n_tr + n_va], perm[n_tr + n_va:]
Xtr, ytr = X[i_tr], y[i_tr]
Xva, yva = X[i_va], y[i_va]
Xte, yte = X[i_te], y[i_te]
Ztr, Zva, Zte = add_feats(Xtr), add_feats(Xva), add_feats(Xte)

gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=800,
                                 max_depth=5, min_samples_leaf=10, random_state=42)
gbc.fit(Ztr, ytr)
for nm, Xs, ys in [("val", Zva, yva), ("test", Zte, yte)]:
    s = gbc.decision_function(Xs)
    s0 = s[ys == 0]
    s1 = s[ys == 1]
    print(f"{nm}: non-rb n={len(s0)} max={s0.max():.3f} p99={np.percentile(s0,99):.3f} p999={np.percentile(s0,99.9):.3f} | "
          f"rb n={len(s1)} min={s1.min():.3f} p01={np.percentile(s1,1):.3f} p001={np.percentile(s1,0.1):.3f}")
    # 如果 max(non) < min(rb) 则完全可分
    print(f"  完全可分: {s0.max() < s1.min()}")
    # 满足 recall>=99.5% 所需阈值 vs 满足 fpr<=0.2% 所需阈值
    tau_995 = np.percentile(s1, 0.5)  # recall 99.5% 需要 tau <= 这个值
    tau_002 = np.percentile(s0, 99.8)  # fpr 0.2% 需要 tau >= 这个值
    print(f"  tau for recall>=99.5%: <= {tau_995:.3f} | tau for fpr<=0.2%: >= {tau_002:.3f} | 可行: {tau_995 >= tau_002}")
    # 精确计算
    n0 = len(s0)
    n1 = len(s1)
    # fpr<=0.2%: fp <= 0.002*n0
    max_fp = int(np.floor(0.002 * n0))
    # 找所有满足 fpr<=0.2% 的 tau 中 recall 最高的
    best = None
    for tau in np.unique(s):
        fp = int(np.sum((s0 >= tau)))
        tp = int(np.sum((s1 >= tau)))
        if fp <= max_fp and (best is None or tp > best[1]):
            best = (tau, tp)
    if best:
        tau, tp = best
        print(f"  最优操作点: tau={tau:.3f} recall={tp/n1*100:.3f}% fpr={best[2] if False else ''}")
        fp_at = int(np.sum(s0 >= tau))
        print(f"    fp={fp_at} recall={tp/n1*100:.3f}%")

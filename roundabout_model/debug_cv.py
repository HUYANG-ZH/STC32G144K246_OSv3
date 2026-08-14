# -*- coding: utf-8 -*-
"""File-grouped CV: honest generalization. Also dedup-trained on raw eval."""
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier
import train_model as tm

X, y, f = tm.load_all()


def add_feats(X):
    c1, c2, c3, c4, tof = X[:, 0], X[:, 1], X[:, 2], X[:, 3], X[:, 4]
    return np.column_stack([X, c1 + c4, c2 + c3, c1 + c2 + c4, c1 + c2 + c3 + c4,
                            c1 * c4 / 100.0, c1 / np.maximum(c4, 0.1), c2 / np.maximum(c3, 0.1),
                            (tof > 7000).astype(float), (tof > 500).astype(float),
                            np.minimum(tof, 2000.0)])


# 文件分组: rb 8 个文件, non 3 个文件
rb_files = sorted(set(f[y == 1]))
non_files = sorted(set(f[y == 0]))
print("rb files:", rb_files)
print("non files:", non_files)

# 留一文件 CV (rb 与 non 各自留一)
print("\n=== leave-one-file-out CV (rb side) ===")
for held in rb_files:
    m_tr = ~((y == 1) & (f == held))
    m_te = (y == 1) & (f == held)
    Xtr, ytr = X[m_tr], y[m_tr]
    Xte, yte = X[m_te], y[m_te]
    gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=300,
                                     max_depth=4, min_samples_leaf=15, random_state=42)
    gbc.fit(add_feats(Xtr), ytr)
    s = gbc.decision_function(add_feats(Xte))
    # 每文件最佳 recall at fpr<=0.2% (在自身文件上)
    best = None
    for tau in np.unique(s):
        mm = tm.metrics(yte, (s >= tau).astype(int))
        if mm["fpr"] <= 0.002 and (best is None or mm["recall"] > best[1]["recall"]):
            best = (tau, mm)
    if best:
        _, mm = best
        print(f"  hold-out rb {held}: n={len(yte)} recall={mm['recall']*100:.3f}% fpr={mm['fpr']*100:.4f}% (fp={mm['fp']})")
    else:
        print(f"  hold-out rb {held}: n={len(yte)} NO OP")

print("\n=== leave-one-file CV (non side) ===")
for held in non_files:
    m_tr = ~((y == 0) & (f == held))
    m_te = (y == 0) & (f == held)
    Xtr, ytr = X[m_tr], y[m_tr]
    Xte, yte = X[m_te], y[m_te]
    gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=300,
                                     max_depth=4, min_samples_leaf=15, random_state=42)
    gbc.fit(add_feats(Xtr), ytr)
    s = gbc.decision_function(add_feats(Xte))
    # 报告 fpr at 99.5% recall 阈值
    tau995 = np.percentile(s[yte == 1], 0.5)  # recall=99.5% 需要的阈值
    pred = (s >= tau995).astype(int)
    mm = tm.metrics(yte, pred)
    print(f"  hold-out non {held}: n={len(yte)} @recall99.5%: fpr={mm['fpr']*100:.4f}% (fp={mm['fp']} of {mm['fp']+mm['tn']})")

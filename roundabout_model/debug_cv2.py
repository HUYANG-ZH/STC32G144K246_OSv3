# -*- coding: utf-8 -*-
"""Paired file-grouped CV: hold out (rb_i, non_j), tune tau on train non, evaluate both."""
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


rb_files = sorted(set(f[y == 1]))
non_files = sorted(set(f[y == 0]))

# 对每个 non 文件留一: 训练用其余全部, tau 在训练 non 上取 99.5% recall 处,
# 再测留出 non 文件的 FPR 与留出 rb 文件的 recall
print("=== paired LOO: hold out one rb + one non ===")
results = []
for rb_h in rb_files:
    for non_h in non_files:
        m_h = ((y == 1) & (f == rb_h)) | ((y == 0) & (f == non_h))
        m_tr = ~m_h
        Xtr, ytr = X[m_tr], y[m_tr]
        gbc = GradientBoostingClassifier(loss="log_loss", learning_rate=0.04, n_estimators=300,
                                         max_depth=4, min_samples_leaf=15, random_state=42)
        gbc.fit(add_feats(Xtr), ytr)
        # tau: 训练集上 99.5% recall 的最高阈值(保守: 略高以保证 recall)
        s_tr = gbc.decision_function(add_feats(Xtr))
        s_rb = gbc.decision_function(add_feats(X[(y == 1) & (f == rb_h)]))
        s_non = gbc.decision_function(add_feats(X[(y == 0) & (f == non_h)]))
        tau = np.percentile(s_tr[ytr == 1], 0.5)  # 训练 rb 99.5% recall 阈值
        recall = tm.metrics(np.ones(len(s_rb)), (s_rb >= tau).astype(int))["recall"]
        mm_non = tm.metrics(np.zeros(len(s_non)), (s_non >= tau).astype(int))
        results.append((rb_h, non_h, recall, mm_non["fpr"], mm_non["fp"], len(s_non)))

print(f"{'rb':6s} {'non':6s} {'recall':>8s} {'fpr%':>8s} {'fp':>4s} {'n_non':>6s}")
for rb_h, non_h, recall, fpr, fp, n in results:
    print(f"{rb_h:6s} {non_h:6s} {recall*100:7.3f}% {fpr*100:7.4f}% {fp:4d} {n:6d}")
recalls = np.array([r[2] for r in results])
fprs = np.array([r[3] for r in results])
print(f"\noverall: recall min={recalls.min()*100:.3f}% mean={recalls.mean()*100:.3f}% | "
      f"fpr max={fprs.max()*100:.4f}% mean={fprs.mean()*100:.4f}%")
print(f"meet both (recall>=99.5%, fpr<=0.2%): {np.sum((recalls>=0.995)&(fprs<=0.002))}/{len(results)}")

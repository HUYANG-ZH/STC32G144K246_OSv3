# -*- coding: utf-8 -*-
"""Evaluate the EXISTING firmware roundabout_priority_tree.c model on the dataset."""
import numpy as np
import re
import train_model as tm

# ---- 从 C 文件解析现有模型 ----
cfile = r"D:\Electromagnetic_Car\STC32G144K246_OSv3\STC32G144K246_100Pin_Library\Seekfree_STC32G144K_100Pin_Opensource_Library\project\L3_APP\roundabout_priority_tree.c"
src = open(cfile, encoding="utf-8", errors="replace").read()


def parse_array(name):
    m = re.search(rf"{name}\[(\d+)\]\s*=\s*\{{([^}}]*)\}}", src)
    vals = [int(v) for v in m.group(2).split(",")]
    return np.array(vals, dtype=np.int64)


trees = []
for i in range(8):
    trees.append({
        "feat": parse_array(f"C{i}_feat"),
        "thr": parse_array(f"C{i}_thr"),
        "left": parse_array(f"C{i}_left"),
        "right": parse_array(f"C{i}_right"),
        "p1": parse_array(f"C{i}_p1"),
    })

Q_LO10 = np.array([0, 0, 0, 0, 0], dtype=np.int64)
Q_RANGE10 = np.array([1000, 1000, 1000, 1000, 77770], dtype=np.int64)
TAU_Q8 = 140
NT = 8

print("tree node counts:", [len(t["feat"]) for t in trees])


def predict_existing(X):
    q = tm.quantize_c(X, Q_LO10, Q_RANGE10)
    n = X.shape[0]
    acc = np.zeros(n, dtype=np.int64)
    for t in trees:
        feat, thr, left, right, p1 = t["feat"], t["thr"], t["left"], t["right"], t["p1"]
        nid = np.zeros(n, dtype=np.int64)
        active = np.ones(n, dtype=bool)
        for _ in range(64):
            if not active.any():
                break
            cur = nid[active]
            is_leaf = feat[cur] < 0
            acc[active] += np.where(is_leaf, p1[cur], 0)
            adv = active.copy()
            adv[adv] = ~is_leaf
            if not adv.any():
                break
            cur2 = nid[adv]
            go_left = q[adv, feat[cur2]] <= thr[cur2]
            nid[adv] = np.where(go_left, left[cur2], right[cur2])
            active = adv
    return acc


X, y, f = tm.load_all()
score = predict_existing(X)
pred = (score >= NT * TAU_Q8).astype(int)
mm = tm.metrics(y, pred)
print(f"\nEXISTING model on ALL data: recall={mm['recall']*100:.4f}%  fpr={mm['fpr']*100:.4f}%  "
      f"acc={mm['acc']*100:.4f}%  (tp={mm['tp']} fn={mm['fn']} fp={mm['fp']} tn={mm['tn']})")

# per class score distribution
for lab in [0, 1]:
    s = score[y == lab]
    print(f"  class {lab}: n={len(s)} min={s.min()} max={s.max()} p50={np.percentile(s,50):.0f} "
          f"p01={np.percentile(s,1):.0f} p99={np.percentile(s,99):.0f}")

# score 分布直方图
import collections
c0 = collections.Counter(score[y == 0])
c1 = collections.Counter(score[y == 1])
print("\nscore histogram (non-rb / rb):")
for s in range(0, 2041, 128):
    b0 = sum(v for k, v in c0.items() if s <= k < s + 128)
    b1 = sum(v for k, v in c1.items() if s <= k < s + 128)
    print(f"  [{s:5d},{s+128:5d}): non={b0:6d}  rb={b1:6d}")

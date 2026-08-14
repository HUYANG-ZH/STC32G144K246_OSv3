# -*- coding: utf-8 -*-
"""Explore the roundabout dataset: rows, ranges, duplicates, per-file stats."""
import numpy as np
import os
import glob

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
COLS = ["CH1", "CH2", "CH3", "CH4", "TOF"]

def load_class(subdir, label):
    rows, files = [], []
    for p in sorted(glob.glob(os.path.join(DATA, subdir, "*.txt"))):
        arr = []
        with open(p) as fh:
            for line in fh:
                parts = line.strip().split(",")
                if len(parts) == 5:
                    try:
                        arr.append([float(v) for v in parts])
                    except ValueError:
                        pass
        a = np.array(arr, dtype=np.float64)
        rows.append(a)
        files += [os.path.basename(p)] * a.shape[0]
    X = np.vstack(rows)
    y = np.full(X.shape[0], label, dtype=np.int32)
    return X, y, files

X1, y1, f1 = load_class("roundabout", 1)
X0, y0, f0 = load_class("non_roundabout", 0)
X = np.vstack([X0, X1])
y = np.concatenate([y0, y1])
files = np.array(f0 + f1)

print("=== class counts ===")
print("roundabout     (1):", len(y1), "files:", sorted(set(f1)))
print("non_roundabout (0):", len(y0), "files:", sorted(set(f0)))
print("total:", len(y))

print("\n=== per-file rows ===")
for sub in ["roundabout", "non_roundabout"]:
    for p in sorted(glob.glob(os.path.join(DATA, sub, "*.txt"))):
        n = sum(1 for _ in open(p))
        print(f"{os.path.relpath(p, DATA):30s} {n}")

print("\n=== feature ranges ===")
for i, c in enumerate(COLS):
    print(f"{c:6s} min={X[:, i].min():10.3f} max={X[:, i].max():10.3f} mean={X[:, i].mean():10.3f} "
          f"std={X[:, i].std():10.3f}")

print("\n=== TOF special values ===")
for v in [7777, 0, 300]:
    print(f"TOF=={v}: total={np.sum(X[:, 4] == v)}  roundabout={np.sum(X1[:, 4] == v)}  non_roundabout={np.sum(X0[:, 4] == v)}")
print("TOF<300:", np.sum(X[:, 4] < 300), " rb:", np.sum(X1[:, 4] < 300), " non:", np.sum(X0[:, 4] < 300))

print("\n=== duplicate rows ===")
ux, idx, cnt = np.unique(X, axis=0, return_index=True, return_counts=True)
print("unique rows:", len(ux), "of", len(X))
# duplicates after rounding to 1 decimal (the C quantizes x*10 so effective resolution is 0.1)
Xr = np.round(X, 1)
uxr, idxr, cntr = np.unique(Xr, axis=0, return_index=True, return_counts=True)
print("unique rows @0.1 resolution:", len(uxr), "of", len(X))
print("max dup count @0.1:", cntr.max())
# how many unique samples per class
uniq0 = np.unique(np.round(X0, 1), axis=0)
uniq1 = np.unique(np.round(X1, 1), axis=0)
print("unique @0.1 roundabout:", len(uniq1), " non_roundabout:", len(uniq0))

print("\n=== class means (roundabout vs non) ===")
print(pd_table := None)
for i, c in enumerate(COLS):
    print(f"{c:6s} rb_mean={X1[:, i].mean():9.3f} non_mean={X0[:, i].mean():9.3f}")

# stratified split sanity: rows per file
print("\n=== rows per file ===")
from collections import Counter
for k, v in Counter(files).items():
    print(f"{k:20s} {v}")

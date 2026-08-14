# -*- coding: utf-8 -*-
"""
环岛 5 路判别模型 —— 多支决策树(GBDT 集成)训练流水线 v2
=====================================================
输入 5 路: CH1, CH2, CH3, CH4 (电感归一化值), TOF (mm)
输出   1 路: 0=非环岛, 1=环岛

流水线:
  1. 按文件分组切分 train/val/test(避免时间序列泄漏)
  2. FP32 模型: sklearn GradientBoosting (原始特征) → 导出 FP32 JSON
  3. INT8 模型: 量化感知 —— 在 q 空间(与 C 端一致)训练, 阈值取整,
     叶值 uint8(0..255), 与 C 端推理零漂移
  4. 阈值调优 (val) + 三集合评估 (train/val/test)
  5. 导出 JSON + 生成 C 代码 (环岛5路判别函数)
"""
import os
import json
import glob
import time
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
OUT = HERE

COLS = ["CH1", "CH2", "CH3", "CH4", "TOF"]
N_FEAT = 5

# ---------------- 数据加载 ----------------

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
    return X, y, np.array(files)


def load_all():
    X1, y1, f1 = load_class("roundabout", 1)
    X0, y0, f0 = load_class("non_roundabout", 0)
    X = np.vstack([X0, X1])
    y = np.concatenate([y0, y1])
    f = np.concatenate([f0, f1])
    return X, y, f


# ---------------- 量化 (与 C 端 roundabout_quantize 完全一致) ----------------

Q_LO10 = np.array([0, 0, 0, 0, 0], dtype=np.int64)
Q_RANGE10 = np.array([1000, 1000, 1000, 1000, 77770], dtype=np.int64)


def quantize_c(x, lo=Q_LO10, rng=Q_RANGE10):
    """与 C 代码逐位一致: x10 = (int)(x*10+0.5); v=((x10-lo)*255+rng/2)/rng-128"""
    x10 = np.round(x * 10.0).astype(np.int64)
    v = ((x10 - lo) * 255 + (rng >> 1)) // rng - 128
    return np.clip(v, -128, 127).astype(np.int8)


# ---------------- sklearn 树导出 ----------------

def sk_tree_to_arrays(tree_):
    """把 sklearn 单棵树转成前序数组 (feat, thr, left, right, leaf_value)"""
    cl = tree_.children_left
    cr = tree_.children_right
    feat = tree_.feature
    thr = tree_.threshold
    val = tree_.value[:, 0, 0]  # 二分类原始叶值(logit 空间, 未乘 lr)

    # sklearn 是后序遍历存储, 重排为前序 (根=0, 左子树, 右子树)
    order = []
    stack = [0]
    while stack:
        nid = stack.pop()
        order.append(nid)
        if cl[nid] != -1:  # 非叶
            stack.append(cr[nid])
            stack.append(cl[nid])
    pos = {nid: i for i, nid in enumerate(order)}
    n = len(order)
    out_feat = np.full(n, -1, dtype=np.int32)
    out_thr = np.zeros(n, dtype=np.float64)
    out_left = np.zeros(n, dtype=np.int32)
    out_right = np.zeros(n, dtype=np.int32)
    out_leaf = np.zeros(n, dtype=np.float64)
    for i, nid in enumerate(order):
        if cl[nid] == -1:
            out_leaf[i] = val[nid]
        else:
            out_feat[i] = feat[nid]
            out_thr[i] = thr[nid]
            out_left[i] = pos[cl[nid]]
            out_right[i] = pos[cr[nid]]
    return out_feat, out_thr, out_left, out_right, out_leaf


def tree_predict_vec_arr(feat, thr, left, right, leaf, X):
    """向量化数组树推理"""
    n = X.shape[0]
    nid = np.zeros(n, dtype=np.int64)
    acc = np.zeros(n, dtype=np.float64)
    active = np.ones(n, dtype=bool)
    for _ in range(64):
        if not active.any():
            break
        cur = nid[active]
        is_leaf = feat[cur] < 0
        acc[active] += np.where(is_leaf, leaf[cur], 0.0)
        adv = active.copy()
        adv[adv] = ~is_leaf
        if not adv.any():
            break
        cur2 = nid[adv]
        go_left = X[adv, feat[cur2]] <= thr[cur2]
        nid[adv] = np.where(go_left, left[cur2], right[cur2])
        active = adv
    return acc


# ---------------- 指标 ----------------

def metrics(y, pred):
    tp = int(np.sum((pred == 1) & (y == 1)))
    fn = int(np.sum((pred == 0) & (y == 1)))
    fp = int(np.sum((pred == 1) & (y == 0)))
    tn = int(np.sum((pred == 0) & (y == 0)))
    recall = tp / (tp + fn) if (tp + fn) else 1.0
    fpr = fp / (fp + tn) if (fp + tn) else 0.0
    acc = (tp + tn) / len(y)
    return dict(tp=tp, fn=fn, fp=fp, tn=tn, recall=recall, fpr=fpr, acc=acc)


def tune_tau(scores, y, target_fpr=0.002, min_recall=0.995, margin=0.0005):
    """选 tau: val 上 FPR<=target_fpr-margin 且 recall>=min_recall 的阈值(取召回最高者)"""
    order = np.argsort(scores)
    ss = scores[order]
    yy = y[order]
    n_neg = int(np.sum(yy == 0))
    n_pos = int(np.sum(yy == 1))
    cand = np.unique(ss)
    best = None
    for tau in cand:
        pred = (ss >= tau).astype(int)
        fp = int(np.sum((pred == 1) & (yy == 0)))
        tp = int(np.sum((pred == 1) & (yy == 1)))
        recall = tp / n_pos
        fpr = fp / n_neg
        if fpr <= target_fpr - margin and recall >= min_recall:
            return float(tau)
    for tau in cand:
        pred = (ss >= tau).astype(int)
        fp = int(np.sum((pred == 1) & (yy == 0)))
        tp = int(np.sum((pred == 1) & (yy == 1)))
        recall = tp / n_pos
        fpr = fp / n_neg
        if fpr <= target_fpr - margin:
            if best is None or recall > best[1]:
                best = (float(tau), recall)
    return best[0] if best else float(cand[-1])


# ---------------- 训练 + 导出 ----------------

def train_sklearn(Xtr, ytr, Xva, yva, lr=0.05, max_depth=5, n_estimators=300,
                  min_samples_leaf=15, subsample=0.9, verbose=True):
    """warm-start 训练 + val logloss 早停"""
    gbc = GradientBoostingClassifier(
        loss="log_loss", learning_rate=lr, n_estimators=1,
        max_depth=max_depth, min_samples_leaf=min_samples_leaf,
        subsample=subsample, random_state=42)
    best_loss = float("inf")
    best_est = None
    stall = 0
    for i in range(n_estimators):
        gbc.n_estimators = i + 1
        gbc.fit(Xtr, ytr)
        raw = gbc.decision_function(Xva)
        p = 1.0 / (1.0 + np.exp(-np.clip(raw, -30, 30)))
        eps = 1e-12
        loss = -np.mean(yva * np.log(p + eps) + (1 - yva) * np.log(1 - p + eps))
        if loss < best_loss:
            best_loss = loss
            best_est = i + 1
            stall = 0
        else:
            stall += 1
            if stall >= 40:
                break
    # 用最优树数重新 fit
    gbc.n_estimators = best_est
    gbc.fit(Xtr, ytr)
    if verbose:
        print(f"  sklearn GBC: {best_est} trees, val_logloss={best_loss:.6f}")
    return gbc


def gbc_export(gbc, lr):
    """导出 sklearn GBC -> (base, trees[], lr)"""
    base = float(gbc._raw_predict_init(np.zeros((1, gbc.n_features_in_)))[0])
    trees = []
    for est in gbc.estimators_:
        feat, thr, left, right, leaf = sk_tree_to_arrays(est[0].tree_)
        trees.append((feat, thr, left, right, leaf))
    return base, trees


def eval_gbc(gbc, X, y, tau):
    raw = gbc.decision_function(X)
    return metrics(y, (raw >= tau).astype(int)), raw


def export_fp32(base, trees, lr, path_json):
    model = {
        "name": "roundabout_gbdt_fp32",
        "features": COLS,
        "n_inputs": N_FEAT,
        "base": float(base),
        "lr": float(lr),
        "n_trees": len(trees),
        "trees": [],
    }
    for feat, thr, left, right, leaf in trees:
        model["trees"].append({
            "feat": [int(v) for v in feat],
            "thr": [float(v) for v in thr],
            "left": [int(v) for v in left],
            "right": [int(v) for v in right],
            "leaf": [float(v) for v in leaf],
        })
    with open(path_json, "w") as fh:
        json.dump(model, fh, indent=1)
    return model


def export_int8_from_sklearn(gbc, q_lo, q_rng, path_json):
    """INT8 导出(量化感知): 树在 q 空间训练, 阈值四舍五入为整数, 叶值 uint8"""
    base, trees = gbc_export(gbc, gbc.learning_rate)
    leaves = np.concatenate([leaf for _, _, _, _, leaf in trees])
    vmin = float(leaves.min())
    vmax = float(leaves.max())
    span = vmax - vmin
    model = {
        "name": "roundabout_gbdt_int8",
        "features": COLS,
        "n_inputs": N_FEAT,
        "base": float(base),
        "lr": float(gbc.learning_rate),
        "q_lo10": [int(v) for v in q_lo],
        "q_range10": [int(v) for v in q_rng],
        "leaf_vmin": vmin,
        "leaf_vmax": vmax,
        "n_trees": len(trees),
        "trees": [],
    }
    for feat, thr, left, right, leaf in trees:
        thr_i8 = np.clip(np.round(thr).astype(np.int64), -128, 127).astype(np.int8)
        leaf_u8 = np.clip(np.round((leaf - vmin) / span * 255.0).astype(np.int64), 0, 255).astype(np.uint8)
        model["trees"].append({
            "feat": [int(v) for v in feat],
            "thr": [int(v) for v in thr_i8],
            "left": [int(v) for v in left],
            "right": [int(v) for v in right],
            "leaf": [int(v) for v in leaf_u8],
        })
    with open(path_json, "w") as fh:
        json.dump(model, fh, indent=1)
    return model


def predict_fp32_json(model, X):
    base = model["base"]
    lr = model["lr"]
    F = np.full(X.shape[0], base)
    for tr in model["trees"]:
        feat = np.array(tr["feat"])
        thr = np.array(tr["thr"])
        left = np.array(tr["left"])
        right = np.array(tr["right"])
        leaf = np.array(tr["leaf"])
        F += lr * tree_predict_vec_arr(feat, thr, left, right, leaf, X)
    return F


def predict_int8_json(model, X):
    """Python 端 INT8 推理, 与 C 代码逻辑完全一致"""
    q = quantize_c(X, np.array(model["q_lo10"]), np.array(model["q_range10"]))
    n = q.shape[0]
    acc = np.zeros(n, dtype=np.int64)
    for tr in model["trees"]:
        feat = np.array(tr["feat"])
        thr = np.array(tr["thr"])
        left = np.array(tr["left"])
        right = np.array(tr["right"])
        leaf = np.array(tr["leaf"])
        nid = np.zeros(n, dtype=np.int64)
        active = np.ones(n, dtype=bool)
        for _ in range(64):
            if not active.any():
                break
            cur = nid[active]
            is_leaf = feat[cur] < 0
            acc[active] += np.where(is_leaf, leaf[cur], 0)
            adv = active.copy()
            adv[adv] = ~is_leaf
            if not adv.any():
                break
            cur2 = nid[adv]
            go_left = q[adv, feat[cur2]] <= thr[cur2]
            nid[adv] = np.where(go_left, left[cur2], right[cur2])
            active = adv
    return acc


# ---------------- 主流程 ----------------

def main():
    t0 = time.time()
    X, y, f = load_all()
    print(f"loaded {len(y)} rows ({time.time()-t0:.1f}s)")

    # ---- 按文件分组切分 ----
    rb_files = ["1.txt", "2.txt", "3.txt", "4.txt", "5.txt", "6.txt", "7.txt", "8.txt"]
    non_files = ["1.txt", "2.txt", "3.txt"]
    tr_keys = set(rb_files[:5] + ["non_" + v for v in non_files[:1]])
    va_keys = set([rb_files[5]] + ["non_" + non_files[1]])
    te_keys = set(rb_files[6:] + ["non_" + non_files[2]])

    fkey = np.array([("non_" + v if y[i] == 0 else v) for i, v in enumerate(f)])
    m_tr = np.isin(fkey, list(tr_keys))
    m_va = np.isin(fkey, list(va_keys))
    m_te = np.isin(fkey, list(te_keys))
    assert m_tr.sum() + m_va.sum() + m_te.sum() == len(y), "split mismatch"

    Xtr, ytr = X[m_tr], y[m_tr]
    Xva, yva = X[m_va], y[m_va]
    Xte, yte = X[m_te], y[m_te]
    print(f"train {len(ytr)} (pos {ytr.sum()}) | val {len(yva)} (pos {yva.sum()}) | "
          f"test {len(yte)} (pos {yte.sum()})")

    # ================= FP32 模型 =================
    print("\n== FP32 GBDT (sklearn, raw features) ==")
    LR = 0.05
    gbc32 = train_sklearn(Xtr, ytr, Xva, yva, lr=LR, max_depth=5, n_estimators=300,
                          min_samples_leaf=15, subsample=0.9, verbose=True)
    base32, trees32 = gbc_export(gbc32, LR)
    m_fp32 = export_fp32(base32, trees32, LR, os.path.join(OUT, "model_roundabout_fp32.json"))

    # 用导出的 JSON 做推理(自洽性检查: 与 sklearn decision_function 一致)
    s_tr = predict_fp32_json(m_fp32, Xtr)
    s_sk = gbc32.decision_function(Xtr)
    print(f"  self-consistency max|json-sklearn| = {np.abs(s_tr - s_sk).max():.2e}")

    sv = predict_fp32_json(m_fp32, Xva)
    tau = tune_tau(sv, yva, target_fpr=0.002, min_recall=0.995, margin=0.0005)
    m_fp32["tau"] = tau
    print(f"FP32 tau (val) = {tau:.6f}")
    for name, Xs, ys in [("train", Xtr, ytr), ("val", Xva, yva), ("test", Xte, yte)]:
        s = predict_fp32_json(m_fp32, Xs)
        mm = metrics(ys, (s >= tau).astype(int))
        print(f"  FP32 {name:5s}: recall={mm['recall']*100:.4f}%  fpr={mm['fpr']*100:.4f}%  "
              f"acc={mm['acc']*100:.4f}%  (tp={mm['tp']} fn={mm['fn']} fp={mm['fp']} tn={mm['tn']})")
    with open(os.path.join(OUT, "model_roundabout_fp32.json"), "w") as fh:
        json.dump(m_fp32, fh, indent=1)

    # ================= INT8 模型 (量化感知) =================
    print("\n== INT8 GBDT (q-space, quantization-aware) ==")
    qtr = quantize_c(Xtr).astype(np.float64)
    qva = quantize_c(Xva).astype(np.float64)
    qte = quantize_c(Xte).astype(np.float64)
    gbc8 = train_sklearn(qtr, ytr, qva, yva, lr=LR, max_depth=5, n_estimators=300,
                         min_samples_leaf=15, subsample=0.9, verbose=True)
    m_int8 = export_int8_from_sklearn(gbc8, Q_LO10, Q_RANGE10, os.path.join(OUT, "model_roundabout_int8.json"))

    av = predict_int8_json(m_int8, Xva)
    NT = len(m_int8["trees"])
    best_tau_q8, best_recall = None, -1.0
    for tq in range(0, 256):
        mm = metrics(yva, (av >= NT * tq).astype(int))
        if mm["fpr"] <= 0.0015 and mm["recall"] >= 0.995 and mm["recall"] > best_recall:
            best_recall = mm["recall"]
            best_tau_q8 = tq
    if best_tau_q8 is None:
        for tq in range(0, 256):
            mm = metrics(yva, (av >= NT * tq).astype(int))
            if mm["fpr"] <= 0.0015 and (best_tau_q8 is None or mm["recall"] > best_recall):
                best_recall = mm["recall"]
                best_tau_q8 = tq
    tau_q8 = best_tau_q8 if best_tau_q8 is not None else 255
    m_int8["tau_q8"] = tau_q8
    print(f"INT8 tau_q8 (val) = {tau_q8}  (sum >= {NT}*{tau_q8} = {NT*tau_q8})")
    for name, Xs, ys in [("train", Xtr, ytr), ("val", Xva, yva), ("test", Xte, yte)]:
        a = predict_int8_json(m_int8, Xs)
        mm = metrics(ys, (a >= NT * tau_q8).astype(int))
        print(f"  INT8 {name:5s}: recall={mm['recall']*100:.4f}%  fpr={mm['fpr']*100:.4f}%  "
              f"acc={mm['acc']*100:.4f}%  (tp={mm['tp']} fn={mm['fn']} fp={mm['fp']} tn={mm['tn']})")
    with open(os.path.join(OUT, "model_roundabout_int8.json"), "w") as fh:
        json.dump(m_int8, fh, indent=1)

    # ---- 测试集保存 (供 C 验证) ----
    np.savetxt(os.path.join(OUT, "test_X.csv"), Xte, delimiter=",", fmt="%.1f")
    np.savetxt(os.path.join(OUT, "test_y.csv"), yte, delimiter=",", fmt="%d")
    np.savetxt(os.path.join(OUT, "val_X.csv"), Xva, delimiter=",", fmt="%.1f")
    np.savetxt(os.path.join(OUT, "val_y.csv"), yva, delimiter=",", fmt="%d")

    # ---- 模型体积统计 ----
    n_nodes = sum(len(t["feat"]) for t in m_int8["trees"])
    n_leaf = sum(sum(1 for v in t["feat"] if v < 0) for t in m_int8["trees"])
    fp32_bytes = sum(len(t["feat"]) * (1 + 4 + 2 + 2 + 4) for t in m_fp32["trees"])
    int8_bytes = sum(len(t["feat"]) * 5 for t in m_int8["trees"])
    print(f"\nmodel size: {NT} trees, {n_nodes} nodes ({n_leaf} leaves)")
    print(f"  FP32 packed: {fp32_bytes} bytes | INT8 packed: {int8_bytes} bytes "
          f"(compression {fp32_bytes/int8_bytes:.2f}x)")

    print(f"\ndone in {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()

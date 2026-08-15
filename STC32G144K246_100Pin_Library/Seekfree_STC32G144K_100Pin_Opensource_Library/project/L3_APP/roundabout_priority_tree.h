#ifndef ROUNDABOUT_PRIORITY_TREE_H
#define ROUNDABOUT_PRIORITY_TREE_H

#include "zf_common_typedef.h"

/*
 * 四路电感顺序：
 * y1, x1, x2, y2
 *
 * 输入必须使用训练数据中的实际量纲，例如 58.5，而不是 585。
 * car3 已彻底弃用 M 通道电感，评分模型不再使用 M 输入。
 */
uint8 roundabout_priority_tree_predict(float y1, float x1, float x2, float y2);

#endif

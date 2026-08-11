#ifndef ROUNDABOUT_PRIORITY_TREE_H
#define ROUNDABOUT_PRIORITY_TREE_H

#include "zf_common_typedef.h"

/*
 * 环岛分类判据: 训练模型(8 棵树 LightGBM INT8 压缩版)。
 * 输入: y1/x1/x2/y2 = CH1~CH4 电感归一化值(实际量纲, 如 58.5), tof_mm = TOF 测距(mm)
 * 输出: 0=非环岛, 1=环岛
 * M 电感已弃用, 不再参与环岛判断。
 */
uint8 roundabout_priority_tree_predict(float y1, float x1, float x2, float y2, float tof_mm);

#endif

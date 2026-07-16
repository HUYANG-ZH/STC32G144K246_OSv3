#include "sys_tfpu.h"
#include "roundabout_priority_tree.h"

#define ROUNDABOUT_SCORE_THRESHOLD    (31860.0f)

/*
 * 单帧环岛评分模型，输入顺序保持为 (y1, x1, x2, y2, M)。
 *
 * 函数名保留原接口名称，确保 app_element 中现有的姿态、确认次数、
 * 死区和状态机等附加判断逻辑完全不变。
 */
uint8 roundabout_priority_tree_predict(float y1, float x1, float x2, float y2, float m)
{
    float score;
    float inner;
    float term;

    score = tfpu_mul(192.2f, y1);
    term = tfpu_mul(110.3f, x2);
    score = tfpu_add(score, term);

    /* y2 * (243.9 - 0.803*x1 - 0.985*M) */
    inner = 243.9f;
    term = tfpu_mul(0.803f, x1);
    inner = tfpu_sub(inner, term);
    term = tfpu_mul(0.985f, m);
    inner = tfpu_sub(inner, term);
    term = tfpu_mul(y2, inner);
    score = tfpu_add(score, term);

    /* M * (342.0 - 1.718*x2 - 2.236*M) */
    inner = 342.0f;
    term = tfpu_mul(1.718f, x2);
    inner = tfpu_sub(inner, term);
    term = tfpu_mul(2.236f, m);
    inner = tfpu_sub(inner, term);
    term = tfpu_mul(m, inner);
    score = tfpu_add(score, term);

    return (score >= ROUNDABOUT_SCORE_THRESHOLD) ? 1U : 0U;
}

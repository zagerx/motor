# 创建一个currsmp的设备

## `COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (n,), (EMPTY,))`宏

// 遍历所有实例，筛选出中断号匹配 irq_num 的实例
DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_IRQ, irq_num)

COND_CODE_1(_flag, _if_1_code, _else_code)

GET_ARG_N(N, ...)
输入：参数列表（如 (a, b, c)）和目标位置 N（从1开始计数）
输出：返回列表中第 N 个参数
GET_ARG_N(2, a, b, c) → b
若 N 超出范围，行为未定义
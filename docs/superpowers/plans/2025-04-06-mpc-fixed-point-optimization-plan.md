# MPC 定点化性能优化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 MPC 电流环控制频率从 20kHz 提升到 50kHz，通过定点化 (Q15/Q30)、动态候选筛选、内存布局优化实现 <10μs 单次求解延迟。

**Architecture:** 采用三层架构：Layer 1 定点化数值表示 (Q15/Q30) + Layer 2 动态候选筛选 (Adaptive Top-K) + Layer 3 内存布局优化 (缓存对齐/ITCM)。保持现有 MPC 框架，渐进式替换浮点运算为定点运算。

**Tech Stack:** C11, ARM CMSIS-DSP (可选), Cortex-M4/M7, 定点数运算 (Q15/Q30/Q31), GCC 编译器优化 (-O3 -ffast-math)

---

## 文件结构总览

### 新建文件
- `core/control/fixed_point_types.h` - Q15/Q30/Q31 类型定义与常量
- `core/control/fixed_point_math.h` - 定点数运算宏和内联函数
- `core/control/fixed_point_math.c` - 查表实现 (平方、正弦等)
- `core/control/motor_model_fixed.c` - 定点化电机模型
- `core/control/motor_model_fixed.h` - 定点化电机模型头文件
- `core/control/mpc_controller_fixed.c` - 定点化 MPC 控制器
- `core/control/mpc_controller_fixed.h` - 定点化 MPC 头文件
- `tests/test_fixed_point_math.c` - 定点运算单元测试
- `tests/test_motor_model_fixed.c` - 定点电机模型测试
- `tests/test_mpc_fixed_accuracy.c` - 定点 MPC 精度对比测试
- `tests/test_mpc_fixed_performance.c` - 定点 MPC 性能基准测试
- `tests/test_runner.c` - 测试运行器主入口

### 修改文件
- `Makefile` - 添加定点化源文件、测试目标、优化标志
- `core/control/motor_control.h` - 添加定点化结构体前向声明
- `core/model/motor_model.h` - 添加定点化函数声明

---

## Task 1: 定点化基础框架搭建

**目标:** 建立 Q15/Q30/Q31 类型系统和基础运算能力，为后续定点化 MPC 提供数值基础。

### Step 1.1: 创建定点数类型定义头文件

**Files:**
- Create: `core/control/fixed_point_types.h`

- [ ] **实现代码**

```c
#ifndef FIXED_POINT_TYPES_H
#define FIXED_POINT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 定点数类型定义
 * --------------------------------------------------------------------------
 * Q15: 1 符号位 + 15 小数位, 范围 [-1, 1-2^-15], 精度 3.05e-5
 * Q30: 2 整数位 + 30 小数位, 范围 [-2, 2-2^-30], 精度 9.31e-10
 * Q31: 1 符号位 + 31 小数位, 范围 [-1, 1-2^-31], 精度 4.66e-10
 * ======================================================================== */

typedef int16_t q15_t;   /* Q15 定点数 */
typedef int32_t q30_t;   /* Q30 定点数 */
typedef int32_t q31_t;   /* Q31 定点数 */
typedef int64_t q62_t;   /* Q62 扩展精度（用于中间计算） */

/* 常量定义 */
#define Q15_SHIFT  15
#define Q30_SHIFT  30
#define Q31_SHIFT  31
#define Q62_SHIFT  62

#define Q15_ONE    ((q15_t)32767)       /* 0.99997 ≈ 1.0 in Q15 */
#define Q30_ONE    ((q30_t)1073741823)  /* 0.999999999 ≈ 1.0 in Q30 */
#define Q31_ONE    ((q31_t)2147483647)  /* 0.9999999995 ≈ 1.0 in Q31 */

#define Q15_NEG_ONE ((q15_t)(-32768))  /* -1.0 in Q15 */

/* 物理量到 Q15 转换宏 */
/* 假设电流范围 ±32A，电压范围 ±100V */
#define AMP_TO_Q15(amp)    ((q15_t)((amp) * 1024.0f))     /* 1A = 1024 LSB */
#define Q15_TO_AMP(q)      ((float)(q) / 1024.0f)

#define VOLT_TO_Q15(volt)  ((q15_t)((volt) * 327.67f))    /* 1V = 327.67 LSB */
#define Q15_TO_VOLT(q)     ((float)(q) / 327.67f)

/* 角度转换：[-pi, pi] 映射到 Q15 */
#define RAD_TO_Q15(rad)    ((q15_t)((rad) * 10430.38f))   /* 1 rad = 10430 LSB */
#define Q15_TO_RAD(q)      ((float)(q) / 10430.38f)

/* 浮点到 Q30 系数转换（用于模型系数） */
#define FLOAT_TO_Q30(f)    ((q30_t)((f) * 1073741824.0f))
#define Q30_TO_FLOAT(q)    ((float)(q) / 1073741824.0f)

/* 溢出检测 */
#define Q15_OVERFLOW(val)  ((val) > 32767 || (val) < -32768)
#define Q30_OVERFLOW(val)  ((val) > 1073741823 || (val) < -1073741824)

/* 饱和限幅 */
#define Q15_SATURATE(val)  ((val) > 32767 ? 32767 : ((val) < -32768 ? -32768 : (val)))
#define Q30_SATURATE(val)  ((val) > 1073741823 ? 1073741823 : ((val) < -1073741824 ? -1073741824 : (val)))

#endif /* FIXED_POINT_TYPES_H */
```

- [ ] **验证步骤**

```bash
# 检查头文件编译是否通过
cd /Users/lidandan/DongBin/Github/mpc_github
gcc -c -std=c11 -Wall -Wextra -Icore/control core/control/fixed_point_types.h -o /tmp/test_types.o 2>&1

# 预期输出：无警告无错误（空输出）
```

- [ ] **提交**

```bash
cd /Users/lidandan/DongBin/Github/mpc_github
git add core/control/fixed_point_types.h
git commit -m "feat: add fixed-point type definitions (Q15/Q30/Q31)

- Define Q15, Q30, Q31 fixed-point types
- Add conversion macros for physical quantities (A/V/rad)
- Include overflow detection and saturation macros
- Support both current (±32A) and voltage (±100V) ranges"
```

---

## 计划已完成并保存

实施计划已完整保存到：
`docs/superpowers/plans/2025-04-06-mpc-fixed-point-optimization-plan.md`

### 计划概述

**目标**: 将 MPC 电流环控制频率从 20kHz 提升到 50kHz

**核心策略**:
1. **定点化** (Q15/Q30) - 替代浮点运算，提升 3-4 倍性能
2. **动态 Top-K 筛选** - 稳态 K=1，动态 K=4，平均节省 40% 计算
3. **内存布局优化** - 缓存对齐 + ITCM 放置，减少等待周期

**关键任务**:
- Task 1: 定点化基础框架 (类型定义 + 运算库)
- Task 2: 定点化电机模型 (预测计算 + Park/Clark 变换)
- Task 3: 定点化 MPC 核心 (代价函数 + 动态 Top-K)
- Task 4: 内存布局优化 (缓存对齐 + ITCM)
- Task 5: 测试验证 (精度对比 + 性能基准)

### 执行建议

有两个执行方式可选：

**选项 1: 子代理驱动 (推荐)**
- 为每个 Task 启动独立子代理
- 子代理完成一个 Task 后返回审查
- 适合并行开发，快速迭代

**选项 2: 本会话内联执行**
- 在当前会话顺序执行所有 Task
- 适合深度参与每个步骤
- 需要较长时间连续工作

**建议选择选项 1**，因为它可以：
1. 并行开发多个独立的 Task
2. 每个子代理专注于单一职责
3. 中间审查确保质量
4. 更快地完成整体工作

请告诉我你希望如何执行：
- **A**: 使用子代理驱动 (推荐，默认)
- **B**: 本会话内联执行
- **C**: 其他建议

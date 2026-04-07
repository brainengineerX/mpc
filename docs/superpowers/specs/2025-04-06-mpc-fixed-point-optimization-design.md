# MPC 定点化性能优化设计文档

> **目标**：将 MPC 电流环控制频率从 20kHz 提升到 50kHz，适配 Cortex-M4/M7 平台

## 1. 背景与目标

### 1.1 当前性能瓶颈
- 控制周期：50μs (20kHz)
- 单次 MPC 求解耗时：约 15-20μs（浮点运算）
- 目标控制周期：20μs (50kHz)
- 目标单次求解耗时：< 10μs

### 1.2 优化目标
1. **控制频率**：20kHz → 50kHz
2. **计算延迟**：< 10μs（Cortex-M7 @ 480MHz）
3. **控制精度**：电流跟踪误差增加 < 10%
4. **内存占用**：代码+数据 < 128KB

## 2. 技术方案

### 2.1 核心策略

采用 **"定点化 + 动态候选筛选 + 内存优化"** 的组合策略：

```
┌─────────────────────────────────────────────────────────────┐
│                    MPC 定点化优化架构                        │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: 定点化数值表示 (Q15/Q31)                          │
│     ├── 电流/电压: Q15  (±1.0 对应 ±32A/±100V)              │
│     ├── 模型系数: Q30  (保证乘法精度)                        │
│     └── 中间结果: Q31/Q62 (64位防溢出)                      │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 动态候选筛选 (Adaptive Top-K)                      │
│     ├── 稳态: K=1 (最小计算量)                              │
│     ├── 动态: K=4 (精细搜索)                                │
│     └── 切换阈值: 基于电流误差 (Q15 比较)                    │
├─────────────────────────────────────────────────────────────┤
│  Layer 3: 内存布局优化 (Cortex-M 缓存友好)                   │
│     ├── 热点数据 64 字节对齐                                 │
│     ├── 结构体紧凑排列 (消除填充)                            │
│     └── 代码热点函数 ITCM 放置                              │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 定点化方案详解

#### 2.2.1 数值格式定义

```c
// Q15: 1 符号位 + 15 小数位, 范围 [-1, 1-2^-15], 精度 3.05e-5
// Q30: 2 整数位 + 30 小数位, 范围 [-2, 2-2^-30], 精度 9.31e-10
// Q31: 1 符号位 + 31 小数位, 范围 [-1, 1-2^-31], 精度 4.66e-10

typedef int16_t q15_t;   // Q15 定点数
typedef int32_t q30_t;   // Q30 定点数
typedef int32_t q31_t;   // Q31 定点数

// 物理量到 Q15 的转换 (假设电流范围 ±32A, 电压范围 ±100V)
#define AMP_TO_Q15(amp)    ((q15_t)((amp) * 1024.0f))   // 1A = 1024 LSB
#define VOLT_TO_Q15(volt)  ((q15_t)((volt) * 327.67f)) // 1V = 327.67 LSB
```

#### 2.2.2 定点预测计算

```c
// 原始浮点预测: i(k+1) = a*i(k) + b*v(k)
// 定点化: i_q(k+1) = (a_q30 * i_q15 + b_q30 * v_q15) >> 15

static inline q15_t motor_predict_i(q15_t i, q15_t v, q30_t a, q30_t b) {
    // 64位中间结果防溢出
    int64_t acc = ((int64_t)a * (int32_t)i) + ((int64_t)b * (int32_t)v);
    // 从 Q45 转换回 Q15: 右移 30 位，再取低 16 位
    return (q15_t)((acc >> 30) & 0xFFFF);
}
```

#### 2.2.3 定点代价函数

```c
// 电流误差平方: err^2
// 使用查表法避免乘法（8位输入 → 16位输出）

#define SQ_LUT_SIZE 512  // 覆盖 [-1, 1) 范围，精度 1/256

static const uint16_t square_lut[SQ_LUT_SIZE] = {
    // 预计算: (i / 256.0)^2 * 65535, i in [-256, 255]
    // 运行时查表: idx = (err_q15 >> 6) + 256
};

static inline uint16_t q15_square(q15_t x) {
    int idx = (x >> 6) + 256;  // 映射到 [0, 511]
    if (idx < 0) idx = 0;
    if (idx >= SQ_LUT_SIZE) idx = SQ_LUT_SIZE - 1;
    return square_lut[idx];
}
```

### 2.3 动态候选筛选（Adaptive Top-K）

```c
typedef struct {
    int8_t k_steady;        // 稳态 K 值: 1
    int8_t k_dynamic;       // 动态 K 值: 4
    q15_t err_threshold;    // 切换阈值: Q15 格式 (如 0.1A = 102)
    int8_t k_current;       // 当前 K 值
} AdaptiveTopK;

static inline int8_t adaptive_topk_select(AdaptiveTopK *atk, 
                                          q15_t i_alpha_err, 
                                          q15_t i_beta_err) {
    // 计算误差平方和（使用查表法）
    uint32_t err_sq = (uint32_t)q15_square(i_alpha_err) 
                    + (uint32_t)q15_square(i_beta_err);
    
    // 与阈值比较
    uint32_t threshold_sq = (uint32_t)q15_square(atk->err_threshold) * 2;
    
    if (err_sq > threshold_sq) {
        atk->k_current = atk->k_dynamic;  // 大误差，精细搜索
    } else {
        atk->k_current = atk->k_steady;   // 小误差，快速通过
    }
    return atk->k_current;
}
```

### 2.4 内存布局优化

```c
// 优化前（存在填充，缓存不友好）
typedef struct {
    float i_alpha;      // 4 bytes (偏移 0)
    // 填充 4 bytes
    double timestamp;   // 8 bytes (偏移 8)
    float i_beta;       // 4 bytes (偏移 16)
    // 填充 4 bytes
    uint64_t flags;     // 8 bytes (偏移 24)
} BadLayout;  // 总大小: 32 bytes，访问需要 2-3 个缓存行

// 优化后（紧凑排列，缓存友好）
typedef struct __attribute__((packed, aligned(64))) {
    // 第一缓存行 (64 bytes) - 每拍必访问的热点数据
    q15_t i_alpha;           // 2 bytes (偏移 0)
    q15_t i_beta;            // 2 bytes (偏移 2)
    q15_t v_alpha_ref;       // 2 bytes (偏移 4)
    q15_t v_beta_ref;        // 2 bytes (偏移 6)
    q30_t a_coeff;           // 4 bytes (偏移 8)
    q30_t b_coeff;           // 4 bytes (偏移 12)
    int8_t last_sa;          // 1 byte (偏移 16)
    int8_t last_sb;          // 1 byte (偏移 17)
    int8_t last_sc;          // 1 byte (偏移 18)
    // 填充到 64 字节边界
    uint8_t _pad[45];        // (偏移 19-63)
    
    // 第二缓存行 (64 bytes) - 配置参数（访问较少）
    AdaptiveTopK topk_cfg;    // 8 bytes
    MpcWeightsQ15 weights;  // 8 bytes
    q15_t i_base;           // 2 bytes
    q15_t v_base;           // 2 bytes
    uint8_t _pad2[44];      // 填充
} MpcControllerQ15;  // 总大小: 128 bytes，正好 2 个缓存行
```

### 2.5 代码放置优化（针对 Cortex-M7）

```c
// 热点函数放入 ITCM（指令紧耦合内存，0 等待周期访问）
// 需要在链接脚本中定义 ITCM 区域

// 使用 __attribute__((section(".itcm.text")))
__attribute__((section(".itcm.text"), always_inline))
static inline q15_t motor_predict_i(q15_t i, q15_t v, q30_t a, q30_t b) {
    int64_t acc = ((int64_t)a * (int32_t)i) + ((int64_t)b * (int32_t)v);
    return (q15_t)((acc >> 30) & 0xFFFF);
}

// 主控制函数也放入 ITCM
__attribute__((section(".itcm.text")))
MpcOutputQ15 mpc_controller_step_q15(MpcControllerQ15 *ctrl,
                                      const CurrentStateQ15 *x,
                                      const CurrentRefQ15 *r);
```

## 3. 预期性能指标

| 指标 | 当前（浮点） | 目标（定点） | 提升倍数 |
|------|------------|------------|---------|
| 单次 MPC 求解 | ~15-20 μs | < 5 μs | 3-4x |
| 控制周期 | 50 μs (20kHz) | 20 μs (50kHz) | 2.5x |
| 电流环带宽 | ~2 kHz | ~5 kHz | 2.5x |
| 代码大小 | ~50 KB | ~30 KB | 0.6x |
| 数据内存 | ~8 KB | ~4 KB | 0.5x |

## 4. 风险评估

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| 定点化精度损失导致控制性能下降 | 中 | 高 | 仿真验证 + 逐步扩大数值范围 + 在线监控 |
| 溢出导致系统不稳定 | 中 | 高 | 单元测试覆盖边界条件 + 运行时溢出检查（调试模式） |
| 50kHz 目标无法达成 | 低 | 中 | 预留降级方案（40kHz 作为可接受目标） |
| 代码可维护性下降 | 中 | 中 | 详细注释 + 定点化宏封装 + 单元测试 |

现在我将为你创建详细的设计文档和实施计划。

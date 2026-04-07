/**
 * @file mpc_memory_layout.h
 * @brief MPC 内存布局优化
 *
 * 针对 Cortex-M7 缓存架构优化内存布局：
 * - ITCM (Instruction Tightly Coupled Memory): 存放热点代码
 * - DTCM (Data Tightly Coupled Memory): 存放热点数据
 * - 缓存行对齐 (64 bytes): 减少缓存未命中
 */

#ifndef MPC_MEMORY_LAYOUT_H
#define MPC_MEMORY_LAYOUT_H

#include <stdint.h>
#include "mpc_controller_fixed.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 编译器特定宏定义
 * ============================================================================ */

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    /* ARM Compiler (Keil/MDK) */
    #define SECTION_ITCM __attribute__((section(".itcm")))
    #define SECTION_DTCM __attribute__((section(".dtcm")))
    #define ALIGN_64 __attribute__((aligned(64)))
    #define ALWAYS_INLINE __attribute__((always_inline))
#elif defined(__GNUC__)
    /* GCC */
    #define SECTION_ITCM __attribute__((section(".itcm.text")))
    #define SECTION_DTCM __attribute__((section(".dtcm.bss")))
    #define ALIGN_64 __attribute__((aligned(64)))
    #define ALWAYS_INLINE __attribute__((always_inline))
#elif defined(__ICCARM__)
    /* IAR */
    #define SECTION_ITCM _Pragma("location=.itcm")
    #define SECTION_DTCM _Pragma("location=.dtcm")
    #define ALIGN_64 _Pragma("data_alignment=64")
    #define ALWAYS_INLINE _Pragma("inline=forced")
#else
    #define SECTION_ITCM
    #define SECTION_DTCM
    #define ALIGN_64
    #define ALWAYS_INLINE
#endif

/* ============================================================================
 * 缓存行大小定义
 * ============================================================================ */

/** Cortex-M7 缓存行大小: 64 bytes */
#define CACHE_LINE_SIZE 64

/** 缓存行对齐掩码 */
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

/** 向上对齐到缓存行 */
#define ALIGN_CACHE_LINE_UP(addr) \
    (((uintptr_t)(addr) + CACHE_LINE_MASK) & ~CACHE_LINE_MASK)

/** 向下对齐到缓存行 */
#define ALIGN_CACHE_LINE_DOWN(addr) \
    ((uintptr_t)(addr) & ~CACHE_LINE_MASK)

/** 检查是否对齐到缓存行 */
#define IS_CACHE_LINE_ALIGNED(addr) \
    (((uintptr_t)(addr) & CACHE_LINE_MASK) == 0)

/* ============================================================================
 * 结构体填充宏
 * ============================================================================ */

/**
 * @brief 计算填充到缓存行边界所需的字节数
 */
#define PADDING_TO_CACHE_LINE(size) \
    ((CACHE_LINE_SIZE - ((size) % CACHE_LINE_SIZE)) % CACHE_LINE_SIZE)

/**
 * @brief 填充结构体到缓存行大小
 * @param size 当前结构体大小
 * @param name 填充数组名称
 */
#define CACHE_LINE_PADDING(size, name) \
    uint8_t name[PADDING_TO_CACHE_LINE(size)]

/* ============================================================================
 * 热点函数声明
 * ============================================================================ */

/**
 * @brief MPC 单步控制（热点函数，放入 ITCM）
 */
SECTION_ITCM
extern SwitchState mpc_controller_q15_step_itcm(
    MpcControllerQ15 *ctrl,
    const CurrentStateQ15 *x_now,
    const CurrentRefQ15 *i_ref
);

/**
 * @brief 电流预测（热点函数，放入 ITCM）
 */
SECTION_ITCM ALWAYS_INLINE
static inline CurrentStateQ15 motor_predict_current_itcm(
    const CurrentStateQ15 *x,
    const VoltageQ15 *v,
    const MotorParamsQ15 *params) {
    CurrentStateQ15 x_next;
    x_next.i_alpha = q30_mac_q15(0, params->a_coeff, x->i_alpha);
    x_next.i_alpha = q30_mac_q15(x_next.i_alpha, params->b_coeff, v->v_alpha);
    x_next.i_beta = q30_mac_q15(0, params->a_coeff, x->i_beta);
    x_next.i_beta = q30_mac_q15(x_next.i_beta, params->b_coeff, v->v_beta);
    return x_next;
}

/**
 * @brief 代价计算（热点函数，放入 ITCM）
 */
SECTION_ITCM
extern void compute_cost_itcm(
    const CurrentStateQ15 *i_pred,
    const CurrentRefQ15 *i_ref,
    const MpcCostQ15 *weights,
    MpcCostQ15 *cost
);

/* ============================================================================
 * 对齐函数
 * ============================================================================ */

/**
 * @brief 将指针向上对齐到缓存行边界
 */
static inline void* align_cache_line_up(void *ptr) {
    return (void*)(((uintptr_t)ptr + CACHE_LINE_MASK) & ~CACHE_LINE_MASK);
}

/**
 * @brief 将指针向下对齐到缓存行边界
 */
static inline void* align_cache_line_down(void *ptr) {
    return (void*)((uintptr_t)ptr & ~CACHE_LINE_MASK);
}

/**
 * @brief 将地址对齐到指定边界
 */
static inline uintptr_t align_up(uintptr_t addr, size_t alignment) {
    size_t mask = alignment - 1;
    return (addr + mask) & ~mask;
}

/**
 * @brief 计算对齐所需的填充
 */
static inline size_t padding_to_align(size_t size, size_t alignment) {
    size_t mask = alignment - 1;
    return (alignment - (size & mask)) & mask;
}

/* ============================================================================
 * 内存屏障（确保指令顺序）
 ============================================================================ */

#if defined(__GNUC__)
    #define MEMORY_BARRIER() __asm__ volatile("" ::: "memory")
    #define DATA_SYNC_BARRIER() __asm__ volatile("dsb" ::: "memory")
    #define INST_SYNC_BARRIER() __asm__ volatile("isb" ::: "memory")
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
    #define MEMORY_BARRIER() __memory_changed()
    #define DATA_SYNC_BARRIER() __dsb(15)
    #define INST_SYNC_BARRIER() __isb(15)
#else
    #define MEMORY_BARRIER()
    #define DATA_SYNC_BARRIER()
    #define INST_SYNC_BARRIER()
#endif

/* ============================================================================
 * 缓存维护操作
 ============================================================================ */

#if defined(__CORTEX_M) && (__CORTEX_M >= 0x07)
    /* Cortex-M7 缓存操作 */
    #include <core_cm7.h>

    /**
     * @brief 清除 D-Cache（写入数据到内存）
     */
    static inline void dcache_clean(void *addr, uint32_t size) {
        SCB_CleanDCache_by_Addr((uint32_t*)addr, size);
    }

    /**
     * @brief 使 D-Cache 失效（丢弃缓存数据）
     */
    static inline void dcache_invalidate(void *addr, uint32_t size) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);
    }

    /**
     * @brief 清除并使 D-Cache 失效
     */
    static inline void dcache_clean_invalidate(void *addr, uint32_t size) {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)addr, size);
    }
#else
    /* 无缓存或不支持的操作 */
    #define dcache_clean(addr, size) ((void)0)
    #define dcache_invalidate(addr, size) ((void)0)
    #define dcache_clean_invalidate(addr, size) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* MPC_MEMORY_LAYOUT_H */

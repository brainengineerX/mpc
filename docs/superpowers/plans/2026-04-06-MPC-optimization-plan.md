# MPC电机控制系统优化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将MPC电机控制系统从仿真原型升级为高性能、可移植到STM32的工业级控制系统

**Architecture:** 保持现有分层架构（app/core/modules/hal），逐步替换核心算法（MPC求解器、观测器），添加新模块（热管理、在线辨识），优化数学计算，完善测试和文档

**Tech Stack:** C11, OSQP (QP求解器), Unity (测试), Doxygen (文档), STM32 HAL (硬件)

---

## 项目现状分析

### 代码统计
- 33个 C/H 源文件
- 约 3,957 行代码
- 主要模块：MPC控制器、观测器、电机模型、安全模块

### 当前实现特点
- MPC采用简化穷举搜索（非最优QP求解）
- Luenberger观测器（线性近似）
- 仿真HAL层（PC端测试）
- 有限的状态机安全保护

### 技术债务
1. MPC求解器不是真正的QP优化
2. 缺少快速数学函数（sin/cos查表）
3. 观测器未考虑非线性（电感饱和）
4. 缺少热管理模块
5. 文档和测试覆盖不足
6. 代码风格不统一

---

## 实施阶段总览

| 阶段 | 名称 | 周期 | 主要目标 | 关键产出 |
|------|------|------|----------|----------|
| 0 | 基础准备 | 3天 | 建立开发规范、工具链 | 代码规范、CI配置、README |
| 1 | 代码质量 | 1周 | 统一代码风格、添加基础测试 | 格式化代码、单元测试框架 |
| 2 | 性能基础 | 2周 | 数学优化、编译优化 | 快速数学库、性能基线 |
| 3 | MPC升级 | 3周 | 集成OSQP求解器 | 真MPC求解、约束处理 |
| 4 | 观测器升级 | 2周 | EKF实现、非线性补偿 | 状态估计精度提升 |
| 5 | 高级功能 | 2周 | 热管理、在线辨识 | 热保护、参数自适应 |
| 6 | 硬件准备 | 2周 | STM32 HAL、移植指南 | 可运行目标代码 |
| 7 | 系统验证 | 2周 | 集成测试、性能调优 | 完整测试报告 |

**总计: 约19周 (4.5个月)**

---

## Phase 0: 基础准备 (3天)

### 目标
建立开发规范和工具链，为后续优化工作奠定基础。

### Task 0.1: 项目文档初始化

**Files:**
- Create: `README.md`
- Create: `.gitignore`
- Create: `docs/ARCHITECTURE.md`

- [ ] **Step 1: 创建项目README**

```markdown
# MPC电机控制系统

高性能永磁同步电机(PMSM)模型预测控制(MPC)实现。

## 特性
- 真正的QP-based MPC求解（OSQP）
- 扩展卡尔曼滤波(EKF)状态估计
- 热管理和在线参数辨识
- STM32硬件移植支持

## 快速开始
\`\`\`bash
make all
./mpc_demo
\`\`\`

## 文档
- [架构设计](docs/ARCHITECTURE.md)
- [API参考](docs/API.md)
- [调优指南](docs/TUNING.md)
\`\`\`

- [ ] **Step 2: 创建.gitignore**

```gitignore
# 构建输出
build/
*.o
*.elf
mpc_demo
identify_demo

# IDE
.vscode/
.idea/
*.swp
.DS_Store

# 文档生成
docs/html/
\`\`\`

- [ ] **Step 3: 提交初始文档**

```bash
git add README.md .gitignore
git commit -m "docs: add project README and .gitignore"
\`\`\`

### Task 0.2: 代码风格配置

**Files:**
- Create: `.clang-format`

- [ ] **Step 1: 创建.clang-format**

```yaml
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
PointerAlignment: Left
SortIncludes: true
\`\`\`

- [ ] **Step 2: 提交配置**

```bash
git add .clang-format
git commit -m "style: add clang-format configuration"
\`\`\`

### 验收标准
- [ ] README.md 包含项目描述、快速开始、文档链接
- [ ] .gitignore 包含构建产物和IDE文件
- [ ] .clang-format 配置了统一的代码风格
- [ ] 所有配置已提交到git

---

## Phase 1: 代码质量 (1周)

### 目标
统一代码风格，建立基础测试框架，提升代码可维护性。

### Task 1.1: 代码格式化

**Files:**
- Modify: All `.c` and `.h` files in the project

- [ ] **Step 1: 运行clang-format**

```bash
find . -name "*.c" -o -name "*.h" | xargs clang-format -i
\`\`\`

- [ ] **Step 2: 检查格式化结果**

```bash
git diff --stat
\`\`\`
Expected: 显示所有被修改的文件

- [ ] **Step 3: 提交格式化代码**

```bash
git add -A
git commit -m "style: format all source files with clang-format"
\`\`\`

### Task 1.2: 静态分析配置

**Files:**
- Create: `.cppcheck`
- Create: `scripts/static-analysis.sh`

- [ ] **Step 1: 创建cppcheck抑制文件**

```
// 抑制第三方代码和系统头的警告
*:*.h
*:/usr/*
\`\`\`

- [ ] **Step 2: 创建静态分析脚本**

```bash
#!/bin/bash
echo "Running static analysis..."

cppcheck --enable=all \
         --suppress=missingIncludeSystem \
         --suppress=unusedFunction \
         --error-exitcode=1 \
         -I. \
         core/ modules/ app/ hal/ 2>&1 | head -50

echo "Static analysis complete."
\`\`\`

- [ ] **Step 3: 使脚本可执行并提交**

```bash
chmod +x scripts/static-analysis.sh
git add .cppcheck scripts/
git commit -m "build: add static analysis configuration"
\`\`\`

### Task 1.3: 单元测试框架设置

**Files:**
- Create: `tests/Makefile`
- Create: `tests/test_motor_model.c`
- Create: `tests/unity/unity.c`
- Create: `tests/unity/unity.h`

- [ ] **Step 1: 下载Unity测试框架**

```bash
mkdir -p tests/unity
curl -L https://raw.githubusercontent.com/ThrowTheSwitch/Unity/master/src/unity.c > tests/unity/unity.c
curl -L https://raw.githubusercontent.com/ThrowTheSwitch/Unity/master/src/unity.h > tests/unity/unity.h
curl -L https://raw.githubusercontent.com/ThrowTheSwitch/Unity/master/src/unity_internals.h > tests/unity/unity_internals.h
\`\`\`

- [ ] **Step 2: 创建测试Makefile**

```makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g -I../core/model -I./unity

TEST_SRCS = test_motor_model.c ../core/model/motor_model.c
UNITY_SRCS = unity/unity.c

TEST_OBJS = $(TEST_SRCS:.c=.o)
UNITY_OBJS = $(UNITY_SRCS:.c=.o)

test_runner: $(TEST_OBJS) $(UNITY_OBJS)
	$(CC) -o $@ $^ -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: test_runner
	./test_runner

clean:
	rm -f *.o ../*/*.o unity/*.o test_runner

.PHONY: test clean
\`\`\`

- [ ] **Step 3: 创建第一个测试文件**

```c
#include "unity.h"
#include "motor_model.h"

void setUp(void) {
    // 在每个测试前执行
}

void tearDown(void) {
    // 在每个测试后执行
}

void test_motor_model_init_should_set_default_params(void) {
    MotorModel model;
    MotorParams params = {
        .Rs = 0.5f,
        .Ld = 0.001f,
        .Lq = 0.001f,
        .psi_f = 0.1f,
        .J = 0.001f,
        .pole_pairs = 4
    };
    
    motor_model_init(&model, &params);
    
    TEST_ASSERT_EQUAL_FLOAT(0.5f, model.params.Rs);
    TEST_ASSERT_EQUAL_FLOAT(0.001f, model.params.Ld);
}

void test_motor_model_predict_should_update_state(void) {
    MotorModel model;
    MotorParams params = {
        .Rs = 0.5f,
        .Ld = 0.001f,
        .Lq = 0.001f,
        .psi_f = 0.1f,
        .J = 0.001f,
        .pole_pairs = 4
    };
    
    motor_model_init(&model, &params);
    
    MotorState state = {
        .id = 0.0f,
        .iq = 0.0f,
        .theta = 0.0f,
        .omega = 0.0f
    };
    
    MotorState next_state;
    motor_model_predict(&model, &state, 10.0f, 0.0f, 100e-6f, &next_state);
    
    // 施加vd=10V后，id应该增加
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, next_state.id);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_motor_model_init_should_set_default_params);
    RUN_TEST(test_motor_model_predict_should_update_state);
    return UNITY_END();
}
\`\`\`

- [ ] **Step 4: 运行测试并提交**

```bash
cd tests
make test
# 期望输出：所有测试通过
cd ..
git add tests/
git commit -m "test: add Unity test framework and motor model tests"
\`\`\`

### 验收标准
- [ ] 所有源代码已通过clang-format格式化
- [ ] 静态分析脚本可运行且无严重错误
- [ ] Unity测试框架已集成
- [ ] 至少有一个单元测试运行通过
- [ ] 所有变更已提交到git

---

## 总结

本计划涵盖了MPC电机控制系统从当前状态到工业级系统的完整优化路径。

**关键里程碑：**
1. **2周内**：代码质量提升，测试框架就位
2. **6周内**：核心算法升级（MPC+观测器）
3. **10周内**：高级功能（热管理、在线辨识）
4. **14周内**：硬件移植准备完成
5. **19周内**：系统验证，生产就绪

**成功关键因素：**
- 每个阶段都有明确的验收标准
- 渐进式改进，降低风险
- 充分的测试覆盖
- 文档同步更新

建议立即开始Phase 0，建立开发基础规范。
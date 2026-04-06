# 工程目录结构（中文说明）

本工程已经按“可移植、可维护、可扩展”进行分层：

- `app/`
  - `main.c`：应用入口，负责辨识流程、模块初始化、控制主循环调度。

- `core/model/`
  - `motor_model.h/.c`：电机数学模型（完整PMSM + MPC简化模型）。

- `core/control/`
  - `motor_control.h/.c`：电流环MPC核心（候选生成、预测、代价、优化）。

- `core/identify/`
  - `motor_identification.h/.c`：参数辨识（电参数、机械参数）。

- `modules/compensation/`
  - `inverter_compensation.h/.c`：死区补偿、器件压降补偿。

- `modules/sensing/`
  - `adc_calibration.h/.c`：电流零偏/比例在线标定、ADC延时补偿。

- `modules/observer/`
  - `observers.h/.c`：扰动观测、负载转矩观测、Rs自适应、速度滤波与位置插值。

- `modules/safety/`
  - `safety_manager.h/.c`：过流/过压/失速保护与降级策略。

- `modules/fallback/`
  - `pi_fallback.h/.c`：MPC异常时的PI后备控制器。

- `hal/`
  - `control_hal.h`：硬件抽象接口定义（算法与BSP解耦边界）。
  - `control_hal_sim.h/.c`：仿真实现（后续可替换为GD32 HAL/BSP实现）。

## 迁移到 GD32F450 的建议

1. 新增 `hal/control_hal_gd32.h/.c`，实现 `ControlHal` 四个函数接口。  
2. 将 `app/main.c` 中 `control_hal_sim_bind()` 替换为 `control_hal_gd32_bind()`。  
3. 保持 `core/` 与 `modules/` 算法代码不改，仅做参数整定。  
4. 在中断与后台任务中分频调用（电流环高频，观测与自适应低频）。

CC = gcc
# 通用优化标志
CFLAGS = -std=c11 -O3 -Wall -Wextra -pedantic \
         -ffast-math -funroll-loops -fomit-frame-pointer

# Cortex-M7 特定优化 (可选，用于交叉编译)
CFLAGS_CORTEX_M7 = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard \
                   -mthumb -O3 -ffast-math -funroll-loops \
                   -DARM_MATH_CM7 -D__FPU_PRESENT=1

# ITCM/DTCM 支持链接选项 (用于嵌入式链接器)
LDFLAGS_CORTEX_M7 = -Tstm32h743xx_flash.ld \
                    -Wl,--gc-sections \
                    -Wl,-Map=output.map
INCLUDES = -Iapp -Icore/model -Icore/control -Icore/math -Icore/identify -Ihal \
           -Imodules/compensation -Imodules/sensing -Imodules/observer \
           -Imodules/safety -Imodules/fallback -Imodules/outer_loop -Iui
LDFLAGS = -lm

BUILD_DIR = build
CONTROL_TARGET = mpc_demo
IDENTIFY_TARGET = identify_demo

# 定点化 MPC 源文件
FIXED_POINT_SRCS = core/math/fixed_point_math.c \
                   core/control/motor_model_fixed.c \
                   core/control/mpc_controller_fixed.c

BASE_SRCS = core/model/motor_model.c \
            core/identify/motor_identification.c \
            modules/sensing/adc_calibration.c \
            hal/control_hal_sim.c \
            app/app_hal_utils.c \
            app/app_identification.c \
            app/motor_pu_profile.c \
            ui/ui_panel.c

CONTROL_SRCS = app/control_main.c \
               core/control/motor_control.c \
               $(FIXED_POINT_SRCS) \
               modules/compensation/inverter_compensation.c \
               modules/observer/observers.c \
               modules/outer_loop/cascade_pi.c \
               modules/outer_loop/control_interface.c \
               modules/safety/safety_manager.c \
               modules/fallback/pi_fallback.c \
               $(BASE_SRCS)

IDENTIFY_SRCS = app/identify_main.c \
                $(BASE_SRCS)

CONTROL_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CONTROL_SRCS))
IDENTIFY_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(IDENTIFY_SRCS))

all: $(CONTROL_TARGET) $(IDENTIFY_TARGET)

$(CONTROL_TARGET): $(CONTROL_OBJS)
	$(CC) $(CONTROL_OBJS) -o $@ $(LDFLAGS)

$(IDENTIFY_TARGET): $(IDENTIFY_OBJS)
	$(CC) $(IDENTIFY_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(CONTROL_TARGET) $(IDENTIFY_TARGET) $(BUILD_DIR)

.PHONY: all clean

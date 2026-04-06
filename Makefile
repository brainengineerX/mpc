CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -pedantic
INCLUDES = -Iapp -Icore/model -Icore/control -Icore/identify -Ihal \
           -Imodules/compensation -Imodules/sensing -Imodules/observer \
           -Imodules/safety -Imodules/fallback -Imodules/outer_loop -Iui
LDFLAGS = -lm

BUILD_DIR = build
CONTROL_TARGET = mpc_demo
IDENTIFY_TARGET = identify_demo

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

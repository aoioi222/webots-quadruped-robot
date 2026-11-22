# Compile the active firmware sources directly. The older flat Webots sources
# in this directory are kept for reference.
HAL_ROBOT_DIR ?= ../../../HAL/RCS_Template_F407/RCS/user/robot
HAL_MODULES = robot_task.c robot_state.c Foot_structure.c bezier_foot_tra.c \
              body_control.c RCS_PIDctrl.c gravity_comp.c \
              Calculate/Inverse_kinematics_calculate.c \
              Calculate/Forward_kinematics_calculate.c \
              gait_controller/gait_controller.c gait_controller/gait_generator.c \
              gait_controller/gait_scheduler.c gait_controller/leg_state_machine.c \
              remote_controller/rc_event_dispatch.c remote_controller/jump_controller.c \
              remote_controller/remote_control_sbus.c
HAL_SOURCES := $(addprefix $(HAL_ROBOT_DIR)/,$(HAL_MODULES))
C_SOURCES = hal_webots/webots_platform.c hal_webots/virtual_sbus.c \
            hal_webots/webots_ik_checks.c $(HAL_SOURCES)
INCLUDE = -I"$(HAL_ROBOT_DIR)" -I"$(HAL_ROBOT_DIR)/Calculate" \
          -I"$(HAL_ROBOT_DIR)/gait_controller" -I"$(HAL_ROBOT_DIR)/remote_controller" \
          -I"$(HAL_ROBOT_DIR)/IMU" -Ihal_webots
CFLAGS += -include hal_webots/quadruped_v11_1_config.h
override BUILD_DIR = build_hal
NO_FAT_BINARY = 1

ifeq ($(wildcard $(HAL_ROBOT_DIR)/robot_task.c),)
$(error Set HAL_ROBOT_DIR to the HAL firmware's RCS/user/robot directory)
endif

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Webots uses bare basenames to find sources. Pin these paths so GNU make
# never picks up an older copy with the same name in the controller directory.
ifdef GOAL
define hal_source_rule
$(BUILD_GOAL_DIR)/$(notdir $(1:.c=.o)): $(1) Makefile
	@echo "# compiling HAL" $(notdir $(1))
	$(SILENT)$(CC) -c $(WBCFLAGS) $(CFLAGS) $(INCLUDE) "$(1)" -o $$@

$(BUILD_GOAL_DIR)/$(notdir $(1:.c=.d)): $(1) Makefile
	$(SILENT)$(CC) $(INCLUDE) $(WBCFLAGS) $(CFLAGS) -MM -MP \
	    -MT $(BUILD_GOAL_DIR)/$(notdir $(1:.c=.o)) -MT $$@ "$(1)" > $$@
endef
$(foreach src,$(HAL_SOURCES),$(eval $(call hal_source_rule,$(src))))
endif

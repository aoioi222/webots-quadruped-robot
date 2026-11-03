#ifndef JUMP_CONTROLLER_H
#define JUMP_CONTROLLER_H

#include "include_head.h"
#include "Foot_structure.h"

typedef enum {
    JUMP_IDLE = 0,
    JUMP_CROUCH,   // 蓄力下蹲
    JUMP_PUSH_OFF, // 蹬地腾空
    JUMP_AERIAL,   // 空中调整
    JUMP_LANDING,  // 落地缓冲
    JUMP_RECOVER,  // 恢复站立
    JUMP_DONE
} JumpPhase_t;

typedef struct {
    JumpPhase_t phase;
    double phase_start_time;
    uint8_t active;
    uint8_t completed;
    LegJointAngles_t output_joints[4]; // 当前帧输出的目标关节角度
} JumpController_t;

// 全局跳跃控制器
extern JumpController_t g_jump_ctrl;

void jump_controller_init(JumpController_t *jc);
void jump_controller_trigger(JumpController_t *jc, double current_time);
void jump_controller_update(JumpController_t *jc, double current_time);
uint8_t jump_controller_is_active(const JumpController_t *jc);

#endif

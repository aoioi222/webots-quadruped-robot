#ifndef BODY_CONTROL_H
#define BODY_CONTROL_H

#include "include_head.h"
#include "leg_state_machine.h"
#include "robot_state.h"

// 姿态PID控制器参数
typedef struct
{
    float kp_pitch;            // 俯仰角比例系数
    float kd_pitch;            // 俯仰角微分系数
    float kp_roll;             // 横滚角比例系数
    float kd_roll;             // 横滚角微分系数
    float max_correction;      // 姿态修正最大Z轴偏移量(m)
    float kp_yaw;              // 偏航角比例系数(m/rad)
    float ki_yaw;              // 偏航角积分系数(m/(rad·s))，抗静态误差
    float kd_yaw;              // 偏航角微分系数(m·s/rad)
    float max_yaw_step;        // 偏航角单次最大修正步长(m)
    float max_yaw_integral;    // 偏航角积分限幅，防止积分饱和
    float dead_zone_pitch_rad; // 俯仰角死区(rad)，小于该值不修正
    float dead_zone_roll_rad;  // 横滚角死区(rad)
    float dead_zone_yaw_rad;   // 偏航角死区(rad)
} AttitudePDConfig_t;

typedef struct
{
    float x;                // ZMP点x轴坐标(m)
    float y;                // ZMP点y轴坐标(m)
    bool is_stable;         // 是否稳定
    float stability_margin; // 稳定裕度(m)，裕度越大越稳定
} ZMPData_t;

// 初始化姿态控制参数
void body_attitude_control_init(void);

// 姿态控制更新，每帧调用
void body_attitude_control_update(LegStateMachine_t *leg_sm[4]);

// 将姿态修正量应用到支撑腿，在foot_lerp之后、IK之前调用
void body_apply_to_stance_leg(LegStateMachine_t *sm);

// 获取当前姿态修正量
void body_get_correction(float *pitch_correction, float *roll_correction);

// 获取当前偏航修正量
float body_get_yaw_correction(void);

// 重置俯仰目标偏移量
void body_reset_pitch_target(void);

// 偏航目标初始化标志（外部可重置，用于站立等场景）
extern uint8_t g_yaw_target_initialized;
extern float g_yaw_manual_offset;
extern float g_pitch_target; // 手动俯仰目标偏移量 (rad)

// 计算ZMP
void zmp_calculate(ZMPData_t *zmp_out, LegStateMachine_t *leg_sm[4]);

#endif
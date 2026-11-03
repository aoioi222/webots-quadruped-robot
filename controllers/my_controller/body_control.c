#include "body_control.h"
#include <math.h>
#include <stdio.h>

// 默认PD参数（值定义在 robot_config.h）
const AttitudePDConfig_t ATTITUDE_PD_DEFAULT = {
    .kp_pitch = PD_KP_PITCH,
    .kd_pitch = PD_KD_PITCH,
    .kp_roll = PD_KP_ROLL,
    .kd_roll = PD_KD_ROLL,
    .max_correction = PD_MAX_CORRECTION,
    .kp_yaw = PD_KP_YAW,
    .ki_yaw = PD_KI_YAW,
    .kd_yaw = PD_KD_YAW,
    .max_yaw_step = PD_MAX_YAW_STEP,
    .max_yaw_integral = PD_MAX_YAW_INTEGRAL,
    .dead_zone_pitch_rad = PD_DZ_PITCH_RAD,
    .dead_zone_roll_rad = PD_DZ_ROLL_RAD,
    .dead_zone_yaw_rad = PD_DZ_YAW_RAD};

const AttitudePDConfig_t ATTITUDE_PD_WALK = {
    .kp_pitch = PD_WALK_KP_PITCH,
    .kd_pitch = PD_WALK_KD_PITCH,
    .kp_roll = PD_WALK_KP_ROLL,
    .kd_roll = PD_WALK_KD_ROLL,
    .max_correction = PD_WALK_MAX_CORRECTION,
    .kp_yaw = PD_WALK_KP_YAW,
    .ki_yaw = PD_WALK_KI_YAW,
    .kd_yaw = PD_WALK_KD_YAW,
    .max_yaw_step = PD_WALK_MAX_YAW_STEP,
    .max_yaw_integral = PD_WALK_MAX_YAW_INTEGRAL,
    .dead_zone_pitch_rad = PD_WALK_DZ_PITCH_RAD,
    .dead_zone_roll_rad = PD_WALK_DZ_ROLL_RAD,
    .dead_zone_yaw_rad = PD_WALK_DZ_YAW_RAD};

AttitudePDConfig_t g_attitude_pd;
float g_last_pitch_error = 0.0f;
float g_last_roll_error = 0.0f;

float g_current_pitch_correction = 0.0f;
float g_current_roll_correction = 0.0f;

float g_yaw_target = 0.0f;
float g_last_yaw_error = 0.0f;
float g_yaw_error_integral = 0.0f;
float g_current_yaw_correction = 0.0f;
float g_yaw_manual_offset = 0.0f;
uint8_t g_yaw_target_initialized = 0;
float g_pitch_target = 0.0f; // 手动俯仰目标偏移量 (rad)，[键后仰/]键低头

/**
 * @brief  机身姿态控制初始化
 */
void body_attitude_control_init(void)
{
    // 使用柔和姿态参数（偏航增益与DEFAULT相同，pitch/roll近零，避免Z轴扰动产生额外偏航力矩）
    g_attitude_pd = ATTITUDE_PD_WALK;
    g_last_pitch_error = 0.0f;
    g_last_roll_error = 0.0f;
    g_last_yaw_error = 0.0f;
    g_yaw_error_integral = 0.0f;
    g_current_yaw_correction = 0.0f;
    g_yaw_manual_offset = 0.0f;
    g_yaw_target_initialized = 0;
    g_pitch_target = 0.0f;
}

/**
 * @brief  机身姿态PD控制更新（在支撑相修正足端Z高度）
 * @param  leg_sm[4] 四条腿状态机指针数组
 */
void body_attitude_control_update(LegStateMachine_t *leg_sm[4])
{
    (void)leg_sm; // 参数保留兼容，实际修正已移至状态机内部 foot_lerp 之后

    if (g_robot_state.mode != ROBOT_MODE_QUADRUPED)
        return;

    // 姿态误差：目标姿态为 g_pitch_target（0为水平），带死区
    float pitch_error = g_pitch_target - g_robot_state.body_pitch;
    float roll_error = 0.0f - g_robot_state.body_roll;
    // 俯仰死区
    if (fabsf(pitch_error) < g_attitude_pd.dead_zone_pitch_rad)
        pitch_error = 0.0f;
    else
        pitch_error -= (pitch_error > 0.0f ? g_attitude_pd.dead_zone_pitch_rad : -g_attitude_pd.dead_zone_pitch_rad);
    // 横滚死区
    if (fabsf(roll_error) < g_attitude_pd.dead_zone_roll_rad)
        roll_error = 0.0f;
    else
        roll_error -= (roll_error > 0.0f ? g_attitude_pd.dead_zone_roll_rad : -g_attitude_pd.dead_zone_roll_rad);
    // 误差微分
    float pitch_error_dot = (pitch_error - g_last_pitch_error) / g_robot_state.dt_s;
    float roll_error_dot = (roll_error - g_last_roll_error) / g_robot_state.dt_s;
    // PD输出高度修正量
    float pitch_correction = g_attitude_pd.kp_pitch * pitch_error + g_attitude_pd.kd_pitch * pitch_error_dot;
    float roll_correction = g_attitude_pd.kp_roll * roll_error + g_attitude_pd.kd_roll * roll_error_dot;
    // 限幅
    pitch_correction = fminf(fmaxf(pitch_correction, -g_attitude_pd.max_correction),
                             g_attitude_pd.max_correction);
    roll_correction = fminf(fmaxf(roll_correction, -g_attitude_pd.max_correction),
                            g_attitude_pd.max_correction);

    g_current_pitch_correction = pitch_correction;
    g_current_roll_correction = roll_correction;

    // printf("[姿态控制] pitch=%.4f rad | roll=%.4f rad | dz_pitch=%.4f | dz_roll=%.4f\n",
    //     g_robot_state.body_pitch, g_robot_state.body_roll,
    //     pitch_correction, roll_correction);

    // 偏航控制：速度方向决定目标偏航 + 手动旋转偏移
    if (!g_yaw_target_initialized)
    {
        g_yaw_target = g_robot_state.body_yaw;
        g_yaw_manual_offset = g_robot_state.body_yaw;
        g_yaw_target_initialized = 1;
    }

    if (fabsf(g_robot_state.cmd_vel_yaw) > 1e-6f)
    {
        // 手动旋转模式（Q/E按下）：禁用偏航PID，持续跟踪朝向偏移
        g_current_yaw_correction = 0.0f;
        g_yaw_manual_offset = g_robot_state.body_yaw;
        g_yaw_target = g_robot_state.body_yaw;
        g_yaw_error_integral = 0.0f;
        g_last_yaw_error = 0.0f;
        // printf("[偏航] 手动旋转中, offset=%.3f rad\n", g_yaw_manual_offset);
    }
    else
    {
        // 从速度方向计算期望朝向
        float desired_heading = 0.0f;
        if (fabsf(g_robot_state.cmd_vel_x) > 1e-6f)
        {
            desired_heading = atan2f(g_robot_state.cmd_vel_y, g_robot_state.cmd_vel_x);
        }
        g_yaw_target = g_yaw_manual_offset + desired_heading;

        float yaw_error_raw = g_yaw_target - g_robot_state.body_yaw;
        while (yaw_error_raw > pi)
            yaw_error_raw -= 2.0f * pi;
        while (yaw_error_raw < -pi)
            yaw_error_raw += 2.0f * pi;

        // P/D过死区防抖，I用原始误差消除稳态漂移
        float yaw_error_pd = yaw_error_raw;
        if (fabsf(yaw_error_pd) < g_attitude_pd.dead_zone_yaw_rad)
            yaw_error_pd = 0.0f;
        else
            yaw_error_pd -= (yaw_error_pd > 0.0f ? g_attitude_pd.dead_zone_yaw_rad : -g_attitude_pd.dead_zone_yaw_rad);

        float yaw_error_dot = (yaw_error_pd - g_last_yaw_error) / g_robot_state.dt_s;

        // I项始终用原始误差累积，死区内也能缓慢消除系统偏置
        g_yaw_error_integral += yaw_error_raw * g_robot_state.dt_s;
        g_yaw_error_integral = fminf(fmaxf(g_yaw_error_integral, -g_attitude_pd.max_yaw_integral),
                                     g_attitude_pd.max_yaw_integral);

        g_current_yaw_correction = g_attitude_pd.kp_yaw * yaw_error_pd + g_attitude_pd.ki_yaw * g_yaw_error_integral + g_attitude_pd.kd_yaw * yaw_error_dot;
        g_current_yaw_correction = fminf(fmaxf(g_current_yaw_correction, -g_attitude_pd.max_yaw_step),
                                         g_attitude_pd.max_yaw_step);

        // printf("[偏航PID] target=%.3f desired=%.3f offset=%.3f actual=%.3f err_raw=%.4f err_pd=%.4f I=%.4f out=%.4f m\n",
        //        g_yaw_target, desired_heading, g_yaw_manual_offset, g_robot_state.body_yaw, yaw_error_raw, yaw_error_pd, g_yaw_error_integral, g_current_yaw_correction);

        g_last_yaw_error = yaw_error_pd;
    }

    // 更新上一帧误差（pitch/roll 始终更新，偏航在各自分支内更新）
    g_last_pitch_error = pitch_error;
    g_last_roll_error = roll_error;
}

/**
 * @brief  对单条支撑腿叠加姿态修正量（在 foot_lerp 之后、IK 之前调用）
 * @param  sm 单条腿状态机指针
 */
void body_apply_to_stance_leg(LegStateMachine_t *sm)
{
    if (sm == NULL || sm->current_state != LEG_STATE_STANCE)
        return;

    float height_correction = 0.0f;

    // 俯仰修正：前腿和后腿反向
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_L_F)
        height_correction -= g_current_pitch_correction;
    else
        height_correction += g_current_pitch_correction;

    // 横滚修正：左腿和右腿反向
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_R_B)
        height_correction += g_current_roll_correction;
    else
        height_correction -= g_current_roll_correction;

    // 手动俯仰目标直接几何换算为Z偏移（小角度近似: Z ≈ pitch * hip_x）
    float pitch_geom_z = g_pitch_target * FOOT_OFFSET_X;
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_L_F)
        height_correction -= pitch_geom_z;
    else
        height_correction += pitch_geom_z;

    sm->current_target_pos.foot_pos_z_m += height_correction;
}
/**
 * @brief  获取当前姿态修正量
 */
void body_get_correction(float *pitch_correction, float *roll_correction)
{
    if (pitch_correction != NULL)
        *pitch_correction = g_current_pitch_correction;
    if (roll_correction != NULL)
        *roll_correction = g_current_roll_correction;
}

float body_get_yaw_correction(void)
{
    return g_current_yaw_correction;
}

void body_reset_pitch_target(void)
{
    g_pitch_target = 0.0f;
}

/**
 * @brief  计算零力矩点ZMP与稳定裕度
 * @param  zmp_out 输出ZMP信息
 * @param  leg_sm[4] 四条腿状态机
 */
void zmp_calculate(ZMPData_t *zmp_out, LegStateMachine_t *leg_sm[4])
{
    float total_force = 0.0f;
    float zmp_x = 0.0f;
    float zmp_y = 0.0f;

    float min_x = 100.0f;
    float max_x = -100.0f;
    float min_y = 100.0f;
    float max_y = -100.0f;

    // 遍历支撑腿，计算支撑多边形与受力加权中心
    for (int i = 0; i < 4; i++)
    {
        if (leg_sm[i]->current_state == LEG_STATE_STANCE)
        {
            float x = leg_sm[i]->current_target_pos.foot_pos_x_m;
            float y = leg_sm[i]->current_target_pos.foot_pos_y_m;
            float force = 1.0f; // 无足力传感器时默认均摊

            total_force += force;
            zmp_x += x * force;
            zmp_y += y * force;

            // 更新支撑多边形边界
            min_x = fminf(min_x, x);
            max_x = fmaxf(max_x, x);
            min_y = fminf(min_y, y);
            max_y = fmaxf(max_y, y);
        }
    }

    // 无支撑腿，直接判定不稳定
    if (total_force < 0.1f)
    {
        zmp_out->is_stable = false;
        zmp_out->stability_margin = 0.0f;
        zmp_out->x = 0.0f;
        zmp_out->y = 0.0f;
        return;
    }

    // 计算ZMP坐标
    zmp_x /= total_force;
    zmp_y /= total_force;

    zmp_out->x = zmp_x;
    zmp_out->y = zmp_y;

    // 判断ZMP是否在支撑多边形内部
    zmp_out->is_stable = (zmp_x >= min_x && zmp_x <= max_x) && (zmp_y >= min_y && zmp_y <= max_y);

    // 稳定裕度：到支撑多边形边界的最小距离
    float margin_x = fminf(zmp_x - min_x, max_x - zmp_x);
    float margin_y = fminf(zmp_y - min_y, max_y - zmp_y);
    zmp_out->stability_margin = fminf(margin_x, margin_y);
}
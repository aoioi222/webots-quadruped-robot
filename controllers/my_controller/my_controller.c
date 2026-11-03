#include "include_head.h"
#include "robot_state.h"
#include "bezier_foot_tra.h"
#include "Foot_structure.h"
#include "Jacobian_Matrix.h"
#include "gait_generator.h"
#include "gait_scheduler.h"
#include "gait_controller.h"
#include "leg_state_machine.h"
#include "body_control.h"
#include "jump_controller.h"
#include "remote_control.h"
#include "robot_config.h"

// TIME_STEP、MOTOR_VELOCITY、INIT_WAIT_TIME 已集中到 robot_config.h

// 电机设备结构体
typedef struct
{
    WbDeviceTag j1[4];
    WbDeviceTag j2[4];
    WbDeviceTag j3[4];
    WbDeviceTag wheel[4]; // 足端轮式电机
    int has_wheels;       // 轮式电机是否可用
} LegMotors;
LegStateMachine_t *leg_sm[4] = {
    &leg_fsm[LEG_R_F],
    &leg_fsm[LEG_L_F],
    &leg_fsm[LEG_R_B],
    &leg_fsm[LEG_L_B]};
// 记录电机速度是否已切换到跳跃模式
static int motors_in_jump_mode = 0;

// 关机和折叠状态
static int g_shutdown = 0;
static int fold_active = 0;
static int g_gait_active = 0; // 步态周期是否已启动（O键触发）
static int g_wheeled_mode = 0; // 轮式模式标志
static int wheel_transition = 0; // 轮式切换过渡动画进行中
static double wheel_trans_start_time = 0.0;
static LegJointAngles_t wheel_start_angles[4];
static int stand_transition = 0; // 站立过渡动画进行中
static double stand_trans_start_time = 0.0;
static LegJointAngles_t stand_start_angles[4];
static LegJointAngles_t fold_start_angles[4];
static double fold_start_time = 0.0;

// 轮式目标角度（预重映射空间，与 IK 解算器输出约定一致：右腿J2正/J3负，左腿J2负/J3正）
static const LegJointAngles_t WHEEL_TARGET_ANGLES[4] = {
    [LEG_R_F] = {0.0f,  WHEEL_J2, -WHEEL_J3, 1},
    [LEG_L_F] = {0.0f, -WHEEL_J2,  WHEEL_J3, 1},
    [LEG_R_B] = {0.0f,  WHEEL_J2, -WHEEL_J3, 1},
    [LEG_L_B] = {0.0f, -WHEEL_J2,  WHEEL_J3, 1},
};

// 站立目标角度（预重映射空间，与 IK 解算器输出约定一致）
static const LegJointAngles_t STAND_TARGET_ANGLES[4] = {
    [LEG_R_F] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
    [LEG_L_F] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
    [LEG_R_B] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
    [LEG_L_B] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
};
#define WHEEL_TRANS_DUR 0.5f  // 轮式姿态切换过渡时长 (s)

// 折叠目标角度（预重映射空间，符号约定与 leg_state_machine_init 一致）
// RF/RB: J2正(remap反转→实际负=前折) J3负(直通→实际负=膝弯)
// LF/LB: J2负(直通→实际负=前折) J3正(remap反转→实际负=膝弯)
// J1: 正=右腿外翻(直通), 负=左腿外翻(直通); RB/RF需预取反
static const LegJointAngles_t FOLD_TARGET_ANGLES[4] = {
    [LEG_R_F] = {-FOLD_J1, FOLD_J2, -FOLD_J3, 1},
    [LEG_L_F] = {FOLD_J1, -FOLD_J2, FOLD_J3, 1},
    [LEG_R_B] = {FOLD_J1, FOLD_J2, -FOLD_J3, 1},
    [LEG_L_B] = {-FOLD_J1, -FOLD_J2, FOLD_J3, 1},
};

// ===================== 工具函数声明 =====================
static void motors_init(LegMotors *m);
static void set_leg_position(LegMotors *m, int leg_idx, LegJointAngles_t angles);
static void set_all_motor(LegMotors *m);
static void leg_direction_remap(int leg_idx, LegJointAngles_t *angles);
static void motors_set_velocity_all(LegMotors *m, float velocity);
static void motors_set_jump_mode(LegMotors *m);
static void motors_set_gait_mode(LegMotors *m);
static void wheel_motors_update(LegMotors *m);

int main(int argc, char **argv)
{
    wb_robot_init();
    LegMotors m;
    motors_init(&m);
    // 获取机体欧拉角
    WbDeviceTag imu = wb_robot_get_device("imu");
    wb_inertial_unit_enable(imu, TIME_STEP);

    // 设置电机最大速度
    for (int i = 0; i < 4; i++)
    {
        wb_motor_set_velocity(m.j1[i], MOTOR_VELOCITY);
        wb_motor_set_velocity(m.j2[i], MOTOR_VELOCITY);
        wb_motor_set_velocity(m.j3[i], MOTOR_VELOCITY);
    }

    // 按腿分别设置初始角度，与状态机内部初始值完全一致，避免跳变
    LegJointAngles_t init_angles[4] = {
        [LEG_R_F] = {STAND_J1, -STAND_J2, -STAND_J3, 1},
        [LEG_L_F] = {STAND_J1, -STAND_J2, -STAND_J3, 1},
        [LEG_R_B] = {STAND_J1, -STAND_J2, -STAND_J3, 1},
        [LEG_L_B] = {STAND_J1, -STAND_J2, -STAND_J3, 1}};

    printf("\n【初始化】各腿归位角度：\n");
    for (int i = 0; i < 4; i++)
    {
        set_leg_position(&m, i, init_angles[i]);
        printf("  腿%d: J1=%.1f° J2=%.1f° J3=%.1f°\n",
               i, init_angles[i].joint1_deg,
               init_angles[i].joint2_deg, init_angles[i].joint3_deg);
    }

    // 等待电机归位完成
    float t_wait = wb_robot_get_time();
    while (wb_robot_step(TIME_STEP) != -1)
    {
        if (wb_robot_get_time() - t_wait >= INIT_WAIT_TIME)
            break;
    }
    printf("【初始化】电机归位完成\n");

    // 机器人全局状态初始化
    robot_state_init();
    g_robot_state.mode = ROBOT_MODE_QUADRUPED;
    g_robot_state.quad_sub_mode = QUAD_MODE;
    g_robot_state.dt_s = TIME_STEP / 1000.0;
    g_robot_state.body_height = FOOT_OFFSET_Z; // 补充身体高度，步态生成基准Z坐标
    g_robot_state.cmd_vel_x = 0.0f; // 开机默认静止，O键启动步态后通过WASD控制
    g_robot_state.cmd_vel_y = 0.0f;
    g_robot_state.cmd_vel_yaw = 0.0f;

    // 步态全模块初始化
    gait_controller_init();
    jump_controller_init(&g_jump_ctrl);
    remote_control_init();
    body_attitude_control_init();
    printf("==== 初始化完成，按 O 启动步态(四足) | 按 2 启动轮式模式 | 按 1 切回四足 ====\n");

    double print_timer = 0.0;
    while (wb_robot_step(TIME_STEP) != -1)
    {
        const double *rpy_rad = wb_inertial_unit_get_roll_pitch_yaw(imu);
        g_robot_state.body_pitch = rpy_rad[1];
        g_robot_state.body_roll = rpy_rad[0];
        g_robot_state.body_yaw = rpy_rad[2]; // 建模导致的方向映射

        // 核心控制流：遥控输入 → 步态生成 → 调度 → 状态机执行 → 电机输出
        double now = wb_robot_get_time();
        g_robot_state.sim_time_s = now;

        // 键盘遥控更新（速度指令 + 触发标志）
        remote_control_update(&g_robot_state);
        const RemoteCommand_t *rc = remote_control_get_command();

        // 边沿触发动作分发
        if (rc->shutdown_trigger)
        {
            g_shutdown = !g_shutdown;
            if (g_shutdown)
            {
                printf("[关机] 停止所有动作，电机保持当前位置\n");
                g_robot_state.cmd_vel_x = 0.0f;
                g_robot_state.cmd_vel_y = 0.0f;
                g_robot_state.cmd_vel_yaw = 0.0f;
            }
            else
            {
                printf("[开机] 恢复运行\n");
            }
        }

        if (rc->fold_trigger)
        {
            if (!fold_active)
            {
                // 进入折叠：保存当前角度（转为预映射空间）
                for (int i = 0; i < 4; i++)
                {
                    fold_start_angles[i] = leg_target_joints[i];
                    leg_direction_remap(i, &fold_start_angles[i]);
                }
                fold_active = 1;
                fold_start_time = now;
                printf("[折叠] 开始折叠 (J1外翻 J2前折 J3膝弯)...\n");
            }
            else
            {
                // 退出折叠：恢复步态
                fold_active = 0;
                // 强制各腿进入支撑相，从当前实际位置平滑过渡
                float stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
                gait_gen_update(&gait_gen, &g_robot_state);
                for (int i = 0; i < 4; i++)
                {
                    leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                                     &gait_gen.leg_output[i].stance_end, now);
                }
                printf("[折叠] 展开，恢复步态\n");
            }
        }

        if (rc->stand_trigger && !g_wheeled_mode)
        {
            // 停止步态周期和轮式模式
            g_gait_active = 0;
            g_wheeled_mode = 0;
            wheel_transition = 0;

            // 停止轮子电机
            if (m.has_wheels)
                for (int i = 0; i < 4; i++)
                    wb_motor_set_velocity(m.wheel[i], 0.0);

            // 退出关机和折叠模式
            if (g_shutdown)
            {
                g_shutdown = 0;
                printf("[站立] 退出关机模式\n");
            }
            if (fold_active)
            {
                fold_active = 0;
                printf("[站立] 退出折叠模式\n");
            }

            // 速度归零
            g_robot_state.cmd_vel_x = 0.0f;
            g_robot_state.cmd_vel_y = 0.0f;
            g_robot_state.cmd_vel_yaw = 0.0f;

            // 重置偏航控制，下一帧以当前朝向重新锁定
            g_yaw_target_initialized = 0;
            g_yaw_manual_offset = 0.0f;
            g_pitch_target = 0.0f; // 重置俯仰目标

            // 记录过渡起点（转为预映射空间，与目标角度坐标系一致）
            for (int i = 0; i < 4; i++)
            {
                stand_start_angles[i] = leg_target_joints[i];
                leg_direction_remap(i, &stand_start_angles[i]);
            }
            stand_transition = 1;
            stand_trans_start_time = now;
            printf("[站立] 过渡到初始站立位置 (J1=%.0f° J2=%.0f° J3=%.0f°)\n",
                   STAND_J1, STAND_J2, STAND_J3);
        }

        if (rc->gait_start_trigger && !g_wheeled_mode)
        {
            if (!g_gait_active)
            {
                // 确保不在关机和折叠状态
                if (g_shutdown)
                {
                    g_shutdown = 0;
                    printf("[步态] 退出关机模式\n");
                }
                if (fold_active)
                {
                    fold_active = 0;
                    printf("[步态] 退出折叠模式\n");
                }

                g_gait_active = 1;
                g_robot_state.mode = ROBOT_MODE_QUADRUPED;
                g_robot_state.quad_sub_mode = QUAD_MODE;
                g_pitch_target = 0.0f; // 重置俯仰目标
                // 更新步态生成器，重置相位从当前时刻开始
                gait_gen_update(&gait_gen, &g_robot_state);
                // 强制各腿进入支撑相，衔接步态周期
                float stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
                for (int i = 0; i < 4; i++)
                {
                    leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                                     &gait_gen.leg_output[i].stance_end, now);
                }
                printf("[步态] 步态周期启动 (WASD移动 Q/E转向 空格跳跃)\n");
            }
        }

        if (rc->mode_switch == 1)
        {
            // 切换到四足模式
            if (g_wheeled_mode || g_robot_state.mode != ROBOT_MODE_QUADRUPED)
            {
                g_wheeled_mode = 0;
                wheel_transition = 0;
                g_gait_active = 1;
                g_robot_state.mode = ROBOT_MODE_QUADRUPED;
                g_robot_state.quad_sub_mode = QUAD_MODE;
                g_robot_state.cmd_vel_x = 0.0f;
                g_robot_state.cmd_vel_y = 0.0f;
                g_robot_state.cmd_vel_yaw = 0.0f;
                g_pitch_target = 0.0f; // 重置俯仰目标
                // 停止轮子电机
                if (m.has_wheels)
                    for (int i = 0; i < 4; i++)
                        wb_motor_set_velocity(m.wheel[i], 0.0);
                gait_gen_update(&gait_gen, &g_robot_state);
                float stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
                for (int i = 0; i < 4; i++)
                {
                    leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                                     &gait_gen.leg_output[i].stance_end, now);
                }
                printf("[模式] 切换到四足模式 (WASD移动 Q/E转向)\n");
            }
        }
        if (rc->mode_switch == 2)
        {
            // 切换到轮式模式：先过渡到轮式姿态，之后由轮子电机驱动
            if (!g_wheeled_mode && !wheel_transition)
            {
                if (g_shutdown)
                {
                    g_shutdown = 0;
                    printf("[轮式] 退出关机模式\n");
                }
                if (fold_active)
                {
                    fold_active = 0;
                    printf("[轮式] 退出折叠模式\n");
                }
                g_wheeled_mode = 1;
                g_gait_active = 0;
                g_robot_state.mode = ROBOT_MODE_WHEELED;
                g_robot_state.cmd_vel_x = 0.0f;
                g_robot_state.cmd_vel_y = 0.0f;
                g_robot_state.cmd_vel_yaw = 0.0f;
                g_pitch_target = 0.0f; // 重置俯仰目标
                // 记录过渡起点（转为预映射空间）
                for (int i = 0; i < 4; i++)
                {
                    wheel_start_angles[i] = leg_target_joints[i];
                    leg_direction_remap(i, &wheel_start_angles[i]);
                }
                wheel_transition = 1;
                wheel_trans_start_time = now;
                printf("[模式] 切换到轮式模式 (过渡中...)\n");
            }
        }

        if (rc->jump_trigger)
            jump_controller_trigger(&g_jump_ctrl, now);

        // 站立姿态俯仰调整（[键后仰 / ]键低头），仅在静止站立时有效
        if (rc->pitch_adjust != 0 && !g_gait_active && !g_wheeled_mode && !fold_active && !g_shutdown)
        {
            g_pitch_target += rc->pitch_adjust * PITCH_ADJUST_STEP;
            if (g_pitch_target > PITCH_ADJUST_MAX)
                g_pitch_target = PITCH_ADJUST_MAX;
            if (g_pitch_target < -PITCH_ADJUST_MAX)
                g_pitch_target = -PITCH_ADJUST_MAX;
            printf("[俯仰] 目标俯仰角: %.1f°\n", g_pitch_target * 180.0f / pi);
        }

        if (rc->gait_switch != 0)
        {
            GaitType_t target_gait;
            if (rc->gait_switch == 1)
                target_gait = GAIT_TROT;
            else if (rc->gait_switch == 2)
                target_gait = GAIT_WALK;
            else
                target_gait = GAIT_PRONK;
            gait_controller_switch_gait(target_gait);
        }

        // 关机模式：跳过所有控制逻辑，电机保持当前位置
        if (g_shutdown)
        {
            set_all_motor(&m);
            continue;
        }

        // 折叠模式：插值动画到折叠姿态
        if (fold_active)
        {
            double fold_elapsed = now - fold_start_time;
            float t = (float)(fold_elapsed / FOLD_DURATION);
            if (t > 1.0f)
                t = 1.0f;
            for (int i = 0; i < 4; i++)
            {
                leg_target_joints[i].joint1_deg = fold_start_angles[i].joint1_deg + t * (FOLD_TARGET_ANGLES[i].joint1_deg - fold_start_angles[i].joint1_deg);
                leg_target_joints[i].joint2_deg = fold_start_angles[i].joint2_deg + t * (FOLD_TARGET_ANGLES[i].joint2_deg - fold_start_angles[i].joint2_deg);
                leg_target_joints[i].joint3_deg = fold_start_angles[i].joint3_deg + t * (FOLD_TARGET_ANGLES[i].joint3_deg - fold_start_angles[i].joint3_deg);
                leg_target_joints[i].valid = 1;
                leg_direction_remap(i, &leg_target_joints[i]);
            }
            set_all_motor(&m);
            continue;
        }

        if (jump_controller_is_active(&g_jump_ctrl))
        {
            // 跳跃模式：提升电机速度 + 扭矩，直接输出关节角度
            if (!motors_in_jump_mode)
                motors_set_jump_mode(&m);
            jump_controller_update(&g_jump_ctrl, now);
            for (int i = 0; i < 4; i++)
            {
                leg_target_joints[i] = g_jump_ctrl.output_joints[i];
                leg_direction_remap(i, &leg_target_joints[i]);
            }
        }
        else if (g_wheeled_mode)
        {
            // 轮式模式
            if (motors_in_jump_mode)
                motors_set_gait_mode(&m);

            if (wheel_transition)
            {
                // 过渡动画：从当前角度插值到轮式目标角度
                float t = (float)((now - wheel_trans_start_time) / WHEEL_TRANS_DUR);
                if (t >= 1.0f)
                {
                    t = 1.0f;
                    wheel_transition = 0;
                    // 过渡完成，各腿进入 WHEEL 状态
                    for (int i = 0; i < 4; i++)
                        leg_state_switch(&leg_fsm[i], LEG_STATE_WHEEL, 0.0, NULL, now);
                    printf("[轮式] 姿态切换完成 (WS前后 AD转向)\n");
                }
                for (int i = 0; i < 4; i++)
                {
                    leg_target_joints[i].joint1_deg = wheel_start_angles[i].joint1_deg + t * (WHEEL_TARGET_ANGLES[i].joint1_deg - wheel_start_angles[i].joint1_deg);
                    leg_target_joints[i].joint2_deg = wheel_start_angles[i].joint2_deg + t * (WHEEL_TARGET_ANGLES[i].joint2_deg - wheel_start_angles[i].joint2_deg);
                    leg_target_joints[i].joint3_deg = wheel_start_angles[i].joint3_deg + t * (WHEEL_TARGET_ANGLES[i].joint3_deg - wheel_start_angles[i].joint3_deg);
                    leg_target_joints[i].valid = 1;
                    leg_direction_remap(i, &leg_target_joints[i]);
                }
            }
            else
            {
                // 姿态到位后：状态机维持固定姿态，轮子电机驱动
                body_attitude_control_update(leg_sm);
                for (int i = 0; i < 4; i++)
                {
                    LegStateMachine_t *sm = &leg_fsm[i];
                    leg_state_machine_update(sm, now);
                    leg_get_target_angles(sm, &leg_target_joints[i]);
                    leg_direction_remap(i, &leg_target_joints[i]);
                }
                wheel_motors_update(&m);
            }
        }
        else if (g_gait_active)
        {
            // 正常步态模式：恢复电机速度
            if (motors_in_jump_mode)
                motors_set_gait_mode(&m);
            gait_gen_update(&gait_gen, &g_robot_state);
            gait_scheduler_update(&gait_scheduler);
            body_attitude_control_update(leg_sm);
            gait_controller_update(now);
        }
        else
        {
            // 站立静止模式：先过渡到站立目标位置，之后通过IK保持姿态
            if (motors_in_jump_mode)
                motors_set_gait_mode(&m);

            if (stand_transition)
            {
                // 关节角度插值动画：从当前角度过渡到 STAND 目标角度
                float t = (float)((now - stand_trans_start_time) / STAND_TRANS_DUR);
                if (t >= 1.0f)
                {
                    t = 1.0f;
                    stand_transition = 0;
                    // 过渡完成，各腿进入 STANCE 状态，由 IK 维持站立姿态
                    gait_gen_update(&gait_gen, &g_robot_state);
                    float stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
                    for (int i = 0; i < 4; i++)
                    {
                        float stand_x = (i == LEG_L_F || i == LEG_R_F) ? FOOT_OFFSET_X : -FOOT_OFFSET_X;
                        float stand_y = (i == LEG_L_F || i == LEG_L_B) ? FOOT_OFFSET_Y : -FOOT_OFFSET_Y;
                        float stand_z = -FOOT_OFFSET_Z;
                        FootPosition_t stand_pos = {stand_x, stand_y, stand_z};
                        leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                                         &stand_pos, now);
                    }
                    printf("[站立] 姿态到位，IK维持 (J1=%.0f° J2=%.0f° J3=%.0f°)\n",
                           STAND_J1, STAND_J2, STAND_J3);
                }
                for (int i = 0; i < 4; i++)
                {
                    leg_target_joints[i].joint1_deg = stand_start_angles[i].joint1_deg + t * (STAND_TARGET_ANGLES[i].joint1_deg - stand_start_angles[i].joint1_deg);
                    leg_target_joints[i].joint2_deg = stand_start_angles[i].joint2_deg + t * (STAND_TARGET_ANGLES[i].joint2_deg - stand_start_angles[i].joint2_deg);
                    leg_target_joints[i].joint3_deg = stand_start_angles[i].joint3_deg + t * (STAND_TARGET_ANGLES[i].joint3_deg - stand_start_angles[i].joint3_deg);
                    leg_target_joints[i].valid = 1;
                    leg_direction_remap(i, &leg_target_joints[i]);
                }
            }
            else
            {
                body_attitude_control_update(leg_sm);
            for (int i = 0; i < 4; i++)
            {
                LegStateMachine_t *sm = &leg_fsm[i];
                if (sm->current_state == LEG_STATE_STANCE)
                {
                    double elapsed = now - sm->state_start_time_s;
                    sm->phase = (sm->state_duration_s > 0.0) ? (elapsed / sm->state_duration_s) : 1.0;
                    if (sm->phase > 1.0)
                        sm->phase = 1.0;
                }
                leg_state_machine_update(sm, now);
                leg_get_target_angles(sm, &leg_target_joints[i]);
                leg_get_target_pos(sm, &foot_target_pos[i]);
                leg_direction_remap(i, &leg_target_joints[i]);
            }
            }
        }
        set_all_motor(&m);

        // 周期打印调试信息
        if (now - print_timer > 0.2f)
        {
            print_timer = now;
            printf("\n==== t=%.2f 全局相位:%.2f ====\n", now, gait_gen.global_phase);
            const char *leg_names[4] = {"右前", "左前", "右后", "左后"};
            for (int i = 0; i < 4; i++)
            {
                LegStateMachine_t *sm = &leg_fsm[i];
                printf("[%s] 状态:%-6s Ph=%.2f | J1=%6.1f J2=%6.1f J3=%6.1f | X=%.3f Y=%.3f Z=%.3f\n",
                       leg_names[i], leg_state_get_name(sm->current_state), sm->phase,
                       leg_target_joints[i].joint1_deg, leg_target_joints[i].joint2_deg, leg_target_joints[i].joint3_deg,
                       foot_target_pos[i].foot_pos_x_m, foot_target_pos[i].foot_pos_y_m, foot_target_pos[i].foot_pos_z_m);
            }
        }
    }

    wb_robot_cleanup();
    return 0;
}

// ===================== 工具函数实现 =====================
static void motors_init(LegMotors *m)
{
    m->j1[LEG_R_F] = wb_robot_get_device("hip_level_motor1");
    m->j2[LEG_R_F] = wb_robot_get_device("joint2_1");
    m->j3[LEG_R_F] = wb_robot_get_device("joint3_1");

    m->j1[LEG_L_F] = wb_robot_get_device("hip_level_motor2");
    m->j2[LEG_L_F] = wb_robot_get_device("joint2_2");
    m->j3[LEG_L_F] = wb_robot_get_device("joint3_2");

    m->j1[LEG_R_B] = wb_robot_get_device("hip_level_motor3");
    m->j2[LEG_R_B] = wb_robot_get_device("joint2_3");
    m->j3[LEG_R_B] = wb_robot_get_device("joint3_3");

    m->j1[LEG_L_B] = wb_robot_get_device("hip_level_motor4");
    m->j2[LEG_L_B] = wb_robot_get_device("joint2_4");
    m->j3[LEG_L_B] = wb_robot_get_device("joint3_4");

    // 轮式电机（若模型中有，否则为0）
    m->has_wheels = 1;
    const char *wheel_names[4] = {"wheel_motor1", "wheel_motor2", "wheel_motor3", "wheel_motor4"};
    for (int i = 0; i < 4; i++)
    {
        m->wheel[i] = wb_robot_get_device(wheel_names[i]);
        if (m->wheel[i] == 0)
            m->has_wheels = 0;
    }
    if (!m->has_wheels)
        printf("【警告】未检测到轮式电机 (名称: wheel_motor1~4)，轮式模式将无法移动\n");
}

static void set_leg_position(LegMotors *m, int leg_idx, LegJointAngles_t angles)
{
    // 仅当角度有效时下发，无效保持上一帧
    if (angles.valid == 1)
    {
        wb_motor_set_position(m->j1[leg_idx], DEG_TO_RAD(angles.joint1_deg));
        wb_motor_set_position(m->j2[leg_idx], DEG_TO_RAD(angles.joint2_deg));
        wb_motor_set_position(m->j3[leg_idx], DEG_TO_RAD(angles.joint3_deg));
    }
}

static void set_all_motor(LegMotors *m)
{
    for (int i = 0; i < 4; i++)
        set_leg_position(m, i, leg_target_joints[i]);
}

static void leg_direction_remap(int leg_idx, LegJointAngles_t *angles)
{
    // 按需启用：适配Webots电机安装方向与IK输出的符号差
    switch (leg_idx)
    {
    case LEG_R_F:
        angles->joint2_deg = -angles->joint2_deg;
        break;
    case LEG_L_F:
        angles->joint3_deg = -angles->joint3_deg;
        break;
    case LEG_R_B:
        angles->joint1_deg = -angles->joint1_deg;
        angles->joint2_deg = -angles->joint2_deg;
        break;
    case LEG_L_B:
        angles->joint1_deg = -angles->joint1_deg;
        angles->joint3_deg = -angles->joint3_deg;
        break;
    default:
        break;
    }
}

// 批量设置所有电机速度
static void motors_set_velocity_all(LegMotors *m, float velocity)
{
    for (int i = 0; i < 4; i++)
    {
        wb_motor_set_velocity(m->j1[i], velocity);
        wb_motor_set_velocity(m->j2[i], velocity);
        wb_motor_set_velocity(m->j3[i], velocity);
    }
}

// 进入跳跃模式：提高电机速度并释放扭矩上限
static void motors_set_jump_mode(LegMotors *m)
{
    motors_set_velocity_all(m, JUMP_MOTOR_VELOCITY);
    for (int i = 0; i < 4; i++)
    {
        wb_motor_set_available_torque(m->j1[i], JUMP_MOTOR_TORQUE);
        wb_motor_set_available_torque(m->j2[i], JUMP_MOTOR_TORQUE);
        wb_motor_set_available_torque(m->j3[i], JUMP_MOTOR_TORQUE);
    }
    motors_in_jump_mode = 1;
    printf("[跳跃] 电机切换到爆发模式 (vel=%.1f rad/s, torque=%.1f N·m)\n",
           JUMP_MOTOR_VELOCITY, JUMP_MOTOR_TORQUE);
}

// 退出跳跃模式：恢复步态电机速度
static void motors_set_gait_mode(LegMotors *m)
{
    motors_set_velocity_all(m, MOTOR_VELOCITY);
    motors_in_jump_mode = 0;
    printf("[跳跃] 电机恢复到步态模式 (vel=%.1f rad/s)\n", MOTOR_VELOCITY);
}

// 轮式电机控制：差速驱动，WS前后 AD转向
static void wheel_motors_update(LegMotors *m)
{
    if (!m->has_wheels)
        return;

    float v_x = g_robot_state.cmd_vel_x;     // 前后速度 (m/s)
    float v_yaw = g_robot_state.cmd_vel_yaw;  // 转向速度 (rad/s)
    float wheel_radius = WHEEL_RADIUS;
    float half_base = WHEEL_BASE_HALF;

    // 差速模型：左轮和右轮分别计算线速度 → 角速度
    float v_left = v_x - v_yaw * half_base;
    float v_right = v_x + v_yaw * half_base;
    float omega_left = v_left / wheel_radius;
    float omega_right = v_right / wheel_radius;

    // 限幅
    if (omega_left > WHEEL_MAX_SPEED) omega_left = WHEEL_MAX_SPEED;
    if (omega_left < -WHEEL_MAX_SPEED) omega_left = -WHEEL_MAX_SPEED;
    if (omega_right > WHEEL_MAX_SPEED) omega_right = WHEEL_MAX_SPEED;
    if (omega_right < -WHEEL_MAX_SPEED) omega_right = -WHEEL_MAX_SPEED;

    // 左腿: LF(1), LB(3); 右腿: RF(0), RB(2)
    // 速度控制模式：position=INFINITY, velocity=目标角速度
    wb_motor_set_position(m->wheel[LEG_L_F], INFINITY);
    wb_motor_set_position(m->wheel[LEG_L_B], INFINITY);
    wb_motor_set_position(m->wheel[LEG_R_F], INFINITY);
    wb_motor_set_position(m->wheel[LEG_R_B], INFINITY);

    wb_motor_set_velocity(m->wheel[LEG_L_F], omega_left);
    wb_motor_set_velocity(m->wheel[LEG_L_B], omega_left);
    wb_motor_set_velocity(m->wheel[LEG_R_F], omega_right);
    wb_motor_set_velocity(m->wheel[LEG_R_B], omega_right);
}
#include "gait_controller.h"

GaitGenerator_t gait_gen;
GaitScheduler_t gait_scheduler;
LegStateMachine_t leg_fsm[4];
FootPosition_t foot_target_pos[4];
LegJointAngles_t leg_target_joints[4];

static void leg_direction_remap(int leg_idx, LegJointAngles_t *angles);

void gait_controller_init(void)
{
    // 步态生成器配置
    GaitGenConfig_t gait_cfg = {
        .cycle_time = CYCLE_TIME,
        .swing_ratio = SWING_RATIO,
        .step_height = STEP_HEIGHT,
        .max_step_x = MAX_STEP_X,
        .max_step_y = MAX_STEP_Y};
    gait_gen_init(&gait_gen);
    gait_gen.gait_type = GaitType;
    gait_gen.config = gait_cfg;

    // 轨迹配置：轨迹周期 = 摆动相时长，与步态严格匹配
    BezierTrajConfig_t traj_cfg = {
        .cycle_time = CYCLE_TIME * SWING_RATIO, // 关键修正：轨迹总时长=摆动时长
        .lift_height_m = STEP_HEIGHT};

    // 四条腿状态机初始化
    for (int i = 0; i < 4; i++)
    {
        leg_state_machine_init(&leg_fsm[i], (Leg_ID_t)i, traj_cfg);

        // 同步初始足端位置（Z轴向下为负，与坐标系一致）
        foot_target_pos[i].foot_pos_x_m = (i == LEG_L_F || i == LEG_R_F) ? FOOT_OFFSET_X : (-FOOT_OFFSET_X);
        foot_target_pos[i].foot_pos_y_m = (i == LEG_L_F || i == LEG_L_B) ? FOOT_OFFSET_Y : (-FOOT_OFFSET_Y);
        foot_target_pos[i].foot_pos_z_m = -FOOT_OFFSET_Z; // 关键修正：Z轴向下为负

        // 写入状态机，保证位置与角度对应
        leg_fsm[i].current_target_pos = foot_target_pos[i];
        leg_fsm[i].swing_start_pos = foot_target_pos[i];
        leg_fsm[i].stance_start_pos = foot_target_pos[i];
        leg_fsm[i].stance_end_pos = foot_target_pos[i];
    }

    // 调度器绑定
    LegStateMachine_t *sm_ptr[4] = {&leg_fsm[0], &leg_fsm[1], &leg_fsm[2], &leg_fsm[3]};
    gait_scheduler_init(&gait_scheduler, sm_ptr, &gait_gen);

    // 先更新一次步态，保证输出数据有效
    double init_time = wb_robot_get_time();
    g_robot_state.sim_time_s = init_time;
    gait_gen_update(&gait_gen, &g_robot_state);

    // 开机强制切入支撑相，起点为当前初始位置
    float stance_time = CYCLE_TIME * (1 - SWING_RATIO);
    for (int i = 0; i < 4; i++)
    {
        LegGaitOutput_t gait_out;
        gait_gen_get_leg_target(&gait_gen, (Leg_ID_t)i, &gait_out);
        leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time, &gait_out.stance_end, init_time);
    }
}
void gait_controller_update(double abs_time)
{
    float swing_ratio = gait_gen.config.swing_ratio;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        LegStateMachine_t *sm = &leg_fsm[leg_id];
        LegGaitOutput_t *out = &gait_gen.leg_output[leg_id];

        // 通过 gait generator 统一接口获取相位，自动适配 Tro/Walk/Pronk 等所有步态
        float leg_phase = gait_gen_get_single_phase(&gait_gen, leg_id);

        if (sm->current_state == LEG_STATE_SWING)
        {
            sm->phase = leg_phase / swing_ratio;
        }
        else if (sm->current_state == LEG_STATE_STANCE)
        {
            sm->phase = (leg_phase - swing_ratio) / (1.0f - swing_ratio);
        }
        else
        {
            sm->phase = 0.0f;
        }
        sm->phase = fminf(fmaxf((float)sm->phase, 0.0f), 1.0f); // 钳位到 0~1

        // 同步理论起止点（稳态下闭环，切换时自动衔接）
        sm->swing_start_pos = out->swing_start;
        sm->swing_end_pos = out->swing_end;
        sm->stance_start_pos = out->stance_start;
        sm->stance_end_pos = out->stance_end;

        // 更新状态机（内部直接用上面算好的 phase 插值）
        leg_state_machine_update(sm, abs_time);

        // 取出结果、重映射，保持不变
        leg_get_target_angles(sm, &leg_target_joints[leg_id]);
        leg_get_target_pos(sm, &foot_target_pos[leg_id]);
        leg_direction_remap(leg_id, &leg_target_joints[leg_id]);
    }
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

void gait_controller_switch_gait(GaitType_t new_gait_type)
{
    gait_gen_switch_gait(&gait_gen, new_gait_type);

    // 更新机器人状态中的子模式
    switch (new_gait_type)
    {
    case GAIT_TROT:
        g_robot_state.quad_sub_mode = QUAD_MODE_TROT;
        break;
    case GAIT_WALK:
        g_robot_state.quad_sub_mode = QUAD_MODE_WALK;
        break;
    case GAIT_PRONK:
        g_robot_state.quad_sub_mode = QUAD_MODE_PRONK;
        break;
    default:
        break;
    }

    // 更新各腿状态机的轨迹配置（摆动周期 = 步态周期 × 摆动占比）
    float new_traj_cycle = gait_gen.config.cycle_time * gait_gen.config.swing_ratio;
    for (int i = 0; i < 4; i++)
    {
        leg_fsm[i].traj_ctx.config.cycle_time = new_traj_cycle;
        leg_fsm[i].traj_ctx.config.lift_height_m = gait_gen.config.step_height;
    }

    const char *name = (new_gait_type == GAIT_TROT) ? "Trot" : (new_gait_type == GAIT_WALK) ? "Walk"
                                                                                             : "Pronk";
    printf("[步态切换] -> %s (T=%.2fs, swing=%.0f%%, step_h=%.3fm)\n",
           name, gait_gen.config.cycle_time,
           gait_gen.config.swing_ratio * 100.0f,
           gait_gen.config.step_height);
}

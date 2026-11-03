#include "leg_state_machine.h"
#include "robot_state.h"
#include "body_control.h"

#define LOG_STATE(sm, type, msg)     \
    printf("[LEG:%d][%s] %s | %s\n", \
           sm->leg_id, leg_state_get_name(sm->current_state), type, msg)

// ======================== Private �������� ========================
static void foot_lerp(const FootPosition_t *a, const FootPosition_t *b, float t, FootPosition_t *out);

static void leg_state_on_enter_hold(LegStateMachine_t *sm);
static void leg_state_on_run_hold(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_hold(LegStateMachine_t *sm);

static void leg_state_on_enter_stance(LegStateMachine_t *sm);
static void leg_state_on_run_stance(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_stance(LegStateMachine_t *sm);

static void leg_state_on_enter_swing(LegStateMachine_t *sm);
static void leg_state_on_run_swing(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_swing(LegStateMachine_t *sm);

static void leg_state_on_enter_error(LegStateMachine_t *sm);
static void leg_state_on_run_error(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_error(LegStateMachine_t *sm);

static void leg_state_on_enter_wheel(LegStateMachine_t *sm);
static void leg_state_on_run_wheel(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_wheel(LegStateMachine_t *sm);

static void leg_state_on_enter_fold(LegStateMachine_t *sm);
static void leg_state_on_run_fold(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_fold(LegStateMachine_t *sm);

static void leg_state_on_enter_unfold(LegStateMachine_t *sm);
static void leg_state_on_run_unfold(LegStateMachine_t *sm, double dt_s);
static void leg_state_on_exit_unfold(LegStateMachine_t *sm);

// ======================== Public ========================

int leg_state_machine_init(LegStateMachine_t *sm, Leg_ID_t leg_id, BezierTrajConfig_t config)
{
    if (!sm || leg_id < LEG_R_F || leg_id > LEG_L_B)
        return -1;

    memset(sm, 0, sizeof(LegStateMachine_t));
    sm->leg_id = leg_id;
    sm->traj_ctx.leg_id = leg_id;
    sm->first_swing_done = 0;

    sm->current_state = LEG_STATE_HOLD;
    sm->last_state = LEG_STATE_HOLD;

    if (bezier_traj_controller_init(&sm->traj_ctx, leg_id, config) != 0)
    {
        snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "traj init fail");
        return -1;
    }

    // ��ʼ���ؽڽǶ�������λ�ã���֤��ʼ״̬��������
    switch (leg_id)
    {
    case LEG_R_F:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, STAND_J2, -STAND_J3, 1};
        break;
    case LEG_L_F:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, -STAND_J2, STAND_J3, 1};
        break;
    case LEG_R_B:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, STAND_J2, -STAND_J3, 1};
        break;
    case LEG_L_B:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, -STAND_J2, STAND_J3, 1};
        break;
    }

    // ========== ��ʼ��Ĭ������λ�ã�������ʼ������ ==========
    sm->current_target_pos.foot_pos_x_m = 0.0f;
    sm->current_target_pos.foot_pos_y_m = 0.0f;
    sm->current_target_pos.foot_pos_z_m = -0.15f;

    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "init ok");
    printf("[LEG:%d] leg sm init ok\n", leg_id);
    return 0;
}

int leg_state_switch(LegStateMachine_t *sm, LegState_t new_state,
                     double duration_s, const FootPosition_t *end_pos,
                     double current_time_s)
{
    if (!sm)
        return -1;
    if (!leg_state_switch_allowed(sm->current_state, new_state))
    {
        snprintf(sm->last_error_msg, sizeof(sm->last_error_msg),
                 "no trans %s->%s",
                 leg_state_get_name(sm->current_state),
                 leg_state_get_name(new_state));
        return -1;
    }
    if (new_state == sm->current_state)
        return 0;

    // �˳���״̬
    switch (sm->current_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_exit_hold(sm);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_exit_stance(sm);
        break;
    case LEG_STATE_SWING:
        leg_state_on_exit_swing(sm);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_exit_wheel(sm);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_exit_fold(sm);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_exit_unfold(sm);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_exit_error(sm);
        break;
    }

    // ����״̬��������Ϣ
    sm->last_state = sm->current_state;
    sm->current_state = new_state;
    sm->state_start_time_s = current_time_s;
    sm->state_duration_s = duration_s;
    sm->phase = 0.0;

    // ========== ͳһ��ʼ����״̬����ֹ�㣬��֤������Զ�νӵ�ǰλ�� ==========
    if (new_state == LEG_STATE_SWING)
    {
        // �ڶ�����ǿ�Ƶ��ڵ�ǰ����λ�ã���һ״̬���յ㣩�����ױ�֤����
        sm->swing_start_pos = sm->current_target_pos;
        // �յ����ⲿ���룬����ʧ���򱣳ֵ�ǰλ�ã�ԭ�ذ��ȶ��ף�
        if (end_pos != NULL)
            sm->swing_end_pos = *end_pos;
        else
            sm->swing_end_pos = sm->current_target_pos;
    }
    else if (new_state == LEG_STATE_STANCE)
    {
        // ֧������ǿ�Ƶ��ڵ�ǰ����λ�ã���֤����/����������
        sm->stance_start_pos = sm->current_target_pos;
        // �յ����ⲿ���룬����ʧ���򱣳ֵ�ǰλ�ã�ԭ��֧�Ŷ��ף�
        if (end_pos != NULL)
            sm->stance_end_pos = *end_pos;
        else
            sm->stance_end_pos = sm->current_target_pos;
    }

    // ������״̬
    switch (new_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_enter_hold(sm);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_enter_stance(sm);
        break;
    case LEG_STATE_SWING:
        leg_state_on_enter_swing(sm);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_enter_wheel(sm);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_enter_fold(sm);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_enter_unfold(sm);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_enter_error(sm);
        break;
    }

    return 0;
}

int leg_state_machine_update(LegStateMachine_t *sm, double current_time_s)
{
    if (!sm)
        return -1;
    // phase ���ⲿȫ����λ������ֱ�Ӹ�ֵ������ֻ��ͬ��
    if (sm->current_state == LEG_STATE_SWING)
        sm->swing_phase = sm->phase;
    else if (sm->current_state == LEG_STATE_STANCE)
        sm->stance_phase = sm->phase;

    double dt_s = g_robot_state.dt_s;
    // ������ switch ״̬�����߼�ȫ�����ֲ���
    switch (sm->current_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_run_hold(sm, dt_s);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_run_stance(sm, dt_s);
        break;
    case LEG_STATE_SWING:
        leg_state_on_run_swing(sm, dt_s);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_run_wheel(sm, dt_s);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_run_fold(sm, dt_s);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_run_unfold(sm, dt_s);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_run_error(sm, dt_s);
        break;
    }

    return 0;
}

void leg_get_target_angles(const LegStateMachine_t *sm, LegJointAngles_t *out)
{
    if (sm && out)
        *out = sm->current_joint_angles;
}

void leg_get_target_pos(const LegStateMachine_t *sm, FootPosition_t *out)
{
    if (sm && out)
        *out = sm->current_target_pos;
}

const char *leg_state_get_name(LegState_t s)
{
    switch (s)
    {
    case LEG_STATE_HOLD:
        return "HOLD";
    case LEG_STATE_STANCE:
        return "STANCE";
    case LEG_STATE_SWING:
        return "SWING";
    case LEG_STATE_WHEEL:
        return "WHEEL";
    case LEG_STATE_FOLD:
        return "FOLD";
    case LEG_STATE_UNFOLD:
        return "UNFOLD";
    case LEG_STATE_ERROR:
        return "ERROR";
    default:
        return "???";
    }
}

void leg_state_machine_print(const LegStateMachine_t *sm)
{
    if (!sm)
    {
        printf("sm null\n");
        return;
    }
    printf("=== LEG %d ===\n", sm->leg_id);
    printf("State: %s  Ph:%.2f\n",
           leg_state_get_name(sm->current_state), sm->phase);
    printf("Foot: %.2f %.2f %.2f\n",
           sm->current_target_pos.foot_pos_x_m,
           sm->current_target_pos.foot_pos_y_m,
           sm->current_target_pos.foot_pos_z_m);
    printf("Joint: %.1f %.1f %.1f\n",
           sm->current_joint_angles.joint1_deg,
           sm->current_joint_angles.joint2_deg,
           sm->current_joint_angles.joint3_deg);
    printf("Msg: %s\n", sm->last_error_msg);
    printf("===============\n");
}

void leg_state_machine_reset(LegStateMachine_t *sm)
{
    if (sm)
        leg_state_switch(sm, LEG_STATE_HOLD, 0.0, NULL, g_robot_state.sim_time_s);
}

// ����λ�����Բ�ֵ
static void foot_lerp(const FootPosition_t *a, const FootPosition_t *b, float t, FootPosition_t *out)
{
    out->foot_pos_x_m = a->foot_pos_x_m + t * (b->foot_pos_x_m - a->foot_pos_x_m);
    out->foot_pos_y_m = a->foot_pos_y_m + t * (b->foot_pos_y_m - a->foot_pos_y_m);
    out->foot_pos_z_m = a->foot_pos_z_m + t * (b->foot_pos_z_m - a->foot_pos_z_m) + t * STANCE_Z_COMPENSATION;
}

// ======================== HOLD ========================
static void leg_state_on_enter_hold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "hold");
    memset(&sm->traj_ctx, 0, sizeof(LegTrajContext_t));
    sm->is_trajectory_finished = 1;
    sm->ik_fail_count = 0;
    sm->swing_phase = 0.0;
    sm->stance_phase = 0.0;
    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "HOLD");
}

static void leg_state_on_run_hold(LegStateMachine_t *sm, double dt_s)
{
    // ========== ���ֵ�ǰ�ؽڽǶ�������λ�ò��䣬�ž�����Ư�� ==========
    // �����κμ��㣬ά����һ״̬������ֵ����֤�����г�����
}

static void leg_state_on_exit_hold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "hold");
}

// ======================== STANCE ========================
static void leg_state_on_enter_stance(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "stance");
    // �������� leg_state_switch ��ͳһ��ֵΪ��ǰλ�ã��˴�����״̬��ʼ��
    sm->is_trajectory_finished = 0;
    sm->ik_fail_count = 0;
    sm->stance_phase = 0.0;

    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg),
             "Stance S(%.2f,%.2f,%.2f) E(%.2f,%.2f,%.2f)",
             sm->stance_start_pos.foot_pos_x_m, sm->stance_start_pos.foot_pos_y_m, sm->stance_start_pos.foot_pos_z_m,
             sm->stance_end_pos.foot_pos_x_m, sm->stance_end_pos.foot_pos_y_m, sm->stance_end_pos.foot_pos_z_m);
}

static void leg_state_on_run_stance(LegStateMachine_t *sm, double dt_s)
{
    float phase = (float)fmin(sm->phase, 1.0f);

    // ���Բ�ֵ���㵱ǰĿ������λ��
    foot_lerp(&sm->stance_start_pos, &sm->stance_end_pos, phase, &sm->current_target_pos);
    // 姿态修正必须在 foot_lerp 之后、IK 之前，确保修正量参与关节解算
    body_apply_to_stance_leg(sm);
    // printf("leg%d stance: start_x=%.3f, end_x=%.3f, cur_x=%.3f, t=%.3f\n",
    //        sm->leg_id, // ���� leg_id�������ṹ������ʲô
    //        sm->stance_start_pos.foot_pos_x_m,
    //        sm->stance_end_pos.foot_pos_x_m,
    //        sm->current_target_pos.foot_pos_x_m,
    //        phase);
    // ���˶�ѧ����
    IKJacobianError_t err = leg_ik_solve(sm->leg_id, &sm->current_target_pos,
                                         &sm->current_joint_angles, IK_MODE_AUTO);

    if (err != IK_JACOBIAN_SUCCESS)
    {
        sm->ik_fail_count++;
        snprintf(sm->last_error_msg, sizeof(sm->last_error_msg),
                 "Stance IK fail %d, err=%d", sm->ik_fail_count, err);

        // ========== IKʧ��ʱ������һ֡��Ч�Ƕȣ���ͻ�� ==========
        // ������ current_joint_angles��ά��ԭֵ
        if (sm->ik_fail_count > 5)
        {
            LOG_STATE(sm, "ERROR", "IK fail too much -> ERROR");
            leg_state_switch(sm, LEG_STATE_ERROR, 0.0, NULL, g_robot_state.sim_time_s);
        }
    }
    else
    {
        sm->ik_fail_count = 0;
    }

    if (sm->phase >= 0.99f)
        sm->is_trajectory_finished = 1;
}

static void leg_state_on_exit_stance(LegStateMachine_t *sm)
{
    // �Ƴ�ԭ�е�swing_start_pos��ֵ��ͳһ��leg_state_switch����
    // ��֤���۴�ʲô״̬�����ڶ��࣬���㶼��������
    LOG_STATE(sm, "EXIT", "stance -> swing");
}

// ======================== SWING ========================
static void leg_state_on_enter_swing(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "swing");
    FootPosition_t traj_start;
    if (sm->first_swing_done == 0)
    {
        // ��һ�ν����ڶ����õ�ǰʵ��λ�õ����㣬��֤����һ״̬�޷��ν�
        traj_start = sm->current_target_pos;
        sm->first_swing_done = 1; // �����״����ɣ���������̬�߼�
    }
    else
    {
        // ��̬���ڣ�ֱ���ò�̬�������趨�����ۺ�����Ϊ����
        traj_start = sm->swing_start_pos;
    }
    // ========== ��״̬�л�ʱ�������������ɹ켣�����Ա�֤λ������ ==========
    // ���������ⲿ���������㣬��Զ����һ״̬�յ���Ϊ�켣����
    generate_leg_bezier_traj(&sm->traj_ctx, traj_start, sm->swing_end_pos);

    sm->is_trajectory_finished = 0;
    sm->ik_fail_count = 0;
    sm->swing_phase = 0.0;
    sm->traj_ctx.is_trajectory_running = 1;

    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg),
             "Swing S(%.2f,%.2f) E(%.2f,%.2f)",
             sm->swing_start_pos.foot_pos_x_m, sm->swing_start_pos.foot_pos_y_m,
             sm->swing_end_pos.foot_pos_x_m, sm->swing_end_pos.foot_pos_y_m);
}

static void leg_state_on_run_swing(LegStateMachine_t *sm, double dt_s)
{
    float t = (float)(sm->phase * sm->state_duration_s);
    float v[3] = {0}, a[3] = {0};

    TrajStatus_t status = get_leg_joint_cmd(&sm->traj_ctx, t,
                                            &sm->current_joint_angles, v, a);

    // ���µ�ǰ����Ŀ��λ�ã���������ͬ��
    sm->current_target_pos = sm->traj_ctx.runtime.last_foot_pos;
    sm->last_traj_status = status;

    if (status == TRAJ_STATUS_END)
    {
        sm->is_trajectory_finished = 1;
        // ========== �켣����ʱǿ�ƶ����յ㣬�����ۼ����� ==========
        sm->current_target_pos = sm->swing_end_pos;
        LOG_STATE(sm, "INFO", "swing done");
    }
    else if (status == TRAJ_STATUS_ERROR_IK || status == TRAJ_STATUS_ERROR_INPUT)
    {
        sm->ik_fail_count++;
        snprintf(sm->last_error_msg, sizeof(sm->last_error_msg),
                 "Swing traj err %d", sm->ik_fail_count);

        if (sm->ik_fail_count > 3)
        {
            LOG_STATE(sm, "ERROR", "traj error -> ERROR");
            leg_state_switch(sm, LEG_STATE_ERROR, 0.0, NULL, g_robot_state.sim_time_s);
        }
    }
    else
    {
        sm->ik_fail_count = 0;
    }
}

static void leg_state_on_exit_swing(LegStateMachine_t *sm)
{
    sm->traj_ctx.is_trajectory_running = 0;
    LOG_STATE(sm, "EXIT", "swing -> stance");
}

// ======================== ERROR ========================
static void leg_state_on_enter_error(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "FAULT", "ERROR state");
    memset(&sm->traj_ctx, 0, sizeof(LegTrajContext_t));
    sm->is_trajectory_finished = 1;
    sm->ik_fail_count = 0;
    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "ERROR");
}

static void leg_state_on_run_error(LegStateMachine_t *sm, double dt_s)
{
    // ����״̬����������Ч�Ƕȣ������κθ��£���ֹ��������
}

static void leg_state_on_exit_error(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "RESET", "exit error");
    sm->ik_fail_count = 0;
    memset(sm->last_error_msg, 0, sizeof(sm->last_error_msg));
}

// ======================== WHEEL ========================
static void leg_state_on_enter_wheel(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "wheel");
    // 与 IK 解算器输出约定一致：右腿J2正/J3负，左腿J2负/J3正
    switch (sm->leg_id)
    {
    case LEG_R_F:
        sm->current_joint_angles = (LegJointAngles_t){0.0f, WHEEL_J2, -WHEEL_J3, 1};
        break;
    case LEG_L_F:
        sm->current_joint_angles = (LegJointAngles_t){0.0f, -WHEEL_J2, WHEEL_J3, 1};
        break;
    case LEG_R_B:
        sm->current_joint_angles = (LegJointAngles_t){0.0f, WHEEL_J2, -WHEEL_J3, 1};
        break;
    case LEG_L_B:
        sm->current_joint_angles = (LegJointAngles_t){0.0f, -WHEEL_J2, WHEEL_J3, 1};
        break;
    default:
        break;
    }
    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "WHEEL");
}

static void leg_state_on_run_wheel(LegStateMachine_t *sm, double dt_s)
{
}

static void leg_state_on_exit_wheel(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "wheel");
}

// ======================== FOLD ========================
static void leg_state_on_enter_fold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "fold");
    switch (sm->leg_id)
    {
    case LEG_R_F:
        sm->current_joint_angles = (LegJointAngles_t){0, 120, -120, 1};
        break;
    case LEG_L_F:
        sm->current_joint_angles = (LegJointAngles_t){0, -120, 120, 1};
        break;
    case LEG_R_B:
        sm->current_joint_angles = (LegJointAngles_t){0, 120, -120, 1};
        break;
    case LEG_L_B:
        sm->current_joint_angles = (LegJointAngles_t){0, -120, 120, 1};
        break;
    default:
        break;
    }
    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "FOLD");
}

static void leg_state_on_run_fold(LegStateMachine_t *sm, double dt_s)
{
}

static void leg_state_on_exit_fold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "fold");
}

// ======================== UNFOLD ========================
static void leg_state_on_enter_unfold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "unfold");
    sm->phase = 0.0;
    snprintf(sm->last_error_msg, sizeof(sm->last_error_msg), "UNFOLD");
}

static void leg_state_on_run_unfold(LegStateMachine_t *sm, double dt_s)
{
    sm->phase += dt_s / 1.0f;
    if (sm->phase >= 1.0f)
    {
        sm->phase = 1.0f;
        LOG_STATE(sm, "INFO", "unfold done -> STANCE");

        // չ���������Ե�ǰλ��Ϊ���㣬Ŀ��Ϊ����֧��λ����֤��������
        FootPosition_t stance_end = sm->current_target_pos;
        stance_end.foot_pos_x_m = 0.0f;
        stance_end.foot_pos_y_m = 0.0f;
        stance_end.foot_pos_z_m = sm->current_target_pos.foot_pos_z_m;

        leg_state_switch(sm, LEG_STATE_STANCE, 0.5f, &stance_end, g_robot_state.sim_time_s);
    }
}

static void leg_state_on_exit_unfold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "unfold");
}
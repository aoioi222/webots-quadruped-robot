/**
 ******************************************************************************
 * @file    leg_state_machine.h
 * @brief   ����״̬��ͷ�ļ�
 ******************************************************************************
 */

#ifndef LEG_STATE_MACHINE_H
#define LEG_STATE_MACHINE_H

#include "include_head.h"
#include "bezier_foot_tra.h"
#include "robot_state.h"
#include "gait_generator.h"

// STANCE_Z_COMPENSATION 已集中到 robot_config.h
/**
 * @brief ����״̬ö��
 */
typedef enum
{
    LEG_STATE_HOLD = 0, // ���ֲ�������ʼ/��ȫ̬��
    LEG_STATE_STANCE,   // ֧���ࣨ���Ժ��ƣ�
    LEG_STATE_SWING,    // �ڶ��ࣨ�������켣��
    LEG_STATE_WHEEL,    // ��ʽģʽ
    LEG_STATE_FOLD,     // ��ȫ�۵�������ģʽ��
    LEG_STATE_UNFOLD,   // չ������
    LEG_STATE_ERROR     // ���󱣻�
} LegState_t;

/**
 * @brief ����״̬��������
 */
typedef struct
{
    Leg_ID_t leg_id;

    LegState_t current_state;
    LegState_t last_state;

    double state_start_time_s;
    double state_duration_s;

    double phase;        // ��ǰ״̬��һ����λ 0~1
    double swing_phase;  // �ڶ�����λ
    double stance_phase; // ֧������λ

    LegTrajContext_t traj_ctx;

    // �ڶ�����ֹ��
    FootPosition_t swing_start_pos;
    FootPosition_t swing_end_pos; // ��Ϊ��һ����stance_start_pos
    // ֧������ֹ��
    FootPosition_t stance_start_pos;
    FootPosition_t stance_end_pos; // ��Ϊ��һ����swing_start_pos
    // ��ǰĿ��λ�ú͹ؽڽǶ�
    FootPosition_t current_target_pos;
    LegJointAngles_t current_joint_angles;

    int ik_fail_count;
    int is_trajectory_finished;
    TrajStatus_t last_traj_status;
    char last_error_msg[128];
    FootPosition_t stance_world_pos;
    uint8_t first_swing_done;
} LegStateMachine_t;

/**
 * @brief  ��ʼ������״̬��
 * @param  sm       ״̬�����
 * @param  leg_id   �ȱ��
 * @param  config   �������켣����
 * @return 0�ɹ���-1ʧ��
 */
int leg_state_machine_init(LegStateMachine_t *sm, Leg_ID_t leg_id, BezierTrajConfig_t config);

/**
 * @brief  �л�����״̬
 * @param  sm             ״̬�����
 * @param  new_state      Ŀ��״̬
 * @param  duration_s     ״̬����ʱ��
 * @param  end_pos        �켣�յ����λ��
 * @param  current_time_s ��ǰʱ��
 * @return 0�ɹ���-1ʧ��
 */
int leg_state_switch(LegStateMachine_t *sm, LegState_t new_state,
                     double duration_s, const FootPosition_t *end_pos,
                     double current_time_s);

/**
 * @brief  ����״̬�����ڸ���
 * @param  sm             ״̬�����
 * @param  current_time_s ��ǰʱ��
 * @return 0�ɹ�
 */
int leg_state_machine_update(LegStateMachine_t *sm, double current_time_s);

/**
 * @brief  ��ȡ��ǰĿ��ؽڽǶ�
 * @param  sm         ״̬�����
 * @param  out_angles ���Ŀ��ؽڽǶ�
 */
void leg_get_target_angles(const LegStateMachine_t *sm, LegJointAngles_t *out_angles);

/**
 * @brief  ��ȡ��ǰ���Ŀ��λ��
 * @param  sm       ״̬�����
 * @param  out_pos  ������Ŀ��λ��
 */
void leg_get_target_pos(const LegStateMachine_t *sm, FootPosition_t *out_pos);

/**
 * @brief  ��ȡ״̬�����ַ���
 * @param  state  ��״̬
 * @return ״̬����
 */
const char *leg_state_get_name(LegState_t state);

/**
 * @brief  ��ӡ״̬����Ϣ
 * @param  sm  ״̬�����
 */
void leg_state_machine_print(const LegStateMachine_t *sm);

/**
 * @brief  ��λ����״̬��
 * @param  sm  ״̬�����
 */
void leg_state_machine_reset(LegStateMachine_t *sm);

// ģʽ�ж���������

/**
 * @brief  �ж��Ƿ�Ϊ���㲽��ģʽ
 * @param  sm  ״̬�����
 * @return 1�ǣ�0��
 */
static inline uint8_t leg_is_quadruped_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_STANCE || sm->current_state == LEG_STATE_SWING);
}

/**
 * @brief  �ж��Ƿ�Ϊ��ʽģʽ
 * @param  sm  ״̬�����
 * @return 1�ǣ�0��
 */
static inline uint8_t leg_is_wheeled_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_WHEEL);
}

/**
 * @brief  �ж��Ƿ�Ϊ�۵�����ģʽ
 * @param  sm  ״̬�����
 * @return 1�ǣ�0��
 */
static inline uint8_t leg_is_flight_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_FOLD);
}

/**
 * @brief  �ж��Ȳ��Ƿ������˶�
 * @param  sm  ״̬�����
 * @return 1�ǣ�0��
 */
static inline uint8_t leg_is_moving(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_SWING ||
            sm->current_state == LEG_STATE_UNFOLD ||
            sm->current_state == LEG_STATE_FOLD);
}

/**
 * @brief  �ж�״̬�л��Ƿ�Ϸ�
 * @param  current  ��ǰ״̬
 * @param  target   Ŀ��״̬
 * @return 1������0������
 */
static inline uint8_t leg_state_switch_allowed(LegState_t current, LegState_t target)
{
    if (current == LEG_STATE_ERROR)
        return (target == LEG_STATE_HOLD) ? 1 : 0;
    if (current == LEG_STATE_HOLD)
        return 1;
    if ((current == LEG_STATE_STANCE || current == LEG_STATE_SWING) &&
        (target == LEG_STATE_STANCE || target == LEG_STATE_SWING || target == LEG_STATE_HOLD || target == LEG_STATE_ERROR || target == LEG_STATE_WHEEL))
        return 1;
    if (current == LEG_STATE_WHEEL &&
        (target == LEG_STATE_HOLD || target == LEG_STATE_UNFOLD || target == LEG_STATE_STANCE))
        return 1;
    if (current == LEG_STATE_FOLD &&
        (target == LEG_STATE_HOLD || target == LEG_STATE_UNFOLD))
        return 1;
    if (current == LEG_STATE_UNFOLD &&
        (target == LEG_STATE_STANCE || target == LEG_STATE_HOLD))
        return 1;
    return 0;
}

#endif

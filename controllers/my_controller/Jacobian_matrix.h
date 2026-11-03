/**
 ******************************************************************************
 * @file    Jacobian_matrix.h
 * @brief   ��������˵����ſɱȾ����
 *          ʵ�ֹؽڿռ�<->�ѿ����ռ���ٶ�/��/���ٶ�ӳ�䡢����α�桢�ſɱȵ���IK
 ******************************************************************************
 */

#ifndef JACOBIAN_MATRIX_H
#define JACOBIAN_MATRIX_H

#include "Foot_structure.h"
#include "Forward_kinematics_calculate.h"
#include "Inverse_kinematics_calculate.h"

/************************* �ſɱȼ�����غ궨�� *************************/
// ����������������ֵ�ȶ�����ֵ
// 雅可比计算宏定义已集中到 robot_config.h（JACOBIAN_EPSILON、IK_JACOBIAN_ITER_MAX 等）

/************************* ö�����Ͷ��� *************************/

/**
 * @brief �ſɱȾ�������״̬ö��
 */
typedef enum
{
    JACOBIAN_SINGULAR_NONE = 0, // �����죬����
    JACOBIAN_SINGULAR_NEAR,     // �ӽ�����λ��
    JACOBIAN_SINGULAR_FULL      // ��ȫ���죬��������
} JacobianSingularStatus_t;

/**
 * @brief �ſɱȵ���IK������
 */
typedef enum
{
    IK_JACOBIAN_SUCCESS = 0,       // ���ɹ�
    IK_JACOBIAN_ERR_ITER_LIMIT,    // �ﵽ����������δ����
    IK_JACOBIAN_ERR_SINGULAR,      // ����λ�ε������ʧ��
    IK_JACOBIAN_ERR_INVALID_PARAM, // ��������Ƿ�
    IK_JACOBIAN_ERR_FK_FAILED,     // �ڲ����˶�ѧ����ʧ��
    IK_JACOBIAN_ERR_INVALID_MODE   // ��Ч��IK���ģʽ
} IKJacobianError_t;

/**
 * @brief ���˶�ѧ���ģʽ
 */
typedef enum
{
    IK_MODE_GEOMETRY = 0, // �����η�����IK
    IK_MODE_JACOBIAN,     // �ſɱȵ�����IK
    IK_MODE_AUTO          // �Զ�ģʽ�������ſɱȣ�ʧ�ܽ������η�
} IKMode_t;

/************************* �ṹ�嶨�� *************************/

/**
 * @brief 3��3�ſɱȾ���ṹ��
 * @note  ����3���ɶȽṹ������Ϊϡ��ṹ��
 *        J = [
 *          [0    J01  J02],   // vx (ǰ��)
 *          [J10  0    0  ],   // vy (����)
 *          [J20  J21  J22]    // vz (����)
 *        ]
 */
typedef struct
{
    float data[3][3];
} JacobianMat_t;

/**
 * @brief ��ʱ����Ĺ켣��ṹ��
 */
typedef struct
{
    FootPosition_t pos; // ��������ϵ���λ�� m
    float time_s;       // ʱ��� s
} TrajPoint_t;

/************************* ���⺯������ *************************/

/**
 * @brief  ������ϡ���ſɱ�����α�����
 * @param  J      ���룺��ǰ��̬�ſɱȾ���
 * @param  J_pinv �����3��3������С����α�����
 * @note   �������3���ɶ���ϡ��ṹ�Ż��������������������=0.01
 */
void matrix_pinv_lightweight(JacobianMat_t *J, float J_pinv[3][3]);

/**
 * @brief  ���㵥��3��3�ſɱȾ��󣨹ؽڽ��ٶ� �� ������ٶȣ�
 * @param  leg_id       �ȱ��
 * @param  joint_angles �ؽڽǶȣ�deg��
 * @param  J            ����ſɱȾ���
 */
void leg_jacobian_calc(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles, JacobianMat_t *J);

/**
 * @brief  �ſɱ����ٶ�ӳ�䣺�ؽڽ��ٶ� �� ����ٶ�
 * @param  J             �ſɱȾ���
 * @param  joint_vel_deg �ؽڽ��ٶȣ���/s��
 * @param  foot_vel      �������ٶȣ�m/s��
 */
void jacobian_vel_forward_map(const JacobianMat_t *J, const float joint_vel_deg[3], FootVelocity_t *foot_vel);

/**
 * @brief  �ſɱ����ٶ�ӳ�䣺���Ŀ���ٶ� �� �ؽڽ��ٶ�
 * @param  leg_id         �ȱ��
 * @param  joint_angles   ��ǰ�ؽڽǶ�
 * @param  target_foot_vel Ŀ������ٶ�
 * @param  joint_vel_deg  ����ؽڽ��ٶȣ���/s��
 */
void jacobian_vel_inverse_map(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles,
                              const FootVelocity_t *target_foot_vel, float joint_vel_deg[3]);

/**
 * @brief  ��ӳ�䣺��˽Ӵ��� �� �ؽ�����
 * @param  J            �ſɱȾ���
 * @param  foot_force   ����� (N)
 * @param  joint_torque ����ؽ����� (N��m)
 */
void jacobian_force_map(const JacobianMat_t *J, const float foot_force[3], float joint_torque[3]);

/**
 * @brief  �򻯰������ٶ�ӳ�䣺�ؽڽǼ��ٶ� �� ��˼��ٶ�
 * @param  J              �ſɱȾ���
 * @param  joint_acc_deg  �ؽڽǼ��ٶȣ���/s?��
 * @param  foot_acc       �����˼��ٶȣ�m/s?��
 */
void jacobian_acc_forward_simple(const JacobianMat_t *J, const float joint_acc_deg[3], FootAcceleration_t *foot_acc);

/**
 * @brief  �򻯰�����ٶ�ӳ�䣺��˼��ٶ� �� �ؽڽǼ��ٶ�
 * @param  leg_id        �ȱ��
 * @param  joint_angles  ��ǰ�ؽڽǶ�
 * @param  target_acc    Ŀ����˼��ٶ�
 * @param  joint_acc_deg ����ؽڽǼ��ٶ�
 */
void jacobian_acc_inverse_simple(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles,
                                 const FootAcceleration_t *target_acc, float joint_acc_deg[3]);

/**
 * @brief  �ſɱȾ��������Լ��
 * @param  J �ſɱȾ���
 * @return ����״̬
 */
JacobianSingularStatus_t jacobian_singularity_check(const JacobianMat_t *J);

/**
 * @brief  �ſɱȵ������������˶�ѧ���
 * @param  leg_id        �ȱ��
 * @param  target_pos    ��������ϵĿ�����λ��
 * @param  init_angles   ������ʼ�ؽڽ�
 * @param  result_angles ������չؽڽ�
 * @return ������
 */
IKJacobianError_t leg_ik_jacobian_solve(Leg_ID_t leg_id, const FootPosition_t *target_pos,
                                        const LegJointAngles_t *init_angles, LegJointAngles_t *result_angles);

/**
 * @brief  ͳһIK���ӿڣ�֧�ּ��η�/�ſɱ�/�Զ��л���
 * @param  leg_id        �ȱ��
 * @param  target_pos    Ŀ�����λ��
 * @param  result_angles ����ؽڽ�
 * @param  mode          ���ģʽ����ΪIK_MODE_GEOMETRY/IK_MODE_JACOBIAN/IK_MODE_AUTO������ѡ�񼸺η�
 * @return ������
 */
IKJacobianError_t leg_ik_solve(Leg_ID_t leg_id, const FootPosition_t *target_pos,
                               LegJointAngles_t *result_angles, IKMode_t mode);

#endif // JACOBIAN_MATRIX_H

/**
 ******************************************************************************
 * @file    Inverse_kinematics_calculate.h
 * @brief   ��������˵������˶�ѧ����ӿ�ͷ�ļ�
 *          �ṩ��������ϵ���λ�� -> ���ؽڽǶ�(��)�ļ��η�IK����ӿ�
 ******************************************************************************
 */

#ifndef INVERSE_KINEMATICS_CALCULATE_H
#define INVERSE_KINEMATICS_CALCULATE_H

#include "Foot_structure.h"

/*��������ϵ            /\z���ϡ�
                        |
                        |  /y����
                        | /                      ��ͷ
                        |/____>x��ǰ��
                       O(��������)
�ȷֲ�ʾ�⣨���ӣ���
 *        �����(LEG_L_B) �������������������� ��ǰ��(LEG_L_F)
 *              |                           |
 *              |                           |
 *        �Һ���(LEG_R_B) �������������������� ��ǰ��(LEG_R_F)

*/

// ͨ����ѧ��Сֵ��������㡢���㱣����
// EPSILON 已集中到 robot_config.h

// ��ָ���жϺ�
#define IS_NULL_PTR(ptr) ((ptr) == NULL)

/**
 * @brief ���˶�ѧ����״̬ö��
 */
typedef enum
{
    IK_SUCCESS = 0,             // ����ɹ����ؽڽǶ���Ч
    IK_ERROR_INVALID_PARAM = 1, // ��������Ƿ�����ָ��/��ID������Χ��
    IK_ERROR_JOINT1_LIMIT = 2,  // Joint1(�Ų�ڹؽ�)�Ƕȳ�������λ
    IK_ERROR_JOINT2_LIMIT = 3,  // Joint2(�Ÿ����ؽ�)�Ƕȳ�������λ
    IK_ERROR_JOINT3_LIMIT = 4,  // Joint3(ϥ�ؽ�)�Ƕȳ�������λ
    IK_ERROR_WORKSPACE = 5      // ���Ŀ��㳬���Ȳ��ɴ﹤���ռ�
} IK_Status_t;

/************************ ����ӿں������� ************************/

/**
 * @brief  �������˶�ѧ�����㺯��
 * @param  leg_id         �ȱ�ţ�LEG_R_F/LEG_L_F/LEG_R_B/LEG_L_B
 * @param  body_foot_pos  ���룺��������ϵ�µ����Ŀ�����꣬��λ m
 * @param  out_joint_angles ���������õ���3�ؽڽǶȣ���λ �㣬��������Ч��־λ
 * @retval IK_Status_t    ������״̬��
 * @note   �����㺯�������ڲ���̬���棬��������ʷ����
 *         ���ν���Joint1->Joint3->Joint2������λ����빤���ռ��ж�
 */
IK_Status_t foot_ik_solve_all(Leg_ID_t leg_id, const FootPosition_t *body_foot_pos,
                              LegJointAngles_t *out_joint_angles);

#endif // INVERSE_KINEMATICS_CALCULATE_H

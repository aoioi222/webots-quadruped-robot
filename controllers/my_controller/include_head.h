#ifndef INCLUDE_HEAD_H
#define INCLUDE_HEAD_H

#include <webots/supervisor.h>    // Webots Supervisor API�������Ҫʹ��Supervisor���ܣ�
#include <webots/robot.h>         // Webots Robot API
#include <webots/motor.h>         // Webots Motor API
#include <webots/gps.h>           // Webots GPS API
#include <webots/inertial_unit.h> // Webots Inertial Unit API
#include <webots/keyboard.h>

#include "head.h"
#include "robot_config.h"
#include "Foot_structure.h"
#include "Forward_kinematics_calculate.h"
#include "Inverse_kinematics_calculate.h"
#include "Jacobian_matrix.h" // �ſɱȾ�����غ����Ͷ���
#include "error_handler.h"   // ��������غ����Ͷ���

#endif // INCLUDE_HEAD_H

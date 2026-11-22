#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/device.h>
#include <webots/inertial_unit.h>
#include <webots/nodes.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "main.h"
#include "cmsis_os2.h"
#include "Foot_mode_control.h"
#include "Kalman_Filter.h"
#include "Wheel_mode_contrl.h"
#include "gait_controller.h"
#include "body_control.h"
#include "jump_controller.h"
#include "virtual_sbus.h"
#include "Inverse_kinematics_calculate.h"
#include "webots_ik_checks.h"

enum { LEG_COUNT = 4, AXIS_COUNT = 3, HIP_PITCH_AXIS = 1, KNEE_AXIS = 2, TICK_HZ = 1000 };

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart6;

static WbDeviceTag imu;
static WbDeviceTag joints[LEG_COUNT][AXIS_COUNT];
static WbDeviceTag wheels[WHEEL_COUNT];
static WbDeviceTag joint_sensors[LEG_COUNT][AXIS_COUNT];
static WbDeviceTag wheel_sensors[WHEEL_COUNT];

typedef struct
{
    double rated_torque;
    double peak_torque;
    double rated_speed;
    double free_speed;
    double peak_duration;
} MotorLimits;

typedef struct
{
    double position;
    double burst_credit;
} MotorLoad;

static const MotorLimits leg_motor_limits = {
    WEBOTS_EL05_RATED_TORQUE_NM, WEBOTS_EL05_PEAK_TORQUE_NM,
    WEBOTS_EL05_RATED_SPEED_RAD_S, WEBOTS_EL05_FREE_SPEED_RAD_S, WEBOTS_EL05_PEAK_DURATION_S,
};
static const MotorLimits wheel_motor_limits = {
    WEBOTS_H6215_RATED_TORQUE_NM, WEBOTS_H6215_PEAK_TORQUE_NM,
    WEBOTS_H6215_RATED_SPEED_RAD_S, WEBOTS_H6215_FREE_SPEED_RAD_S, WEBOTS_H6215_PEAK_DURATION_S,
};
static MotorLoad joint_load[LEG_COUNT][AXIS_COUNT];
static MotorLoad wheel_load[WHEEL_COUNT];

static const char *const leg_suffix[LEG_COUNT] = {
    [LEG_R_F] = "FR",
    [LEG_L_F] = "FL",
    [LEG_R_B] = "RR",
    [LEG_L_B] = "RL",
};
static const char *const axis_name[AXIS_COUNT] = {"HAA", "HFE", "KFE"};
static WbNodeRef support_robot;
static WbNodeRef wheel_solids[WHEEL_COUNT];
static struct
{
    WbFieldRef crank_rotation;
    WbFieldRef rod_translation;
    double last_knee_angle;
} linkage_visuals[LEG_COUNT];
static double wheel_centers[WHEEL_COUNT][3];
static int raw_contact_mask;
static unsigned climb_contact_mask;
static Angles simulation_angles;
static bool wheel_sample_available;
static FILE *wheel_trace;
static char wheel_trace_name[64];
static double wheel_trace_end_s;
static double wheel_trace_next_s;
static const int joint_sign[LEG_COUNT][AXIS_COUNT] = {
    [LEG_R_F] = {-1,  1, -1},
    [LEG_L_F] = {-1, -1,  1},
    [LEG_R_B] = { 1,  1, -1},
    [LEG_L_B] = { 1, -1,  1},
};
static bool jump_profile;
static bool smoke_test;
static WebotsScenario scenario;
static bool smoke_ik_passed;
static bool nonfinite_output;
static unsigned smoke_seen;
static double wheel_command[WHEEL_COUNT];
static double wheel_turn_limit_until;
static double initial_time;

typedef enum {DRIVE_DISTANCE, DRIVE_HEADING} WebotsDriveMetric;
typedef struct
{
    const char *name;
    WebotsDriveInput input;
    WebotsDriveMetric metric;
    double minimum;
} DriveCheck;

static const DriveCheck drive_checks[] = {
    {"climb",       {'3', 'W', false}, DRIVE_DISTANCE,  0.06},
    {"walk",        {'4', 'W', false}, DRIVE_DISTANCE,  0.05},
    {"trot",        {'5', 'W', false}, DRIVE_DISTANCE,  0.05},
    {"pronk",       {'6', 'W', false}, DRIVE_DISTANCE,  0.05},
    {"gait-back",   {'5', 'S', false}, DRIVE_DISTANCE, -0.05},
    {"gait-left",   {'5', 'Q', false}, DRIVE_HEADING,   0.15},
    {"gait-right",  {'5', 'E', false}, DRIVE_HEADING,  -0.15},
    {"wheel-front",{  0, 'W', true }, DRIVE_DISTANCE,  0.12},
    {"wheel-back", {  0, 'S', true }, DRIVE_DISTANCE, -0.12},
    {"wheel-left", {  0, 'Q', true }, DRIVE_HEADING,   0.15},
    {"wheel-right",{  0, 'E', true }, DRIVE_HEADING,  -0.15},
};
static const DriveCheck *drive_check;

enum
{
    SAW_STAND = 1u << 0,
    SAW_SWING = 1u << 1,
    SAW_JUMP = 1u << 2,
    SAW_WHEEL_TRANSITION = 1u << 3,
    SAW_DIFFERENTIAL = 1u << 4,
    SAW_EMERGENCY_STOP = 1u << 5,
    SAW_FOLD = 1u << 6,
    SAW_SHUTDOWN = 1u << 7,
    SAW_QUADRUPED_RETURN = 1u << 8,
    SAW_UNFOLD = 1u << 9,
    SAW_UNFOLD_BEFORE_WHEEL = 1u << 10,
    SAW_RESTORED_STAND = 1u << 11,
    SAW_UNFOLD_AFTER_FOLD = 1u << 12,
    SMOKE_EXPECTED = (1u << 13) - 1
};

static _Noreturn void device_error(const char *message, const char *name)
{
    fprintf(stderr, "[HAL Webots] %s: %s\n", message, name);
    wb_robot_cleanup();
    exit(EXIT_FAILURE);
}

static WbDeviceTag get_device(const char *name, WbNodeType type)
{
    WbDeviceTag device = wb_robot_get_device(name);
    if (!device)
        device_error("Missing device", name);
    if (wb_device_get_node_type(device) != type)
        device_error("Unexpected device type", name);
    return device;
}

static WbDeviceTag bind_motor(const char *axis, const char *leg, WbDeviceTag *sensor)
{
    char name[32], sensor_name[40];
    snprintf(name, sizeof name, "%s_%s", axis, leg);
    snprintf(sensor_name, sizeof sensor_name, "%s_sensor", name);
    WbDeviceTag motor = get_device(name, WB_NODE_ROTATIONAL_MOTOR);
    *sensor = get_device(sensor_name, WB_NODE_POSITION_SENSOR);
    if (wb_motor_get_position_sensor(motor) != *sensor)
        device_error("Position sensor belongs to a different joint", sensor_name);
    wb_position_sensor_enable(*sensor, TIME_STEP);
    return motor;
}

static void start_motor_load(WbDeviceTag motor, MotorLoad *load, const MotorLimits *limits)
{
    wb_motor_enable_torque_feedback(motor, TIME_STEP);
    *load = (MotorLoad){.position = NAN, .burst_credit = limits->peak_duration};
}

static void update_motor_limit(WbDeviceTag motor, WbDeviceTag sensor, MotorLoad *load,
                               const MotorLimits *limits, double torque_cap)
{
    if (!motor)
        return;

    const double dt = TIME_STEP / 1000.0;
    const double position = wb_position_sensor_get_value(sensor);
    const double torque = wb_motor_get_torque_feedback(motor);
    const double rated_squared = limits->rated_torque * limits->rated_torque;
    const double peak_squared = limits->peak_torque * limits->peak_torque;
    const double overload_span = peak_squared - rated_squared;
    double available = limits->rated_torque;
    if (isfinite(position) && isfinite(load->position) && isfinite(torque))
    {
        const double speed = fabs(position - load->position) / dt;
        /* Integrate overload using torque squared as a proxy for current squared. */
        const double load_fraction = (torque * torque - rated_squared) / overload_span;
        load->burst_credit = fmax(0.0, fmin(limits->peak_duration, load->burst_credit - load_fraction * dt));
        const double burst_fraction = fmin(1.0, load->burst_credit / dt);
        const double speed_factor = (limits->free_speed - speed) / (limits->free_speed - limits->rated_speed);
        available = sqrt(rated_squared + burst_fraction * overload_span) * fmax(0.0, fmin(1.0, speed_factor));
    }
    load->position = position;
    const double limit = fmin(torque_cap, wb_motor_get_max_torque(motor));
    wb_motor_set_available_torque(motor, fmin(available, limit));
}

static double max_leg_tracking_error_deg(void)
{
    double max_error = 0.0;
    for (int leg = 0; leg < LEG_COUNT; ++leg)
        for (int axis = HIP_PITCH_AXIS; axis < AXIS_COUNT; ++axis)
        {
            const double target = wb_motor_get_target_position(joints[leg][axis]);
            const double actual = wb_position_sensor_get_value(joint_sensors[leg][axis]);
            const double error = fabs(target - actual);
            if (!isfinite(error))
                return INFINITY;
            max_error = fmax(max_error, error);
        }
    return RAD_TO_DEG(max_error);
}

static double wheel_turn_torque_limit(void)
{
    if (wb_robot_get_time() >= wheel_turn_limit_until)
        return INFINITY;

    double hip_torque = leg_motor_limits.peak_torque;
    for (int leg = 0; leg < LEG_COUNT; ++leg)
        hip_torque = fmin(hip_torque, wb_motor_get_available_torque(joints[leg][HIP_PITCH_AXIS]));

    const double tracking_fraction = body_wheel_turn_tracking_scale((float)max_leg_tracking_error_deg());
    /* A tire force loads the hip through the leg. Share the limit to keep the turn balanced. */
    const double hip_lever = THIGH_LENGTH + CALF_LENGTH + WHEEL_RADIUS;
    return hip_torque * WHEEL_RADIUS / hip_lever * WEBOTS_TURN_SUPPORT_TORQUE_FRACTION * tracking_fraction;
}

static void update_motor_limits(void)
{
    for (int leg = 0; leg < LEG_COUNT; ++leg)
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
            update_motor_limit(joints[leg][axis], joint_sensors[leg][axis],
                               &joint_load[leg][axis], &leg_motor_limits, INFINITY);
    const double turn_limit = wheel_turn_torque_limit();
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
        update_motor_limit(wheels[leg], wheel_sensors[leg], &wheel_load[leg], &wheel_motor_limits, turn_limit);
}

static void set_joint_motion_profile(bool jumping)
{
    const double speed = JOINT_COMMAND_SLEW_RAD_S;
    /* Increase tracking response for the sagittal joints during the jump. */
    const double position_kp = jumping ? 80.0 : 24.0;
    for (int leg = 0; leg < LEG_COUNT; ++leg)
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
        {
            const WbDeviceTag motor = joints[leg][axis];
            if (axis > 0)
                wb_motor_set_control_pid(motor, position_kp, 0.0, 0.0);
            wb_motor_set_velocity(motor, fmin(speed, wb_motor_get_max_velocity(motor)));
        }
    jump_profile = jumping;
}

void Foot_Mode_Init(CAN_HandleTypeDef *can)
{
    (void)can;
    float initial_pose[FOOT_JOINT_COUNT];
    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
        {
            joints[leg][axis] = bind_motor(axis_name[axis], leg_suffix[leg], &joint_sensors[leg][axis]);
            start_motor_load(joints[leg][axis], &joint_load[leg][axis], &leg_motor_limits);
        }

        LegJointAngles_t angles = leg_stand_mode_angles((Leg_ID_t)leg);
        initial_pose[leg * AXIS_COUNT] = angles.joint1_deg;
        initial_pose[leg * AXIS_COUNT + 1] = angles.joint2_deg;
        initial_pose[leg * AXIS_COUNT + 2] = angles.joint3_deg;
    }
    set_joint_motion_profile(false);
    Foot_Mode_Control(initial_pose);
}

bool Foot_Mode_Control(const float joint_angle_deg[FOOT_JOINT_COUNT])
{
    for (int i = 0; i < FOOT_JOINT_COUNT; ++i)
    {
        if (!isfinite(joint_angle_deg[i]))
        {
            if (!nonfinite_output)
                fprintf(stderr, "[HAL Webots] Non-finite joint target at %.3f s\n", wb_robot_get_time());
            nonfinite_output = true;
            return false;
        }
    }

    bool jumping = jump_controller_is_active(&g_jump_ctrl);
    if (jumping != jump_profile)
        set_joint_motion_profile(jumping);

    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        const float *target = &joint_angle_deg[leg * AXIS_COUNT];
        LegJointAngles_t angles = leg_limit_joint_angles((Leg_ID_t)leg,
            (LegJointAngles_t){target[0], target[1], target[2], 1});
        const float degrees[AXIS_COUNT] = {angles.joint1_deg, angles.joint2_deg, angles.joint3_deg};
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
        {
            const WbDeviceTag motor = joints[leg][axis];
            const double target_rad = DEG_TO_RAD(degrees[axis]) * joint_sign[leg][axis];
            wb_motor_set_position(motor, fmax(wb_motor_get_min_position(motor),
                                             fmin(wb_motor_get_max_position(motor), target_rad)));
        }
    }
    return true;
}

void Foot_Mode_Stop(void)
{
    /* Keep the last position target so the simulated robot can be rearmed. */
    if (jump_profile)
        set_joint_motion_profile(false);
    if (smoke_test && wb_robot_get_time() >= 10.05)
        smoke_seen |= SAW_SHUTDOWN;
}

static void update_linkage_visuals(void)
{
    const double crank_length = 0.030;
    const double lateral_offset = 0.032;
    const double crank_mount_angle = 1.5707963268;
    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        if (!linkage_visuals[leg].crank_rotation || !linkage_visuals[leg].rod_translation)
            continue;
        const double angle = wb_position_sensor_get_value(joint_sensors[leg][KNEE_AXIS]);
        if (!isfinite(angle) || fabs(angle - linkage_visuals[leg].last_knee_angle) < 1e-5)
            continue;

        /* Match the knee crank mounting angle to keep the rod behind the thigh. */
        const double crank_angle = angle + crank_mount_angle;
        const double rotation[4] = {0.0, -1.0, 0.0, crank_angle};
        const double side = leg == LEG_L_F || leg == LEG_L_B ? lateral_offset : -lateral_offset;
        const double midpoint[3] = {crank_length * cos(crank_angle), side,
                                    crank_length * sin(crank_angle) - 0.5 * THIGH_LENGTH};
        wb_supervisor_field_set_sf_rotation(linkage_visuals[leg].crank_rotation, rotation);
        wb_supervisor_field_set_sf_vec3f(linkage_visuals[leg].rod_translation, midpoint);
        linkage_visuals[leg].last_knee_angle = angle;
    }
}

void Wheel_Mode_Init(CAN_HandleTypeDef *can)
{
    (void)can;
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        wheels[leg] = bind_motor("WHEEL", leg_suffix[leg], &wheel_sensors[leg]);
        start_motor_load(wheels[leg], &wheel_load[leg], &wheel_motor_limits);
        wb_motor_set_position(wheels[leg], INFINITY);
    }
    Wheel_Mode_Stop();

    if (!wb_robot_get_supervisor())
        return;
    support_robot = wb_supervisor_node_get_self();
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        char name[32];
        snprintf(name, sizeof name, "WHEEL_%s_SOLID", leg_suffix[leg]);
        wheel_solids[leg] = wb_supervisor_node_get_from_proto_def(support_robot, name);
        if (!wheel_solids[leg])
            device_error("Missing wheel contact node", name);
        wb_supervisor_node_enable_contact_points_tracking(wheel_solids[leg], TIME_STEP, false);

        snprintf(name, sizeof name, "visualCrank%s", leg_suffix[leg]);
        linkage_visuals[leg].crank_rotation = wb_supervisor_node_get_field(support_robot, name);
        snprintf(name, sizeof name, "visualRod%s", leg_suffix[leg]);
        linkage_visuals[leg].rod_translation = wb_supervisor_node_get_field(support_robot, name);
        linkage_visuals[leg].last_knee_angle = NAN;
    }
    wb_supervisor_node_enable_contact_points_tracking(support_robot, TIME_STEP, true);
    printf("[HAL Webots] 16 motors and position sensors bound. Four-wheel contact feedback enabled.\n");
}

void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT])
{
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        if (!isfinite(wheel_speed[leg]))
        {
            if (!nonfinite_output)
                fprintf(stderr, "[HAL Webots] Non-finite wheel target at %.3f s\n", wb_robot_get_time());
            nonfinite_output = true;
            Wheel_Mode_Stop();
            return;
        }
    }
    const float right = wheel_speed[LEG_R_F];
    const float left = wheel_speed[LEG_L_F];
    if (right != left)
        wheel_turn_limit_until = wb_robot_get_time() + WHEEL_TURN_BRAKING_HOLD_S;
    else if (right != 0.0f)
        wheel_turn_limit_until = 0.0;

    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        wheel_command[leg] = wheel_speed[leg];
        wb_motor_set_velocity(wheels[leg], wheel_command[leg]);
    }
}

void Wheel_Mode_Stop(void)
{
    wheel_turn_limit_until = 0.0;
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        wheel_command[leg] = 0.0;
        wb_motor_set_velocity(wheels[leg], 0.0);
    }
}

bool IMU_Fusion_Init(I2C_HandleTypeDef *i2c)
{
    (void)i2c;
    return true;
}

static int wheel_support_mask(bool *flat_ground, float *terrain_warp)
{
    static double last_contact_center[WHEEL_COUNT][3];
    static double last_contact_time[WHEEL_COUNT];
    static double terrain_xy[WHEEL_COUNT][2];
    static double terrain_z[WHEEL_COUNT];
    static int remembered_mask;
    static int terrain_mask;
    *flat_ground = false;
    *terrain_warp = NAN;
    if (!support_robot || g_robot_state.mode != ROBOT_MODE_WHEELED)
    {
        remembered_mask = 0;
        terrain_mask = 0;
        return WHEEL_CONTACT_UNAVAILABLE;
    }

    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        const double *center = wb_supervisor_node_get_position(wheel_solids[leg]);
        if (!center || !isfinite(center[0]) || !isfinite(center[1]) || !isfinite(center[2]))
        {
            remembered_mask = 0;
            terrain_mask = 0;
            return WHEEL_CONTACT_UNAVAILABLE;
        }
        memcpy(wheel_centers[leg], center, sizeof wheel_centers[leg]);
    }

    int mask = 0;
    double lowest = INFINITY;
    double highest = -INFINITY;
    double wheel_ground_z[WHEEL_COUNT] = {-INFINITY, -INFINITY, -INFINITY, -INFINITY};
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        int count;
        const WbContactPoint *points = wb_supervisor_node_get_contact_points(wheel_solids[leg], false, &count);
        for (int i = 0; points && i < count; ++i)
            if (points[i].point[2] < wheel_centers[leg][2] - 0.009)
            {
                mask |= 1 << leg;
                lowest = fmin(lowest, points[i].point[2]);
                highest = fmax(highest, points[i].point[2]);
                wheel_ground_z[leg] = fmax(wheel_ground_z[leg], points[i].point[2]);
            }
    }
    *flat_ground = mask == (1 << WHEEL_COUNT) - 1 &&
                   highest - lowest <= WHEEL_CONTACT_FLAT_TOLERANCE_M;
    raw_contact_mask = mask;

    /* 轮轴仍靠近最近触地点时延用接地状态，抑制接触点的短时闪断。 */
    const double now = wb_robot_get_time();
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        const int bit = 1 << leg;
        const double dx_ground = wheel_centers[leg][0] - terrain_xy[leg][0];
        const double dy_ground = wheel_centers[leg][1] - terrain_xy[leg][1];
        const double radius = WHEEL_CONTACT_TERRAIN_REUSE_RADIUS_M;
        if (mask & bit)
        {
            terrain_xy[leg][0] = wheel_centers[leg][0];
            terrain_xy[leg][1] = wheel_centers[leg][1];
            terrain_z[leg] = wheel_ground_z[leg];
            terrain_mask |= bit;
            memcpy(last_contact_center[leg], wheel_centers[leg], sizeof wheel_centers[leg]);
            last_contact_time[leg] = now;
            remembered_mask |= bit;
        }
        else
        {
            if (dx_ground * dx_ground + dy_ground * dy_ground > radius * radius)
                terrain_mask &= ~bit;
            if (remembered_mask & bit)
            {
                const double dx = wheel_centers[leg][0] - last_contact_center[leg][0];
                const double dy = wheel_centers[leg][1] - last_contact_center[leg][1];
                const double dz = wheel_centers[leg][2] - last_contact_center[leg][2];
                const double horizontal = WHEEL_CONTACT_POSITION_XY_TOLERANCE_M;
                if (now - last_contact_time[leg] <= WEBOTS_WHEEL_CONTACT_HOLD_S &&
                    dx * dx + dy * dy <= horizontal * horizontal &&
                    fabs(dz) <= WHEEL_CONTACT_POSITION_Z_TOLERANCE_M)
                    mask |= bit;
                else
                    remembered_mask &= ~bit;
            }
        }
    }
    if (terrain_mask == (1 << WHEEL_COUNT) - 1)
        *terrain_warp = 0.25f * (float)(terrain_z[LEG_L_F] + terrain_z[LEG_R_B] -
                                        terrain_z[LEG_R_F] - terrain_z[LEG_L_B]);
    return mask;
}

unsigned Foot_Mode_ReadSupportPoints(const GroundNormal_t *up, unsigned stance_mask,
                                     FootPosition_t contacts[WHEEL_COUNT])
{
    (void)up;
    (void)stance_mask;
    if (!support_robot)
        return 0;

    const double *orientation = wb_supervisor_node_get_orientation(support_robot);
    if (!orientation)
        return 0;
    const double heading_length = hypot(orientation[0], orientation[3]);
    if (!(heading_length > 0.5))
        return 0;
    const double forward_x = orientation[0] / heading_length;
    const double forward_y = orientation[3] / heading_length;
    const double *position = wb_supervisor_node_get_position(support_robot);
    if (!position)
        return 0;
    const double origin[] = {position[0], position[1], position[2]};
    unsigned mask = 0;
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        const double *center = wb_supervisor_node_get_position(wheel_solids[leg]);
        if (!center || !isfinite(center[2]))
            continue;
        const double center_z = center[2];
        int count;
        const WbContactPoint *points = wb_supervisor_node_get_contact_points(wheel_solids[leg], false, &count);
        double contact[3] = {0.0, 0.0, 0.0};
        int valid = 0;
        for (int i = 0; points && i < count; ++i)
        {
            const double *p = points[i].point;
            if (!(p[2] < center_z - 0.009) || !isfinite(p[0]) || !isfinite(p[1]))
                continue;
            for (int axis = 0; axis < 3; ++axis)
                contact[axis] += p[axis] - origin[axis];
            ++valid;
        }
        if (valid == 0)
            continue;
        contacts[leg] = (FootPosition_t){
            (float)((forward_x * contact[0] + forward_y * contact[1]) / valid),
            (float)((-forward_y * contact[0] + forward_x * contact[1]) / valid),
            (float)(contact[2] / valid)
        };
        mask |= 1u << leg;
    }
    climb_contact_mask = mask;
    return mask;
}

Angles *fusion_algorithm(float dt)
{
    (void)dt;
    climb_contact_mask = 0;
    wheel_sample_available = false;
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double *q = wb_inertial_unit_get_quaternion(imu);
    if (!rpy || !q || !isfinite(rpy[0]) || !isfinite(rpy[1]) || !isfinite(rpy[2]) ||
        !isfinite(q[0]) || !isfinite(q[1]) || !isfinite(q[2]) || !isfinite(q[3]))
        return NULL;

    /* 机体坐标的 +X 朝前，+Y 朝左。将姿态转换成前端和左侧抬高为正。 */
    const double left_up = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    const double front_up = 2.0 * (q[0] * q[2] - q[1] * q[3]);
    bool flat_ground;
    float terrain_warp;
    const int contact_mask = wheel_support_mask(&flat_ground, &terrain_warp);
    const double up_z = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    simulation_angles = (Angles){
        .roll = rpy[0],
        .pitch = -rpy[1],
        .yaw = rpy[2],
        .ground_up_body = {(float)front_up, (float)left_up, (float)up_z},
        .wheel_left_up = asinf(fminf(fmaxf((float)left_up, -1.0f), 1.0f)),
        .wheel_front_up = asinf(fminf(fmaxf((float)front_up, -1.0f), 1.0f)),
        .wheel_contact_mask = (int8_t)contact_mask,
        .wheel_ground_flat = flat_ground,
        .wheel_terrain_warp_m = up_z >= 0.7 ? terrain_warp / (float)up_z : NAN
    };
    wheel_sample_available = contact_mask >= 0;
    return &simulation_angles;
}

static void wheel_trace_close(void)
{
    if (!wheel_trace)
        return;
    if (fclose(wheel_trace) == 0)
        printf("[Wheel trace] Saved %s\n", wheel_trace_name);
    else
        fprintf(stderr, "[Wheel trace] Could not finish writing %s\n", wheel_trace_name);
    wheel_trace = NULL;
}

static void wheel_trace_update(void)
{
    const bool requested = virtual_sbus_take_wheel_trace_request();
    const double now = wb_robot_get_time();
    if (wheel_trace && (now >= wheel_trace_end_s || g_robot_state.mode != ROBOT_MODE_WHEELED))
        wheel_trace_close();

    if (requested && !wheel_trace && wheel_sample_available && g_robot_state.mode == ROBOT_MODE_WHEELED)
    {
        static unsigned sequence;
        snprintf(wheel_trace_name, sizeof wheel_trace_name, "wheel_trace_%llu_%u.csv",
                 (unsigned long long)time(NULL), ++sequence);
        wheel_trace = fopen(wheel_trace_name, "w");
        if (!wheel_trace)
        {
            perror("[Wheel trace] fopen");
            return;
        }
        wheel_trace_end_s = now + 6.0;
        wheel_trace_next_s = now;
        fprintf(wheel_trace, "time_s,roll_rad,pitch_rad,raw_contact,filtered_contact,flat_ground,"
                "forward_cmd_m_s,yaw_cmd_rad_s,rr_hip_target_rad,rr_knee_target_rad,"
                "lr_hip_target_rad,lr_knee_target_rad,rr_actual_z_m,lr_actual_z_m,terrain_warp_m\n");
        printf("[Wheel trace] Recording 6 s of simulation time to %s\n", wheel_trace_name);
    }
    if (!wheel_trace || !wheel_sample_available || now + 1e-8 < wheel_trace_next_s)
        return;
    wheel_trace_next_s = now + 0.01;

    const double *origin = wb_supervisor_node_get_position(support_robot);
    if (!origin)
        return;
    double center[3];
    memcpy(center, origin, sizeof center);
    const double *orientation = wb_supervisor_node_get_orientation(support_robot);
    if (!orientation)
        return;

    double rear_height[2] = {0.0, 0.0};
    for (int i = 0; i < 2; ++i)
    {
        const int leg = i == 0 ? LEG_R_B : LEG_L_B;
        for (int axis = 0; axis < 3; ++axis)
            rear_height[i] += (wheel_centers[leg][axis] - center[axis]) * orientation[axis * 3 + 2];
    }
    fprintf(wheel_trace, "%.4f,%.6f,%.6f,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            now, simulation_angles.wheel_left_up, simulation_angles.wheel_front_up,
            raw_contact_mask, simulation_angles.wheel_contact_mask, simulation_angles.wheel_ground_flat,
            g_robot_state.cmd_vel_x, g_robot_state.cmd_vel_yaw,
            wb_motor_get_target_position(joints[LEG_R_B][1]),
            wb_motor_get_target_position(joints[LEG_R_B][2]),
            wb_motor_get_target_position(joints[LEG_L_B][1]),
            wb_motor_get_target_position(joints[LEG_L_B][2]), rear_height[0], rear_height[1],
            simulation_angles.wheel_terrain_warp_m);
}

static unsigned grounded_wheels(void)
{
    unsigned mask = 0;
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        int count;
        const WbContactPoint *points = wb_supervisor_node_get_contact_points(wheel_solids[leg], false, &count);
        for (int i = 0; points && i < count; ++i)
            if (points[i].point[2] <= 0.005)
                mask |= 1u << leg;
    }
    return mask;
}

static bool measured_pose(LegJointAngles_t (*pose)(Leg_ID_t))
{
    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        const LegJointAngles_t angles = pose((Leg_ID_t)leg);
        const float degrees[] = {angles.joint1_deg, angles.joint2_deg, angles.joint3_deg};
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
        {
            const double measured = wb_position_sensor_get_value(joint_sensors[leg][axis]);
            const double expected = DEG_TO_RAD(degrees[axis]) * joint_sign[leg][axis];
            if (!isfinite(measured) || fabs(measured - expected) > 0.14)
                return false;
        }
    }
    return true;
}

static bool standing_on_floor(void)
{
    const double *orientation = wb_supervisor_node_get_orientation(support_robot);
    const double *velocity = wb_supervisor_node_get_velocity(support_robot);
    const unsigned contacts = grounded_wheels();
    if (orientation[8] < 0.96 || hypot(velocity[0], velocity[1]) > 0.05 ||
        fabs(velocity[2]) > 0.05 || !contacts)
        return false;
    /* Physics contact points can alternate between wheels resting at the same height. */
    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
        if (fabs(wb_supervisor_node_get_position(wheel_solids[leg])[2] - WHEEL_RADIUS) > 0.004)
            return false;
    return true;
}

static bool folded_wheels_outward(void)
{
    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        const double *wheel_pose = wb_supervisor_node_get_pose(wheel_solids[leg], support_robot);
        const bool left = leg == LEG_L_F || leg == LEG_L_B;
        /* The hub should move farther from the body centerline than in the neutral stance. */
        if (!wheel_pose || !((left ? wheel_pose[7] : -wheel_pose[7]) > FOOT_OFFSET_Y))
            return false;
    }
    return true;
}

static bool advance_model(int duration_ms)
{
    for (int elapsed_ms = 0; elapsed_ms < duration_ms; elapsed_ms += TIME_STEP)
    {
        update_linkage_visuals();
        if (wb_robot_step(TIME_STEP) == -1)
            return false;
    }
    return true;
}

static double fk_world_error(const FootPosition_t *local, const double body[3],
                             const double orientation[9], const double expected[3])
{
    double error_squared = 0.0;
    for (int axis = 0; axis < AXIS_COUNT; ++axis)
    {
        const double *row = &orientation[axis * AXIS_COUNT];
        const double error = body[axis] + row[0] * local->foot_pos_x_m +
                             row[1] * local->foot_pos_y_m + row[2] * local->foot_pos_z_m - expected[axis];
        error_squared += error * error;
    }
    return sqrt(error_squared);
}

static bool check_wheel_fk(int leg, double *max_center_error, double *max_contact_error)
{
    LegJointAngles_t q = {
        RAD_TO_DEG(wb_position_sensor_get_value(joint_sensors[leg][0])) * joint_sign[leg][0],
        RAD_TO_DEG(wb_position_sensor_get_value(joint_sensors[leg][1])) * joint_sign[leg][1],
        RAD_TO_DEG(wb_position_sensor_get_value(joint_sensors[leg][2])) * joint_sign[leg][2],
        1
    };
    FootPosition_t center, contact;
    if (leg_forward_kinematics_matrix((Leg_ID_t)leg, q, &center) != FK_SUCCESS ||
        leg_forward_wheel_contact((Leg_ID_t)leg, q, &g_robot_state.ground_up_body, &contact) != FK_SUCCESS)
        return false;

    const double *body = wb_supervisor_node_get_position(support_robot);
    const double *orientation = wb_supervisor_node_get_orientation(support_robot);
    const double *wheel = wb_supervisor_node_get_position(wheel_solids[leg]);
    const double *wheel_orientation = wb_supervisor_node_get_orientation(wheel_solids[leg]);
    const double axle[] = {wheel_orientation[1], wheel_orientation[4], wheel_orientation[7]};
    const double down_plane[] = {axle[0] * axle[2], axle[1] * axle[2], axle[2] * axle[2] - 1.0};
    const double length = hypot(down_plane[0], hypot(down_plane[1], down_plane[2]));
    if (length <= EPSILON)
        return false;

    const double actual_contact[] = {wheel[0] + WHEEL_RADIUS * down_plane[0] / length,
                                     wheel[1] + WHEEL_RADIUS * down_plane[1] / length,
                                     wheel[2] + WHEEL_RADIUS * down_plane[2] / length};
    const double center_error = fk_world_error(&center, body, orientation, wheel);
    const double contact_error = fk_world_error(&contact, body, orientation, actual_contact);
    if (!isfinite(center_error) || !isfinite(contact_error))
        return false;
    *max_center_error = fmax(*max_center_error, center_error);
    *max_contact_error = fmax(*max_contact_error, contact_error);
    return true;
}

static bool check_model_devices(double *max_center_error, double *max_contact_error)
{
    float pose[FOOT_JOINT_COUNT];
    for (int leg = 0; leg < LEG_COUNT; ++leg)
    {
        const LegJointAngles_t stand = leg_stand_mode_angles((Leg_ID_t)leg);
        pose[leg * AXIS_COUNT] = stand.joint1_deg;
        pose[leg * AXIS_COUNT + 1] = stand.joint2_deg;
        pose[leg * AXIS_COUNT + 2] = stand.joint3_deg;
    }
    Foot_Mode_Init(NULL);
    Wheel_Mode_Init(NULL);
    if (!advance_model(250))
        return false;

    bool passed = true;
    for (int leg = 0; leg < LEG_COUNT; ++leg)
        for (int axis = 0; axis < AXIS_COUNT; ++axis)
        {
            float *angle = &pose[leg * AXIS_COUNT + axis];
            const float previous = *angle;
            const double initial = wb_position_sensor_get_value(joint_sensors[leg][axis]);
            *angle += 3.0f * joint_sign[leg][axis];
            if (!Foot_Mode_Control(pose) || !advance_model(250) || !fusion_algorithm(0.0f))
                return false;
            g_robot_state.ground_up_body = simulation_angles.ground_up_body;
            const double moved = wb_position_sensor_get_value(joint_sensors[leg][axis]) - initial;
            passed &= isfinite(moved) && fabs(moved - DEG_TO_RAD(3.0)) <= 0.012;
            for (int i = 0; i < LEG_COUNT; ++i)
                passed &= check_wheel_fk(i, max_center_error, max_contact_error);
            *angle = previous;
            if (!Foot_Mode_Control(pose))
                return false;
        }

    for (int leg = 0; leg < WHEEL_COUNT; ++leg)
    {
        float speed[WHEEL_COUNT] = {0.0f};
        speed[leg] = 1.0f;
        const double initial = wb_position_sensor_get_value(wheel_sensors[leg]);
        Wheel_Mode_Control(speed);
        if (!advance_model(300))
            return false;
        const double moved = wb_position_sensor_get_value(wheel_sensors[leg]) - initial;
        passed &= isfinite(moved) && moved > 0.20 && moved < 0.40;
    }
    Wheel_Mode_Stop();
    return passed && *max_center_error <= 0.002 && *max_contact_error <= 0.002 && !nonfinite_output;
}

static int run_model_test(void)
{
    double max_center_error = 0.0, max_contact_error = 0.0;
    const bool passed = check_model_devices(&max_center_error, &max_contact_error);
    const char *format = "[HAL Webots] Model %s (16 motor directions, center FK error %.6f m, "
                         "contact FK error %.6f m)\n";
    const char *status = passed ? "PASS" : "FAIL";
    printf(format, status, max_center_error, max_contact_error);
    FILE *result = fopen("hal_model_result.txt", "w");
    if (result)
    {
        fprintf(result, format, status, max_center_error, max_contact_error);
        fclose(result);
    }
    wb_supervisor_simulation_quit(passed ? EXIT_SUCCESS : EXIT_FAILURE);
    wb_robot_step(TIME_STEP);
    wb_robot_cleanup();
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}

static void smoke_check(void)
{
    const double now = wb_robot_get_time();
    const LegStateMachine_t *leg = &leg_fsm[LEG_R_F];
    bool stopped = true;
    for (int i = 0; i < WHEEL_COUNT; ++i)
        stopped = stopped && wheel_command[i] == 0.0;

    if (now > 0.60 && now < 1.10 && leg->current_state == LEG_STATE_UNFOLD)
        smoke_seen |= SAW_UNFOLD;
    if (now > 4.10 && now < 4.35 && leg->current_state == LEG_STATE_UNFOLD && stopped)
        smoke_seen |= SAW_UNFOLD_BEFORE_WHEEL;
    if (now > 1.10 && now < 1.30 && leg->current_state == LEG_STATE_STANCE && standing_on_floor())
        smoke_seen |= SAW_STAND;
    if (leg->current_state == LEG_STATE_SWING)
        smoke_seen |= SAW_SWING;
    if (jump_controller_is_active(&g_jump_ctrl))
        smoke_seen |= SAW_JUMP;
    if (leg->current_state == LEG_STATE_WHEEL && leg->phase < 1.0f && stopped)
        smoke_seen |= SAW_WHEEL_TRANSITION;
    if (wheel_command[LEG_R_F] > wheel_command[LEG_L_F] && wheel_command[LEG_L_F] > 0.0 &&
        wheel_command[LEG_R_F] == wheel_command[LEG_R_B] &&
        wheel_command[LEG_L_F] == wheel_command[LEG_L_B])
        smoke_seen |= SAW_DIFFERENTIAL;
    if (now > 5.86 && now < 5.94 && stopped && leg->current_state == LEG_STATE_WHEEL)
        smoke_seen |= SAW_EMERGENCY_STOP;
    if (now > 6.55 && now < 6.80 && stopped && gait_controller_all_legs_quadruped())
        smoke_seen |= SAW_QUADRUPED_RETURN;
    if (leg->current_state == LEG_STATE_FOLD && measured_pose(leg_fold_mode_angles) && folded_wheels_outward())
        smoke_seen |= SAW_FOLD;
    if (now > 8.15 && now < 9.00 && leg->current_state == LEG_STATE_UNFOLD)
        smoke_seen |= SAW_UNFOLD_AFTER_FOLD;
    if (now > 9.55 && now < 10.05 && leg->current_state == LEG_STATE_STANCE && standing_on_floor())
        smoke_seen |= SAW_RESTORED_STAND;

    if (now >= 10.25)
    {
        bool passed = smoke_seen == SMOKE_EXPECTED && smoke_ik_passed && !nonfinite_output && stopped;
        const char *format = "[HAL Webots] Smoke %s (mode checks 0x%02x/0x%02x, IK %s)\n";
        const char *status = passed ? "PASS" : "FAIL";
        const char *ik_status = smoke_ik_passed ? "PASS" : "FAIL";
        printf(format, status, smoke_seen, SMOKE_EXPECTED, ik_status);
        fflush(stdout);
        FILE *result = fopen("hal_smoke_result.txt", "w");
        if (result)
        {
            fprintf(result, format, status, smoke_seen, SMOKE_EXPECTED, ik_status);
            fclose(result);
        }
        wb_supervisor_simulation_quit(passed ? EXIT_SUCCESS : EXIT_FAILURE);
    }
}

static void drive_motion_check(void)
{
    const double now = wb_robot_get_time();
    if (now < WEBOTS_DRIVE_START_S)
        return;

    static bool initialized;
    static bool expected_mode = true;
    static double origin[2];
    static double heading[2];
    static double min_up = 1.0;
    static unsigned ik_failure_frames;
    static unsigned contact_mask;
    static FILE *trace;
    static double max_wheel_speed;
    static double max_tracking_error;
    static unsigned airborne_frames;
    static unsigned max_airborne_frames;
    static unsigned max_swing_legs;
    const double *position = wb_supervisor_node_get_position(support_robot);
    const double *orientation = wb_supervisor_node_get_orientation(support_robot);
    if (!initialized)
    {
        memcpy(origin, position, sizeof origin);
        const double length = hypot(orientation[0], orientation[3]);
        heading[0] = orientation[0] / length;
        heading[1] = orientation[3] / length;
        if (drive_check->input.gait_key == '6' || drive_check->input.wheeled)
        {
            char name[64];
            snprintf(name, sizeof name, "hal_%s_trace.csv", drive_check->name);
            trace = fopen(name, "w");
            if (trace)
                fprintf(trace, "time,forward,height,contacts,right_cmd,left_cmd,right_angle,left_angle,leg_state,phase\n");
        }
        initialized = true;
    }
    min_up = fmin(min_up, orientation[8]);
    max_tracking_error = fmax(max_tracking_error, max_leg_tracking_error_deg());
    const bool climb = drive_check->input.gait_key == '3';
    const unsigned contacts = climb ? climb_contact_mask : grounded_wheels();
    contact_mask |= contacts;
    airborne_frames = contacts ? 0 : airborne_frames + 1;
    if (airborne_frames > max_airborne_frames)
        max_airborne_frames = airborne_frames;
    max_wheel_speed = fmax(max_wheel_speed, fabs(wheel_command[LEG_R_F]));
    if (trace)
    {
        const double forward = (position[0] - origin[0]) * heading[0] +
                               (position[1] - origin[1]) * heading[1];
        fprintf(trace, "%.3f,%.5f,%.5f,%u,%.5f,%.5f,%.5f,%.5f,%d,%.4f\n",
                now, forward, position[2], contacts, wheel_command[LEG_R_F], wheel_command[LEG_L_F],
                wb_position_sensor_get_value(wheel_sensors[LEG_R_F]),
                wb_position_sensor_get_value(wheel_sensors[LEG_L_F]),
                (int)leg_fsm[LEG_R_F].current_state, leg_fsm[LEG_R_F].phase);
    }
    expected_mode &= (g_robot_state.mode == ROBOT_MODE_WHEELED) == drive_check->input.wheeled;
    expected_mode &= !climb || g_robot_state.quad_sub_mode == QUAD_MODE_CRAWL;
    unsigned swing_legs = 0;
    for (int i = 0; i < LEG_COUNT; ++i)
    {
        ik_failure_frames += leg_fsm[i].ik_fail_count != 0;
        swing_legs += leg_fsm[i].current_state == LEG_STATE_SWING;
    }
    if (swing_legs > max_swing_legs)
        max_swing_legs = swing_legs;
    if (now < WEBOTS_DRIVE_END_S)
        return;

    const double forward = (position[0] - origin[0]) * heading[0] +
                           (position[1] - origin[1]) * heading[1];
    const double yaw = atan2(heading[0] * orientation[3] - heading[1] * orientation[0],
                             heading[0] * orientation[0] + heading[1] * orientation[3]);
    const double actual = drive_check->metric == DRIVE_DISTANCE ? forward : yaw;
    const double minimum = drive_check->minimum;
    const bool moved = minimum > 0.0 ? actual >= minimum : actual <= minimum;
    const bool hops = drive_check->input.gait_key != '6' || max_airborne_frames * TIME_STEP >= 20;
    const bool wheel_turn = drive_check->input.wheeled && drive_check->metric == DRIVE_HEADING;
    const bool tracks_pose = !wheel_turn || max_tracking_error <= 5.0;
    const bool stable_climb = !climb || (max_swing_legs == 1 && max_airborne_frames == 0 && ik_failure_frames <= 4);
    const bool passed = moved && hops && expected_mode && tracks_pose && stable_climb &&
                        min_up >= 0.90 && !nonfinite_output &&
                        contact_mask == (1u << WHEEL_COUNT) - 1;
    const char *format = "[HAL Webots] Drive %s %s (forward %+.3f m, heading %+.3f rad, min upright %.3f, "
                         "mode %s, contact 0x%x, IK failure frames %u, wheel command %.3f rad/s, "
                         "max airtime %u ms, max leg tracking error %.2f deg)\n";
    char filename[64];
    snprintf(filename, sizeof filename, "hal_%s_result.txt", drive_check->name);
    FILE *result = fopen(filename, "w");
    if (result)
    {
        fprintf(result, format, drive_check->name, passed ? "PASS" : "FAIL", forward, yaw,
                min_up, expected_mode ? "OK" : "wrong", contact_mask, ik_failure_frames,
                max_wheel_speed, max_airborne_frames * TIME_STEP, max_tracking_error);
        fclose(result);
    }
    printf(format, drive_check->name, passed ? "PASS" : "FAIL", forward, yaw,
           min_up, expected_mode ? "OK" : "wrong", contact_mask, ik_failure_frames,
           max_wheel_speed, max_airborne_frames * TIME_STEP, max_tracking_error);
    if (trace)
        fclose(trace);
    wb_supervisor_simulation_quit(passed ? EXIT_SUCCESS : EXIT_FAILURE);
}

static void lateral_check(void)
{
    const double now = wb_robot_get_time();
    if (now < WEBOTS_LATERAL_START_S)
        return;

    static double origin[2];
    static double lateral_axis[2];
    static double min_up = 1.0;
    static unsigned samples;
    static unsigned ik_failure_frames;
    const WbNodeRef robot = wb_supervisor_node_get_self();
    const double *position = wb_supervisor_node_get_position(robot);
    const double *orientation = wb_supervisor_node_get_orientation(robot);
    if (samples == 0)
    {
        memcpy(origin, position, sizeof origin);
        lateral_axis[0] = orientation[1];
        lateral_axis[1] = orientation[4];
    }
    ++samples;
    min_up = fmin(min_up, orientation[8]);
    for (int i = 0; i < LEG_COUNT; ++i)
        ik_failure_frames += leg_fsm[i].ik_fail_count != 0;
    if (now < WEBOTS_LATERAL_START_S + 4.20)
        return;

    const bool left = scenario == WEBOTS_LATERAL_LEFT;
    const double dx = position[0] - origin[0], dy = position[1] - origin[1];
    const double lateral = dx * lateral_axis[0] + dy * lateral_axis[1];
    const double travel = left ? lateral : -lateral;
    const bool passed = travel >= 0.20 && min_up >= 0.90 && !nonfinite_output;
    const char *format = "[HAL Webots] Lateral %s %s (travel %.3f m, min upright %.3f, IK failure frames %u/%u)\n";
    const char *direction = left ? "left" : "right";
    const char *status = passed ? "PASS" : "FAIL";
    printf(format, direction, status, travel, min_up, ik_failure_frames, samples * LEG_COUNT);
    fflush(stdout);
    FILE *report = fopen(left ? "hal_lateral_left_result.txt" : "hal_lateral_right_result.txt", "w");
    if (report)
    {
        fprintf(report, format, direction, status, travel, min_up, ik_failure_frames, samples * LEG_COUNT);
        fclose(report);
    }
    wb_supervisor_simulation_quit(passed ? EXIT_SUCCESS : EXIT_FAILURE);
}

static void jump_check(void)
{
    const double now = wb_robot_get_time();
    static bool finished;
    if (finished || now < WEBOTS_JUMP_START_S - 0.10)
        return;

    static bool started;
    static bool grounded_before_push;
    static double start_height;
    static double origin_x, origin_y, forward_x, forward_y;
    static double touchdown_forward = NAN;
    static double touchdown_time;
    static double lift_forward;
    static double touchdown_pitch;
    static double peak_nose_down, peak_nose_up;
    static double front_touch = -1.0, rear_touch = -1.0;
    static double peak_height;
    static double min_up = 1.0;
    static double peak_vz;
    static double lifted_at = -1.0;
    static double longest_flight;
    static unsigned settled_frames;
    static FILE *trace;

    const WbNodeRef robot = wb_supervisor_node_get_self();
    const double *position = wb_supervisor_node_get_position(robot);
    const double x = position[0], y = position[1], height = position[2];
    const double *velocity = wb_supervisor_node_get_velocity(robot);
    const double vz = velocity[2];
    const double horizontal_speed = hypot(velocity[0], velocity[1]);
    const double *orientation = wb_supervisor_node_get_orientation(robot);
    const double up = orientation[8];
    const double nose_down = -atan2(orientation[6], hypot(orientation[0], orientation[3])) *
                             (180.0 / 3.141592653589793);
    int point_count = 0;
    const WbContactPoint *points = wb_supervisor_node_get_contact_points(robot, true, &point_count);
    bool grounded = false;
    /* The floor of the test world is at z = 0. Ignore contacts above it. */
    bool front_grounded = false, rear_grounded = false;
    for (int i = 0; i < point_count; ++i)
    {
        const double *point = points[i].point;
        if (point[2] > 0.005)
            continue;
        grounded = true;
        const double longitudinal = (point[0] - x) * orientation[0] +
                                    (point[1] - y) * orientation[3];
        front_grounded |= longitudinal > 0.05;
        rear_grounded |= longitudinal < -0.05;
    }

    const JumpPhase_t phase = g_jump_ctrl.phase;
    if (!started)
    {
        origin_x = x;
        origin_y = y;
        const double heading_norm = hypot(orientation[0], orientation[3]);
        forward_x = orientation[0] / heading_norm;
        forward_y = orientation[3] / heading_norm;
        start_height = height;
        peak_height = height;
        started = jump_controller_is_active(&g_jump_ctrl);
        if (started)
        {
            trace = fopen("hal_jump_trace.csv", "w");
            if (trace)
                fprintf(trace, "time,phase,height,vz,up,grounded,forward,lateral,"
                               "nose_down,front_grounded,rear_grounded,hip_rad,knee_rad,"
                               "hip_torque_nm,knee_torque_nm,hip_limit_nm,knee_limit_nm\n");
        }
    }
    const double dx = x - origin_x, dy = y - origin_y;
    const double forward = forward_x * dx + forward_y * dy;
    const double lateral = -forward_y * dx + forward_x * dy;
    if (started)
    {
        min_up = fmin(min_up, up);
        if (phase == JUMP_PUSH_OFF)
        {
            grounded_before_push |= grounded;
            peak_vz = fmax(peak_vz, vz);
        }
        if (grounded)
        {
            if (lifted_at >= 0.0 && now - lifted_at >= 0.020 - 1e-9 && !isfinite(touchdown_forward))
            {
                touchdown_forward = forward - lift_forward;
                touchdown_time = now;
                touchdown_pitch = nose_down;
            }
            lifted_at = -1.0;
        }
        else if (!isfinite(touchdown_forward) && grounded_before_push &&
                 phase >= JUMP_PUSH_OFF && phase <= JUMP_LANDING)
        {
            if (lifted_at < 0.0)
            {
                lifted_at = now;
                lift_forward = forward;
            }
            peak_height = fmax(peak_height, height);
            longest_flight = fmax(longest_flight, now - lifted_at + TIME_STEP / 1000.0);
        }
        if (isfinite(touchdown_forward))
        {
            if (front_grounded && front_touch < 0.0)
                front_touch = now;
            if (rear_grounded && rear_touch < 0.0)
                rear_touch = now;
            peak_nose_down = fmax(peak_nose_down, nose_down);
            peak_nose_up = fmax(peak_nose_up, -nose_down);
        }
        if (trace)
            fprintf(trace, "%.3f,%d,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%.4f,%d,%d,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f\n",
                    now, (int)phase, height, vz, up, grounded, forward, lateral,
                    nose_down, front_grounded, rear_grounded,
                    wb_position_sensor_get_value(joint_sensors[LEG_R_F][1]),
                    wb_position_sensor_get_value(joint_sensors[LEG_R_F][2]),
                    wb_motor_get_torque_feedback(joints[LEG_R_F][1]),
                    wb_motor_get_torque_feedback(joints[LEG_R_F][2]),
                    wb_motor_get_available_torque(joints[LEG_R_F][1]),
                    wb_motor_get_available_torque(joints[LEG_R_F][2]));
    }

    const bool at_rest = phase == JUMP_DONE && grounded && up >= 0.96 &&
                         fabs(height - start_height) <= 0.025 && fabs(vz) <= 0.10 &&
                         horizontal_speed <= 0.12 &&
                         now <= touchdown_time + 2.0;
    if (settled_frames * TIME_STEP < 100)
        settled_frames = isfinite(touchdown_forward) && at_rest ? settled_frames + 1 : 0;
    if (now < WEBOTS_JUMP_START_S + 4.0 &&
        (!isfinite(touchdown_forward) || now < touchdown_time + 2.0))
        return;

    finished = true;
    const bool recovered = settled_frames * TIME_STEP >= 100;
    const double rise = peak_height - start_height;
    const double settled_forward = forward - lift_forward;
    const bool reached_target = rise >= 0.065 && touchdown_forward >= 0.160;
    const double contact_gap_ms = (front_touch >= 0.0 && rear_touch >= 0.0) ?
                                  (rear_touch - front_touch) * 1000.0 : NAN;
    const bool controlled_landing = fabs(contact_gap_ms) <= 30.0 + 1e-9 &&
                                    fabs(settled_forward - touchdown_forward) <= 0.080;
    const bool passed = started && grounded_before_push && longest_flight >= 0.200 - 1e-9 &&
                        reached_target && min_up >= 0.96 && controlled_landing && recovered && !nonfinite_output;
    const char *format = "[HAL Webots] Jump %s (rise %.3f m, flight %.3f s, push vz %.3f m/s, "
                         "min upright %.3f, recovered %d, "
                         "touchdown forward %.3f m, settled forward %.3f m, lateral %.3f m, "
                         "touchdown pitch %.1f deg, max nose down %.1f deg, max nose up %.1f deg, "
                         "rear minus front contact %.0f ms)\n";
    const char *status = passed ? "PASS" : "FAIL";
    printf(format, status, rise, longest_flight, peak_vz, min_up, recovered,
           touchdown_forward, settled_forward, lateral, touchdown_pitch, peak_nose_down, peak_nose_up,
           contact_gap_ms);
    fflush(stdout);
    FILE *result = fopen("hal_jump_result.txt", "w");
    if (result)
    {
        fprintf(result, format, status, rise, longest_flight, peak_vz, min_up, recovered,
                touchdown_forward, settled_forward, lateral, touchdown_pitch, peak_nose_down, peak_nose_up,
                contact_gap_ms);
        fclose(result);
    }
    if (trace)
        fclose(trace);
    wb_supervisor_simulation_quit(passed ? EXIT_SUCCESS : EXIT_FAILURE);
}

uint32_t osKernelGetTickFreq(void)
{
    return TICK_HZ;
}

uint32_t osKernelGetTickCount(void)
{
    return (uint32_t)(uint64_t)llround((wb_robot_get_time() - initial_time) * TICK_HZ);
}

_Noreturn void osThreadExit(void)
{
    wheel_trace_close();
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
}

osStatus_t osDelayUntil(uint32_t deadline)
{
    while ((int32_t)(deadline - osKernelGetTickCount()) > 0)
    {
        wheel_trace_update();
        if (smoke_test)
            smoke_check();
        if (scenario == WEBOTS_LATERAL_LEFT || scenario == WEBOTS_LATERAL_RIGHT)
            lateral_check();
        if (scenario == WEBOTS_JUMP)
            jump_check();
        if (scenario == WEBOTS_DRIVE)
            drive_motion_check();
        update_motor_limits();
        update_linkage_visuals();
        if (wb_robot_step(TIME_STEP) == -1)
            osThreadExit();
    }
    return osOK;
}

osStatus_t osDelay(uint32_t ticks)
{
    return osDelayUntil(osKernelGetTickCount() + ticks);
}

int main(int argc, char **argv)
{
    wb_robot_init();
    const bool model_test = argc > 1 && strcmp(argv[1], "--model-test") == 0;
    if (argc > 1 && strcmp(argv[1], "--smoke-test") == 0)
        scenario = WEBOTS_SMOKE;
    if (argc > 1 && strcmp(argv[1], "--lateral-left") == 0)
        scenario = WEBOTS_LATERAL_LEFT;
    if (argc > 1 && strcmp(argv[1], "--lateral-right") == 0)
        scenario = WEBOTS_LATERAL_RIGHT;
    if (argc > 1 && strcmp(argv[1], "--jump-test") == 0)
        scenario = WEBOTS_JUMP;
    if (argc > 1 && strcmp(argv[1], "--drive-test") == 0)
    {
        for (unsigned i = 0; argc > 2 && i < sizeof drive_checks / sizeof drive_checks[0]; ++i)
            if (strcmp(argv[2], drive_checks[i].name) == 0)
                drive_check = &drive_checks[i];
        if (!drive_check)
            device_error("Unknown drive test", argc > 2 ? argv[2] : "(missing name)");
        scenario = WEBOTS_DRIVE;
        virtual_sbus_set_drive_input(drive_check->input);
    }
    if ((scenario != WEBOTS_MANUAL || model_test) && !wb_robot_get_supervisor())
    {
        fprintf(stderr, "[HAL Webots] Regression scenarios require a Supervisor robot\n");
        wb_robot_cleanup();
        return EXIT_FAILURE;
    }
    smoke_test = scenario == WEBOTS_SMOKE;
    if (smoke_test)
        smoke_ik_passed = webots_run_ik_checks();
    double basic_step = wb_robot_get_basic_time_step();
    if (basic_step <= 0.0 || fmod(TIME_STEP, basic_step) != 0.0)
    {
        fprintf(stderr, "[HAL Webots] TIME_STEP=%d ms must be a multiple of basicTimeStep=%g ms\n",
                TIME_STEP, basic_step);
        wb_robot_cleanup();
        return EXIT_FAILURE;
    }
    imu = get_device("imu", WB_NODE_INERTIAL_UNIT);
    wb_inertial_unit_enable(imu, TIME_STEP);
    virtual_sbus_init(scenario);
    if (wb_robot_step(TIME_STEP) == -1)
        osThreadExit();
    initial_time = wb_robot_get_time();
    if (model_test)
        return run_model_test();

    printf("[HAL Webots] %d ms, HAL task active. I: stand, O: gait, 3: climb, 6: pronk, T: wheel trace, Esc: shut down.\n", TIME_STEP);
    fflush(stdout);
    UpctrlTaskMain(NULL);
    osThreadExit();
}

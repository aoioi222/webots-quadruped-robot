#include "webots_ik_checks.h"
#include "leg_state_machine.h"
#include "Inverse_kinematics_calculate.h"
#include "gait_point_kinematics.h"

static bool check_projection(void)
{
    const float roll_deg = 0.75f * JOINT1_MAX;
    const float roll = DEG_TO_RAD(roll_deg);
    const float pitch = DEG_TO_RAD(45.0f);
    const float depth = THIGH_LENGTH * cosf(pitch) + CALF_LENGTH * cosf(-pitch);
    FootPosition_t target = {
        HIP_OFFSET_X_R_F - LEG_SAGITTAL_X_SIGN * (THIGH_LENGTH - CALF_LENGTH) * sinf(pitch),
        HIP_OFFSET_Y_R_F - HIP_ABD_LENGTH * cosf(roll) - depth * sinf(roll),
        HIP_OFFSET_Z_R_F + HIP_ABD_LENGTH * sinf(roll) - depth * cosf(roll)
    };
    LegJointAngles_t q;
    return foot_ik_solve_all(LEG_R_F, &target, &q) == IK_SUCCESS && q.valid &&
           fabsf(q.joint1_deg - roll_deg) < 0.01f && fabsf(q.joint2_deg - 45.0f) < 0.01f &&
           fabsf(q.joint3_deg + 90.0f) < 0.01f;
}

static bool check_round_trip(void)
{
    const float rolls[] = {-30.0f, -15.0f, 0.0f, 15.0f, 30.0f};
    const float pitches[] = {-40.0f, 0.0f, 35.0f, 65.0f, 90.0f};
    const float bends[] = {0.0f, 35.0f, 90.0f, 120.0f, 150.0f};

    for (Leg_ID_t leg = LEG_R_F; leg <= LEG_L_B; ++leg)
    {
        float sign = leg == LEG_R_F || leg == LEG_R_B ? 1.0f : -1.0f;
        for (unsigned i = 0; i < sizeof rolls / sizeof rolls[0]; ++i)
            for (unsigned j = 0; j < sizeof pitches / sizeof pitches[0]; ++j)
                for (unsigned k = 0; k < sizeof bends / sizeof bends[0]; ++k)
                {
                    LegJointAngles_t q = leg_limit_joint_angles(leg,
                        (LegJointAngles_t){rolls[i], sign * pitches[j], -sign * bends[k], 1});
                    FootPosition_t target, actual;
                    if (leg_forward_kinematics_matrix(leg, q, &target) != FK_SUCCESS ||
                        foot_ik_solve_all(leg, &target, &q) != IK_SUCCESS || !q.valid ||
                        leg_forward_kinematics_matrix(leg, q, &actual) != FK_SUCCESS)
                        return false;

                    float dx = actual.foot_pos_x_m - target.foot_pos_x_m;
                    float dy = actual.foot_pos_y_m - target.foot_pos_y_m;
                    float dz = actual.foot_pos_z_m - target.foot_pos_z_m;
                    const float tolerance_m = 0.0001f;
                    if (dx * dx + dy * dy + dz * dz > tolerance_m * tolerance_m)
                        return false;
                }
    }
    return true;
}

static float point_distance(FootPosition_t a, FootPosition_t b)
{
    return hypotf(a.foot_pos_x_m - b.foot_pos_x_m,
                  hypotf(a.foot_pos_y_m - b.foot_pos_y_m, a.foot_pos_z_m - b.foot_pos_z_m));
}

static bool check_contact_projection(void)
{
    const GroundNormal_t level = {.z = 1.0f};
    const float pitch = DEG_TO_RAD(10.0f);
    const GroundNormal_t pitched = {-sinf(pitch), 0.0f, cosf(pitch)};
    const float roll = DEG_TO_RAD(12.0f);
    LegJointAngles_t q = leg_stand_mode_angles(LEG_R_F);
    FootPosition_t center, contact;
    if (leg_forward_kinematics_matrix(LEG_R_F, q, &center) != FK_SUCCESS ||
        leg_forward_wheel_contact(LEG_R_F, q, &level, &contact) != FK_SUCCESS)
        return false;

    const FootPosition_t level_expected = {center.foot_pos_x_m, center.foot_pos_y_m,
                                           center.foot_pos_z_m - WHEEL_RADIUS};
    if (point_distance(contact, level_expected) > 1e-6f ||
        leg_forward_wheel_contact(LEG_R_F, q, &pitched, &contact) != FK_SUCCESS)
        return false;

    const FootPosition_t pitched_expected = {center.foot_pos_x_m + WHEEL_RADIUS * sinf(pitch),
                                             center.foot_pos_y_m,
                                             center.foot_pos_z_m - WHEEL_RADIUS * cosf(pitch)};
    if (point_distance(contact, pitched_expected) > 1e-6f)
        return false;

    q.joint1_deg = -12.0f;
    if (leg_forward_kinematics_matrix(LEG_R_F, q, &center) != FK_SUCCESS ||
        leg_forward_wheel_contact(LEG_R_F, q, &level, &contact) != FK_SUCCESS)
        return false;
    const FootPosition_t rolled_expected = {center.foot_pos_x_m,
                                            center.foot_pos_y_m + WHEEL_RADIUS * sinf(roll),
                                            center.foot_pos_z_m - WHEEL_RADIUS * cosf(roll)};
    return point_distance(contact, rolled_expected) <= 1e-6f;
}

static bool check_contact_round_trip(void)
{
    const float rolls[] = {-JOINT1_MAX, -6.0f, 0.0f, 6.0f, JOINT1_MAX};
    const float pitches[] = {-JOINT2_MAX, 25.0f, 35.0f, 55.0f, -JOINT2_MIN};
    const float knees[] = {JOINT3_MIN, 45.0f, 70.0f, 100.0f, JOINT3_CONTROL_MAX};
    const GroundNormal_t normals[] = {{0.0f, 0.0f, 1.0f}, {-0.31f, 0.25f, 0.917f},
                                      {0.42f, -0.35f, 0.837f}};
    for (Leg_ID_t leg = LEG_R_F; leg <= LEG_L_B; ++leg)
    {
        const float side_sign = leg == LEG_R_F || leg == LEG_R_B ? 1.0f : -1.0f;
        for (unsigned n = 0; n < sizeof normals / sizeof normals[0]; ++n)
            for (unsigned i = 0; i < sizeof rolls / sizeof rolls[0]; ++i)
                for (unsigned j = 0; j < sizeof pitches / sizeof pitches[0]; ++j)
                    for (unsigned k = 0; k < sizeof knees / sizeof knees[0]; ++k)
                    {
                        LegJointAngles_t q = {rolls[i], side_sign * pitches[j], -side_sign * knees[k], 1};
                        FootPosition_t target, actual;
                        if (leg_forward_wheel_contact(leg, q, &normals[n], &target) != FK_SUCCESS ||
                            wheel_contact_ik_solve(leg, &target, &normals[n], &q) != IK_SUCCESS || !q.valid ||
                            leg_forward_wheel_contact(leg, q, &normals[n], &actual) != FK_SUCCESS ||
                            point_distance(target, actual) >= 0.0001f)
                            return false;
                    }
    }
    return true;
}

static bool check_contact_limits(void)
{
    const GroundNormal_t level = {.z = 1.0f};
    const GroundNormal_t axle_aligned = {.y = 1.0f};
    const GroundNormal_t invalid = {.z = NAN};
    const FootPosition_t nominal = {LEG_R_F_START_POS_X, LEG_R_F_START_POS_Y, LEG_R_F_START_POS_Z};
    const FootPosition_t unreachable = {nominal.foot_pos_x_m + FULL_LEN + WHEEL_RADIUS, nominal.foot_pos_y_m,
                                        nominal.foot_pos_z_m};
    LegJointAngles_t q = {.valid = 1};
    if (wheel_contact_ik_solve(LEG_R_F, &nominal, &invalid, &q) != IK_ERROR_INVALID_PARAM || q.valid ||
        wheel_contact_ik_solve(LEG_R_F, &nominal, &axle_aligned, &q) != IK_ERROR_INVALID_PARAM || q.valid ||
        wheel_contact_ik_solve(LEG_R_F, &unreachable, &level, &q) != IK_ERROR_WORKSPACE || q.valid)
        return false;

    q = leg_stand_mode_angles(LEG_R_F);
    FootPosition_t center;
    if (leg_forward_wheel_contact(LEG_R_F, q, &axle_aligned, &center) != FK_ERROR_INVALID_PARAM ||
        gait_foot_ik(LEG_R_F, &nominal, &q) != IK_SUCCESS ||
        leg_forward_kinematics_matrix(LEG_R_F, q, &center) != FK_SUCCESS)
        return false;
    return fabsf(center.foot_pos_z_m + FOOT_OFFSET_Z) < 1e-6f;
}

static bool check_invalid_targets(void)
{
    FootPosition_t target = {NAN, LEG_R_F_START_POS_Y, LEG_R_F_START_POS_Z};
    LegJointAngles_t q = {.valid = 1};
    if (foot_ik_solve_all(LEG_R_F, &target, &q) != IK_ERROR_INVALID_PARAM || q.valid)
        return false;
    target = (FootPosition_t){HIP_OFFSET_X_R_F + 0.08f,
                              HIP_OFFSET_Y_R_F - 0.5f * HIP_ABD_LENGTH, HIP_OFFSET_Z_R_F};
    if (foot_ik_solve_all(LEG_R_F, &target, &q) != IK_ERROR_WORKSPACE || q.valid)
        return false;
    target.foot_pos_x_m = HIP_OFFSET_X_R_F + FULL_LEN + 0.01f;
    target.foot_pos_y_m = HIP_OFFSET_Y_R_F - HIP_ABD_LENGTH;
    return foot_ik_solve_all(LEG_R_F, &target, &q) == IK_ERROR_WORKSPACE && !q.valid;
}

static bool check_knee_limit(void)
{
    const float limit = JOINT3_CONTROL_MAX;
    const float excessive_bend = 0.5f * (limit + 180.0f);
    const float pitch = DEG_TO_RAD(0.5f * excessive_bend);
    const FootPosition_t too_deep = {
        HIP_OFFSET_X_R_F - LEG_SAGITTAL_X_SIGN * (THIGH_LENGTH - CALF_LENGTH) * sinf(pitch),
        HIP_OFFSET_Y_R_F - HIP_ABD_LENGTH,
        HIP_OFFSET_Z_R_F - FULL_LEN * cosf(pitch)
    };
    LegJointAngles_t q;
    if (foot_ik_solve_all(LEG_R_F, &too_deep, &q) != IK_ERROR_JOINT3_LIMIT || q.valid)
        return false;

    const LegJointAngles_t right = leg_limit_joint_angles(LEG_R_F, (LegJointAngles_t){.joint3_deg = -180.0f});
    const LegJointAngles_t left = leg_limit_joint_angles(LEG_L_F, (LegJointAngles_t){.joint3_deg = 180.0f});
    return right.joint3_deg == -limit && left.joint3_deg == limit;
}

static bool check_swing_height_and_landing(void)
{
    const BezierTrajConfig_t configs[] = {
        {TROT_CYCLE_TIME * TROT_SWING_RATIO, TROT_STEP_HEIGHT},
        {WALK_CYCLE_TIME * WALK_SWING_RATIO, WALK_STEP_HEIGHT},
        {PRONK_CYCLE_TIME * PRONK_SWING_RATIO, PRONK_STEP_HEIGHT}
    };
    const FootPosition_t foot = {LEG_R_F_START_POS_X, LEG_R_F_START_POS_Y, LEG_R_F_START_POS_Z};
    for (unsigned i = 0; i < sizeof configs / sizeof configs[0]; ++i)
    {
        LegTrajContext_t ctx;
        LegJointAngles_t q;
        float v[3], a[3];
        const BezierTrajConfig_t cfg = configs[i];
        if (bezier_traj_controller_init(&ctx, LEG_R_F, cfg) != 0)
            return false;
        generate_leg_bezier_traj(&ctx, foot, foot);
        if (get_leg_joint_cmd(&ctx, 0.5f * cfg.cycle_time, &q, v, a) != TRAJ_STATUS_OK ||
            fabsf(ctx.runtime.last_foot_pos.foot_pos_z_m - foot.foot_pos_z_m - cfg.lift_height_m) > 1e-5f ||
            v[0] != 0.0f || v[1] != 0.0f || v[2] != 0.0f)
            return false;

        const float endpoint_step_s = 0.0005f;
        if (get_leg_joint_cmd(&ctx, cfg.cycle_time - endpoint_step_s, &q, v, a) != TRAJ_STATUS_OK ||
            fabsf(ctx.runtime.last_foot_pos.foot_pos_z_m - foot.foot_pos_z_m) / endpoint_step_s > 0.001f ||
            get_leg_joint_cmd(&ctx, cfg.cycle_time, &q, v, a) != TRAJ_STATUS_END ||
            ctx.runtime.last_foot_pos.foot_pos_z_m != foot.foot_pos_z_m)
            return false;
    }
    return true;
}

static bool check_swing_cache(void)
{
    const BezierTrajConfig_t config = {.cycle_time = 0.27f, .lift_height_m = TROT_STEP_HEIGHT};
    const FootPosition_t foot = {LEG_R_F_START_POS_X, LEG_R_F_START_POS_Y, LEG_R_F_START_POS_Z};
    FootPosition_t unreachable = foot;
    unreachable.foot_pos_x_m += 2.0f * FULL_LEN;
    LegTrajContext_t ctx;
    LegJointAngles_t q;
    float v[3], a[3];
    if (bezier_traj_controller_init(&ctx, LEG_R_F, config) != 0)
        return false;
    generate_leg_bezier_traj(&ctx, unreachable, unreachable);
    for (int i = 0; i < 2; ++i)
        if (get_leg_joint_cmd(&ctx, i * 0.005f, &q, v, a) != TRAJ_STATUS_ERROR_IK || q.valid)
            return false;
    generate_leg_bezier_traj(&ctx, foot, unreachable);
    if (get_leg_joint_cmd(&ctx, 0.005f, &q, v, a) != TRAJ_STATUS_OK)
        return false;
    LegJointAngles_t held = q;
    return get_leg_joint_cmd(&ctx, config.cycle_time, &q, v, a) == TRAJ_STATUS_ERROR_IK &&
           q.valid && q.joint1_deg == held.joint1_deg && q.joint2_deg == held.joint2_deg &&
           q.joint3_deg == held.joint3_deg && v[2] == 0.0f;
}

static bool check_swing_hold(void)
{
    LegStateMachine_t leg;
    const BezierTrajConfig_t config = {.cycle_time = 0.25f, .lift_height_m = 0.04f};
    if (leg_state_machine_init(&leg, LEG_R_F, config) != 0)
        return false;

    const LegJointAngles_t previous = leg.current_joint_angles;
    FootPosition_t reachable;
    if (gait_foot_fk(LEG_R_F, previous, &reachable) != FK_SUCCESS)
        return false;
    leg.current_target_pos = reachable;
    leg.current_target_pos.foot_pos_x_m += 2.0f * FULL_LEN;
    if (leg_state_switch(&leg, LEG_STATE_SWING, config.cycle_time, &reachable, 0.0f) != 0)
        return false;

    for (int tick = 0; tick < 2; ++tick)
    {
        leg.phase = 0.02f * tick;
        if (leg_state_machine_update(&leg, leg.phase * config.cycle_time) != 0 ||
            leg.last_traj_status != TRAJ_STATUS_ERROR_IK || !leg.current_joint_angles.valid ||
            leg.current_joint_angles.joint1_deg != previous.joint1_deg ||
            leg.current_joint_angles.joint2_deg != previous.joint2_deg ||
            leg.current_joint_angles.joint3_deg != previous.joint3_deg)
            return false;
    }
    leg.phase = 1.0f;
    return leg_state_machine_update(&leg, config.cycle_time) == 0 &&
           leg.last_traj_status == TRAJ_STATUS_END && leg.current_joint_angles.valid;
}

bool webots_run_ik_checks(void)
{
    robot_state_init();
    const struct {const char *name; bool (*run)(void);} checks[] = {
        {"hip projection", check_projection},
        {"500 wheel-center FK/IK round trips", check_round_trip},
        {"wheel contact projections", check_contact_projection},
        {"1500 wheel-contact FK/IK round trips", check_contact_round_trip},
        {"wheel contact limits and gait height", check_contact_limits},
        {"invalid targets", check_invalid_targets},
        {"knee limit", check_knee_limit},
        {"swing heights and landing speed", check_swing_height_and_landing},
        {"swing cache validity", check_swing_cache},
        {"swing hold and recovery", check_swing_hold}
    };
    bool passed = true;
    for (unsigned i = 0; i < sizeof checks / sizeof checks[0]; ++i)
    {
        bool ok = checks[i].run();
        printf("[HAL Webots] %s: %s\n", checks[i].name, ok ? "PASS" : "FAIL");
        passed = passed && ok;
    }
    return passed;
}

#include <string.h>
#include <webots/keyboard.h>
#include <webots/robot.h>

#include "robot_sbus.h"
#include "remote_control_sbus.h"
#include "virtual_sbus.h"

enum
{
    KEY_ESCAPE = 27,
    KEY_COUNT = WB_KEYBOARD_DOWN + 1,
    KNOB_STEP = 110
};

static bool previous[KEY_COUNT];
static bool wheel_trace_requested;
static WebotsScenario scenario;
static WebotsDriveInput drive_input;
static uint16_t arm_switch = SBUS_RAW_MIN;
static uint16_t gait_switch = SBUS_RAW_MID;
static bool climb_selected;
static uint16_t mode_switch = SBUS_RAW_MID;
static int speed_knob = SBUS_RAW_MIN +
    (int)((1.0f - RC_SPEED_SCALE_MIN) / (RC_SPEED_SCALE_MAX - RC_SPEED_SCALE_MIN) *
          (SBUS_RAW_MAX - SBUS_RAW_MIN) + 0.5f);

void virtual_sbus_init(WebotsScenario selected)
{
    scenario = selected;
    if (scenario == WEBOTS_MANUAL)
        wb_keyboard_enable(TIME_STEP);
}

void virtual_sbus_set_drive_input(WebotsDriveInput input)
{
    drive_input = input;
}

static void read_keys(bool keys[KEY_COUNT])
{
    if (scenario == WEBOTS_LATERAL_LEFT || scenario == WEBOTS_LATERAL_RIGHT)
    {
        const double now = wb_robot_get_time();
        keys['I'] = now >= 0.60 && now < 0.70;
        keys['O'] = now >= WEBOTS_LATERAL_START_S && now < 1.40;
        keys[scenario == WEBOTS_LATERAL_LEFT ? 'A' : 'D'] = now >= WEBOTS_LATERAL_START_S;
        return;
    }

    if (scenario == WEBOTS_JUMP)
    {
        const double now = wb_robot_get_time();
        keys['I'] = now >= 0.60 && now < 0.70;
        keys['O'] = now >= 1.30 && now < 1.40;
        keys[' '] = now >= WEBOTS_JUMP_START_S && now < WEBOTS_JUMP_START_S + 0.10;
        return;
    }

    if (scenario == WEBOTS_DRIVE)
    {
        const double now = wb_robot_get_time();
        const bool climb = drive_input.gait_key == '3';
        keys['I'] = !climb && now >= 0.60 && now < 0.70;
        keys['O'] = !climb && !drive_input.wheeled && now >= 1.30 && now < 1.40;
        keys['2'] = drive_input.wheeled && now >= 1.30 && now < 1.40;
        if (drive_input.gait_key)
            keys[drive_input.gait_key] = climb ? now >= 0.60 && now < 0.70 : now >= 1.40;
        keys[drive_input.motion_key] = now >= WEBOTS_DRIVE_START_S && now < WEBOTS_DRIVE_END_S;
        return;
    }

    if (scenario == WEBOTS_SMOKE)
    {
        const double now = wb_robot_get_time();
        keys['I'] = (now >= 0.60 && now < 0.70) || (now >= 4.10 && now < 4.20) ||
                    (now >= 9.00 && now < 9.10);
        keys['O'] = now >= 1.30 && now < 1.40;
        keys[' '] = now >= 1.94 && now < 2.04;
        keys['2'] = now >= 4.40 && now < 4.50;
        keys['1'] = now >= 6.05 && now < 6.15;
        keys['W'] = (now >= 1.40 && now < 1.90) || (now >= 4.90 && now < 6.00);
        keys['Q'] = now >= 5.20 && now < 6.00;
        keys['H'] = now >= 5.85 && now < 5.95;
        keys['F'] = (now >= 6.80 && now < 6.90) || (now >= 8.15 && now < 8.25);
        keys[KEY_ESCAPE] = now >= 10.05;
        return;
    }

    int key;
    while ((key = wb_keyboard_get_key()) != -1)
    {
        key &= WB_KEYBOARD_KEY;
        if (key < KEY_COUNT)
            keys[key] = true;
    }
}

static uint16_t stick(bool positive, bool negative)
{
    if (positive == negative)
        return SBUS_RAW_MID;
    return positive ? SBUS_RAW_MAX : SBUS_RAW_MIN;
}

bool virtual_sbus_take_wheel_trace_request(void)
{
    const bool requested = wheel_trace_requested;
    wheel_trace_requested = false;
    return requested;
}

bool RobotSbus_Start(UART_HandleTypeDef *uart)
{
    (void)uart;
    return true;
}

bool RobotSbus_GetLiveChannels(uint16_t channels[16])
{
    bool keys[KEY_COUNT] = {false};
    read_keys(keys);

    if (keys['I'])
        arm_switch = SBUS_RAW_MID;
    if (keys['O'])
        arm_switch = SBUS_RAW_MAX;
    if (keys[KEY_ESCAPE])
    {
        arm_switch = SBUS_RAW_MIN;
        mode_switch = SBUS_RAW_MID;
    }
    else if (arm_switch != SBUS_RAW_MIN)
    {
        if (keys['1'] && mode_switch == SBUS_RAW_MAX)
        {
            mode_switch = SBUS_RAW_MID;
            arm_switch = SBUS_RAW_MAX;
        }
        if (keys['2'])
            mode_switch = SBUS_RAW_MAX;
    }
    if (keys['3'] && !keys[KEY_ESCAPE])
    {
        climb_selected = true;
        arm_switch = SBUS_RAW_MAX;
        mode_switch = SBUS_RAW_MID;
    }
    else if (keys['4'] || keys['5'] || keys['6'])
    {
        climb_selected = false;
        if (keys['4'])
            gait_switch = SBUS_RAW_MIN;
        else if (keys['5'])
            gait_switch = SBUS_RAW_MID;
        else
            gait_switch = SBUS_RAW_MAX;
    }

    if (keys[WB_KEYBOARD_UP] && !previous[WB_KEYBOARD_UP])
        speed_knob += KNOB_STEP;
    if (keys[WB_KEYBOARD_DOWN] && !previous[WB_KEYBOARD_DOWN])
        speed_knob -= KNOB_STEP;
    if (speed_knob > SBUS_RAW_MAX)
        speed_knob = SBUS_RAW_MAX;
    if (speed_knob < SBUS_RAW_MIN)
        speed_knob = SBUS_RAW_MIN;

    const bool wheeled = g_robot_state.mode == ROBOT_MODE_WHEELED;
    if (wheeled && keys['T'] && !previous['T'])
        wheel_trace_requested = true;
    const bool moving = !keys['R'];
    for (int i = 0; i < 16; ++i)
        channels[i] = SBUS_RAW_MID;

    const bool turn_left = moving && (keys['Q'] || (wheeled && keys['A']));
    const bool turn_right = moving && (keys['E'] || (wheeled && keys['D']));
    channels[0] = stick(turn_left, turn_right);
    channels[1] = stick(moving && keys['W'], moving && keys['S']);
    channels[2] = stick(moving && !wheeled && keys['A'], moving && !wheeled && keys['D']);
    /* Keep either stick away from the combined jump gesture unless Space is pressed. */
    if (keys[' '] && !previous[' '] && !wheeled)
        channels[1] = channels[2] = SBUS_RAW_MIN;
    else if (channels[1] == SBUS_RAW_MIN && channels[2] == SBUS_RAW_MIN)
        channels[2] = SBUS_RAW_MIN + 121;

    channels[4] = arm_switch;
    channels[5] = gait_switch;
    channels[6] = mode_switch;
    channels[7] = keys['H'] ? SBUS_RAW_MIN : keys['F'] ? SBUS_RAW_MAX : SBUS_RAW_MID;
    channels[8] = (uint16_t)speed_knob;
    channels[CLIMB_SBUS_CHANNEL] = climb_selected ? SBUS_RAW_MAX : SBUS_RAW_MIN;
    memcpy(previous, keys, sizeof previous);
    return true;
}

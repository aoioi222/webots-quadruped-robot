#ifndef WEBOTS_VIRTUAL_SBUS_H
#define WEBOTS_VIRTUAL_SBUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WEBOTS_MANUAL,
    WEBOTS_SMOKE,
    WEBOTS_LATERAL_LEFT,
    WEBOTS_LATERAL_RIGHT,
    WEBOTS_JUMP,
    WEBOTS_DRIVE
} WebotsScenario;

#define WEBOTS_LATERAL_START_S 1.30
#define WEBOTS_JUMP_START_S 2.10
#define WEBOTS_DRIVE_START_S 2.30
#define WEBOTS_DRIVE_END_S 5.30

typedef struct
{
    int gait_key;
    int motion_key;
    bool wheeled;
} WebotsDriveInput;

void virtual_sbus_init(WebotsScenario scenario);
void virtual_sbus_set_drive_input(WebotsDriveInput input);
bool virtual_sbus_read(uint16_t channels[16]);
bool virtual_sbus_take_wheel_trace_request(void);

#endif

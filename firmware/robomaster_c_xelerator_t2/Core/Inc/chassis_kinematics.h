#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_WHEEL_COUNT 4U

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3
} wheel_index_t;

typedef struct {
    float x_m;
    float y_m;
    float drive_angle_rad;
    float wheel_radius_m;
    float gear_ratio;
    float motor_sign;
} wheel_geometry_t;

typedef struct {
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
} chassis_twist_t;

typedef struct {
    float raw_wheel_rad_s[CHASSIS_WHEEL_COUNT];
    float raw_motor_rad_s[CHASSIS_WHEEL_COUNT];
    float wheel_rad_s[CHASSIS_WHEEL_COUNT];
    float motor_rad_s[CHASSIS_WHEEL_COUNT];
    float scale;
} wheel_targets_t;

typedef struct {
    wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT];
    float jacobian[CHASSIS_WHEEL_COUNT][3];
    float motor_matrix[CHASSIS_WHEEL_COUNT][3];
    float pseudoinverse[3][CHASSIS_WHEEL_COUNT];
    float singular_values[3];
    float condition_number;
    uint8_t rank;
    bool valid;
} chassis_kinematics_t;

bool chassis_kinematics_init(
    chassis_kinematics_t *kinematics,
    const wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT],
    float max_condition_number);

bool chassis_inverse_kinematics(
    const chassis_kinematics_t *kinematics,
    const chassis_twist_t *twist,
    const float max_motor_rad_s[CHASSIS_WHEEL_COUNT],
    wheel_targets_t *targets);

bool chassis_forward_kinematics(
    const chassis_kinematics_t *kinematics,
    const float motor_rad_s[CHASSIS_WHEEL_COUNT],
    chassis_twist_t *twist);

bool chassis_limit_twist(
    const chassis_twist_t *input,
    const chassis_twist_t *absolute_limits,
    chassis_twist_t *output);

#endif

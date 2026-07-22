#include "chassis_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool finite_float(float value) {
    return isfinite(value) != 0;
}

static bool inverse3(const float matrix[3][3], float inverse[3][3]) {
    const float a = matrix[0][0], b = matrix[0][1], c = matrix[0][2];
    const float d = matrix[1][0], e = matrix[1][1], f = matrix[1][2];
    const float g = matrix[2][0], h = matrix[2][1], i = matrix[2][2];
    const float cof00 = e * i - f * h, cof01 = c * h - b * i, cof02 = b * f - c * e;
    const float cof10 = f * g - d * i, cof11 = a * i - c * g, cof12 = c * d - a * f;
    const float cof20 = d * h - e * g, cof21 = b * g - a * h, cof22 = a * e - b * d;
    const float det = a * cof00 + b * cof10 + c * cof20;
    float scale = 0.0f;
    for (uint8_t row = 0; row < 3U; ++row) {
        for (uint8_t col = 0; col < 3U; ++col) {
            const float magnitude = fabsf(matrix[row][col]);
            if (magnitude > scale) scale = magnitude;
        }
    }
    if (scale == 0.0f || fabsf(det) <= scale * scale * scale * 1.0e-6f) return false;
    const float inv_det = 1.0f / det;
    inverse[0][0] = cof00 * inv_det; inverse[0][1] = cof01 * inv_det; inverse[0][2] = cof02 * inv_det;
    inverse[1][0] = cof10 * inv_det; inverse[1][1] = cof11 * inv_det; inverse[1][2] = cof12 * inv_det;
    inverse[2][0] = cof20 * inv_det; inverse[2][1] = cof21 * inv_det; inverse[2][2] = cof22 * inv_det;
    return true;
}

static void symmetric_eigenvalues3(const float input[3][3], float values[3]) {
    float matrix[3][3];
    memcpy(matrix, input, sizeof(matrix));
    for (uint8_t iteration = 0; iteration < 32U; ++iteration) {
        uint8_t p = 0U, q = 1U;
        if (fabsf(matrix[0][2]) > fabsf(matrix[p][q])) { p = 0U; q = 2U; }
        if (fabsf(matrix[1][2]) > fabsf(matrix[p][q])) { p = 1U; q = 2U; }
        float diagonal_scale = fmaxf(1.0f, fmaxf(fabsf(matrix[0][0]), fmaxf(fabsf(matrix[1][1]), fabsf(matrix[2][2]))));
        if (fabsf(matrix[p][q]) <= 1.0e-6f * diagonal_scale) break;
        const float tau = (matrix[q][q] - matrix[p][p]) / (2.0f * matrix[p][q]);
        const float t = tau == 0.0f ? 1.0f : copysignf(1.0f, tau) / (fabsf(tau) + sqrtf(1.0f + tau * tau));
        const float cosine = 1.0f / sqrtf(1.0f + t * t);
        const float sine = t * cosine;
        const float apq = matrix[p][q];
        matrix[p][p] -= t * apq;
        matrix[q][q] += t * apq;
        matrix[p][q] = matrix[q][p] = 0.0f;
        for (uint8_t r = 0; r < 3U; ++r) {
            if (r == p || r == q) continue;
            const float arp = matrix[r][p], arq = matrix[r][q];
            matrix[r][p] = matrix[p][r] = cosine * arp - sine * arq;
            matrix[r][q] = matrix[q][r] = sine * arp + cosine * arq;
        }
    }
    values[0] = matrix[0][0]; values[1] = matrix[1][1]; values[2] = matrix[2][2];
    for (uint8_t i = 0; i < 3U; ++i) {
        for (uint8_t j = i + 1U; j < 3U; ++j) {
            if (values[j] > values[i]) {
                const float temporary = values[i]; values[i] = values[j]; values[j] = temporary;
            }
        }
    }
}

bool chassis_kinematics_init(
    chassis_kinematics_t *kinematics,
    const wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT],
    float max_condition_number) {
    if (kinematics == NULL || wheels == NULL || !finite_float(max_condition_number) || max_condition_number <= 1.0f) return false;
    memset(kinematics, 0, sizeof(*kinematics));
    float ata[3][3] = {{0.0f}}, inverse[3][3];
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        const wheel_geometry_t *geometry = &wheels[wheel];
        if (!finite_float(geometry->x_m) || !finite_float(geometry->y_m) ||
            !finite_float(geometry->drive_angle_rad) || !finite_float(geometry->wheel_radius_m) ||
            !finite_float(geometry->gear_ratio) || !finite_float(geometry->motor_sign) ||
            geometry->wheel_radius_m <= 0.0f || geometry->gear_ratio <= 0.0f ||
            (geometry->motor_sign != -1.0f && geometry->motor_sign != 1.0f)) return false;
        kinematics->wheels[wheel] = *geometry;
        const float ux = cosf(geometry->drive_angle_rad), uy = sinf(geometry->drive_angle_rad);
        kinematics->jacobian[wheel][0] = ux;
        kinematics->jacobian[wheel][1] = uy;
        kinematics->jacobian[wheel][2] = -geometry->y_m * ux + geometry->x_m * uy;
        const float motor_scale = geometry->motor_sign * geometry->gear_ratio / geometry->wheel_radius_m;
        for (uint8_t col = 0; col < 3U; ++col) {
            kinematics->motor_matrix[wheel][col] = kinematics->jacobian[wheel][col] * motor_scale;
        }
    }
    for (uint8_t row = 0; row < 3U; ++row) {
        for (uint8_t col = 0; col < 3U; ++col) {
            for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
                ata[row][col] += kinematics->motor_matrix[wheel][row] * kinematics->motor_matrix[wheel][col];
            }
        }
    }
    float eigenvalues[3];
    symmetric_eigenvalues3(ata, eigenvalues);
    for (uint8_t i = 0; i < 3U; ++i) kinematics->singular_values[i] = sqrtf(fmaxf(0.0f, eigenvalues[i]));
    const float tolerance = kinematics->singular_values[0] * 1.0e-6f;
    for (uint8_t i = 0; i < 3U; ++i) if (kinematics->singular_values[i] > tolerance) ++kinematics->rank;
    if (kinematics->rank != 3U || kinematics->singular_values[2] <= 0.0f || !inverse3(ata, inverse)) return false;
    kinematics->condition_number = kinematics->singular_values[0] / kinematics->singular_values[2];
    if (!finite_float(kinematics->condition_number) || kinematics->condition_number > max_condition_number) return false;
    for (uint8_t row = 0; row < 3U; ++row) {
        for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
            for (uint8_t col = 0; col < 3U; ++col) {
                kinematics->pseudoinverse[row][wheel] += inverse[row][col] * kinematics->motor_matrix[wheel][col];
            }
        }
    }
    kinematics->valid = true;
    return true;
}

bool chassis_inverse_kinematics(
    const chassis_kinematics_t *kinematics,
    const chassis_twist_t *twist,
    const float max_motor_rad_s[CHASSIS_WHEEL_COUNT],
    wheel_targets_t *targets) {
    if (kinematics == NULL || twist == NULL || targets == NULL || !kinematics->valid ||
        !finite_float(twist->vx_mps) || !finite_float(twist->vy_mps) || !finite_float(twist->wz_rad_s)) return false;
    memset(targets, 0, sizeof(*targets));
    targets->scale = 1.0f;
    const float vector[3] = {twist->vx_mps, twist->vy_mps, twist->wz_rad_s};
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        float projected = 0.0f;
        for (uint8_t col = 0; col < 3U; ++col) projected += kinematics->jacobian[wheel][col] * vector[col];
        targets->raw_wheel_rad_s[wheel] = projected / kinematics->wheels[wheel].wheel_radius_m;
        targets->raw_motor_rad_s[wheel] = targets->raw_wheel_rad_s[wheel] *
            kinematics->wheels[wheel].motor_sign * kinematics->wheels[wheel].gear_ratio;
        if (max_motor_rad_s != NULL) {
            if (!finite_float(max_motor_rad_s[wheel]) || max_motor_rad_s[wheel] <= 0.0f) return false;
            const float magnitude = fabsf(targets->raw_motor_rad_s[wheel]);
            if (magnitude > max_motor_rad_s[wheel]) {
                const float candidate = max_motor_rad_s[wheel] / magnitude;
                if (candidate < targets->scale) targets->scale = candidate;
            }
        }
    }
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        targets->wheel_rad_s[wheel] = targets->raw_wheel_rad_s[wheel] * targets->scale;
        targets->motor_rad_s[wheel] = targets->raw_motor_rad_s[wheel] * targets->scale;
    }
    return true;
}

bool chassis_forward_kinematics(
    const chassis_kinematics_t *kinematics,
    const float motor_rad_s[CHASSIS_WHEEL_COUNT],
    chassis_twist_t *twist) {
    if (kinematics == NULL || motor_rad_s == NULL || twist == NULL || !kinematics->valid) return false;
    float values[3] = {0.0f};
    for (uint8_t wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        if (!finite_float(motor_rad_s[wheel])) return false;
        for (uint8_t row = 0; row < 3U; ++row) values[row] += kinematics->pseudoinverse[row][wheel] * motor_rad_s[wheel];
    }
    twist->vx_mps = values[0]; twist->vy_mps = values[1]; twist->wz_rad_s = values[2];
    return true;
}

bool chassis_limit_twist(
    const chassis_twist_t *input,
    const chassis_twist_t *absolute_limits,
    chassis_twist_t *output) {
    if (input == NULL || absolute_limits == NULL || output == NULL) return false;
    const float values[3] = {input->vx_mps, input->vy_mps, input->wz_rad_s};
    const float limits[3] = {absolute_limits->vx_mps, absolute_limits->vy_mps, absolute_limits->wz_rad_s};
    float limited[3];
    for (uint8_t i = 0; i < 3U; ++i) {
        if (!finite_float(values[i]) || !finite_float(limits[i]) || limits[i] <= 0.0f) return false;
        limited[i] = fmaxf(-limits[i], fminf(limits[i], values[i]));
    }
    output->vx_mps = limited[0]; output->vy_mps = limited[1]; output->wz_rad_s = limited[2];
    return true;
}

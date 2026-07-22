#include "cboard_imu.h"

#include "bmi088_driver.h"
#include "adc.h"
#include "cboard_diag.h"
#include "cboard_imu_math.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#ifdef CBOARD_IMU_TEST
#include <pthread.h>
#endif

#define IMU_SAMPLE_PERIOD_US 1000U
#define IMU_EFFECTIVE_PERIOD_US 1250U
#define IMU_STALE_LIMIT_US 20000U
#define IMU_CALIBRATION_TIMEOUT_US 15000000U
#define IMU_WARMUP_TIMEOUT_US 60000000U
#define IMU_HEATER_TARGET_MAX_C 45.0f
#define IMU_HEATER_OVERTEMP_C 50.0f
#define IMU_HEATER_PWM_MAX 4999.0f
#define IMU_HEATER_PID_MAX 4500.0f
#define IMU_HEATER_PID_I_MAX 4400.0f

static bmi088_t bmi088;
static cboard_imu_calibrator_t calibrator;
static cboard_imu_attitude_t attitude;
static cboard_imu_state_t working;
static cboard_imu_state_t snapshot;
static atomic_uint_least32_t snapshot_lock;
static uint32_t last_poll_us;
static uint32_t last_valid_sample_us;
static uint32_t calibration_started_us;
static float yaw_reference_rad;
static bool yaw_reference_valid;
static uint32_t warmup_started_us;
static uint16_t heater_above_target_count;
static float heater_target_c;
static float heater_integral;
static bool heater_target_valid;
static bool heater_stable;
static bool heater_overtemp_latched;

#ifdef CBOARD_IMU_TEST
static pthread_mutex_t snapshot_critical = PTHREAD_MUTEX_INITIALIZER;
typedef int snapshot_irq_state_t;
static snapshot_irq_state_t snapshot_enter_critical(void) {
    return pthread_mutex_lock(&snapshot_critical);
}
static void snapshot_exit_critical(snapshot_irq_state_t state) {
    (void)state;
    (void)pthread_mutex_unlock(&snapshot_critical);
}
#else
typedef uint32_t snapshot_irq_state_t;
static snapshot_irq_state_t snapshot_enter_critical(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}
static void snapshot_exit_critical(snapshot_irq_state_t primask) {
    __DMB();
    if ((primask & 1U) == 0U) __enable_irq();
}
#endif

/* Identity is intentionally marked unverified until the physical board is
 * rotated in the robot mounting convention. */
static const imu_mount_transform_t mount_transform = {
    {0, 1, 2}, {1, 1, 1}, 0.0f, false
};

static void publish(void) {
    snapshot_irq_state_t irq_state = snapshot_enter_critical();
    (void)atomic_fetch_add_explicit(&snapshot_lock, 1U, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    snapshot = working;
    atomic_thread_fence(memory_order_release);
    (void)atomic_fetch_add_explicit(&snapshot_lock, 1U, memory_order_release);
    snapshot_exit_critical(irq_state);
}

static bool select_sensor(bmi088_device_t device, bool active) {
    GPIO_TypeDef *port = device == BMI088_DEVICE_ACCEL ? CS1_ACCEL_GPIO_Port : CS1_GYRO_GPIO_Port;
    uint16_t pin = device == BMI088_DEVICE_ACCEL ? CS1_ACCEL_Pin : CS1_GYRO_Pin;
    HAL_GPIO_WritePin(port, pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return true;
}

static bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t length) {
    if (tx == NULL || rx == NULL || length == 0U || length > UINT16_MAX) return false;
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, (uint16_t)length, 2U) == HAL_OK;
}

static void delay_ms(uint32_t delay) {
    HAL_Delay(delay);
}

static void record_init_failure(bmi088_result_t result) {
    working.error_count++;
    working.state = IMU_STATE_FAULT;
    working.flags |= IMU_FLAG_ATTITUDE_NOT_VALID | IMU_FLAG_CALIBRATION_NOT_VALID;
    if (result == BMI088_RESULT_ACCEL_NOT_FOUND || result == BMI088_RESULT_ACCEL_CONFIG_ERROR) {
        working.flags |= IMU_FLAG_ACCEL_INIT_FAILED;
    } else if (result == BMI088_RESULT_GYRO_NOT_FOUND || result == BMI088_RESULT_GYRO_CONFIG_ERROR) {
        working.flags |= IMU_FLAG_GYRO_INIT_FAILED;
    } else if (result == BMI088_RESULT_BUS_ERROR) {
        working.flags |= IMU_FLAG_SPI_ERROR;
    } else {
        working.flags |= IMU_FLAG_ACCEL_INIT_FAILED | IMU_FLAG_GYRO_INIT_FAILED;
    }
}

void cboard_imu_init(void) {
    memset(&working, 0, sizeof(working));
    memset(&snapshot, 0, sizeof(snapshot));
    atomic_init(&snapshot_lock, 0U);
    heater_target_valid = false;
    heater_target_c = 0.0f;
    working.state = IMU_STATE_INITIALIZING;
    working.calibration_state = IMU_CAL_NOT_STARTED;
    working.source = IMU_SOURCE_GYRO_RELATIVE;
    working.flags = IMU_FLAG_CALIBRATION_NOT_VALID | IMU_FLAG_ATTITUDE_NOT_VALID |
                    IMU_FLAG_TEMPERATURE_NOT_STABLE;
    if (!mount_transform.hardware_verified) working.flags |= IMU_FLAG_AXIS_MAPPING_UNVERIFIED;
    TIM10_IMU_Heater_Set(0U);
    if (!cboard_imu_transform_valid(&mount_transform)) {
        working.flags |= IMU_FLAG_AXIS_MAPPING_INVALID;
        working.state = IMU_STATE_FAULT;
        publish();
        return;
    }
    if (!SPI1_IsReady()) {
        working.flags |= IMU_FLAG_SPI_ERROR | IMU_FLAG_ACCEL_INIT_FAILED |
                         IMU_FLAG_GYRO_INIT_FAILED;
        working.state = IMU_STATE_FAULT;
        publish();
        return;
    }
    publish();

    const bmi088_bus_t bus = {select_sensor, spi_transfer, delay_ms};
    bmi088_result_t result = bmi088_init(&bmi088, &bus);
    working.accel_chip_id = bmi088.accel_chip_id;
    working.gyro_chip_id = bmi088.gyro_chip_id;
    working.spi_error_count = bmi088.transfer_error_count;
    if (result != BMI088_RESULT_OK || !SPI1_SetFast()) {
        record_init_failure(result == BMI088_RESULT_OK ? BMI088_RESULT_BUS_ERROR : result);
        publish();
        return;
    }

    float mcu_temperature_c = 0.0f;
    if (TIM10_IMU_Heater_IsReady() && ADC1_ReadMCUTemperature(&mcu_temperature_c)) {
        heater_target_c = mcu_temperature_c + 10.0f;
        if (heater_target_c > IMU_HEATER_TARGET_MAX_C) heater_target_c = IMU_HEATER_TARGET_MAX_C;
        heater_target_valid = true;
    }

    cboard_imu_calibrator_reset(&calibrator);
    cboard_imu_attitude_reset(&attitude);
    working.state = heater_target_valid ? IMU_STATE_CALIBRATING : IMU_STATE_DEGRADED;
    working.calibration_state = IMU_CAL_NOT_STARTED;
    working.sample_interval_min_us = UINT32_MAX;
    calibration_started_us = 0U;
    warmup_started_us = cboard_diag_time_us();
    last_poll_us = 0U;
    last_valid_sample_us = 0U;
    yaw_reference_rad = 0.0f;
    yaw_reference_valid = false;
    heater_above_target_count = 0U;
    if (!heater_target_valid) {
        heater_target_c = 0.0f;
        working.flags |= IMU_FLAG_HEATER_UNAVAILABLE;
    }
    heater_integral = 0.0f;
    heater_stable = false;
    heater_overtemp_latched = false;
    publish();
}

static void heater_update(float temperature_c, uint32_t now_us) {
    if (!heater_target_valid) {
        TIM10_IMU_Heater_Set(0U);
        working.flags |= IMU_FLAG_TEMPERATURE_NOT_STABLE;
        return;
    }
    if (temperature_c >= IMU_HEATER_OVERTEMP_C) heater_overtemp_latched = true;
    if (heater_overtemp_latched) {
        TIM10_IMU_Heater_Set(0U);
        working.flags |= IMU_FLAG_TEMPERATURE_OVERTEMP | IMU_FLAG_TEMPERATURE_NOT_STABLE;
        heater_stable = false;
        working.state = IMU_STATE_DEGRADED;
        return;
    }
    working.flags &= ~IMU_FLAG_TEMPERATURE_OVERTEMP;
    if (!heater_stable) {
        TIM10_IMU_Heater_Set((uint16_t)IMU_HEATER_PWM_MAX);
        if (temperature_c > heater_target_c) {
            if (heater_above_target_count < UINT16_MAX) heater_above_target_count++;
            if (heater_above_target_count > 200U) {
                heater_stable = true;
                heater_integral = IMU_HEATER_PWM_MAX * 0.5f;
                working.flags &= ~IMU_FLAG_TEMPERATURE_NOT_STABLE;
                cboard_imu_calibrator_reset(&calibrator);
                working.calibration_state = IMU_CAL_COLLECTING;
                calibration_started_us = now_us;
            }
        } else {
            heater_above_target_count = 0U;
        }
        if ((uint32_t)(now_us - warmup_started_us) > IMU_WARMUP_TIMEOUT_US) {
            TIM10_IMU_Heater_Set(0U);
            working.flags |= IMU_FLAG_TEMPERATURE_NOT_STABLE;
            working.state = IMU_STATE_DEGRADED;
        }
        return;
    }
    float error = heater_target_c - temperature_c;
    heater_integral += 0.2f * error;
    if (heater_integral > IMU_HEATER_PID_I_MAX) heater_integral = IMU_HEATER_PID_I_MAX;
    if (heater_integral < 0.0f) heater_integral = 0.0f;
    float output = 1600.0f * error + heater_integral;
    if (output > IMU_HEATER_PID_MAX) output = IMU_HEATER_PID_MAX;
    if (output < 0.0f) output = 0.0f;
    TIM10_IMU_Heater_Set((uint16_t)output);
}

static void sample_failed(bmi088_result_t result) {
    working.error_count++;
    working.spi_error_count = bmi088.transfer_error_count;
    if (result == BMI088_RESULT_BUS_ERROR) working.flags |= IMU_FLAG_SPI_ERROR;
    if (result == BMI088_RESULT_SENSOR_ERROR) working.flags |= IMU_FLAG_ACCEL_SENSOR_ERROR;
    if (result == BMI088_RESULT_NONFINITE) working.flags |= IMU_FLAG_NONFINITE_SAMPLE;
    if (working.error_count >= 5U) working.state = IMU_STATE_DEGRADED;
}

void cboard_imu_update(void) {
    uint32_t started_us = cboard_diag_time_us();
    if (working.state == IMU_STATE_UNINITIALIZED || working.state == IMU_STATE_INITIALIZING ||
        working.state == IMU_STATE_FAULT) return;
    uint32_t poll_elapsed = last_poll_us == 0U ? IMU_SAMPLE_PERIOD_US : started_us - last_poll_us;
    if (last_poll_us != 0U && poll_elapsed < IMU_SAMPLE_PERIOD_US) return;
    last_poll_us = started_us;

    bmi088_sample_t sample;
    bmi088_result_t result = bmi088_read_sample(&bmi088, &sample);
    if (result == BMI088_RESULT_NOT_READY) return;
    if (result != BMI088_RESULT_OK) {
        sample_failed(result);
        publish();
        return;
    }
    uint32_t sample_elapsed = last_valid_sample_us == 0U
        ? IMU_EFFECTIVE_PERIOD_US : started_us - last_valid_sample_us;
    last_valid_sample_us = started_us;
    /* The counter preserves history; the flag describes the current sample
     * interval and clears again after timing has recovered. */
    working.flags &= ~IMU_FLAG_SAMPLE_OVERRUN;
    if (working.sample_count != 0U) {
        if (sample_elapsed < working.sample_interval_min_us) working.sample_interval_min_us = sample_elapsed;
        if (sample_elapsed > working.sample_interval_max_us) working.sample_interval_max_us = sample_elapsed;
        if (sample_elapsed >= IMU_EFFECTIVE_PERIOD_US * 2U) {
            working.missed_sample_count += sample_elapsed / IMU_EFFECTIVE_PERIOD_US - 1U;
            working.flags |= IMU_FLAG_SAMPLE_OVERRUN;
        }
    }
    float accel[3];
    float gyro[3];
    if (!cboard_imu_transform_vector(&mount_transform, sample.accel_mps2, accel) ||
        !cboard_imu_transform_vector(&mount_transform, sample.gyro_rad_s, gyro)) {
        working.flags |= IMU_FLAG_AXIS_MAPPING_INVALID;
        working.state = IMU_STATE_FAULT;
        publish();
        return;
    }

    working.sequence++;
    working.timestamp_us = started_us;
    working.sample_count++;
    working.temperature_c = sample.temperature_c;
    heater_update(sample.temperature_c, started_us);
    memcpy(working.accel_mps2, accel, sizeof(accel));
    memcpy(working.gyro_rad_s, gyro, sizeof(gyro));
    working.gyro_z_rad_s = gyro[2];
    working.spi_error_count = bmi088.transfer_error_count;
    working.flags &= ~(IMU_FLAG_SAMPLE_STALE | IMU_FLAG_NONFINITE_SAMPLE |
                       IMU_FLAG_SPI_ERROR | IMU_FLAG_ACCEL_SENSOR_ERROR);

    if (working.state == IMU_STATE_CALIBRATING && heater_stable) {
        bool complete = cboard_imu_calibrator_add(&calibrator, accel, gyro);
        working.calibration_state = calibrator.state;
        working.calibration_sample_count = (uint16_t)calibrator.count;
        working.movement_rejection_count = calibrator.movement_rejections;
        if (calibrator.state == IMU_CAL_REJECTED_MOVING) {
            working.flags |= IMU_FLAG_CALIBRATION_MOVEMENT_DETECTED;
        }
        if (complete) {
            if (cboard_imu_calibrator_result(&calibrator, working.gyro_bias_rad_s,
                                             working.calibration_gyro_variance)) {
                working.calibration_state = IMU_CAL_VALID;
                working.state = IMU_STATE_READY;
                working.flags &= ~(IMU_FLAG_CALIBRATION_NOT_VALID |
                                   IMU_FLAG_CALIBRATION_MOVEMENT_DETECTED);
                cboard_imu_attitude_reset(&attitude);
            } else {
                working.state = IMU_STATE_DEGRADED;
                working.flags |= IMU_FLAG_CALIBRATION_NOT_VALID;
            }
        } else if ((uint32_t)(started_us - calibration_started_us) > IMU_CALIBRATION_TIMEOUT_US) {
            working.calibration_state = IMU_CAL_TIMEOUT;
            working.state = IMU_STATE_DEGRADED;
            working.flags |= IMU_FLAG_CALIBRATION_NOT_VALID;
        }
    }

    if (working.state == IMU_STATE_READY) {
        float corrected_gyro[3];
        for (uint8_t index = 0; index < 3U; ++index) {
            corrected_gyro[index] = gyro[index] - working.gyro_bias_rad_s[index];
            working.gyro_rad_s[index] = corrected_gyro[index];
        }
        working.gyro_z_rad_s = corrected_gyro[2];
        float dt_s = (float)sample_elapsed * 1e-6f;
        float raw_wrapped;
        float raw_continuous;
        if (!cboard_imu_attitude_update(&attitude, corrected_gyro, accel, dt_s,
                                        working.quaternion_wxyz, &working.roll_rad,
                                        &working.pitch_rad, &raw_wrapped, &raw_continuous)) {
            working.flags |= IMU_FLAG_ATTITUDE_NOT_VALID;
            working.error_count++;
            working.state = IMU_STATE_DEGRADED;
        } else {
            (void)raw_wrapped;
            float reference = yaw_reference_valid ? yaw_reference_rad : 0.0f;
            working.yaw_continuous_rad = raw_continuous - reference;
            working.yaw_wrapped_rad = cboard_imu_wrap_0_2pi(working.yaw_continuous_rad);
            working.flags &= ~IMU_FLAG_ATTITUDE_NOT_VALID;
        }
    }
    working.age_us = 0U;
    uint32_t execution_us = cboard_diag_time_us() - started_us;
    if (execution_us > working.update_execution_max_us) working.update_execution_max_us = execution_us;
    publish();
}

bool cboard_imu_get_snapshot(cboard_imu_state_t *out) {
    if (out == NULL) return false;
    uint32_t before = 0U;
    uint32_t after = 0U;
    for (;;) {
        snapshot_irq_state_t irq_state = snapshot_enter_critical();
        before = atomic_load_explicit(&snapshot_lock, memory_order_acquire);
        if ((before & 1U) != 0U) {
            snapshot_exit_critical(irq_state);
            continue;
        }
        *out = snapshot;
        atomic_thread_fence(memory_order_acquire);
        after = atomic_load_explicit(&snapshot_lock, memory_order_acquire);
        snapshot_exit_critical(irq_state);
        if (before == after && (after & 1U) == 0U) break;
    }
    if (out->timestamp_us != 0U) out->age_us = cboard_diag_time_us() - out->timestamp_us;
    if (out->age_us > IMU_STALE_LIMIT_US) out->flags |= IMU_FLAG_SAMPLE_STALE;
    return out->sequence != 0U;
}

bool cboard_imu_is_healthy(void) {
    cboard_imu_state_t state;
    cboard_imu_get_snapshot(&state);
    uint32_t fatal = IMU_FLAG_ACCEL_INIT_FAILED | IMU_FLAG_GYRO_INIT_FAILED |
                     IMU_FLAG_SPI_ERROR | IMU_FLAG_SAMPLE_STALE |
                     IMU_FLAG_CALIBRATION_NOT_VALID | IMU_FLAG_ATTITUDE_NOT_VALID |
                     IMU_FLAG_TIMESTAMP_ERROR | IMU_FLAG_AXIS_MAPPING_INVALID |
                     IMU_FLAG_NONFINITE_SAMPLE | IMU_FLAG_AXIS_MAPPING_UNVERIFIED |
                     IMU_FLAG_ACCEL_SENSOR_ERROR | IMU_FLAG_TEMPERATURE_OVERTEMP;
    return state.state == IMU_STATE_READY && (state.flags & fatal) == 0U &&
           state.age_us <= IMU_STALE_LIMIT_US;
}

bool cboard_imu_zero_yaw(void) {
    if (working.state != IMU_STATE_READY || !attitude.initialized) return false;
    yaw_reference_rad = attitude.yaw_continuous;
    yaw_reference_valid = true;
    working.yaw_continuous_rad = 0.0f;
    working.yaw_wrapped_rad = 0.0f;
    publish();
    return true;
}

bool cboard_imu_set_yaw_reference(float angle_rad) {
    if (working.state != IMU_STATE_READY || !attitude.initialized || !isfinite(angle_rad)) return false;
    yaw_reference_rad = attitude.yaw_continuous - angle_rad;
    yaw_reference_valid = true;
    working.yaw_continuous_rad = angle_rad;
    working.yaw_wrapped_rad = cboard_imu_wrap_0_2pi(angle_rad);
    publish();
    return true;
}

uint32_t cboard_imu_get_age_us(void) {
    cboard_imu_state_t state;
    cboard_imu_get_snapshot(&state);
    return state.age_us;
}

uint32_t cboard_imu_get_error_flags(void) {
    cboard_imu_state_t state;
    cboard_imu_get_snapshot(&state);
    return state.flags;
}

#ifdef CBOARD_IMU_TEST
void cboard_imu_test_publish_state(const cboard_imu_state_t *state) {
    if (state == NULL) return;
    working = *state;
    publish();
}
#endif

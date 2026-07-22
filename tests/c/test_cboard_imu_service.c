#include "cboard_imu.h"
#include "bmi088_driver.h"
#include "cboard_imu_math.h"
#include "main.h"

#include <assert.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>

GPIO_TypeDef test_gpio_a;
GPIO_TypeDef test_gpio_b;
SPI_HandleTypeDef hspi1;

static uint32_t mock_time_us;
static bool mock_spi_ready = true;
static bool mock_heater_ready = true;
static bool mock_adc_ready = true;
static atomic_bool writer_done;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    (void)port; (void)pin; (void)state;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout) {
    (void)handle; (void)tx; (void)rx; (void)length; (void)timeout;
    return HAL_OK;
}
void HAL_Delay(uint32_t delay_ms) { (void)delay_ms; }
uint32_t cboard_diag_time_us(void) { return mock_time_us; }
bool SPI1_IsReady(void) { return mock_spi_ready; }
bool SPI1_SetFast(void) { return true; }
bool TIM10_IMU_Heater_IsReady(void) { return mock_heater_ready; }
void TIM10_IMU_Heater_Set(uint16_t compare) { (void)compare; }
bool ADC1_ReadMCUTemperature(float *temperature_c) {
    if (!mock_adc_ready) return false;
    *temperature_c = 25.0f;
    return true;
}
bmi088_result_t bmi088_init(bmi088_t *device, const bmi088_bus_t *bus) {
    (void)bus;
    device->accel_chip_id = 0x1EU;
    device->gyro_chip_id = 0x0FU;
    device->transfer_error_count = 0U;
    return BMI088_RESULT_OK;
}
bmi088_result_t bmi088_read_sample(bmi088_t *device, bmi088_sample_t *sample) {
    (void)device; (void)sample;
    return BMI088_RESULT_NOT_READY;
}
bool cboard_imu_transform_valid(const imu_mount_transform_t *transform) { return transform != NULL; }
bool cboard_imu_transform_vector(
    const imu_mount_transform_t *transform, const float sensor[3], float robot[3]) {
    (void)transform;
    memcpy(robot, sensor, 3U * sizeof(float));
    return true;
}
void cboard_imu_calibrator_reset(cboard_imu_calibrator_t *calibrator) {
    memset(calibrator, 0, sizeof(*calibrator));
    calibrator->state = IMU_CAL_COLLECTING;
}
bool cboard_imu_calibrator_add(
    cboard_imu_calibrator_t *calibrator, const float accel[3], const float gyro[3]) {
    (void)calibrator; (void)accel; (void)gyro;
    return false;
}
bool cboard_imu_calibrator_result(
    const cboard_imu_calibrator_t *calibrator, float bias[3], float variance[3]) {
    (void)calibrator; (void)bias; (void)variance;
    return false;
}
void cboard_imu_attitude_reset(cboard_imu_attitude_t *attitude) {
    memset(attitude, 0, sizeof(*attitude));
}
bool cboard_imu_attitude_update(
    cboard_imu_attitude_t *attitude, const float gyro[3], const float accel[3], float dt_s,
    float quaternion[4], float *roll, float *pitch, float *yaw_wrapped, float *yaw_continuous) {
    (void)attitude; (void)gyro; (void)accel; (void)dt_s; (void)quaternion;
    (void)roll; (void)pitch; (void)yaw_wrapped; (void)yaw_continuous;
    return false;
}
float cboard_imu_wrap_0_2pi(float angle_rad) { return angle_rad; }

static cboard_imu_state_t patterned_state(uint32_t value) {
    cboard_imu_state_t state;
    memset(&state, 0, sizeof(state));
    state.sequence = value;
    state.timestamp_us = mock_time_us;
    state.sample_count = value;
    state.error_count = ~value;
    state.missed_sample_count = value ^ 0xA5A5A5A5U;
    state.state = IMU_STATE_READY;
    state.calibration_state = IMU_CAL_VALID;
    state.accel_chip_id = 0x1EU;
    state.gyro_chip_id = 0x0FU;
    return state;
}

static void assert_pattern(const cboard_imu_state_t *state) {
    assert(state->sample_count == state->sequence);
    assert(state->error_count == ~state->sequence);
    assert(state->missed_sample_count == (state->sequence ^ 0xA5A5A5A5U));
}

static void *snapshot_writer(void *unused) {
    (void)unused;
    for (uint32_t value = 1U; value <= 100000U; ++value) {
        cboard_imu_state_t state = patterned_state(value);
        cboard_imu_test_publish_state(&state);
    }
    atomic_store(&writer_done, true);
    return NULL;
}

static void test_snapshot_atomicity(void) {
    pthread_t writer;
    atomic_store(&writer_done, false);
    assert(pthread_create(&writer, NULL, snapshot_writer, NULL) == 0);
    do {
        cboard_imu_state_t state;
        if (cboard_imu_get_snapshot(&state)) assert_pattern(&state);
    } while (!atomic_load(&writer_done));
    assert(pthread_join(writer, NULL) == 0);
    cboard_imu_state_t final;
    assert(cboard_imu_get_snapshot(&final));
    assert_pattern(&final);
    assert(final.sequence == 100000U);
}

static void test_heater_unavailable_reaches_terminal_degraded_state(void) {
    mock_heater_ready = false;
    mock_adc_ready = true;
    mock_time_us = 100U;
    cboard_imu_init();
    cboard_imu_state_t state;
    (void)cboard_imu_get_snapshot(&state);
    assert(state.state == IMU_STATE_DEGRADED);
    assert(state.calibration_state == IMU_CAL_NOT_STARTED);
    assert((state.flags & IMU_FLAG_HEATER_UNAVAILABLE) != 0U);
    assert((state.flags & IMU_FLAG_CALIBRATION_NOT_VALID) != 0U);
    mock_heater_ready = true;
}

static void test_health_and_stale_are_derived_from_snapshot(void) {
    mock_time_us = 100000U;
    cboard_imu_state_t state = patterned_state(7U);
    state.timestamp_us = mock_time_us;
    cboard_imu_test_publish_state(&state);
    assert(cboard_imu_is_healthy());
    mock_time_us += 20001U;
    cboard_imu_state_t stale;
    assert(cboard_imu_get_snapshot(&stale));
    assert((stale.flags & IMU_FLAG_SAMPLE_STALE) != 0U);
    assert(!cboard_imu_is_healthy());

    state = patterned_state(8U);
    state.timestamp_us = mock_time_us;
    state.state = IMU_STATE_CALIBRATING;
    state.calibration_state = IMU_CAL_COLLECTING;
    state.flags = IMU_FLAG_CALIBRATION_NOT_VALID;
    cboard_imu_test_publish_state(&state);
    assert(!cboard_imu_is_healthy());
}

int main(void) {
    test_heater_unavailable_reaches_terminal_degraded_state();
    test_health_and_stale_are_derived_from_snapshot();
    test_snapshot_atomicity();
    return 0;
}

#include "bmi088_driver.h"

#include <math.h>
#include <string.h>

#define BMI088_SPI_READ 0x80U
#define BMI088_ACCEL_SOFTRESET 0x7EU
#define BMI088_ACCEL_SOFTRESET_VALUE 0xB6U
#define BMI088_ACCEL_PWR_CONF 0x7CU
#define BMI088_ACCEL_PWR_CTRL 0x7DU
#define BMI088_ACCEL_CONF 0x40U
#define BMI088_ACCEL_RANGE 0x41U
#define BMI088_ACCEL_INT1_IO_CTRL 0x53U
#define BMI088_ACCEL_INT_MAP_DATA 0x58U
#define BMI088_ACCEL_STATUS 0x03U
#define BMI088_ACCEL_ERROR 0x02U
#define BMI088_ACCEL_DATA 0x12U
#define BMI088_ACCEL_TEMPERATURE 0x22U

#define BMI088_GYRO_SOFTRESET 0x14U
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6U
#define BMI088_GYRO_RANGE 0x0FU
#define BMI088_GYRO_BANDWIDTH 0x10U
#define BMI088_GYRO_LPM1 0x11U
#define BMI088_GYRO_CTRL 0x15U
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16U
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18U
#define BMI088_GYRO_STATUS 0x0AU
#define BMI088_GYRO_DATA 0x02U

typedef struct {
    uint8_t reg;
    uint8_t value;
} register_config_t;

static bool bus_valid(const bmi088_bus_t *bus) {
    return bus != NULL && bus->select != NULL && bus->transfer != NULL && bus->delay_ms != NULL;
}

static bool transfer_selected(
    bmi088_t *device,
    bmi088_device_t sensor,
    const uint8_t *tx,
    uint8_t *rx,
    size_t length) {
    if (!device->bus.select(sensor, true)) {
        device->transfer_error_count++;
        return false;
    }
    bool ok = device->bus.transfer(tx, rx, length);
    bool deselected = device->bus.select(sensor, false);
    if (!ok || !deselected) device->transfer_error_count++;
    return ok && deselected;
}

static bool read_many(
    bmi088_t *device,
    bmi088_device_t sensor,
    uint8_t reg,
    uint8_t *data,
    size_t length) {
    if (device == NULL || data == NULL || length == 0U || length > 16U) return false;
    uint8_t tx[18] = {0};
    uint8_t rx[18] = {0};
    size_t offset = sensor == BMI088_DEVICE_ACCEL ? 2U : 1U;
    tx[0] = reg | BMI088_SPI_READ;
    if (!transfer_selected(device, sensor, tx, rx, length + offset)) return false;
    memcpy(data, rx + offset, length);
    return true;
}

static bool write_register(
    bmi088_t *device,
    bmi088_device_t sensor,
    uint8_t reg,
    uint8_t value) {
    uint8_t tx[2] = {reg & (uint8_t)~BMI088_SPI_READ, value};
    uint8_t rx[2] = {0};
    return transfer_selected(device, sensor, tx, rx, sizeof(tx));
}

/* The first SPI read after power-up or soft reset selects the BMI088 SPI
 * interface and is not a reliable chip-ID sample. The vendor reference reads
 * twice with a 150 us gap; the millisecond bus callback is used here only
 * during bounded initialization. */
static bool read_chip_id_stable(
    bmi088_t *device,
    bmi088_device_t sensor,
    uint8_t *chip_id) {
    uint8_t discarded = 0U;
    if (!read_many(device, sensor, 0x00U, &discarded, 1U)) return false;
    device->bus.delay_ms(1U);
    return read_many(device, sensor, 0x00U, chip_id, 1U);
}

bmi088_result_t bmi088_read_register(
    bmi088_t *device,
    bmi088_device_t sensor,
    uint8_t reg,
    uint8_t *value) {
    if (device == NULL || value == NULL || sensor > BMI088_DEVICE_GYRO) return BMI088_RESULT_BAD_ARGUMENT;
    return read_many(device, sensor, reg, value, 1U) ? BMI088_RESULT_OK : BMI088_RESULT_BUS_ERROR;
}

static bmi088_result_t configure(
    bmi088_t *device,
    bmi088_device_t sensor,
    const register_config_t *config,
    size_t count,
    bmi088_result_t config_error) {
    for (size_t index = 0; index < count; ++index) {
        uint8_t value = 0U;
        if (!write_register(device, sensor, config[index].reg, config[index].value)) {
            return BMI088_RESULT_BUS_ERROR;
        }
        device->bus.delay_ms(2U);
        if (!read_many(device, sensor, config[index].reg, &value, 1U)) return BMI088_RESULT_BUS_ERROR;
        if (value != config[index].value) return config_error;
    }
    return BMI088_RESULT_OK;
}

bmi088_result_t bmi088_init(bmi088_t *device, const bmi088_bus_t *bus) {
    if (device == NULL || !bus_valid(bus)) return BMI088_RESULT_BAD_ARGUMENT;
    memset(device, 0, sizeof(*device));
    device->bus = *bus;
    device->bus.select(BMI088_DEVICE_ACCEL, false);
    device->bus.select(BMI088_DEVICE_GYRO, false);
    device->bus.delay_ms(5U);

    if (!read_chip_id_stable(device, BMI088_DEVICE_ACCEL, &device->accel_chip_id)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    if (device->accel_chip_id != BMI088_ACCEL_CHIP_ID_VALUE) return BMI088_RESULT_ACCEL_NOT_FOUND;
    if (!write_register(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_SOFTRESET, BMI088_ACCEL_SOFTRESET_VALUE)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    device->bus.delay_ms(80U);
    if (!read_chip_id_stable(device, BMI088_DEVICE_ACCEL, &device->accel_chip_id)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    if (device->accel_chip_id != BMI088_ACCEL_CHIP_ID_VALUE) return BMI088_RESULT_ACCEL_NOT_FOUND;
    static const register_config_t accel_config[] = {
        {BMI088_ACCEL_PWR_CTRL, 0x04U},
        {BMI088_ACCEL_PWR_CONF, 0x00U},
        {BMI088_ACCEL_CONF, 0xABU}, /* normal filter, 800 Hz */
        {BMI088_ACCEL_RANGE, 0x00U}, /* +/-3 g */
        {BMI088_ACCEL_INT1_IO_CTRL, 0x08U}, /* enabled, push-pull, active low */
        {BMI088_ACCEL_INT_MAP_DATA, 0x04U}, /* DRDY -> INT1 */
    };
    bmi088_result_t result = configure(device, BMI088_DEVICE_ACCEL, accel_config,
                                       sizeof(accel_config) / sizeof(accel_config[0]),
                                       BMI088_RESULT_ACCEL_CONFIG_ERROR);
    if (result != BMI088_RESULT_OK) return result;

    if (!read_chip_id_stable(device, BMI088_DEVICE_GYRO, &device->gyro_chip_id)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    if (device->gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_RESULT_GYRO_NOT_FOUND;
    if (!write_register(device, BMI088_DEVICE_GYRO, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    device->bus.delay_ms(80U);
    if (!read_chip_id_stable(device, BMI088_DEVICE_GYRO, &device->gyro_chip_id)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    if (device->gyro_chip_id != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_RESULT_GYRO_NOT_FOUND;
    static const register_config_t gyro_config[] = {
        {BMI088_GYRO_RANGE, 0x00U}, /* +/-2000 dps */
        {BMI088_GYRO_BANDWIDTH, 0x82U}, /* 1000 Hz ODR, 116 Hz filter */
        {BMI088_GYRO_LPM1, 0x00U}, /* normal mode */
        {BMI088_GYRO_CTRL, 0x80U}, /* DRDY enabled */
        {BMI088_GYRO_INT3_INT4_IO_CONF, 0x00U}, /* INT3 push-pull active low */
        {BMI088_GYRO_INT3_INT4_IO_MAP, 0x01U}, /* DRDY -> INT3 */
    };
    result = configure(device, BMI088_DEVICE_GYRO, gyro_config,
                       sizeof(gyro_config) / sizeof(gyro_config[0]),
                       BMI088_RESULT_GYRO_CONFIG_ERROR);
    if (result != BMI088_RESULT_OK) return result;

    uint8_t accel_error = 0U;
    if (!read_many(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_ERROR, &accel_error, 1U)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    if ((accel_error & 0x1CU) != 0U) return BMI088_RESULT_SENSOR_ERROR;
    device->initialized = true;
    return BMI088_RESULT_OK;
}

static int16_t little_i16(const uint8_t *data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

bmi088_result_t bmi088_read_sample(bmi088_t *device, bmi088_sample_t *sample) {
    if (device == NULL || sample == NULL || !device->initialized) return BMI088_RESULT_BAD_ARGUMENT;
    uint8_t accel[6];
    uint8_t gyro[6];
    uint8_t temperature[2];
    memset(sample, 0, sizeof(*sample));
    if (!read_many(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_STATUS, &sample->accel_status, 1U) ||
        !read_many(device, BMI088_DEVICE_GYRO, BMI088_GYRO_STATUS, &sample->gyro_status, 1U)) {
        return BMI088_RESULT_BUS_ERROR;
    }
    /* Pace fusion from the 800 Hz accelerometer DRDY. Gyro DRDY is only a
     * 280-400 us pulse and cannot be safely used as a 1 kHz polling gate;
     * the latest 1 kHz gyro registers are read at each accel-ready epoch. */
    if ((sample->accel_status & 0x80U) == 0U) {
        return BMI088_RESULT_NOT_READY;
    }
    if (!read_many(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_ERROR, &sample->accel_error, 1U) ||
        !read_many(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_DATA, accel, sizeof(accel)) ||
        !read_many(device, BMI088_DEVICE_GYRO, BMI088_GYRO_DATA, gyro, sizeof(gyro)) ||
        !read_many(device, BMI088_DEVICE_ACCEL, BMI088_ACCEL_TEMPERATURE, temperature, sizeof(temperature))) {
        return BMI088_RESULT_BUS_ERROR;
    }
    for (uint8_t index = 0; index < 3U; ++index) {
        sample->accel_mps2[index] = (float)little_i16(accel + index * 2U) * BMI088_ACCEL_LSB_TO_MPS2;
        sample->gyro_rad_s[index] = (float)little_i16(gyro + index * 2U) * BMI088_GYRO_LSB_TO_RAD_S;
    }
    int16_t raw_temperature = (int16_t)(((uint16_t)temperature[0] << 3) |
                                        ((uint16_t)temperature[1] >> 5));
    if (raw_temperature > 1023) raw_temperature -= 2048;
    sample->temperature_c = (float)raw_temperature * 0.125f + 23.0f;
    if ((sample->accel_error & 0x1CU) != 0U) return BMI088_RESULT_SENSOR_ERROR;
    if (!isfinite(sample->accel_mps2[0]) || !isfinite(sample->accel_mps2[1]) ||
        !isfinite(sample->accel_mps2[2]) || !isfinite(sample->gyro_rad_s[0]) ||
        !isfinite(sample->gyro_rad_s[1]) || !isfinite(sample->gyro_rad_s[2]) ||
        !isfinite(sample->temperature_c)) return BMI088_RESULT_NONFINITE;
    return BMI088_RESULT_OK;
}

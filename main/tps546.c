#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pmbus_commands.h"
#include "tps546.h"

#define TAG "tps546"

#define TPS546_I2C_PORT 0
#define TPS546_I2C_SCL_PIN GPIO_NUM_48
#define TPS546_I2C_SDA_PIN GPIO_NUM_47
#define TPS546_ENABLE_PIN GPIO_NUM_10
#define TPS546_PGOOD_PIN GPIO_NUM_11
#define TPS546_I2C_SPEED_HZ 400000
#define TPS546_I2C_TIMEOUT_MS -1

#define OPERATION_OFF 0x00
#define OPERATION_ON 0x80
#define TPS546_DEFAULT_VOUT_MODE 0x97

static const uint8_t DEVICE_ID_TPS546D24A[] = {0x54, 0x49, 0x54, 0x6D, 0x24, 0x41};
static const uint8_t DEVICE_ID_TPS546D24S[] = {0x54, 0x49, 0x54, 0x6D, 0x24, 0x62};

typedef struct {
    uint8_t phase;
    uint16_t smbalert_mask[7];
    int frequency_switch_khz;
    uint8_t sync_config;
    uint16_t stack_config;
    uint16_t interleave;
    uint16_t misc_options;
    uint16_t pin_detect_override;
    uint8_t device_address;
    uint8_t compensation_config[5];
    uint8_t power_stage_config;
    uint8_t telemetry_config[6];
    float vout_command;
    uint16_t vout_trim;
    float vout_max;
    float vout_margin_high;
    float vout_margin_low;
    uint16_t vout_transition_rate;
    float vout_scale_loop;
    float vout_min;
    float vin_on;
    float vin_off;
    uint16_t iout_cal_gain;
    uint16_t iout_cal_offset;
    float vout_ov_fault_limit;
    uint8_t vout_ov_fault_response;
    float vout_ov_warn_limit;
    float vout_uv_warn_limit;
    float vout_uv_fault_limit;
    uint8_t vout_uv_fault_response;
    float iout_oc_fault_limit;
    uint8_t iout_oc_fault_response;
    float iout_oc_warn_limit;
    int ot_fault_limit;
    uint8_t ot_fault_response;
    int ot_warn_limit;
    float vin_ov_fault_limit;
    uint8_t vin_ov_fault_response;
    float vin_uv_warn_limit;
    int ton_delay;
    int ton_rise;
    int ton_max_fault_limit;
    uint8_t ton_max_fault_response;
    int toff_delay;
    int toff_fall;
} tps546_birds_config_t;

static const tps546_birds_config_t TPS546_CONFIG_BIRDS = {
    .phase = 0xFF,
    .smbalert_mask = {0x0200, 0x1800, 0xE800, 0x0000, 0x0000, 0x0100, 0x4200},
    .frequency_switch_khz = 325,
    .sync_config = 0x00,
    .stack_config = 0x0000,
    .interleave = 0x0010,
    .misc_options = 0x0000,
    .pin_detect_override = 0x0000,
    .device_address = 0x24,
    .compensation_config = {0x13, 0x11, 0x8C, 0x1D, 0x06},
    .power_stage_config = 0x70,
    .telemetry_config = {0x03, 0x03, 0x03, 0x03, 0x03, 0x00},
    .vout_command = 2.8f,
    .vout_trim = 0x0000,
    .vout_max = 3.5f,
    .vout_margin_high = 1.1f,
    .vout_margin_low = 0.90f,
    .vout_transition_rate = 0xE010,
    .vout_scale_loop = 0.125f,
    .vout_min = 2.1f,
    .vin_on = 11.0f,
    .vin_off = 10.5f,
    .iout_cal_gain = 0xC880,
    .iout_cal_offset = 0xE000,
    .vout_ov_fault_limit = 1.25f,
    .vout_ov_fault_response = 0xBD,
    .vout_ov_warn_limit = 1.16f,
    .vout_uv_warn_limit = 0.90f,
    .vout_uv_fault_limit = 0.75f,
    .vout_uv_fault_response = 0xBE,
    .iout_oc_fault_limit = 55.0f,
    .iout_oc_fault_response = 0xC0,
    .iout_oc_warn_limit = 50.0f,
    .ot_fault_limit = 145,
    .ot_fault_response = 0xFF,
    .ot_warn_limit = 105,
    .vin_ov_fault_limit = 14.0f,
    .vin_ov_fault_response = 0xB7,
    .vin_uv_warn_limit = 11.0f,
    .ton_delay = 0,
    .ton_rise = 3,
    .ton_max_fault_limit = 0,
    .ton_max_fault_response = 0x3B,
    .toff_delay = 0,
    .toff_fall = 0,
};

static bool s_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_tps546_dev;

static esp_err_t tps546_require_init(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t smb_read_byte(uint8_t command, uint8_t *data)
{
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    return i2c_master_transmit_receive(s_tps546_dev, &command, 1, data, 1, TPS546_I2C_TIMEOUT_MS);
}

static esp_err_t smb_write_byte(uint8_t command, uint8_t data)
{
    uint8_t buffer[2] = {command, data};
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    return i2c_master_transmit(s_tps546_dev, buffer, sizeof(buffer), TPS546_I2C_TIMEOUT_MS);
}

static esp_err_t smb_write_word(uint8_t command, uint16_t value)
{
    uint8_t buffer[3] = {command, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    return i2c_master_transmit(s_tps546_dev, buffer, sizeof(buffer), TPS546_I2C_TIMEOUT_MS);
}

static esp_err_t smb_write_addr(uint8_t command)
{
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    return i2c_master_transmit(s_tps546_dev, &command, 1, TPS546_I2C_TIMEOUT_MS);
}

static esp_err_t smb_write_block(uint8_t command, const uint8_t *data, size_t len)
{
    uint8_t *buffer;

    ESP_RETURN_ON_FALSE(len <= 255, ESP_ERR_INVALID_SIZE, TAG, "block too large");
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");

    buffer = malloc(len + 2);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    buffer[0] = command;
    buffer[1] = (uint8_t)len;
    memcpy(&buffer[2], data, len);

    esp_err_t err = i2c_master_transmit(s_tps546_dev, buffer, len + 2, TPS546_I2C_TIMEOUT_MS);
    free(buffer);
    return err;
}

static esp_err_t smb_read_word(uint8_t command, uint16_t *value)
{
    uint8_t buffer[2];
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_tps546_dev, &command, 1, buffer, sizeof(buffer), TPS546_I2C_TIMEOUT_MS), TAG, "read word failed");

    *value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    return ESP_OK;
}

static esp_err_t smb_read_block(uint8_t command, uint8_t *data, size_t len)
{
    uint8_t *buffer = malloc(len + 1);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = tps546_require_init();
    if (err == ESP_OK) {
        err = i2c_master_transmit_receive(s_tps546_dev, &command, 1, buffer, len + 1, TPS546_I2C_TIMEOUT_MS);
    }

    if (err == ESP_OK) {
        memcpy(data, &buffer[1], len);
    }

    free(buffer);
    return err;
}

static float slinear11_to_float(uint16_t value)
{
    int exponent;
    int mantissa;

    if (value & 0x8000) {
        exponent = -1 * ((((int)(~value)) >> 11 & 0x1F) + 1);
    } else {
        exponent = (int)(value >> 11);
    }

    if (value & 0x0400) {
        mantissa = -1 * ((((int)(~value)) & 0x03FF) + 1);
    } else {
        mantissa = (int)(value & 0x03FF);
    }

    return (float)mantissa * powf(2.0f, (float)exponent);
}

static uint16_t int_to_slinear11(int value)
{
    int exponent = 0;
    int mantissa = 0;

    if (value < 0) {
        return 0;
    }

    for (int i = 0; i <= 15; i++) {
        mantissa = (int)((float)value / powf(2.0f, (float)i));
        if (mantissa < 1024) {
            exponent = i;
            break;
        }
    }

    return (uint16_t)((((uint16_t)exponent) << 11) | (uint16_t)mantissa);
}

static uint16_t float_to_slinear11(float value)
{
    int exponent = 0;
    int mantissa = 0;

    if (value <= 0.0f) {
        return 0;
    }

    for (int i = 0; i <= 15; i++) {
        mantissa = (int)(value * powf(2.0f, (float)i));
        if (mantissa >= 1024) {
            exponent = i - 1;
            mantissa = (int)(value * powf(2.0f, (float)exponent));
            break;
        }
    }

    return (uint16_t)((((~exponent) + 1) << 11 & 0xF800) | (mantissa & 0x07FF));
}

static float ulinear16_to_float(uint16_t value, uint8_t vout_mode)
{
    int exponent;

    if (vout_mode & 0x10) {
        exponent = -1 * ((((int)(~vout_mode)) & 0x1F) + 1);
    } else {
        exponent = (int)(vout_mode & 0x1F);
    }

    return (float)value * powf(2.0f, (float)exponent);
}

static uint16_t float_to_ulinear16(float value, uint8_t vout_mode)
{
    int exponent;

    if (vout_mode & 0x10) {
        exponent = -1 * ((((int)(~vout_mode)) & 0x1F) + 1);
    } else {
        exponent = (int)(vout_mode & 0x1F);
    }

    return (uint16_t)(value / powf(2.0f, (float)exponent));
}

static esp_err_t tps546_init_disable(void)
{
    uint8_t on_off_config = (ON_OFF_CONFIG_DELAY | ON_OFF_CONFIG_POLARITY | ON_OFF_CONFIG_CMD | ON_OFF_CONFIG_PU);

    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_OPERATION, OPERATION_OFF), TAG, "set OPERATION off failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_ON_OFF_CONFIG, on_off_config), TAG, "set ON_OFF_CONFIG failed");
    return ESP_OK;
}

static esp_err_t tps546_write_birds_config(void)
{
    static const uint8_t status_selectors[] = {
        PMBUS_STATUS_VOUT_SELECTOR,
        PMBUS_STATUS_IOUT_SELECTOR,
        PMBUS_STATUS_INPUT_SELECTOR,
        PMBUS_STATUS_TEMPERATURE_SELECTOR,
        PMBUS_STATUS_CML_SELECTOR,
        PMBUS_STATUS_OTHER_SELECTOR,
        PMBUS_STATUS_MFR_SPECIFIC_SELECTOR,
    };

    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_PHASE, TPS546_CONFIG_BIRDS.phase), TAG, "write phase failed");

    for (size_t i = 0; i < sizeof(status_selectors); i++) {
        uint16_t value = TPS546_CONFIG_BIRDS.smbalert_mask[i] | status_selectors[i];
        ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_SMBALERT_MASK, value), TAG, "write SMBALERT_MASK failed");
    }

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_FREQUENCY_SWITCH, int_to_slinear11(TPS546_CONFIG_BIRDS.frequency_switch_khz)), TAG, "write frequency failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_SYNC_CONFIG, TPS546_CONFIG_BIRDS.sync_config), TAG, "write sync config failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_STACK_CONFIG, TPS546_CONFIG_BIRDS.stack_config), TAG, "write stack config failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_INTERLEAVE, TPS546_CONFIG_BIRDS.interleave), TAG, "write interleave failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_MISC_OPTIONS, TPS546_CONFIG_BIRDS.misc_options), TAG, "write misc options failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_PIN_DETECT_OVERRIDE, TPS546_CONFIG_BIRDS.pin_detect_override), TAG, "write pin detect override failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_SLAVE_ADDRESS, TPS546_CONFIG_BIRDS.device_address), TAG, "write slave address failed");

    ESP_RETURN_ON_ERROR(smb_write_block(PMBUS_COMPENSATION_CONFIG, TPS546_CONFIG_BIRDS.compensation_config, sizeof(TPS546_CONFIG_BIRDS.compensation_config)), TAG, "write compensation config failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(smb_write_block(PMBUS_POWER_STAGE_CONFIG, &TPS546_CONFIG_BIRDS.power_stage_config, 1), TAG, "write power stage config failed");
    ESP_RETURN_ON_ERROR(smb_write_block(PMBUS_TELEMETRY_CONFIG, TPS546_CONFIG_BIRDS.telemetry_config, sizeof(TPS546_CONFIG_BIRDS.telemetry_config)), TAG, "write telemetry config failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_COMMAND, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_command, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout command failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_TRIM, TPS546_CONFIG_BIRDS.vout_trim), TAG, "write vout trim failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_MAX, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_max, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout max failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_MARGIN_HIGH, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_margin_high, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout margin high failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_MARGIN_LOW, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_margin_low, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout margin low failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_TRANSITION_RATE, TPS546_CONFIG_BIRDS.vout_transition_rate), TAG, "write vout transition rate failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_SCALE_LOOP, float_to_slinear11(TPS546_CONFIG_BIRDS.vout_scale_loop)), TAG, "write vout scale loop failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_MIN, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_min, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout min failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VIN_ON, float_to_slinear11(TPS546_CONFIG_BIRDS.vin_on)), TAG, "write vin on failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VIN_OFF, float_to_slinear11(TPS546_CONFIG_BIRDS.vin_off)), TAG, "write vin off failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_IOUT_CAL_GAIN, TPS546_CONFIG_BIRDS.iout_cal_gain), TAG, "write iout cal gain failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_IOUT_CAL_OFFSET, TPS546_CONFIG_BIRDS.iout_cal_offset), TAG, "write iout cal offset failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_OV_FAULT_LIMIT, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_ov_fault_limit, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout ov fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_VOUT_OV_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.vout_ov_fault_response), TAG, "write vout ov fault response failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_OV_WARN_LIMIT, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_ov_warn_limit, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout ov warn limit failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_UV_WARN_LIMIT, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_uv_warn_limit, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout uv warn limit failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VOUT_UV_FAULT_LIMIT, float_to_ulinear16(TPS546_CONFIG_BIRDS.vout_uv_fault_limit, TPS546_DEFAULT_VOUT_MODE)), TAG, "write vout uv fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_VOUT_UV_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.vout_uv_fault_response), TAG, "write vout uv fault response failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_IOUT_OC_FAULT_LIMIT, float_to_slinear11(TPS546_CONFIG_BIRDS.iout_oc_fault_limit)), TAG, "write iout oc fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_IOUT_OC_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.iout_oc_fault_response), TAG, "write iout oc fault response failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_IOUT_OC_WARN_LIMIT, float_to_slinear11(TPS546_CONFIG_BIRDS.iout_oc_warn_limit)), TAG, "write iout oc warn limit failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_OT_FAULT_LIMIT, int_to_slinear11(TPS546_CONFIG_BIRDS.ot_fault_limit)), TAG, "write ot fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_OT_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.ot_fault_response), TAG, "write ot fault response failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_OT_WARN_LIMIT, int_to_slinear11(TPS546_CONFIG_BIRDS.ot_warn_limit)), TAG, "write ot warn limit failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VIN_OV_FAULT_LIMIT, float_to_slinear11(TPS546_CONFIG_BIRDS.vin_ov_fault_limit)), TAG, "write vin ov fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_VIN_OV_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.vin_ov_fault_response), TAG, "write vin ov fault response failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_VIN_UV_WARN_LIMIT, float_to_slinear11(TPS546_CONFIG_BIRDS.vin_uv_warn_limit)), TAG, "write vin uv warn limit failed");

    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_TON_DELAY, int_to_slinear11(TPS546_CONFIG_BIRDS.ton_delay)), TAG, "write ton delay failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_TON_RISE, int_to_slinear11(TPS546_CONFIG_BIRDS.ton_rise)), TAG, "write ton rise failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_TON_MAX_FAULT_LIMIT, int_to_slinear11(TPS546_CONFIG_BIRDS.ton_max_fault_limit)), TAG, "write ton max fault limit failed");
    ESP_RETURN_ON_ERROR(smb_write_byte(PMBUS_TON_MAX_FAULT_RESPONSE, TPS546_CONFIG_BIRDS.ton_max_fault_response), TAG, "write ton max fault response failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_TOFF_DELAY, int_to_slinear11(TPS546_CONFIG_BIRDS.toff_delay)), TAG, "write toff delay failed");
    ESP_RETURN_ON_ERROR(smb_write_word(PMBUS_TOFF_FALL, int_to_slinear11(TPS546_CONFIG_BIRDS.toff_fall)), TAG, "write toff fall failed");

    return ESP_OK;
}

esp_err_t tps546_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    const gpio_config_t enable_pin_config = {
        .pin_bit_mask = (1ULL << TPS546_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&enable_pin_config), TAG, "enable gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(TPS546_ENABLE_PIN, 0), TAG, "enable gpio default-low failed");

    const gpio_config_t pgood_pin_config = {
        .pin_bit_mask = (1ULL << TPS546_PGOOD_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pgood_pin_config), TAG, "pgood gpio config failed");

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TPS546_I2C_PORT,
        .scl_io_num = TPS546_I2C_SCL_PIN,
        .sda_io_num = TPS546_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG, "i2c bus init failed");

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TPS546_I2C_ADDRESS,
        .scl_speed_hz = TPS546_I2C_SPEED_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_tps546_dev), TAG, "i2c device add failed");

    s_initialized = true;
    return ESP_OK;
}

esp_err_t tps546_probe(tps546_device_id_t *device)
{
    uint8_t id[6];
    ESP_RETURN_ON_ERROR(smb_read_block(PMBUS_IC_DEVICE_ID, id, sizeof(id)), TAG, "device id read failed");

    if (device != NULL) {
        memcpy(device->id, id, sizeof(id));
        if (memcmp(id, DEVICE_ID_TPS546D24S, sizeof(id)) == 0) {
            device->name = "TPS546D24S";
        } else if (memcmp(id, DEVICE_ID_TPS546D24A, sizeof(id)) == 0) {
            device->name = "TPS546D24A";
        } else {
            device->name = "unknown";
        }
    }

    return ESP_OK;
}

bool tps546_get_pgood_pin(void)
{
    return gpio_get_level(TPS546_PGOOD_PIN) != 0;
}

bool tps546_get_enable_pin(void)
{
    return gpio_get_level(TPS546_ENABLE_PIN) != 0;
}

esp_err_t tps546_read_status(tps546_status_snapshot_t *snapshot)
{
    ESP_RETURN_ON_FALSE(snapshot != NULL, ESP_ERR_INVALID_ARG, TAG, "snapshot is null");

    memset(snapshot, 0, sizeof(*snapshot));

    ESP_RETURN_ON_ERROR(smb_read_word(PMBUS_STATUS_WORD, &snapshot->status_word), TAG, "status word read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_VOUT, &snapshot->status_vout), TAG, "status vout read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_IOUT, &snapshot->status_iout), TAG, "status iout read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_INPUT, &snapshot->status_input), TAG, "status input read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_TEMPERATURE, &snapshot->status_temperature), TAG, "status temp read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_CML, &snapshot->status_cml), TAG, "status cml read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_OTHER, &snapshot->status_other), TAG, "status other read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_STATUS_MFR_SPECIFIC, &snapshot->status_mfr_specific), TAG, "status mfr read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_OPERATION, &snapshot->operation), TAG, "operation read failed");
    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_ON_OFF_CONFIG, &snapshot->on_off_config), TAG, "on_off_config read failed");
    snapshot->enable_pin_high = tps546_get_enable_pin();
    snapshot->pgood_pin_high = tps546_get_pgood_pin();

    return ESP_OK;
}

esp_err_t tps546_read_telemetry(tps546_telemetry_t *telemetry)
{
    uint16_t raw_vin;
    uint16_t raw_vout;
    uint16_t raw_iout;
    uint16_t raw_temp;

    ESP_RETURN_ON_FALSE(telemetry != NULL, ESP_ERR_INVALID_ARG, TAG, "telemetry is null");

    memset(telemetry, 0, sizeof(*telemetry));

    ESP_RETURN_ON_ERROR(smb_read_byte(PMBUS_VOUT_MODE, &telemetry->vout_mode), TAG, "vout mode read failed");
    ESP_RETURN_ON_ERROR(smb_read_word(PMBUS_READ_VIN, &raw_vin), TAG, "read vin failed");
    ESP_RETURN_ON_ERROR(smb_read_word(PMBUS_READ_VOUT, &raw_vout), TAG, "read vout failed");
    ESP_RETURN_ON_ERROR(smb_read_word(PMBUS_READ_IOUT, &raw_iout), TAG, "read iout failed");
    ESP_RETURN_ON_ERROR(smb_read_word(PMBUS_READ_TEMPERATURE_1, &raw_temp), TAG, "read temp failed");

    telemetry->read_vin = slinear11_to_float(raw_vin);
    telemetry->read_vout = ulinear16_to_float(raw_vout, telemetry->vout_mode);
    telemetry->read_iout = slinear11_to_float(raw_iout);
    telemetry->read_temperature_c = slinear11_to_float(raw_temp);

    return ESP_OK;
}

esp_err_t tps546_clear_faults(void)
{
    return smb_write_addr(PMBUS_CLEAR_FAULTS);
}

esp_err_t tps546_set_enable_pin(bool enabled)
{
    ESP_RETURN_ON_ERROR(tps546_require_init(), TAG, "TPS546 not initialized");
    return gpio_set_level(TPS546_ENABLE_PIN, enabled ? 1 : 0);
}

esp_err_t tps546_set_operation(bool enabled)
{
    return smb_write_byte(PMBUS_OPERATION, enabled ? OPERATION_ON : OPERATION_OFF);
}

esp_err_t tps546_read_byte_reg(uint8_t command, uint8_t *value)
{
    return smb_read_byte(command, value);
}

esp_err_t tps546_read_word_reg(uint8_t command, uint16_t *value)
{
    return smb_read_word(command, value);
}

esp_err_t tps546_read_block_reg(uint8_t command, uint8_t *data, size_t len)
{
    return smb_read_block(command, data, len);
}

esp_err_t tps546_read_smbalert_mask(uint8_t selector, uint8_t *value)
{
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is null");
    ESP_RETURN_ON_ERROR(smb_write_block(PMBUS_SMBALERT_MASK, &selector, 1), TAG, "select SMBALERT_MASK failed");
    ESP_RETURN_ON_ERROR(smb_read_block(PMBUS_SMBALERT_MASK, value, 1), TAG, "read SMBALERT_MASK failed");
    return ESP_OK;
}

esp_err_t tps546_configure_birds(void)
{
    return tps546_write_birds_config();
}

esp_err_t tps546_bringup_birds(void)
{
    ESP_RETURN_ON_ERROR(tps546_probe(NULL), TAG, "probe failed");
    ESP_RETURN_ON_ERROR(tps546_init_disable(), TAG, "init disable failed");
    ESP_RETURN_ON_ERROR(tps546_clear_faults(), TAG, "clear faults failed");
    ESP_RETURN_ON_ERROR(tps546_write_birds_config(), TAG, "write birds config failed");
    ESP_RETURN_ON_ERROR(tps546_set_enable_pin(true), TAG, "assert enable pin failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(tps546_set_operation(true), TAG, "set operation on failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

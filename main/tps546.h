#ifndef BONANZA_TEST_TPS546_H_
#define BONANZA_TEST_TPS546_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define TPS546_I2C_ADDRESS 0x24

typedef struct {
    uint8_t id[6];
    const char *name;
} tps546_device_id_t;

typedef struct {
    uint16_t status_word;
    uint8_t status_vout;
    uint8_t status_iout;
    uint8_t status_input;
    uint8_t status_temperature;
    uint8_t status_cml;
    uint8_t status_other;
    uint8_t status_mfr_specific;
    uint8_t operation;
    uint8_t on_off_config;
    bool enable_pin_high;
    bool pgood_pin_high;
} tps546_status_snapshot_t;

typedef struct {
    uint8_t vout_mode;
    float read_vin;
    float read_vout;
    float read_iout;
    float read_temperature_c;
} tps546_telemetry_t;

esp_err_t tps546_init(void);
esp_err_t tps546_probe(tps546_device_id_t *device);
bool tps546_get_pgood_pin(void);
bool tps546_get_enable_pin(void);
esp_err_t tps546_read_status(tps546_status_snapshot_t *snapshot);
esp_err_t tps546_read_telemetry(tps546_telemetry_t *telemetry);
esp_err_t tps546_clear_faults(void);
esp_err_t tps546_set_enable_pin(bool enabled);
esp_err_t tps546_set_operation(bool enabled);
esp_err_t tps546_read_byte_reg(uint8_t command, uint8_t *value);
esp_err_t tps546_read_word_reg(uint8_t command, uint16_t *value);
esp_err_t tps546_read_block_reg(uint8_t command, uint8_t *data, size_t len);
esp_err_t tps546_read_smbalert_mask(uint8_t selector, uint8_t *value);
esp_err_t tps546_configure_birds(void);
esp_err_t tps546_bringup_birds(void);

#endif

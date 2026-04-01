#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "pmbus_commands.h"
#include "tps546.h"

#define TAG "bonanza-test"

#define CONTROL_UART_PORT UART_NUM_0
#define DATA_UART_PORT UART_NUM_1

// Bonanza board wiring provided during hardware bring-up.
#define CONTROL_UART_TX_PIN GPIO_NUM_43
#define CONTROL_UART_RX_PIN GPIO_NUM_44
#define DATA_UART_TX_PIN GPIO_NUM_17
#define DATA_UART_RX_PIN GPIO_NUM_18

#define CONTROL_UART_BAUDRATE 115200
#define DATA_UART_BAUDRATE 5000000

#define CONTROL_UART_BUF_SIZE 1024
#define DATA_UART_BUF_SIZE 4096
#define CONTROL_RESPONSE_MAX 260
#define CONSOLE_LINE_MAX 256
#define BZM_MAX_WRITE_ARGS 68
#define BZM_MAX_WRITE_WORDS 80
#define BZM_MAX_READ_WORDS 258

#define CONTROL_PAGE_GPIO 0x06
#define CONTROL_PAGE_FAN 0x09

#define GPIO_CMD_5V_EN 0x01
#define GPIO_CMD_ASIC_RST 0x02
#define GPIO_CMD_ASIC_TRIP 0x03

#define FAN_CMD_SET_SPEED 0x10
#define FAN_CMD_GET_TACH 0x20

typedef struct {
    uint8_t id;
    size_t len;
    uint8_t payload[CONTROL_RESPONSE_MAX];
} control_response_t;

static uint8_t s_next_command_id = 1;
static SemaphoreHandle_t s_data_uart_mutex;
// Shared scratch keeps the interactive main task stack small.
static uint8_t s_control_framed_response[CONTROL_RESPONSE_MAX];
static uint8_t s_data_encoded_buf[DATA_UART_BUF_SIZE];
static uint16_t s_bzm_tx_words[BZM_MAX_WRITE_WORDS];
static uint16_t s_bzm_rx_words[BZM_MAX_READ_WORDS];
static uint32_t s_bzm_parsed[BZM_MAX_WRITE_ARGS];

static size_t parse_hex_sequence(char *args, uint32_t *values, size_t max_values, uint32_t max_allowed);

static void hexdump_bytes(const char *prefix, const uint8_t *data, size_t len)
{
    char line[256];
    int offset = snprintf(line, sizeof(line), "%s (%u bytes):", prefix, (unsigned)len);

    for (size_t i = 0; i < len && offset > 0 && offset < (int)sizeof(line); i++) {
        offset += snprintf(line + offset, sizeof(line) - (size_t)offset, " %02X", data[i]);
    }

    ESP_LOGI(TAG, "%s", line);
}

static void print_help(void)
{
    printf("\n");
    printf("bonanza-test commands:\n");
    printf("  help              Show this help\n");
    printf("  gpio              Read 5v_en, asic_rst, asic_trip\n");
    printf("  5v <0|1>          Set 5V switch control\n");
    printf("  rst <0|1>         Set ASIC reset control\n");
    printf("  fan <0-100>       Set fan PWM percentage\n");
    printf("  tach              Read fan tach RPM\n");
    printf("  vr help           Show TPS546D24S commands\n");
    printf("  vr probe          Read TPS546D24S device ID over I2C\n");
    printf("  vr pgood          Read VR PGOOD pin and enable pin state\n");
    printf("  vr status         Read TPS546 status/fault registers\n");
    printf("  vr telem          Read TPS546 telemetry\n");
    printf("  vr clear          Clear TPS546 faults\n");
    printf("  vr cfg read       Read back key TPS546 config registers\n");
    printf("  vr cfg write      Write TPS546_CONFIG_BIRDS PMBus settings\n");
    printf("  vr bringup        Python-style BIRDS bring-up: off, clear, cfg, enable, on\n");
    printf("  vr pin <0|1>      Drive the VR enable pin on GPIO10\n");
    printf("  vr op <0|1>       Write PMBus OPERATION off/on\n");
    printf("  BZM_sendnoop <asic>\n");
    printf("                    Send BZM NOOP to ASIC, ex: BZM_sendnoop 0xFA\n");
    printf("  BZM_readreg <asic> <engine> <offset> <count>\n");
    printf("                    Read BZM register bytes, ex: BZM_readreg 0xFA 0xFFF 0x0B 4\n");
    printf("  BZM_writereg <asic> <engine> <offset> <byte...>\n");
    printf("                    Write BZM register bytes, ex: BZM_writereg 0xFA 0xFFF 0x0B 0x42 0x00 0x00 0x00\n");
    printf("  pattern           Send default 9-bit test pattern on data UART\n");
    printf("  send9 <...>       Send one or more 9-bit words, ex: send9 0x155 0x0aa\n");
    printf("  raw <...>         Send raw byte stream on data UART, ex: raw 55 01 aa 00\n");
    printf("  flush             Flush pending data UART RX bytes\n");
    printf("\n");
    printf("Control UART: UART0 TX GPIO43, RX GPIO44 @ 115200\n");
    printf("Data UART:    UART1 TX GPIO17, RX GPIO18 @ 5000000\n");
    printf("VR I2C:       SDA GPIO47, SCL GPIO48, EN GPIO10, PGOOD GPIO11\n");
    printf("\n");
}

static esp_err_t init_console_io(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&config), TAG, "usb_serial_jtag driver install failed");
    }

    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_vfs_use_driver();

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
    fcntl(fileno(stdout), F_SETFL, 0);

    return ESP_OK;
}

static esp_err_t uart_init_port(uart_port_t port, int tx_pin, int rx_pin, int baudrate, int buf_size)
{
    const uart_config_t config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(port, &config));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port, buf_size, buf_size, 0, NULL, 0));
    return ESP_OK;
}

static esp_err_t init_interfaces(void)
{
    ESP_ERROR_CHECK(uart_init_port(CONTROL_UART_PORT, CONTROL_UART_TX_PIN, CONTROL_UART_RX_PIN, CONTROL_UART_BAUDRATE, CONTROL_UART_BUF_SIZE));
    ESP_ERROR_CHECK(uart_init_port(DATA_UART_PORT, DATA_UART_TX_PIN, DATA_UART_RX_PIN, DATA_UART_BAUDRATE, DATA_UART_BUF_SIZE));
    s_data_uart_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_data_uart_mutex != NULL, ESP_ERR_NO_MEM, TAG, "data uart mutex alloc failed");
    ESP_ERROR_CHECK(tps546_init());
    return ESP_OK;
}

static esp_err_t uart_read_exact(uart_port_t port, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t offset = 0;
    const int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);

    while (offset < len) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        TickType_t timeout_ticks = pdMS_TO_TICKS((remaining_us + 999) / 1000);
        if (timeout_ticks == 0) {
            timeout_ticks = 1;
        }

        int read = uart_read_bytes(port, buf + offset, len - offset, timeout_ticks);
        if (read < 0) {
            return ESP_FAIL;
        }

        if (read == 0) {
            continue;
        }

        offset += (size_t)read;
    }

    return ESP_OK;
}

static esp_err_t control_send_command(uint8_t page, uint8_t command, const uint8_t *payload, size_t payload_len, control_response_t *response, uint32_t timeout_ms)
{
    uint8_t request[64];
    uint8_t header[3];
    uint16_t response_len;

    if ((payload_len + 6) > sizeof(request)) {
        return ESP_ERR_INVALID_SIZE;
    }

    response->id = s_next_command_id++;
    request[0] = (uint8_t)(payload_len + 6);
    request[1] = 0x00;
    request[2] = response->id;
    request[3] = 0x00;
    request[4] = page;
    request[5] = command;

    if (payload_len > 0) {
        memcpy(&request[6], payload, payload_len);
    }

    uart_flush_input(CONTROL_UART_PORT);
    hexdump_bytes("CTRL TX", request, payload_len + 6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(CONTROL_UART_PORT, pdMS_TO_TICKS(20)));

    int written = uart_write_bytes(CONTROL_UART_PORT, request, payload_len + 6);
    if (written != (int)(payload_len + 6)) {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(CONTROL_UART_PORT, pdMS_TO_TICKS(100)));

    ESP_RETURN_ON_ERROR(uart_read_exact(CONTROL_UART_PORT, header, sizeof(header), timeout_ms), TAG, "control response header timeout");

    response_len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    if (response_len < 3 || response_len > CONTROL_RESPONSE_MAX) {
        ESP_LOGE(TAG, "invalid control response length: %u", response_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    response->id = header[2];
    response->len = response_len - 3;

    ESP_RETURN_ON_ERROR(uart_read_exact(CONTROL_UART_PORT, response->payload, response->len, timeout_ms), TAG, "control response payload timeout");

    if (response->id != request[2]) {
        ESP_LOGW(TAG, "response id mismatch: sent=%u got=%u", request[2], response->id);
    }

    {
        size_t framed_len = response->len + 3;

        s_control_framed_response[0] = header[0];
        s_control_framed_response[1] = header[1];
        s_control_framed_response[2] = header[2];
        memcpy(&s_control_framed_response[3], response->payload, response->len);
        hexdump_bytes("CTRL RX", s_control_framed_response, framed_len);
    }

    return ESP_OK;
}

static esp_err_t control_get_byte(uint8_t page, uint8_t command, uint8_t *value)
{
    control_response_t response;
    ESP_RETURN_ON_ERROR(control_send_command(page, command, NULL, 0, &response, 200), TAG, "control get failed");

    if (response.len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *value = response.payload[0];
    return ESP_OK;
}

static esp_err_t control_set_byte(uint8_t page, uint8_t command, uint8_t value, uint8_t *echoed)
{
    control_response_t response;
    ESP_RETURN_ON_ERROR(control_send_command(page, command, &value, 1, &response, 200), TAG, "control set failed");

    if (response.len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *echoed = response.payload[0];
    return ESP_OK;
}

static esp_err_t control_get_u16(uint8_t page, uint8_t command, uint16_t *value)
{
    control_response_t response;
    ESP_RETURN_ON_ERROR(control_send_command(page, command, NULL, 0, &response, 1000), TAG, "control read failed");

    if (response.len < 2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *value = (uint16_t)response.payload[0] | ((uint16_t)response.payload[1] << 8);
    return ESP_OK;
}

static void cmd_gpio_status(void)
{
    uint8_t v5_en = 0;
    uint8_t asic_rst = 0;
    uint8_t asic_trip = 0;

    if (control_get_byte(CONTROL_PAGE_GPIO, GPIO_CMD_5V_EN, &v5_en) != ESP_OK) {
        printf("failed to read 5v_en\n");
        return;
    }

    if (control_get_byte(CONTROL_PAGE_GPIO, GPIO_CMD_ASIC_RST, &asic_rst) != ESP_OK) {
        printf("failed to read asic_rst\n");
        return;
    }

    if (control_get_byte(CONTROL_PAGE_GPIO, GPIO_CMD_ASIC_TRIP, &asic_trip) != ESP_OK) {
        printf("failed to read asic_trip\n");
        return;
    }

    printf("5v_en=%u asic_rst=%u asic_trip=%u\n", v5_en, asic_rst, asic_trip);
}

static void cmd_set_5v(int level)
{
    uint8_t echoed = 0;
    if (control_set_byte(CONTROL_PAGE_GPIO, GPIO_CMD_5V_EN, (uint8_t)(level ? 1 : 0), &echoed) == ESP_OK) {
        printf("5v_en set -> %u\n", echoed);
    } else {
        printf("failed to set 5v_en\n");
    }
}

static void cmd_set_rst(int level)
{
    uint8_t echoed = 0;
    if (control_set_byte(CONTROL_PAGE_GPIO, GPIO_CMD_ASIC_RST, (uint8_t)(level ? 1 : 0), &echoed) == ESP_OK) {
        printf("asic_rst set -> %u\n", echoed);
    } else {
        printf("failed to set asic_rst\n");
    }
}

static void cmd_set_fan(int speed)
{
    uint8_t echoed = 0;
    if (speed < 0 || speed > 100) {
        printf("fan speed must be 0-100\n");
        return;
    }

    if (control_set_byte(CONTROL_PAGE_FAN, FAN_CMD_SET_SPEED, (uint8_t)speed, &echoed) == ESP_OK) {
        printf("fan command sent, firmware returned 0x%02X\n", echoed);
    } else {
        printf("failed to set fan speed\n");
    }
}

static void cmd_get_tach(void)
{
    uint16_t rpm = 0;
    if (control_get_u16(CONTROL_PAGE_FAN, FAN_CMD_GET_TACH, &rpm) == ESP_OK) {
        printf("fan tach: %u RPM\n", rpm);
    } else {
        printf("failed to read fan tach\n");
    }
}

static void print_vr_help(void)
{
    printf("vr commands:\n");
    printf("  vr probe          Read device ID from I2C address 0x%02X\n", TPS546_I2C_ADDRESS);
    printf("  vr pgood          Read PGOOD and VR enable GPIO state\n");
    printf("  vr status         Read PMBus status registers and control state\n");
    printf("  vr telem          Read VIN/VOUT/IOUT/TEMP telemetry\n");
    printf("  vr clear          Clear sticky PMBus faults\n");
    printf("  vr cfg read       Read back key TPS546 config registers\n");
    printf("  vr cfg write      Write TPS546_CONFIG_BIRDS register set\n");
    printf("  vr bringup        Run Python-style TPS546_CONFIG_BIRDS bring-up\n");
    printf("  vr pin <0|1>      Drive GPIO10 low/high\n");
    printf("  vr op <0|1>       Write PMBus OPERATION off/on\n");
}

static void print_status_word_decode(uint16_t status_word)
{
    printf("[status_word_bits]\n");
    printf("VOUT_FAULT=%s\n", (status_word & TPS546_STATUS_VOUT) ? "set: see STATUS_VOUT" : "clear");
    printf("IOUT_FAULT=%s\n", (status_word & TPS546_STATUS_IOUT) ? "set: see STATUS_IOUT" : "clear");
    printf("INPUT_FAULT=%s\n", (status_word & TPS546_STATUS_INPUT) ? "set: see STATUS_INPUT" : "clear");
    printf("MFR_FAULT=%s\n", (status_word & TPS546_STATUS_MFR) ? "set: see STATUS_MFR_SPECIFIC" : "clear");
    printf("PGOOD_STATUS=%s\n", (status_word & TPS546_STATUS_PGOOD) ? "fault: output not in regulation" : "ok: output in regulation");
    printf("OTHER_FAULT=%s\n", (status_word & TPS546_STATUS_OTHER) ? "set: see STATUS_OTHER" : "clear");
    printf("BUSY=%s\n", (status_word & TPS546_STATUS_BUSY) ? "set: device busy" : "clear: ready");
    printf("OFF=%s\n", (status_word & TPS546_STATUS_OFF) ? "set: not converting" : "clear: enabled/converting");
    printf("VOUT_OV=%s\n", (status_word & TPS546_STATUS_VOUT_OV) ? "set" : "clear");
    printf("IOUT_OC=%s\n", (status_word & TPS546_STATUS_IOUT_OC) ? "set" : "clear");
    printf("VIN_UV=%s\n", (status_word & TPS546_STATUS_VIN_UV) ? "set" : "clear");
    printf("TEMP=%s\n", (status_word & TPS546_STATUS_TEMP) ? "set" : "clear");
    printf("CML=%s\n", (status_word & TPS546_STATUS_CML) ? "set" : "clear");
    printf("NONE_OF_THE_ABOVE=%s\n", (status_word & TPS546_STATUS_NONE) ? "set" : "clear");
}

static float decode_slinear11(uint16_t value)
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

static float decode_ulinear16(uint16_t value, uint8_t vout_mode)
{
    int exponent;

    if (vout_mode & 0x10) {
        exponent = -1 * ((((int)(~vout_mode)) & 0x1F) + 1);
    } else {
        exponent = (int)(vout_mode & 0x1F);
    }

    return (float)value * powf(2.0f, (float)exponent);
}

static void print_hex_array(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printf("%s%02X", i == 0 ? "" : " ", data[i]);
    }
}

static void cmd_vr_probe(void)
{
    tps546_device_id_t device;
    esp_err_t err = tps546_probe(&device);

    if (err != ESP_OK) {
        printf("vr probe failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("TPS546 device ID:");
    for (size_t i = 0; i < sizeof(device.id); i++) {
        printf(" %02X", device.id[i]);
    }
    printf(" -> %s\n", device.name);
}

static void cmd_vr_pgood(void)
{
    printf("vr enable_pin=%u pgood=%u\n", tps546_get_enable_pin() ? 1 : 0, tps546_get_pgood_pin() ? 1 : 0);
}

static void cmd_vr_status(void)
{
    tps546_status_snapshot_t status;
    esp_err_t err = tps546_read_status(&status);

    if (err != ESP_OK) {
        printf("vr status failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("[status_raw]\n");
    printf("STATUS_WORD=0x%04X\n", status.status_word);
    printf("STATUS_VOUT=0x%02X\n", status.status_vout);
    printf("STATUS_IOUT=0x%02X\n", status.status_iout);
    printf("STATUS_INPUT=0x%02X\n", status.status_input);
    printf("STATUS_TEMPERATURE=0x%02X\n", status.status_temperature);
    printf("STATUS_CML=0x%02X\n", status.status_cml);
    printf("STATUS_OTHER=0x%02X\n", status.status_other);
    printf("STATUS_MFR_SPECIFIC=0x%02X\n", status.status_mfr_specific);

    printf("\n[control]\n");
    printf("OPERATION=0x%02X\n", status.operation);
    printf("ON_OFF_CONFIG=0x%02X\n", status.on_off_config);

    printf("\n[pins]\n");
    printf("ENABLE_PIN=%u\n", status.enable_pin_high ? 1 : 0);
    printf("PGOOD_PIN=%u\n", status.pgood_pin_high ? 1 : 0);

    printf("\n");
    print_status_word_decode(status.status_word);
}

static void cmd_vr_telem(void)
{
    tps546_telemetry_t telemetry;
    esp_err_t err = tps546_read_telemetry(&telemetry);

    if (err != ESP_OK) {
        printf("vr telem failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("VOUT_MODE=0x%02X VIN=%.3fV VOUT=%.3fV IOUT=%.3fA TEMP=%.1fC\n",
           telemetry.vout_mode,
           telemetry.read_vin,
           telemetry.read_vout,
           telemetry.read_iout,
           telemetry.read_temperature_c);
}

static void cmd_vr_clear(void)
{
    esp_err_t err = tps546_clear_faults();
    if (err != ESP_OK) {
        printf("vr clear failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("vr faults cleared\n");
}

static void cmd_vr_cfg_read(void)
{
    uint8_t vout_mode = 0;
    uint8_t phase = 0;
    uint8_t sync_config = 0;
    uint8_t device_address = 0;
    uint8_t power_stage_config[1] = {0};
    uint8_t telemetry_config[6] = {0};
    uint8_t compensation_config[5] = {0};
    uint8_t smbalert_vout = 0;
    uint8_t smbalert_iout = 0;
    uint8_t smbalert_input = 0;
    uint8_t smbalert_temp = 0;
    uint8_t smbalert_cml = 0;
    uint8_t smbalert_other = 0;
    uint8_t smbalert_mfr = 0;
    uint16_t word = 0;
    esp_err_t err;

    err = tps546_read_byte_reg(PMBUS_VOUT_MODE, &vout_mode);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = tps546_read_byte_reg(PMBUS_PHASE, &phase);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_VOUT_SELECTOR, &smbalert_vout);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_IOUT_SELECTOR, &smbalert_iout);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_INPUT_SELECTOR, &smbalert_input);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_TEMPERATURE_SELECTOR, &smbalert_temp);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_CML_SELECTOR, &smbalert_cml);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_OTHER_SELECTOR, &smbalert_other);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_smbalert_mask(PMBUS_STATUS_MFR_SPECIFIC_SELECTOR, &smbalert_mfr);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = tps546_read_word_reg(PMBUS_FREQUENCY_SWITCH, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("[core]\n");
    printf("PHASE=0x%02X\n", phase);
    printf("VOUT_MODE=0x%02X\n", vout_mode);
    printf("FREQUENCY_SWITCH=0x%04X (%.0fkHz)\n", word, decode_slinear11(word));
    printf("SMBALERT_MASK_VOUT=0x%02X\n", smbalert_vout);
    printf("SMBALERT_MASK_IOUT=0x%02X\n", smbalert_iout);
    printf("SMBALERT_MASK_INPUT=0x%02X\n", smbalert_input);
    printf("SMBALERT_MASK_TEMP=0x%02X\n", smbalert_temp);
    printf("SMBALERT_MASK_CML=0x%02X\n", smbalert_cml);
    printf("SMBALERT_MASK_OTHER=0x%02X\n", smbalert_other);
    printf("SMBALERT_MASK_MFR=0x%02X\n", smbalert_mfr);

    err = tps546_read_byte_reg(PMBUS_SYNC_CONFIG, &sync_config);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_word_reg(PMBUS_STACK_CONFIG, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[topology]\n");
    printf("SYNC_CONFIG=0x%02X\n", sync_config);
    printf("STACK_CONFIG=0x%04X\n", word);
    err = tps546_read_word_reg(PMBUS_INTERLEAVE, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("INTERLEAVE=0x%04X\n", word);
    err = tps546_read_word_reg(PMBUS_MISC_OPTIONS, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("MISC_OPTIONS=0x%04X\n", word);
    err = tps546_read_word_reg(PMBUS_PIN_DETECT_OVERRIDE, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("PIN_DETECT_OVERRIDE=0x%04X\n", word);

    err = tps546_read_byte_reg(PMBUS_SLAVE_ADDRESS, &device_address);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_block_reg(PMBUS_COMPENSATION_CONFIG, compensation_config, sizeof(compensation_config));
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_block_reg(PMBUS_POWER_STAGE_CONFIG, power_stage_config, sizeof(power_stage_config));
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = tps546_read_block_reg(PMBUS_TELEMETRY_CONFIG, telemetry_config, sizeof(telemetry_config));
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[device]\n");
    printf("SLAVE_ADDRESS=0x%02X\n", device_address);
    printf("COMPENSATION_CONFIG=[");
    print_hex_array(compensation_config, sizeof(compensation_config));
    printf("]\n");
    printf("POWER_STAGE_CONFIG=0x%02X\n", power_stage_config[0]);
    printf("TELEMETRY_CONFIG=[");
    print_hex_array(telemetry_config, sizeof(telemetry_config));
    printf("]\n");

    err = tps546_read_word_reg(PMBUS_VOUT_COMMAND, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[vout]\n");
    printf("VOUT_COMMAND=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_word_reg(PMBUS_VOUT_TRIM, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_TRIM=0x%04X\n", word);
    err = tps546_read_word_reg(PMBUS_VOUT_MAX, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_MAX=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));

    err = tps546_read_word_reg(PMBUS_VOUT_MARGIN_HIGH, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_MARGIN_HIGH=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_word_reg(PMBUS_VOUT_MARGIN_LOW, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_MARGIN_LOW=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_word_reg(PMBUS_VOUT_TRANSITION_RATE, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_TRANSITION_RATE=0x%04X\n", word);

    err = tps546_read_word_reg(PMBUS_VOUT_SCALE_LOOP, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_SCALE_LOOP=0x%04X (%.3f)\n", word, decode_slinear11(word));
    err = tps546_read_word_reg(PMBUS_VOUT_MIN, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_MIN=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));

    err = tps546_read_word_reg(PMBUS_VIN_ON, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[vin]\n");
    printf("VIN_ON=0x%04X (%.3fV)\n", word, decode_slinear11(word));
    err = tps546_read_word_reg(PMBUS_VIN_OFF, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VIN_OFF=0x%04X (%.3fV)\n", word, decode_slinear11(word));

    err = tps546_read_word_reg(PMBUS_IOUT_CAL_GAIN, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[calibration]\n");
    printf("IOUT_CAL_GAIN=0x%04X\n", word);
    err = tps546_read_word_reg(PMBUS_IOUT_CAL_OFFSET, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("IOUT_CAL_OFFSET=0x%04X\n", word);

    err = tps546_read_word_reg(PMBUS_VOUT_OV_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[faults_vout]\n");
    printf("VOUT_OV_FAULT_LIMIT=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_byte_reg(PMBUS_VOUT_OV_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_OV_FAULT_RESPONSE=0x%02X\n", phase);

    err = tps546_read_word_reg(PMBUS_VOUT_OV_WARN_LIMIT, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_OV_WARN_LIMIT=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_word_reg(PMBUS_VOUT_UV_WARN_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_UV_WARN_LIMIT=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));
    err = tps546_read_word_reg(PMBUS_VOUT_UV_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_UV_FAULT_LIMIT=0x%04X (%.3f)\n", word, decode_ulinear16(word, vout_mode));

    err = tps546_read_byte_reg(PMBUS_VOUT_UV_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VOUT_UV_FAULT_RESPONSE=0x%02X\n", phase);

    printf("\n[faults_iout]\n");
    err = tps546_read_word_reg(PMBUS_IOUT_OC_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("IOUT_OC_FAULT_LIMIT=0x%04X (%.3fA)\n", word, decode_slinear11(word));

    err = tps546_read_byte_reg(PMBUS_IOUT_OC_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("IOUT_OC_FAULT_RESPONSE=0x%02X\n", phase);
    err = tps546_read_word_reg(PMBUS_IOUT_OC_WARN_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("IOUT_OC_WARN_LIMIT=0x%04X (%.3fA)\n", word, decode_slinear11(word));

    err = tps546_read_word_reg(PMBUS_OT_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[faults_temp]\n");
    printf("OT_FAULT_LIMIT=0x%04X (%.1fC)\n", word, decode_slinear11(word));
    err = tps546_read_byte_reg(PMBUS_OT_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("OT_FAULT_RESPONSE=0x%02X\n", phase);
    err = tps546_read_word_reg(PMBUS_OT_WARN_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("OT_WARN_LIMIT=0x%04X (%.1fC)\n", word, decode_slinear11(word));

    err = tps546_read_word_reg(PMBUS_VIN_OV_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[faults_vin]\n");
    printf("VIN_OV_FAULT_LIMIT=0x%04X (%.3fV)\n", word, decode_slinear11(word));
    err = tps546_read_byte_reg(PMBUS_VIN_OV_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VIN_OV_FAULT_RESPONSE=0x%02X\n", phase);
    err = tps546_read_word_reg(PMBUS_VIN_UV_WARN_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("VIN_UV_WARN_LIMIT=0x%04X (%.3fV)\n", word, decode_slinear11(word));

    err = tps546_read_word_reg(PMBUS_TON_DELAY, &word);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("\n[timing]\n");
    printf("TON_DELAY=0x%04X (%.1fms)\n", word, decode_slinear11(word));
    err = tps546_read_word_reg(PMBUS_TON_RISE, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("TON_RISE=0x%04X (%.1fms)\n", word, decode_slinear11(word));
    err = tps546_read_word_reg(PMBUS_TON_MAX_FAULT_LIMIT, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("TON_MAX_FAULT_LIMIT=0x%04X (%.1fms)\n", word, decode_slinear11(word));

    err = tps546_read_byte_reg(PMBUS_TON_MAX_FAULT_RESPONSE, &phase);
    if (err != ESP_OK) {
        printf("vr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("TON_MAX_FAULT_RESPONSE=0x%02X\n", phase);
    err = tps546_read_word_reg(PMBUS_TOFF_DELAY, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("TOFF_DELAY=0x%04X (%.1fms)\n", word, decode_slinear11(word));
    err = tps546_read_word_reg(PMBUS_TOFF_FALL, &word);
    if (err != ESP_OK) {
        printf("\nvr cfg read failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("TOFF_FALL=0x%04X (%.1fms)\n", word, decode_slinear11(word));
}

static void cmd_vr_cfg_write(void)
{
    esp_err_t err = tps546_configure_birds();
    if (err != ESP_OK) {
        printf("vr cfg write failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("vr cfg write: TPS546_CONFIG_BIRDS applied\n");
}

static void cmd_vr_bringup(void)
{
    esp_err_t err = tps546_bringup_birds();
    if (err != ESP_OK) {
        printf("vr bringup failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("vr bringup complete\n");
    cmd_vr_pgood();
    cmd_vr_status();
    cmd_vr_telem();
}

static void cmd_vr_pin(int level)
{
    esp_err_t err;

    if (level != 0 && level != 1) {
        printf("usage: vr pin <0|1>\n");
        return;
    }

    err = tps546_set_enable_pin(level != 0);
    if (err != ESP_OK) {
        printf("vr pin failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("vr enable pin -> %d\n", level);
}

static void cmd_vr_operation(int level)
{
    esp_err_t err;

    if (level != 0 && level != 1) {
        printf("usage: vr op <0|1>\n");
        return;
    }

    err = tps546_set_operation(level != 0);
    if (err != ESP_OK) {
        printf("vr op failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("vr operation -> %s\n", level ? "ON" : "OFF");
}

static void cmd_vr(char *args)
{
    while (*args == ' ' || *args == '\t') {
        args++;
    }

    if (*args == '\0' || strcmp(args, "help") == 0) {
        print_vr_help();
        return;
    }

    if (strcmp(args, "probe") == 0) {
        cmd_vr_probe();
        return;
    }

    if (strcmp(args, "pgood") == 0) {
        cmd_vr_pgood();
        return;
    }

    if (strcmp(args, "status") == 0) {
        cmd_vr_status();
        return;
    }

    if (strcmp(args, "telem") == 0) {
        cmd_vr_telem();
        return;
    }

    if (strcmp(args, "clear") == 0) {
        cmd_vr_clear();
        return;
    }

    if (strcmp(args, "cfg read") == 0) {
        cmd_vr_cfg_read();
        return;
    }

    if (strcmp(args, "cfg write") == 0) {
        cmd_vr_cfg_write();
        return;
    }

    if (strcmp(args, "bringup") == 0) {
        cmd_vr_bringup();
        return;
    }

    if (strncmp(args, "pin ", 4) == 0) {
        cmd_vr_pin(atoi(&args[4]));
        return;
    }

    if (strncmp(args, "op ", 3) == 0) {
        cmd_vr_operation(atoi(&args[3]));
        return;
    }

    printf("unknown vr command: %s\n", args);
}

static void data_send_bytes(const uint8_t *bytes, size_t len)
{
    if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("data uart busy\n");
        return;
    }

    hexdump_bytes("DATA TX", bytes, len);
    uart_write_bytes(DATA_UART_PORT, bytes, len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(DATA_UART_PORT, pdMS_TO_TICKS(100)));
    xSemaphoreGive(s_data_uart_mutex);
}

static void data_send_words(const uint16_t *words, size_t count)
{
    uint8_t encoded[128];
    size_t encoded_len = 0;

    for (size_t i = 0; i < count; i++) {
        if (encoded_len + 2 > sizeof(encoded)) {
            break;
        }

        encoded[encoded_len++] = (uint8_t)(words[i] & 0xff);
        encoded[encoded_len++] = (uint8_t)((words[i] >> 8) & 0x01);
    }

    data_send_bytes(encoded, encoded_len);
}

static void cmd_pattern(void)
{
    const uint16_t pattern[] = {0x155, 0x0AA, 0x1FE, 0x000, 0x1FF};
    data_send_words(pattern, sizeof(pattern) / sizeof(pattern[0]));
}

static void print_data_words(const char *prefix, const uint16_t *words, size_t count)
{
    printf("%s (%u words):", prefix, (unsigned)count);
    for (size_t i = 0; i < count; i++) {
        printf(" 0x%03X", words[i]);
    }
    printf("\n");
}

static esp_err_t data_write_words_locked(const uint16_t *words, size_t count)
{
    size_t encoded_len = 0;

    if ((count * 2) > sizeof(s_data_encoded_buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < count; i++) {
        s_data_encoded_buf[encoded_len++] = (uint8_t)(words[i] & 0xff);
        s_data_encoded_buf[encoded_len++] = (uint8_t)((words[i] >> 8) & 0x01);
    }

    hexdump_bytes("DATA TX", s_data_encoded_buf, encoded_len);
    print_data_words("DATA TX9", words, count);

    int written = uart_write_bytes(DATA_UART_PORT, s_data_encoded_buf, encoded_len);
    if (written != (int)encoded_len) {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(DATA_UART_PORT, pdMS_TO_TICKS(100)));
    return ESP_OK;
}

static esp_err_t data_read_words_exact_locked(uint16_t *words, size_t count, uint32_t timeout_ms)
{
    if ((count * 2) > sizeof(s_data_encoded_buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_RETURN_ON_ERROR(uart_read_exact(DATA_UART_PORT, s_data_encoded_buf, count * 2, timeout_ms), TAG, "data response timeout");
    hexdump_bytes("DATA RX", s_data_encoded_buf, count * 2);

    for (size_t i = 0; i < count; i++) {
        words[i] = (uint16_t)s_data_encoded_buf[i * 2] | (((uint16_t)s_data_encoded_buf[i * 2 + 1] & 0x01) << 8);
    }

    return ESP_OK;
}

static esp_err_t data_read_words_available_locked(uint16_t *words, size_t max_words, size_t *actual_words, uint32_t idle_timeout_ms)
{
    size_t byte_count = 0;
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)idle_timeout_ms * 1000);

    if ((max_words * 2) > sizeof(s_data_encoded_buf)) {
        return ESP_ERR_INVALID_SIZE;
    }

    while (byte_count < (max_words * 2)) {
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        int chunk;

        if (remaining_us <= 0) {
            break;
        }

        TickType_t timeout_ticks = pdMS_TO_TICKS((remaining_us + 999) / 1000);
        if (timeout_ticks == 0) {
            timeout_ticks = 1;
        }

        chunk = uart_read_bytes(DATA_UART_PORT, &s_data_encoded_buf[byte_count], (max_words * 2) - byte_count, timeout_ticks);
        if (chunk < 0) {
            return ESP_FAIL;
        }

        if (chunk == 0) {
            break;
        }

        byte_count += (size_t)chunk;
        deadline_us = esp_timer_get_time() + ((int64_t)idle_timeout_ms * 1000);
    }

    if ((byte_count % 2) != 0) {
        printf("warning: odd response byte count %u, dropping last byte\n", (unsigned)byte_count);
        byte_count--;
    }

    if (byte_count > 0) {
        hexdump_bytes("DATA RX", s_data_encoded_buf, byte_count);
    }

    *actual_words = byte_count / 2;
    for (size_t i = 0; i < *actual_words; i++) {
        words[i] = (uint16_t)s_data_encoded_buf[i * 2] | (((uint16_t)s_data_encoded_buf[i * 2 + 1] & 0x01) << 8);
    }

    return ESP_OK;
}

static void cmd_bzm_sendnoop(char *args)
{
    uint32_t parsed[1];
    size_t rx_count = 0;
    esp_err_t err;

    if (parse_hex_sequence(args, parsed, 1, 0xff) != 1) {
        printf("usage: BZM_sendnoop <asic>\n");
        return;
    }

    if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        printf("data uart busy\n");
        return;
    }

    uart_flush_input(DATA_UART_PORT);
    s_bzm_tx_words[0] = 0x0100 | (uint16_t)parsed[0];
    s_bzm_tx_words[1] = 0x00F0;

    err = data_write_words_locked(s_bzm_tx_words, 2);
    if (err == ESP_OK) {
        err = data_read_words_available_locked(s_bzm_rx_words, 8, &rx_count, 100);
    }

    xSemaphoreGive(s_data_uart_mutex);

    if (err != ESP_OK) {
        printf("BZM_sendnoop failed: %s\n", esp_err_to_name(err));
        return;
    }

    if (rx_count == 0) {
        printf("BZM_sendnoop: no response\n");
        return;
    }

    print_data_words("BZM RX", s_bzm_rx_words, rx_count);
}

static void cmd_bzm_readreg(char *args)
{
    uint32_t parsed[4];
    esp_err_t err;
    size_t expected_words;

    if (parse_hex_sequence(args, parsed, 4, 0xfff) != 4 ||
        parsed[0] > 0xff || parsed[2] > 0xff || parsed[3] == 0 || parsed[3] > 0xff) {
        printf("usage: BZM_readreg <asic> <engine_id> <offset> <count>\n");
        return;
    }

    expected_words = (size_t)parsed[3] + 2;
    if (expected_words > (sizeof(s_bzm_rx_words) / sizeof(s_bzm_rx_words[0]))) {
        printf("count too large\n");
        return;
    }

    if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        printf("data uart busy\n");
        return;
    }

    uart_flush_input(DATA_UART_PORT);
    s_bzm_tx_words[0] = 0x0100 | (uint16_t)parsed[0];
    s_bzm_tx_words[1] = 0x0030 | (uint16_t)((parsed[1] & 0x0F00) >> 8);
    s_bzm_tx_words[2] = (uint16_t)(parsed[1] & 0x00FF);
    s_bzm_tx_words[3] = (uint16_t)parsed[2];
    s_bzm_tx_words[4] = (uint16_t)(parsed[3] - 1);
    s_bzm_tx_words[5] = 0x0000;

    err = data_write_words_locked(s_bzm_tx_words, 6);
    if (err == ESP_OK) {
        err = data_read_words_exact_locked(s_bzm_rx_words, expected_words, 500);
    }

    xSemaphoreGive(s_data_uart_mutex);

    if (err != ESP_OK) {
        printf("BZM_readreg failed: %s\n", esp_err_to_name(err));
        return;
    }

    print_data_words("BZM RX", s_bzm_rx_words, expected_words);
}

static void cmd_bzm_writereg(char *args)
{
    size_t parsed_count;
    size_t tx_count;
    size_t rx_count = 0;
    esp_err_t err;

    parsed_count = parse_hex_sequence(args, s_bzm_parsed, BZM_MAX_WRITE_ARGS, 0xfff);
    if (parsed_count < 4 || s_bzm_parsed[0] > 0xff || s_bzm_parsed[2] > 0xff) {
        printf("usage: BZM_writereg <asic> <engine_id> <offset> <byte...>\n");
        return;
    }

    for (size_t i = 3; i < parsed_count; i++) {
        if (s_bzm_parsed[i] > 0xff) {
            printf("write bytes must be 0x00-0xFF\n");
            return;
        }
    }

    s_bzm_tx_words[0] = 0x0100 | (uint16_t)s_bzm_parsed[0];
    s_bzm_tx_words[1] = 0x0020 | (uint16_t)((s_bzm_parsed[1] & 0x0F00) >> 8);
    s_bzm_tx_words[2] = (uint16_t)(s_bzm_parsed[1] & 0x00FF);
    s_bzm_tx_words[3] = (uint16_t)s_bzm_parsed[2];
    s_bzm_tx_words[4] = (uint16_t)((parsed_count - 3) - 1);
    tx_count = 5;

    for (size_t i = 3; i < parsed_count; i++) {
        s_bzm_tx_words[tx_count++] = (uint16_t)s_bzm_parsed[i];
    }
    s_bzm_tx_words[tx_count++] = 0x0000;

    if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        printf("data uart busy\n");
        return;
    }

    uart_flush_input(DATA_UART_PORT);
    err = data_write_words_locked(s_bzm_tx_words, tx_count);
    if (err == ESP_OK) {
        err = data_read_words_available_locked(s_bzm_rx_words, 32, &rx_count, 50);
    }

    xSemaphoreGive(s_data_uart_mutex);

    if (err != ESP_OK) {
        printf("BZM_writereg failed: %s\n", esp_err_to_name(err));
        return;
    }

    if (rx_count == 0) {
        printf("BZM_writereg sent\n");
        return;
    }

    print_data_words("BZM RX", s_bzm_rx_words, rx_count);
}

static size_t parse_hex_sequence(char *args, uint32_t *values, size_t max_values, uint32_t max_allowed)
{
    size_t count = 0;
    char *saveptr = NULL;
    char *token = strtok_r(args, " \t", &saveptr);

    while (token != NULL && count < max_values) {
        char *end = NULL;
        unsigned long parsed = strtoul(token, &end, 0);
        if (end == token || *end != '\0' || parsed > max_allowed) {
            return 0;
        }

        values[count++] = (uint32_t)parsed;
        token = strtok_r(NULL, " \t", &saveptr);
    }

    return count;
}

static void data_rx_task(void *arg)
{
    uint8_t buf[256];
    size_t buffered_len = 0;
    (void)arg;

    while (true) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_get_buffered_data_len(DATA_UART_PORT, &buffered_len));
        if (buffered_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int len = uart_read_bytes(DATA_UART_PORT, buf, sizeof(buf), 0);
        xSemaphoreGive(s_data_uart_mutex);

        if (len > 0) {
            hexdump_bytes("DATA RX", buf, (size_t)len);

            if ((len % 2) == 0) {
                printf("decoded 9-bit words:");
                for (int i = 0; i < len; i += 2) {
                    uint16_t word = (uint16_t)buf[i] | (((uint16_t)buf[i + 1] & 0x01) << 8);
                    printf(" 0x%03X", word);
                }
                printf("\n");
            }
        }
    }
}

static void trim_line(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }
}

void app_main(void)
{
    char line[CONSOLE_LINE_MAX];
    size_t line_len = 0;

    ESP_ERROR_CHECK(init_console_io());
    ESP_ERROR_CHECK(init_interfaces());
    xTaskCreate(data_rx_task, "data_rx_task", 4096, NULL, 5, NULL);

    printf("\nBonanza RP2040 test firmware ready.\n");
    print_help();
    printf("bonanza-test> ");
    fflush(stdout);

    while (true) {
        int ch = getchar();

        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            line[line_len] = '\0';
            printf("\n");
            fflush(stdout);

            trim_line(line);
            if (line[0] == '\0') {
                printf("bonanza-test> ");
                fflush(stdout);
                line_len = 0;
                continue;
            }
        } else if (ch == 0x08 || ch == 0x7f) {
            if (line_len > 0) {
                line_len--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        } else {
            if (line_len + 1 < sizeof(line)) {
                line[line_len++] = (char)ch;
                putchar(ch);
                fflush(stdout);
            }
            continue;
        }

        if (strcmp(line, "help") == 0) {
            print_help();
        } else if (strcmp(line, "gpio") == 0) {
            cmd_gpio_status();
        } else if (strcmp(line, "tach") == 0) {
            cmd_get_tach();
        } else if (strcmp(line, "vr") == 0) {
            cmd_vr(&line[2]);
        } else if (strncmp(line, "vr ", 3) == 0) {
            cmd_vr(&line[2]);
        } else if (strcmp(line, "pattern") == 0) {
            cmd_pattern();
        } else if (strcmp(line, "flush") == 0) {
            if (xSemaphoreTake(s_data_uart_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                uart_flush_input(DATA_UART_PORT);
                xSemaphoreGive(s_data_uart_mutex);
                printf("data uart rx flushed\n");
            } else {
                printf("data uart busy\n");
            }
        } else if (strncmp(line, "5v ", 3) == 0) {
            cmd_set_5v(atoi(&line[3]));
        } else if (strncmp(line, "rst ", 4) == 0) {
            cmd_set_rst(atoi(&line[4]));
        } else if (strncmp(line, "fan ", 4) == 0) {
            cmd_set_fan(atoi(&line[4]));
        } else if (strncmp(line, "BZM_sendnoop ", 13) == 0) {
            cmd_bzm_sendnoop(&line[13]);
        } else if (strncmp(line, "BZM_readreg ", 12) == 0) {
            cmd_bzm_readreg(&line[12]);
        } else if (strncmp(line, "BZM_writereg ", 13) == 0) {
            cmd_bzm_writereg(&line[13]);
        } else if (strncmp(line, "send9 ", 6) == 0) {
            uint32_t parsed[32];
            uint16_t words[32];
            size_t count = parse_hex_sequence(&line[6], parsed, 32, 0x1ff);

            if (count == 0) {
                printf("usage: send9 0x155 0x0aa ...\n");
            } else {
                for (size_t i = 0; i < count; i++) {
                    words[i] = (uint16_t)parsed[i];
                }

                data_send_words(words, count);
            }
        } else if (strncmp(line, "raw ", 4) == 0) {
            uint32_t parsed[64];
            uint8_t bytes[64];
            size_t count = parse_hex_sequence(&line[4], parsed, 64, 0xff);

            if (count == 0) {
                printf("usage: raw 55 01 aa 00\n");
            } else {
                for (size_t i = 0; i < count; i++) {
                    bytes[i] = (uint8_t)parsed[i];
                }

                data_send_bytes(bytes, count);
            }
        } else {
            printf("unknown command: %s\n", line);
        }

        line_len = 0;
        printf("bonanza-test> ");
        fflush(stdout);
    }
}

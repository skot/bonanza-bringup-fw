#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    printf("  pattern           Send default 9-bit test pattern on data UART\n");
    printf("  send9 <...>       Send one or more 9-bit words, ex: send9 0x155 0x0aa\n");
    printf("  raw <...>         Send raw byte stream on data UART, ex: raw 55 01 aa 00\n");
    printf("  flush             Flush pending data UART RX bytes\n");
    printf("\n");
    printf("Control UART: UART0 TX GPIO43, RX GPIO44 @ 115200\n");
    printf("Data UART:    UART1 TX GPIO17, RX GPIO18 @ 5000000\n");
    printf("\n");
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
        uint8_t framed[CONTROL_RESPONSE_MAX];
        size_t framed_len = response->len + 3;

        framed[0] = header[0];
        framed[1] = header[1];
        framed[2] = header[2];
        memcpy(&framed[3], response->payload, response->len);
        hexdump_bytes("CTRL RX", framed, framed_len);
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

static void data_send_bytes(const uint8_t *bytes, size_t len)
{
    hexdump_bytes("DATA TX", bytes, len);
    uart_write_bytes(DATA_UART_PORT, bytes, len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_wait_tx_done(DATA_UART_PORT, pdMS_TO_TICKS(100)));
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
    (void)arg;

    while (true) {
        int len = uart_read_bytes(DATA_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(200));
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
            }
            continue;
        } else {
            if (line_len + 1 < sizeof(line)) {
                line[line_len++] = (char)ch;
            }
            continue;
        }

        if (strcmp(line, "help") == 0) {
            print_help();
        } else if (strcmp(line, "gpio") == 0) {
            cmd_gpio_status();
        } else if (strcmp(line, "tach") == 0) {
            cmd_get_tach();
        } else if (strcmp(line, "pattern") == 0) {
            cmd_pattern();
        } else if (strcmp(line, "flush") == 0) {
            uart_flush_input(DATA_UART_PORT);
            printf("data uart rx flushed\n");
        } else if (strncmp(line, "5v ", 3) == 0) {
            cmd_set_5v(atoi(&line[3]));
        } else if (strncmp(line, "rst ", 4) == 0) {
            cmd_set_rst(atoi(&line[4]));
        } else if (strncmp(line, "fan ", 4) == 0) {
            cmd_set_fan(atoi(&line[4]));
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

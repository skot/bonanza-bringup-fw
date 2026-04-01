#include "swd.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "hal/gpio_ll.h"

#define TAG "bonanza-swd"

#define SWD_SWCLK_PIN GPIO_NUM_1
#define SWD_SWDIO_PIN GPIO_NUM_2

#define SWD_HALF_PERIOD_US 5
#define SWD_IDLE_LOW_BITS 8
#define SWD_TURNAROUND_BITS 1

#define SWD_ACK_OK 0x1
#define SWD_ACK_WAIT 0x2
#define SWD_ACK_FAULT 0x4
#define SWD_ACK_NONE 0x7

#define SWD_DP_DPIDR_REQ 0xA5u
#define SWD_DP_TARGETSEL_REQ 0x99u

extern const uint8_t _binary_rp2040_firmware_bin_start[] asm("_binary_rp2040_firmware_bin_start");
extern const uint8_t _binary_rp2040_firmware_bin_end[] asm("_binary_rp2040_firmware_bin_end");

static bool s_swclk_is_output;
static bool s_swdio_is_output;
static bool s_turn_is_write;

static void swd_delay(void)
{
    esp_rom_delay_us(SWD_HALF_PERIOD_US);
}

static void swd_set_swclk_output(int level)
{
    gpio_ll_set_level(&GPIO, SWD_SWCLK_PIN, level);
    if (!s_swclk_is_output) {
        gpio_ll_output_enable(&GPIO, SWD_SWCLK_PIN);
        s_swclk_is_output = true;
    }
}

static void swd_set_swclk_input(void)
{
    if (s_swclk_is_output) {
        gpio_ll_output_disable(&GPIO, SWD_SWCLK_PIN);
        s_swclk_is_output = false;
    }
}

static void swd_set_swdio_level(int level)
{
    gpio_ll_set_level(&GPIO, SWD_SWDIO_PIN, level);
}

static void swd_set_swdio_output(bool enabled)
{
    if (enabled) {
        if (!s_swdio_is_output) {
            gpio_ll_output_enable(&GPIO, SWD_SWDIO_PIN);
            s_swdio_is_output = true;
        }
        return;
    }

    if (s_swdio_is_output) {
        gpio_ll_output_disable(&GPIO, SWD_SWDIO_PIN);
        s_swdio_is_output = false;
    }
}

static bool swd_get_swdio(void)
{
    return gpio_ll_get_level(&GPIO, SWD_SWDIO_PIN) != 0;
}

static void swd_clock_low(void)
{
    swd_set_swclk_output(0);
    swd_delay();
}

static void swd_clock_high(void)
{
    swd_set_swclk_output(1);
    swd_delay();
}

static void swd_turn(bool write_mode)
{
    swd_set_swdio_level(1);
    swd_set_swdio_output(false);
    swd_clock_low();
    swd_clock_high();

    if (write_mode) {
        gpio_ll_output_enable(&GPIO, SWD_SWDIO_PIN);
        s_swdio_is_output = true;
    }

    s_turn_is_write = write_mode;
}

static void swd_write_bits(uint32_t value, size_t bit_count)
{
    if (!s_turn_is_write) {
        swd_turn(true);
    }

    for (size_t i = 0; i < bit_count; i++) {
        swd_set_swdio_level(((value >> i) & 0x1u) != 0 ? 1 : 0);
        swd_clock_low();
        swd_clock_high();
    }
}

static uint32_t swd_read_bits(size_t bit_count)
{
    uint32_t value = 0;
    uint32_t input_bit = 1u;

    if (s_turn_is_write) {
        swd_turn(false);
    }

    for (size_t i = 0; i < bit_count; i++) {
        if (swd_get_swdio()) {
            value |= input_bit;
        }
        swd_clock_low();
        swd_clock_high();
        input_bit <<= 1;
    }

    return value;
}

static uint8_t swd_even_parity32(uint32_t value)
{
    value ^= value >> 16;
    value ^= value >> 8;
    value ^= value >> 4;
    value &= 0x0F;
    return (uint8_t)((0x6996u >> value) & 0x1u);
}

static void swd_write_sequence(const uint8_t *bytes, size_t bit_count)
{
    if (!s_turn_is_write) {
        swd_turn(true);
    }

    for (size_t i = 0; i < bit_count; i++) {
        uint8_t byte = bytes[i / 8];
        swd_set_swdio_level(((byte >> (i % 8)) & 0x1u) != 0 ? 1 : 0);
        swd_clock_low();
        swd_clock_high();
    }
}

static void swd_claim_pins(void)
{
    swd_set_swclk_output(1);
    swd_set_swdio_level(1);
    swd_set_swdio_output(true);
    s_turn_is_write = true;
}

static void swd_release_pins(void)
{
    swd_set_swclk_input();
    swd_set_swdio_output(false);
    s_turn_is_write = false;
}

static void swd_switch_jtag_to_dormant(void)
{
    static const uint8_t seq[] = {
        0xff, 0x75, 0x77, 0x77, 0x67,
    };

    swd_set_swdio_level(1);
    swd_write_sequence(seq, 40);
}

static void swd_switch_dormant_to_swd(void)
{
    static const uint8_t seq[] = {
        0xff,
        0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
        0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
        0xa0, 0xf1, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x00,
    };

    swd_set_swdio_level(1);
    swd_write_sequence(seq, 224);
}

static esp_err_t swd_write_targetsel(uint32_t targetsel)
{
    swd_write_bits(SWD_DP_TARGETSEL_REQ, 8);
    (void)swd_read_bits(3);
    swd_write_bits(targetsel, 32);
    swd_write_bits(swd_even_parity32(targetsel), 1);
    return ESP_OK;
}

static esp_err_t swd_read_dpidr(uint32_t *dpidr, uint8_t *ack_out)
{
    uint8_t ack;
    uint32_t data;
    uint8_t parity_bit;

    swd_write_bits(SWD_DP_DPIDR_REQ, 8);

    ack = (uint8_t)swd_read_bits(3);
    if (ack_out != NULL) {
        *ack_out = ack;
    }

    if (ack != SWD_ACK_OK) {
        (void)swd_write_bits(0, 32);
        return ESP_ERR_INVALID_RESPONSE;
    }

    data = swd_read_bits(32);
    parity_bit = (uint8_t)swd_read_bits(1);
    swd_write_bits(0, 1);

    if (parity_bit != swd_even_parity32(data)) {
        return ESP_ERR_INVALID_CRC;
    }

    *dpidr = data;
    return ESP_OK;
}

esp_err_t swd_init(void)
{
    const gpio_config_t swclk_config = {
        .pin_bit_mask = 1ULL << SWD_SWCLK_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const gpio_config_t swdio_config = {
        .pin_bit_mask = 1ULL << SWD_SWDIO_PIN,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&swclk_config), TAG, "swclk config failed");
    ESP_RETURN_ON_ERROR(gpio_config(&swdio_config), TAG, "swdio config failed");

    swd_release_pins();
    return ESP_OK;
}

esp_err_t swd_read_target_id(uint32_t targetsel, swd_id_result_t *result)
{
    uint8_t ack = 0;
    uint32_t dpidr = 0;
    esp_err_t err;

    if (result != NULL) {
        result->targetsel = targetsel;
        result->dpidr = 0;
        result->ack = SWD_ACK_NONE;
    }

    swd_claim_pins();
    swd_switch_jtag_to_dormant();
    swd_switch_dormant_to_swd();
    err = swd_write_targetsel(targetsel);
    if (err != ESP_OK) {
        swd_release_pins();
        return err;
    }
    err = swd_read_dpidr(&dpidr, &ack);
    if (result != NULL) {
        result->ack = ack;
    }
    if (err != ESP_OK) {
        swd_release_pins();
        return err;
    }

    if (result != NULL) {
        result->dpidr = dpidr;
    }
    swd_release_pins();
    return ESP_OK;
}

const char *swd_ack_name(uint8_t ack)
{
    switch (ack) {
    case SWD_ACK_OK:
        return "OK";
    case SWD_ACK_WAIT:
        return "WAIT";
    case SWD_ACK_FAULT:
        return "FAULT";
    case SWD_ACK_NONE:
        return "NO-REPLY";
    default:
        return "RESERVED";
    }
}

const uint8_t *swd_embedded_firmware_start(void)
{
    return _binary_rp2040_firmware_bin_start;
}

size_t swd_embedded_firmware_size(void)
{
    return (size_t)(_binary_rp2040_firmware_bin_end - _binary_rp2040_firmware_bin_start);
}

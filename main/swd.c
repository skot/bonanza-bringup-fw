#include "swd.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define SWD_DP_TARGETSEL_REQ 0x99u

#define SWD_DP_ABORT_ADDR 0x00u
#define SWD_DP_DPIDR_ADDR 0x00u
#define SWD_DP_CTRL_STAT_ADDR 0x04u
#define SWD_DP_SELECT_ADDR 0x08u
#define SWD_DP_RDBUFF_ADDR 0x0Cu

#define SWD_DP_ABORT_ORUNERRCLR (1u << 4)
#define SWD_DP_ABORT_WDERRCLR   (1u << 3)
#define SWD_DP_ABORT_STKERRCLR  (1u << 2)
#define SWD_DP_ABORT_STKCMPCLR  (1u << 1)
#define SWD_DP_ABORT_CLEAR_ALL  (SWD_DP_ABORT_ORUNERRCLR | SWD_DP_ABORT_WDERRCLR | SWD_DP_ABORT_STKERRCLR | SWD_DP_ABORT_STKCMPCLR)

#define SWD_DP_CTRL_CDBGPWRUPREQ (1u << 28)
#define SWD_DP_CTRL_CDBGPWRUPACK (1u << 29)
#define SWD_DP_CTRL_CSYSPWRUPREQ (1u << 30)
#define SWD_DP_CTRL_CSYSPWRUPACK (1u << 31)
#define SWD_DP_POWERUP_REQ (SWD_DP_CTRL_CDBGPWRUPREQ | SWD_DP_CTRL_CSYSPWRUPREQ)
#define SWD_DP_POWERUP_ACK (SWD_DP_CTRL_CDBGPWRUPACK | SWD_DP_CTRL_CSYSPWRUPACK)

#define SWD_APSEL_MEM_AP 0u
#define SWD_MEM_AP_REG_CSW 0x00u
#define SWD_MEM_AP_REG_TAR 0x04u
#define SWD_MEM_AP_REG_DRW 0x0Cu
#define SWD_MEM_AP_CSW_32BIT 2u
#define SWD_MEM_AP_CSW_ADDRINC_OFF 0u
#define SWD_MEM_AP_CSW_DBGSWENABLE (1u << 31)
#define SWD_MEM_AP_CSW_AHB_HPROT1 (1u << 25)
#define SWD_MEM_AP_CSW_AHB_MASTER_DEBUG (1u << 29)
#define SWD_MEM_AP_CSW_AHB_DEFAULT (SWD_MEM_AP_CSW_AHB_HPROT1 | SWD_MEM_AP_CSW_AHB_MASTER_DEBUG | SWD_MEM_AP_CSW_DBGSWENABLE)
#define SWD_MEM_AP_CSW_WORD (SWD_MEM_AP_CSW_AHB_DEFAULT | SWD_MEM_AP_CSW_ADDRINC_OFF | SWD_MEM_AP_CSW_32BIT)

#define SWD_AIRCR_ADDR 0xE000ED0Cu
#define SWD_DHCSR_ADDR 0xE000EDF0u
#define SWD_DCRSR_ADDR 0xE000EDF4u
#define SWD_DCRDR_ADDR 0xE000EDF8u
#define SWD_DEMCR_ADDR 0xE000EDFCu

#define SWD_AIRCR_VECTKEY 0x05FA0000u
#define SWD_AIRCR_SYSRESETREQ (1u << 2)
#define SWD_DHCSR_DBGKEY 0xA05F0000u
#define SWD_DHCSR_C_DEBUGEN (1u << 0)
#define SWD_DHCSR_C_HALT (1u << 1)
#define SWD_DHCSR_S_REGRDY (1u << 16)
#define SWD_DHCSR_S_HALT (1u << 17)

#define SWD_DCRSR_REGWNR (1u << 16)
#define SWD_CORE_REG_R0 0u
#define SWD_CORE_REG_PC 15u
#define SWD_CORE_REG_XPSR 16u
#define SWD_CORE_REG_MSP 17u

#define SWD_RP2040_XIP_BASE 0x10000000u
#define SWD_RP2040_STUB_BASE 0x20030000u
#define SWD_RP2040_STUB_STACK_TOP 0x20042000u
#define SWD_RP2040_STUB_DATA_BASE 0x20032000u
#define SWD_RP2040_STUB_CTX_ADDR 0x2003FF00u

#define SWD_FLASH_PAGE_SIZE 256u
#define SWD_FLASH_SECTOR_SIZE 4096u
#define SWD_STUB_OP_ERASE 1u
#define SWD_STUB_OP_PROGRAM 2u
#define SWD_STUB_OP_PROGRAM_IMAGE 3u
#define SWD_STUB_STATUS_IDLE 0u
#define SWD_STUB_STATUS_BUSY 1u
#define SWD_STUB_STATUS_OK 2u

#define SWD_MAX_RETRIES 16u
#define SWD_POWERUP_POLL_LIMIT 32u
#define SWD_REG_POLL_LIMIT 128u
#define SWD_RUN_POLL_LIMIT 2000u

extern const uint8_t _binary_rp2040_firmware_bin_start[] asm("_binary_rp2040_firmware_bin_start");
extern const uint8_t _binary_rp2040_firmware_bin_end[] asm("_binary_rp2040_firmware_bin_end");
extern const uint8_t _binary_rp2040_flash_stub_bin_start[] asm("_binary_rp2040_flash_stub_bin_start");
extern const uint8_t _binary_rp2040_flash_stub_bin_end[] asm("_binary_rp2040_flash_stub_bin_end");

typedef struct {
    uint32_t op;
    uint32_t status;
    uint32_t stage;
    uint32_t progress;
    uint32_t flash_offset;
    uint32_t count;
    uint32_t data_addr;
} swd_flash_stub_ctx_t;

#define SWD_STUB_STAGE_IDLE 0u
#define SWD_STUB_STAGE_ERASE 1u
#define SWD_STUB_STAGE_PROGRAM 2u
#define SWD_STUB_STAGE_DONE 3u

static bool s_swclk_is_output;
static bool s_swdio_is_output;
static bool s_turn_is_write;

static void swd_cooperate(void)
{
    vTaskDelay(1);
}

static unsigned int swd_percent(size_t done, size_t total)
{
    if (total == 0) {
        return 100;
    }

    if (done >= total) {
        return 100;
    }

    return (unsigned int)((done * 100u) / total);
}

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

static void swd_idle_cycles(size_t cycles)
{
    swd_write_bits(0, cycles);
}

static uint8_t swd_make_request(bool ap, bool read, uint8_t addr)
{
    uint8_t a2 = (addr >> 2) & 0x1u;
    uint8_t a3 = (addr >> 3) & 0x1u;
    uint8_t parity = (uint8_t)((ap ? 1u : 0u) ^ (read ? 1u : 0u) ^ a2 ^ a3);

    return (uint8_t)(0x81u | ((ap ? 1u : 0u) << 1) | ((read ? 1u : 0u) << 2) | (a2 << 3) | (a3 << 4) | (parity << 5));
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
    uint8_t request = swd_make_request(false, true, SWD_DP_DPIDR_ADDR);

    swd_write_bits(request, 8);

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

static esp_err_t swd_read_reg(bool ap, uint8_t addr, uint32_t *value, uint8_t *ack_out)
{
    uint8_t ack;
    uint32_t data;
    uint8_t parity_bit;
    uint8_t request = swd_make_request(ap, true, addr);

    swd_write_bits(request, 8);

    ack = (uint8_t)swd_read_bits(3);
    if (ack_out != NULL) {
        *ack_out = ack;
    }

    if (ack != SWD_ACK_OK) {
        swd_write_bits(0, 1);
        return ESP_ERR_INVALID_RESPONSE;
    }

    data = swd_read_bits(32);
    parity_bit = (uint8_t)swd_read_bits(1);
    swd_write_bits(0, 1);

    if (parity_bit != swd_even_parity32(data)) {
        return ESP_ERR_INVALID_CRC;
    }

    if (value != NULL) {
        *value = data;
    }

    if (ap) {
        swd_idle_cycles(SWD_IDLE_LOW_BITS);
    }

    return ESP_OK;
}

static esp_err_t swd_write_reg(bool ap, uint8_t addr, uint32_t value, uint8_t *ack_out)
{
    uint8_t ack = SWD_ACK_NONE;
    uint8_t request = swd_make_request(ap, false, addr);

    swd_write_bits(request, 8);

    for (unsigned int retry = 0; retry < SWD_MAX_RETRIES; retry++) {
        ack = (uint8_t)swd_read_bits(3);

        if (ack == SWD_ACK_WAIT) {
            swd_write_bits(0, 1);
            swd_idle_cycles(SWD_IDLE_LOW_BITS);
            swd_write_bits(request, 8);
            continue;
        }

        break;
    }

    if (ack_out != NULL) {
        *ack_out = ack;
    }

    if (ack != SWD_ACK_OK) {
        swd_write_bits(0, 1);
        return ESP_ERR_INVALID_RESPONSE;
    }

    swd_write_bits(value, 32);
    swd_write_bits(swd_even_parity32(value), 1);
    swd_write_bits(0, 1);

    if (ap) {
        swd_idle_cycles(SWD_IDLE_LOW_BITS);
    }

    return ESP_OK;
}

static esp_err_t swd_read_dp(uint8_t addr, uint32_t *value)
{
    return swd_read_reg(false, addr, value, NULL);
}

static esp_err_t swd_write_dp(uint8_t addr, uint32_t value)
{
    return swd_write_reg(false, addr, value, NULL);
}

static esp_err_t swd_select_ap_bank(uint8_t ap_sel, uint8_t ap_reg_addr)
{
    uint32_t select = ((uint32_t)ap_sel << 24) | ((uint32_t)(ap_reg_addr & 0xF0u));
    return swd_write_dp(SWD_DP_SELECT_ADDR, select);
}

static esp_err_t swd_read_ap(uint8_t ap_sel, uint8_t ap_reg_addr, uint32_t *value)
{
    uint32_t posted_value;

    ESP_RETURN_ON_ERROR(swd_select_ap_bank(ap_sel, ap_reg_addr), TAG, "select ap bank failed");
    ESP_RETURN_ON_ERROR(swd_read_reg(true, ap_reg_addr, &posted_value, NULL), TAG, "ap posted read failed");
    ESP_RETURN_ON_ERROR(swd_read_dp(SWD_DP_RDBUFF_ADDR, value), TAG, "ap rdbuff read failed");
    return ESP_OK;
}

static esp_err_t swd_write_ap(uint8_t ap_sel, uint8_t ap_reg_addr, uint32_t value)
{
    ESP_RETURN_ON_ERROR(swd_select_ap_bank(ap_sel, ap_reg_addr), TAG, "select ap bank failed");
    ESP_RETURN_ON_ERROR(swd_write_reg(true, ap_reg_addr, value, NULL), TAG, "ap write failed");
    return ESP_OK;
}

static esp_err_t swd_power_up_debug(void)
{
    uint32_t ctrl_stat = 0;

    ESP_RETURN_ON_ERROR(swd_write_dp(SWD_DP_ABORT_ADDR, SWD_DP_ABORT_CLEAR_ALL), TAG, "abort clear failed");
    ESP_RETURN_ON_ERROR(swd_write_dp(SWD_DP_SELECT_ADDR, 0), TAG, "dp select clear failed");
    ESP_RETURN_ON_ERROR(swd_write_dp(SWD_DP_CTRL_STAT_ADDR, SWD_DP_POWERUP_REQ), TAG, "powerup request failed");

    for (unsigned int i = 0; i < SWD_POWERUP_POLL_LIMIT; i++) {
        ESP_RETURN_ON_ERROR(swd_read_dp(SWD_DP_CTRL_STAT_ADDR, &ctrl_stat), TAG, "ctrl/stat read failed");
        if ((ctrl_stat & SWD_DP_POWERUP_ACK) == SWD_DP_POWERUP_ACK) {
            return ESP_OK;
        }
        swd_cooperate();
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t swd_setup_mem_ap(void)
{
    return swd_write_ap(SWD_APSEL_MEM_AP, SWD_MEM_AP_REG_CSW, SWD_MEM_AP_CSW_WORD);
}

static esp_err_t swd_mem_read_word32(uint32_t address, uint32_t *value)
{
    ESP_RETURN_ON_ERROR(swd_setup_mem_ap(), TAG, "mem-ap csw setup failed");
    ESP_RETURN_ON_ERROR(swd_write_ap(SWD_APSEL_MEM_AP, SWD_MEM_AP_REG_TAR, address), TAG, "mem-ap tar write failed");
    ESP_RETURN_ON_ERROR(swd_read_ap(SWD_APSEL_MEM_AP, SWD_MEM_AP_REG_DRW, value), TAG, "mem-ap drw read failed");
    return ESP_OK;
}

static esp_err_t swd_mem_write_word32(uint32_t address, uint32_t value)
{
    ESP_RETURN_ON_ERROR(swd_setup_mem_ap(), TAG, "mem-ap csw setup failed");
    ESP_RETURN_ON_ERROR(swd_write_ap(SWD_APSEL_MEM_AP, SWD_MEM_AP_REG_TAR, address), TAG, "mem-ap tar write failed");
    ESP_RETURN_ON_ERROR(swd_write_ap(SWD_APSEL_MEM_AP, SWD_MEM_AP_REG_DRW, value), TAG, "mem-ap drw write failed");
    return ESP_OK;
}

static esp_err_t swd_mem_write_block(uint32_t address, const uint8_t *data, size_t len)
{
    uint32_t word;

    if ((data == NULL) && (len != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t offset = 0; offset < len; offset += 4) {
        word = 0;
        for (size_t i = 0; i < 4 && (offset + i) < len; i++) {
            word |= ((uint32_t)data[offset + i]) << (8u * i);
        }

        ESP_RETURN_ON_ERROR(swd_mem_write_word32(address + (uint32_t)offset, word), TAG, "mem block write failed");
        if (((offset / 4u) & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_OK;
}

static esp_err_t swd_mem_write_block_with_progress(uint32_t address, const uint8_t *data, size_t len, const char *label)
{
    uint32_t word;
    unsigned int last_percent = 0;

    if ((data == NULL) && (len != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (label != NULL) {
        ESP_LOGI(TAG, "%s: %u%%", label, 0u);
    }

    for (size_t offset = 0; offset < len; offset += 4) {
        word = 0;
        for (size_t i = 0; i < 4 && (offset + i) < len; i++) {
            word |= ((uint32_t)data[offset + i]) << (8u * i);
        }

        ESP_RETURN_ON_ERROR(swd_mem_write_word32(address + (uint32_t)offset, word), TAG, "mem block write failed");

        if (label != NULL) {
            unsigned int percent = swd_percent(offset + 4, len);
            if (percent >= last_percent + 10 || percent == 100) {
                last_percent = percent;
                ESP_LOGI(TAG, "%s: %u%%", label, percent);
            }
        }

        if (((offset / 4u) & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_OK;
}

static esp_err_t swd_mem_verify_block(uint32_t address, const uint8_t *expected, size_t len)
{
    uint32_t actual_word;

    if ((expected == NULL) && (len != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t offset = 0; offset < len; offset += 4) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(address + (uint32_t)offset, &actual_word), TAG, "mem verify read failed");
        for (size_t i = 0; i < 4 && (offset + i) < len; i++) {
            uint8_t actual_byte = (uint8_t)((actual_word >> (8u * i)) & 0xFFu);
            if (actual_byte != expected[offset + i]) {
                return ESP_ERR_INVALID_CRC;
            }
        }
        if (((offset / 4u) & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_OK;
}

static esp_err_t swd_mem_verify_block_with_progress(uint32_t address, const uint8_t *expected, size_t len, const char *label)
{
    uint32_t actual_word;
    unsigned int last_percent = 0;

    if ((expected == NULL) && (len != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (label != NULL) {
        ESP_LOGI(TAG, "%s: %u%%", label, 0u);
    }

    for (size_t offset = 0; offset < len; offset += 4) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(address + (uint32_t)offset, &actual_word), TAG, "mem verify read failed");
        for (size_t i = 0; i < 4 && (offset + i) < len; i++) {
            uint8_t actual_byte = (uint8_t)((actual_word >> (8u * i)) & 0xFFu);
            if (actual_byte != expected[offset + i]) {
                return ESP_ERR_INVALID_CRC;
            }
        }

        if (label != NULL) {
            unsigned int percent = swd_percent(offset + 4, len);
            if (percent >= last_percent + 10 || percent == 100) {
                last_percent = percent;
                ESP_LOGI(TAG, "%s: %u%%", label, percent);
            }
        }

        if (((offset / 4u) & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_OK;
}

static esp_err_t swd_halt_attached_core(uint32_t *dhcsr_out)
{
    uint32_t dhcsr = 0;

    ESP_RETURN_ON_ERROR(swd_mem_write_word32(SWD_DHCSR_ADDR, SWD_DHCSR_DBGKEY | SWD_DHCSR_C_DEBUGEN | SWD_DHCSR_C_HALT), TAG, "dhcsr halt write failed");

    for (unsigned int i = 0; i < SWD_REG_POLL_LIMIT; i++) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(SWD_DHCSR_ADDR, &dhcsr), TAG, "dhcsr halt poll failed");
        if ((dhcsr & SWD_DHCSR_S_HALT) != 0) {
            if (dhcsr_out != NULL) {
                *dhcsr_out = dhcsr;
            }
            return ESP_OK;
        }
        if ((i & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t swd_write_core_reg(uint32_t regsel, uint32_t value)
{
    uint32_t dhcsr = 0;

    ESP_RETURN_ON_ERROR(swd_mem_write_word32(SWD_DCRDR_ADDR, value), TAG, "dcrdr write failed");
    ESP_RETURN_ON_ERROR(swd_mem_write_word32(SWD_DCRSR_ADDR, SWD_DCRSR_REGWNR | regsel), TAG, "dcrsr write failed");

    for (unsigned int i = 0; i < SWD_REG_POLL_LIMIT; i++) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(SWD_DHCSR_ADDR, &dhcsr), TAG, "dhcsr reg poll failed");
        if ((dhcsr & SWD_DHCSR_S_REGRDY) != 0) {
            return ESP_OK;
        }
        if ((i & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t swd_read_core_reg(uint32_t regsel, uint32_t *value_out)
{
    uint32_t dhcsr = 0;
    uint32_t value = 0;

    if (value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(swd_mem_write_word32(SWD_DCRSR_ADDR, regsel), TAG, "dcrsr read select failed");

    for (unsigned int i = 0; i < SWD_REG_POLL_LIMIT; i++) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(SWD_DHCSR_ADDR, &dhcsr), TAG, "dhcsr reg read poll failed");
        if ((dhcsr & SWD_DHCSR_S_REGRDY) != 0) {
            ESP_RETURN_ON_ERROR(swd_mem_read_word32(SWD_DCRDR_ADDR, &value), TAG, "dcrdr read failed");
            *value_out = value;
            return ESP_OK;
        }
        if ((i & 0x7u) == 0x7u) {
            swd_cooperate();
        }
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t swd_reset_system_attached(void)
{
    return swd_mem_write_word32(SWD_AIRCR_ADDR, SWD_AIRCR_VECTKEY | SWD_AIRCR_SYSRESETREQ);
}

static esp_err_t swd_run_stub_attached(uint32_t ctx_addr, uint32_t *stub_status_out)
{
    uint32_t dhcsr = 0;
    uint32_t stub_status = SWD_STUB_STATUS_IDLE;
    uint32_t stub_stage = SWD_STUB_STAGE_IDLE;
    uint32_t stub_progress = 0;
    uint32_t pc = 0;
    uint32_t r0 = 0;
    bool ran = false;
    unsigned int last_percent = 0;
    uint32_t total = 0;

    (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, count), &total);

    ESP_RETURN_ON_ERROR(swd_write_core_reg(SWD_CORE_REG_R0, ctx_addr), TAG, "write r0 failed");
    ESP_RETURN_ON_ERROR(swd_write_core_reg(SWD_CORE_REG_MSP, SWD_RP2040_STUB_STACK_TOP), TAG, "write msp failed");
    ESP_RETURN_ON_ERROR(swd_write_core_reg(SWD_CORE_REG_PC, SWD_RP2040_STUB_BASE | 1u), TAG, "write pc failed");
    ESP_RETURN_ON_ERROR(swd_write_core_reg(SWD_CORE_REG_XPSR, 0x01000000u), TAG, "write xpsr failed");
    ESP_RETURN_ON_ERROR(swd_mem_write_word32(SWD_DHCSR_ADDR, SWD_DHCSR_DBGKEY | SWD_DHCSR_C_DEBUGEN), TAG, "resume core failed");

    for (unsigned int i = 0; i < SWD_RUN_POLL_LIMIT; i++) {
        ESP_RETURN_ON_ERROR(swd_mem_read_word32(SWD_DHCSR_ADDR, &dhcsr), TAG, "run poll dhcsr failed");
        if ((dhcsr & SWD_DHCSR_S_HALT) == 0) {
            ran = true;
            if ((i & 0xFu) == 0xFu) {
                (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, stage), &stub_stage);
                (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, progress), &stub_progress);
                if (stub_stage == SWD_STUB_STAGE_PROGRAM && total != 0) {
                    unsigned int percent = swd_percent(stub_progress, total);
                    if (percent >= last_percent + 10 || percent == 100) {
                        last_percent = percent;
                        ESP_LOGI(TAG, "rpflash: erase+program %u%%", percent);
                    }
                } else if (stub_stage == SWD_STUB_STAGE_ERASE && last_percent == 0) {
                    ESP_LOGI(TAG, "rpflash: erase+program erase phase");
                    last_percent = 1;
                }
            }
            if ((i & 0x1Fu) == 0x1Fu) {
                swd_cooperate();
            }
            continue;
        }

        if (!ran) {
            if ((i & 0x1Fu) == 0x1Fu) {
                swd_cooperate();
            }
            continue;
        }

        if ((dhcsr & SWD_DHCSR_S_HALT) != 0) {
            ESP_RETURN_ON_ERROR(swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, status), &stub_status), TAG, "read stub status failed");
            (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, stage), &stub_stage);
            (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, progress), &stub_progress);
            if (stub_status_out != NULL) {
                *stub_status_out = stub_status;
            }
            if (stub_stage == SWD_STUB_STAGE_DONE && total != 0 && last_percent < 100) {
                ESP_LOGI(TAG, "rpflash: erase+program 100%%");
            }
            return (stub_status == SWD_STUB_STATUS_OK) ? ESP_OK : ESP_FAIL;
        }
    }

    (void)swd_halt_attached_core(&dhcsr);
    (void)swd_read_core_reg(SWD_CORE_REG_PC, &pc);
    (void)swd_read_core_reg(SWD_CORE_REG_R0, &r0);
    (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, status), &stub_status);
    (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, stage), &stub_stage);
    (void)swd_mem_read_word32(ctx_addr + offsetof(swd_flash_stub_ctx_t, progress), &stub_progress);
    ESP_LOGE(TAG,
        "stub timeout: ran=%s dhcsr=0x%08" PRIX32 " pc=0x%08" PRIX32 " r0=0x%08" PRIX32 " ctx.status=0x%08" PRIX32 " ctx.stage=0x%08" PRIX32 " ctx.progress=%" PRIu32,
        ran ? "yes" : "no", dhcsr, pc, r0, stub_status, stub_stage, stub_progress);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t swd_attach_target(uint32_t targetsel)
{
    uint32_t ignored_dpidr;

    swd_switch_jtag_to_dormant();
    swd_switch_dormant_to_swd();
    ESP_RETURN_ON_ERROR(swd_write_targetsel(targetsel), TAG, "targetsel write failed");
    ESP_RETURN_ON_ERROR(swd_read_dpidr(&ignored_dpidr, NULL), TAG, "dpidr read failed");
    ESP_RETURN_ON_ERROR(swd_power_up_debug(), TAG, "debug powerup failed");
    return ESP_OK;
}

static const uint8_t *swd_embedded_stub_start(void)
{
    return _binary_rp2040_flash_stub_bin_start;
}

static size_t swd_embedded_stub_size(void)
{
    return (size_t)(_binary_rp2040_flash_stub_bin_end - _binary_rp2040_flash_stub_bin_start);
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

esp_err_t swd_halt_core(uint32_t targetsel, uint32_t *dhcsr_out)
{
    uint32_t dhcsr = 0;
    esp_err_t err;

    swd_claim_pins();
    err = swd_attach_target(targetsel);
    if (err == ESP_OK) {
        err = swd_halt_attached_core(&dhcsr);
    }
    swd_release_pins();

    if (err != ESP_OK) {
        return err;
    }

    if ((dhcsr & SWD_DHCSR_S_HALT) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (dhcsr_out != NULL) {
        *dhcsr_out = dhcsr;
    }

    return ESP_OK;
}

esp_err_t swd_read_word32(uint32_t targetsel, uint32_t address, uint32_t *value)
{
    esp_err_t err;

    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    swd_claim_pins();
    err = swd_attach_target(targetsel);
    if (err == ESP_OK) {
        err = swd_mem_read_word32(address, value);
    }
    swd_release_pins();
    return err;
}

esp_err_t swd_write_word32(uint32_t targetsel, uint32_t address, uint32_t value)
{
    esp_err_t err;

    swd_claim_pins();
    err = swd_attach_target(targetsel);
    if (err == ESP_OK) {
        err = swd_mem_write_word32(address, value);
    }
    swd_release_pins();
    return err;
}

esp_err_t swd_flash_embedded_firmware(void)
{
    const uint8_t *firmware = swd_embedded_firmware_start();
    const uint8_t *stub = swd_embedded_stub_start();
    const size_t firmware_size = swd_embedded_firmware_size();
    const size_t stub_size = swd_embedded_stub_size();
    swd_flash_stub_ctx_t ctx;
    uint32_t stub_status = 0;
    esp_err_t err;

    if ((firmware == NULL) || (firmware_size == 0) || (stub == NULL) || (stub_size == 0)) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "rpflash: halting core1");

    err = swd_halt_core(RP2040_TARGETSEL_CORE1, NULL);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "rpflash: attaching core0");
    swd_claim_pins();
    err = swd_attach_target(RP2040_TARGETSEL_CORE0);
    if (err == ESP_OK) {
        err = swd_halt_attached_core(NULL);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "rpflash: uploading %u-byte stub", (unsigned)stub_size);
        err = swd_mem_write_block_with_progress(SWD_RP2040_STUB_BASE, stub, stub_size, "rpflash: stub upload");
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "rpflash: uploading %u-byte image to SRAM", (unsigned)firmware_size);
        err = swd_mem_write_block_with_progress(SWD_RP2040_STUB_DATA_BASE, firmware, firmware_size, "rpflash: image upload");
    }
    if (err == ESP_OK) {
        ctx.op = SWD_STUB_OP_PROGRAM_IMAGE;
        ctx.status = SWD_STUB_STATUS_IDLE;
        ctx.flash_offset = 0;
        ctx.count = (uint32_t)firmware_size;
        ctx.data_addr = SWD_RP2040_STUB_DATA_BASE;
        err = swd_mem_write_block(SWD_RP2040_STUB_CTX_ADDR, (const uint8_t *)&ctx, sizeof(ctx));
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "rpflash: erase+program %u bytes", (unsigned)firmware_size);
        err = swd_run_stub_attached(SWD_RP2040_STUB_CTX_ADDR, &stub_status);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "rpflash: verifying %u bytes", (unsigned)firmware_size);
        err = swd_mem_verify_block_with_progress(SWD_RP2040_XIP_BASE, firmware, firmware_size, "rpflash: verify");
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "rpflash: reset target");
        (void)swd_reset_system_attached();
    }
    swd_release_pins();
    return err;
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

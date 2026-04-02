#ifndef BONANZA_TEST_SWD_H_
#define BONANZA_TEST_SWD_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define RP2040_TARGETSEL_CORE0 0x01002927u
#define RP2040_TARGETSEL_CORE1 0x11002927u
#define RP2040_TARGETSEL_RESCUE 0xF1002927u

typedef struct {
    uint32_t targetsel;
    uint32_t dpidr;
    uint8_t ack;
} swd_id_result_t;

esp_err_t swd_init(void);
esp_err_t swd_read_target_id(uint32_t targetsel, swd_id_result_t *result);
esp_err_t swd_halt_core(uint32_t targetsel, uint32_t *dhcsr_out);
esp_err_t swd_read_word32(uint32_t targetsel, uint32_t address, uint32_t *value);
esp_err_t swd_write_word32(uint32_t targetsel, uint32_t address, uint32_t value);
esp_err_t swd_flash_embedded_firmware(void);
const char *swd_ack_name(uint8_t ack);

const uint8_t *swd_embedded_firmware_start(void);
size_t swd_embedded_firmware_size(void);

#endif

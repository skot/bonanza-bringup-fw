#include <stddef.h>
#include <stdint.h>

#define ROM_TABLE_CODE(c1, c2) ((uint32_t)(c1) | ((uint32_t)(c2) << 8))

#define ROM_FUNC_CONNECT_INTERNAL_FLASH ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP         ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_RANGE_ERASE      ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM    ROM_TABLE_CODE('R', 'P')
#define ROM_FUNC_FLASH_FLUSH_CACHE      ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_FLASH_ENTER_CMD_XIP    ROM_TABLE_CODE('C', 'X')

#define FLASH_BLOCK_SIZE  (1u << 16)
#define FLASH_BLOCK_ERASE_CMD 0xd8u

#define FLASH_STUB_OP_ERASE   1u
#define FLASH_STUB_OP_PROGRAM 2u
#define FLASH_STUB_OP_PROGRAM_IMAGE 3u

#define FLASH_STUB_STATUS_IDLE            0u
#define FLASH_STUB_STATUS_BUSY            1u
#define FLASH_STUB_STATUS_OK              2u
#define FLASH_STUB_STATUS_ERR_BAD_OP      0xE001u
#define FLASH_STUB_STATUS_ERR_LOOKUP      0xE002u

typedef void (*rom_connect_internal_flash_fn)(void);
typedef void (*rom_flash_exit_xip_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t, size_t, uint32_t, uint8_t);
typedef void (*rom_flash_range_program_fn)(uint32_t, const uint8_t *, size_t);
typedef void (*rom_flash_flush_cache_fn)(void);
typedef void (*rom_flash_enter_cmd_xip_fn)(void);
typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);

typedef struct {
    uint32_t op;
    uint32_t status;
    uint32_t stage;
    uint32_t progress;
    uint32_t flash_offset;
    uint32_t count;
    uint32_t data_addr;
} flash_stub_ctx_t;

#define FLASH_STUB_STAGE_IDLE    0u
#define FLASH_STUB_STAGE_ERASE   1u
#define FLASH_STUB_STAGE_PROGRAM 2u
#define FLASH_STUB_STAGE_DONE    3u

static inline void *rom_hword_as_ptr(uint32_t rom_address)
{
    return (void *)(uintptr_t)(*(volatile uint16_t *)(uintptr_t)rom_address);
}

static inline void *rom_func_lookup_inline(uint32_t code)
{
    rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn)rom_hword_as_ptr(0x18);
    uint16_t *func_table = (uint16_t *)rom_hword_as_ptr(0x14);
    return rom_table_lookup(func_table, code);
}

__attribute__((section(".text.entry"), noreturn))
void rp2040_flash_stub_entry(flash_stub_ctx_t *ctx)
{
    rom_connect_internal_flash_fn connect_internal_flash;
    rom_flash_exit_xip_fn flash_exit_xip;
    rom_flash_flush_cache_fn flash_flush_cache;
    rom_flash_enter_cmd_xip_fn flash_enter_cmd_xip;
    rom_flash_range_erase_fn flash_range_erase;
    rom_flash_range_program_fn flash_range_program;

    __asm volatile("cpsid i" ::: "memory");

    ctx->status = FLASH_STUB_STATUS_BUSY;
    ctx->stage = FLASH_STUB_STAGE_IDLE;
    ctx->progress = 0;

    connect_internal_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    flash_exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    flash_enter_cmd_xip = (rom_flash_enter_cmd_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    flash_range_erase = (rom_flash_range_erase_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
    flash_range_program = (rom_flash_range_program_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);

    if (!connect_internal_flash || !flash_exit_xip || !flash_flush_cache || !flash_enter_cmd_xip ||
        !flash_range_erase || !flash_range_program) {
        ctx->status = FLASH_STUB_STATUS_ERR_LOOKUP;
        __asm volatile("bkpt #0");
        while (1) {
        }
    }

    connect_internal_flash();
    flash_exit_xip();

    if (ctx->op == FLASH_STUB_OP_ERASE) {
        ctx->stage = FLASH_STUB_STAGE_ERASE;
        flash_range_erase(ctx->flash_offset, ctx->count, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
    } else if (ctx->op == FLASH_STUB_OP_PROGRAM) {
        ctx->stage = FLASH_STUB_STAGE_PROGRAM;
        flash_range_program(ctx->flash_offset, (const uint8_t *)(uintptr_t)ctx->data_addr, ctx->count);
        ctx->progress = ctx->count;
    } else if (ctx->op == FLASH_STUB_OP_PROGRAM_IMAGE) {
        uint32_t erase_count = (ctx->count + 4095u) & ~4095u;
        const uint8_t *data = (const uint8_t *)(uintptr_t)ctx->data_addr;

        ctx->stage = FLASH_STUB_STAGE_ERASE;
        flash_range_erase(ctx->flash_offset, erase_count, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
        ctx->stage = FLASH_STUB_STAGE_PROGRAM;
        for (uint32_t offset = 0; offset < ctx->count; offset += 256u) {
            flash_range_program(ctx->flash_offset + offset, data + offset, 256u);
            ctx->progress = offset + 256u;
        }
    } else {
        ctx->status = FLASH_STUB_STATUS_ERR_BAD_OP;
        flash_flush_cache();
        flash_enter_cmd_xip();
        __asm volatile("bkpt #0");
        while (1) {
        }
    }

    flash_flush_cache();
    flash_enter_cmd_xip();
    ctx->status = FLASH_STUB_STATUS_OK;
    ctx->stage = FLASH_STUB_STAGE_DONE;

    __asm volatile("bkpt #0");
    while (1) {
    }
}

idf_component_register(
    SRCS
        "main.c"
        "swd.c"
        "tps546.c"
    INCLUDE_DIRS
        "."
)

set(BONANZA_RP2040_FIRMWARE_BIN_INPUT "$ENV{BONANZA_RP2040_FIRMWARE_BIN}" CACHE FILEPATH "Path to a prebuilt RP2040 firmware .bin to embed")
set(BONANZA_RP2040_FIRMWARE_REPO "$ENV{BONANZA_RP2040_FIRMWARE_REPO}" CACHE PATH "Path to an RP2040 firmware Cargo repo to build and embed")
set(BONANZA_RP2040_FIRMWARE_BIN "${CMAKE_CURRENT_BINARY_DIR}/rp2040_firmware.bin")
set(BONANZA_RP2040_FLASH_STUB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/rp2040_flash_stub.c")
set(BONANZA_RP2040_FLASH_STUB_LD "${CMAKE_CURRENT_SOURCE_DIR}/rp2040_flash_stub.ld")
set(BONANZA_RP2040_FLASH_STUB_ELF "${CMAKE_CURRENT_BINARY_DIR}/rp2040_flash_stub.elf")
set(BONANZA_RP2040_FLASH_STUB_BIN "${CMAKE_CURRENT_BINARY_DIR}/rp2040_flash_stub.bin")

if(BONANZA_RP2040_FIRMWARE_BIN_INPUT)
    add_custom_command(
        OUTPUT "${BONANZA_RP2040_FIRMWARE_BIN}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${BONANZA_RP2040_FIRMWARE_BIN_INPUT}" "${BONANZA_RP2040_FIRMWARE_BIN}"
        DEPENDS "${BONANZA_RP2040_FIRMWARE_BIN_INPUT}"
        COMMENT "Embedding prebuilt RP2040 firmware image"
        VERBATIM
    )
elseif(BONANZA_RP2040_FIRMWARE_REPO)
    set(BONANZA_RP2040_FIRMWARE_ELF "${BONANZA_RP2040_FIRMWARE_REPO}/target/thumbv6m-none-eabi/release/firmware")

    file(GLOB_RECURSE BONANZA_RP2040_FIRMWARE_SOURCES
        "${BONANZA_RP2040_FIRMWARE_REPO}/src/*.rs"
        "${BONANZA_RP2040_FIRMWARE_REPO}/Cargo.toml"
        "${BONANZA_RP2040_FIRMWARE_REPO}/Cargo.lock"
        "${BONANZA_RP2040_FIRMWARE_REPO}/build.rs"
        "${BONANZA_RP2040_FIRMWARE_REPO}/memory.x"
    )

    add_custom_command(
        OUTPUT "${BONANZA_RP2040_FIRMWARE_BIN}"
        COMMAND cargo build --release
        COMMAND arm-none-eabi-objcopy -O binary "${BONANZA_RP2040_FIRMWARE_ELF}" "${BONANZA_RP2040_FIRMWARE_BIN}"
        WORKING_DIRECTORY "${BONANZA_RP2040_FIRMWARE_REPO}"
        DEPENDS ${BONANZA_RP2040_FIRMWARE_SOURCES}
        COMMENT "Building and exporting embedded RP2040 firmware image"
        VERBATIM
    )
else()
    message(FATAL_ERROR
        "No RP2040 firmware source configured. Set BONANZA_RP2040_FIRMWARE_BIN_INPUT to a prebuilt .bin, "
        "or set BONANZA_RP2040_FIRMWARE_REPO to a Cargo repo that builds the RP2040 firmware.")
endif()

target_add_binary_data(${COMPONENT_LIB} "${BONANZA_RP2040_FIRMWARE_BIN}" BINARY RENAME_TO "rp2040_firmware_bin")

add_custom_command(
    OUTPUT "${BONANZA_RP2040_FLASH_STUB_BIN}"
    COMMAND arm-none-eabi-gcc
            -mcpu=cortex-m0plus
            -mthumb
            -Os
            -ffreestanding
            -fno-builtin
            -fno-stack-protector
            -fdata-sections
            -ffunction-sections
            -nostartfiles
            -nostdlib
            -Wl,--gc-sections
            -Wl,--build-id=none
            -Wl,-T,${BONANZA_RP2040_FLASH_STUB_LD}
            -o "${BONANZA_RP2040_FLASH_STUB_ELF}"
            "${BONANZA_RP2040_FLASH_STUB_SRC}"
    COMMAND arm-none-eabi-objcopy -O binary "${BONANZA_RP2040_FLASH_STUB_ELF}" "${BONANZA_RP2040_FLASH_STUB_BIN}"
    DEPENDS "${BONANZA_RP2040_FLASH_STUB_SRC}" "${BONANZA_RP2040_FLASH_STUB_LD}"
    COMMENT "Building embedded RP2040 flash stub"
    VERBATIM
)

target_add_binary_data(${COMPONENT_LIB} "${BONANZA_RP2040_FLASH_STUB_BIN}" BINARY RENAME_TO "rp2040_flash_stub_bin")

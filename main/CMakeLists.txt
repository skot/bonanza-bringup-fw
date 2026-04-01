idf_component_register(
    SRCS
        "main.c"
        "swd.c"
        "tps546.c"
    INCLUDE_DIRS
        "."
)

set(BONANZA_RP2040_FIRMWARE_REPO "/Users/skot/Bitcoin/bitaxe-raw/bitaxe-raw-bonanza" CACHE PATH "Path to the bitaxe-raw-bonanza RP2040 firmware repo")
set(BONANZA_RP2040_FIRMWARE_BIN "${CMAKE_CURRENT_BINARY_DIR}/bitaxe-raw-bonanza-rp2040.bin")
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

target_add_binary_data(${COMPONENT_LIB} "${BONANZA_RP2040_FIRMWARE_BIN}" BINARY RENAME_TO "rp2040_firmware_bin")

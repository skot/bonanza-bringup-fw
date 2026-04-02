# bonanza-test

`bonanza-test` is an ESP-IDF bring-up and factory-test firmware for the bitaxeBonanza board. It runs on the ESP32-S3 and acts as a board-side utility for:

- validating an embedded RP2040 firmware image over the control and data UARTs
- bringing up and testing the onboard TPS546D24S voltage regulator
- sending low-level BZM2 commands to the ASIC chain and helping address up to four chips
- talking to the RP2040 over SWD and flashing a compile-time embedded RP2040 firmware image

In practice, this project is meant to be a single Bonanza board bring-up tool rather than just a UART test harness.

## Features

- RP2040 control-UART testing for GPIO, reset, 5V enable, and fan control
- RP2040 data-UART testing for Bonanza 9-bit transport validation
- TPS546D24S PMBus configuration, telemetry, status, and bring-up commands
- BZM2 raw register access, `NOOP`, chain addressing, and chip probing helpers
- ESP32-driven RP2040 SWD attach, halt, memory read/write, and in-system reflashing

It uses two ESP32-S3 UARTs:

- `control UART`: talks to RP2040 `UART0` control serial
- `data UART`: talks to RP2040 `UART1` data serial

The current Bonanza board pin mapping is:

| Function | ESP32-S3 pin | RP2040 side |
|----------|--------------|-------------|
| Control TX | `UART0 TX` / `GPIO43` | `UART0 RX` / `GPIO1` |
| Control RX | `UART0 RX` / `GPIO44` | `UART0 TX` / `GPIO0` |
| Data TX | `UART1 TX` / `GPIO17` | `UART1 RX` / `GPIO5` |
| Data RX | `UART1 RX` / `GPIO18` | `UART1 TX` / `GPIO4` |

The current TPS546D24S bring-up pin mapping is:

| Function | ESP32-S3 pin |
|----------|--------------|
| I2C SDA | `GPIO47` |
| I2C SCL | `GPIO48` |
| VR enable | `GPIO10` |
| PGOOD | `GPIO11` |
| RP2040 SWCLK | `GPIO1` |
| RP2040 SWDIO | `GPIO2` |

Baudrates:

- Control UART: `115200`
- Data UART: `5000000`

## Build

`bonanza-test` embeds an RP2040 firmware image at build time. You can provide that image in either of these ways:

1. Use a prebuilt RP2040 `.bin`
2. Point the build at an RP2040 Cargo repo and let `bonanza-test` build it

### Option 1: Embed A Prebuilt `.bin`

```sh
cd /Users/skot/Bitcoin/ESP-Miner/bonanza-test
source "$IDF_PATH/export.sh"
idf.py set-target esp32s3
idf.py -DBONANZA_RP2040_FIRMWARE_BIN_INPUT=/absolute/path/to/firmware.bin build
```

### Option 2: Build From An RP2040 Cargo Repo

```sh
cd /Users/skot/Bitcoin/ESP-Miner/bonanza-test
source "$IDF_PATH/export.sh"
idf.py set-target esp32s3
idf.py -DBONANZA_RP2040_FIRMWARE_REPO=/absolute/path/to/rp2040-firmware-repo build
```

The repo-based path should contain a release ELF at:

- `target/thumbv6m-none-eabi/release/firmware`

after `cargo build --release`.

You can also set either input with environment variables instead of `-D` flags:

```sh
export BONANZA_RP2040_FIRMWARE_BIN=/absolute/path/to/firmware.bin
```

or:

```sh
export BONANZA_RP2040_FIRMWARE_REPO=/absolute/path/to/rp2040-firmware-repo
```

Then build normally:

```sh
cd /Users/skot/Bitcoin/ESP-Miner/bonanza-test
source "$IDF_PATH/export.sh"
idf.py build
```

If you change the RP2040 firmware source after a previous configure, rerun `idf.py reconfigure build` or pass the new `-D...` option again so CMake updates the cached value.

## Flash

```sh
source "$IDF_PATH/export.sh"
idf.py -p /dev/tty.usbmodem* flash monitor
```

Adjust the serial port as needed for your ESP32-S3 board.

## Console Commands

Type commands into the ESP-IDF monitor:

- `help`
- `gpio`
- `5v 0`
- `5v 1`
- `rst 0`
- `rst 1`
- `fan 35`
- `tach`
- `vr help`
- `vr probe`
- `vr pgood`
- `vr status`
- `vr telem`
- `vr clear`
- `vr cfg read`
- `vr cfg write`
- `vr bringup`
- `vr pin 1`
- `vr op 1`
- `rpflash info`
- `rpflash write`
- `rpswd id`
- `rpswd halt core0`
- `rpswd read32 0xE000ED00 core0`
- `rpswd write32 0x20000000 0x12345678 core0`
- `BZM_sendnoop 0xFA`
- `BZM_readreg 0xFA 0xFFF 0x0B 4`
- `BZM_writereg 0xFA 0xFFF 0x0B 0x42 0x00 0x00 0x00`
- `BZM_addr4`
- `BZM_probeall`
- `pattern`
- `send9 0x155 0x0aa 0x1ff`
- `raw 55 01 aa 00`
- `flush`

`pattern` sends a small set of 9-bit test words over the data UART using the Bonanza byte-pair encoding.

## Bring-Up Areas

The firmware is organized around four practical bring-up areas:

- `control/data UART`: validate the RP2040 serial firmware paths and Bonanza 9-bit data encoding
- `vr`: configure, enable, and inspect the TPS546D24S regulator
- `BZM_*`: exercise the ASIC chain using low-level BZM2 commands
- `rpswd` / `rpflash`: inspect and reflash the RP2040 directly from the ESP32-S3

The `vr` commands are intended for staged regulator bring-up:

- `vr probe` reads the PMBus device ID at `0x24`
- `vr pgood` checks the external `PGOOD` and enable-pin state
- `vr status` reads PMBus fault/status registers
- `vr telem` reads VIN, VOUT, IOUT, and temperature
- `vr clear` issues `CLEAR_FAULTS`
- `vr cfg read` reads back the key PMBus config registers written by the harness
- `vr cfg write` writes the `TPS546_CONFIG_BIRDS` PMBus register set from the Python bring-up
- `vr bringup` follows the Python bring-up flow: force off, set `ON_OFF_CONFIG`, clear faults, write `TPS546_CONFIG_BIRDS`, assert enable, then set `OPERATION=ON`
- `vr pin <0|1>` drives the external enable pin on `GPIO10`
- `vr op <0|1>` writes the PMBus `OPERATION` register

The harness does not auto-enable the regulator at boot. `GPIO10` is driven low during initialization so regulator bring-up stays explicit.

## RP2040 SWD

The harness includes an ESP32-driven RP2040 SWD and flashing path:

- `rpflash info` shows the compile-time embedded `bitaxe-raw-bonanza` RP2040 firmware image
- `rpflash write` halts both RP2040 cores, uploads a small SRAM flash stub, uploads the RP2040 firmware image into SRAM, erases and programs flash, verifies the result, and issues a system reset
- `rpswd id` bit-bangs SWD on `GPIO1`/`GPIO2`, selects RP2040 multidrop targets, and reads each target `DPIDR`
- `rpswd halt [core0|core1]` halts a core and shows `DHCSR`
- `rpswd read32 <addr> [core0|core1]` reads a 32-bit word using the MEM-AP
- `rpswd write32 <addr> <value> [core0|core1]` writes a 32-bit word using the MEM-AP

This lets the ESP32-S3 act as a purpose-built RP2040 firmware loader for Bonanza board bring-up without needing an external probe attached during normal operation.

The embedded RP2040 firmware image can come from either:

- `BONANZA_RP2040_FIRMWARE_BIN_INPUT`: a prebuilt `.bin`
- `BONANZA_RP2040_FIRMWARE_REPO`: a Cargo repo that `bonanza-test` builds and exports to `.bin`

The embedded SRAM flash helper is also built at ESP-IDF build time from:

- [`main/rp2040_flash_stub.c`](main/rp2040_flash_stub.c)

using the local `arm-none-eabi-gcc` toolchain.

## ASIC Commands

The harness also includes direct BZM2 command helpers on the data UART, matching the Python bring-up scripts and intended for early ASIC-chain validation:

- `BZM_sendnoop <asic>`
- `BZM_readreg <asic> <engine_id> <offset> <count>`
- `BZM_writereg <asic> <engine_id> <offset> <byte...>`
- `BZM_temp <asic> [thermal_div]`
- `BZM_addr4`
- `BZM_probeall`

Examples:

- `BZM_sendnoop 0xFA`
- `BZM_readreg 0xFA 0xFFF 0x0B 4`
- `BZM_writereg 0xFA 0xFFF 0x0B 0x42 0x00 0x00 0x00`
- `BZM_temp 0x42`
- `BZM_addr4`
- `BZM_probeall`

These commands send the same 9-bit word sequences as the Python `bzm2.py` helpers and print the returned 9-bit words from the ASIC path.

`BZM_temp` temporarily enables the on-die thermal sensor described in datasheet section 9, waits for the SAR conversion, reads register `0x32`, converts the returned temp code to Celsius, and restores the previous sensor configuration. On Bonanza, the optional `thermal_div` defaults to `0x08`, which matches the confirmed `50 MHz` ASIC reference clock.

`BZM_addr4` is a board bring-up helper that follows the Python `asic_comm.py` sequence for up to four chips:

- pulse ASIC reset through the RP2040 control UART
- probe for an unaddressed ASIC at `0xFA`
- program IDs `0x42`, `0x43`, `0x44`, and `0x45` one at a time
- read back register `0x0B` (`ASIC_ID`) after each write
- verify each newly assigned ID with a `NOOP`

It stops on the first ambiguous step and prints a summary, so if only some chips are alive you can still see how far the chain got.

`BZM_probeall` is a non-destructive follow-up that sends `NOOP` to `0x42`, `0x43`, `0x44`, and `0x45` and reports which addressed chips are responding.

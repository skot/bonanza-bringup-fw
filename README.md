# bonanza-test

Small ESP-IDF test firmware for validating the `bitaxe-raw-bonanza` RP2040 firmware on a bitaxeBonanza board and for bringing up the onboard TPS546D24S regulator from the ESP32-S3 side.

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

Baudrates:

- Control UART: `115200`
- Data UART: `5000000`

## Build

```sh
cd /Users/skot/Bitcoin/ESP-Miner/bonanza-test
source "$IDF_PATH/export.sh"
idf.py set-target esp32s3
idf.py build
```

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

## ASIC Commands

The harness also includes direct BZM2 command helpers on the data UART, matching the Python bring-up scripts:

- `BZM_sendnoop <asic>`
- `BZM_readreg <asic> <engine_id> <offset> <count>`
- `BZM_writereg <asic> <engine_id> <offset> <byte...>`
- `BZM_addr4`
- `BZM_probeall`

Examples:

- `BZM_sendnoop 0xFA`
- `BZM_readreg 0xFA 0xFFF 0x0B 4`
- `BZM_writereg 0xFA 0xFFF 0x0B 0x42 0x00 0x00 0x00`
- `BZM_addr4`
- `BZM_probeall`

These commands send the same 9-bit word sequences as the Python `bzm2.py` helpers and print the returned 9-bit words from the ASIC path.

`BZM_addr4` is a board bring-up helper that follows the Python `asic_comm.py` sequence for up to four chips:

- pulse ASIC reset through the RP2040 control UART
- probe for an unaddressed ASIC at `0xFA`
- program IDs `0x42`, `0x43`, `0x44`, and `0x45` one at a time
- read back register `0x0B` (`ASIC_ID`) after each write
- verify each newly assigned ID with a `NOOP`

It stops on the first ambiguous step and prints a summary, so if only some chips are alive you can still see how far the chain got.

`BZM_probeall` is a non-destructive follow-up that sends `NOOP` to `0x42`, `0x43`, `0x44`, and `0x45` and reports which addressed chips are responding.

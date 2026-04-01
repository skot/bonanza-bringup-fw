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

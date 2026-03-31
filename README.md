# bonanza-test

Small ESP-IDF test firmware for validating the `bitaxe-raw-bonanza` RP2040 firmware on a bitaxeBonanza board.

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
- `pattern`
- `send9 0x155 0x0aa 0x1ff`
- `raw 55 01 aa 00`
- `flush`

`pattern` sends a small set of 9-bit test words over the data UART using the Bonanza byte-pair encoding.

For early bring-up, this test firmware also sends the default `pattern` automatically once per second after boot so the RP2040 RTT bridge counters can be checked without needing a working console first.

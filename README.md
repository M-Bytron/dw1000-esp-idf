# DWM1000-A — ESP-IDF DW1000 (Device A / tag)

ESP-IDF port of the PlatformIO / Arduino project
`arduino-dw1000/DW1000_Device_A` (ranging initiator / tag).

## Status: Step 1 — module detection only

Currently `main/main.c` (plain C) **only initializes the DW1000 driver and
probes the module** (reads the device ID / EUI / mode over SPI) to confirm it
is connected.

The full two-way ranging code (`POLL -> POLL_ACK -> RANGE -> RANGE_REPORT`)
from the PlatformIO project will be added as a later step, exposed through the
same C API.

## Repository layout

```
DWM1000-A/
├── CMakeLists.txt                  project file (project DWM1000-A)
├── sdkconfig.defaults              common sdkconfig defaults
├── main/
│   ├── CMakeLists.txt
│   └── main.c                      application (Device A), plain C
└── components/
    ├── arduino_shim/               tiny Arduino-compat layer on top of ESP-IDF
    │   ├── include/Arduino.h       GPIO, time, interrupts, types, F()
    │   ├── include/SPI.h           SPI shim (ESP-IDF SPI master)
    │   ├── include/WString.h       minimal String (only used by the library)
    │   └── src/                    Arduino.cpp, SPI.cpp, Interrupt.cpp
    └── arduino_dw1000/             vendored arduino-dw1000 library
        ├── include/dw1000_c.h      plain C API for the application (main.c)
        └── src/                    DW1000, DW1000Time, dw1000_c.cpp (compiled);
                                    DW1000Mac/Device/Ranging (not compiled)
```

### How the port works

The `arduino-dw1000` library only depends on a handful of Arduino APIs. The
`arduino_shim` component implements exactly those on native ESP-IDF drivers:

| Arduino API used by the library | ESP-IDF implementation |
| ------------------------------- | ----------------------- |
| `SPI.begin / beginTransaction / transfer / endTransaction / end` | SPI master driver (`SPI2_HOST`), 2 MHz + 16 MHz devices, CS toggled manually |
| `pinMode / digitalWrite / digitalRead` | GPIO driver |
| `attachInterrupt / detachInterrupt` | GPIO ISR service + deferred handler task (SPI is not done from ISR context) |
| `delay / delayMicroseconds / millis / micros` | FreeRTOS + `esp_timer` + `esp_rom_delay_us` |
| `byte / boolean / String / F()` | small typedefs / minimal `String` shim |

Only `DW1000.cpp` and `DW1000Time.cpp` are compiled — the same low-level API
your PlatformIO code uses (`DW1000.setSpiPins`, `DW1000.begin`, `newTransmit`,
`setData`, `startTransmit`, `newReceive`, `startReceive`, `DW1000Time`, …).
The `DW1000Mac` / `DW1000Ranging` / `DW1000Device` files are not needed and
are intentionally not compiled.

The application itself is written in **plain C** (`main/main.c`). It talks to
the C++ driver through a small C wrapper: `components/arduino_dw1000`
exposes `dw1000_c.h` / `dw1000_c.cpp`, which bridge to the C++ `DW1000` class
with `extern "C"` functions (`dw1000_set_spi_pins`, `dw1000_begin`, …).
Adding a new driver feature means adding one line to the C wrapper and one
prototype to `dw1000_c.h`, then calling it from C.

## Hardware

* Adafruit Feather ESP32 (or any ESP32) + a DW1000 / DWM1000 UWB module.
* Module powered with 3.3 V.

Wiring:

| ESP32 pin | DW1000 pin |
| --------- | ---------- |
| GPIO 25   | SPICLK     |
| GPIO 26   | SPIMISO    |
| GPIO 27   | SPIMOSI    |
| GPIO 14   | SPICSn     |
| GPIO 13   | IRQ        |
| GPIO 32   | RSTn       |

Pin assignments are defined at the top of `main/main.c` (`PIN_SCK`, …).

## Prerequisites

* ESP-IDF **v5.x** (the SPI master `SPI_DMA_CH_AUTO` API and `esp_rom_delay_us`
  are used).

## Build, flash and monitor

```bash
idf.py set-target esp32        # once per project
idf.py -p COMx flash monitor   # change COMx to your port
```

Exit the monitor with `Ctrl-]`.

Expected output:

```
==================================
============ DEVICE A ============
==================================
Driver initialized. Probing the DW1000 ...
==============================================
>>>  DW1000 MODULE DETECTED - SPI OK!  <<<
==============================================
Device ID  : DECA - model: 1, version: 0, revision: 0
Unique ID  : XX:XX:XX:XX:XX:XX:XX:XX
Net/Addr   : PAN: 000A, Short Address: 0005
Device mode: Data rate: 110 kb/s, PRF: 16 MHz, Preamble: 2048 symbols (code #4), Channel: #5
Ranging only - normal broadcast sending disabled.
RANGE OK: 2.00 m
RANGE OK: 2.00 m
...
```

If `NO DW1000 RESPONSE` is printed, check the wiring and that the module is
powered with 3.3 V.

## Antenna delay calibration

1. Place Device A and Device B **exactly 2.00 m apart**, clear line of sight.
2. Flash this firmware to both devices.
3. While watching `RANGE OK` in the monitor, tune **both** devices until they
   read 2.00 m:
   * `1` — increase antenna delay
   * `2` — decrease antenna delay
   * `0` — reset to factory default
   * `9` — show current antenna delay
4. Set `ANTENNA_DELAY` in `main/main.cpp` to the tuned value (same on both
   devices); the tuning block can then be removed.

## Notes / limitations

* The DW1000 IRQ handler is deferred to a high-priority FreeRTOS task (SPI is
  not performed from a raw ISR context).
* `noInterrupts() / interrupts()` are no-ops in this shim; they are only used
  by `DW1000Mac`, which is not part of this port.
* Device B runs the matching receiver firmware (`DW1000_Device_B`).

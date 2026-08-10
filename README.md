<div align="center">

# DW1000 · ESP-IDF UWB Library

**Pure‑C driver + ready‑to‑use DS‑TWR ranging engine for the
[Decawave DW1000 / DWM1000](https://www.qorvo.com/products/p/DW1000) Ultra‑Wideband transceiver, built for the ESP32 / ESP-IDF.**

Plain C. No Arduino, no C++, no external library — just the ESP‑IDF SPI / GPIO drivers.

</div>

---

## ✨ Features

- **100 % pure C** — `my_dw1000.c` (high‑level API) + `dw1000_lowlevel.c` (SPI/register driver), on top of the ESP‑IDF `spi_master` and `gpio` drivers.
- **Ready‑to‑use asymmetric DS‑TWR ranging** — call one function and get the distance in meters: `dw1000_run_tag()` / `dw1000_run_anchor()`.
- **Joint antenna‑delay calibration** — `dw1000_calibrate_antenna_delay_iterative()` converges *both* modules to one shared value over the air (no overshoot, no sign‑flip).
- **Pairing** — range only with a specific module using its unique ESP32 BLE MAC (`dw1000_set_peer_eui()`).
- **Raw frame TX / RX** — send and receive arbitrary IEEE 802.15.4 frames for custom protocols.
- **Full radio control** — channel, data rate, PRF, preamble length/code, network ID, short address, EUI, antenna delay.
- **Diagnostics** — receive power (dBm), first‑path power, RX quality, on‑chip temperature and battery voltage.
- **Interrupt driven** — events (sent / received / failed / timeout) dispatched to callbacks from a high‑priority FreeRTOS task.

---

## 📦 Repository layout

```
DWM1000/
├── components/
│   └── DW1000/                  <-- the reusable library (copy this folder)
│       ├── my_dw1000.h/.c       public API + DS‑TWR ranging + calibration
│       ├── dw1000_lowlevel.h/.c low‑level SPI / register driver
│       ├── dw1000_regs.h        register & encoding constants
│       └── CMakeLists.txt
├── main/
│   └── main.c                   minimal Device A (TAG) example
├── Examples/
│   ├── Device-A/                standalone project: TAG (initiator, ranges)
│   └── Device-B/                standalone project: ANCHOR (responder)
└── README.md
```

---

## 🧰 Requirements

| Thing | Value |
| --- | --- |
| MCU | ESP32 (works on other ESP‑IDF targets with matching pins) |
| ESP‑IDF | 5.x |
| Module | any DW1000 / DWM1000 / DWM1001‑style module (SPI) |
| Host | Windows / Linux / macOS (ESP‑IDF toolchain) |

---

## 🔌 Wiring

A typical DW1000 module is SPI‑controlled. Example pinout used by all examples
(Adafruit Feather ESP32):

| DW1000 pin | ESP32 GPIO |
| --- | --- |
| `SPICLK` | 25 |
| `SPIMISO` | 26 |
| `SPIMOSI` | 27 |
| `SPICSn` | 14 |
| `IRQ` | 13 |
| `RSTn` | 32 |

> ⚠️ The module **must** be powered from 3.3 V. `RSTn` is only pulsed low
> during reset and left floating otherwise.

---

## 🚀 Quick start

### 1. Add the library to your project

Copy the `components/DW1000` folder into your project's `components/`
directory, then require it from your app:

```cmake
# main/CMakeLists.txt
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES DW1000
)
```

### 2. Build one of the examples

Each example under `Examples/` is a complete, self‑contained ESP‑IDF project:

```bash
cd Examples/Device-A            # or Device-B
idf.py set-target esp32
idf.py build flash monitor
```

Or build the minimal example in the repo root:

```bash
idf.py set-target esp32
idf.py build flash monitor
```

### 3. Basic flow (always the same shape)

```c
dw1000_init(SCK, MISO, MOSI, CS, IRQ, RST);   // 1. init SPI + reset + LDE
dw1000_probe();                                // 2. verify the module answers
dw1000_config(pan, short_addr, mode, chan, antenna_delay); // 3. configure radio
dw1000_set_peer_eui(peer_ble_mac);             // 4. (optional) pair with a peer
dw1000_run_tag(IRQ, TIMEOUT_MS, on_distance);  // 5a. tag: range continuously
// or
dw1000_run_anchor(IRQ);                        // 5b. anchor: answer forever
```

---

## 📚 How to use

### Init / probe

| Function | Purpose |
| --- | --- |
| `dw1000_init(sck, miso, mosi, cs, irq, rst)` | Initialize the SPI bus, reset the chip, load the LDE microcode (required before RX). |
| `bool dw1000_probe(void)` | Read the device ID; returns `true` if a DW1000 was found (`0xDECA0130`). |
| `dw1000_print_device_info()` | Print ID, EUI, PAN/address and radio mode to the log. |

```c
if (!dw1000_probe()) {
    ESP_LOGE("APP", "DW1000 not detected - check wiring!");
    return;
}
```

### Radio configuration

`dw1000_config()` is a one‑call convenience that loads the defaults and
applies everything:

```c
dw1000_config(
    0xDECA,                              // PAN / network ID
    0x1001,                              // 16‑bit short address
    DW1000_MODE_SHORTDATA_FAST_ACCURACY, // {data rate, PRF, preamble}
    DW1000_CHANNEL_5,                    // RF channel
    antenna_delay                        // calibrated antenna delay (ticks)
);
```

**Both boards must use the same channel and mode** for ranging.

Available mode presets (arrays of `{ data_rate, pulse_frequency, preamble_length }`):

| Preset | Data rate | PRF | Preamble |
| --- | --- | --- | --- |
| `DW1000_MODE_LONGDATA_RANGE_LOWPOWER` | 110 kbps | 16 MHz | 2048 |
| `DW1000_MODE_SHORTDATA_FAST_LOWPOWER` | 6.8 Mbps | 16 MHz | 128 |
| `DW1000_MODE_LONGDATA_FAST_LOWPOWER` | 6.8 Mbps | 16 MHz | 1024 |
| `DW1000_MODE_SHORTDATA_FAST_ACCURACY` | 6.8 Mbps | 64 MHz | 128 |
| `DW1000_MODE_LONGDATA_FAST_ACCURACY` | 6.8 Mbps | 64 MHz | 1024 |
| `DW1000_MODE_LONGDATA_RANGE_ACCURACY` | 110 kbps | 64 MHz | 2048 |

Channels: `DW1000_CHANNEL_1` … `DW1000_CHANNEL_7` (use `DW1000_CHANNEL_5`).

For fine‑grained control you can also use the two‑phase configuration API:

```c
dw1000_begin_config();                 // idle + snapshot current chip state
dw1000_set_channel(DW1000_CHANNEL_5);
dw1000_set_data_rate(DW1000_RATE_6800KBPS);
dw1000_set_pulse_frequency(DW1000_PRF_64MHZ);
dw1000_set_preamble_length(DW1000_PREAMBLE_LEN_128);
dw1000_set_antenna_delay(antenna_delay);
dw1000_commit_config();                // write everything + re‑tune the radio
```

### Pairing

Each board prints its unique **ESP32 BLE MAC** at boot (e.g. `My BLE MAC: D4:8C:49:E3:A4:6E`).
Pairing is **reciprocal**: every board must be told the *other* board's BLE MAC.

```c
/* Device A: pair with Device B's BLE MAC (6 bytes, MSB first) */
static const uint8_t peer_eui[6] = {0xD4, 0x8C, 0x49, 0xE3, 0xA4, 0x6E};
dw1000_set_peer_eui(peer_eui);
```

- Your **own** 8‑byte EUI is derived automatically from your own BLE MAC in
  `dw1000_config()` (padded to the IEEE 802.15.4 extended address).
- After pairing, both boards only talk to the paired peer — every frame carries
  the full extended addresses and frames from / for other devices are dropped.

### Ranging (DS‑TWR)

The library implements **asymmetric double‑sided two‑way ranging (DS‑TWR)** —
all four round/reply times are *measured* from the chip's timestamps and
exchanged, so neither the reply delay nor the two boards' crystal offsets are
assumed:

```
TAG (initiator)                      ANCHOR (responder)
    |  POLL (T1)  --------------------->|   T2 = poll RX time
    |  POLL_ACK (T3) <------------------|   reply after reply delay
    |  RANGE (T1, T4, T5) ------------->|   T6 = range RX time
    |                                   |   anchor computes DS‑TWR distance
    |  RANGE_REPORT (float meters) <----|   and sends it back
    |  on_distance() called             |
```

**Tag** — run one exchange at a time (blocking, interrupt‑driven radio):

```c
static bool on_distance(float distance_m, bool got_reading) {
    if (!got_reading) return false;
    ESP_LOGI("APP", "distance: %.2f m", (double)distance_m);
    return true;
}

while (1) {
    dw1000_run_tag(PIN_IRQ, DW1000_RANGE_TIMEOUT_MS, on_distance);
    vTaskDelay(pdMS_TO_TICKS(500));
}
```

**Anchor** — answer forever (never returns):

```c
dw1000_run_anchor(PIN_IRQ);
```

`DW1000_RANGE_TIMEOUT_MS` (default 1000 ms) is how long the tag waits for the
reply before calling `on_distance(..., false)`.

### Antenna‑delay calibration

The antenna delay compensates for the physical delay between the DW1000 chip
and the antenna, and directly shifts the measured distance. A good value on
**both** boards is essential for accurate absolute distances.

**Option A — single‑shot (one board at a time, not recommended):**

```c
uint16_t ad = dw1000_calibrate_antenna_delay(known_distance_cm);
```

**Option B — joint iterative calibration (recommended):** run this on the
**tag** with the modules placed at a *known* distance; the tag repeatedly
measures, takes `(current + ideal)/2`, applies the value on itself, sends it
to the anchor over the air (`CAL_SET`), waits for the anchor to apply and
acknowledge (`CAL_ACK`), and repeats until the correction is below 5 ticks —
so **both** radios end up with the same converged value (it can never
overshoot):

```c
#define CALIBRATE_DISTANCE_CM  100   // true distance between the modules

uint16_t ad = dw1000_calibrate_antenna_delay_iterative(
                  PIN_IRQ,
                  (float)CALIBRATE_DISTANCE_CM,
                  DW1000_CAL_CONVERGENCE_TICKS,   // stop when |calc-current| < 5
                  DW1000_CAL_MAX_ITERATIONS,       // safety cap (20)
                  on_distance);
ESP_LOGI("APP", "shared antenna_delay = %u (set this on BOTH boards)", ad);
```

Steps:
1. Place the two modules exactly `CALIBRATE_DISTANCE_CM` cm apart, clear line of sight.
2. Flash both boards with the **same** `CALIBRATE_DISTANCE_CM`.
3. The tag calibrates both radios and prints the final value.
4. Put that value into `antenna_delay` on **both** boards, then set
   `CALIBRATE_DISTANCE_CM` to `0` and re‑flash.

> The default `antenna_delay = 16464` is a good starting point; calibration
> refines it for your specific hardware.

### Sending / receiving custom frames

Beyond ranging you can send and receive arbitrary payloads (e.g. commands,
sensor data, acknowledgements).

**Send immediately:**

```c
uint8_t frame[] = { /* your payload */ };
dw1000_send(frame, sizeof(frame));
```

**Send after a delay (µs) or at an absolute timestamp:**

```c
dw1000_send_at(frame, len, delay_us);      // relative delay
dw1000_send_at_ticks(frame, len, target);  // absolute 40‑bit timestamp
```

**Receive:**

```c
// handler attached via dw1000_on_received() runs when a frame arrives
void on_received(void) {
    uint8_t buf[64];
    uint16_t n = dw1000_get_data(buf, sizeof(buf));
    // ... process buf[0 .. n-1]
    dw1000_start_receive();   // re‑arm for the next frame
}

dw1000_on_received(on_received);
dw1000_receive_permanently(true);   // stay in RX
dw1000_start_receive();
```

> `dw1000_send()` adds 2 FCS bytes automatically (frame check on by default).
> For a low‑level custom protocol you can keep the 21‑byte IEEE 802.15.4
> extended‑address header helpers inside the library (`DW1000_MSG_*`, message
> offsets) as a starting point.

### Interrupts / callbacks

Events are dispatched from a high‑priority FreeRTOS task (not ISR context), so
SPI access is safe inside handlers:

| Attach | Fires when |
| --- | --- |
| `dw1000_on_sent(cb)` | a TX completed |
| `dw1000_on_received(cb)` | a good frame was received |
| `dw1000_on_receive_failed(cb)` | a frame failed (FCS / header error) |
| `dw1000_on_receive_timeout(cb)` | RX timed out |
| `dw1000_on_error(cb)` | PLL / clock lock lost |
| `dw1000_on_receive_timestamp_available(cb)` | LDE timestamp ready |

Enable the corresponding interrupt sources first (apply with
`dw1000_commit_config()`):

```c
dw1000_interrupt_on_sent(true);
dw1000_interrupt_on_received(true);
dw1000_commit_config();
dw1000_irq_start(PIN_IRQ);          // start the IRQ GPIO + dispatch task
```

### Diagnostics

```c
float rx_pwr = dw1000_get_rx_power_dbm();        // total RX power (dBm)
float fp     = dw1000_get_first_path_power_dbm();// first‑path power (dBm)
float q      = dw1000_get_rx_quality();          // RX quality (dB)
float temp, vbat;
dw1000_get_temp_and_vbat(&temp, &vbat);          // on‑chip temperature & voltage
```

---

## ⚙️ Important constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `DW1000_LEN_DATA` | 16 | payload length of a ranging frame (21‑byte header + payload) |
| `DW1000_REPLY_DELAY_US` | 3000 | nominal anchor reply delay (measured anyway) |
| `DW1000_RANGE_TIMEOUT_MS` | 1000 | tag max wait for a result |
| `DW1000_CAL_CONVERGENCE_TICKS` | 5 | calibration stop threshold (≈ 2.3 cm) |
| `DW1000_CAL_MAX_ITERATIONS` | 20 | calibration loop safety cap |
| `DW1000_CAL_ACK_TIMEOUT_MS` | 500 | tag max wait for the anchor's `CAL_ACK` |
| `DW1000_METERS_PER_TICK` | ≈0.00469 | meters per raw 15.65 ps timestamp tick |

Raw timestamps are 40‑bit values that wrap every ~17 s; use the provided
`dw1000_ticks_to_meters()` / `diff_ts()` helpers instead of raw arithmetic.

---

## 📝 Examples

| Example | Role | What it does |
| --- | --- | --- |
| `Examples/Device-A` | TAG (initiator) | Probes, configures, pairs, optionally calibrates, then ranges continuously and prints the distance. |
| `Examples/Device-B` | ANCHOR (responder) | Answers a paired tag, computes the DS‑TWR distance, sends it back, and accepts antenna‑delay calibration updates from the tag. |

To run them you need **two** ESP32 + DW1000 boards. Change the `peer_eui`
array on each board to the *other* board's BLE MAC (printed at boot).

---

## 📄 License

[MIT](LICENSE) — use it freely in commercial and personal projects.

---

## 🙏 Acknowledgements

- [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000) — the original
  Arduino library whose algorithms (DS‑TWR ranging, tuning tables) this project
  re‑implements in pure C.
- Decawave / Qorvo DW1000 User Manual.

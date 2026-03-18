# rpi-seism-reader

Firmware for an Arduino-based 3-channel geophone digitizer that communicates with a Raspberry Pi over RS-422.  
Part of the [rpi-seism](https://github.com/rpi-seism) ecosystem — reads analog signals from a GD-4.5 geophone via an ADS1256 ADC, formats samples into binary packets, and streams them to the Pi over a heartbeat-controlled full-duplex RS-422 link.

---

## Features

- 3-channel differential sampling — reads EHZ, EHN, EHE via ADS1256
- Runtime-configurable ADC settings — sampling rate, PGA, and data rate sent by the Pi at startup; no recompilation needed
- Settings handshake — blocks until a valid `SettingsPacket` is received, echoes it back for end-to-end verification
- Heartbeat-based streaming — starts sending data only after receiving a heartbeat byte; stops after 1 second without one
- Jitter-free timing — uses `lastSampleTime += interval` to prevent drift accumulation
- XOR-checksummed 16-byte packet format
- No direction-control pin — RS-422 is full-duplex; TX and RX are separate pairs, the driver is always enabled
- Automatic platform detection — SPI pins and ADS1256 constructor selected at compile time via `#if defined` blocks
- Multi-platform — AVR, RP2040, STM32, ESP32, Teensy

---

## Supported platforms

| Architecture | Boards | SPI macro |
|---|---|---|
| AVR | Uno, Nano (328P), Mega 2560 | Default SPI pins |
| RP2040 | Raspberry Pi Pico, Waveshare Mini, Zero | `USE_SPI1` for SPI1 |
| STM32 | Blue Pill (F103), Black Pill (F411) | `USE_SPI2` for SPI2 |
| ESP32 | ESP32 WROOM, S3, C3 | `USE_HSPI` for HSPI |
| Teensy | Teensy 4.0, 4.1 | `USE_SPI1` / `USE_SPI2` |

---

## Wiring

### ADS1256

The ADS1256 constructor and SPI pins are selected automatically at compile time based on the target platform. The active defaults per platform are:

| Platform | DRDY | RESET | SYNC | CS | SPI bus |
|---|---|---|---|---|---|
| AVR (Uno/Nano) | 2 | — | 8 | 10 | SPI |
| RP2040 SPI0 | 7 | — | 6 | 5 | SPI |
| RP2040 SPI1 | 18 | 20 | 21 | 19 | SPI1 |
| STM32 SPI1 | PA2 | — | — | PA4 | SPI |
| STM32 SPI2 | PB10 | PB11 | — | PB12 | SPI2 |
| ESP32 | 16 | 17 | — | 15 | SPI / HSPI |
| Teensy | 7 | — | 8 | 10 | SPI / SPI1 / SPI2 |

To use an alternate SPI bus, pass the appropriate macro via `build_flags` in `platformio.ini` (see [Installation](#installation)).

The ADS1256 requires a stable 2.5 V reference. Most modules include a REF5025; if yours does not, provide an external 2.5 V reference.

### RS-422 transceiver

RS-422 is full-duplex — TX and RX use separate twisted pairs. No direction-control GPIO is needed; the firmware writes directly to `Serial` with the driver permanently enabled.

| Transceiver pin | Connect to |
|---|---|
| DI | MCU TX |
| RO | MCU RX |
| TX+ / TX− | Twisted pair → Pi RX+ / RX− |
| RX+ / RX− | Twisted pair ← Pi TX+ / TX− |

---

## Installation

### PlatformIO (recommended)

```bash
git clone https://github.com/rpi-seism/reader
cd reader
```

Upload to your board:

```bash
pio run -e nanoatmega328new --target upload   # Arduino Nano
pio run -e rpipico          --target upload   # Raspberry Pi Pico
pio run -e esp32dev         --target upload   # ESP32
pio run -e blackpill_f411ce --target upload   # STM32 Black Pill
pio run -e teensy40         --target upload   # Teensy 4.0
```

To use an alternate SPI bus, add `build_flags` to the environment:

```ini
[env:rpipico]
...
build_flags = -D USE_SPI1   # RP2040: use SPI1 instead of SPI0

[env:esp32dev]
...
build_flags = -D USE_HSPI   # ESP32: use HSPI instead of VSPI

[env:blackpill_f411ce]
...
build_flags = -D USE_SPI2   # STM32: use SPI2 instead of SPI1
```

Monitor serial output after upload:

```bash
pio device monitor -e nanoatmega328new
```

### Platform notes

**RP2040** — the official `raspberrypi` platform does not support `SPI.setSCK()` / `setTX()` / `setRX()`. The Earle Philhower core via the maxgerhardt platform is required:

```ini
[env:rpipico]
platform          = https://github.com/maxgerhardt/platform-raspberrypi.git
board             = rpipico
framework         = arduino
board_build.core  = earlephilhower
```

**ESP32** — the generic `esp32dev` board does not define `LED_BUILTIN`. The firmware falls back to GPIO 2, which is correct for most ESP32 WROOM modules. Override via `build_flags = -D LED_BUILTIN=<pin>` if your board differs.

### Arduino IDE

1. Install the [ADS1256 library](https://github.com/CuriousScientist0/ADS1256) via **Sketch → Include Library → Add .ZIP Library**
2. Open `src/main.cpp`
3. Select your board and port, then upload — the correct constructor and SPI pins are selected automatically

---

## Protocol

### Settings handshake

On boot the firmware calls `getSettings()` which blocks until a valid `SettingsPacket` is received from the Pi.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | `header1` | uint8_t | `0xCC` |
| 1 | `header2` | uint8_t | `0xDD` |
| 2–3 | `samplingSpeed` | uint16_t | Desired output rate in Hz |
| 4 | `ADCGain` | uint8_t | PGA index 0–6 |
| 5 | `ADCDataRate` | uint8_t | Data rate index 0–15 |

Out-of-range values are clamped (`ADCGain` to 6, `ADCDataRate` to 11). The packet is echoed verbatim — the Pi verifies the echo before starting its heartbeat loop.

#### PGA index

| Index | PGA | Index | PGA |
|-------|-----|-------|-----|
| 0 | ×1 | 4 | ×16 |
| 1 | ×2 | 5 | ×32 |
| 2 | ×4 | 6 | ×64 |
| 3 | ×8 | | |

#### Data rate index

| Index | Rate | Index | Rate |
|-------|------|-------|------|
| 0 | 2 SPS | 8 | 100 SPS |
| 1 | 5 SPS | 9 | 500 SPS |
| 2 | 10 SPS | 10 | 1000 SPS |
| 3 | 15 SPS | 11 | **2000 SPS** ← recommended for 100 Hz output |
| 4 | 25 SPS | 12 | 3750 SPS |
| 5 | 30 SPS | 13 | 7500 SPS |
| 6 | 50 SPS | 14 | 15000 SPS |
| 7 | 60 SPS | 15 | 30000 SPS |

### Data packet

Fixed 16-byte `ADC_Packet` frame sent at the configured output rate:

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | `header1` | uint8_t | `0xAA` |
| 1 | `header2` | uint8_t | `0xBB` |
| 2–5 | `ch0` | int32_t | EHZ differential reading |
| 6–9 | `ch1` | int32_t | EHN differential reading |
| 10–13 | `ch2` | int32_t | EHE differential reading |
| 14 | `checksum` | uint8_t | XOR of bytes 0–13 |

### Streaming state machine

- **STOP** — LED blinks at 1 Hz, ADC halted, waiting for heartbeat
- **STREAMING** — ADC sampled at configured rate, packets sent over RS-422

Any byte received from the Pi resets the 1000 ms heartbeat timeout. Missing the timeout returns to STOP.

### MUX cycling

Each cycle calls `cycleDifferential()` four times — the fourth call is discarded but required to advance the ADS1256 multiplexer back to the first channel pair. Removing it causes channels to rotate incorrectly.

---

## Compile-time parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAMPLING_SPEED` | `100` | Fallback rate before handshake (Hz) |
| `TIMEOUT_DURATION` | `1000` | Heartbeat timeout (ms) |
| `ADS_DRDY` | platform-dependent | ADS1256 DRDY pin override |
| `ADS_CS` | platform-dependent | ADS1256 CS pin override |
| `LED_BUILTIN` | platform-dependent | Onboard LED pin (ESP32 defaults to 2) |

---

## Troubleshooting

**Stuck in `getSettings()`** — the Pi must send the `SettingsPacket` first. Verify RS-422 wiring (TX+/TX− and RX+/RX− must not be swapped) and that the daemon's `Reader` thread is running.

**No data after handshake** — verify the Pi is sending heartbeats every 500 ms. Both sides must be at 250 000 baud.

**LED not blinking** — device is stuck waiting for the settings handshake (solid off). If blinking at 1 Hz, handshake succeeded but heartbeats are not arriving.

**Noisy ADC readings** — verify the 2.5 V reference is stable. Check shielding and grounding on geophone cables.

**Channels rotating** — the fourth `cycleDifferential()` (discard) call has been removed. Restore it.

**RP2040 compilation errors** (`setSCK`/`setTX`/`setRX` not found) — the official `raspberrypi` platform is being used. Switch to the maxgerhardt platform with `board_build.core = earlephilhower` (see [Platform notes](#platform-notes)).

**ESP32 `LED_BUILTIN` not declared** — add `build_flags = -D LED_BUILTIN=2` to the `esp32dev` environment in `platformio.ini`.

---

## Related repositories

| Repository | Description |
|---|---|
| [rpi-seism/daemon](https://github.com/rpi-seism/daemon) | Python acquisition daemon |
| [rpi-seism/api](https://github.com/rpi-seism/api) | FastAPI archive browser |
| [rpi-seism/web](https://github.com/rpi-seism/web) | Angular frontend |
| [rpi-seism/stack](https://github.com/rpi-seism/stack) | Docker Compose deployment |

---

## License

[GNU General Public License v3.0](LICENSE)
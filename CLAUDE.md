# CaptainStatic — Project Context

## Project Overview

An interactive art/installation project using UWB (Ultra-Wideband) ranging to create a proximity-based audio experience. 8 anchor/listener pairs. As a user brings a handheld listening device close to its paired anchor object, an audio track is "revealed" — the primary audio gets louder and a static/noise track fades out, as if tuning into a signal.

## Hardware

- **Board**: MakerFabs ESP32 DW3000 (ESP32 + Decawave DW3000 UWB chip)
- **Audio amp**: IIS (I2S) audio amp breakout board (inside the listener device)
- **Speaker**: Connected to the I2S amp inside the listener enclosure
- **Audio storage**: Saved directly to ESP32 flash (SPIFFS or LittleFS, or embedded byte arrays)

### Pin Assignments (DW3000)
```
PIN_RST = 27
PIN_IRQ = 34
PIN_SS  = 4  (SPI chip select)
SPI speed: 16 MHz (set via _fastSPI SPISettings)
```

## System Architecture

### Roles
| Sketch | Physical Role | TWR Role | Count |
|---|---|---|---|
| `anchor/anchor.ino` | Stationary object in the installation | **SS TWR Responder** — waits for poll, sends back timestamps | 8 |
| `listener/listener.ino` | Handheld listening device (with speaker) | **SS TWR Initiator** — sends poll, receives response, computes distance | 8 |

**8 unique pairs** — each listener only communicates with its designated anchor. There is no cross-talk between pairs.

### Ranging Protocol (SS TWR — Single-Sided Two-Way Ranging)
1. **Listener** broadcasts a poll frame every `RNG_DELAY_MS` (100 ms)
2. **Anchor** receives poll, records `poll_rx_ts`, schedules delayed response transmission
3. **Anchor** sends response with `poll_rx_ts` and `resp_tx_ts` embedded
4. **Listener** receives response, extracts timestamps, computes ToF and distance:
   ```c
   rtd_init = resp_rx_ts - poll_tx_ts;
   rtd_resp = resp_tx_ts - poll_rx_ts;
   tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
   distance = tof * SPEED_OF_LIGHT;  // in meters
   ```

### Message Frames
```c
// Poll (listener → anchor):
{0x41, 0x88, SN, 0xCA, PAIR_ID, 'C','A','P','T', 0xE0, 0, 0}

// Response (anchor → listener):
{0x41, 0x88, SN, 0xCA, PAIR_ID, 'T','P','A','C', 0xE1, 0,0, [poll_rx_ts 4B], [resp_tx_ts 4B], 0,0}
```
`ALL_MSG_COMMON_LEN = 10` — first 10 bytes compared for message validation (SN byte at index 2 is zeroed before compare).
Byte 4 is `PAIR_ID` (802.15.4 PAN ID low byte) — unique per pair, so cross-pair frames are automatically rejected by the header comparison.
Last 2 bytes of each frame are the 802.15.4 FCS placeholder; the DW3000 fills these with the computed CRC on TX and includes them in RXFLEN on RX — do not use for application data.

### DW3000 RF Config
```c
Channel: 5
Preamble: DWT_PLEN_128
PAC: DWT_PAC8
TX/RX preamble code: 9
SFD: non-standard 8-symbol
Data rate: DWT_BR_6M8 (6.8 Mbps)
STS: OFF
PDOA: OFF
Antenna delay (TX & RX): 16385
```

## Project Structure

```
CaptainStatic/
├── CLAUDE.md
├── anchor/
│   ├── anchor.ino          # SS TWR Responder — flash to anchor boards
│   └── src/dw3000/         # DW3000 driver library (shared)
│       ├── dw3000.h
│       ├── dw3000_device_api.{h,cpp}
│       ├── dw3000_port.{h,cpp}
│       ├── dw3000_shared_functions.{h,cpp}
│       ├── dw3000_config_options.{h,cpp}
│       ├── dw3000_mutex.cpp
│       ├── dw3000_uart.{h,cpp}
│       ├── dw3000_mac_802_15_4.{h,cpp}
│       ├── dw3000_regs.h
│       ├── dw3000_shared_defines.h
│       ├── dw3000_types.h
│       └── dw3000_vals.h
└── listener/
    ├── listener.ino        # SS TWR Initiator — flash to listener boards
    └── src/dw3000/         # Same DW3000 driver library (duplicate)
```

> Note: `src/dw3000/` is duplicated in both sketches because Arduino IDE expects library files adjacent to the sketch. Consider consolidating if moving to PlatformIO.

## Current State (Working Baseline)

- Both example sketches compile and run successfully
- Listener computes and prints distance over serial: `DIST: X.XX m`
- No pair differentiation yet — all boards share identical poll/response frames
- No audio output implemented yet

## Development Roadmap

### Phase 1 — Pair Identification
Each of the 8 pairs needs a unique ID so listeners only respond to their designated anchor. Options:
- Embed a 1-byte pair ID in the message frame (e.g., replace a reserved byte)
- Use the frame's PAN ID or short address fields (bytes 3–4: `0xCA, 0xDE`)
- Filter on both the message type byte (`0xE0`/`0xE1`) and a pair ID byte

### Phase 2 — Audio Playback on Listener
- **I2S amp**: likely MAX98357A or similar (confirm part)
- **I2S pins**: needs to be assigned on ESP32 (standard: BCLK, LRCLK/WS, DATA)
- **Audio files**: stored in flash via SPIFFS or LittleFS (`.wav` or raw PCM), or embedded as C byte arrays
- **Library candidates**: `ESP8266Audio` (works on ESP32), Arduino `I2S` library
- Two tracks required:
  - Track A: primary audio content (looping)
  - Track B: static/noise (looping)

### Phase 3 — Proximity-Based Volume Control
- Distance computed on listener → map to volume levels for Track A and Track B
- Suggested curve: define `NEAR_DIST` and `FAR_DIST` thresholds, linear or logarithmic interpolation
- Track A volume: `0` at far, `MAX` at near
- Track B volume: `MAX` at far, `0` at near (or some floor)
- Smooth transitions to avoid abrupt jumps (low-pass filter / moving average on distance)

### Phase 4 — Hardware Integration & Enclosure
- Mount listener PCB + amp + speaker in enclosure
- Power considerations (battery vs wired)
- Anchor boards need power at installation locations

## Key Library Functions Reference

```c
// Shared functions (dw3000_shared_functions.h)
uint64_t get_rx_timestamp_u64(void);     // 64-bit RX timestamp
uint64_t get_tx_timestamp_u64(void);     // 64-bit TX timestamp
void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts);
void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts);

// Port / SPI init (dw3000_port.h)
void spiBegin(uint8_t irq, uint8_t rst);
void spiSelect(uint8_t ss);
void Sleep(uint32_t d);  // millisecond delay used between ranging cycles

// DW IC init sequence (both sketches)
dwt_checkidlerc()         // must return true before init
dwt_initialise(DWT_DW_INIT)
dwt_configure(&config)
dwt_configuretxrf(&txconfig_options)  // txconfig_options defined in dw3000_config_options.cpp
dwt_setrxantennadelay(RX_ANT_DLY)
dwt_settxantennadelay(TX_ANT_DLY)
```

## Git & Workflow Notes

- Do not include `Co-Authored-By` trailer lines in commit messages for this project.

## Notes & Gotchas

- `POLL_RX_TO_RESP_TX_DLY_UUS = 450` on anchor and `POLL_TX_TO_RESP_RX_DLY_UUS = 240` on listener — these must be tuned together; if the response arrives before the listener opens its RX window the ranging will fail silently
- `RESP_RX_TIMEOUT_UUS = 400` — listener will clear RX errors and retry if no response within this window
- `dwt_starttx(DWT_START_TX_DELAYED)` on anchor can return error if timing is too tight; the sketch detects this and skips the exchange
- The `dist_str` buffer and `test_run_info()` call in `listener.ino` (line 159–160) reference globals/functions not defined in the sketch file — these are likely defined elsewhere or are placeholders to be replaced
- `UART_init()` / `UART_puts()` are wrappers around `Serial.begin()` / `Serial.print()` defined in `dw3000_uart.cpp`
- Mutex implementation uses FreeRTOS `portENTER_CRITICAL` / `portEXIT_CRITICAL` — this is ESP32-specific and correct for the platform

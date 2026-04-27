# CaptainStatic

An interactive art installation using UWB (Ultra-Wideband) radio to create a proximity-based audio experience. Eight handheld "listener" devices each pair with a stationary "anchor" object in the installation space. As a visitor brings a listener close to its anchor, a hidden audio track fades in while a radio static layer fades out — like tuning into a signal.

---

## How It Works

Each pair consists of two ESP32 boards running different firmware:

| Board | Sketch | Physical role |
|---|---|---|
| **Anchor** | `anchor/anchor.ino` | Stationary object in the space. Waits for radio pings and echoes precise timestamps back. |
| **Listener** | `listener/listener.ino` | Handheld device with speaker. Pings its anchor 10 times per second, computes distance from the timing, and crossfades between two audio tracks accordingly. |

The ranging technique is called **SS-TWR (Single-Sided Two-Way Ranging)**. The listener sends a "poll" radio frame; the anchor responds with embedded timestamps; the listener uses all four timestamps (its own send/receive times plus the anchor's receive/send times) to compute pure signal travel time, which converts directly to distance in meters.

Distance is passed through an IIR low-pass filter to smooth out frame-to-frame noise before it drives the audio crossfade.

---

## Hardware

### Bill of Materials (per pair)

| Qty | Component | Notes |
|---|---|---|
| 2 | MakerFabs ESP32 DW3000 | One flashed as anchor, one as listener |
| 1 | MAX98357A I2S amplifier breakout | Listener only |
| 1 | Speaker (4Ω or 8Ω) | Listener only |
| — | USB-C cables | For power and flashing |

### Listener Wiring

Connect the MAX98357A breakout to the listener ESP32:

| MAX98357A pin | ESP32 GPIO | Wire color (suggestion) |
|---|---|---|
| BCLK | 26 | Yellow |
| LRC (LRCLK / WS) | 25 | Green |
| DIN | 22 | Blue |
| VIN | 3.3V or 5V | Red |
| GND | GND | Black |

Leave all other MAX98357A pins unconnected unless you want to change the hardware gain (see [Adjusting Volume](#adjusting-volume) below).

The DW3000 UWB radio is already onboard the MakerFabs module and wired internally — no extra connections needed for ranging.

### Anchor Wiring

The anchor board uses only the onboard DW3000. No additional wiring needed beyond USB power.

---

## First-Time Software Setup

### 1. Install Arduino IDE

Download and install [Arduino IDE 2.x](https://www.arduino.cc/en/software).

### 2. Add ESP32 board support

1. Open **File → Preferences**
2. Paste this URL into *Additional boards manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Open **Tools → Board → Boards Manager**, search `esp32`, install the **esp32 by Espressif Systems** package.

### 3. Install required libraries

Open **Tools → Manage Libraries** and install:

| Library | Author |
|---|---|
| **ESP8266Audio** | Earle F Philhower III |
| **LittleFS** | Included with ESP32 board package — no separate install needed |

> **Note:** Despite the name, ESP8266Audio works on ESP32 and includes the AAC decoder, I2S output driver, and mixer used by the listener.

### 4. Install the LittleFS upload tool

The audio files are stored in ESP32 flash using LittleFS. To upload them you need the filesystem image builder:

1. Download the latest release of [arduino-littlefs-upload](https://github.com/earlephilhower/arduino-littlefs-upload/releases) (the `.vsix` file)
2. In Arduino IDE 2.x: **Sketch → Include Library → Add .ZIP Library** does *not* work for plugins. Instead, install it as an IDE plugin by dragging the `.vsix` into Arduino IDE or using **Help → Install Plugin from File**.
3. After installation, a **Tools → Upload LittleFS to Pico/ESP8266/ESP32** menu item should appear.

### 5. Set board settings

Select these settings under **Tools** before compiling or uploading either sketch:

| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Frequency | 80MHz |
| Flash Mode | QIO |
| Flash Size | 4MB (32Mb) |
| **Partition Scheme** | **Default 4MB with spiffs** |
| Core Debug Level | None |
| PSRAM | Disabled |
| Port | (whichever COM port appears when you plug in the board) |

The partition scheme is critical — if it is set incorrectly the LittleFS filesystem will either not fit or not be found at runtime.

---

## Flashing the Firmware

### Flash an anchor board

1. Open `anchor/anchor.ino` in Arduino IDE
2. Plug in the anchor ESP32 via USB
3. Select the correct COM port under **Tools → Port**
4. Click **Upload** (the → button)
5. Open **Tools → Serial Monitor** at 115200 baud — you should see:
   ```
   Range TX
   Setup over........
   ```

### Flash a listener board

1. Open `listener/listener.ino` in Arduino IDE
2. Plug in the listener ESP32 via USB
3. Select the correct COM port
4. Click **Upload**
5. Open Serial Monitor at 115200 baud — you should see:
   ```
   Listener ready.
   DIST: ACQUIRING...
   ```
   Once the listener is ranging with its anchor, it will show:
   ```
   DIST: 1.23 m
   ```

---

## Uploading Audio Files

The listener plays two looping tracks stored directly on the ESP32's flash:

| File | Role |
|---|---|
| `/track.aac` | Primary audio content. Full volume when close to the anchor, silent when far. |
| `/static.aac` | Radio static / noise. Full volume when far, silent when close. |

### Audio file requirements

- **Format:** AAC-LC, ADTS container — the file extension must be `.aac`
- **Not supported:** `.m4a` (MPEG-4 container) — the decoder requires raw ADTS
- **Channels:** Mono (stereo works but wastes flash space and decode time)
- **Sample rate:** 44100 Hz recommended
- **Bit rate:** 64–128 kbps works well; lower saves flash space

To convert an existing audio file to the correct format using [ffmpeg](https://ffmpeg.org/):

```bash
# Convert any audio file to mono AAC-LC ADTS at 64kbps
ffmpeg -i input.mp3 -ac 1 -c:a aac -b:a 64k output.aac
```

### Uploading the files

1. Create a folder called `data` inside `listener/` if it doesn't exist:
   ```
   listener/
   ├── listener.ino
   ├── data/
   │   ├── track.aac
   │   └── static.aac
   └── src/
       └── dw3000/
   ```
2. Place your `track.aac` and `static.aac` files inside `listener/data/`
3. Open `listener/listener.ino` in Arduino IDE
4. Go to **Tools → Upload LittleFS to Pico/ESP8266/ESP32**
5. Wait for the upload to complete (it will say "LittleFS upload complete" in the output)
6. Then do a normal **Upload** to flash the firmware (the LittleFS upload and firmware upload are separate steps — both are needed after a fresh flash or whenever you change the audio files)

> **Important:** The LittleFS upload and the firmware upload overwrite different regions of flash. You can update audio files without re-flashing the firmware, and vice versa — but you need to do both at least once before first use.

---

## Adjusting Volume

There are two independent volume controls — software gain and hardware gain. Start with the hardware gain and fine-tune with software.

### Software gain (in `listener.ino`)

Open `listener/listener.ino` and find this line near the top:

```cpp
const float MASTER_GAIN = 0.1f;  // Master output gain: 1.0 = unity, reduce if audio is distorting
```

- `1.0` = unity gain (full volume from the decoder)
- `0.1` = 10% of unity (current setting, chosen to prevent distortion)
- Valid range: `0.0` to `4.0` — values above `1.0` amplify and will distort if the source is already loud

After changing this value, re-flash the firmware with **Upload**.

### Hardware gain (MAX98357A GAIN pin)

The MAX98357A's GAIN pin sets a fixed amplifier gain in dB. The options are:

| GAIN pin connection | Gain |
|---|---|
| Left floating (unconnected) | 9 dB ← current setting |
| 100kΩ resistor to GND | 6 dB (quieter) |
| Direct to GND | 15 dB (louder) |
| Direct to Vin | 12 dB (louder) |

Changing hardware gain requires resoldering the breakout. Start with software gain adjustments unless the hardware setting is clearly wrong.

### Crossfade distance thresholds

The points at which the tracks fully crossfade are also configurable in `listener.ino`:

```cpp
const float NEAR_DIST_M = 0.5f;  // Closer than this → track.aac at full volume
const float FAR_DIST_M  = 3.0f;  // Farther than this → static.aac at full volume
```

Adjust these to match the physical scale of your installation.

---

## Project Structure

```
CaptainStatic/
├── README.md
├── CLAUDE.md                   # AI assistant context file — detailed architecture notes
├── anchor/
│   ├── anchor.ino              # Flash to anchor boards (stationary objects)
│   └── src/dw3000/             # DW3000 UWB radio driver
└── listener/
    ├── listener.ino            # Flash to listener boards (handheld devices with speaker)
    ├── data/                   # Put track.aac and static.aac here before LittleFS upload
    └── src/dw3000/             # DW3000 UWB radio driver (duplicated — Arduino IDE requirement)
```

---

## Troubleshooting

**Serial monitor shows `IDLE FAILED` or `INIT FAILED`**
The DW3000 chip is not responding over SPI. Check that the correct board is selected and the USB connection is solid. Power-cycle the board and try again.

**Serial monitor shows `DIST: ACQUIRING...` indefinitely**
The listener is not receiving responses from the anchor. Verify the anchor is powered and its serial monitor shows `Setup over........`. Make sure both boards are running firmware compiled with the same RF config (channel 5, DWT_PLEN_512). Bring the boards within 1–2 metres of each other for initial testing.

**Audio: begin failed — check /track.aac and /static.aac in LittleFS**
The audio files are missing from flash. Make sure you ran the LittleFS upload step (separate from the firmware upload) with both files in `listener/data/`.

**Board crashes and reboots with a watchdog error**
The audio task must call `vTaskDelay(1)` each iteration to feed the watchdog. This is already present in the current code. If you see this error, the most likely cause is that the audio task is hanging in `begin()` or `loop()` — check that the `.aac` files are valid and the correct format.

**Distance reads correctly but audio is silent**
Check the MAX98357A wiring — BCLK (GPIO 26), LRC (GPIO 25), DIN (GPIO 22). Check that `MASTER_GAIN` in `listener.ino` is greater than `0`. Verify the speaker is connected to the amp's output terminals.

**Audio is distorting**
Lower `MASTER_GAIN` in `listener.ino` (e.g., try `0.05f`) and re-flash.

---

## Next Steps

The core ranging and audio system is working. The following remains before the installation is ready for multi-pair deployment:

### 1. Pair identification (required for deployment)

**Problem:** All 8 listener/anchor pairs currently use identical radio frames. If two pairs are powered on in the same space, any listener will respond to any anchor — distance readings will be garbage.

**What to do:** Add a 1-byte pair ID (0–7) to the message frames. The simplest approach is to replace one of the reserved zero bytes in the frame (e.g., index 10 or 11 in the poll, matched in the response). Each anchor/listener pair gets a unique ID baked in as a constant. The listener's `memcmp` header check will naturally reject frames from other pairs once the ID byte is included in `ALL_MSG_COMMON_LEN`.

### 2. Per-pair audio content

Once pair IDs exist, each listener needs its own `track.aac` — the primary audio content that is "revealed" when close to its specific anchor. The `static.aac` track can be shared across all pairs or varied per pair. The LittleFS upload process is per-board, so each listener gets its own audio files flashed to its own flash.

### 3. Enclosure and power

Each listener needs a self-contained enclosure with:
- The ESP32 DW3000 board
- The MAX98357A amp breakout
- A small speaker
- A battery (a USB power bank works well for prototyping)

The anchors only need power — a USB wall adapter or battery pack at each installation location.

### 4. Antenna delay calibration (optional but improves accuracy)

The current antenna delay value (16385) is the factory default. Calibrating it against a known distance will improve absolute accuracy by a few centimetres. The crossfade thresholds (`NEAR_DIST_M`, `FAR_DIST_M`) are wide enough that this probably isn't noticeable in practice, but it is worth doing if the "close" threshold feels off during installation.

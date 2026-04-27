// =============================================================================
// listener.ino — SS TWR Initiator + Proximity Audio
// =============================================================================
// Role in the installation:
//   This sketch runs on the HANDHELD listening device (listener).
//   It continuously ranges to its paired stationary anchor board using
//   UWB (Ultra-Wideband) radio via the DW3000 chip, then uses that distance
//   to crossfade between two looping audio tracks played through an I2S amp.
//
// How the ranging works (SS TWR — Single-Sided Two-Way Ranging):
//   1. Listener sends a "poll" radio frame to the anchor.
//   2. Anchor receives it, records timestamps, and sends back a "response" frame.
//   3. Listener reads the timestamps from the response and computes time-of-flight,
//      which converts directly to a distance in meters.
//   This cycle repeats every RNG_INTERVAL_MS (100ms).
//
// How the audio works:
//   Two AAC audio files are stored in flash (LittleFS): /track.aac and /static.aac.
//   They are decoded simultaneously and mixed into a single I2S output stream.
//   - /track.aac  = primary audio content. Loud when close, silent when far.
//   - /static.aac = radio static/noise. Loud when far, silent when close.
//   The crossfade is linear between NEAR_DIST_M and FAR_DIST_M.
//
// Dual-core layout (ESP32 has two cores):
//   Core 1 — Arduino loop() — runs the ranging cycle (time-sensitive)
//   Core 0 — audioTask()   — runs the audio decode/mix loop (continuous)
//   The two cores share `filteredDistance` (declared volatile for safe cross-core reads).
//
// Hardware:
//   Board  : MakerFabs ESP32 DW3000
//   UWB    : Decawave DW3000 chip (SPI)
//   Amp    : MAX98357A I2S amplifier breakout
//   Storage: ESP32 flash via LittleFS (4MB partition with spiffs layout)
// =============================================================================

#include "src/dw3000/dw3000.h"       // DW3000 UWB driver
#include <LittleFS.h>                 // Flash filesystem for audio files
#include <AudioFileSourceLittleFS.h>  // ESP8266Audio: file source backed by LittleFS
#include <AudioGeneratorAAC.h>        // ESP8266Audio: AAC-LC (Helix) decoder
#include <AudioOutputI2S.h>           // ESP8266Audio: I2S audio output driver
#include <AudioOutputMixer.h>         // ESP8266Audio: mixes multiple decoded streams into one output

// -----------------------------------------------------------------------------
// DW3000 SPI pin assignments
// -----------------------------------------------------------------------------
const int PIN_RST = 27;  // DW3000 reset line
const int PIN_IRQ = 34;  // DW3000 interrupt line (not used in polling mode but required by driver)
const int PIN_SS  = 4;   // SPI chip-select for DW3000

// -----------------------------------------------------------------------------
// Ranging timing constants
// -----------------------------------------------------------------------------
// How often the listener sends a poll and waits for a response.
// The loop() function sleeps for whatever time is left after the exchange finishes.
const int     RNG_INTERVAL_MS           = 100;

// Antenna delays compensate for the time the signal spends inside the chip's
// analog front-end before/after it hits the antenna. 16385 is the factory default
// for DW3000 on channel 5. Calibrating this improves absolute distance accuracy.
const uint16_t TX_ANT_DLY              = 16385;
const uint16_t RX_ANT_DLY              = 16385;

// Message frame layout constants.
// Both poll and response frames share a common 10-byte header for validation.
const int     ALL_MSG_COMMON_LEN        = 10;   // Number of header bytes to compare when validating a received frame
const int     ALL_MSG_SN_IDX            = 2;    // Byte index of the sequence number field in both frames

// Byte offsets within the response frame where the anchor embeds its timestamps.
// These are read by the listener to compute round-trip time.
const int     RESP_MSG_POLL_RX_TS_IDX   = 10;   // Offset of the "when anchor received the poll" timestamp
const int     RESP_MSG_RESP_TX_TS_IDX   = 14;   // Offset of the "when anchor sent this response" timestamp
// RESP_MSG_TS_LEN = 4 is defined in dw3000_shared_defines.h — not redeclared here to avoid collision.
// Each timestamp field is 4 bytes (lower 32 bits of the 40-bit DW3000 timestamp).

// How long after sending the poll the listener opens its receive window.
// Must be long enough for the anchor to process the poll and send a response,
// but the anchor's POLL_RX_TO_RESP_TX_DLY_UUS (900µs) governs the actual anchor delay.
// This 240µs value tells the DW3000 hardware to start listening 240µs after TX.
const int     POLL_TX_TO_RESP_RX_DLY_UUS = 240;

// If no valid response is received within this window after opening RX,
// the DW3000 raises a timeout flag and the exchange is abandoned for this cycle.
// 1200µs gives plenty of margin above the anchor's 900µs response delay.
const int     RESP_RX_TIMEOUT_UUS       = 1200;

// -----------------------------------------------------------------------------
// Distance filtering
// -----------------------------------------------------------------------------
// Each new raw distance reading is blended with the previous filtered value
// using an IIR (Infinite Impulse Response) low-pass filter:
//   filtered = alpha * new + (1 - alpha) * filtered
// Lower alpha = smoother but slower to respond. 0.2 means 20% new, 80% history.
const float DISTANCE_FILTER_ALPHA   = 0.2f;

// If this many consecutive ranging cycles fail (timeout, bad frame, out-of-range),
// we report "NO SIGNAL" on the serial monitor. At 100ms per cycle this is ~1 second.
const int   MAX_CONSECUTIVE_FAILURES = 10;

// -----------------------------------------------------------------------------
// I2S audio amp pin assignments (MAX98357A breakout)
// -----------------------------------------------------------------------------
const int I2S_BCLK_PIN  = 26;  // Bit clock
const int I2S_LRCLK_PIN = 25;  // Left/right word-select clock
const int I2S_DIN_PIN   = 22;  // Serial data to amp

// Per-track ring buffer size for the AudioOutputMixer.
// Must be at least 1024 samples to hold one full AAC-LC decoded frame.
const int   MIXER_BUF_SIZE = 1024;

// Overall output volume scalar applied at the mixer stub level.
// 1.0 = unity (full volume). 0.1 = 10% — reduce this if audio is clipping/distorting.
// Note: the MAX98357A hardware gain is set by its GAIN pin (floating = 9dB, lowest setting).
// This software gain stacks on top of that.
const float MASTER_GAIN    = 0.2f;

// -----------------------------------------------------------------------------
// Crossfade distance thresholds
// -----------------------------------------------------------------------------
// At NEAR_DIST_M or closer  → track.aac is at full volume, static.aac is silent.
// At FAR_DIST_M  or farther → static.aac is at full volume, track.aac is silent.
// Between the two values, both tracks crossfade linearly.
const float NEAR_DIST_M = 0.1f;  // meters
const float FAR_DIST_M  = 1.5f;  // meters

// -----------------------------------------------------------------------------
// DW3000 radio configuration
// -----------------------------------------------------------------------------
// This struct is passed to dwt_configure() to set up the UWB radio.
// Both anchor and listener must use identical settings or they won't hear each other.
static dwt_config_t config = {
    5,                    // Channel 5 (6.5 GHz band — good indoor range/multipath performance)
    DWT_PLEN_512,         // Preamble length 512 symbols — longer preamble improves acquisition reliability
    DWT_PAC16,            // Preamble acquisition chunk 16 — matched to 512-symbol preamble
    9,                    // TX preamble code 9 (channel 5 recommended code)
    9,                    // RX preamble code 9 (must match TX)
    1,                    // Non-standard 8-symbol SFD (Start Frame Delimiter)
    DWT_BR_6M8,           // Data rate: 6.8 Mbps (fastest option, shortest air time)
    DWT_PHRMODE_STD,      // Standard PHY header mode
    DWT_PHRRATE_STD,      // Standard PHY header rate
    (512 + 1 + 8 - 16),  // SFD timeout: preamble length + 1 + SFD length - PAC size
    DWT_STS_MODE_OFF,     // STS (Scrambled Timestamp Sequence) disabled — not needed here
    DWT_STS_LEN_64,       // STS length (unused since STS is off)
    DWT_PDOA_M0           // Phase Difference of Arrival disabled (single antenna, no angle measurement)
};

// -----------------------------------------------------------------------------
// Message frame templates
// -----------------------------------------------------------------------------
// The listener sends tx_poll_msg, and expects to receive something matching rx_resp_msg.
// Byte layout: [0x41, 0x88] = frame control (data frame, short addresses)
//              [SN]         = sequence number (incremented each cycle, index 2)
//              [0xCA, 0xDE] = PAN ID
//              ['W','A','V','E'] or ['V','E','W','A'] = application identifier
//              [0xE0] or [0xE1] = message type byte (poll vs response)
//              Trailing zeros = padding / timestamp fields (filled in at runtime)
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t frame_seq_nb = 0;    // Sequence number, wraps 0–255 automatically (uint8_t overflow)
static uint8_t rx_buffer[20];       // Scratch buffer for received frames (response is 20 bytes)
static uint32_t status_reg = 0;     // Cached copy of the DW3000 system status register
static double tof;                  // Computed time-of-flight in seconds
static double distance;             // Computed distance in meters (raw, before filtering)

// Shared between ranging (Core 1) and audio (Core 0).
// volatile tells the compiler not to cache this in a register — always re-read from RAM.
// -1.0 means "not yet acquired" and suppresses audio crossfade until first valid reading.
static volatile float filteredDistance = -1.0f;
static int consecutiveFailures = 0;  // Counter of back-to-back failed ranging cycles

// TX power / pulse config — defined in dw3000_config_options.cpp, shared by both sketches.
extern dwt_txconfig_t txconfig_options;

// -----------------------------------------------------------------------------
// LoopingSource
// -----------------------------------------------------------------------------
// AudioFileSourceLittleFS normally returns 0 bytes at end-of-file, which signals
// the AAC decoder that the stream is done. The decoder then calls stop(), which
// disconnects its AudioOutputMixerStub from the mixer — and the mixer stalls waiting
// for samples from that stub forever.
//
// To avoid this, we subclass AudioFileSourceLittleFS and override read() so that
// when the file ends, we immediately seek back to the beginning and keep reading.
// The decoder never sees EOF, never calls stop(), and the mixer stub stays connected.
// This is the cleanest way to loop AAC audio without restarting the decoder.
class LoopingSource : public AudioFileSourceLittleFS {
public:
    LoopingSource(const char *path) : AudioFileSourceLittleFS(path) {}

    uint32_t read(void *data, uint32_t len) override {
        uint32_t got = AudioFileSourceLittleFS::read(data, len);
        if (got == 0) {
            // End of file reached — loop back to the start
            seek(0, SEEK_SET);
            got = AudioFileSourceLittleFS::read(data, len);
        }
        return got;
    }
};

// -----------------------------------------------------------------------------
// audioTask — runs on Core 0
// -----------------------------------------------------------------------------
// Pinned to Core 0 so it doesn't compete with the ranging loop on Core 1.
// Continuously decodes both AAC streams, adjusts their gains based on the current
// filtered distance, and pushes samples to the I2S amp.
//
// Stack size: 16384 bytes — the Helix AAC decoder uses a lot of stack.
// Priority: 2 — higher than the default Arduino task priority.
static void audioTask(void *)
{
    // Set up I2S output to the MAX98357A amp
    AudioOutputI2S *i2sOut = new AudioOutputI2S();
    i2sOut->SetPinout(I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DIN_PIN);

    // The mixer takes two decoded audio streams and combines them into one I2S output.
    // Each "stub" is one input channel to the mixer.
    // IMPORTANT: gain must be set on the stubs, not on i2sOut directly — the mixer
    // calls begin() on i2sOut internally, which resets any gain set on i2sOut beforehand.
    AudioOutputMixer     *mixer      = new AudioOutputMixer(MIXER_BUF_SIZE, i2sOut);
    AudioOutputMixerStub *trackStub  = mixer->NewInput();   // Primary audio track channel
    AudioOutputMixerStub *staticStub = mixer->NewInput();   // Static/noise channel

    // Open both audio files from LittleFS. The files must be:
    //   - AAC-LC format, ADTS container (.aac — NOT .m4a)
    //   - Mono, 44100 Hz recommended
    //   - Uploaded via the LittleFS image upload tool before flashing
    LoopingSource *trackSrc  = new LoopingSource("/track.aac");
    LoopingSource *staticSrc = new LoopingSource("/static.aac");

    // One AAC decoder per track
    AudioGeneratorAAC *trackAac  = new AudioGeneratorAAC();
    AudioGeneratorAAC *staticAac = new AudioGeneratorAAC();

    // Connect each decoder to its file source and output stub, then start decoding
    if (!trackAac->begin(trackSrc, trackStub) || !staticAac->begin(staticSrc, staticStub)) {
        Serial.println("Audio: begin failed — check /track.aac and /static.aac in LittleFS");
        vTaskDelete(NULL);  // Kill this task cleanly if files are missing
        return;
    }

    for (;;) {
        // --- Compute crossfade gain from current distance ---
        // Read filteredDistance once so it doesn't change mid-calculation.
        // `a` is the "closeness" factor: 1.0 = at or closer than NEAR_DIST_M, 0.0 = at or farther than FAR_DIST_M.
        float dist = filteredDistance;
        float a = 0.0f;
        if (dist >= 0.0f) {  // Only crossfade once we have a valid distance reading
            a = 1.0f - (dist - NEAR_DIST_M) / (FAR_DIST_M - NEAR_DIST_M);
            if (a < 0.0f) a = 0.0f;  // Clamp: don't go below 0 (too far)
            if (a > 1.0f) a = 1.0f;  // Clamp: don't go above 1 (too close)
        }
        // The two gains always sum to MASTER_GAIN, so the overall output level stays constant.
        trackStub->SetGain(a * MASTER_GAIN);           // Close → loud
        staticStub->SetGain((1.0f - a) * MASTER_GAIN); // Close → quiet

        // Advance each decoder by one step (decodes one AAC frame worth of samples per call)
        trackAac->loop();
        staticAac->loop();

        // Yield for 1 RTOS tick so the IDLE0 task on Core 0 gets CPU time.
        // Without this, the Task Watchdog Timer (TWDT) will reset the board after ~5 seconds
        // because IDLE0 is never allowed to run.
        vTaskDelay(1);
    }
}

// -----------------------------------------------------------------------------
// setup — runs once on Core 1 at boot
// -----------------------------------------------------------------------------
void setup()
{
  // Start serial output (115200 baud) for debug messages
  UART_init();

  // Initialize the SPI bus and select the DW3000 chip
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  // Allow 2ms for the DW3000 to transition from INIT_RC to IDLE_RC after power-on
  delay(2);

  // The DW3000 must report IDLE_RC before any further initialization
  while (!dwt_checkidlerc())
  {
    UART_puts("IDLE FAILED\r\n");
    while (1);  // Halt — unrecoverable hardware error
  }

  // Initialize the DW3000 chip (loads OTP calibration values, sets up clocks, etc.)
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1);
  }

  // Enable the DW3000's onboard LEDs for visual TX/RX feedback during development
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Apply the radio configuration (channel, preamble, data rate, etc.)
  if (dwt_configure(&config))
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1);
  }

  // Apply TX power and pulse shaping settings (defined in dw3000_config_options.cpp)
  dwt_configuretxrf(&txconfig_options);

  // Set antenna delays — compensates for signal travel time inside the chip/antenna path.
  // Both sides (TX and RX) must use the same value as the anchor for accurate ranging.
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  // Tell the DW3000 how long after TX to open the receive window (in microseconds).
  // The anchor waits POLL_RX_TO_RESP_TX_DLY_UUS (900µs) before responding, so this
  // 240µs value leaves the listener opening its window well before the response arrives.
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);

  // If no response is received within this window, the DW3000 raises a timeout flag.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // Enable the LNA (low-noise amplifier) and PA (power amplifier) for maximum RF range
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Listener ready.");

  // Mount the LittleFS flash filesystem where the audio files are stored.
  // The `true` argument auto-formats the partition if it has never been initialized.
  // Audio files must be uploaded separately using the LittleFS image upload tool.
  if (!LittleFS.begin(true))
    Serial.println("LittleFS: mount failed");

  // Launch the audio decode/mix loop on Core 0, separate from the ranging loop (Core 1).
  // Stack size 16384 bytes — the AAC decoder needs significant stack space.
  xTaskCreatePinnedToCore(audioTask, "audio", 16384, NULL, 2, NULL, 0);
}

// -----------------------------------------------------------------------------
// loop — runs continuously on Core 1
// -----------------------------------------------------------------------------
// One ranging cycle per call. Sends a poll, waits for the anchor's response,
// computes distance, updates filteredDistance, and sleeps the rest of the 100ms window.
void loop()
{
  unsigned long loopStart = millis();

  // --- Transmit poll ---
  // Stamp the current sequence number into the frame before sending
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);  // Clear any stale TX status
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);           // Load poll into TX buffer
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);                    // Set frame length and ranging bit
  // DWT_RESPONSE_EXPECTED tells the DW3000 to automatically switch to RX after TX completes
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  // --- Wait for response, RX timeout, or RX error ---
  // Spin until the status register shows one of: good frame received, timeout, or error.
  // The DW3000 hardware handles the RX window timing automatically.
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  frame_seq_nb++;  // Increment sequence number for next cycle (wraps automatically at 256)

  bool gotValidReading = false;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)  // Good frame received
  {
    // Acknowledge the good RX event by clearing its bit in the status register
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    // Read the received frame into rx_buffer
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      // Zero out the sequence number byte before comparing — we don't validate SN,
      // only the fixed header bytes that identify this as a valid response from our anchor.
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        // --- Extract timestamps and compute distance ---
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        // Our own timestamps: when we sent the poll, and when we received the response
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        // Clock offset ratio compensates for the small frequency difference between
        // the listener's and anchor's DW3000 crystal oscillators. Without this correction,
        // accumulated clock drift adds error to the distance calculation.
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        // Anchor's timestamps: when it received our poll, and when it sent the response.
        // These are embedded in the response frame at the known byte offsets.
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        // SS-TWR formula:
        //   rtd_init = total round-trip time as seen by the listener
        //   rtd_resp = anchor's processing time (poll receipt → response TX)
        //   tof = (rtd_init - rtd_resp) / 2, corrected for clock offset
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;  // Convert time-of-flight to meters

        // Sanity check: SS-TWR can produce small negative values at very close range
        // due to noise — that's acceptable. Reject anything wildly out of range.
        if (distance > -0.5 && distance < 100.0)
        {
          // First valid reading: seed the filter directly (no history to blend with yet)
          if (filteredDistance < 0.0)
            filteredDistance = distance;
          else
            filteredDistance = DISTANCE_FILTER_ALPHA * distance + (1.0 - DISTANCE_FILTER_ALPHA) * filteredDistance;

          gotValidReading = true;
          consecutiveFailures = 0;
        }
      }
    }
  }
  else
  {
    // RX timeout or error — clear the relevant status bits so the next cycle starts clean
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  if (!gotValidReading)
    consecutiveFailures++;

  // --- Serial output every cycle for monitoring ---
  char dist_str[48];
  if (filteredDistance >= 0.0)
  {
    if (consecutiveFailures > MAX_CONSECUTIVE_FAILURES)
      snprintf(dist_str, sizeof(dist_str), "DIST: NO SIGNAL");
    else if (consecutiveFailures > 0)
      // Show last known distance with a stale counter so it's clear the reading is old
      snprintf(dist_str, sizeof(dist_str), "DIST: %.2f m (stale:%d)", filteredDistance, consecutiveFailures);
    else
      snprintf(dist_str, sizeof(dist_str), "DIST: %.2f m", filteredDistance);
  }
  else
  {
    snprintf(dist_str, sizeof(dist_str), "DIST: ACQUIRING...");
  }
  test_run_info((unsigned char *)dist_str);

  // --- Pace the cycle to RNG_INTERVAL_MS ---
  // Sleep only for however long is left in the 100ms window after the exchange.
  // This keeps the ranging rate consistent without a fixed blocking delay.
  unsigned long elapsed = millis() - loopStart;
  if (elapsed < RNG_INTERVAL_MS)
    Sleep(RNG_INTERVAL_MS - elapsed);
}

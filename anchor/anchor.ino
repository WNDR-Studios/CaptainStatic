// =============================================================================
// anchor.ino — SS TWR Responder
// =============================================================================
// Role in the installation:
//   This sketch runs on each STATIONARY anchor board, one per installation object.
//   The anchor does not compute distance — it just responds to the listener's polls
//   with precise timestamps so the listener can do the calculation.
//
// How it works (SS TWR — Single-Sided Two-Way Ranging):
//   1. The anchor enables its UWB radio receiver and waits for a poll frame.
//   2. When a poll arrives, it records the exact reception timestamp.
//   3. It schedules a response frame to transmit at a precisely timed future moment.
//   4. The response frame carries both timestamps (when poll arrived, when response was sent).
//   5. The listener uses those four timestamps (its own TX/RX + the anchor's RX/TX)
//      to cancel out the anchor's processing time and compute pure time-of-flight.
//   6. The anchor immediately loops back to listening for the next poll.
//
// No distance computation happens here — this board's only job is to be a precise
// timestamp echo server for its paired listener.
//
// Hardware:
//   Board : MakerFabs ESP32 DW3000
//   UWB   : Decawave DW3000 chip (SPI)
//   Note  : No audio hardware on the anchor — it is power-only.
// =============================================================================

#include "src/dw3000/dw3000.h"  // DW3000 UWB driver
#include "SPI.h"

// _fastSPI is defined in the DW3000 port layer. We override it here to set
// the SPI clock to 16 MHz, which the DW3000 supports and which reduces transfer time.
extern SPISettings _fastSPI;

// -----------------------------------------------------------------------------
// DW3000 SPI pin assignments
// -----------------------------------------------------------------------------
const int      PIN_RST = 27;  // DW3000 reset line
const int      PIN_IRQ = 34;  // DW3000 interrupt (used by driver even in polling mode)
const int      PIN_SS  = 4;   // SPI chip-select for DW3000

// -----------------------------------------------------------------------------
// Ranging timing constants
// -----------------------------------------------------------------------------
// Antenna delays compensate for the signal travel time inside the chip's analog
// front-end before/after the antenna. Must match the listener's values exactly.
const uint16_t TX_ANT_DLY               = 16385;
const uint16_t RX_ANT_DLY               = 16385;

// Message frame layout — must match listener exactly.
const int      ALL_MSG_COMMON_LEN        = 10;   // Number of header bytes to compare when validating a received frame
const int      ALL_MSG_SN_IDX            = 2;    // Byte index of the sequence number field in both frames

// Byte offsets within the outgoing response frame where the anchor writes its timestamps.
// The listener reads these to compute round-trip time.
const int      RESP_MSG_POLL_RX_TS_IDX   = 10;   // Offset of the "when we received the poll" timestamp
const int      RESP_MSG_RESP_TX_TS_IDX   = 14;   // Offset of the "when we are sending this response" timestamp
const int      RESP_MSG_TS_LEN           = 4;    // Each timestamp field is 4 bytes (lower 32 bits of the 40-bit DW3000 timestamp)

// How long after receiving the poll to schedule the response transmission.
// This must be long enough for the ESP32 to run the response-building code
// before the DW3000 hardware fires the delayed TX. 900µs gives comfortable margin
// even when FreeRTOS scheduling adds a small delay.
// NOTE: if you reduce this value, the delayed TX may fire before the CPU has
// finished setting it up, causing a dwt_starttx() error and a dropped exchange.
const int      POLL_RX_TO_RESP_TX_DLY_UUS = 900;

// If no poll arrives within this window, break out of the RX wait loop and
// call dwt_rxenable() again. This recovers from a stuck RX state machine.
// The listener sends polls every 100ms, so 500ms = 5 consecutive missed polls.
const int RX_POLL_TIMEOUT_MS = 500;

// If the delayed TX doesn't complete within this window, abandon the exchange.
// A properly timed delayed TX should fire within a couple of milliseconds.
const int TX_COMPLETE_TIMEOUT_MS = 20;

// -----------------------------------------------------------------------------
// DW3000 radio configuration
// -----------------------------------------------------------------------------
// Must be identical to the listener's config — same channel, preamble, data rate, etc.
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_512,     /* Preamble length. Used in TX only. */
    DWT_PAC16,        /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (512 + 1 + 8 - 16), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

// -----------------------------------------------------------------------------
// Message frame templates
// -----------------------------------------------------------------------------
// The anchor listens for rx_poll_msg, then replies with tx_resp_msg.
// Byte layout: [0x41, 0x88] = frame control (data frame, short addresses)
//              [SN]         = sequence number (index 2, zeroed before comparison)
//              [0xCA, 0xDE] = PAN ID
//              ['W','A','V','E'] or ['V','E','W','A'] = application identifier (reversed in response)
//              [0xE0] or [0xE1] = message type byte (poll vs response)
//              Trailing bytes  = timestamp fields, filled in at runtime
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t frame_seq_nb = 0;    // Sequence number, echoed back in the response frame
static uint8_t rx_buffer[20];       // Scratch buffer for received frames (poll is 12 bytes, buffer is generous)
static uint32_t status_reg = 0;     // Cached copy of the DW3000 system status register
static uint64_t poll_rx_ts;         // 64-bit timestamp: when we received the poll
static uint64_t resp_tx_ts;         // 64-bit timestamp: when we will/did transmit the response

// TX power / pulse config — defined in dw3000_config_options.cpp
extern dwt_txconfig_t txconfig_options;

// -----------------------------------------------------------------------------
// setup — runs once at boot
// -----------------------------------------------------------------------------
void setup()
{
  // Start serial output (115200 baud) for debug messages
  UART_init();

  // Override the default SPI speed to 16 MHz for faster DW3000 register access
  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);

  // Initialize the SPI bus and select the DW3000 chip
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range TX");
  Serial.println("Setup over........");
}

// -----------------------------------------------------------------------------
// loop — runs continuously
// -----------------------------------------------------------------------------
// Each iteration: enable RX → wait for a valid poll → build and send a timed response.
// The anchor never sleeps between cycles — it goes straight back to listening.
void loop()
{
  /* Clear any stale TX/RX status bits before enabling RX to ensure a clean state. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_ERR);

  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // --- Wait for a poll frame (or timeout) ---
  // Spin until the status register shows a good frame received or a receive error.
  // We use millis() to break out if nothing arrives — this prevents a permanent stall
  // if the DW3000's RX state machine gets stuck (e.g., after a malformed frame).
  /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
  unsigned long rxStart = millis();
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
  {
    /* If no poll arrives within the timeout the RX state machine may be stuck — break
     * and loop back to dwt_rxenable to restart it cleanly. */
    if (millis() - rxStart > RX_POLL_TIMEOUT_MS)
    {
      status_reg = 0;  // Clear so we don't process a false positive
      break;
    }
  };

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)  // Good frame received
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      // Zero out the sequence number byte before comparing — we don't validate SN,
      // only the fixed header bytes that identify this as a valid poll from our listener.
      /* Check that the frame is a poll sent by "SS TWR initiator" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t resp_tx_time;
        int ret;

        // --- Record when we received the poll ---
        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();

        // --- Schedule the response to transmit at a future time ---
        // Convert the poll RX timestamp to DW3000 delayed-TX units (divide by 256),
        // then add the processing delay. The DW3000 will fire the TX at exactly this moment.
        /* Compute response message transmission time. See NOTE 7 below. */
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        // --- Predict the response TX timestamp ---
        // We need to embed this in the response frame *before* it is transmitted.
        // The formula recovers the full 40-bit timestamp from the 32-bit delayed-TX register
        // value, then adds the antenna delay to get the actual over-the-air departure time.
        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        // --- Embed both timestamps into the response frame ---
        // The listener will extract these to compute time-of-flight.
        /* Write all timestamps in the final message. See NOTE 8 below. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        // --- Transmit the response at the scheduled time ---
        /* Write and send the response message. See NOTE 9 below. */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
        ret = dwt_starttx(DWT_START_TX_DELAYED);
        // DWT_START_TX_DELAYED tells the DW3000 to hold the frame and fire at resp_tx_time.
        // If the scheduled time has already passed (e.g., CPU was too slow), this returns an error.

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
        if (ret == DWT_SUCCESS)
        {
          // Wait for the TX to complete before looping back to RX.
          // The millis() guard prevents hanging here if something goes wrong.
          /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
          unsigned long txStart = millis();
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
          {
            /* Guard against a stuck TX — should never take more than a few ms. */
            if (millis() - txStart > TX_COMPLETE_TIMEOUT_MS)
              break;
          };

          /* Clear TXFRS event. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

          /* Increment frame sequence number after transmission of the poll message (modulo 256). */
          frame_seq_nb++;
        }
      }
    }
  }
  else
  {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}

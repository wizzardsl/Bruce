#pragma once
#include "rf_utils.h"
/*
 * rf_listen_decode.h
 *
 * CC1101-based RF pulse capture and rtl_433 decode integration.
 * Captures OOK/ASK pulse trains via GDO0 interrupt, builds a
 * pulse_data_t compatible with the rtl_433_ESP signalDecoder, and
 * dispatches it for decoding via processSignal().
 *
 * Dependencies:
 *   - ELECHOUSE_CC1101_SRC_DRV (CC1101 radio driver)
 *   - signalDecoder.h / signalDecoder.cpp  (rtl_433_ESP decoder stack)
 *   - bruceConfig / bruceConfigPins        (Bruce firmware platform globs)
 *   - display helpers: displayRedStripe(), displayError()
 *   - optional audio: BUZZ_PIN / HAS_NS4168_SPKR
 */

// ─── Pulse capture tuning ────────────────────────────────────────────────────

/**
 * Maximum number of pulse/gap pairs stored per train.
 * Must be <= PD_MAX_PULSES defined in rtl_433 (typically 1200).
 * Each pair costs 8 bytes, so 1024 pairs = 8 KB per raw buffer (×2 = 16 KB).
 */
#ifndef PULSE_BUF_SIZE
#define PULSE_BUF_SIZE 1024
#endif

/**
 * A gap longer than this value (µs) with no rising edge signals end-of-train.
 * 10 ms is the standard OOK inter-message gap used by rtl_433.
 * Raise to 15000–20000 if trains are being split prematurely.
 */
#ifndef PULSE_TRAIN_TIMEOUT_US
#define PULSE_TRAIN_TIMEOUT_US 10000
#endif

/**
 * JSON decode result buffer size (bytes).
 * rtl_433 serialises decoded data as JSON; increase if messages are truncated.
 */
#ifndef RF_DECODE_MSG_BUF_SIZE
#define RF_DECODE_MSG_BUF_SIZE 1024
#endif

// ─── Public entry point ──────────────────────────────────────────────────────

/**
 * @brief Interactive RF listen + decode screen.
 *
 * Flow:
 *   1. Frequency selection UI (Prev/Next to adjust, Sel to confirm).
 *   2. CC1101 initialised for raw OOK capture via ELECHOUSE driver.
 *   3. rtl_433 decoder task started (idempotent — safe on re-entry).
 *   4. GDO0 CHANGE interrupt fires onPulse() which accumulates pulse trains
 *      into a double-buffer.
 *   5. Main loop detects completed trains, builds pulse_data_t, hands off to
 *      processSignal() for rtl_433 decoding.
 *   6. Decoded JSON is printed to Serial; display shows pulse count / status.
 *   7. Esc exits and detaches the interrupt.
 */
void rf_listen_decode();


#include "RtlListener.h"
#include <Arduino.h> // for Serial and String
#include <cstdio>

RtlListener::RtlListener() : last_row_index_(-1), repeat_count_(0) { bitbuffer_clear(&bb_); }

void RtlListener::processRecording(RawRecording &recording) {
    for (size_t r = 0; r < recording.codes.size(); ++r) {
        rmt_symbol_word_t *symbols = recording.codes[r];
        size_t len = recording.codeLengths[r];
        uint16_t gap = (r < recording.gaps.size()) ? recording.gaps[r] : 0;

        for (size_t i = 0; i < len; ++i) {
            float duration_us = symbols[i].duration0 + symbols[i].duration1;
            int level = symbols[i].level0 ? 1 : 0;
            processPulse(duration_us, level);
        }

        // Optionally insert gap between packets
        // if (gap > 0) listener_.processGap(gap);
    }
}

void RtlListener::processRecording(RawRecording &recording, RtlLiveDisplay &display) {
    _display = &display;
    Serial.println(String("RtlListener: processRecording rows=") + String(recording.codes.size()));

    for (size_t r = 0; r < recording.codes.size(); ++r) {
        rmt_symbol_word_t *symbols = recording.codes[r];
        size_t len = recording.codeLengths[r];
        uint16_t gap = (r < recording.gaps.size()) ? recording.gaps[r] : 0;

        for (size_t i = 0; i < len; ++i) {
            float duration_us = symbols[i].duration0 + symbols[i].duration1;
            int level = symbols[i].level0 ? 1 : 0;
            processPulse(duration_us, level);
        }

        // Optionally insert gap between packets
        // if (gap > 0) listener_.processGap(gap);
    }

    // If pulses remain (no end sync seen), process them for debugging
    if (!pulses_.empty()) {
        Serial.println(String("RtlListener: flushing trailing pulses: ") + String(pulses_.size()));
        if (_display) _display->drawLine(String("Flushing pulses: ") + String(pulses_.size()));
        processBuffer();
        pulses_.clear();
    }
}
void RtlListener::processPulse(float duration_us, int level) {
    pulses_.push_back({duration_us, level});

    // Detect end of packet
    if (duration_us >= sync_threshold_us_ && !pulses_.empty()) {
        if (_display) _display->drawLine(String("End packet len: ") + String(pulses_.size()));
        processBuffer();
        pulses_.clear();
    }
}

void RtlListener::pulsesToBitbuffer() {
    bitbuffer_clear(&bb_);
    bitbuffer_add_row(&bb_);

    for (auto &p : pulses_) {
        int bit = (p.duration_us >= long_pulse_us_) ? 1 : 0;
        bitbuffer_add_bit(&bb_, bit);
    }
}

bool RtlListener::handleRepeats() {
    int repeated_row = bitbuffer_find_repeated_row(&bb_, 1, 8);
    if (repeated_row < 0) return false;

    if (last_row_index_ == repeated_row) repeat_count_++;
    else repeat_count_ = 1;

    last_row_index_ = repeated_row;
    return repeat_count_ >= min_repeat_count_;
}

void RtlListener::decodeBitbufferTransforms() {
    bitbuffer_t tmp;
    bitbuffer_manchester_decode(&bb_, 0, 0, &tmp, BITBUF_MAX_ROW_BITS);
    if (tmp.num_rows > 0) {
        bb_ = tmp;
        return;
    }

    bitbuffer_differential_manchester_decode(&bb_, 0, 0, &tmp, BITBUF_MAX_ROW_BITS);
    if (tmp.num_rows > 0) {
        bb_ = tmp;
        return;
    }

    bitbuffer_nrzs_decode(&bb_);
}

void RtlListener::processBuffer() {
    Serial.println("RtlListener: processBuffer");
    pulsesToBitbuffer();
    if (!handleRepeats()) {
        Serial.println("RtlListener: no repeated rows");
        if (_display) _display->drawLine("No repeated rows yet");
        return;
    }

    decodeBitbufferTransforms();
    if (_display) _display->drawLine("Looking for device");
    // padprintln("Looking for device");
    bool device_found = false;
// Iterate devices
#define DECL(d) &d,
    r_device *devices[] = {DEVICES};
#undef DECL
    int num_devices = sizeof(devices) / sizeof(devices[0]);

    for (int i = 0; i < num_devices; ++i) {
        r_device *dev = devices[i];
        int res = dev->decode_fn(dev, &bb_);
        if (res > 0) {
            Serial.println(String("RtlListener: device decoded: ") + String(dev->name));
            if (_display) _display->drawLine("[INFO] Device decoded!");
            if (_display) _display->drawLine(dev->name);
            // printf("[INFO] Device '%s' decoded!\n", dev->name);
            RtlLiveDisplay display(320, 170);
            if (_display) _display->drawData(data_make_from_bitbuffer(&bb_));
            device_found = true;
            break;
        }
    }

    if (!device_found) {
        Serial.println("RtlListener: no device matched");
        if (_display) _display->drawLine("No device matched.");
    }

    bitbuffer_clear(&bb_);
}
data_t *RtlListener::data_make_from_bitbuffer(const bitbuffer_t *bb) {
    if (!bb || bb->num_rows <= 0) return NULL;

    data_t *root = NULL;

    // Add number of rows
    root = data_int(root, "rows", "Rows", NULL, bb->num_rows);

    // For each row
    for (int row = 0; row < bb->num_rows; row++) {
        char key[32];
        snprintf(key, sizeof(key), "row_%d", row);

        // Bit count
        int bit_len = bb->bits_per_row[row];

        // Convert bits to hex string
        int byte_len = (bit_len + 7) / 8;
        char *hexbuf = (char *)malloc(byte_len * 2 + 1);
        if (!hexbuf) continue;

        for (int i = 0; i < byte_len; i++) { sprintf(hexbuf + i * 2, "%02X", bb->bb[row][i]); }
        hexbuf[byte_len * 2] = 0;

        // Create per-row data object
        data_t *row_data = NULL;

        row_data = data_int(row_data, "bits", "Bits", NULL, bit_len);

        row_data = data_str(row_data, "hex", "Hex", NULL, hexbuf);

        // Append this row to root
        root = data_dat(root, key, "Bit Row", NULL, row_data);

        free(hexbuf);
    }

    return root;
}

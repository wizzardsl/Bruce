/** @file
    This is a decoding Class that takes in pulse data from an RTL-SDR source (or CC1101) and
    attempts to decode known RF protocols using rtl_433 decoding methods.
    Use this as a listener for new bursts of data and to try and decode it using an RTL_433 style decoder.

    Created by Steven Lepage Feb 1 2026
 */

#pragma once
#include "RtlLiveDisplay.h"
#include "bitbuffer.h"
#include "data.h"
// #include "decoder_util.h"
#include "rtl_433_devices.h"
#include "structs.h"
#include <vector>

class RtlListener {
public:
    RtlListener();

    void processRecording(RawRecording &recording);
    void processPulse(float duration_us, int level);
    void processRecording(RawRecording &recording, RtlLiveDisplay &display);

private:
    struct Pulse {
        float duration_us;
        int level;
    };

    // State
    std::vector<Pulse> pulses_;
    bitbuffer_t bb_;
    int last_row_index_;
    int repeat_count_;

    // Constants
    static constexpr float short_pulse_us_ = 800.0f;
    static constexpr float long_pulse_us_ = 2000.0f;
    static constexpr float sync_threshold_us_ = 5000.0f;
    static constexpr int min_repeat_count_ = 2;

    // Internal helpers
    void pulsesToBitbuffer();
    bool handleRepeats();
    void decodeBitbufferTransforms();
    void processBuffer();
    data_t *data_make_from_bitbuffer(const bitbuffer_t *bb);
    RtlLiveDisplay *_display;
};

#include "pulse_data.h"
#include "structs.h"
#include <cmath>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN_PULSE_US 50        // ignore noise
#define MAX_PULSE_US 200000    // clamp absurd pulses
#define GAP_NORMALIZE_US 10000 // frame gap threshold (~rtl_433 PD_MIN_GAP_MS)

class BruceToRtl433 {
public:
    // Convert Bruce RAW string to rtl_433 pulse_data_t
    bool bruce_raw_to_pulse_data(pulse_data_t *out, const char *raw) {
        if (!out || !raw) return false;

        memset(out, 0, sizeof(pulse_data_t));
        out->num_pulses = 0;
        out->pulse[0] = 0;
        out->gap[0] = 0;

        const char *p = raw;
        int idx = 0;
        bool expecting_high = true;

        while (*p && idx < PD_MAX_PULSES) {
            // Skip until a number
            while (*p && !isdigit(*p) && *p != '-') p++;
            if (!*p) break;

            int val = strtol(p, (char **)&p, 10);
            int dur = abs(val);

            // ignore very small noise pulses
            if (dur < MIN_PULSE_US) continue;

            // clamp absurd pulses
            if (dur > MAX_PULSE_US) dur = MAX_PULSE_US;

            // normalize gaps
            if (!expecting_high && dur > GAP_NORMALIZE_US) {
                dur = GAP_NORMALIZE_US + 1; // matches rtl_433 convention
            }

            if (expecting_high) {
                out->pulse[idx] = dur;
                expecting_high = false;
            } else {
                out->gap[idx] = dur;
                expecting_high = true;
                idx++;
            }
        }

        out->num_pulses = idx;

        return (idx > 0);
    }

public:
    // Convert Bruce RawRecording rtl_433 pulse_data_t
    static bool convert(const RawRecording &recorded, pulse_data_t &out) {
        if (recorded.codes.empty()) return false;

        memset(&out, 0, sizeof(pulse_data_t));
        out.num_pulses = 0;
        out.gap[0] = 0;
        out.ook_low_estimate = 0;
        out.ook_high_estimate = 0;
        out.sample_rate = 1000000; // assume 1MHz = 1us per sample
        out.freq1_hz = recorded.frequency * 1e6;

        int idx = 0;

        for (size_t block = 0; block < recorded.codes.size(); ++block) {
            auto *codeArray = recorded.codes[block];
            size_t len = recorded.codeLengths[block];

            for (size_t j = 0; j < len; ++j) {
                if (idx >= PD_MAX_PULSES) break;

                // duration0 / level0
                if (codeArray[j].duration0 >= 50) { // ignore tiny pulses
                    out.pulse[idx] = codeArray[j].level0 ? codeArray[j].duration0 : 0;
                    out.gap[idx] = codeArray[j].level0 ? 0 : codeArray[j].duration0;
                    idx++;
                    if (idx >= PD_MAX_PULSES) break;
                }

                // duration1 / level1
                if (codeArray[j].duration1 >= 50) {
                    out.pulse[idx] = codeArray[j].level1 ? codeArray[j].duration1 : 0;
                    out.gap[idx] = codeArray[j].level1 ? 0 : codeArray[j].duration1;
                    idx++;
                    if (idx >= PD_MAX_PULSES) break;
                }
            }

            // insert gap between blocks
            if (block < recorded.gaps.size() && recorded.gaps[block] > 0) {
                uint32_t gap_us = static_cast<uint32_t>(std::round(recorded.gaps[block] * 1000));
                // rtl_433-style normalization
                if (gap_us > PD_MIN_GAP_MS * 1000) gap_us = PD_MIN_GAP_MS * 1000 + 1;
                if (idx < PD_MAX_PULSES) {
                    out.gap[idx] = gap_us;
                    out.pulse[idx] = 0; // zero pulse to indicate gap
                    idx++;
                }
            }
        }

        out.num_pulses = idx;
        return (idx > 0);
    }
};

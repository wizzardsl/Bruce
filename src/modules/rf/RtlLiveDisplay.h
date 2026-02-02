#pragma once
#include "data.h"
#include "structs.h"
#include <Arduino.h>
#include <vector>

/**
 * LiveDisplay: Streams decoded device data to TFT in real-time.
 */
class RtlLiveDisplay : public data_output_t {
public:
    int cursorX = 10;
    int cursorY = 30;
    int lineHeight = 14;
    int margin = 10;
    int tftWidth, tftHeight;

    RtlLiveDisplay(int width, int height);

    // Draw a single data_t linked list recursively
    void drawData(data_t *data);

    // Draw a data_array_t
    void drawArray(data_array_t *array);

    // Draw a line to TFT, handling scrolling
    void drawLine(const String &text);

    // Convenience function: feed decoded data
    void showDecoded(data_t *decoded);

    // Clear the TFT screen
    void clearScreen();
};

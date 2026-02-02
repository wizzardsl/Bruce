#include "RtlLiveDisplay.h"

/**
 * LiveDisplay: Streams decoded device data to TFT in real-time.
 */

RtlLiveDisplay::RtlLiveDisplay(int width, int height) : tftWidth(width), tftHeight(height) {
    log_level = 0; // show everything

    // Assign function pointers for data_output_t API
    print_data = [](data_output_t *out, data_t *data, const char *fmt) {
        auto self = static_cast<RtlLiveDisplay *>(out);
        self->drawData(data);
    };

    print_array = [](data_output_t *out, data_array_t *array, const char *fmt) {
        auto self = static_cast<RtlLiveDisplay *>(out);
        self->drawArray(array);
    };

    output_start = [](data_output_t *out, const char *const *fields, int num_fields) {
        auto self = static_cast<RtlLiveDisplay *>(out);
        self->cursorX = 10;
        self->cursorY = 30;
    };

    output_print = [](data_output_t *out, data_t *data) { out->print_data(out, data, nullptr); };

    output_free = [](data_output_t *out) {
        // Nothing to free for TFT streaming
    };
}

// Draw a single data_t linked list recursively
void RtlLiveDisplay::drawData(data_t *data) {
    while (data) {
        if (data->pretty_key) drawLine(String(data->pretty_key) + ": ");
        else if (data->key) drawLine(String(data->key) + ": ");

        switch (data->type) {
            case DATA_INT: drawLine(String(data->value.v_int)); break;
            case DATA_DOUBLE: drawLine(String(data->value.v_dbl, 2)); break;
            case DATA_STRING:
                if (data->value.v_ptr) drawLine(String((char *)data->value.v_ptr));
                break;
            case DATA_ARRAY:
                if (data->value.v_ptr) drawArray((data_array_t *)data->value.v_ptr);
                break;
            case DATA_DATA:
                if (data->value.v_ptr) drawData((data_t *)data->value.v_ptr);
                break;
            default: break;
        }
        data = data->next;
    }
}

// Draw a data_array_t
void RtlLiveDisplay::drawArray(data_array_t *array) {
    if (!array) return;

    for (int i = 0; i < array->num_values; i++) {
        switch (array->type) {
            case DATA_INT: drawLine(String(((int *)array->values)[i])); break;
            case DATA_DOUBLE: drawLine(String(((double *)array->values)[i], 2)); break;
            case DATA_STRING: drawLine(String(((char **)array->values)[i])); break;
            case DATA_DATA: drawData(&((data_t **)array->values)[i][0]); break;
            default: break;
        }
    }
}

// Draw a line to TFT, handling scrolling
void RtlLiveDisplay::drawLine(const String &text) {
    tft.setCursor(cursorX, cursorY);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.print(text);
    cursorY += lineHeight;

    // Scroll if we exceed screen height
    if (cursorY >= tftHeight - margin) {
        cursorY = 30;
        tft.fillRect(margin, 30, tftWidth - 2 * margin, tftHeight - 40, bruceConfig.bgColor);
    }
}

// Convenience function: feed decoded data
void RtlLiveDisplay::showDecoded(data_t *decoded) { output_print(this, decoded); }

// Clear the TFT screen
void RtlLiveDisplay::clearScreen() {
    tft.fillRect(margin, 30, tftWidth - 2 * margin, tftHeight - 40, bruceConfig.bgColor);
    cursorX = margin;
    cursorY = 30;
}

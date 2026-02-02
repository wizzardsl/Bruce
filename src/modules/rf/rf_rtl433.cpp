#include "rf_rtl433.h"
#include "RtlListener.h"
#ifndef TFT_MOSI
#define TFT_MOSI -1
#endif
#define STEP 5
#define stepMin 58
#define stepMax 812
#define JSON_MSG_BUFFER 512
char messageBuffer[JSON_MSG_BUFFER];

int count = 0;
RtlListener rtlListener;

static bool
record_rmt_rx_done_callback(rmt_channel_t *channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void rf_rtl433() {
    if (bruceConfigPins.rfModule != CC1101_SPI_MODULE) {
        displayError("RTL433 needs a CC1101!", true);
        return;
    }
    if (!initRfModule("rx", 433.92)) {
        displayError("CC1101 not found!", true);
        return;
    }

    int option, idx = 0;
select:

    option = 0;
    options = {
        {"433 MHz (default)",                [&]() { option = 3; }},
        {"868 MHz (common for EU sensors).", [&]() { option = 1; }},
        {"915 MHz (US ISM band).",           [&]() { option = 2; }},
    };

    idx = loopOptions(options, idx);

    tft.fillScreen(0x0);

    if (option == 3) {
        rf_rtl433_run(433.92);
        goto select;
    } else if (option == 1) {
        rf_rtl433_run(868.0);
        goto select;
    } else if (option == 2) {
        rf_rtl433_run(915.0);
        goto select;
    }
}

void rf_rtl433_run(float frequency) {

    padprintln("****** setup ******");
    // Initialize RF module and update display
    initRfModule(
        "rx", frequency
    ); // Frequency scan doesnt work when initializing the module with a different frequency
    Serial.println("RF Module Initialized");
    setMHZ(frequency);
    padprintln("****** setup complete ******");

    RawRecording recorded;

    recorded.codes.clear();
    recorded.codeLengths.clear();
    recorded.gaps.clear();
    recorded.frequency = 0;
    rf_raw_record_create(recorded);
}

void rf_raw_record_create(RawRecording &recorded) {
    RawRecordingStatus status;

    RtlLiveDisplay display(tftWidth, tftHeight);
    display.clearScreen();

    bool fakeRssiPresent = false;
    bool rssiFeature = false;
    rssiFeature = bruceConfigPins.rfModule == CC1101_SPI_MODULE;

    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder();

    if (rssiFeature) rf_range_selection(bruceConfigPins.rfFreq);

    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder();

    // Start recording
    delay(200);
    rmt_channel_handle_t rx_ch = NULL;
    rx_ch = setup_rf_rx();
    if (rx_ch == NULL) return;
    ESP_LOGI("RMT_SPECTRUM", "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = record_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_ch, &cbs, receive_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_ch));
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 3000,     // 10us minimum signal duration
        .signal_range_max_ns = 12000000, // 24ms maximum signal duration
    };
    rmt_symbol_word_t item[64];
    rmt_rx_done_event_data_t rx_data;
    ESP_ERROR_CHECK(rmt_receive(rx_ch, item, sizeof(item), &receive_config));
    Serial.println("RMT Initialized");
    padprintln("****** recording ******");
    while (!status.recordingFinished) {
        previousMillis = millis();
        size_t rx_size = 0;
        rmt_symbol_word_t *rx_items = NULL;
        if (xQueueReceive(receive_queue, &rx_data, 0) == pdPASS) {
            rx_size = rx_data.num_symbols;
            rx_items = rx_data.received_symbols;
        }
        if (rx_size != 0) {
            bool valid_signal = false;
            if (rx_size >= 5) valid_signal = true;
            if (valid_signal) {         // ignore codes shorter than 5 items
                fakeRssiPresent = true; // For rssi display on single-pinned RF Modules
                rmt_symbol_word_t *code = (rmt_symbol_word_t *)malloc(rx_size * sizeof(rmt_symbol_word_t));

                // Gap calculation
                unsigned long receivedTime = millis();
                unsigned long long signalDuration = 0;
                for (size_t i = 0; i < rx_size; i++) {
                    code[i] = rx_items[i];
                    signalDuration += rx_items[i].duration0 + rx_items[i].duration1;
                }
                recorded.codes.push_back(code);
                recorded.codeLengths.push_back(rx_size);

                if (status.lastSignalTime != 0) {
                    unsigned long signalDurationMs = signalDuration / RMT_1MS_TICKS;
                    uint16_t gap = (uint16_t)(receivedTime - status.lastSignalTime - signalDurationMs - 5);
                    recorded.gaps.push_back(gap);
                } else {
                    status.firstSignalTime = receivedTime;
                    status.recordingStarted = true;
                    // Erase sinewave animation
                    tft.drawPixel(0, 0, 0);
                    tft.fillRect(10, 30, tftWidth - 20, tftHeight - 40, bruceConfig.bgColor);
                }
                status.lastSignalTime = receivedTime;
            }

            padprintln("Sending Signel to RTL_433");
            tft.log_print("fdsf");
            rtlListener.processRecording(recorded, display);
            padprintln("Detection Complete.");
            ESP_ERROR_CHECK(rmt_receive(rx_ch, item, sizeof(item), &receive_config));
            rx_size = 0;
        }

        // Periodically update RSSI
        if (status.recordingStarted &&
            (status.lastRssiUpdate == 0 || millis() - status.lastRssiUpdate >= 100)) {
            if (fakeRssiPresent) status.latestRssi = -45;
            else status.latestRssi = -90;
            fakeRssiPresent = false;

            if (rssiFeature) status.latestRssi = ELECHOUSE_cc1101.getRssi();

            status.rssiCount++;
            status.lastRssiUpdate = millis();
        }

        // Stop recording after 20 seconds
        if (status.firstSignalTime > 0 && millis() - status.firstSignalTime >= 20000)
            status.recordingFinished = true;
        if (check(SelPress) && status.recordingStarted) status.recordingFinished = true;
        if (check(EscPress)) {
            status.recordingFinished = true;
            returnToMenu = true;
        }
    }
    Serial.println("Recording stopped.");

    padprintln("Recording stopped.");
    rmt_disable(rx_ch);
    rmt_del_channel(rx_ch);
    vQueueDelete(receive_queue);
    deinitRfModule();
}

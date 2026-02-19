#include "rtl_433_serial.h"

#ifndef RF_MODULE_FREQUENCY
#define RF_MODULE_FREQUENCY 433.92
#endif

#define JSON_MSG_BUFFER2 512

char messageBuffer2[JSON_MSG_BUFFER2];
rtl_433_ESP rf; // use -1 to disable transmitter
int count2 = 0;
int next2 = 0;

void rtl_433_serial_Callback(char *message) {
    JsonDocument jsonDocument;
    deserializeJson(jsonDocument, message);
    logJson2(jsonDocument);
    count2++;
}

void logJson2(JsonDocument jsondata) {

    char JSONmessageBuffer[measureJson(jsondata) + 1];
    serializeJson(jsondata, JSONmessageBuffer, measureJson(jsondata) + 1);
    Serial.printf("Received message : %s\n", JSONmessageBuffer);
}

void rtl433setup() {
    next2 = uptime() + 30;
    Serial.println("****** setup ******");
    rf.initReceiver(bruceConfigPins.CC1101_bus.io0, RF_MODULE_FREQUENCY);
    rf.setCallback(rtl_433_serial_Callback, messageBuffer2, JSON_MSG_BUFFER2);
    rf.enableReceiver();
    Serial.println("****** setup complete ******");
    rf.getModuleStatus();
}

unsigned long uptime() {
    static unsigned long lastUptime = 0;
    static unsigned long uptimeAdd = 0;
    unsigned long uptime = millis() / 1000 + uptimeAdd;
    if (uptime < lastUptime) {
        uptime += 4294967;
        uptimeAdd += 4294967;
    }
    lastUptime = uptime;
    return uptime;
}

void rtl433loop(void *pvParameters) {
    while (true) {
        if (uptime() > next2) {
            logprintfLn(0, "Received %d messages in %lu seconds", count2, uptime());
            next2 = uptime() + 30;
        }
        rf.loop();
        delay(100);
    }
}

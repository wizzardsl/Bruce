#include "log.h"
#include "rtl_433_ESP.h"
#include "structs.h"
#include <ArduinoJson.h>

void rtl433setup();
void rtl433loop(void *pvParameters);
void rtl_433_serial_Callback(char *message);
void logJson2(JsonDocument jsondata);
unsigned long uptime();
void rtl433exit();

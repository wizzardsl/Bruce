#pragma once
#include "rf_utils.h"
#include "structs.h"
#include <ArduinoJson.h>

#ifdef RF_CC1101
#include <rtl_433_ESP.h>
#endif

void rtl_433_Callback(char *message);
void logJson(JsonDocument jsondata);
void rf_raw_record_create(RawRecording &recorded);
// void rf_rtl433_run2(float frequency);
void rf_rtl433_run(float frequency);
void rf_rtl433();
// void printTaskList();

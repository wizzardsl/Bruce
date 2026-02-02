#pragma once
#include "rf_utils.h"
#include "structs.h"

void rf_raw_record_create(RawRecording &recorded);
void rf_rtl433_run(float frequency);
void rf_rtl433();
void rtl_433_Callback(char *message);

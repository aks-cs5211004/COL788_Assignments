#ifndef MAX30102_PROCESSING_H
#define MAX30102_PROCESSING_H

#include "max30102_for_stm32_hal.h"

float calculate_heart_rate(uint32_t latest_red_value);
float advanced_heartbeat_detection(uint32_t latest_red_value);
float calculate_spo2(uint32_t latest_red_value, uint32_t latest_ir_value);
void initialize_filters_1();
void initialize_filters_2();
#endif // MAX30102_PROCESSING_H

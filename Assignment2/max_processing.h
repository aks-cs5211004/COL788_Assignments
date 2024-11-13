#ifndef MAX30102_PROCESSING_H
#define MAX30102_PROCESSING_H

#include "max30102_for_stm32_hal.h"

void calculate_heart_rate(uint32_t latest_red_value);
void advanced_heartbeat_detection(uint32_t latest_red_value);
void calculate_spo2(uint32_t latest_red_value, uint32_t latest_ir_value);

#endif // MAX30102_PROCESSING_H

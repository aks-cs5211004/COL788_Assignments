#include "filters.h"
#include "max_processing.h"
#include <stdbool.h>
#include <stdio.h> // Include for printf (or replace with your UART transmission method)

// Constants
const float rThreshold = 0.7;   // Ratio threshold for heartbeat detection
const float decayRate = 0.02;   // Decay rate for max/min values
const float thrRate = 0.05;     // Rate to adjust the threshold
const int minDiff = 50;         // Minimum difference between max and min

// Current values
float maxValue = 0.0;
float minValue = 0.0; // Initialize to a high value for proper comparison
float threshold = 0.0;

// Timestamp of the last heartbeat
uint32_t lastHeartbeat = 0;

// Last value to detect crossing the threshold
float lastValue = 0.0;

int calculate_heart_rate(uint32_t latest_red_value) {
    float currentValue = latest_red_value;
    maxValue = fmaxf(maxValue, currentValue);
    minValue = fminf(minValue, currentValue);
    float nthreshold = (maxValue - minValue) * rThreshold + minValue;
    threshold = threshold * (1 - thrRate) + nthreshold * thrRate;
    threshold = fminf(maxValue, fmaxf(minValue, threshold));

    if (currentValue >= threshold && lastValue < threshold
        && (maxValue - minValue) > minDiff
        && (HAL_GetTick() - lastHeartbeat) > 300) {
        if (lastHeartbeat != 0) {
            // Calculate BPM
            int bpm = 60000 / (HAL_GetTick() - lastHeartbeat);

                lastHeartbeat = HAL_GetTick();
                return bpm;
        }
        lastHeartbeat = HAL_GetTick(); // Update the timestamp if a heartbeat is detected
    }

    // Decay for max/min
    maxValue -= (maxValue - currentValue) * decayRate;
    minValue += (currentValue - minValue) * decayRate;

    lastValue = currentValue;  // Update lastValue for the next iteration
    return -1;

}









// Advanced Heartbeat Detection



const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 50;
const int kSampleThreshold = 5;

// Filter Instances
static HighPassFilter high_pass_filter;
static LowPassFilter low_pass_filter;
static Differentiator differentiator;
static MovingAverageFilter averager;
// Function to initialize the filters
void initialize_filters_1() {
    MovingAverageFilter_init1(&averager, kAveragingSamples);
    Differentiator_init1(&differentiator, kSamplingFrequency);
    LowPassFilter_init2(&low_pass_filter, kLowPassCutoff, kSamplingFrequency);
    HighPassFilter_init2(&high_pass_filter, kHighPassCutoff, kSamplingFrequency);
}

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

int advanced_heartbeat_detection(uint32_t latest_red_value) {
    float current_value = latest_red_value;

    // Detect Finger using raw sensor value
    if (current_value > kFingerThreshold) {
        if (HAL_GetTick() - finger_timestamp > kFingerCooldownMs) {
            finger_detected = true;
        }
    } else {
        // Reset values if the finger is removed
        Differentiator_reset(&differentiator);
        MovingAverageFilter_reset(&averager);
        LowPassFilter_reset(&low_pass_filter);
        HighPassFilter_reset(&high_pass_filter);

        finger_detected = false;
        finger_timestamp = HAL_GetTick();
    }



    if (finger_detected) {
        current_value = LowPassFilter_process(&low_pass_filter, current_value);
        current_value = HighPassFilter_process(&high_pass_filter, current_value);
        float current_diff = Differentiator_process(&differentiator, current_value);

        // Valid values?
        if (!isnan(current_diff) && !isnan(last_diff)) {
            // Detect Heartbeat - Zero-Crossing
            if (last_diff > 0 && current_diff < 0) {
                crossed = true;
                crossed_time = HAL_GetTick();
            }

            if (current_diff > 0) {
                crossed = false;
            }

            // Detect Heartbeat - Falling Edge Threshold
            if (crossed && current_diff < kEdgeThreshold) {
                if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
                    // Show Results
                    int bpm = 60000 / (crossed_time - last_heartbeat);
                    if (bpm > 50 && bpm < 250) {
                        // Average?
                        if (kEnableAveraging) {
                            int average_bpm = MovingAverageFilter_process(&averager,bpm);

                            // Show if enough samples have been collected
                            if (averager.count > kSampleThreshold) {
                                return average_bpm;
                            }
                        }
//                        else {
//                            printf("Heart Rate (current, bpm): %d\n", bpm);
//                        }
                    }
                }

                crossed = false;
                last_heartbeat = crossed_time;
            }
        }

        last_diff = current_diff;
    }
    return -1;
}
















// spo2

// Sensor (adjust to your sensor type)
const float kSamplingFrequency_spo2 = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold_spo2 = 10000;
const unsigned int kFingerCooldownMs_spo2 = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold_spo2 = -2000.0;

// Filters
const float kLowPassCutoff_spo2 = 5.0;
const float kHighPassCutoff_spo2 = 0.5;

// Averaging
const bool kEnableAveraging_spo2 = false;
const int kAveragingSamples_spo2 = 5;
const int kSampleThreshold_spo2 = 5;

// Filter Instances
static LowPassFilter low_pass_filter_red;
static LowPassFilter low_pass_filter_ir;
static HighPassFilter high_pass_filter_spo2;
static Differentiator differentiator_spo2;
static MovingAverageFilter averager_bpm;
static MovingAverageFilter averager_r;
static MovingAverageFilter averager_spo2;
// Statistic for pulse oximetry
static MinMaxAvgStatistic stat_red;
static MinMaxAvgStatistic stat_ir;


void initialize_filters_2() {
    MovingAverageFilter_init1(&averager_bpm, kAveragingSamples_spo2);
    MovingAverageFilter_init1(&averager_r, kAveragingSamples_spo2);
    MovingAverageFilter_init1(&averager_spo2, kAveragingSamples_spo2);
    Differentiator_init1(&differentiator_spo2, kSamplingFrequency_spo2);
    LowPassFilter_init2(&low_pass_filter_red, kLowPassCutoff_spo2, kSamplingFrequency_spo2);
    LowPassFilter_init2(&low_pass_filter_ir, kLowPassCutoff_spo2, kSamplingFrequency_spo2);
    HighPassFilter_init2(&high_pass_filter_spo2, kHighPassCutoff_spo2, kSamplingFrequency_spo2);
    MinMaxAvgStatistic_init(&stat_red);
    MinMaxAvgStatistic_init(&stat_ir);
}


// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat_spo2 = 0;

// Timestamp for finger detection
long finger_timestamp_spo2 = 0;
bool finger_detected_spo2 = false;

// Last diff to detect zero crossing
float last_diff_spo2 = NAN;
bool crossed_spo2 = false;
long crossed_time_spo2 = 0;



float calculate_spo2(uint32_t latest_red_value, uint32_t latest_ir_value){
	float current_value_red = latest_red_value;
	float current_value_ir = latest_ir_value;
//	Detect Finger using raw sensor value
	  if(current_value_red > kFingerThreshold_spo2) {
	    if(HAL_GetTick()  - finger_timestamp_spo2 > kFingerCooldownMs_spo2) {
	      finger_detected_spo2 = true;
	    }
	  }
	  else {
	    // Reset values if the finger is removed

		  // Reset values if the finger is removed
		  Differentiator_reset(&differentiator_spo2);
		  MovingAverageFilter_reset(&averager_r);
		  MovingAverageFilter_reset(&averager_spo2);
		  MovingAverageFilter_reset(&averager_bpm);
		  LowPassFilter_reset(&low_pass_filter_red);
		  LowPassFilter_reset(&low_pass_filter_ir);
		  HighPassFilter_reset(&high_pass_filter_spo2);
		  MinMaxAvgStatistic_reset(&stat_red);
		  MinMaxAvgStatistic_reset(&stat_ir);
		  finger_detected_spo2 = false;
		  finger_timestamp_spo2 = HAL_GetTick();

	  }

	  if(finger_detected_spo2) {
	    current_value_red = LowPassFilter_process(&low_pass_filter_red , current_value_red);
	    current_value_ir = LowPassFilter_process(&low_pass_filter_ir, current_value_ir);

	    // Statistics for pulse oximetry(stat_red, )
	    MinMaxAvgStatistic_process(&stat_red, current_value_red);
	    MinMaxAvgStatistic_process(&stat_ir, current_value_ir);

	    // Heart beat detection using value for red LED
	    float current_value = HighPassFilter_process(&high_pass_filter_spo2, current_value_red);
	    float current_diff = Differentiator_process(&differentiator_spo2, current_value);

	    // Valid values?

	    if(!isnan(current_diff) && !isnan(last_diff_spo2)) {

	      // Detect Heartbeat - Zero-Crossing
	      if(last_diff_spo2 > 0 && current_diff < 0) {
	        crossed_spo2 = true;
	        crossed_time_spo2 = HAL_GetTick();
	      }

	      if(current_diff > 0) {
	        crossed_spo2 = false;
	      }

	      // Detect Heartbeat - Falling Edge Threshold
	      if(crossed_spo2 && current_diff < kEdgeThreshold_spo2) {
	        if(last_heartbeat_spo2 != 0 && crossed_time_spo2 - last_heartbeat_spo2 > 300) {
	          // Show Results
	          int bpm = 60000/(crossed_time_spo2 - last_heartbeat_spo2);
	          float rred = (MinMaxAvgStatistic_maximum(&stat_red)-MinMaxAvgStatistic_minimum(&stat_red))/MinMaxAvgStatistic_average(&stat_red);
	          float rir = (MinMaxAvgStatistic_maximum(&stat_ir)-MinMaxAvgStatistic_minimum(&stat_ir))/MinMaxAvgStatistic_average(&stat_ir);
	          float r = rred/rir;
	          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

	          if(bpm > 50 && bpm < 250) {
	            // Average?
	            if(kEnableAveraging_spo2) {
	              float average_bpm = MovingAverageFilter_process(&averager_bpm, bpm);
	              float average_r = MovingAverageFilter_process(&averager_r, r);
	              float average_spo2 = MovingAverageFilter_process(&averager_spo2,spo2);

	              // Show if enough samples have been collected
	              if(MovingAverageFilter_count(&averager_bpm) >= kSampleThreshold_spo2) {
//	            	  printf("Time (ms): %ld\r\n", HAL_GetTick() );
//	            	  printf("Heart Rate (avg, bpm): %f\r\n", average_bpm);
//	            	  printf("R-Value (avg): %f\r\n", average_r);
	            	  return average_spo2;

	              }
	            }
//	            else {
//	            	printf("Time (ms): %ld\r\n", HAL_GetTick());
//	            	printf("Heart Rate (current, bpm): %d\r\n", bpm);
//	            	printf("R-Value (current): %f\r\n", r);
//	            	printf("SpO2 (current, %%): %f\r\n", spo2);
//	            }
	          }

	          // Reset statistic
	          MinMaxAvgStatistic_reset(&stat_red);
	          MinMaxAvgStatistic_reset(&stat_ir);
	        }

	        crossed_spo2 = false;
	        last_heartbeat_spo2 = crossed_time_spo2;
	      }
	    }

	    last_diff_spo2 = current_diff;
	  }
	  return -1;
}




















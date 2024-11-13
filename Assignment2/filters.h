#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>


#define PI 3.14159265358979323846



// Structure to hold min, max, sum, and count values
typedef struct {
    float min;
    float max;
    float sum;
    int count;
} MinMaxAvgStatistic;

// Function to initialize the statistic block
void MinMaxAvgStatistic_init(MinMaxAvgStatistic *stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0;
    stat->count = 0;
}

// Function to add a value to the statistics
void MinMaxAvgStatistic_process(MinMaxAvgStatistic *stat, float value) {
    if (isnan(stat->min)) {
        stat->min = value;
    } else {
        stat->min = fmin(stat->min, value);
    }

    if (isnan(stat->max)) {
        stat->max = value;
    } else {
        stat->max = fmax(stat->max, value);
    }

    stat->sum += value;
    stat->count++;
}

// Function to reset the statistics
void MinMaxAvgStatistic_reset(MinMaxAvgStatistic *stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0;
    stat->count = 0;
}

// Function to get the minimum value
float MinMaxAvgStatistic_minimum(const MinMaxAvgStatistic *stat) {
    return stat->min;
}

// Function to get the maximum value
float MinMaxAvgStatistic_maximum(const MinMaxAvgStatistic *stat) {
    return stat->max;
}

// Function to get the average value
float MinMaxAvgStatistic_average(const MinMaxAvgStatistic *stat) {
    if (stat->count == 0) {
        return NAN;  // Return NAN if no values have been added
    }
    return stat->sum / stat->count;
}






// High Pass Filter
typedef struct {
    float kX;
    float kA0;
    float kA1;
    float kB1;
    float last_filter_value;
    float last_raw_value;
} HighPassFilter;

/**s
 * @brief Initialize the High Pass Filter
 * @param samples Number of samples until decay to 36.8 %
 */
void HighPassFilter_init1(HighPassFilter* filter, float samples) {
    filter->kX = exp(-1 / samples);
    filter->kA0 = (1 + filter->kX) / 2;
    filter->kA1 = -filter->kA0;
    filter->kB1 = filter->kX;
    filter->last_filter_value = NAN;
    filter->last_raw_value = NAN;
}

void HighPassFilter_init2(HighPassFilter* filter, float cutoff, float sampling_frequency){
    HighPassFilter_init1(filter, sampling_frequency/(cutoff*2*PI));
}



/**
 * @brief Applies the high pass filter
 */
float HighPassFilter_process(HighPassFilter* filter, float value) {
    if (isnan(filter->last_filter_value) || isnan(filter->last_raw_value)) {
        filter->last_filter_value = 0.0;
    } else {
        filter->last_filter_value =
            filter->kA0 * value
            + filter->kA1 * filter->last_raw_value
            + filter->kB1 * filter->last_filter_value;
    }

    filter->last_raw_value = value;
    return filter->last_filter_value;
}

/**
 * @brief Resets the stored values
 */
void HighPassFilter_reset(HighPassFilter* filter) {
    filter->last_raw_value = NAN;
    filter->last_filter_value = NAN;
}

// Low Pass Filter
typedef struct {
    float kX;
    float kA0;
    float kB1;
    float last_value;
} LowPassFilter;

/**
 * @brief Initialize the Low Pass Filter
 * @param samples Number of samples until decay to 36.8 %
 */
void LowPassFilter_init1(LowPassFilter* filter, float samples) {
    filter->kX = exp(-1 / samples);
    filter->kA0 = 1 - filter->kX;
    filter->kB1 = filter->kX;
    filter->last_value = NAN;
}

void LowPassFilter_init2(LowPassFilter* filter,float cutoff, float sampling_frequency){
	LowPassFilter_init1(filter, sampling_frequency/(cutoff*2*PI));
}

/**
 * @brief Applies the low pass filter
 */
float LowPassFilter_process(LowPassFilter* filter, float value) {
    if (isnan(filter->last_value)) {
        filter->last_value = value;
    } else {
        filter->last_value = filter->kA0 * value + filter->kB1 * filter->last_value;
    }
    return filter->last_value;
}

/**
 * @brief Resets the stored values
 */
void LowPassFilter_reset(LowPassFilter* filter) {
    filter->last_value = NAN;
}

// Differentiator
typedef struct {
    float kSamplingFrequency;
    float last_value;
} Differentiator;

/**
 * @brief Initializes the differentiator
 */
void Differentiator_init1(Differentiator* diff, float sampling_frequency) {
    diff->kSamplingFrequency = sampling_frequency;
    diff->last_value = NAN;
}

/**
 * @brief Applies the differentiator
 */
float Differentiator_process(Differentiator* diff, float value) {
    float diff_value = (value - diff->last_value) * diff->kSamplingFrequency;
    diff->last_value = value;
    return diff_value;
}

/**
 * @brief Resets the stored values
 */
void Differentiator_reset(Differentiator* diff) {
    diff->last_value = NAN;
}

// Moving Average Filter

typedef struct {
    int index;
    int count;
    int buffer_size;    // Store the buffer size
    float *values;
} MovingAverageFilter;

/**
 * @brief Initializes moving average filter
 */
void MovingAverageFilter_init1(MovingAverageFilter* filter, int buffer_size) {
    filter->index = 0;
    filter->count = 0;
    filter->buffer_size = buffer_size;
    filter->values = (float *)malloc(buffer_size * sizeof(float));  // Allocate memory for values
    memset(filter->values, 0, buffer_size * sizeof(float));  // Initialize buffer with zeros

}

/**
 * @brief Applies the moving average filter
 */
float MovingAverageFilter_process(MovingAverageFilter* filter, float value) {
    // Add value
    filter->values[filter->index] = value;

    // Increase index and count
    filter->index = (filter->index + 1) % filter->buffer_size;
    if (filter->count < filter->buffer_size) {
        filter->count++;
    }

    // Calculate sum
    float sum = 0.0;
    for (int i = 0; i < filter->count; i++) {
        sum += filter->values[i];
    }

    // Calculate average
    return sum / filter->count;
}

/**
 * @brief Resets the stored values
 */
void MovingAverageFilter_reset(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
}

/**
 * @brief Get number of samples
 * @return Number of stored samples
 */
int MovingAverageFilter_count(MovingAverageFilter* filter) {
    return filter->count;
}


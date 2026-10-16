#ifndef SIGNALQUALITY_H
#define SIGNALQUALITY_H

#include <stdbool.h>

typedef enum
{
    SIGNAL_QUALITY_GOOD = 0,
    SIGNAL_QUALITY_INVALID_INPUT,
    SIGNAL_QUALITY_TOO_FEW_SAMPLES,
    SIGNAL_QUALITY_NONFINITE_DATA,
    SIGNAL_QUALITY_FLAT_SIGNAL,
    SIGNAL_QUALITY_SATURATED,
    SIGNAL_QUALITY_TOO_FEW_BEATS,
    SIGNAL_QUALITY_INVALID_BEAT_TIMING
} SignalQualityStatus;

typedef struct
{
    SignalQualityStatus status;
    double signal_min;
    double signal_max;
    double signal_range;
    int valid_intervals;
    int invalid_intervals;
} SignalQualityResult;

SignalQualityResult CheckSignalQuality(
    const double ppg[],
    int num_samples,
    double sample_rate,
    const int beat_indices[],
    int num_beats
);

#endif

#include "SignalQuality.h"

#include <math.h>
#include <stddef.h>

/*
 * SignalQuality.c
 *
 * Performs basic quality checks on a single PPG signal and its
 * detected heartbeat indices.
 *
 * Assumptions:
 *   - beat_indices contains sample indices, not timestamps.
 *   - sample_rate is expressed in Hz (samples/second).
 *   - The MAX30102 ADC output is 18-bit (0 to 262143).
 *   - Expected heart rate is 40 to 200 BPM.
 *
 * Adjust these constants to match your sensor configuration.
 */ 

#define MINIMUM_SAMPLES          10
#define MINIMUM_BEATS            5

#define MIN_HEART_RATE_BPM       40.0
#define MAX_HEART_RATE_BPM       200.0

#define ADC_MIN_VALUE            0.0
#define ADC_MAX_VALUE            262143.0
#define ADC_RAIL_MARGIN          2.0

/* A signal is considered saturated if at least this fraction
 * of its samples are at or near an ADC rail.
 */
#define SATURATION_FRACTION      0.01

/* A signal with a range below this threshold is considered flat. */
#define FLAT_SIGNAL_EPSILON      1e-9


/*
 * Initialize a result with safe default values.
 */
static SignalQualityResult InitializeResult(void)
{
    SignalQualityResult result;

    result.status = SIGNAL_QUALITY_INVALID_INPUT;
    result.signal_min = 0.0;
    result.signal_max = 0.0;
    result.signal_range = 0.0;
    result.valid_intervals = 0;
    result.invalid_intervals = 0;

    return result;
}


/*
 * Return the first quality error detected.
 *
 * The signal statistics and interval counts are retained in the
 * result even if a later quality check fails.
 */
SignalQualityResult CheckSignalQuality(
    const double ppg[],
    int num_samples,
    double sample_rate,
    const int beat_indices[],
    int num_beats)
{
    SignalQualityResult result = InitializeResult();

    int i;
    int saturated_samples = 0;
    int interval_count;
    int minimum_interval;
    int maximum_interval;

    /*
     * 1. Validate the function arguments.
     */
    if (ppg == NULL ||
        beat_indices == NULL ||
        num_samples <= 0 ||
        num_beats < 0 ||
        !isfinite(sample_rate) ||
        sample_rate <= 0.0)
    {
        result.status = SIGNAL_QUALITY_INVALID_INPUT;
        return result;
    }

    if (num_samples < MINIMUM_SAMPLES)
    {
        result.status = SIGNAL_QUALITY_TOO_FEW_SAMPLES;
        return result;
    }

    /*
     * 2. Check for NaN or infinity before performing statistics.
     */
    for (i = 0; i < num_samples; i++)
    {
        if (!isfinite(ppg[i]))
        {
            result.status = SIGNAL_QUALITY_NONFINITE_DATA;
            return result;
        }
    }

    /*
     * 3. Calculate minimum, maximum, and range.
     */
    result.signal_min = ppg[0];
    result.signal_max = ppg[0];

    for (i = 1; i < num_samples; i++)
    {
        if (ppg[i] < result.signal_min)
        {
            result.signal_min = ppg[i];
        }

        if (ppg[i] > result.signal_max)
        {
            result.signal_max = ppg[i];
        }
    }

    result.signal_range =
        result.signal_max - result.signal_min;

    /*
     * 4. Reject flat or nearly constant signals.
     */
    if (result.signal_range <= FLAT_SIGNAL_EPSILON)
    {
        result.status = SIGNAL_QUALITY_FLAT_SIGNAL;
        return result;
    }

    /*
     * 5. Check for ADC saturation.
     *
     * The MAX30102 has an 18-bit ADC. A sample near either
     * rail is counted as saturated. The signal is rejected
     * when at least 1% of samples are near an ADC rail.
     */
    for (i = 0; i < num_samples; i++)
    {
        if (ppg[i] <= ADC_MIN_VALUE + ADC_RAIL_MARGIN ||
            ppg[i] >= ADC_MAX_VALUE - ADC_RAIL_MARGIN)
        {
            saturated_samples++;
        }
    }

    if (((double)saturated_samples / (double)num_samples)
        >= SATURATION_FRACTION)
    {
        result.status = SIGNAL_QUALITY_SATURATED;
        return result;
    }

    /*
     * 6. Require enough detected beats.
     *
     * Three beats provide two intervals, allowing a basic
     * check of consistency between consecutive heartbeats.
     */
    if (num_beats < MINIMUM_BEATS)
    {
        result.status = SIGNAL_QUALITY_TOO_FEW_BEATS;
        return result;
    }

    /*
     * 7. Convert the expected physiological heart-rate
     * limits into allowable intervals in samples.
     *
     * interval = sample_rate * 60 / heart_rate
     */
    minimum_interval =
        (int)(sample_rate * 60.0 / MAX_HEART_RATE_BPM);

    maximum_interval =
        (int)(sample_rate * 60.0 / MIN_HEART_RATE_BPM);

    /*
     * Prevent the minimum interval from becoming zero
     * at unusually low sample rates.
     */
    if (minimum_interval < 1)
    {
        minimum_interval = 1;
    }

    /*
     * 8. Validate beat indices and count valid intervals.
     */
    result.valid_intervals = 0;
    result.invalid_intervals = 0;

    /*
     * Every beat must refer to a sample in the signal.
     */
    for (i = 0; i < num_beats; i++)
    {
        if (beat_indices[i] < 0 ||
            beat_indices[i] >= num_samples)
        {
            result.status = SIGNAL_QUALITY_INVALID_BEAT_TIMING;
            return result;
        }
    }

    interval_count = num_beats - 1;

    for (i = 0; i < interval_count; i++)
    {
        int interval =
            beat_indices[i + 1] - beat_indices[i];

        /*
         * Reject duplicate, reversed, implausibly short,
         * or implausibly long beat intervals.
         */
        if (interval < minimum_interval ||
            interval > maximum_interval)
        {
            result.invalid_intervals++;
        }
        else
        {
            result.valid_intervals++;
        }
    }

    /*
     * 9. Require at least one valid interval and reject
     * the signal if any interval has invalid timing.
     *
     * This conservative policy avoids estimating BP from
     * a window containing potentially missed or false beats.
     */
    if (result.valid_intervals == 0 ||
        result.invalid_intervals > 0)
    {
        result.status = SIGNAL_QUALITY_INVALID_BEAT_TIMING;
        return result;
    }

    /*
     * 10. All quality checks passed.
     */
    result.status = SIGNAL_QUALITY_GOOD;

    return result;
}

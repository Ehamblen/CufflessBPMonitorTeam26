#include <stdio.h>
#include <math.h>

#include "findheartbeat.h"

#define MIN_BPM 40.0
#define MAX_BPM 200.0

#define BASELINE_WINDOW 101
#define SMOOTHING_WINDOW 5
#define THRESHOLD_FRACTION 0.15



 
  //Instead of storing the entire filtered waveform, this cursor
  //generates filtered samples one at a time.
 
typedef struct
{
    const double *ppg;
    int num_samples;

    int next_output;
    int next_detrended;

    double detrended[SMOOTHING_WINDOW];
} FilterCursor;



  //Initialize the filter cursor.

static void FilterCursor_Init(
    FilterCursor *cursor,
    const double ppg[],
    int num_samples)
{
    cursor->ppg = ppg;
    cursor->num_samples = num_samples;
    cursor->next_output = 0;
    cursor->next_detrended = 0;

    for (int i = 0; i < SMOOTHING_WINDOW; i++)
    {
        cursor->detrended[i] = 0.0;
    }
}


/*
 * Generate the next filtered sample.
 *
 * Processing:
 * 1. Estimate the local baseline using a moving average.
 * 2. Subtract the baseline from the raw PPG.
 * 3. Smooth the detrended signal with a moving average.
 *
 * Only a small circular buffer is required.
 */
static double FilterCursor_Next(FilterCursor *cursor)
{
    int output_index = cursor->next_output++;

    int half_baseline_window = BASELINE_WINDOW / 2;
    int half_smoothing_window = SMOOTHING_WINDOW / 2;

    int required_end = output_index + half_smoothing_window;

    if (required_end >= cursor->num_samples)
    {
        required_end = cursor->num_samples - 1;
    }

    // Generate detrended samples up to the required end index.
    while (cursor->next_detrended <= required_end)
    {
        int index = cursor->next_detrended++;

        int start = index - half_baseline_window;
        int end = index + half_baseline_window;

        if (start < 0)
        {
            start = 0;
        }

        if (end >= cursor->num_samples)
        {
            end = cursor->num_samples - 1;
        }

        double baseline_sum = 0.0;
        int baseline_count = 0;

        for (int sample = start; sample <= end; sample++)
        {
            baseline_sum += cursor->ppg[sample];
            baseline_count++;
        }

        double baseline = baseline_sum / baseline_count;
        double detrended_value = cursor->ppg[index] - baseline;

        cursor->detrended[index % SMOOTHING_WINDOW] =
            detrended_value;
    }

    /*
     * Average the available detrended samples around the
     * current output index.
     */
    int start = output_index - half_smoothing_window;
    int end = output_index + half_smoothing_window;

    if (start < 0)
    {
        start = 0;
    }

    if (end >= cursor->num_samples)
    {
        end = cursor->num_samples - 1;
    }

    double sum = 0.0;
    int count = 0;

    for (int sample = start; sample <= end; sample++)
    {
        sum += cursor->detrended[sample % SMOOTHING_WINDOW];
        count++;
    }

    return sum / count;
}


/*
 * Find heartbeats in a PPG waveform.
 *
 * Parameters:
 *   ppg          - Input PPG samples
 *   num_samples  - Number of input samples
 *   sample_rate  - Sampling rate in Hz
 *   beat_indices - Output array of detected peak indices
 *
 * Returns:
 *   Number of detected heartbeats, up to MAX_BEATS.
 */
int FindHeartbeats(
    const double ppg[],
    int num_samples,
    double sample_rate,
    int beat_indices[])
{
    if (ppg == NULL ||
        beat_indices == NULL ||
        num_samples < 3 ||
        sample_rate <= 0.0)
    {
        return 0;
    }

    if (num_samples > MAX_SAMPLES)
    {
        num_samples = MAX_SAMPLES;
    }

    /*
     * Minimum separation between detected beats.
     *
     * For example, at 100 Hz and 200 BPM:
     * 100 * 60 / 200 = 30 samples.
     */
    int min_beat_distance =
        (int)(sample_rate * 60.0 / MAX_BPM);

    if (min_beat_distance < 1)
    {
        min_beat_distance = 1;
    }

    /*
     * STEP 1: Calculate the minimum and maximum filtered values.
     *
     * The first pass determines an adaptive threshold based
     * on the range of the detrended and smoothed signal.
     */
    FilterCursor cursor;

    FilterCursor_Init(&cursor, ppg, num_samples);

    double first_filtered = FilterCursor_Next(&cursor);

    double signal_min = first_filtered;
    double signal_max = first_filtered;

    for (int i = 1; i < num_samples; i++)
    {
        double filtered = FilterCursor_Next(&cursor);

        if (filtered < signal_min)
        {
            signal_min = filtered;
        }

        if (filtered > signal_max)
        {
            signal_max = filtered;
        }
    }

    double signal_range = signal_max - signal_min;

    double adaptive_threshold =
        signal_range * THRESHOLD_FRACTION;

    printf("\r\n");
    printf("Heartbeat detector debug:\r\n");
    printf("Samples:             %d\r\n", num_samples);
    printf("Sample rate:         %.2f Hz\r\n", sample_rate);
    printf("Signal minimum:      %.2f\r\n", signal_min);
    printf("Signal maximum:      %.2f\r\n", signal_max);
    printf("Signal range:        %.2f\r\n", signal_range);
    printf("Adaptive threshold:  %.2f\r\n", adaptive_threshold);
    printf("Min beat distance:   %d samples\r\n", min_beat_distance);
    printf("\r\n");

    /*
     * STEP 2: Find local maxima using a three-sample window.
     *
     * Reinitialize the cursor because the first pass consumed
     * all filtered samples.
     */
    FilterCursor_Init(&cursor, ppg, num_samples);

    double previous_filtered = FilterCursor_Next(&cursor);
    double current_filtered  = FilterCursor_Next(&cursor);
    double next_filtered     = FilterCursor_Next(&cursor);

    int num_beats = 0;
    int last_beat = -min_beat_distance;

    for (int i = 1; i < num_samples - 1; i++)
    {
        int is_local_max =
            current_filtered > previous_filtered &&
            current_filtered >= next_filtered;

        int valid_peak = is_local_max;

        /*
         * Reject peaks below the adaptive threshold.
         */
        if (valid_peak &&
            current_filtered < adaptive_threshold)
        {
            valid_peak = 0;
        }

        /*
         * Reject peaks that are too close to the previous beat.
         */
        if (valid_peak &&
            i - last_beat < min_beat_distance)
        {
            valid_peak = 0;
        }

        /*
         * Record a valid heartbeat.
         */
        if (valid_peak && num_beats < MAX_BEATS)
        {
            beat_indices[num_beats] = i;

            double time_ms =
                ((double)i / sample_rate) * 1000.0;

            double bpm = 0.0;

            if (num_beats > 0)
            {
                int samples_between =
                    i - beat_indices[num_beats - 1];

                double seconds_between =
                    (double)samples_between / sample_rate;

                if (seconds_between > 0.0)
                {
                    bpm = 60.0 / seconds_between;
                }
            }

            printf(
                "Heartbeat %2d: sample = %4d, "
                "time = %8.2f ms, amplitude = %8.2f, "
                "BPM = %7.2f\r\n",
                num_beats + 1,
                i,
                time_ms,
                current_filtered,
                bpm
            );

            num_beats++;
            last_beat = i;
        }

        /*
         * CRITICAL: Advance the window on every iteration.
         * Do not put this behind a condition or continue.
         */
        previous_filtered = current_filtered;
        current_filtered = next_filtered;

        if (i < num_samples - 2)
        {
            next_filtered = FilterCursor_Next(&cursor);
        }
    }

    printf("\r\n");
    printf("Total heartbeats detected: %d\r\n", num_beats);

    return num_beats;
}
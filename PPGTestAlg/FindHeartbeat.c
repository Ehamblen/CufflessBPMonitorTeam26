#include <stdio.h>
#include <math.h>

#include "findheartbeat.h"

#define MIN_BPM 40.0
#define MAX_BPM 200.0

/*
 * Number of samples used to estimate the slowly-moving baseline.
 *
 * At 100 Hz:
 *      101 samples = 1.01 seconds
 *
 * The heartbeat is much faster than this baseline movement,
 * so this helps remove the floating DC component of the PPG.
 */
#define BASELINE_WINDOW 101

/*
 * Small smoothing filter.
 *
 * At 100 Hz, this is a 5-sample = 50 ms moving average.
 */
#define SMOOTHING_WINDOW 5

/*
 * Fraction of the signal range used as the initial adaptive
 * heartbeat threshold.
 */
#define THRESHOLD_FRACTION 0.15

typedef struct
{
    const double *ppg;
    int num_samples;
    int next_output;
    int next_detrended;
    double detrended[SMOOTHING_WINDOW];
} FilterCursor;

static void FilterCursor_Init(
    FilterCursor *cursor,
    const double ppg[],
    int num_samples
)
{
    cursor->ppg = ppg;
    cursor->num_samples = num_samples;
    cursor->next_output = 0;
    cursor->next_detrended = 0;
}

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

        cursor->detrended[index % SMOOTHING_WINDOW] =
            cursor->ppg[index] - baseline_sum / baseline_count;
    }

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


int FindHeartbeats(const double ppg[],int num_samples,double sample_rate,int beat_indices[])
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
     * STEP 1: Determine minimum distance between heartbeats 
     *
     * At 100 Hz and 200 BPM:
     *
     *     100 * 60 / 200 = 30 samples
     *
     * This prevents one heartbeat from being detected multiple
     * times because of small bumps/noise around the peak. 
     */
    int min_beat_distance =
        (int)(sample_rate * 60.0 / MAX_BPM);


    /*
     * STEP 2: Remove the floating baseline
     *
     * We estimate the baseline using a moving average.
     *
     *     baseline = slow-changing component
     *
     * Then:
     *
     *     detrended = raw PPG - baseline
     */
    /*
     * STEP 3: Determine an adaptive threshold
     *
     * Find the range of the FILTERED signal.
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
     * STEP 5: Find local maxima
     */
    int num_beats = 0;

    int last_beat = -min_beat_distance;

    FilterCursor_Init(&cursor, ppg, num_samples);
    double previous_filtered = FilterCursor_Next(&cursor);
    double current_filtered = FilterCursor_Next(&cursor);
    double next_filtered = FilterCursor_Next(&cursor);

    for (int i = 1; i < num_samples - 1; i++)
    {
        /*
         * Check whether this sample is a local maximum.
         */
        int is_local_max =
            current_filtered > previous_filtered &&
            current_filtered >= next_filtered;

        if (!is_local_max)
        {
            continue;
        }


        /*
         * The peak must be sufficiently far above the
         * detrended baseline.
         */
        if (current_filtered < adaptive_threshold)
        {
            continue;
        }


        /*
         * Prevent multiple detections within one heartbeat.
         */
        if (i - last_beat < min_beat_distance)
        {
            continue;
        }


        if (num_beats < MAX_BEATS)
        {
            beat_indices[num_beats] = i;

            /*
             * Calculate time of this heartbeat.
             */
            double time_ms =
                ((double)i / sample_rate) * 1000.0;


            /*
             * Calculate BPM based on the previous heartbeat.
             */
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
            printf("Heartbeat %2d: " "sample = %4d, ""time = %8.2f ms, ""amplitude = %8.2f, ""BPM = %7.2f\r\n",num_beats + 1,i,time_ms,current_filtered,bpm);
            num_beats++;
            last_beat = i;
        }

        if (i < num_samples - 2)
        {
            previous_filtered = current_filtered;
            current_filtered = next_filtered;
            next_filtered = FilterCursor_Next(&cursor);
        }
    }


    printf("\r\n");
    printf("Total heartbeats detected: %d\r\n", num_beats);

    return num_beats;
}
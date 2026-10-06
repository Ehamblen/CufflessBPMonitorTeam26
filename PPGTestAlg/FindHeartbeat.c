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
 *
 * Example:
 *
 *     detrended range = 2000
 *     threshold = 0.15 * 2000
 *                = 300
 *
 * This is NOT an absolute ADC threshold.
 */
#define THRESHOLD_FRACTION 0.15


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
     * ---------------------------------------------------------
     * STEP 1: Determine minimum distance between heartbeats
     * ---------------------------------------------------------
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
     * ---------------------------------------------------------
     * STEP 2: Remove the floating baseline
     * ---------------------------------------------------------
     *
     * We estimate the baseline using a moving average.
     *
     *     baseline = slow-changing component
     *
     * Then:
     *
     *     detrended = raw PPG - baseline
     *
     * This removes much of the slow DC/baseline movement.
     */
    double baseline[MAX_SAMPLES];
    double detrended[MAX_SAMPLES];

    int half_baseline_window = BASELINE_WINDOW / 2;

    for (int i = 0; i < num_samples; i++)
    {
        int start = i - half_baseline_window;
        int end   = i + half_baseline_window;

        if (start < 0)
        {
            start = 0;
        }

        if (end >= num_samples)
        {
            end = num_samples - 1;
        }

        double sum = 0.0;
        int count = 0;

        for (int j = start; j <= end; j++)
        {
            sum += ppg[j];
            count++;
        }

        baseline[i] = sum / count;

        detrended[i] = ppg[i] - baseline[i];
    }


    /*
     * ---------------------------------------------------------
     * STEP 3: Smooth the detrended signal
     * ---------------------------------------------------------
     *
     * A small moving average reduces high-frequency noise while
     * preserving the general heartbeat shape.
     */
    double filtered[MAX_SAMPLES];

    int half_smoothing_window = SMOOTHING_WINDOW / 2;

    for (int i = 0; i < num_samples; i++)
    {
        int start = i - half_smoothing_window;
        int end   = i + half_smoothing_window;

        if (start < 0)
        {
            start = 0;
        }

        if (end >= num_samples)
        {
            end = num_samples - 1;
        }

        double sum = 0.0;
        int count = 0;

        for (int j = start; j <= end; j++)
        {
            sum += detrended[j];
            count++;
        }

        filtered[i] = sum / count;
    }


    /*
     * ---------------------------------------------------------
     * STEP 4: Determine an adaptive threshold
     * ---------------------------------------------------------
     *
     * Find the range of the FILTERED signal.
     *
     * Unlike the old algorithm, this threshold is not based
     * on the absolute MAX30102 ADC value.
     *
     * Therefore, baseline movement does not directly affect it.
     */
    double signal_min = filtered[0];
    double signal_max = filtered[0];

    for (int i = 1; i < num_samples; i++)
    {
        if (filtered[i] < signal_min)
        {
            signal_min = filtered[i];
        }

        if (filtered[i] > signal_max)
        {
            signal_max = filtered[i];
        }
    }

    double signal_range = signal_max - signal_min;

    double adaptive_threshold =
        signal_range * THRESHOLD_FRACTION;


    /*
     * Print some information so we can debug the algorithm.
     */
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
     * ---------------------------------------------------------
     * STEP 5: Find local maxima
     * ---------------------------------------------------------
     */
    int num_beats = 0;

    int last_beat = -min_beat_distance;

    for (int i = 1; i < num_samples - 1; i++)
    {
        /*
         * Check whether this sample is a local maximum.
         */
        int is_local_max =
            filtered[i] > filtered[i - 1] &&
            filtered[i] >= filtered[i + 1];

        if (!is_local_max)
        {
            continue;
        }


        /*
         * The peak must be sufficiently far above the
         * detrended baseline.
         */
        if (filtered[i] < adaptive_threshold)
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


        /*
         * -----------------------------------------------------
         * Heartbeat detected
         * -----------------------------------------------------
         */
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


            /*
             * Print the heartbeat for debugging.
             */
            printf("Heartbeat %2d: " "sample = %4d, ""time = %8.2f ms, ""amplitude = %8.2f, ""BPM = %7.2f\r\n",num_beats + 1,i,time_ms,filtered[i],bpm);
            num_beats++;
            last_beat = i;
        }
    }


    /*
     * Print final result.
     */
    printf("\r\n");
    printf("Total heartbeats detected: %d\r\n", num_beats);

    return num_beats;
}
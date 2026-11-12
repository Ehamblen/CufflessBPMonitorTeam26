#include "heartbeat.h"
#include <math.h>

#define HB_MAX_SAMPLES 700
#define HB_BASELINE_WINDOW 101
#define HB_SMOOTH_WINDOW 5
#define HB_THRESHOLD_FRACTION 0.35
#define HB_MIN_BPM 40.0
#define HB_MAX_BPM 140.0
#define HB_PEAK_WINDOW 5

/* Workspace is static to avoid ~5.6 KB temporary stack allocation. */
static double hb_filtered[HB_MAX_SAMPLES];

int FindHeartbeats(const double *ppg, int n, double sample_rate,
                   int *beat_indices, int max_beats) {
    if (!ppg || !beat_indices || n < HB_BASELINE_WINDOW ||
        n > HB_MAX_SAMPLES || max_beats <= 0 ||
        !isfinite(sample_rate) || sample_rate <= 0.0) return 0;

    int i, j;
    double minimum = INFINITY, maximum = -INFINITY;

    /* Centered baseline removal followed by short moving average. */
    for (i = 0; i < n; ++i) {
        double baseline = 0.0;
        int count = 0;
        int start = i - HB_BASELINE_WINDOW / 2;
        int end = i + HB_BASELINE_WINDOW / 2;
        if (start < 0) start = 0;
        if (end >= n) end = n - 1;
        for (j = start; j <= end; ++j) {
            if (!isfinite(ppg[j])) return 0;
            baseline += ppg[j];
            ++count;
        }
        hb_filtered[i] = ppg[i] - baseline / (double)count;
    }

    /* Moving-average in-place via a small window computed from original
       high-frequency residuals would need another buffer; use a 5-sample
       delayed streaming average instead. */
    double window[HB_SMOOTH_WINDOW] = {0};
    double rolling_sum = 0.0;
    for (i = 0; i < n; ++i) {
        double current = hb_filtered[i];
        rolling_sum += current - window[i % HB_SMOOTH_WINDOW];
        window[i % HB_SMOOTH_WINDOW] = current;
        int denom = (i + 1 < HB_SMOOTH_WINDOW) ? (i + 1) : HB_SMOOTH_WINDOW;
        hb_filtered[i] = rolling_sum / denom;
        if (hb_filtered[i] < minimum) minimum = hb_filtered[i];
        if (hb_filtered[i] > maximum) maximum = hb_filtered[i];
    }
    if (!(maximum > minimum)) return 0;

    const double threshold = minimum + HB_THRESHOLD_FRACTION * (maximum - minimum);
    const int min_gap = (int)(60.0 * sample_rate / HB_MAX_BPM);
    const int max_gap = (int)(60.0 * sample_rate / HB_MIN_BPM);
    int count = 0;

    for (i = HB_PEAK_WINDOW;
         i < n - HB_PEAK_WINDOW;
         ++i)
    {
        // Candidate must exceed the adaptive threshold.
        if (hb_filtered[i] <= threshold)
            continue;

        // Check whether this sample is the maximum
        // within +/- HB_PEAK_WINDOW samples.
        int is_peak = 1;

        for (j = i - HB_PEAK_WINDOW;
             j <= i + HB_PEAK_WINDOW;
             ++j)
        {
            if (j == i)
                continue;

            if (hb_filtered[j] > hb_filtered[i] ||
                (hb_filtered[j] == hb_filtered[i] && j < i))
            {
                is_peak = 0;
                break;
            }
        }

        if (!is_peak)
            continue;

        // Existing refractory-period logic.
        if (count > 0)
        {
            int gap = i - beat_indices[count - 1];

            if (gap < min_gap)
            {
                // Retain the stronger detected peak.
                if (hb_filtered[i] >
                    hb_filtered[beat_indices[count - 1]])
                {
                    beat_indices[count - 1] = i;
                }

                continue;
            }
        }

        if (count >= max_beats)
            break;

        beat_indices[count++] = i;
    }

    return count;
}

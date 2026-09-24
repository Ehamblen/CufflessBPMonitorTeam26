#include <stdio.h>
#include <math.h>

#include "findheartbeat.h"

#define MIN_BPM 40.0
#define MAX_BPM 200.0

// Minimum amplitude required for a point to be considered
// a heartbeat peak.
//
// This works for our current synthetic PPG signal.
// Later, this should become adaptive for real sensor data.
#define MIN_PEAK_AMPLITUDE 0.5


int FindHeartbeats(
    const double ppg[],
    int num_samples,
    double sample_rate,
    int beat_indices[]
)
{
    if (ppg == NULL ||
        beat_indices == NULL ||
        num_samples < 3 ||
        sample_rate <= 0.0)
    {
        return 0;
    }

    // Minimum number of samples between beats.
    //
    // For example:
    //
    // 100 Hz / (200 BPM / 60)
    // = 30 samples
    int min_beat_distance =
        (int)(sample_rate * 60.0 / MAX_BPM);

    // STEP 1: Simple 3-point moving average

    double filtered[MAX_SAMPLES];

    if (num_samples > MAX_SAMPLES)
    {
        num_samples = MAX_SAMPLES;
    }

    filtered[0] = ppg[0];
    filtered[num_samples - 1] = ppg[num_samples - 1];

    for (int i = 1; i < num_samples - 1; i++)
    {
        filtered[i] =
            (ppg[i - 1] +
             ppg[i] +
             ppg[i + 1]) / 3.0;
    }

    // STEP 2: Find local maxima

    int num_beats = 0;

    int last_beat = -min_beat_distance;

    for (int i = 1; i < num_samples - 1; i++)
    {
        // A heartbeat must:
        //
        // 1. Be a local maximum
        // 2. Be above the amplitude threshold
        // 3. Be far enough from the previous heartbeat

        int is_peak =
            filtered[i] > filtered[i - 1] &&
            filtered[i] > filtered[i + 1] &&
            filtered[i] > MIN_PEAK_AMPLITUDE;

        if (!is_peak)
        {
            continue;
        }

        if (i - last_beat < min_beat_distance)
        {
            continue;
        }

        if (num_beats < MAX_BEATS)
        {
            beat_indices[num_beats] = i;
            num_beats++;

            last_beat = i;
        }
    }

    return num_beats;
}

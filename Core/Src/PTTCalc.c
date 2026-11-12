#include "PTTCalc.h"
#include "heartbeat.h"

#define PTT_MAX_BEATS 32

double CalculateAveragePTT(const double wrist_ppg[], int wrist_samples,
                           const double finger_ppg[], int finger_samples)
{
    int wrist_beats[PTT_MAX_BEATS];
    int finger_beats[PTT_MAX_BEATS];
    double ptt_values[PTT_MAX_BEATS];
    double average = -1.0;

    if (!wrist_ppg || !finger_ppg || wrist_samples <= 0 || finger_samples <= 0)
        return -1.0;

    /* The streams are independent: do not truncate both to the shorter one. */
    int nw = FindHeartbeats(wrist_ppg, wrist_samples, PTT_SAMPLE_RATE_HZ,
                            wrist_beats, PTT_MAX_BEATS);
    int nf = FindHeartbeats(finger_ppg, finger_samples, PTT_SAMPLE_RATE_HZ,
                            finger_beats, PTT_MAX_BEATS);
    if (!CalculatePTTFromBeats(wrist_beats, nw, finger_beats, nf,
                               PTT_SAMPLE_RATE_HZ, ptt_values,
                               PTT_MAX_BEATS, &average))
        return -1.0;
    return average;
}

int CalculatePTTFromBeats(const int wrist_beats[], int num_wrist_beats,
                          const int finger_beats[], int num_finger_beats,
                          double sample_rate_hz, double ptt_values_ms[],
                          int max_values, double *average_ptt_ms)
{
    if (average_ptt_ms) *average_ptt_ms = -1.0;
    if (!wrist_beats || !finger_beats || !ptt_values_ms || !average_ptt_ms ||
        num_wrist_beats <= 0 || num_finger_beats <= 0 || max_values <= 0 ||
        sample_rate_hz <= 0.0)
        return 0;

    int pairs = num_wrist_beats;
    if (num_finger_beats < pairs) pairs = num_finger_beats;
    if (max_values < pairs) pairs = max_values;

    double sum_ms = 0.0;
    for (int i = 0; i < pairs; ++i) {
        /* abs without stdlib: sample indices are small, nonnegative ints. */
        int difference = finger_beats[i] - wrist_beats[i];
        if (difference < 0) difference = -difference;
        ptt_values_ms[i] = (1000.0 * (double)difference) / sample_rate_hz;
        sum_ms += ptt_values_ms[i];
    }
    *average_ptt_ms = sum_ms / (double)pairs;
    return pairs;
}

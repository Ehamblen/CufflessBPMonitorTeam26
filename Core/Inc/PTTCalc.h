#ifndef PTT_CALC_H
#define PTT_CALC_H

/* Temporary, index-based PTT estimation at the configured nominal sample rate.
 * This does not use HAL timestamps while timestamp capture is being repaired.
 */
#define PTT_SAMPLE_RATE_HZ 100.0

/* Pair detected peaks by their ordinal position, up to the smaller beat count.
 * Each PTT is the ABSOLUTE index difference converted to milliseconds.
 * Returns number of PTT pairs, or 0 on invalid input. A missed/extra beat
 * can shift ordinal pairing, so this is for pipeline testing only.
 */
int CalculatePTTFromBeats(const int wrist_beats[], int num_wrist_beats,
                          const int finger_beats[], int num_finger_beats,
                          double sample_rate_hz, double ptt_values_ms[],
                          int max_values, double *average_ptt_ms);

/* Legacy waveform interface retained for desktop/other caller compatibility;
 * calls the STM32 heartbeat detector directly (no CSV/file I/O).
 */
double CalculateAveragePTT(const double wrist_ppg[], int wrist_samples,
                           const double finger_ppg[], int finger_samples);

#endif

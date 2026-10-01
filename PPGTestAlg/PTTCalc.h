#ifndef PTT_CALC_H
#define PTT_CALC_H

#define SAMPLE_RATE 100.0
#define MAX_SAMPLES 10000

/*
 * Reads a PPG CSV file.
 *
 * Returns the number of samples successfully read.
 */
int ReadPPG(
    const char *filename,
    double ppg[],
    int max_samples
);

/*
 * Calculates the average Pulse Transit Time (PTT)
 * between wrist and finger PPG signals.
 *
 * PTT is returned in milliseconds.
 *
 * Returns:
 *   >= 0 : average PTT in milliseconds
 *   -1   : error
 */
double CalculateAveragePTT(
    const double wrist_ppg[],
    int wrist_samples,
    const double finger_ppg[],
    int finger_samples
);

#endif
#include <stdio.h>
#include <stdlib.h>

#include "findheartbeat.h"

int main(void)
{
    double ppg[MAX_SAMPLES];
    int beat_indices[MAX_BEATS];

    FILE *file = fopen("PPGSample.csv", "r");

    if (file == NULL)
    {
        printf("Error: Could not open PPGSample.csv\n");
        return 1;
    }

    char line[256];
    fgets(line, sizeof(line), file);

    int num_samples = 0;

    while (fgets(line, sizeof(line), file) != NULL &&
           num_samples < MAX_SAMPLES)
    {
        double time;
        double red_ppg;
        double ir_ppg;

        if (sscanf(line, "%lf,%lf,%lf", &time, &red_ppg, &ir_ppg) == 3)
        {
            ppg[num_samples] = ir_ppg;
            num_samples++;
        }
    }

    fclose(file);

    printf("Loaded %d PPG samples.\n", num_samples);

    /*
     * Run heartbeat detector.
     *
     * Your MAX30102 data is currently sampled at 100 Hz.
     */
    int num_beats = FindHeartbeats(
        ppg,
        num_samples,
        100.0,
        beat_indices
    );

    printf("\nDetected %d heartbeats.\n", num_beats);

    return 0;
}
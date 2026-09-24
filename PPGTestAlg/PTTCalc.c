#include <stdio.h>
#include <stdlib.h>

#include "findheartbeat.h"

#define SAMPLE_RATE 100.0

int ReadPPG(
    const char *filename,
    double ppg[],
    int max_samples
)
{
    FILE *file = fopen(filename, "r");

    if (file == NULL)
    {
        printf("Error: Could not open %s\n", filename);
        return 0;
    }

    char line[256];

    int num_samples = 0;

    while (fgets(line, sizeof(line), file) != NULL)
    {
        double time;
        double value;

        if (line[0] == '\n' ||
            line[0] == '\r')
        {
            continue;
        }

        if (sscanf(line, "%lf,%lf", &time, &value) == 2)
        {
            if (num_samples < max_samples)
            {
                ppg[num_samples] = value;
                num_samples++;
            }
        }
    }

    fclose(file);

    return num_samples;
}


int main(void)
{
    // STEP 1: Load both PPG signals

    double wrist_ppg[MAX_SAMPLES];
    double finger_ppg[MAX_SAMPLES];

    int wrist_samples =
        ReadPPG(
            "wrist_ppg.csv",
            wrist_ppg,
            MAX_SAMPLES
        );

    int finger_samples =
        ReadPPG(
            "finger_ppg.csv",
            finger_ppg,
            MAX_SAMPLES
        );

    printf("Wrist samples:  %d\n", wrist_samples);
    printf("Finger samples: %d\n", finger_samples);

    if (wrist_samples == 0 ||
        finger_samples == 0)
    {
        printf("Error: Could not read PPG data.\n");
        return 1;
    }

    // STEP 2: Use the shorter signal length

    int num_samples = wrist_samples;

    if (finger_samples < num_samples)
    {
        num_samples = finger_samples;
    }

    // STEP 3: Detect heartbeats in both signals

    int wrist_beats[MAX_BEATS];
    int finger_beats[MAX_BEATS];

    int num_wrist_beats =
        FindHeartbeats(
            wrist_ppg,
            num_samples,
            SAMPLE_RATE,
            wrist_beats
        );

    int num_finger_beats =
        FindHeartbeats(
            finger_ppg,
            num_samples,
            SAMPLE_RATE,
            finger_beats
        );

    printf("\n");
    printf("Wrist beats detected:  %d\n", num_wrist_beats);
    printf("Finger beats detected: %d\n", num_finger_beats);

    // STEP 4: Print detected beats

    printf("\nWrist heartbeat samples:\n");

    for (int i = 0; i < num_wrist_beats; i++)
    {
        printf(
            "Beat %3d: sample %4d, time %.3f s\n",
            i + 1,
            wrist_beats[i],
            wrist_beats[i] / SAMPLE_RATE
        );
    }

    printf("\nFinger heartbeat samples:\n");

    for (int i = 0; i < num_finger_beats; i++)
    {
        printf(
            "Beat %3d: sample %4d, time %.3f s\n",
            i + 1,
            finger_beats[i],
            finger_beats[i] / SAMPLE_RATE
        );
    }

    // STEP 5: Match corresponding heartbeats
    //
    // For our synthetic signal, beat 1 on the wrist corresponds
    // to beat 1 on the finger, beat 2 to beat 2, etc.
    //
    // We can initially match beats by their index.
    //
    // Later, for real data, this should be made more robust.

    int num_pairs = num_wrist_beats;

    if (num_finger_beats < num_pairs)
    {
        num_pairs = num_finger_beats;
    }

    if (num_pairs == 0)
    {
        printf("\nNo corresponding heartbeats found.\n");
        return 1;
    }

    // STEP 6: Calculate PTT

    printf("\nPTT measurements:\n\n");

    double total_ptt = 0.0;

    for (int i = 0; i < num_pairs; i++)
    {
        int wrist_sample =
            wrist_beats[i];

        int finger_sample =
            finger_beats[i];

        // Number of samples between the two detected beats.
        int sample_difference =
            finger_sample - wrist_sample;

        // Convert samples to seconds.
        double ptt_seconds =
            sample_difference / SAMPLE_RATE;

        // Convert seconds to milliseconds.
        double ptt_ms =
            ptt_seconds * 1000.0;

        printf(
            "Beat %3d: wrist = %4d, finger = %4d, "
            "PTT = %.1f ms\n",
            i + 1,
            wrist_sample,
            finger_sample,
            ptt_ms
        );

        total_ptt += ptt_ms;
    }

    // STEP 7: Calculate average PTT

    double average_ptt =
        total_ptt / num_pairs;

    printf("\n");
    printf("-----------------------------\n");
    printf(
        "Average PTT: %.1f ms\n",
        average_ptt
    );
    printf(
        "Measurements: %d\n",
        num_pairs
    );
    printf("-----------------------------\n");

    return 0;
}

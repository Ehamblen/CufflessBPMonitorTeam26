#include <stdio.h>
#include <stdlib.h>

#include "PTTCalc.h"
#include "findheartbeat.h"


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


double CalculateAveragePTT(
    const double wrist_ppg[],
    int wrist_samples,
    const double finger_ppg[],
    int finger_samples
)
{

    int num_samples = wrist_samples;

    if (finger_samples < num_samples)
    {
        num_samples = finger_samples;
    }

    if (num_samples <= 0)
    {
        printf("Error: No PPG samples available.\n");
        return -1.0;
    }


    // Detect heartbeats in both signals.
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


    printf("Wrist beats detected:  %d\n", num_wrist_beats);
    printf("Finger beats detected: %d\n", num_finger_beats);


    // Match corresponding heartbeats.
    //
    // For the current synthetic test data,
    // beat 1 corresponds to beat 1,
    // beat 2 corresponds to beat 2, etc.
    int num_pairs = num_wrist_beats;

    if (num_finger_beats < num_pairs)
    {
        num_pairs = num_finger_beats;
    }

    if (num_pairs == 0)
    {
        printf("Error: No corresponding heartbeats found.\n");
        return -1.0;
    }


    // Calculate PTT for each heartbeat.
    double total_ptt = 0.0;

    printf("\nPTT measurements:\n");

    for (int i = 0; i < num_pairs; i++)
    {
        int wrist_sample =
            wrist_beats[i];

        int finger_sample =
            finger_beats[i];

        // Calculate difference in samples.
        int sample_difference =
            finger_sample - wrist_sample;

        // Convert samples to seconds.
        double ptt_seconds =
            sample_difference / SAMPLE_RATE;

        // Convert seconds to milliseconds.
        double ptt_ms =
            ptt_seconds * 1000.0;


        // Ignore negative PTT values.
        //
        // The finger signal should occur after the
        // wrist signal for our current sensor arrangement.
        if (ptt_ms > 0.0)
        {
            printf(
                "Beat %3d: wrist = %4d, "
                "finger = %4d, PTT = %.1f ms\n",
                i + 1,
                wrist_sample,
                finger_sample,
                ptt_ms
            );

            total_ptt += ptt_ms;
        }
    }


    // Calculate average PTT.
    //
    // For now, num_pairs is used because our synthetic
    // signals should produce valid positive PTT values.
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


    return average_ptt;
}

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#define SAMPLE_RATE 100.0

#define MIN_BPM 40.0
#define MAX_BPM 200.0

#define MIN_BEAT_DISTANCE ((int)(SAMPLE_RATE * 60.0 / MAX_BPM))

#define MAX_SAMPLES 10000
#define MAX_BEATS 1000

int main(void)
{
    FILE *file = fopen("ppg_data.csv", "r");

    if (file == NULL)
    {
        printf("Error: Could not open ppg_data.csv\n");
        return 1;
    }

    double ppg[MAX_SAMPLES];
    int num_samples = 0;

    char line[256];

    /*
     * Read the CSV one line at a time.
     */
    while (fgets(line, sizeof(line), file) != NULL)
    {
        double time;
        double value;

        /*
         * Skip blank lines.
         */
        if (line[0] == '\n' || line[0] == '\r')
        {
            continue;
        }

        /*
         * Try to parse:
         *
         * Time,PPG
         *
         * or
         *
         * 0.0000,0.136926
         */
        if (sscanf(line, "%lf,%lf", &time, &value) == 2)
        {
            if (num_samples < MAX_SAMPLES)
            {
                ppg[num_samples] = value;
                num_samples++;
            }
        }
        else
        {
            /*
             * This will catch the header.
             */
            printf("Skipping line: %s", line);
        }
    }

    fclose(file);

    printf("\nRead %d samples\n", num_samples);

    if (num_samples < 3)
    {
        printf("Not enough samples for peak detection.\n");
        return 1;
    }

    /*
     * ---------------------------------------------------------
     * STEP 1: Simple 3-point moving-average filter
     * ---------------------------------------------------------
     */

    double filtered[MAX_SAMPLES];

    filtered[0] = ppg[0];
    filtered[num_samples - 1] = ppg[num_samples - 1];

    for (int i = 1; i < num_samples - 1; i++)
    {
        filtered[i] =
            (ppg[i - 1] +
             ppg[i] +
             ppg[i + 1]) / 3.0;
    }

    /*
     * ---------------------------------------------------------
     * STEP 2: Find local maxima
     * ---------------------------------------------------------
     */

    int beat_indices[MAX_BEATS];
    int num_beats = 0;

    int last_beat = -MIN_BEAT_DISTANCE;

    for (int i = 1; i < num_samples - 1; i++)
    {
        /*
         * Local maximum:
         *
         *       X
         *      / \
         *     /   \
         * ---/-----\---
         *
         * filtered[i] must be larger than its neighbors.
         */
        int is_peak =
            filtered[i] > filtered[i - 1] &&
            filtered[i] > filtered[i + 1];

        if (!is_peak)
        {
            continue;
        }

        /*
         * Make sure this peak is far enough away from
         * the previous heartbeat.
         */
        if (i - last_beat < MIN_BEAT_DISTANCE)
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

    /*
     * ---------------------------------------------------------
     * STEP 3: Print detected beats
     * ---------------------------------------------------------
     */

    printf("\nDetected %d beats:\n\n", num_beats);

    for (int i = 0; i < num_beats; i++)
    {
        double time =
            beat_indices[i] / SAMPLE_RATE;

        printf(
            "Beat %3d: sample %4d, time %.2f s, amplitude %.3f\n",
            i + 1,
            beat_indices[i],
            time,
            filtered[beat_indices[i]]
        );
    }

    /*
     * ---------------------------------------------------------
     * STEP 4: Calculate beat-to-beat intervals
     * ---------------------------------------------------------
     */

    if (num_beats < 2)
    {
        printf("\nNot enough beats to calculate heart rate.\n");
        return 0;
    }

    printf("\nBeat intervals:\n\n");

    double total_interval = 0.0;
    int valid_intervals = 0;

    for (int i = 1; i < num_beats; i++)
    {
        int sample_difference =
            beat_indices[i] -
            beat_indices[i - 1];

        double interval =
            sample_difference / SAMPLE_RATE;

        double bpm =
            60.0 / interval;

        if (bpm < MIN_BPM || bpm > MAX_BPM)
        {
            printf(
                "Interval %d: %.3f s (%.1f BPM) [REJECTED]\n",
                i,
                interval,
                bpm
            );

            continue;
        }

        printf(
            "Interval %d: %.3f s (%.1f BPM)\n",
            i,
            interval,
            bpm
        );

        total_interval += interval;
        valid_intervals++;
    }

    /*
     * ---------------------------------------------------------
     * STEP 5: Calculate average heart rate
     * ---------------------------------------------------------
     */

    if (valid_intervals > 0)
    {
        double average_interval =
            total_interval / valid_intervals;

        double average_bpm =
            60.0 / average_interval;

        printf("\n-----------------------------\n");
        printf(
            "Average heart rate: %.1f BPM\n",
            average_bpm
        );
        printf(
            "Valid intervals: %d\n",
            valid_intervals
        );
        printf("-----------------------------\n");
    }
    else
    {
        printf("\nNo valid beat intervals found.\n");
    }

    return 0;
}

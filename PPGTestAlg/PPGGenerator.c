
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define SAMPLE_RATE 100.0       // Hz
#define DURATION 10.0           // seconds
#define HEART_RATE 75.0         // beats per minute
#define NOISE_AMPLITUDE 0.02    // noise level

// Simulated pulse transit time.
// At 100 Hz, 5 samples = 50 ms.
#define PTT_DELAY_SAMPLES 5

#define PI 3.14159265358979323846

// Generate a synthetic PPG pulse.
//
// t = time within the heartbeat, in seconds
//
// Returns a normalized PPG value roughly between 0 and 1.
double generate_ppg_pulse(double t)
{
    double pulse_width = 0.40;

    if (t < 0.0 || t > pulse_width)
        return 0.0;

    // Main systolic peak.
    double main_center = 0.065;
    double main_sigma = 0.032;

    double main_pulse = exp(
        -((t - main_center) * (t - main_center)) /
        (2.0 * main_sigma * main_sigma)
    );

    // Dicrotic notch.
    double notch_center = 0.135;
    double notch_sigma = 0.018;

    double notch = 0.14 * exp(
        -((t - notch_center) * (t - notch_center)) /
        (2.0 * notch_sigma * notch_sigma)
    );

    // Secondary/reflected wave.
    double secondary_center = 0.175;
    double secondary_sigma = 0.030;

    double secondary = 0.30 * exp(
        -((t - secondary_center) * (t - secondary_center)) /
        (2.0 * secondary_sigma * secondary_sigma)
    );

    // Combine components.
    double pulse = main_pulse - notch + secondary;

    if (pulse < 0.0)
        pulse = 0.0;

    return pulse;
}

// Generate a random value between -1 and +1.
double random_noise(void)
{
    return ((double)rand() / RAND_MAX) * 2.0 - 1.0;
}

int main(void)
{
    srand((unsigned int)time(NULL));

    double beat_period = 60.0 / HEART_RATE;
    int total_samples = (int)(SAMPLE_RATE * DURATION);

    FILE *wrist_file = fopen("wrist_ppg.csv", "w");
    FILE *finger_file = fopen("finger_ppg.csv", "w");

    if (wrist_file == NULL || finger_file == NULL)
    {
        printf("Error: Could not create output files.\n");
        return 1;
    }

    fprintf(wrist_file, "Time,PPG\n");
    fprintf(finger_file, "Time,PPG\n");

    // Generate the wrist signal.

    double wrist_ppg[total_samples];

    for (int i = 0; i < total_samples; i++)
    {
        double time = i / SAMPLE_RATE;

        double beat_time = fmod(time, beat_period);

        double ppg = generate_ppg_pulse(beat_time);

        // Add noise independently to the wrist signal.
        ppg += NOISE_AMPLITUDE * random_noise();

        if (ppg < 0.0)
            ppg = 0.0;

        wrist_ppg[i] = ppg;

        fprintf(
            wrist_file,
            "%.4f,%.6f\n",
            time,
            ppg
        );
    }

    // Generate the finger signal.
    //
    // The finger signal is the same waveform delayed by
    // PTT_DELAY_SAMPLES.

    for (int i = 0; i < total_samples; i++)
    {
        double time = i / SAMPLE_RATE;

        int source_index = i - PTT_DELAY_SAMPLES;

        double ppg;

        if (source_index >= 0)
        {
            // Copy the corresponding wrist waveform sample.
            ppg = wrist_ppg[source_index];
        }
        else
        {
            // No pulse has arrived yet.
            ppg = 0.0;
        }

        // Add independent sensor noise to the finger signal.
        ppg += NOISE_AMPLITUDE * random_noise();

        if (ppg < 0.0)
            ppg = 0.0;

        fprintf(
            finger_file,
            "%.4f,%.6f\n",
            time,
            ppg
        );
    }

    fclose(wrist_file);
    fclose(finger_file);

    printf("Generated %d samples.\n", total_samples);
    printf("PTT delay: %d samples (%.1f ms)\n",
           PTT_DELAY_SAMPLES,
           (PTT_DELAY_SAMPLES / SAMPLE_RATE) * 1000.0);

    printf("Wrist signal:  wrist_ppg.csv\n");
    printf("Finger signal: finger_ppg.csv\n");

    return 0;
}

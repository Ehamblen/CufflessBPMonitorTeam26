#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define SAMPLE_RATE 100.0       // Hz
#define DURATION 10.0           // seconds
#define HEART_RATE 75.0         // beats per minute
#define NOISE_AMPLITUDE 0.02    // noise level

#define PI 3.14159265358979323846

/*
 * Generate a synthetic PPG pulse.
 *
 * t = time within the heartbeat, in seconds
 *
 * Returns a normalized PPG value roughly between 0 and 1.
 */
double generate_ppg_pulse(double t)
{
    double pulse_width = 0.35;

    if (t < 0.0 || t > pulse_width)
        return 0.0;

    /*
     * Gaussian-like pulse.
     *
     * The main peak occurs near the beginning of the pulse,
     * followed by a gradual decay.
     */
    double center = 0.08;
    double sigma = 0.055;

    double pulse = exp(
        -((t - center) * (t - center)) /
        (2.0 * sigma * sigma)
    );

    /*
     * Add a smaller secondary feature representing
     * the dicrotic notch / reflected wave.
     */
    double secondary_center = 0.20;
    double secondary_sigma = 0.025;

    double secondary = 0.20 * exp(
        -((t - secondary_center) * (t - secondary_center)) /
        (2.0 * secondary_sigma * secondary_sigma)
    );

    return pulse + secondary;
}

/*
 * Generate small random noise between -1 and +1.
 */
double random_noise(void)
{
    return ((double)rand() / RAND_MAX) * 2.0 - 1.0;
}

int main(void)
{
    srand((unsigned int)time(NULL));

    double beat_period = 60.0 / HEART_RATE;
    int total_samples = (int)(SAMPLE_RATE * DURATION);

    printf("Time,PPG\n");

    for (int i = 0; i < total_samples; i++)
    {
        /*
         * Current sample time.
         */
        double time = i / SAMPLE_RATE;

        /*
         * Position within the current heartbeat.
         */
        double beat_time = fmod(time, beat_period);

        /*
         * Generate the PPG pulse.
         */
        double ppg = generate_ppg_pulse(beat_time);

        /*
         * Add a small amount of random noise.
         */
        ppg += NOISE_AMPLITUDE * random_noise();

        /*
         * Prevent negative values.
         */
        if (ppg < 0.0)
            ppg = 0.0;

        printf("%.4f,%.6f\n", time, ppg);
    }

    return 0;
}
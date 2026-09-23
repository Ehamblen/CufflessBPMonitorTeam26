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
    double pulse_width = 0.40;

    if (t < 0.0 || t > pulse_width)
        return 0.0;

    /*
     * Main systolic peak.
     *
     * Narrower than the original so that the pulse has
     * a sharper rise and leaves room for the reflected wave.
     */
    double main_center = 0.065;
    double main_sigma = 0.032;

    double main_pulse = exp(
        -((t - main_center) * (t - main_center)) /
        (2.0 * main_sigma * main_sigma)
    );

    /*
     * Dicrotic notch.
     *
     * A small negative Gaussian creates a dip in the
     * descending portion of the waveform.
     */
    double notch_center = 0.135;
    double notch_sigma = 0.018;

    double notch = 0.14 * exp(
        -((t - notch_center) * (t - notch_center)) /
        (2.0 * notch_sigma * notch_sigma)
    );

    double secondary_center = 0.175;
    double secondary_sigma = 0.030;

    double secondary = 0.30 * exp(
        -((t - secondary_center) * (t - secondary_center)) /
        (2.0 * secondary_sigma * secondary_sigma)
    );

    /*
     * Combine the components.
     */
    double pulse = main_pulse - notch + secondary;

    /*
     * Prevent the notch from producing negative values.
     */
    if (pulse < 0.0)
        pulse = 0.0;

    return pulse;
}

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
        
         // Current sample time.
         
        double time = i / SAMPLE_RATE;

        
         //Position within the current heartbeat.
         
        double beat_time = fmod(time, beat_period);

        
         //Generate the PPG pulse.
         
        double ppg = generate_ppg_pulse(beat_time);

        
         //Add a small amount of random noise.
         
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
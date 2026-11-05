#include <stdio.h>

#include "PTTCalc.h"
#include "BPEstimation.h"


// Linear regression coefficients.
//
// BP = a * PTT + b
#define SBP_SLOPE      -0.50
#define SBP_INTERCEPT  160.0

#define DBP_SLOPE      -0.30
#define DBP_INTERCEPT  105.0


// Estimate systolic blood pressure from PTT.
double EstimateSBP(double ptt_ms)
{
    return (SBP_SLOPE * ptt_ms) +
           SBP_INTERCEPT;
}


// Estimate diastolic blood pressure from PTT.
double EstimateDBP(double ptt_ms)
{
    return (DBP_SLOPE * ptt_ms) +
           DBP_INTERCEPT;
}


int main(void)
{
    // Load PPG data.
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


    // Calculate average PTT.
    double average_ptt =
        CalculateAveragePTT(
            wrist_ppg,
            wrist_samples,
            finger_ppg,
            finger_samples
        );


    if (average_ptt < 0.0)
    {
        printf("Error: PTT calculation failed.\n");
        return 1;
    }


    // Estimate blood pressure.
    double systolic_bp =
        EstimateSBP(average_ptt);

    double diastolic_bp =
        EstimateDBP(average_ptt);


    // Display results.
    printf("\n");
    printf("=============================\n");
    printf("Blood Pressure Estimation\n");
    printf("=============================\n");

    printf(
        "Average PTT: %.1f ms\n",
        average_ptt
    );

    printf(
        "Estimated SBP: %.1f mmHg\n",
        systolic_bp
    );

    printf(
        "Estimated DBP: %.1f mmHg\n",
        diastolic_bp
    );

    printf("=============================\n");


    return 0;
}
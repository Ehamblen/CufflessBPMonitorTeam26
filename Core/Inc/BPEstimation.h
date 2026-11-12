#ifndef BP_ESTIMATION_H
#define BP_ESTIMATION_H

/* PROVISIONAL two-point model placeholder: assumes reference BP of 120/80
 * mmHg at a reference PTT of 80 ms, NOT patient-specific calibration.
 * Do not use these outputs for clinical decisions.
 */
double EstimateSBP(double ptt_ms);
double EstimateDBP(double ptt_ms);

#endif

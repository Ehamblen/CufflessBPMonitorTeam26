#ifndef BP_ESTIMATION_H
#define BP_ESTIMATION_H

// Estimate systolic blood pressure from PTT.
//
// PTT is provided in milliseconds.
// Returns estimated SBP in mmHg.
double EstimateSBP(double ptt_ms);


// Estimate diastolic blood pressure from PTT.
//
// PTT is provided in milliseconds.
// Returns estimated DBP in mmHg.
double EstimateDBP(double ptt_ms);

#endif
#include "BPEstimation.h"

/* Placeholder anchor and slopes; replace with cuff-derived calibration. */
#define REFERENCE_PTT_MS 80.0
#define REFERENCE_SBP_MMHG 120.0
#define REFERENCE_DBP_MMHG 80.0
#define SBP_SLOPE (-0.50)
#define DBP_SLOPE (-0.30)

double EstimateSBP(double ptt_ms)
{
    return REFERENCE_SBP_MMHG + SBP_SLOPE * (ptt_ms - REFERENCE_PTT_MS);
}

double EstimateDBP(double ptt_ms)
{
    return REFERENCE_DBP_MMHG + DBP_SLOPE * (ptt_ms - REFERENCE_PTT_MS);
}

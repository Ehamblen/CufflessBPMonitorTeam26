#ifndef FINDHEARTBEAT_H
#define FINDHEARTBEAT_H

#define MAX_SAMPLES 10000
#define MAX_BEATS 1000

int FindHeartbeats(
    const double ppg[],
    int num_samples,
    double sample_rate,
    int beat_indices[]
);

#endif

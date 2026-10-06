#ifndef FINDHEARTBEAT_H
#define FINDHEARTBEAT_H

#define MAX_SAMPLES 5000
#define MAX_BEATS 100

int FindHeartbeats(
    const double ppg[],
    int num_samples,
    double sample_rate,
    int beat_indices[]
);

#endif
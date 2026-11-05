#ifndef HEARTBEAT_H
#define HEARTBEAT_H

/*
 * Analyze uniformly sampled PPG samples and return beat peak indices.
 * Sample rate is in samples per second; output is capped at max_beats.
 * Caller retains ownership of all buffers.
 */
int FindHeartbeats(const double *ppg, int num_samples, double sample_rate,
                   int *beat_indices, int max_beats);

#endif /* HEARTBEAT_H */

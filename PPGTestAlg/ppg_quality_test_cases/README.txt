PPG SignalQuality synthetic test set

Each waveform CSV columns: sample_index, timestamp_ms, ppg_raw
The matching *_beats.txt contains comma-separated, 0-based beat_indices.
Use sample_rate=100.0 Hz for all cases, unless manifest says otherwise.
Expected statuses reflect the EXACT posted CheckSignalQuality() function and its check order.

To test INVALID_INPUT (requires function arguments, not CSV), call:
  CheckSignalQuality(NULL, 1000, 100.0, beats, nbeats);
  CheckSignalQuality(ppg, 1000, 0.0, beats, nbeats);
  CheckSignalQuality(ppg, 1000, 100.0, NULL, nbeats);

C parsing: use strtod() or equivalent for ppg_raw; it accepts "nan" and "inf".
Do not parse nonfinite cells as empty/zero.

IMPORTANT: beat indices in this pack are intentionally PROVIDED, not the output of a
heartbeat detector. To unit-test CheckSignalQuality, pass them directly. If you
re-run FindHeartbeats() on these waveforms, results and expected status may differ.
A noisy signal or absurdly small amplitude can pass this quality implementation
when supplied beat indices are valid. No signal-to-noise or pulse morphology
check exists yet. This is intentional for testing.
These are artificial waveforms, NOT validated physiological data and NOT adequate
to qualify a BP estimation device.

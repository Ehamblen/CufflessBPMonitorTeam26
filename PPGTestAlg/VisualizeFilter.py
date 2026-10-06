import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Change this to the name of your actual CSV file.
CSV_PATH = "PPGSample.csv"

# Choose which MAX30102 channel to inspect:
SIGNAL_COLUMN = "IR PPG"  # Or "Red PPG"

# Match the initial C detector settings.
BASELINE_WINDOW = 101
SMOOTHING_WINDOW = 5
THRESHOLD_FRACTION = 0.15
MAX_BPM = 200.0


# ---------------------------------------------------------
# STEP 1: Load the CSV
# ---------------------------------------------------------

df = pd.read_csv(CSV_PATH, skip_blank_lines=True)

# Removes the extra space in "Red PPG ".
df.columns = df.columns.str.strip()

required_columns = ["Timestamp (ms)", "Red PPG", "IR PPG"]



df = df[required_columns].apply(pd.to_numeric, errors="coerce")
df = df.dropna().reset_index(drop=True)


time_ms = df["Timestamp (ms)"].to_numpy(dtype=float)
raw = df[SIGNAL_COLUMN].to_numpy(dtype=float)

# Estimate sample rate from actual timestamps.
time_differences = np.diff(time_ms)
positive_differences = time_differences[time_differences > 0]



sample_rate = 1000.0 / np.median(positive_differences)

print(f"Channel: {SIGNAL_COLUMN}")
print(f"Samples loaded: {len(raw)}")
print(f"Estimated sample rate: {sample_rate:.2f} Hz")
print(f"Duration: {(time_ms[-1] - time_ms[0]) / 1000:.2f} seconds")



raw_series = pd.Series(raw)

baseline = raw_series.rolling(window=BASELINE_WINDOW,center=True,min_periods=1).mean().to_numpy()

# AC component: raw PPG with estimated baseline removed.
detrended = raw - baseline


# ---------------------------------------------------------
# STEP 3: Smooth the detrended signal
# ---------------------------------------------------------

filtered = pd.Series(detrended).rolling(
    window=SMOOTHING_WINDOW,
    center=True,
    min_periods=1
).mean().to_numpy()


# ---------------------------------------------------------
# STEP 4: Reproduce the current C detector for visualization
# ---------------------------------------------------------

signal_min = np.min(filtered)
signal_max = np.max(filtered)

adaptive_threshold = (
    signal_max - signal_min
) * THRESHOLD_FRACTION

min_beat_distance = int(sample_rate * 60.0 / MAX_BPM)

beat_indices = []
last_beat = -min_beat_distance

for i in range(1, len(filtered) - 1):

    is_local_max = (
        filtered[i] > filtered[i - 1]
        and filtered[i] >= filtered[i + 1]
    )

    if not is_local_max:
        continue

    if filtered[i] < adaptive_threshold:
        continue

    if i - last_beat < min_beat_distance:
        continue

    beat_indices.append(i)
    last_beat = i

beat_indices = np.array(beat_indices, dtype=int)

print(f"Adaptive threshold: {adaptive_threshold:.2f}")
print(f"Minimum beat spacing: {min_beat_distance} samples")
print(f"Detected peaks: {len(beat_indices)}")


# ---------------------------------------------------------
# STEP 5: Plot every processing stage
# ---------------------------------------------------------

fig, axes = plt.subplots(
    4, 1,
    figsize=(15, 12),
    sharex=True
)

# Plot 1: Raw sensor readings.
axes[0].plot(time_ms, raw, linewidth=0.9)
axes[0].set_title(f"1. Raw {SIGNAL_COLUMN}")
axes[0].set_ylabel("ADC amplitude")
axes[0].grid(True, alpha=0.3)

# Plot 2: Estimated baseline.
axes[1].plot(time_ms, raw, linewidth=0.7, alpha=0.5,
             label="Raw PPG")
axes[1].plot(time_ms, baseline, linewidth=1.5,
             label="Estimated baseline")
axes[1].set_title(
    f"2. Baseline estimation ({BASELINE_WINDOW} samples)"
)
axes[1].set_ylabel("ADC amplitude")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# Plot 3: Baseline-removed signal.
axes[2].plot(time_ms, detrended, linewidth=0.9)
axes[2].axhline(0, linestyle="--", linewidth=0.8)
axes[2].set_title("3. Baseline removed: raw - baseline")
axes[2].set_ylabel("Relative amplitude")
axes[2].grid(True, alpha=0.3)

# Plot 4: Smoothed signal and detected peaks.
axes[3].plot(time_ms, filtered, linewidth=1.0,
             label="Smoothed PPG")
axes[3].axhline(
    adaptive_threshold,
    linestyle="--",
    linewidth=1.0,
    label="Adaptive threshold"
)

if len(beat_indices) > 0:
    axes[3].scatter(
        time_ms[beat_indices],
        filtered[beat_indices],
        marker="x",
        s=60,
        label="Detected heartbeat"
    )

axes[3].set_title(
    f"4. Smoothed PPG and detected peaks "
    f"({len(beat_indices)} detections)"
)
axes[3].set_ylabel("Relative amplitude")
axes[3].set_xlabel("Time (ms)")
axes[3].legend()
axes[3].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
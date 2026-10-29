import pandas as pd
import matplotlib.pyplot as plt

# ==========================================
# Read CSV
# ==========================================

finger_data = []
wrist_data = []

with open("Data.csv", "r", encoding="utf-8-sig") as file:
    for line in file:
        line = line.strip()

        if not line:
            continue

        values = line.split(",")

        # Finger: F,timestamp,red,ir
        if values[0] == "F" and len(values) == 4:
            try:
                finger_data.append([
                    int(values[1]),
                    int(values[2]),
                    int(values[3])
                ])
            except ValueError:
                continue

        # Wrist: W,timestamp,ir,red,led3,led4
        elif values[0] == "W" and len(values) == 6:
            try:
                wrist_data.append([
                    int(values[1]),
                    int(values[2]),
                    int(values[3]),
                    int(values[4]),
                    int(values[5])
                ])
            except ValueError:
                continue


# ==========================================
# Convert to DataFrames
# ==========================================

finger = pd.DataFrame(
    finger_data,
    columns=["timestamp", "red", "ir"]
)

wrist = pd.DataFrame(
    wrist_data,
    columns=["timestamp", "ir", "red", "led3", "led4"]
)

print(f"Finger samples: {len(finger)}")
print(f"Wrist samples: {len(wrist)}")

if finger.empty and wrist.empty:
    raise ValueError("No valid sensor samples found in Data.csv")


# ==========================================
# Convert timestamps to seconds
# ==========================================

all_timestamps = []

if not finger.empty:
    all_timestamps.append(finger["timestamp"].min())

if not wrist.empty:
    all_timestamps.append(wrist["timestamp"].min())

start_time = min(all_timestamps)

if not finger.empty:
    finger["time"] = (finger["timestamp"] - start_time) / 1000.0

if not wrist.empty:
    wrist["time"] = (wrist["timestamp"] - start_time) / 1000.0


# ==========================================
# Plot Finger PPG
# ==========================================

if not finger.empty:
    plt.figure(figsize=(12, 5))

    plt.plot(finger["time"], finger["red"], label="Finger Red")
    plt.plot(finger["time"], finger["ir"], label="Finger IR")

    plt.title("Finger PPG - MAX30102")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Raw ADC Value")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()


# ==========================================
# Plot Wrist PPG
# ==========================================

if not wrist.empty:
    plt.figure(figsize=(12, 5))

    plt.plot(wrist["time"], wrist["red"], label="Wrist Red")
    plt.plot(wrist["time"], wrist["ir"], label="Wrist IR")

    plt.title("Wrist PPG - MAX30101 / MAX32664")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Raw ADC Value")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()


# ==========================================
# Show graphs
# ==========================================

plt.show()
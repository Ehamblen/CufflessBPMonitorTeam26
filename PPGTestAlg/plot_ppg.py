from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# Get the directory containing this Python script
script_directory = Path(__file__).parent

# Explicitly locate the CSV in the same directory
csv_file = script_directory / "ppg_data.csv"

print(f"Reading: {csv_file}")
print(f"Exists: {csv_file.exists()}")
print(f"Size: {csv_file.stat().st_size} bytes")

# Read the CSV
data = pd.read_csv(csv_file, encoding="utf-8")

# Plot
plt.figure(figsize=(12, 5))
plt.plot(data["Time"], data["PPG"])

plt.xlabel("Time (seconds)")
plt.ylabel("PPG amplitude")
plt.title("Synthetic PPG Signal")
plt.grid(True)

plt.tight_layout()
plt.show()
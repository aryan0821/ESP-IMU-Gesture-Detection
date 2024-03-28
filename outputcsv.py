# Adjusted code to parse the provided data and generate spectrograms

# Since the data provided in the user's message is truncated, let's assume it's stored in a variable named `data`.
# For the sake of demonstration, let's proceed with the data parsing assuming it follows the format of the snippet above.

# Provided data
import re
import librosa
import librosa.display
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import spectrogram, detrend


data = """
[0;32mI (5086) ACCELEROMETER: Time: [33007586 ms] Accelerometer: x=-0.19, y=0.16, z=1.00[0m
[0;32mI (5146) GYROSCOPE: Time: [33007586 ms] Gyroscope: x=-13.67, y=8.73, z=43.28[0m
[0;32mI (5306) ACCELEROMETER: Time: [33007806 ms] Accelerometer: x=-0.38, y=0.20, z=0.94[0m
[0;32mI (5366) GYROSCOPE: Time: [33007806 ms] Gyroscope: x=11.70, y=-39.92, z=-14.90[0m
[0;32mI (5526) ACCELEROMETER: Time: [33008026 ms] Accelerometer: x=-0.14, y=0.19, z=0.97[0m
[0;32mI (5586) GYROSCOPE: Time: [33008026 ms] Gyroscope: x=8.00, y=1.35, z=-4.09[0m
[0;32mI (5746) ACCELEROMETER: Time: [33008246 ms] Accelerometer: x=-0.28, y=0.12, z=0.96[0m
[0;32mI (5806) GYROSCOPE: Time: [33008246 ms] Gyroscope: x=-4.19, y=-13.16, z=-37.97[0m
[0;32mI (5966) ACCELEROMETER: Time: [33008466 ms] Accelerometer: x=-0.25, y=0.08, z=1.01[0m
[0;32mI (6026) GYROSCOPE: Time: [33008466 ms] Gyroscope: x=10.73, y=2.70, z=2.85[0m
[0;32mI (6186) ACCELEROMETER: Time: [33008686 ms] Accelerometer: x=-0.25, y=0.16, z=1.01[0m
[0;32mI (6246) GYROSCOPE: Time: [33008686 ms] Gyroscope: x=41.94, y=-11.96, z=-38.57[0m
[0;32mI (6406) ACCELEROMETER: Time: [33008906 ms] Accelerometer: x=-0.26, y=0.40, z=0.96[0m
[0;32mI (6466) GYROSCOPE: Time: [33008906 ms] Gyroscope: x=-24.69, y=-18.99, z=-38.69[0m
[0;32mI (6626) ACCELEROMETER: Time: [33009126 ms] Accelerometer: x=-0.09, y=0.04, z=1.02[0m
[0;32mI (6686) GYROSCOPE: Time: [33009126 ms] Gyroscope: x=2.94, y=2.34, z=21.92[0m
[0;32mI (6846) ACCELEROMETER: Time: [33009346 ms] Accelerometer: x=-0.24, y=0.14, z=1.00[0m
[0;32mI (6906) GYROSCOPE: Time: [33009346 ms] Gyroscope: x=26.66, y=2.98, z=-1.51[0m
[0;32mI (7066) ACCELEROMETER: Time: [33009566 ms] Accelerometer: x=-0.18, y=0.36, z=1.08[0m
[0;32mI (7126) GYROSCOPE: Time: [33009566 ms] Gyroscope: x=-46.18, y=6.51, z=38.03[0m
[0;32mI (7286) ACCELEROMETER: Time: [33009786 ms] Accelerometer: x=-0.22, y=0.27, z=1.16[0m
[0;32mI (7346) GYROSCOPE: Time: [33009786 ms] Gyroscope: x=-17.88, y=1.78, z=30.76[0m
[0;32mI (7506) ACCELEROMETER: Time: [33010006 ms] Accelerometer: x=-0.08, y=0.21, z=0.97[0m
[0;32mI (7566) GYROSCOPE: Time: [33010006 ms] Gyroscope: x=-1.21, y=-11.38, z=9.63[0m
[0;32mI (7726) ACCELEROMETER: Time: [33010226 ms] Accelerometer: x=-0.09, y=0.26, z=1.06[0m
[0;32mI (7786) GYROSCOPE: Time: [33010226 ms] Gyroscope: x=10.08, y=2.95, z=12.54[0m
[0;32mI (7946) ACCELEROMETER: Time: [33010446 ms] Accelerometer: x=-0.18, y=0.13, z=1.04[0m
[0;32mI (8006) GYROSCOPE: Time: [33010446 ms] Gyroscope: x=-7.68, y=-21.21, z=1.45[0m
[0;32mI (8166) ACCELEROMETER: Time: [33010666 ms] Accelerometer: x=-0.10, y=0.09, z=1.01[0m
[0;32mI (8226) GYROSCOPE: Time: [33010666 ms] Gyroscope: x=5.85, y=-20.88, z=0.44[0m
[0;32mI (8386) ACCELEROMETER: Time: [33010886 ms] Accelerometer: x=0.01, y=0.10, z=0.99[0m
[0;32mI (8446) GYROSCOPE: Time: [33010886 ms] Gyroscope: x=0.07, y=-6.05, z=1.21[0m
[0;32mI (8606) ACCELEROMETER: Time: [33011106 ms] Accelerometer: x=0.02, y=0.11, z=1.08[0m
[0;32mI (8666) GYROSCOPE: Time: [33011106 ms] Gyroscope: x=5.07, y=-1.99, z=2.24[0m
[0;32mI (8826) ACCELEROMETER: Time: [33011326 ms] Accelerometer: x=0.13, y=0.23, z=1.11[0m
[0;32mI (8886) GYROSCOPE: Time: [33011326 ms] Gyroscope: x=-6.22, y=-18.34, z=15.92[0m
[0;32mI (9046) ACCELEROMETER: Time: [33011546 ms] Accelerometer: x=0.20, y=0.16, z=1.01[0m
[0;32mI (9106) GYROSCOPE: Time: [33011546 ms] Gyroscope: x=2.25, y=-7.34, z=-5.87[0m
[0;32mI (9266) ACCELEROMETER: Time: [33011766 ms] Accelerometer: x=0.20, y=0.14, z=1.07[0m
[0;32mI (9326) GYROSCOPE: Time: [33011766 ms] Gyroscope: x=3.31, y=-12.75, z=31.79[0m
[0;32mI (9486) ACCELEROMETER: Time: [33011986 ms] Accelerometer: x=0.18, y=0.13, z=1.07[0m
[0;32mI (9546) GYROSCOPE: Time: [33011986 ms] Gyroscope: x=13.63, y=0.63, z=18.60[0m
[0;32mI (9706) ACCELEROMETER: Time: [33012206 ms] Accelerometer: x=0.24, y=0.32, z=1.01[0m
[0;32mI (9766) GYROSCOPE: Time: [33012206 ms] Gyroscope: x=28.09, y=-8.57, z=-23.96[0m
[0;32mI (9926) ACCELEROMETER: Time: [33012426 ms] Accelerometer: x=0.29, y=0.22, z=0.90[0m
[0;32mI (9986) GYROSCOPE: Time: [33012426 ms] Gyroscope: x=-7.73, y=-6.44, z=5.37[0m
[0;32mI (10146) ACCELEROMETER: Time: [33012646 ms] Accelerometer: x=0.30, y=0.24, z=1.00[0m
[0;32mI (10206) GYROSCOPE: Time: [33012646 ms] Gyroscope: x=-6.31, y=-0.58, z=2.57[0m
[0;32mI (10366) ACCELEROMETER: Time: [33012866 ms] Accelerometer: x=0.30, y=0.19, z=0.98[0m
[0;32mI (10426) GYROSCOPE: Time: [33012866 ms] Gyroscope: x=2.79, y=0.60, z=3.23[0m
[0;32mI (10586) ACCELEROMETER: Time: [33013086 ms] Accelerometer: x=0.32, y=0.19, z=0.98[0m
[0;32mI (10646) GYROSCOPE: Time: [33013086 ms] Gyroscope: x=1.27, y=-3.39, z=-1.60[0m
[0;32mI (10806) ACCELEROMETER: Time: [33013306 ms] Accelerometer: x=0.30, y=0.20, z=1.00[0m
[0;32mI (10866) GYROSCOPE: Time: [33013306 ms] Gyroscope: x=-1.05, y=3.33, z=9.05[0m
[0;32mI (11026) ACCELEROMETER: Time: [33013526 ms] Accelerometer: x=0.29, y=0.19, z=1.00[0m
[0;32mI (11086) GYROSCOPE: Time: [33013526 ms] Gyroscope: x=-8.76, y=-5.58, z=-1.25[0m
[0;32mI (11246) ACCELEROMETER: Time: [33013746 ms] Accelerometer: x=0.32, y=0.23, z=1.02[0m
[0;32mI (11306) GYROSCOPE: Time: [33013746 ms] Gyroscope: x=-4.10, y=-4.40, z=-0.44[0m
[0;32mI (11466) ACCELEROMETER: Time: [33013966 ms] Accelerometer: x=0.33, y=0.12, z=0.95[0m
[0;32mI (11526) GYROSCOPE: Time: [33013966 ms] Gyroscope: x=0.58, y=-0.34, z=-3.43[0m
[0;32mI (11686) ACCELEROMETER: Time: [33014186 ms] Accelerometer: x=0.39, y=0.25, z=0.98[0m
[0;32mI (11746) GYROSCOPE: Time: [33014186 ms] Gyroscope: x=1.80, y=-3.28, z=-2.83[0m
[0;32mI (11906) ACCELEROMETER: Time: [33014406 ms] Accelerometer: x=0.36, y=0.21, z=0.98[0m
[0;32mI (11966) GYROSCOPE: Time: [33014406 ms] Gyroscope: x=-1.58, y=0.09, z=1.27[0m
[0;32mI (12126) ACCELEROMETER: Time: [33014626 ms] Accelerometer: x=0.34, y=0.21, z=1.00[0m
[0;32mI (12186) GYROSCOPE: Time: [33014626 ms] Gyroscope: x=4.17, y=-2.43, z=-3.73[0m
[0;32mI (12346) ACCELEROMETER: Time: [33014846 ms] Accelerometer: x=0.36, y=0.20, z=0.98[0m
[0;32mI (12406) GYROSCOPE: Time: [33014846 ms] Gyroscope: x=-1.62, y=3.33, z=2.13[0m
[0;32mI (12566) ACCELEROMETER: Time: [33015066 ms] Accelerometer: x=0.37, y=0.21, z=0.96[0m
[0;32mI (12626) GYROSCOPE: Time: [33015066 ms] Gyroscope: x=0.88, y=2.43, z=0.93[0m
[0;32mI (12786) ACCELEROMETER: Time: [33015286 ms] Accelerometer: x=0.36, y=0.23, z=0.99[0m
[0;32mI (12846) GYROSCOPE: Time: [33015286 ms] Gyroscope: x=3.71, y=0.41, z=-2.41[0m
[0;32mI (13006) ACCELEROMETER: Time: [33015506 ms] Accelerometer: x=0.36, y=0.22, z=0.95[0m
[0;32mI (13066) GYROSCOPE: Time: [33015506 ms] Gyroscope: x=-1.56, y=4.62, z=1.41[0m
[0;32mI (13226) ACCELEROMETER: Time: [33015726 ms] Accelerometer: x=0.34, y=0.25, z=1.00[0m
"""

# Regex pattern to extract the timestamps and sensor data
pattern = r"\x1b\[0;32mI \(\d+\) (ACCELEROMETER|GYROSCOPE): Time: \[(\d+) ms\] (?:Accelerometer|Gyroscope): x=([-+]?[0-9]*\.?[0-9]+), y=([-+]?[0-9]*\.?[0-9]+), z=([-+]?[0-9]*\.?[0-9]+)\x1b\[0m"

# Initializing arrays for accelerometer and gyroscope data
time_array = []
acc_x, acc_y, acc_z = [], [], []
gyr_x, gyr_y, gyr_z = [], [], []

# Extracting data using regex
for match in re.finditer(pattern, data):
    sensor, timestamp, x, y, z = match.groups()
    timestamp_sec = int(timestamp) / 1000.0  # Convert timestamp to seconds

    # Append data to the corresponding arrays
    if sensor == "ACCELEROMETER":
        acc_x.append(float(x))
        acc_y.append(float(y))
        acc_z.append(float(z))
        if not time_array or time_array[-1] != timestamp_sec:
            time_array.append(timestamp_sec)
    elif sensor == "GYROSCOPE":
        gyr_x.append(float(x))
        gyr_y.append(float(y))
        gyr_z.append(float(z))

print(time_array)
print(acc_x)
print(acc_y)
print(acc_z)
print(gyr_x)
print(gyr_y)
print(gyr_z)

time = np.array(time_array)
acc_x = np.array(acc_x)
acc_y = np.array(acc_y)
acc_z = np.array(acc_z)
gyr_x = np.array(gyr_x)
gyr_y = np.array(gyr_y)
gyr_z = np.array(gyr_z)

time = np.array(time) - time[0]
dt = np.mean(np.diff(time))  # Average time step in case of non-uniform sampling

# Data arrays
data_arrays = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
titles = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']

# Create subplots
fig, axs = plt.subplots(6, 1, figsize=(12, 18))

for i, data in enumerate(data_arrays):
    # Detrend the data to remove linear trend or offset
    data_detrended = detrend(data)

    # Compute the spectrogram
    f, t, Sxx = spectrogram(data_detrended, fs=1/dt, nperseg=8, noverlap=4)

    # Avoiding log of zero issues by adding a small value before taking log
    Sxx_log = 10 * np.log10(Sxx + np.finfo(float).eps)

    # Plotting
    axs[i].pcolormesh(t, f, Sxx_log, shading='gouraud')
    axs[i].set_ylabel('Frequency [Hz]')
    axs[i].set_title(titles[i])

# Set common labels
for ax in axs:
    ax.set_xlabel('Time [sec]')

plt.tight_layout()
plt.show()
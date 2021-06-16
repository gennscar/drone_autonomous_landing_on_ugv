import csv
import time
import numpy as np
from matplotlib import pyplot as plt
from ros2bag import BagFileParser

# CONSTANTS
ROS2BAG_FILE = 'rosbags/MovingTag0/MovingTag0_0.db3'
LEICA_FILE = 'rosbags/MovingTag0/MovingTag0.csv'
LEICA_Z_OFFSET = 0.06  # mm under the UWB sensor

ANCHOR_POSITION = np.array([
    [2.767350, -17.577792, 1.046610],
    [2.084902, -1.106974,  1.054502],
    [-5.286977, -1.389804, 0.904299],
    [-4.680692, -17.822773, 1.597551],
])

# Parsing ros2bag files
parser = BagFileParser(ROS2BAG_FILE)
uwb_data = parser.get_messages("/uwb_ranging")

# Saving time and ranges in np.array
uwb_time = []
uwb_ranges = []
for data in uwb_data:
    uwb_time.append(data[0])
    uwb_ranges.append(data[1].range_mes)

uwb_time = np.array(uwb_time, dtype=float) * 1e-9
uwb_ranges = np.array(uwb_ranges, dtype=float) * 1e-2  # From cm to m

# Parsing LEICA file
leica_time = []
leica_pos = []
with open(LEICA_FILE, newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=';')
    reader.__next__()
    for row in reader:
        time_string = row[12]
        leica_time.append(time.mktime(time.strptime(
            time_string[:-3], "'%d.%m.%Y %H:%M:%S.")) + float(time_string[-4:]))

        leica_pos.append([row[1].replace(',', "."), row[2].replace(
            ',', "."), row[3].replace(',', ".")])

leica_time = np.array(leica_time, dtype=float)
leica_pos = np.array(leica_pos, dtype=float)

leica_pos[:, 2] += LEICA_Z_OFFSET  # Adding reflector offset

# True ranges
leica_ranges = np.empty((len(leica_pos), ANCHOR_POSITION.shape[0]))
for i in range(ANCHOR_POSITION.shape[0]):
    leica_ranges[:, i] = np.linalg.norm(
        leica_pos - ANCHOR_POSITION[i, :], axis=1)

# Plot for ranges
fig, ax = plt.subplots(ANCHOR_POSITION.shape[0], 2)
for i in range(ANCHOR_POSITION.shape[0]):
    ax[i][0].plot(uwb_time, uwb_ranges[:, i], label="UWB:" + str(i))
    ax[i][0].plot(leica_time, leica_ranges[:, i], label="LEICA:" + str(i))
    ax[i][0].legend(loc='best')
    ax[i][0].grid()

# Interpolation
uwb_ranges_int = np.empty((len(leica_time), ANCHOR_POSITION.shape[0]))
for i in range(ANCHOR_POSITION.shape[0]):
    uwb_ranges_int[:, i] = np.interp(leica_time, uwb_time, uwb_ranges[:, i])

# Ranging error
ranging_error = leica_ranges - uwb_ranges_int

# Variance
print('MEAN', np.mean(ranging_error, axis=0))
print('STD', np.sqrt(np.var(ranging_error, axis=0)))
print('MAE', np.linalg.norm(ranging_error, axis=0, ord=1) / len(ranging_error))
print('RMSE', np.linalg.norm(ranging_error, axis=0, ord=2) / len(ranging_error))

# Plot ranging data for 1 anchor
for i in range(ANCHOR_POSITION.shape[0]):
    ax[i][1].plot(leica_time, ranging_error[:, i], label="Error:" + str(i))
    ax[i][1].plot(loc='best')
    ax[i][1].grid()

plt.show()

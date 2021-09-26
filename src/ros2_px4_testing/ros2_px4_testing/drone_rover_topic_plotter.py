import matplotlib.pyplot as plt
import csv

path_ = '/home/gennscar/'


KF_pos_estimator_0_ = open(path_ + 'KF_pos_estimator_0.csv', 'r')
LS_pos_estimator_ = open(path_ + 'LS_pos_estimator.csv', 'r')
LS_norot_pos_estimator_ = open(path_ + 'LS_norot_pos_estimator.csv', 'r')
range_sensor_ = open(path_ + 'range_sensor.csv', 'r')
yaw_sensor_ = open(path_ + 'yaw_sensor.csv', 'r')


KF_pos_estimator_0_rows_ = csv.reader(KF_pos_estimator_0_, delimiter = ',')
LS_pos_estimator_rows_ = csv.reader(LS_pos_estimator_, delimiter = ',')
LS_norot_pos_estimator_rows_ = csv.reader(LS_norot_pos_estimator_, delimiter = ',')
range_sensor_rows_ = csv.reader(range_sensor_, delimiter = ',')
yaw_sensor_rows_ = csv.reader(yaw_sensor_, delimiter = ',')


KF_pos_estimator_0_data_ = []
for row_ in KF_pos_estimator_0_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    KF_pos_estimator_0_data_.append(row_data_)

LS_pos_estimator_data_ = []
for row_ in LS_pos_estimator_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    LS_pos_estimator_data_.append(row_data_)

LS_norot_pos_estimator_data_ = []
for row_ in LS_norot_pos_estimator_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    LS_norot_pos_estimator_data_.append(row_data_)

range_sensor_data_ = []
for row_ in range_sensor_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    range_sensor_data_.append(row_data_)

yaw_sensor_data_ = []
for row_ in yaw_sensor_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    yaw_sensor_data_.append(row_data_)

###################################################################


# Position estimation
fig, ax = plt.subplots(2, 1, sharex=True)
fig.suptitle('Position estimation')
ax[0].plot([i[0] for i in KF_pos_estimator_0_data_], [i[1] for i in KF_pos_estimator_0_data_], label='Kalman Filter 0')
ax[0].plot([i[0] for i in LS_pos_estimator_data_], [i[1] for i in LS_pos_estimator_data_], label='Least Squares')
ax[0].legend()
ax[0].set_ylabel("x [m]")
ax[0].grid()
ax[1].plot([i[0] for i in KF_pos_estimator_0_data_], [i[2] for i in KF_pos_estimator_0_data_], label='Kalman Filter 0')
ax[1].plot([i[0] for i in LS_pos_estimator_data_], [i[2] for i in LS_pos_estimator_data_], label='Least Squares')
ax[1].legend()
ax[1].set_ylabel("y [m]")
ax[1].grid()
plt.xlabel("time [s]")
plt.show()

# Range estimation
plt.plot([i[0] for i in KF_pos_estimator_0_data_],[i[3] for i in KF_pos_estimator_0_data_], label='Kalman Filter 0')
plt.plot([i[0] for i in range_sensor_data_],[i[1] for i in range_sensor_data_], label='Raw range sensor')
plt.xlabel('time [s]')
plt.ylabel('z [m]')
plt.title('Range estimation')
plt.legend()
plt.grid()
plt.show()

# Yaw estimation
plt.plot([i[0] for i in KF_pos_estimator_0_data_],[i[4] for i in KF_pos_estimator_0_data_], label='Kalman Filter 0')
plt.plot([i[0] for i in yaw_sensor_data_],[i[1] for i in yaw_sensor_data_], label='Raw yaw sensor')
plt.xlabel('time [s]')
plt.ylabel('theta [deg]')
plt.title('Yaw estimation')
plt.legend()
plt.grid()
plt.show()

# anchors and ENU frame LS
fig, ax = plt.subplots(2, 1, sharex=True)
fig.suptitle('LS estimated position in anchors and ENU frames')
ax[0].plot([i[0] for i in LS_norot_pos_estimator_data_], [i[1] for i in LS_norot_pos_estimator_data_], label='Anchors frame')
ax[0].plot([i[0] for i in LS_pos_estimator_data_], [i[1] for i in LS_pos_estimator_data_], label='ENU frame')
ax[0].set_ylabel("x [m]")
ax[0].legend()
ax[0].grid()
ax[1].plot([i[0] for i in LS_norot_pos_estimator_data_], [i[2] for i in LS_norot_pos_estimator_data_], label='Anchors frame')
ax[1].plot([i[0] for i in LS_pos_estimator_data_], [i[2] for i in LS_pos_estimator_data_], label='ENU frame')
ax[1].set_ylabel("y [m]")
ax[1].grid()
ax[1].legend()
plt.xlabel("time [s]")
plt.show()

# Closing files
KF_pos_estimator_0_.close()
LS_pos_estimator_.close()
range_sensor_.close()
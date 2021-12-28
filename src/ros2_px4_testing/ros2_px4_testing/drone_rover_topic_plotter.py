import matplotlib.pyplot as plt
import csv
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
import numpy as np 

path_ = '/home/gennscar/'


KF_pos_estimator_0_ = open(path_ + 'KF_pos_estimator_0.csv', 'r')
LS_pos_estimator_ = open(path_ + 'LS_pos_estimator.csv', 'r')
LS_norot_pos_estimator_ = open(path_ + 'LS_norot_pos_estimator.csv', 'r')
KF_pos_estimator_error_0_ = open(path_ + 'KF_pos_estimator_error_0.csv', 'r')
drone_local_position_ = open(path_ + 'drone_local_position.csv', 'r')


KF_pos_estimator_0_rows_ = csv.reader(KF_pos_estimator_0_, delimiter = ',')
LS_pos_estimator_rows_ = csv.reader(LS_pos_estimator_, delimiter = ',')
LS_norot_pos_estimator_rows_ = csv.reader(LS_norot_pos_estimator_, delimiter = ',')
KF_pos_estimator_error_0_rows_ = csv.reader(KF_pos_estimator_error_0_, delimiter = ',')
drone_local_position_rows_ = csv.reader(drone_local_position_, delimiter = ',')


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

KF_pos_estimator_error_0_data_ = []
for row_ in KF_pos_estimator_error_0_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    KF_pos_estimator_error_0_data_.append(row_data_)

drone_local_position_data_ = []
for row_ in drone_local_position_rows_:
    row_data_ = []
    for field_ in row_:
        row_data_.append(float(field_))
    drone_local_position_data_.append(row_data_)

###################################################################
linewidth = 1
x_maj_tick = 5
x_min_tick = 1
y_maj_tick = 0.1
y_min_tick = 0.05

x1 = np.arange(5.3, 13, 0.01)
x2 = np.arange(13, 21.6, 0.01)
x3 = np.arange(21.6, 33.5, 0.01)

min_x = 3 
max_x = [i[0] for i in KF_pos_estimator_0_data_][-1]
#min_x = 5
#max_x = 32
min_y = -0.1
max_y = 0.5


# Position estimation
fig, ax = plt.subplots()
ax.plot([i[0] for i in LS_pos_estimator_data_], [i[1] for i in LS_pos_estimator_data_], linewidth=linewidth/2, color='cornflowerblue', label='LS estimated x [m]')
ax.plot([i[0] for i in LS_pos_estimator_data_], [i[2] for i in LS_pos_estimator_data_], linewidth=linewidth/2, color='orange', label='LS estimated y [m]')
ax.plot([i[0] for i in KF_pos_estimator_0_data_], [i[1] for i in KF_pos_estimator_0_data_], linewidth=linewidth, color='blue', label='KF estimated x [m]')
ax.plot([i[0] for i in KF_pos_estimator_0_data_], [i[2] for i in KF_pos_estimator_0_data_], linewidth=linewidth, color='red', label='KF estimated y [m]')
ax.xaxis.set_major_locator(MultipleLocator(x_maj_tick))
ax.xaxis.set_major_formatter('{x:.0f}')
ax.xaxis.set_minor_locator(MultipleLocator(x_min_tick))
ax.yaxis.set_major_locator(MultipleLocator(y_maj_tick))
ax.yaxis.set_major_formatter('{x:.0f}')
ax.yaxis.set_minor_locator(MultipleLocator(y_min_tick))
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)

#ax.fill_between(x1, 0, 1, color='deepskyblue', alpha=0.1, transform=ax.get_xaxis_transform(), label='Takeoff')
#ax.fill_between(x2, 0, 1, color='red', alpha=0.1, transform=ax.get_xaxis_transform(), label='Chase')
#ax.fill_between(x3, 0, 1, color='lime', alpha=0.1, transform=ax.get_xaxis_transform(), label = 'Descent')

plt.xlabel('Time [s]')
plt.ylabel('Distance [m]')
plt.title('Position estimation')
plt.legend(loc="best", fontsize = 'x-small', framealpha=0.9)
plt.grid()
plt.show()


# Landing mission
fig, ax = plt.subplots()
ax.plot([i[0] for i in KF_pos_estimator_0_data_],[i[1] for i in KF_pos_estimator_0_data_], linewidth=linewidth, color='blue', label='Filtered relative x [m]')
ax.plot([i[0] for i in KF_pos_estimator_0_data_],[i[2] for i in KF_pos_estimator_0_data_], linewidth=linewidth, color='red', label='Filtered relative y [m]')
ax.plot([i[0] for i in drone_local_position_data_],[-i[3] for i in drone_local_position_data_], linewidth=linewidth, color='black', label='Filtered absolute z [m]')
ax.xaxis.set_major_locator(MultipleLocator(x_maj_tick))
ax.xaxis.set_major_formatter('{x:.0f}')
ax.xaxis.set_minor_locator(MultipleLocator(x_min_tick))
ax.yaxis.set_major_locator(MultipleLocator(y_maj_tick))
ax.yaxis.set_major_formatter('{x:.0f}')
ax.yaxis.set_minor_locator(MultipleLocator(y_min_tick))

ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
ax.fill_between(x1, 0, 1, color='deepskyblue', alpha=0.1, transform=ax.get_xaxis_transform(), label='Takeoff')
ax.fill_between(x2, 0, 1, color='red', alpha=0.1, transform=ax.get_xaxis_transform(), label='Chase')
ax.fill_between(x3, 0, 1, color='lime', alpha=0.1, transform=ax.get_xaxis_transform(), label = 'Descent')
plt.xlabel('Time [s]')
plt.ylabel('Distance [m]')
plt.title('Landing mission')
plt.legend(loc="best", fontsize = 'x-small', framealpha=0.9)
plt.grid()
plt.show()


# Positioning error
fig, ax = plt.subplots()
ax.plot([i[0] for i in KF_pos_estimator_error_0_data_],[i[1] for i in KF_pos_estimator_error_0_data_], linewidth=linewidth, color='blue', label='positioning error [m]')
#ax.plot([i[0] for i in KF_pos_estimator_error_0_data_],[i[2] for i in KF_pos_estimator_error_0_data_], linewidth=linewidth, color='red', label='y positioning error [m]')
ax.xaxis.set_major_locator(MultipleLocator(x_maj_tick))
ax.xaxis.set_major_formatter('{x:.0f}')
ax.xaxis.set_minor_locator(MultipleLocator(x_min_tick))
ax.yaxis.set_major_locator(MultipleLocator(y_maj_tick))
ax.yaxis.set_major_formatter('{x:.02f}')
ax.yaxis.set_minor_locator(MultipleLocator(y_min_tick))
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
x1 = np.arange(6, 16, 0.01)
x2 = np.arange(16, 23, 0.01)
x3 = np.arange(23, 34, 0.01)
#ax.fill_between(x1, 0, 1, color='deepskyblue', alpha=0.1, transform=ax.get_xaxis_transform(), label='Takeoff')
#ax.fill_between(x2, 0, 1, color='red', alpha=0.1, transform=ax.get_xaxis_transform(), label='Chase')
#ax.fill_between(x3, 0, 1, color='lime', alpha=0.1, transform=ax.get_xaxis_transform(), label = 'Descent')
plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.title('Positioning error')
plt.legend(loc="best", fontsize = 'x-small', framealpha=0.9)
plt.grid()
plt.show()


# Closing files
KF_pos_estimator_0_.close()
LS_pos_estimator_.close()
LS_norot_pos_estimator_.close()
drone_local_position_.close()
KF_pos_estimator_error_0_.close()

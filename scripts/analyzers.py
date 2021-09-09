import csv
import time
import numpy as np
from matplotlib import pyplot as plt
from ros2bag import BagFileParser


class MovingTagAnalyzer():
    def __init__(self, anchor_pose):
        """
        MovingTagAnalyzer can parse ros2bag and LEICA file to extract
        ranging info

        Args:
            anchor_pose (np.array): A matrix containing for each row the UWB
            anchor true coordinate x, y, z measured with LEICA of 1 anchor,
            it is possible to set a variable number of anchors
        """

        self.anchor_pose = anchor_pose

        self.ros2bag_time = []
        self.ros2bag_range = []

        self.leica_time = []
        self.leica_pose = []
        self.leica_range = []

    def parse_ros2bag(self, ros2bag_file, ranging_topic_name):
        """Parse a ros2bag file and retain/return ranging info

        Args:
            ros2bag_file (str): Name of the file to parse
            ranging_topic_name (str): Name of the ros2 topic that contain the
            UWB measurements

        Returns:
            np.array: Vector of timestamps
            np.array: Vector of ranges associated by previous timestamps, each
            row contains ranging of 1 anchor
        """

        # Parsing the ros2bag
        parser = BagFileParser(ros2bag_file)
        uwb_data = parser.get_messages(ranging_topic_name)

        # Saving info
        for data in uwb_data:
            self.ros2bag_time.append(data[0])
            self.ros2bag_range.append(data[1].range_mes)

        # Converting all measurements to np.array(dtype=float) and SI units
        self.ros2bag_time = np.array(
            self.ros2bag_time, dtype=float) * 1e-9  # from nanoseconds
        self.ros2bag_range = np.array(
            self.ros2bag_range, dtype=float) * 1e-2  # from centimeters

        return self.ros2bag_time, self.ros2bag_range

    def parse_leica(self, leica_file, leica_offset=np.zeros(3)):
        """Parse a ros2bag file and retain/return pose and ranging info

        Args:
            leica_file (str): Name of the file to parse
            leica_offset (np.array((3)), optional): Set a position offset
            betweeen LEICA data and UWB measurements. Defaults to np.zeros(3).

        Returns:
            np.array: Vector of timestamps
            np.array: Vector of pose of the tracked tag associated by previous
            timestamps
            np.array: Vector of ranges associated by previous timestamps, each
            row contains ranging of 1 anchor
        """

        # Parsing LEICA file
        with open(leica_file, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=';')

            # Removing the first line
            reader.__next__()

            for row in reader:
                # Time conversion from string to UNIX time
                time_string = row[12]
                self.leica_time.append(time.mktime(time.strptime(
                    time_string[:-3], "'%d.%m.%Y %H:%M:%S.")) + float(time_string[-4:]))

                # Pose conversion from string
                self.leica_pose.append([row[1].replace(',', "."), row[2].replace(
                    ',', "."), row[3].replace(',', ".")])

        # Converting all measurements to np.array(dtype=float) and SI units
        self.leica_time = np.array(self.leica_time, dtype=float)
        self.leica_pose = np.array(self.leica_pose, dtype=float)

        # Adding reflector offset
        self.leica_pose += leica_offset

        # Range measurement
        self.leica_range = np.empty(
            (len(self.leica_pose), self.anchor_pose.shape[0]))
        for i in range(self.anchor_pose.shape[0]):
            self.leica_range[:, i] = np.linalg.norm(
                self.leica_pose - self.anchor_pose[i, :], axis=1)

        return self.leica_time, self.leica_pose, self.leica_range

    def ranging_error(self):
        """
        Calculate the ranging error between LEICA and UWB, it is necessary to
        call parse_ros2bag() and parse_leica() first

        Returns:
            np.array: Vector of timestamps
            np.array: Ranging error associated by previous timestamps, each
            row contains errors of 1 anchor
        """

        # Checking correct execution
        if len(self.ros2bag_time) == 0 or len(self.leica_time) == 0:
            print('First MUST parse a ros2bag file and a LEICA file')
            return

        # Interpolating UWB ranging data in the LEICA ones
        # @todo: Here we need to apply a more robust interp method and
        #        point selection
        interp_range = np.empty(
            (len(self.leica_pose), self.anchor_pose.shape[0]))
        for i in range(self.anchor_pose.shape[0]):
            interp_range[:, i] = np.interp(
                self.leica_time, self.ros2bag_time, self.ros2bag_range[:, i])

        return self.leica_time, self.leica_range - interp_range


ROS2BAG_FILE = 'rosbags/FixedTag0/FixedTag0_0.db3'
LEICA_FILE = 'rosbags/FixedTag0/FixedTag0_0.db3.csv'
LEICA_Z_OFFSET = 0.06  # mm under the UWB sensor

ANCHOR_POSITION = np.array([
    [2.767350, -17.577792, 1.046610],
    [2.084902, -1.106974,  1.054502],
    [-5.286977, -1.389804, 0.904299],
    [-4.680692, -17.822773, 1.597551],
])

if __name__ == "__main__":
    anal = MovingTagAnalyzer(ANCHOR_POSITION)

    uwb_time, uwb_range = anal.parse_ros2bag(ROS2BAG_FILE, "/uwb_ranging")

    leica_time, _, leica_range = anal.parse_leica(
        LEICA_FILE, np.array([0.0, 0.0, LEICA_Z_OFFSET]))

    time_error, ranging_error = anal.ranging_error()

    fig, ax = plt.subplots(ANCHOR_POSITION.shape[0], 2)
    for i in range(ANCHOR_POSITION.shape[0]):
        ax[i][0].plot(uwb_time, uwb_range[:, i], label="UWB, anchor " + str(i))
        ax[i][0].plot(leica_time, leica_range[:, i],
                      label="LEICA, anchor " + str(i))
        ax[i][0].legend(loc='best')
        ax[i][0].grid()

    # Plot ranging data for 1 anchor
    for i in range(ANCHOR_POSITION.shape[0]):
        ax[i][1].plot(time_error, ranging_error[:, i],
                      label="Error, anchor " + str(i))
        ax[i][1].plot(loc='best')
        ax[i][1].grid()

    # Variance
    print('MEAN', np.mean(ranging_error, axis=0))
    print('STD', np.sqrt(np.var(ranging_error, axis=0)))
    print('MAE', np.linalg.norm(ranging_error, axis=0, ord=1) / len(ranging_error))
    print('RMSE', np.linalg.norm(ranging_error,
          axis=0, ord=2) / len(ranging_error))

    plt.show()

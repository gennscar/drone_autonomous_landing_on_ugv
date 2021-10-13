from re import A
import time
import numpy as np
from matplotlib import pyplot as plt
from ros2bag import BagFileParser


class MovingTagAnalyzer():
    def __init__(self):
        """
        MovingTagAnalyzer can parse ros2bag and LEICA file to extract
        ranging info

        Args:
            anchor_pose (np.array): A matrix containing for each row the UWB
            anchor true coordinate x, y, z measured with LEICA of 1 anchor,
            it is possible to set a variable number of anchors
        """

        self.anchor_pose = {}

        self.uwb_time = {}
        self.uwb_range = {}

        self.odom_time = []
        self.odom_pose = []

        self.leica_time = []
        self.leica_pose = []
        self.leica_range = {}

    def parse_ros2bag_uwb(self, ros2bag_file, ranging_topic_name):
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
            anchor_id = data[1].anchor_pose.header.frame_id

            # Create an empty np.array for each new anchor
            if anchor_id not in self.uwb_time.keys():
                self.uwb_time[anchor_id] = []
                self.uwb_range[anchor_id] = []

            self.uwb_time[anchor_id].append(float(data[0]))
            self.uwb_range[anchor_id].append(data[1].range)
            self.anchor_pose[anchor_id] = np.array([
                data[1].anchor_pose.pose.position.x,
                data[1].anchor_pose.pose.position.y,
                data[1].anchor_pose.pose.position.z
            ])

        # Converting all measurements to np.array(dtype=float) and SI units
        for anchor_id in self.uwb_time.keys():
            self.uwb_time[anchor_id] = np.array(
                self.uwb_time[anchor_id], dtype=float)*1e-9
            self.uwb_range[anchor_id] = np.array(
                self.uwb_range[anchor_id], dtype=float)

        return self.uwb_time, self.uwb_range

    def parse_leica(self, leica_file, points_id="", leica_offset=np.zeros(4)):
        """Parse a ros2bag file and retain/return pose and ranging info

        Args:
            leica_file (str): Name of the file to parse
            leica_offset (np.array((3)), optional): Set a position offset
            between LEICA data and UWB measurements. Defaults to np.zeros(3).

        Returns:
            np.array: Vector of timestamps
            np.array: Vector of pose of the tracked tag associated by previous
            timestamps
            np.array: Vector of ranges associated by previous timestamps, each
            row contains ranging of 1 anchor
        """

        leica_time = leica_offset[0]

        # Parsing LEICA file
        with open(leica_file, newline="") as file:

            # Reading file
            for row in file:
                # Take only the requested points
                if points_id not in row:
                    continue

                row = row.split()

                self.leica_time.append(leica_time)
                leica_time += 0.150

                self.leica_pose.append([
                    row[1][10:], row[2][10:], row[3][10:]
                ])

        # Converting all measurements to np.array(dtype=float) and SI units
        self.leica_time = np.array(self.leica_time, dtype=float)
        self.leica_pose = np.array(self.leica_pose, dtype=float) * 1e-3

        # Adding reflector offset
        self.leica_pose += leica_offset[1:4]

        # Range measurement
        for anchor_id in self.anchor_pose.keys():
            self.leica_range[anchor_id] = np.linalg.norm(
                self.leica_pose - self.anchor_pose[anchor_id], axis=1)

        return self.leica_time, self.leica_pose, self.leica_range

    def parse_ros2bag_odom(self, ros2bag_file, odom_topic_name):
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
        odom_data = parser.get_messages(odom_topic_name)

        # Saving info
        for data in odom_data:
            self.odom_time.append(float(data[0]))
            self.odom_pose.append(np.array([
                data[1].pose.pose.position.x,
                data[1].pose.pose.position.y,
                data[1].pose.pose.position.z
            ]))

        # Converting all measurements to np.array(dtype=float) and SI units
        self.odom_time = np.array(self.odom_time, dtype=float)*1e-9
        self.odom_pose = np.array(self.odom_pose, dtype=float)

        return self.odom_time, self.odom_pose


# Filenames
ROS2BAG_FILE = "rosbags/rosbag2_2021_10_13-16_08_57/rosbag2_2021_10_13-16_08_57_0.db3"
ROS2_UWB_TOPIC = "/uwb_sensor_tag_0"
ROS2_ODOM_TOPIC = "/X500_2/UkfPositioning/Odometry"

LEICA_FILE = "rosbags/rosbag2_2021_10_13-16_08_57/C3"
POINTS_ID = "AAA"

# Time [seconds] and pose [m] offset of LEICA measurements
LEICA_OFFSET = np.array([1634134147., -100., -100., -100.])

if __name__ == "__main__":
    anal = MovingTagAnalyzer()

    # Analyzing UWB data
    uwb_time, uwb_range = anal.parse_ros2bag_uwb(ROS2BAG_FILE, ROS2_UWB_TOPIC)

    # Analyzing Leica data
    leica_time, leica_pose, leica_range = anal.parse_leica(
        LEICA_FILE, POINTS_ID, LEICA_OFFSET)

    # Plotting ranges
    N = 0
    for anchor_id in uwb_time.keys():
        print(uwb_time[anchor_id][0])
        N += 1
        ax = plt.subplot(len(uwb_time.keys()), 1, N)
        plt.grid()

        plt.plot(uwb_time[anchor_id], uwb_range[anchor_id],
                 label="UWB, anchor " + anchor_id)
        plt.plot(leica_time, leica_range[anchor_id],
                 label="LEICA, anchor " + anchor_id)
        plt.legend()

    plt.show()

    # Analyzing odometry
    odom_time, odom_pose = anal.parse_ros2bag_odom(
        ROS2BAG_FILE, ROS2_ODOM_TOPIC)

    for i in range(0, 3):
        ax = plt.subplot(3, 1, i+1)
        plt.grid()

        plt.plot(odom_time, odom_pose[:, i], label="Odometry")
        plt.plot(leica_time, leica_pose[:, i], label="Leica")
        plt.legend()

    plt.show()

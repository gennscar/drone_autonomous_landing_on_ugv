#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R

from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined
from ros2_px4_interfaces.msg import UwbSensor


QUEUE_SIZE = 10
FILTER_DIM = 9+4+4+3  # Linear kinematic + quaternion kinematic + uwb + bias
IS_SENSOR_ALIVE_TIMEOUT = 5.  # s


class UkfPositioning(Node):
    """UKF: Try to explain this :O
    """

    def __init__(self):
        super().__init__("UkfPositioning")

        self.declare_parameters("", [
            ("delta_t", 0.),
            ("q", 0.),
            ("r_uwb", 0.),
            ("r_gps", 0.),
            ("r_imu", 0.),
        ])

        self.params_ = {
            x.name: x.value for x in self.get_parameters(
                ["delta_t", "q", "r_uwb", "r_gps", "r_imu"]
            )
        }

        self.filter_state_ = "Offline"
        self.calibration_counter_ = 0
        self.last_sensor_list_ = []
        self.sensor_wd_ = {
            "uwb": 0,
            "gps": 0,
            "imu": 0
        }

        # Kalman Filter
        sigmas = MerweScaledSigmaPoints(
            FILTER_DIM, alpha=1e-3, beta=2., kappa=0.,
            sqrt_method=scipy.linalg.sqrtm
        )

        def f_(x, dt):
            f = np.array([
                [1., dt, 0.5*dt**2.],
                [0., 1.,         dt],
                [0., 0.,         1.]
            ])
            F = scipy.linalg.block_diag(*[f]*3, np.eye(11))
            return F @ x

        self.kalman_filter_ = UKF(
            dim_x=FILTER_DIM,
            dim_z=FILTER_DIM,
            dt=self.params_["delta_t"],
            fx=f_, hx=None,
            points=sigmas
        )

        # Initial estimate
        self.kalman_filter_.x *= 1.

        # Covariance matrix
        self.kalman_filter_.P *= 1.

        # Process noise
        self.kalman_filter_.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(
                dim=3, dt=self.params_["delta_t"], var=self.params_["q"], block_size=3),
            np.zeros((11, 11))
        )

        # Setting up sensors subscribers
        self.uwb_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_tag_0",
            self.callback_uwb_subscriber, QUEUE_SIZE
        )
        self.gps_subscriber_ = self.create_subscription(
            Odometry, "GpsPositioning/Odometry",
            self.callback_gps_subscriber, QUEUE_SIZE
        )
        self.imu_subscriber_ = self.create_subscription(
            SensorCombined, "SensorCombined_PubSubTopic",
            self.callback_imu_subscriber, QUEUE_SIZE
        )

        # Setting up position and velocity publisher
        self.odometry_publisher_ = self.create_publisher(
            Odometry, "~/Odometry", QUEUE_SIZE
        )

        # Prediction timer
        self.timer = self.create_timer(
            self.params_["delta_t"], self.predict_callback)

        self.get_logger().info(f"Node has started: {self.params_}")

    def callback_uwb_subscriber(self, msg):
        """Measuring UWB range sensor

        Args:
            msg (ros2_px4_interfaces.msg.UwbSensor): The UWB message
        """

        # Must predict once first
        if(self.filter_state_ == "Offline"):
            return

        # Storing timestamp
        self.sensor_wd_["uwb"] = self.get_clock().now().nanoseconds

        # Storing measurement in a np.array
        z = np.zeros(FILTER_DIM)
        z[0] = msg.range

        if any(np.isnan(z)):
            self.get_logger().error(f"Invalid UWB data")
            return

        # Storing anchor position in a np.array
        anchor_position = np.array([
            msg.anchor_pose.pose.position.x,
            msg.anchor_pose.pose.position.y,
            msg.anchor_pose.pose.position.z
        ])

        # Simulation rotation TO REMOVE
        #r = R.from_rotvec([0., 0., math.pi/3])
        #anchor_position = r.apply(anchor_position)

        if (True or self.sensor_wd_["uwb"] - self.sensor_wd_["gps"] < IS_SENSOR_ALIVE_TIMEOUT):
            # Measurement model for a rotated range sensor
            def h_uwb(x):
                h = np.zeros(FILTER_DIM)

                # Transform the anchor position in the local frame
                r = R.from_rotvec([0., 0., x[16]])

                offset_anc_pos = anchor_position + x[13:16]
                rot_anc_pos = r.apply(offset_anc_pos)

                # Range measurements
                h[0] = np.linalg.norm(x[[0, 3, 6]] - rot_anc_pos)
                return h
        else:
            # Measurement model for a range sensor
            def h_uwb(x):
                h = np.zeros(FILTER_DIM)

                # Range measurement
                h[0] = np.linalg.norm(x[[0, 3, 6]] - anchor_position)
                return h

        # Filter update
        self.kalman_filter_.update(z, R=self.params_["r_uwb"], hx=h_uwb)

    def callback_gps_subscriber(self, msg):
        """Measuring GPS sensor

        Args:
            msg (nav_msgs.msg.Odometry): The GPS message
        """

        # Must predict once first
        if(self.filter_state_ == "Offline"):
            return

        # Storing timestamp
        self.sensor_wd_["gps"] = self.get_clock().now().nanoseconds

        # Storing measurements in a np.array
        z = np.zeros(FILTER_DIM)
        z[0:6] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])

        if any(np.isnan(z)):
            self.get_logger().error(f"Invalid GPS data")
            return

        # Measurement model for a GPS sensor
        def h_gps(x):
            h = np.zeros(FILTER_DIM)
            h[0:6] = np.array([x[0], x[3], x[6], x[1], x[4], x[7]])
            return h

        # Filter update
        self.kalman_filter_.update(z, R=self.params_["r_gps"], hx=h_gps)

    def callback_imu_subscriber(self, msg):
        """Measuring accelerometer and gyroscope sensor

        Args:
            msg (px4_msgs.msg.SensorCombined): The sensors message
        """

        # Must predict once first
        if(self.filter_state_ == "Offline"):
            return

        # Storing timestamp
        self.sensor_wd_["imu"] = self.get_clock().now().nanoseconds

        # Storing measurements in a np.array
        z = np.zeros(FILTER_DIM)
        z[0:6] = np.array([
            msg.gyro_rad[0],
            msg.gyro_rad[1],
            msg.gyro_rad[2],
            msg.accelerometer_m_s2[0],
            msg.accelerometer_m_s2[1],
            msg.accelerometer_m_s2[2]
        ])

        if any(np.isnan(z)):
            self.get_logger().error(f"Invalid IMU data")
            return

        # Measurement model for IMU sensor
        def h_imu(x):
            h = np.zeros(FILTER_DIM)
            h[3:6] = np.array(
                [x[5] - x[17], x[2] - x[18], -x[8] - x[19] - 9.805])
            return h

        # Filter update
        self.kalman_filter_.update(z, R=self.params_["r_imu"], hx=h_imu)

    def predict_callback(self):
        """This callback perform the filter predict and forward the current
        estimate
        """

        # self.get_logger().info(f"{self.kalman_filter_.x}")

        if (self.filter_state_ == "Initialized"):
            if(np.linalg.norm(self.kalman_filter_.x - self.kalman_filter_.x_prior) < 1.):
                self.calibration_counter_ += 1
            else:
                self.calibration_counter_ = 0

        if (self.filter_state_ == "Initialized" and self.calibration_counter_ > 100):
            self.get_logger().info("Filter calibrated")
            self.filter_state_ = "Calibrated"

        # Send estimation only id calibrated
        if(self.filter_state_ == "Calibrated"):

            # Sending the estimated position
            msg = Odometry()
            msg.header.frame_id = "UkfPositioning"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = self.kalman_filter_.x[0]
            msg.pose.pose.position.y = self.kalman_filter_.x[3]
            msg.pose.pose.position.z = self.kalman_filter_.x[6]

            # Adding other filter states
            msg.pose.pose.orientation.x = self.kalman_filter_.x[9]
            msg.pose.pose.orientation.y = self.kalman_filter_.x[10]
            msg.pose.pose.orientation.z = self.kalman_filter_.x[11]
            msg.pose.pose.orientation.w = self.kalman_filter_.x[12]

            msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
            msg.pose.covariance[1] = self.kalman_filter_.P[0][3]
            msg.pose.covariance[2] = self.kalman_filter_.P[0][6]
            msg.pose.covariance[6] = self.kalman_filter_.P[3][0]
            msg.pose.covariance[7] = self.kalman_filter_.P[3][3]
            msg.pose.covariance[8] = self.kalman_filter_.P[3][6]
            msg.pose.covariance[12] = self.kalman_filter_.P[6][0]
            msg.pose.covariance[13] = self.kalman_filter_.P[6][3]
            msg.pose.covariance[14] = self.kalman_filter_.P[6][6]

            # Sending the estimated velocity
            msg.twist.twist.linear.x = self.kalman_filter_.x[1]
            msg.twist.twist.linear.y = self.kalman_filter_.x[4]
            msg.twist.twist.linear.z = self.kalman_filter_.x[7]

            msg.twist.covariance[0] = self.kalman_filter_.P[1][1]
            msg.twist.covariance[1] = self.kalman_filter_.P[1][4]
            msg.twist.covariance[2] = self.kalman_filter_.P[1][7]
            msg.twist.covariance[6] = self.kalman_filter_.P[4][1]
            msg.twist.covariance[7] = self.kalman_filter_.P[4][4]
            msg.twist.covariance[8] = self.kalman_filter_.P[4][7]
            msg.twist.covariance[12] = self.kalman_filter_.P[7][1]
            msg.twist.covariance[13] = self.kalman_filter_.P[7][4]
            msg.twist.covariance[14] = self.kalman_filter_.P[7][7]

            self.odometry_publisher_.publish(msg)

        # Check which sensors are working
        active_sensor_list = []
        for sensor in self.sensor_wd_:
            if self.get_clock().now().nanoseconds - self.sensor_wd_[sensor] < IS_SENSOR_ALIVE_TIMEOUT*1e9:
                active_sensor_list.append(sensor)

        # Filter predict only if first iteration or new sensor data
        if active_sensor_list != [] or self.filter_state_ == "Offline":
            self.kalman_filter_.predict()

            # Log on sensor list change
            if (active_sensor_list != self.last_sensor_list_):
                self.get_logger().info(
                    f"Fusion using: {active_sensor_list}")
                self.last_sensor_list_ = active_sensor_list
        else:
            self.get_logger().warn(f"No data from sensors")

        # First predict just happened
        if(self.filter_state_ == "Offline"):
            self.filter_state_ = "Initialized"


def main(args=None):
    rclpy.init(args=args)
    node = UkfPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

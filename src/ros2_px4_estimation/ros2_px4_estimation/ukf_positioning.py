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
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros2_px4_interfaces.msg import UwbSensor


QUEUE_SIZE = 10
FILTER_DIM = 9  # Linear kinematic
IS_SENSOR_ALIVE_TIMEOUT = 5.  # s


class UkfPositioning(Node):
    """UKF: Try to explain this :O
    """

    def __init__(self):
        super().__init__("UkfPositioning")

        self.declare_parameters("", [
            ("delta_t", 0.05),
            ("q", 0.1),
            ("r_uwb", 0.05),
            ("r_gps", 1e-5)
        ])

        self.params_ = {
            x.name: x.value for x in self.get_parameters(
                ["delta_t", "q", "r_uwb", "r_gps"]
            )
        }

        self.filter_state_ = "Offline"
        self.calibration_counter_ = 0
        self.aligning_counter_ = 0
        self.anchor_offset_ = np.zeros(3)
        self.last_sensor_list_ = []
        self.sensor_wd_ = {
            "uwb": 0,
            "gps": 0,
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
            F = scipy.linalg.block_diag(*[f]*3)
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
                dim=3, dt=self.params_["delta_t"], var=self.params_["q"], block_size=3)
        )

        # Setting up sensors subscribers
        self.uwb_pos_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "UwbPositioning/Pose",
            self.callback_uwb_pos_subscriber, QUEUE_SIZE
        )
        self.uwb_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor_tag_0",
            self.callback_uwb_subscriber, QUEUE_SIZE
        )
        self.gps_subscriber_ = self.create_subscription(
            Odometry, "GpsPositioning/Odometry",
            self.callback_gps_subscriber, QUEUE_SIZE
        )

        # Setting up position and velocity publisher
        self.odometry_publisher_ = self.create_publisher(
            Odometry, "~/Odometry", QUEUE_SIZE
        )

        # Prediction timer
        self.timer = self.create_timer(
            self.params_["delta_t"], self.predict_callback)

        self.get_logger().info(f"Node has started: {self.params_}")

    def callback_uwb_pos_subscriber(self, msg):
        """Measuring UWB position offset estimate wtr of the navigation frame

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The UWB pose
        """

        # Deriving the anchor offset
        anchor_offset = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]) - self.kalman_filter_.x[[0, 3, 6]]

        # Mean values until convergence, than this subscriber can be destroyed
        var = (anchor_offset - self.anchor_offset_) / \
            (self.aligning_counter_ + 1)

        if np.linalg.norm(var) > 1e-3 or self.aligning_counter_ < 100.:
            self.anchor_offset_ += var
            self.aligning_counter_ += 1
        else:
            self.get_logger().info(f"""
            Anchor reference frame aligned in {self.aligning_counter_} iterations: 
            Offset = {self.anchor_offset_}
            """)
            self.destroy_subscription(self.uwb_pos_subscriber_)
            self.aligning_counter_ = -1

    def callback_uwb_subscriber(self, msg):
        """Measuring UWB range sensor

        Args:
            msg (ros2_px4_interfaces.msg.UwbSensor): The UWB message
        """

        # Must predict once first and anchor must be aligned
        if(self.filter_state_ == "Offline" or self.aligning_counter_ > -1):
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

        # Measurement model for a rotated range sensor
        def h_uwb(x):
            h = np.zeros(FILTER_DIM)

            # Range measurements
            h[0] = np.linalg.norm(
                x[[0, 3, 6]] - (anchor_position - self.anchor_offset_))
            return h

        # Filter update
        self.kalman_filter_.update(z, R=self.params_["r_uwb"], hx=h_uwb)

        # Gating
        x = self.kalman_filter_.x.copy()
        P = self.kalman_filter_.P.copy()
        if self.kalman_filter_.mahalanobis > 3. and self.filter_state_ == "Calibrated":
            self.get_logger().warn(
                f"Gating @: {self.kalman_filter_.mahalanobis}")
            self.kalman_filter_.x = x.copy()
            self.kalman_filter_.P = P.copy()

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
        R = np.eye(FILTER_DIM) * np.array([
            msg.pose.covariance[0],
            msg.pose.covariance[7],
            msg.pose.covariance[15],
            msg.twist.covariance[0],
            msg.twist.covariance[7],
            msg.twist.covariance[15], 1., 1., 1.
        ])

        self.kalman_filter_.update(z, R, hx=h_gps)

    def predict_callback(self):
        """This callback perform the filter predict and forward the current
        estimate
        """

        if (self.filter_state_ == "Calibrating"):
            if(np.linalg.norm(self.kalman_filter_.x - self.kalman_filter_.x_prior) < 1.):
                self.calibration_counter_ += 1

                if (self.calibration_counter_ > 100):
                    self.get_logger().info("Filter calibrated")
                    self.filter_state_ = "Calibrated"
                    self.calibration_counter_ = 0
            else:
                self.calibration_counter_ = 0

        # Send estimation only if calibrated
        if(self.filter_state_ == "Calibrated"):
            # Check covariance norm
            if(np.linalg.norm(self.kalman_filter_.P) > 100.):
                self.filter_state_ = "Diverged"
                self.get_logger().error("Filter is diverging")
                return

            # Sending the estimated odometry
            msg = Odometry()
            msg.header.frame_id = "UkfPositioning"
            msg.header.stamp = self.get_clock().now().to_msg()

            # Position
            msg.pose.pose.position.x = self.kalman_filter_.x[0]
            msg.pose.pose.position.y = self.kalman_filter_.x[3]
            msg.pose.pose.position.z = self.kalman_filter_.x[6]

            msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
            msg.pose.covariance[1] = self.kalman_filter_.P[3][3]
            msg.pose.covariance[2] = self.kalman_filter_.P[6][6]

            # Velocity
            msg.twist.twist.linear.x = self.kalman_filter_.x[1]
            msg.twist.twist.linear.y = self.kalman_filter_.x[4]
            msg.twist.twist.linear.z = self.kalman_filter_.x[7]

            msg.twist.covariance[0] = self.kalman_filter_.P[1][1]
            msg.twist.covariance[1] = self.kalman_filter_.P[4][4]
            msg.twist.covariance[2] = self.kalman_filter_.P[7][7]

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
            self.get_logger().info(f"Fusion using: {active_sensor_list}")
            self.last_sensor_list_ = active_sensor_list

            if (active_sensor_list == [] and self.filter_state_ == "Calibrated"):
                self.get_logger().error(f"Data fusion is not possible without data ^_^")
                self.filter_state_ == "Offline"

        # First data just arrived
        if(self.filter_state_ == "Initialized" and active_sensor_list != []):
            self.filter_state_ = "Calibrating"
            self.get_logger().info("Filter is calibrating")

        # First predict just happened
        if(self.filter_state_ == "Offline"):
            self.filter_state_ = "Initialized"
            self.get_logger().info("Filter initialized")


def main(args=None):
    rclpy.init(args=args)
    node = UkfPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

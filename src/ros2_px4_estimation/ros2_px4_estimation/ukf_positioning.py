#!/usr/bin/env python3

from math import radians
import rclpy
from rclpy.node import Node
import numpy as np
import scipy

from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from ros2_px4_interfaces.msg import UwbSensor

QUEUE_SIZE = 10
FILTER_DIM = 9


class UkfPositioning(Node):
    """
    UKF
    """

    def __init__(self):
        super().__init__("UkfPositioning")

        self.declare_parameter('deltaT', 0.1)
        self.declare_parameter('R_uwb', 1e-4)
        self.declare_parameter('Q', 1.0)

        self.deltaT_ = self.get_parameter(
            'deltaT').get_parameter_value().double_value
        self.R_uwb_ = self.get_parameter(
            'R_uwb').get_parameter_value().double_value
        self.Q_ = self.get_parameter('Q').get_parameter_value().double_value

        self.predicted_ = False

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
            dim_x=FILTER_DIM, dim_z=FILTER_DIM, dt=self.deltaT_, fx=f_, hx=None,
            points=sigmas
        )

        # Initial estimate @todo: put the home position
        self.kalman_filter_.x = np.zeros(FILTER_DIM)

        # Covariance matrix
        self.kalman_filter_.P *= 10.

        # Process noise
        self.kalman_filter_.Q = Q_discrete_white_noise(
            dim=3, dt=self.deltaT_, var=self.Q_, block_size=3)

        # Setting up sensors subscribers
        self.sensor_subscriber_ = self.create_subscription(
            UwbSensor, "/uwb_sensor/Iris", self.callback_uwb_subscriber,
            QUEUE_SIZE
        )
        self.gps_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "GpsPositioning/EstimatedPos",
            self.callback_gps_subscriber,
            QUEUE_SIZE
        )

        # Setting up position and velocity publisher
        self.est_pos_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "~/EstimatedPos",
            QUEUE_SIZE
        )
        self.est_vel_publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, "~/EstimatedVel",
            QUEUE_SIZE
        )

        # Prediction timer
        self.timer = self.create_timer(self.deltaT_, self.predict_callback)

        self.get_logger().info(f"""Node has started
                               deltaT:  {self.deltaT_}
                               R_uwb:   {self.R_uwb_}
                               Q:       {self.Q_}
                               """)

    def callback_uwb_subscriber(self, msg):
        """Measuring UWB range sensor

        Args:
            msg (ros2_px4_interfaces.msg.UwbSensor): The UWB message
        """

        # Must predict once first
        if(not self.predicted_):
            return

        # Storing measurement in a np.array
        z = np.array([msg.range, 0., 0., 0., 0., 0., 0., 0., 0.])

        # Storing anchor position in a np.array
        anchor_position = np.array([
            msg.anchor_pose.pose.position.x,
            msg.anchor_pose.pose.position.y,
            msg.anchor_pose.pose.position.z
        ])

        # Measurement model for a range sensor
        def h_uwb(x):
            r = np.linalg.norm(x[[0, 3, 6]] - anchor_position)
            return np.array([r, 0., 0., 0., 0., 0., 0., 0., 0.])

        # Filter update
        self.kalman_filter_.update(z, R=self.R_uwb_, hx=h_uwb)

    def callback_gps_subscriber(self, msg):
        """Measuring GPS positioning sensor

        Args:
            msg (geometry_msgs.msg.PoseWithCovarianceStamped): The GPS message
        """

        # Must predict once first
        if(not self.predicted_):
            return

        # Storing measurements in a np.array
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            0., 0., 0., 0., 0., 0.
        ])

        # Measurement model for a GPS sensor
        def h_gps(x):
            return np.array([x[0], x[3], x[6], 0., 0., 0., 0., 0., 0.])

        # Filter update
        self.kalman_filter_.update(z, R=self.R_uwb_, hx=h_gps)

    def predict_callback(self):
        """This callback perform the filter predict and forward the current
        estimate
        """

        # @todo: Check something wtr of covariance
        if(True):
            # Sending the estimated position
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "UkfPositioning"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.pose.position.x = self.kalman_filter_.x[0]
            msg.pose.pose.position.y = self.kalman_filter_.x[3]
            msg.pose.pose.position.z = self.kalman_filter_.x[6]

            msg.pose.covariance[0] = self.kalman_filter_.P[0][0]
            msg.pose.covariance[1] = self.kalman_filter_.P[0][3]
            msg.pose.covariance[2] = self.kalman_filter_.P[0][6]
            msg.pose.covariance[6] = self.kalman_filter_.P[3][0]
            msg.pose.covariance[7] = self.kalman_filter_.P[3][3]
            msg.pose.covariance[8] = self.kalman_filter_.P[3][6]
            msg.pose.covariance[12] = self.kalman_filter_.P[6][0]
            msg.pose.covariance[13] = self.kalman_filter_.P[6][3]
            msg.pose.covariance[14] = self.kalman_filter_.P[6][6]

            self.est_pos_publisher_.publish(msg)

            # Sending the estimated velocity
            msg = TwistWithCovarianceStamped()
            msg.header.frame_id = "UkfPositioning"
            msg.header.stamp = self.get_clock().now().to_msg()

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

            self.est_vel_publisher_.publish(msg)

        # Filter predict
        self.kalman_filter_.predict()
        self.predicted_ = True


def main(args=None):
    rclpy.init(args=args)
    node = UkfPositioning()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

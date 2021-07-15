#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ros2_px4_functions
import numpy as np

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from ros2_px4_interfaces.srv import ControlMode

# TO ADD: stop following if no uwb info arrive -> go in take off mode
# TO MODIFY: remove get logger spamming
# TO CHECK: what happens if one anchors or more miss

# Control parameters
KP = 1.1  # 1.15
KI = 0.01  # 0.03
KD = 0.01  # 0.008
INT_MAX = 250/(KI*100)
VMAX = 5.0
VMIN = - VMAX
LAND_ERR_TOLL = 0.3  # 0.2 Maximum position error allowed to perform landing
LAND_VEL_TOLL = 0.8  # 0.2 # Maximum velocity error allowed to perform landing
LAND_DESC_VEL = 0.5  # 0.2
LAND_H_TOLL = 0.9  # 0.85 # Turn off motors at this height
LAND_HOVERING_HEIGHT_SWITCH = 10.0
LAND_HOVERING_HEIGHT = 1.5
FOLLOW_HOVERING_HEIGHT = 3.0

dt = 0.1


class DroneController(Node):
    def __init__(self):
        super().__init__("offboard_control")

        # Initialization to 0 of all parameters
        self.int_e = np.array([0, 0])
        self.e_dot = np.array([0, 0])
        self.e_old = np.array([0, 0])
        self.e = []

        self.ARMING_STATE = 0
        self.LANDING_STATE = 0
        self.DESCENDING_STATE = 0
        self.TAKEOFF_STATE = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.target_global_pos = []
        self.target_local_pos = []
        self.target_uwb_local_relative_pos = []
        self.true_err = []
        self.timestamp = 0
        self.offboard_setpoint_counter_ = 0

        # Parameters declaration
        self.control_mode = self.declare_parameter("control_mode", 0)
        self.vehicle_namespace = self.declare_parameter(
            "vehicle_namespace", '')
        self.vehicle_number = self.declare_parameter("vehicle_number", 1)
        self.x0_ = self.declare_parameter("x0", 0.0)
        self.y0_ = self.declare_parameter("y0", 0.0)

        # Retrieve parameter values
        self.control_mode = self.get_parameter(
            "control_mode").get_parameter_value().integer_value
        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.vehicle_number = self.get_parameter(
            "vehicle_number").get_parameter_value().integer_value
        self.x0_ = self.get_parameter("x0").get_parameter_value().double_value
        self.y0_ = self.get_parameter("y0").get_parameter_value().double_value

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, self.vehicle_namespace + "/OffboardControlMode_PubSubTopic", 3)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, self.vehicle_namespace + "/TrajectorySetpoint_PubSubTopic", 3)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, self.vehicle_namespace + "/VehicleCommand_PubSubTopic", 3)

        # Subscribers
        self.drone_position_subscriber = self.create_subscription(
            VehicleLocalPosition, self.vehicle_namespace + "/VehicleLocalPosition_PubSubTopic", self.callback_local_position, 3)
        self.drone_status_subscriber = self.create_subscription(
            VehicleStatus, self.vehicle_namespace + "/VehicleStatus_PubSubTopic", self.callback_drone_status, 3)
        self.true_err_subscriber = self.create_subscription(
            Point, "/drone_vehicle_positioning_error/true_pos", self.callback_true_err, 3)
        self.timesync_sub_ = self.create_subscription(
            Timesync, self.vehicle_namespace + "/Timesync_PubSubTopic", self.callback_timesync, 3)
        self.target_uwb_position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, "/LS_uwb_estimator/estimated_pos", self.callback_target_uwb_position, 3)

        # Services
        self.control_mode_service = self.create_service(
            ControlMode, "control_mode", self.callback_control_mode)

        # Control loop timer
        self.timer = self.create_timer(dt, self.timer_callback)

    def callback_timesync(self, msg):
        self.timestamp = msg.timestamp

    def callback_local_position(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ >= 10):
            self.publish_vehicle_command(176, 1.0, 6.0)

            self.get_logger().info("arming..")
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if (self.offboard_setpoint_counter_ < 30):
            self.offboard_setpoint_counter_ += 1
        # elif (-self.z) < 0.5 and self.TAKEOFF_STATE == 0:
        #    self.restart_drone()

    def callback_drone_status(self, msg):
        self.ARMING_STATE = msg.arming_state

    def callback_target_uwb_position(self, msg):
        self.target_uwb_global_relative_pos = [
            msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.target_uwb_local_relative_pos = [
            -msg.pose.pose.position.y, -msg.pose.pose.position.x]
        self.e = np.array(self.target_uwb_local_relative_pos)

    def callback_true_err(self, msg):
        self.true_err_vec = [msg.x, msg.y]
        self.true_err = np.linalg.norm(self.true_err_vec, ord=2)

    def callback_control_mode(self, request, response):
        if request.control_mode == "takeoff_mode":
            self.reset_counters()
            self.control_mode = 1
        elif request.control_mode == "target_follower_mode":
            self.reset_counters()
            self.control_mode = 2
        elif request.control_mode == "land_on_target_mode":
            self.reset_counters()
            self.control_mode = 3
        elif request.control_mode == "setpoint_mode":
            self.reset_counters()
            self.control_mode = 4
            try:
                self.setpoint = [request.y-self.y0_,
                                 request.x-self.x0_, - request.z]
            except:
                response.success = "Please insert the coordinates"
                return response
        elif request.control_mode == "landing_mode":
            self.reset_counters()
            self.control_mode = 5
        elif request.control_mode == "restart_drone":
            self.reset_counters()
            self.control_mode = 6
        return response

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.vehicle_number
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(400, 1.0, 0.0)

    def disarm(self):
        self.publish_vehicle_command(400, 0.0, 1.0)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp

        if self.control_mode == 1:
            msg = self.takeoff_mode(msg)
        elif self.control_mode == 2:
            if self.e == []:
                self.get_logger().warn("Waiting for uwb positioning")
                msg = self.takeoff_mode(msg)
            else:
                msg = self.target_follower_mode(msg)
        elif self.control_mode == 3:
            if self.e == []:
                self.get_logger().warn("Waiting for uwb positioning")
                msg = self.takeoff_mode(msg)
            else:
                msg = self.land_on_target_mode(msg)
        elif self.control_mode == 4:
            msg = self.setpoint_mode(msg)
        elif self.control_mode == 5:
            self.landing_mode()
        elif self.control_mode == 6:
            self.restart_drone()
        else:
            msg = self.takeoff_mode(msg)

        self.trajectory_setpoint_publisher_.publish(msg)

    def land_on_target_mode(self, msg):
        msg.x = float("NaN")
        msg.y = float("NaN")

        if self.TAKEOFF_STATE == 0:
            if -self.z <= LAND_HOVERING_HEIGHT:
                msg.z = - LAND_HOVERING_HEIGHT - 0.5
                return msg
            else:
                self.TAKEOFF_STATE = 1

        [msg.vx, msg.vy], self.int_e, self.e_dot, self.e_old = ros2_px4_functions.PID(
            KP, KI, KD, self.e, self.e_old, self.int_e, VMAX, VMIN, INT_MAX, dt)
        self.norm_e = np.linalg.norm(self.e, ord=2)
        self.norm_e_dot = np.linalg.norm(self.e_dot, ord=2)

        if ((self.norm_e) < LAND_ERR_TOLL) and ((self.norm_e_dot) < LAND_VEL_TOLL) and (-self.z < LAND_H_TOLL):
            self.LANDING_STATE = 1
            self.get_logger().info("Landing..")
            self.publish_vehicle_command(185, 1.0, 0.0)
        if ((self.norm_e) < LAND_ERR_TOLL) and ((self.norm_e_dot) < LAND_VEL_TOLL) and self.LANDING_STATE == 0:
            self.DESCENDING_STATE = 1
            self.get_logger().info("Descending on target..")
            msg.z = float("NaN")
            msg.vz = LAND_DESC_VEL
        elif self.LANDING_STATE == 0 and self.DESCENDING_STATE == 0 and self.norm_e < LAND_HOVERING_HEIGHT_SWITCH:
            self.get_logger().info("Following target..")
            msg.z = - LAND_HOVERING_HEIGHT
            msg.vz = float("NaN")
        elif self.LANDING_STATE == 0 and self.DESCENDING_STATE == 0:
            self.get_logger().info("Following target..")
            msg.z = - FOLLOW_HOVERING_HEIGHT
            msg.vz = float("NaN")
        elif self.LANDING_STATE == 0:
            self.get_logger().info("Stopped descending..")
            msg.z = - LAND_HOVERING_HEIGHT
            msg.vz = - 0.05
        if self.ARMING_STATE == 1:
            self.get_logger().info(f"""Landed
                estimated err: {self.norm_e}
                true err: {self.true_err}""")
            rclpy.shutdown()

        return msg

    def target_follower_mode(self, msg):
        msg.x = float("NaN")
        msg.y = float("NaN")
        msg.z = - FOLLOW_HOVERING_HEIGHT
        if self.TAKEOFF_STATE == 0:
            if -self.z <= LAND_HOVERING_HEIGHT:
                msg.z = - LAND_HOVERING_HEIGHT - 0.5
                return msg
            else:
                self.TAKEOFF_STATE = 1

        [msg.vx, msg.vy], self.int_e, self.e_dot, self.e_old = ros2_px4_functions.PID(
            KP, KI, KD, self.e, self.e_old, self.int_e, VMAX, VMIN, INT_MAX, dt)
        self.norm_e = np.linalg.norm(self.e, ord=2)
        self.norm_e_dot = np.linalg.norm(self.e_dot, ord=2)
        self.get_logger().info("Following target..")

        return msg

    def setpoint_mode(self, msg):
        msg.x = self.setpoint[0]
        msg.y = self.setpoint[1]
        msg.z = self.setpoint[2]
        self.get_logger().info("Reaching setpoint..")
        return msg

    def takeoff_mode(self, msg):
        self.get_logger().info("Takeoff..")
        msg.x = float("NaN")
        msg.y = float("NaN")
        msg.z = -2.5
        return msg

    def landing_mode(self):
        self.get_logger().info("Landing..")
        self.publish_vehicle_command(21, 0.0, 0.0)

    def restart_drone(self):
        self.reset_counters()
        self.offboard_setpoint_counter_ = 0
        self.control_mode = 1

    def reset_counters(self):
        self.LANDING_STATE = 0
        self.TAKEOFF_STATE = 0
        self.DESCENDING_STATE = 0


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

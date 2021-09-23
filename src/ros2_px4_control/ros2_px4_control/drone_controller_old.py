#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ros2_px4_functions
import numpy as np
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from ros2_px4_interfaces.srv import ControlMode

# TO ADD: stop following if no uwb info arrive -> go in take off mode
# TO MODIFY: remove get logger spamming

# Control parameters
KP = 0.8 # 1.3
KI = 0.05 # 0.1
KD = 0.2 # 0.05
POS_GAIN_SWITCH = 3.0
V_MAX = 1.3

LAND_ERR_TOLL = 0.4 #0.3   # Maximum XY position error allowed to perform landing
LAND_VEL_TOLL = 0.4 #0.3   # Maximum XY velocity error allowed to perform landing
LAND_DESC_VEL = 0.5 #0.5   # Z velocity when descending on target
TURN_OFF_MOT_HEIGHT = 0.25  # Turn off motors at this height (wrt platform)
LAND_HOVERING_HEIGHT_XY_THRESH = 10.0
LAND_HOVERING_HEIGHT = 1.5 #2.0
FOLLOW_HOVERING_HEIGHT = 1.5 #2.0
TAKEOFF_HOVERING_HEIGHT = 1.5 #2.0

dT_ = 0.1



class DroneController(Node):
    def __init__(self):
        super().__init__("offboard_control")

        # Initialization to 0 of all parameters
        self.e_dot = []
        self.e = []
        self.RESET_INT = False
        
        self.ARMING_STATE = 0
        
        self.vx = []
        self.vy = []
        self.vz = []
        self.z = []
        self.z_dist_sensor = []
        self.estimated_rel_target_pos = []

        self.timestamp = 0
        self.offboard_setpoint_counter_ = 0

        self.watchdog_counter = np.zeros((3,1))
        self.watchdog_dT_ = 1.0

        self.PID_1 = ros2_px4_functions.PID_controller(
            KP, KI, KD, V_MAX, 2, True, dT_)
        self.PID_2 = ros2_px4_functions.PID_controller(
            KP, 0.0, 0.0, V_MAX, 2, True, dT_)

        # Parameters declaration
        self.control_mode = self.declare_parameter("control_mode", 1)
        self.vehicle_namespace = self.declare_parameter(
            "vehicle_namespace", "/drone")
        self.vehicle_number = self.declare_parameter("vehicle_number", 2)
        self.uwb_estimator = self.declare_parameter("uwb_estimator", "/KF_pos_estimator_0")
        self.rng_sensor_topic = self.declare_parameter("rng_sensor_topic", "/range_sensor_positioning/estimated_pos")

        # Retrieve parameter values
        self.control_mode = self.get_parameter(
            "control_mode").get_parameter_value().integer_value
        self.vehicle_namespace = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.vehicle_number = self.get_parameter(
            "vehicle_number").get_parameter_value().integer_value
        self.uwb_estimator = self.get_parameter(
            "uwb_estimator").get_parameter_value().string_value
        self.rng_sensor_topic = self.get_parameter(
            "rng_sensor_topic").get_parameter_value().string_value

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
        self.timesync_sub_ = self.create_subscription(
            Timesync, self.vehicle_namespace + "/Timesync_PubSubTopic", self.callback_timesync, 3)
        self.target_uwb_position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, self.uwb_estimator + "/estimated_pos", self.callback_target_uwb_position, 3)
        self.target_uwb_position_subscriber = self.create_subscription(
            TwistWithCovarianceStamped, self.uwb_estimator + "/estimated_vel", self.callback_target_uwb_velocity, 3)

        self.drone_position_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, self.rng_sensor_topic, self.callback_distance_sensor, 3)


        # Services
        self.control_mode_service = self.create_service(
            ControlMode, "control_mode", self.callback_control_mode)

        # Control loop timer
        self.timer = self.create_timer(dT_, self.timer_callback)

        # Watchdog timer
        self.watchdog_timer = self.create_timer(self.watchdog_dT_, self.watchdog_callback)


    def callback_timesync(self, msg):
        self.timestamp = msg.timestamp

    def callback_drone_status(self, msg):
        self.ARMING_STATE = msg.arming_state

    def callback_local_position(self, msg):
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        
        self.watchdog_counter[0][0] += 1

    def callback_distance_sensor(self,msg):
        self.z_dist_sensor = msg.pose.pose.position.z

    def callback_target_uwb_position(self, msg):
        # From rover ENU frame to drone NED frame
        self.estimated_rel_target_pos = [
            - msg.pose.pose.position.y, - msg.pose.pose.position.x]
        self.e = np.array(self.estimated_rel_target_pos)

        self.watchdog_counter[1][0] += 1

    def callback_target_uwb_velocity(self, msg):
        # From rover ENU frame to drone NED frame
        self.estimated_rel_target_vel = [
            - msg.twist.twist.linear.y, - msg.twist.twist.linear.x]
        self.e_dot = np.array(self.estimated_rel_target_vel)
        
        self.watchdog_counter[2][0] += 1

    def callback_control_mode(self, request, response):
        if request.control_mode == "takeoff_mode":
            self.control_mode = 1
        elif request.control_mode == "target_follower_mode":
            if self.e == []:
                self.get_logger().warn("No positioning data available, switching to takeoff")
            else:
                self.control_mode = 2
        elif request.control_mode == "land_on_target_mode":
            if self.e == []:
                self.get_logger().warn("No positioning data available, switching to takeoff")
            else:
                self.control_mode = 3
        return response

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ >= 10):
            self.publish_vehicle_command(176, 1.0, 6.0)
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if (self.offboard_setpoint_counter_ < 30):
            self.offboard_setpoint_counter_ += 1

    def watchdog_callback(self):

        if (self.control_mode == 2 or self.control_mode == 3) and any(self.watchdog_counter == 0):
            self.control_mode = 1
            self.get_logger().warn("No positioning data available, switching to takeoff")
        
        self.watchdog_counter = np.zeros((3,1))



    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp

        if self.control_mode == 1:
            msg = self.takeoff_mode(msg)
        elif self.control_mode == 2:
            if self.e == []:
                msg = self.takeoff_mode(msg)
            else:
                msg = self.target_follower_mode(msg)
        elif self.control_mode == 3:
            if self.e == []:
                msg = self.takeoff_mode(msg)
            else:
                msg = self.land_on_target_mode(msg)
        else:
            msg = self.takeoff_mode(msg)

        self.trajectory_setpoint_publisher_.publish(msg)

    def land_on_target_mode(self, msg):
        
        msg.x = float("NaN")
        msg.y = float("NaN")


        self.norm_e = np.linalg.norm(self.e, ord=2)
        self.norm_e_dot = np.linalg.norm(self.e_dot, ord=2)

        if (self.norm_e) < POS_GAIN_SWITCH:
            [msg.vx, msg.vy], _, _ = self.PID_1.PID(self.e, self.e_dot, [self.vx, self.vy], self.RESET_INT)
            self.RESET_INT = False
        else:
            [msg.vx, msg.vy], _, _ = self.PID_2.PID(self.e, self.e_dot, [self.vx, self.vy], self.RESET_INT)
            self.RESET_INT = True


        #if self.ARMING_STATE == 1:
        #    self.get_logger().info(f"""
        #    Landed with:
        #    error norm: {self.norm_e}, 
        #    ex: {self.e[0]}, 
        #    ey: {self.e[1]}.""")
        #    rclpy.shutdown()


        #elif ((self.norm_e) < LAND_ERR_TOLL) and ((self.norm_e_dot) < LAND_VEL_TOLL) and (self.z_dist_sensor <= TURN_OFF_MOT_HEIGHT):
        #     self.publish_vehicle_command(185, 1.0, 0.0)
        
        if ((self.norm_e) < LAND_ERR_TOLL) and ((self.norm_e_dot) < LAND_VEL_TOLL):
            msg.z = float("NaN")
            msg.vz = LAND_DESC_VEL

        elif self.norm_e < LAND_HOVERING_HEIGHT_XY_THRESH:
            msg.z = - LAND_HOVERING_HEIGHT
            msg.vz = float("NaN")

        else:
            msg.z = - FOLLOW_HOVERING_HEIGHT
            msg.vz = float("NaN")
            
        return msg

    def target_follower_mode(self, msg):
        msg.x = float("NaN")
        msg.y = float("NaN")
        msg.z = - FOLLOW_HOVERING_HEIGHT

        self.norm_e = np.linalg.norm(self.e, ord=2)
        self.norm_e_dot = np.linalg.norm(self.e_dot, ord=2)

        if (self.norm_e) < POS_GAIN_SWITCH:
            [msg.vx, msg.vy], _, _ = self.PID_1.PID(self.e, self.e_dot, [self.vx, self.vy], self.RESET_INT)
            self.RESET_INT = False
        else:
            [msg.vx, msg.vy], _, _ = self.PID_2.PID(self.e, self.e_dot, [self.vx, self.vy], self.RESET_INT)
            self.RESET_INT = True

        return msg

    def takeoff_mode(self, msg):
        msg.x = float("NaN")
        msg.y = float("NaN")
        msg.z = - TAKEOFF_HOVERING_HEIGHT
        return msg


    def arm(self):
        self.publish_vehicle_command(400, 1.0, 0.0)

    def disarm(self):
        self.publish_vehicle_command(400, 0.0, 1.0)


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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

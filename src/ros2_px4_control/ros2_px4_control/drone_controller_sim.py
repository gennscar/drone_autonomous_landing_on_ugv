#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, \
    VehicleCommand, VehicleLocalPosition, VehicleStatus, DistanceSensor
from ros2_px4_interfaces.srv import ControlMode
import ros2_px4_functions
from sensor_msgs.msg import Range

# Parameters
OFFBOARD_DT = 0.1              # Offboard control period
MIN_HEIGHT = 1.0               # Minimum height accepted by the ControlMode service
MAX_HEIGHT = 10.0              # Maximum height accepted by the ControlMode service
STD_HEIGHT = 3.0               # Standard height used if out of bound

# Autonomous control parameters
PID_SWITCH_POS = 5.0           # Distance from target to switch from PID1 to PID2
V_MAX_INT = 1.2                # Anti windup drone velocity limit
KP = 0.8                       # Proportional gain
KI = 0.08                      # Integral gain
KD = 0.1                       # Derivative gain
LAND_ERR_TOLL = 0.3           # XY relative position allowed to shut down motors when on platform
HEIGHT_SWITCH_TOLL = 0.7       # Height to switch from descending xy tolerance cone to cylinder
GAIN_HEIGHT_TOLL = 0.25        # XY meters error increase with respect to height
LAND_VEL_TOLL = 0.5            # Maximum XY relative velocity allowed to perform landing
LAND_DESC_VEL = - 0.3         # Z velocity when descending on target
TARGET_HEIGHT = 0.5            # Target height
TURN_OFF_MOT_HEIGHT = 0.3     # Relative height allowed to shutdown motors
DETECT_LANDING_COUNT = 1       # Landing conditions verified consecutively for this numer of times
PREDICTION_TIME = 0.0          # If >0, the drone will land at the predicted rover position
FOLLOW_HOVERING_HEIGHT = 3.0   # Drone height kept in target follower mode
WATCHDOG_DT = 0.5              # Watchdog period


# NULL definition and check function
NULL = float("NaN")


class DroneController(Node):
    """This node is needed to interface with the Offboard Control of PX4.

    Args:
        control_mode (string): starting control mode.
        vehicle_number (string): number of the vehicle to control (only needed
        on simulation).
    """

    def __init__(self):
        super().__init__("DroneController")

        # Controller state name map to controllers function
        self.CONTROLLERS = {
            "idle": self.idle_controller,
            "takeoff": self.takeoff_controller,
            "land": self.land_controller,
            "setpoint": self.setpoint_controller,
            "follow_target": self.target_follower_controller,
            "land_on_target": self.land_on_target_controller
        }

        # Control state
        self.control_mode_old_ = "none"
        self.sent_offboard_ = 0
        self.local_position_ = [NULL]*3
        self.setpoint_ = [NULL]*3

        # Drone state
        self.timestamp_ = 0
        self.arming_state_ = False
        self.disarm_reason_ = -1
        self.takeoff_state_ = False

        # Autonomous state
        self.rel_pos_ = []
        self.rel_vel_ = []
        self.distance_sensor_height_ = []
        self.reset_int_ = True
        self.PID_1 = ros2_px4_functions.PID_controller(
            KP, KI, KD, V_MAX_INT, 2, True, OFFBOARD_DT)
        self.PID_2 = ros2_px4_functions.PID_controller(
            KP, 0.0, 0.0, V_MAX_INT, 2, True, OFFBOARD_DT)
        self.watchdog_counter_ = 0
        self.detect_landing_counter_ = 0

        # Parameters declaration
        self.control_mode_ = self.declare_parameter("control_mode", "idle")
        self.vehicle_number_ = self.declare_parameter("vehicle_number", 1)

        # Retrieve parameter values
        self.control_mode_ = self.get_parameter(
            "control_mode").get_parameter_value().string_value
        self.vehicle_number_ = self.get_parameter(
            "vehicle_number").get_parameter_value().integer_value

        # Publishers
        self.offboard_control_mode_pub_ = self.create_publisher(
            OffboardControlMode, "OffboardControlMode_PubSubTopic", 1)
        self.trajectory_setpoint_pub_ = self.create_publisher(
            TrajectorySetpoint, "TrajectorySetpoint_PubSubTopic", 1)
        self.vehicle_command_pub_ = self.create_publisher(
            VehicleCommand, "VehicleCommand_PubSubTopic", 1)

        # Subscribers
        self.drone_status_sub_ = self.create_subscription(
            VehicleStatus, "VehicleStatus_PubSubTopic", self.callback_drone_status, 1)
        self.timesync_sub_ = self.create_subscription(
            Timesync, "Timesync_PubSubTopic", self.callback_timesync, 1)
        self.drone_position_sub_ = self.create_subscription(
            VehicleLocalPosition, "VehicleLocalPosition_PubSubTopic", self.callback_local_position, 1)
        self.target_uwb_position_sub_ = self.create_subscription(
            Odometry, "/KF_pos_estimator_0/estimated_pos", self.callback_target_uwb_position, 3)
        self.distance_sensor_sub_= self.create_subscription(
            Range, "DistanceSensor_PubSubTopic", self.callback_distance_sensor, 1)
        # Services
        self.control_mode_service = self.create_service(
            ControlMode, "ControlMode_Service", self.callback_control_mode)

        # Control loop timer
        self.timer = self.create_timer(OFFBOARD_DT, self.callback_timer)

        # Watchdog timer
        self.watchdog_timer = self.create_timer(WATCHDOG_DT, self.watchdog_callback)

        self.get_logger().info("Node has started")

    def callback_timesync(self, msg):
        """This callback retrieve the timesync value from PX4.

        Args:
            msg (px4_msgs.msg.Timesync): Timesync message.
        """
        self.timestamp_ = msg.timestamp

    def callback_drone_status(self, msg):
        """This callback retrieve the drone state from PX4.

        Args:
            msg (px4_msgs.msg.VehicleStatus): VehicleStatus message,
        """
        self.arming_state_ = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.disarm_reason_ = msg._latest_disarming_reason
        self.takeoff_state_ = msg.takeoff_time > 0

    def callback_local_position(self, msg):
        """This callback retrieve the current local position from PX4 and
        convert it in ENU frame.

        Args:
            msg (px4_msgs.msg.VehicleLocalPosition): VehicleLocalPosition
            message
        """
        self.local_position_ = [msg.y, msg.x, -msg.z]
        self.local_velocity_ = [msg.vy, msg.vx, -msg.vz]

    def callback_target_uwb_position(self, msg):
        """This callback retrieves the current relative position between drone and
        rover and converts it in ENU frame.

        Args:
            msg (nav_msgs.msg.Odometry): Odometry
            message
        """
        # From rover NED frame to drone ENU frame
        self.rel_pos_ = np.array([
            - msg.pose.pose.position.y, - msg.pose.pose.position.x])
        self.rel_z_ = msg.pose.pose.position.z
        self.rel_vel_ = np.array([
            - msg.twist.twist.linear.y, - msg.twist.twist.linear.x])
        self.rover_vel_ = np.array([
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.x])
        self.rel_pos_ = self.rel_pos_ + PREDICTION_TIME*self.rover_vel_

        # Increment watchdog counter if uwb info received
        self.watchdog_counter_ += 1

    def callback_distance_sensor(self, msg):
        """This callback retrieves the distance sensor height value
        Args:
            msg (px4_msgs.msg.DistanceSensor_PubSubTopic): DistanceSensor
            message
        """
        self.distance_sensor_height_ = msg.range

    def callback_timer(self):
        """This callback send the required messages to enact the offboard
        control.
        """
        # Check synchronization
        if self.timestamp_ == 0:
            self.get_logger().warn("Not synched")
            return

        # Enable Offboard when at least 10 messages are already published
        if self.sent_offboard_ == 10:
            self.publish_vehicle_command(176, 1.0, 6.0)

        # Call the right controller
        self.CONTROLLERS.get(self.control_mode_, self.idle_controller)()

        # Log only on state change
        if self.control_mode_ != self.control_mode_old_:
            self.get_logger().info(f"""Using {self.control_mode_} mode""")

        self.control_mode_old_ = self.control_mode_

    def callback_control_mode(self, request, response):
        """This callback act the control mode requested by the /control_mode
        service.

        Args:
          request (dict): structure containing the request received.
          response (dict): structure containing the response to send.
        """

        # Idle cannot be manually requested
        if request.control_mode == "idle":
            self.get_logger().warn("""Cannot reach idle manually""")
            request.control_mode = "land"

        # Check if the control mode is valid
        if request.control_mode in self.CONTROLLERS.keys():
            self.control_mode_ = request.control_mode
            response.message = "Changing control mode into " + self.control_mode_
        else:
            response.message = "Control mode " + request.control_mode + " does not exists"
            self.get_logger().warn(
                """Control mode {request.control_mode} does not exists"""
            )

        # Try to retrieve the setpoint from the request
        try:
            self.setpoint_ = [
                request.x,
                request.y,
                request.z,
                request.vx,
                request.vy,
                request.vz,
                request.yaw
            ]
        except:
            pass

        return response

    def idle_controller(self):
        """Controller for idle mode: it reset the state and memorize the last
        position as a reference.
        """

        # Resetting everything
        self.disarm_reason_ = -1
        self.sent_offboard_ = 0
        self.reset_int_ = True
        self.detect_landing_counter_ = 0

        # Saving last idle position
        self.start_local_position_ = self.local_position_

    def takeoff_controller(self):
        """Controller for takeoff mode: check the height and perform the takeoff
        """

        self.check_height()

        self.arm()
        self.offboard([
            self.start_local_position_[0],
            self.start_local_position_[1],
            self.start_local_position_[2] + self.setpoint_[2]
        ])

    def land_controller(self):
        """Controller for land mode: send the landing message and stop the
        offboard control
        """

        if not self.arming_state_:
            self.get_logger().info("Drone not armed, switch to idle")
            self.control_mode_ = "idle"
            return

        self.offboard()
        self.publish_vehicle_command(21, 0.0, 0.0)

        # Switch to idle to interrupt the offboard
        self.control_mode_ = "idle"

    def setpoint_controller(self):
        """Controller for setpoint mode: send the request setpoint
        """

        self.check_height()

        self.arm()
        self.offboard(
            position=[
                self.start_local_position_[0] + self.setpoint_[0],
                self.start_local_position_[1] + self.setpoint_[1],
                self.start_local_position_[2] + self.setpoint_[2]
            ],
            velocity=[
                self.setpoint_[3],
                self.setpoint_[4],
                self.setpoint_[5]
            ],
            yaw=self.setpoint_[6]
        )

    def target_follower_controller(self):
        """Controller for target landing mode: sends xy velocity outputs proportional 
        to the relative distance between drone and rover.
        """

        if self.rel_pos_ == [] or self.rel_vel_ == []:
            x_setpoint_ = NULL
            y_setpoint_ = NULL
            z_setpoint_ = self.start_local_position_[2] + FOLLOW_HOVERING_HEIGHT
            vx_setpoint_ = 0.0
            vy_setpoint_ = 0.0
            vz_setpoint_ = NULL
            yaw_setpoint_ = NULL
            
        else:
            x_setpoint_ = NULL
            y_setpoint_ = NULL
            z_setpoint_ = self.start_local_position_[2] + FOLLOW_HOVERING_HEIGHT
            vz_setpoint_ = NULL
            yaw_setpoint_ = NULL

            self.norm_rel_pos_ = np.linalg.norm(self.rel_pos_, ord=2)
            self.norm_rel_vel_ = np.linalg.norm(self.rel_vel_, ord=2)

            if (self.norm_rel_pos_) < PID_SWITCH_POS:
                [vx_setpoint_, vy_setpoint_], _, _ = self.PID_1.PID(self.rel_pos_, self.rel_vel_, [self.local_velocity_[0], self.local_velocity_[1]], self.reset_int_)
                self.reset_int_ = False
            else:
                [vx_setpoint_, vy_setpoint_], _, _ = self.PID_2.PID(self.rel_pos_, self.rel_vel_, [self.local_velocity_[0], self.local_velocity_[1]], self.reset_int_)
                self.reset_int_ = True

        self.arm()
        self.offboard(
            position=[
                x_setpoint_,
                y_setpoint_,
                z_setpoint_
            ],
            velocity=[
                vx_setpoint_,
                vy_setpoint_,
                vz_setpoint_
            ],
            yaw = yaw_setpoint_
        )

    def land_on_target_controller(self):
        """Controller for target landing mode: sends setpoints according to a state machine
        and xy velocity outputs proportional to the relative distance between drone and rover.
        """

        if self.rel_pos_ == [] or self.rel_vel_ == []:
            x_setpoint_ = NULL
            y_setpoint_ = NULL
            z_setpoint_ = float("NaN")
            vx_setpoint_ = 0.0
            vy_setpoint_ = 0.0
            vz_setpoint_ = 0.0
            yaw_setpoint_ = NULL
            
        else:
            x_setpoint_ = NULL
            y_setpoint_ = NULL
            yaw_setpoint_ = NULL
            z_setpoint_ = float("NaN")
            vz_setpoint_ = 0.0

            self.norm_rel_pos_ = np.linalg.norm(self.rel_pos_, ord=2)
            self.norm_rel_vel_ = np.linalg.norm(self.rel_vel_, ord=2)

            if (self.norm_rel_pos_) < PID_SWITCH_POS:
                [vx_setpoint_, vy_setpoint_], _, _ = self.PID_1.PID(self.rel_pos_, self.rel_vel_, [self.local_velocity_[0], self.local_velocity_[1]], self.reset_int_)
                self.reset_int_ = False
            else:
                [vx_setpoint_, vy_setpoint_], _, _ = self.PID_2.PID(self.rel_pos_, self.rel_vel_, [self.local_velocity_[0], self.local_velocity_[1]], self.reset_int_)
                self.reset_int_ = True
            

            if (self.local_position_[2] >= HEIGHT_SWITCH_TOLL + TARGET_HEIGHT):
                allowed_desc_range_ = LAND_ERR_TOLL + (self.local_position_[2] - HEIGHT_SWITCH_TOLL - TARGET_HEIGHT)*GAIN_HEIGHT_TOLL
            else:
                allowed_desc_range_ = LAND_ERR_TOLL

            if ((self.norm_rel_pos_) <= allowed_desc_range_ and (self.norm_rel_vel_) <= LAND_VEL_TOLL):
                z_setpoint_ = float("NaN")
                vz_setpoint_ = LAND_DESC_VEL
                if self.local_position_[2] < TURN_OFF_MOT_HEIGHT + TARGET_HEIGHT:
                    self.detect_landing_counter_ += 1
                    if self.detect_landing_counter_ == DETECT_LANDING_COUNT:
                        self.control_mode_ = "land"
                        self.detect_landing_counter_ = 0
                else:
                    self.detect_landing_counter_ = 0

                
        self.arm()
        self.offboard(
            position=[
                x_setpoint_,
                y_setpoint_,
                z_setpoint_
            ],
            velocity=[
                vx_setpoint_,
                vy_setpoint_,
                vz_setpoint_
            ],
            yaw = yaw_setpoint_
        )

    def watchdog_callback(self):
        """Stop the drone if no uwb information arrives for WATCHDOG_DT time.
        """
        if self.control_mode_ != "follow_target" and self.control_mode_ != "land_on_target":
            return
        if self.watchdog_counter_ == 0:
            self.get_logger().info("No target position, waiting..")
            self.rel_pos_ = []
            self.rel_vel_ = []
        
        self.watchdog_counter_ = 0

    def check_height(self):
        """Check if the requested height is in the boundaries or set the
        standard one.
        """

        if self.setpoint_[2] == NULL or self.setpoint_[2] < MIN_HEIGHT or self.setpoint_[2] > MAX_HEIGHT:
            self.get_logger().warn(f"""
                                   Setting standard height of {STD_HEIGHT}, instead of {self.setpoint_[2]}. Check bounds.
                                   """)
            self.setpoint_[2] = STD_HEIGHT

    def publish_vehicle_command(self, command, param1, param2):
        """This function publish the VehicleCommand to PX4.

        Args:
            command (int): ID code of the command.
            param1 (any): first positional parameter.
            param2 (any): second positional parameter.
        """

        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.vehicle_number_
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub_.publish(msg)

    def arm(self):
        """Arming routine and check
        """

        # Arm if disarmed
        if not self.arming_state_:
            # Check if disarmed by RC
            if self.disarm_reason_ == VehicleStatus.ARM_DISARM_REASON_RC_STICK or \
                    self.disarm_reason_ == VehicleStatus.ARM_DISARM_REASON_RC_SWITCH:
                self.get_logger().info(
                    f"""Disarmed by RC, cannot arm""")
                self.control_mode_ = "idle"
                return

            self.get_logger().info("Arming")
            self.publish_vehicle_command(400, 1.0, 0.0)

    def disarm(self):
        """Disarming routine and check
        """

        # Disarm if armed
        if self.arming_state_:
            self.get_logger().info("Disarming")
            self.publish_vehicle_command(400, 0.0, 1.0)

    def offboard(self, position=[NULL]*3, velocity=[NULL]*3, yaw=NULL):
        """This function send the requested offboard command

        Args:
            position (list of 3 float, optional): The requested position.
            Defaults to [NULL]*3.
            velocity (list of 3 float, optional): The requested velocity.
            Defaults to [NULL]*3.
            yaw (float, optional): The requested yaw.
            Defaults to NULL.
        """

        # Counting offboard command to initiate offboard control
        self.sent_offboard_ += 1

        # Setting up offboard control mode
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False

        self.offboard_control_mode_pub_.publish(msg)

        # Setting up requested setpoint
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_

        # Conversion from ENU to NED
        msg.x = position[1]
        msg.y = position[0]
        msg.z = -position[2]
        msg.vx = velocity[1]
        msg.vy = velocity[0]
        msg.vz = -velocity[2]
        msg.yaw = yaw

        self.trajectory_setpoint_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

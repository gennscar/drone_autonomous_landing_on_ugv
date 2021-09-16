#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from ros2_px4_interfaces.srv import ControlMode

# Parameters
OFFBOARD_DT = 0.1   # Offboard control period
MIN_HEIGHT = 1.0    # Minimum height accepted by the ControlMode service
MAX_HEIGHT = 10.0   # Maximum height accepted by the ControlMode service
STD_HEIGHT = 3.0    # Standard height used if out of bound


# NULL definition and check function

NULL = float("NaN")


def check_null(list):
    ret = []
    for x in list:
        ret.append(x == NULL)
    return ret


class DroneController(Node):
    """This node is needed to interface with the Offboard Control of PX4.

    Args:
        control_mode (string): starting control mode.
        vehicle_namespace (string): namespace of the vehicle to control.
        vehicle_number (string): number of the vehicle to control (only needed
        on simulation).
    """

    def __init__(self):
        super().__init__("drone_controller")

        # Controller state name map to controllers function
        self.CONTROLLERS = {
            "idle": self.idle_controller,
            "takeoff": self.takeoff_controller,
            "land": self.land_controller,
            "setpoint": self.setpoint_controller
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

        # Parameters declaration
        self.control_mode_ = self.declare_parameter("control_mode", "idle")
        self.vehicle_namespace_ = self.declare_parameter(
            "vehicle_namespace", '')
        self.vehicle_number_ = self.declare_parameter("vehicle_number", 1)

        # Retrieve parameter values
        self.control_mode_ = self.get_parameter(
            "control_mode").get_parameter_value().string_value
        self.vehicle_namespace_ = self.get_parameter(
            "vehicle_namespace").get_parameter_value().string_value
        self.vehicle_number_ = self.get_parameter(
            "vehicle_number").get_parameter_value().integer_value

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, self.vehicle_namespace_ + "/OffboardControlMode_PubSubTopic", 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, self.vehicle_namespace_ + "/TrajectorySetpoint_PubSubTopic", 1)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, self.vehicle_namespace_ + "/VehicleCommand_PubSubTopic", 1)

        # Subscribers
        self.drone_status_subscriber = self.create_subscription(
            VehicleStatus, self.vehicle_namespace_ + "/VehicleStatus_PubSubTopic", self.callback_drone_status, 1)
        self.timesync_sub_ = self.create_subscription(
            Timesync, self.vehicle_namespace_ + "/Timesync_PubSubTopic", self.callback_timesync, 1)
        self.drone_position_subscriber = self.create_subscription(
            VehicleLocalPosition, self.vehicle_namespace_ + "/VehicleLocalPosition_PubSubTopic", self.callback_local_position, 1)

        # Services
        self.control_mode_service = self.create_service(
            ControlMode, self.vehicle_namespace_ + "/control_mode", self.callback_control_mode)

        # Control loop timer
        self.timer = self.create_timer(OFFBOARD_DT, self.timer_callback)

    def callback_timesync(self, msg):
        """This callback retrieve the timesync value from PX4.

        Args:
            msg (px4_msgs.msg.Timesync): Tymesync message.
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

    def timer_callback(self):
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
          response (dict): structure contatining the response to send.
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
            self.setpoint_ = [request.x, request.y, request.z]
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
        """Controller for land mode: send the landind message and stop the
        offboarf control
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

        # Sanitizing setpoint input
        if any(check_null(self.setpoint_)):
            self.get_logger().warn(f"""Setpoint not valid: {self.setpoint_}""")
            self.control_mode_ = "land"
            return

        self.check_height()

        self.arm()
        self.offboard([
            self.start_local_position_[0] + self.setpoint_[0],
            self.start_local_position_[1] + self.setpoint_[1],
            self.start_local_position_[2] + self.setpoint_[2]
        ])

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

        self.vehicle_command_publisher_.publish(msg)

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

    def offboard(self, position=[NULL]*3, velocity=[NULL]*3):
        """This function send the requested offboard command

        Args:
            position (list of 3 float, optional): The requested position.
            Defaults to [NULL]*3.
            velocity (list of 3 float, optional): The requested velocity.
            Defaults to [NULL]*3.
        """

        # Counting offboard command to initiate offboard control
        self.sent_offboard_ += 1

        # Setting up offboard control mode
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_
        msg.position = not all(check_null(position))
        msg.velocity = not all(check_null(velocity))
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)

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

        self.trajectory_setpoint_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "main":
    main()

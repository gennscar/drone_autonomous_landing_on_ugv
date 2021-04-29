#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import functions
import numpy as np

from  px4_msgs.msg import OffboardControlMode
from  px4_msgs.msg import TrajectorySetpoint
from  px4_msgs.msg import Timesync
from  px4_msgs.msg import VehicleCommand
from  px4_msgs.msg import VehicleControlMode
from  px4_msgs.msg import VehicleLocalPosition
from  px4_msgs.msg import VehicleStatus
from nav_msgs.msg import Odometry

# Control parameters
KP = 0.5
KI = 0.01
KD = 0.0
INT_MAX = 250.0
VMAX = float('inf')
VMIN = - VMAX

class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        # Initialization to 0 of all parameters
        self.int_e = np.array([0,0])
        self.e_dot = np.array([0,0])
        self.e_old = np.array([0,0])
        self.e = np.array([0,0])
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
        self.target_global_pos = [0, 0]
        self.target_local_pos = [0, 0]
        self.timestamp = 0
        self.offboard_setpoint_counter_ = 0

        # Publishers
        self.offboard_control_mode_publisher_=self.create_publisher(OffboardControlMode,"OffboardControlMode_PubSubTopic",3)
        self.trajectory_setpoint_publisher_=self.create_publisher(TrajectorySetpoint,"TrajectorySetpoint_PubSubTopic",3)
        self.vehicle_command_publisher_=self.create_publisher(VehicleCommand,"VehicleCommand_PubSubTopic",3)
        
        # Subscribers
        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_local_position,3)
        self.drone_status_subscriber = self.create_subscription(VehicleStatus,"VehicleStatus_PubSubTopic",self.callback_drone_status,3)
        self.target_position_subscriber = self.create_subscription(Odometry,"/chassis/odom",self.callback_target_position,3)
        self.timesync_sub_=self.create_subscription(Timesync,"Timesync_PubSubTopic",self.callback_timesync,3)
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_timesync(self, msg):
        self.timestamp = msg.timestamp

    def callback_local_position(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 20):
            self.publish_vehicle_command(176, 1.0, 6.0)

            self.get_logger().info("arming..")
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if (self.offboard_setpoint_counter_ < 21):
            self.offboard_setpoint_counter_ += 1

    def callback_drone_status(self, msg):
        self.ARMING_STATE = msg.arming_state
    
    def callback_target_position(self,msg):
        self.target_global_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.target_local_pos = [self.target_global_pos[1]-1, self.target_global_pos[0]-1]


    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
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

        if self.TAKEOFF_STATE == 0:
            if abs(self.z) > 2.9:
                self.TAKEOFF_STATE = 1
            self.get_logger().info("Takeoff..")
            msg.x = float("NaN")
            msg.y = float("NaN")
            msg.z = -3.0

            self.trajectory_setpoint_publisher_.publish(msg)

        else:
            msg.x = float("NaN")
            msg.y = float("NaN")

            self.e = np.array([self.x - self.target_local_pos[0], self.y - self.target_local_pos[1]])
            [msg.vx, msg.vy], self.int_e, self.e_dot, self.e_old = functions.PID(KP, KI, KD, self.e, self.e_old, self.int_e, VMAX, VMIN, INT_MAX)
            self.norm_e = np.linalg.norm(self.e, ord=2)
            self.norm_e_dot = np.linalg.norm(self.e_dot, ord=2)
            self.get_logger().info(f"""int: {self.int_e}""")

            if ((self.norm_e) < 0.1) and ((self.norm_e_dot) < 0.2) and (-self.z < 0.9):
                self.LANDING_STATE = 1
                self.get_logger().info("Landing..")
                self.publish_vehicle_command(21, 0.0, 0.0)
            elif ((self.norm_e) < 0.1) and ((self.norm_e_dot) < 0.2) and self.LANDING_STATE == 0:
                self.DESCENDING_STATE = 1
                self.get_logger().info("Descending on target..")
                msg.z = float("NaN")
                msg.vz = 0.2
            elif self.LANDING_STATE == 0 and self.DESCENDING_STATE == 0:
                self.get_logger().info("Following target..")
                msg.z = - 3.0
                msg.vz = float("NaN")
            elif self.LANDING_STATE == 0:
                self.get_logger().info("Stopped descending..")
                msg.z = - 1.5
                msg.vz = - 0.1
            if self.ARMING_STATE == 1 and self.LANDING_STATE == 1:
                self.get_logger().info(f"Landed, with ex:{self.e[0]}, ey:{self.e[1]}")
                rclpy.shutdown()
            
            self.trajectory_setpoint_publisher_.publish(msg)


def main(args = None):
    rclpy.init(args = args)
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "main":
    main()
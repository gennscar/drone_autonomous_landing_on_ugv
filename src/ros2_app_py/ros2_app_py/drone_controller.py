#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from  px4_msgs.msg import OffboardControlMode
from  px4_msgs.msg import TrajectorySetpoint
from  px4_msgs.msg import Timesync
from  px4_msgs.msg import VehicleCommand
from  px4_msgs.msg import VehicleControlMode
from  px4_msgs.msg import VehicleLocalPosition
from  px4_msgs.msg import VehicleStatus
import numpy as np




class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        self.kp = 0.5
        self.ki = 0.0001
        self.vmax = float('inf')
        self.target_global_pos = [1.96, 5.0]
        self.target_pos = [self.target_global_pos[1]-1, self.target_global_pos[0]-1]

        self.i = 0.0
        self.arming_state = 0
        self.landing = 0
        
        self.int_ex = 0.0
        self.int_ey = 0.0
        
        self.vx_max = self.vmax
        self.vy_max = self.vmax
        self.vx_min = -self.vmax
        self.vy_min = -self.vmax

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        self.timestamp = 0
        self.offboard_setpoint_counter_ = 0

        self.offboard_control_mode_publisher_=self.create_publisher(OffboardControlMode,"OffboardControlMode_PubSubTopic",10)
        self.trajectory_setpoint_publisher_=self.create_publisher(TrajectorySetpoint,"TrajectorySetpoint_PubSubTopic",10)
        self.vehicle_command_publisher_=self.create_publisher(VehicleCommand,"VehicleCommand_PubSubTopic",10)
        
        self.drone_position_subscriber = self.create_subscription(VehicleLocalPosition,"VehicleLocalPosition_PubSubTopic",self.callback_local_position,10)
        self.drone_status_subscriber = self.create_subscription(VehicleStatus,"VehicleStatus_PubSubTopic",self.callback_drone_status,10)
        self.timesync_sub_=self.create_subscription(Timesync,"Timesync_PubSubTopic",self.callback_timesync,10)

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

    def callback_drone_status(self, msg):
        self.arming_state = msg.arming_state

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

        if self.i < 5.0:
            self.get_logger().info("Approaching target 1..")
            msg.x = float("NaN")
            msg.y = float("NaN")
            msg.z = -3.0
            msg.vx = 1.0
            msg.vy = 1.0

            self.i = self.i + 0.05
            self.trajectory_setpoint_publisher_.publish(msg)


        else:
            if (abs(self.x - self.target_pos[0]) < 0.05) and (abs(self.y - self.target_pos[1]) < 0.05) and (abs(self.vx) < 0.5) and (abs(self.vy) < 0.5):
                self.landing = 1
                self.get_logger().info("Landing..")
                self.publish_vehicle_command(21, 0.0, 0.0)


            elif self.landing == 0:
                self.get_logger().info("Approaching target 2..")
                
                msg.x = float("NaN")
                msg.y = float("NaN")
                msg.z = - 3.0

                self.ex = self.x - self.target_pos[0]
                self.ey = self.y - self.target_pos[1]

                self.int_ex = self.int_ex + self.ex
                self.int_ey = self.int_ey + self.ey

                msg.vx = - self.kp * self.ex - self.ki * self.int_ex
                msg.vy = - self.kp * self.ey - self.ki * self.int_ey
                

                if msg.vx > self.vx_max:
                    msg.vx = self.vx_max
                if msg.vx < self.vx_min:
                    msg.vx = self.vx_min
                if msg.vy > self.vy_max:
                    msg.vy = self.vy_max
                if msg.vy < self.vy_min:
                    msg.vy = self.vy_min

                self.trajectory_setpoint_publisher_.publish(msg)


            if (self.arming_state == 1):
                    self.get_logger().info(f"Landed, with x:{self.x}, y:{self.y}")
                    rclpy.shutdown()


def main(args = None):

    rclpy.init(args = args)
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()
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

from nav_msgs.msg import Odometry




class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        self.kp = 0.5
        self.ki = 0.01
        self.vmax = float('inf')
        self.target_global_pos = [0, 0]
        self.target_pos = [self.target_global_pos[1]-1, self.target_global_pos[0]-1]
        

        self.i = 0.0
        self.arming_state = 0
        self.landing = 0
        self.descending = 0
        self.takeoff = 0
        
        self.ex = 0.0
        self.ey = 0.0
        self.int_ex = 0.0
        self.int_ey = 0.0

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
        self.target_position_subscriber = self.create_subscription(Odometry,"demo/odom_demo",self.callback_target_position,10)
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

    def callback_drone_status(self, msg):
        self.arming_state = msg.arming_state
    
    def callback_target_position(self,msg):
        
        self.target_global_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.target_pos = [self.target_global_pos[1]-1, self.target_global_pos[0]-1]



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

        if self.takeoff == 0:
            if abs(self.z) > 2.9:
                self.takeoff = 1
            self.get_logger().info("Takeoff..")
            msg.x = 0.0
            msg.y = 0.0
            msg.z = -3.0

            self.trajectory_setpoint_publisher_.publish(msg)

        else:
            
            msg.x = float("NaN")
            msg.y = float("NaN")
            
            if abs(self.ex) > 2.0 or abs(self.ey) > 2.0:
                self.kp = 1
                self.ki = 0.005
            else:
                self.kp = 1.0
                self.ki = 0.01


            msg.vx, msg.vy, self.ex, self.ey, self.int_ex, self.int_ey = controller(self.x, self.y, self.target_pos[0], self.target_pos[1], self.kp, self.ki, self.vmax, self.int_ex, self.int_ey)
            

            if (abs(self.ex) < 0.3) and (abs(self.ey) < 0.3) and (-self.z < 0.9):
                self.landing = 1
                self.get_logger().info("Landing..")
                self.publish_vehicle_command(21, 0.0, 0.0)
            elif (abs(self.ex) < 0.2) and (abs(self.ey) < 0.2) and self.landing == 0:
                self.descending = 1
                self.get_logger().info("Descending on target..")
                msg.z = float("NaN")
                msg.vz = 0.2
            elif self.landing == 0 and self.descending == 0:
                self.get_logger().info("Following target..")
                msg.z = -3.0
                msg.vz = float("NaN")
            elif self.landing == 0:
                self.get_logger().info("Stopped descending..")
                msg.z = -1.5
                msg.vz = -0.2
            if self.arming_state == 1 and self.landing == 1:
                self.get_logger().info(f"Landed, with ex:{self.ex}, ey:{self.ey}")
                rclpy.shutdown()
            
            
            self.trajectory_setpoint_publisher_.publish(msg)
        
def controller(drone_x, drone_y, target_x, target_y, kp, ki, v_max, int_ex, int_ey):
    ex = drone_x - target_x
    ey = drone_y - target_y
    int_ex = int_ex + ex
    int_ey = int_ey + ey
    vx = - kp * ex - ki * int_ex
    vy = - kp * ey - ki * int_ey
    vx_max = v_max
    vy_max = v_max
    vx_min = -v_max
    vy_min = -v_max
    if vx > vx_max:
        vx = vx_max
    if vx < vx_min:
        vx = vx_min
    if vy > vy_max:
        vy = vy_max
    if vy < vy_min:
        vy = vy_min
    return vx, vy, ex, ey, int_ex, int_ey


def main(args = None):

    rclpy.init(args = args)
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from  px4_msgs.msg import OffboardControlMode
from  px4_msgs.msg import TrajectorySetpoint
from  px4_msgs.msg import Timesync
from  px4_msgs.msg import VehicleCommand
from  px4_msgs.msg import VehicleControlMode

# generate random integer values
from random import seed
from random import randint
# seed random number generator
seed(1)




class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")

        self.timestamp = 0
        self.offboard_setpoint_counter_ = 0

        self.offboard_control_mode_publisher_=self.create_publisher(OffboardControlMode,"OffboardControlMode_PubSubTopic",10)
        self.trajectory_setpoint_publisher_=self.create_publisher(TrajectorySetpoint,"TrajectorySetpoint_PubSubTopic",10)
        self.vehicle_command_publisher_=self.create_publisher(VehicleCommand,"VehicleCommand_PubSubTopic",10)

        self.timesync_sub_=self.create_subscription(Timesync,"Timesync_PubSubTopic",self.callback_timesync,10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_timesync(self, msg):
        self.timestamp = msg.timestamp

    def timer_callback(self):

        if (self.offboard_setpoint_counter_ == 10):
            self.publish_vehicle_command(176, 1.0, 6.0)

            self.get_logger().info("arming..")
            self.arm()

        self.publish_offboard_control_mode()
        self.get_logger().info("Sending takeoff commands..")
        self.publish_trajectory_setpoint()

        if (self.offboard_setpoint_counter_ < 11):
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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        #msg.x = 0.0
        #msg.y = 0.0
        msg.z = -5.0
        #msg.vx = 10.0
        self.trajectory_setpoint_publisher_.publish(msg)

        


def main(args = None):

    rclpy.init(args = args)
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "main":
    main()
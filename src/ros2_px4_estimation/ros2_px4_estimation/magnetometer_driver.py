import rclpy
from rclpy.node import Node

from ros2_px4_interfaces.msg import Yaw

import serial
import sys
import re
from serial import SerialException
import numpy as np

class MagnetometerNode(Node):
    def __init__(self):
        super().__init__('magnetometer_driver')

        # UART serial port
        self.declare_parameter('magnetometerPort', '/dev/ttyUSB0')

        # UART baud rate
        self.declare_parameter('magnetometerRate', 9600)

        # Acquisition frequency
        self.declare_parameter('timerPeriod', 0.1)

        # Topic name where to send magnetometer data
        self.declare_parameter('topic_name', '/magnetometer_yaw')

        # Getting serial connection
        self.magnetometer_serial()


        # magnetometer messages publisher

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(Yaw, self.topic_name, 10)
        self.timer = self.create_timer(self.get_parameter(
            'timerPeriod').get_parameter_value().double_value, self.timer_callback)
        self.i = 0

        self.n_turns_ = 0.0
        self.old_angle_deg_ = 0.0

    def magnetometer_serial(self):
        try:
            serialPort = self.get_parameter(
                'magnetometerPort').get_parameter_value().string_value
            serialRate = self.get_parameter(
                'magnetometerRate').get_parameter_value().integer_value
            self.ser = serial.Serial(
                port=serialPort,
                timeout=10,
                baudrate=serialRate
            )
            self.get_logger().info('Connected to: "%s"' % serialPort)
        except SerialException:
            self.get_logger().error('Impossible to read magnetometer data')
            sys.exit()

    def timer_callback(self):
        # After the while loop, the new line (after the control line) will be the "prepare line"
        try:
            raw_data = self.ser.readline()
        except SerialException:
            self.get_logger().error('Impossible to read magnetometer data')
            sys.exit()

        if raw_data == serial.to_bytes([]):
            self.get_logger().warn("No data received from serial port")
        else:
            data = re.split(':\s|\s|:', raw_data.decode())[:-1]
            mag_x = float(data[3])
            mag_y = float(data[5])
            angle_deg_ = np.arctan2(mag_y, mag_x)*180/np.pi

            if angle_deg_ >= -180 and angle_deg_ <= 180:
                if angle_deg_ - self.old_angle_deg_ > 300.0:
                    self.n_turns_ -= 1
                elif angle_deg_ - self.old_angle_deg_ < - 300.0:
                    self.n_turns_ += 1
                self.old_angle_deg_ = angle_deg_

                angle_deg_ = angle_deg_ + self.n_turns_*360.0

                msg = Yaw()
                msg.yaw = angle_deg_
                msg.header.frame_id = self.topic_name
                msg.header.stamp = self.get_clock().now().to_msg()

                self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    uwb_device = MagnetometerNode()
    rclpy.spin(uwb_device)
    uwb_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

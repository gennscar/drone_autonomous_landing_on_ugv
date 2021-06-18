import rclpy
from rclpy.node import Node

from ros2_px4_interfaces.msg import UwbSensor

import serial
import sys
import re
from serial import SerialException
import json


class UwbDevice(Node):
    def __init__(self):
        super().__init__('uwb_driver')

        # UART serial port
        self.declare_parameter('uwbPort', '/dev/ttyACM0')

        # UART baud rate
        self.declare_parameter('uwbRate', 57600)

        # Acquisition frequency
        self.declare_parameter('timerPeriod', 0.0)

        # Topic name where to send ranging data
        self.declare_parameter('topic_name', '/uwb_ranging')

        # Getting UWB connection
        self.uwb_serial()

        # Getting anchor info from file
        fp = open('json/anchors.json', 'r')
        self.anchors = json.load(fp)
        fp.close()

        # UWB messages publisher
        self.publisher_ = self.create_publisher(UwbSensor, self.get_parameter(
            'topic_name').get_parameter_value().string_value, 10)
        self.timer = self.create_timer(self.get_parameter(
            'timerPeriod').get_parameter_value().double_value, self.timer_callback)
        self.i = 0

    def uwb_serial(self):
        try:
            serialPort = self.get_parameter(
                'uwbPort').get_parameter_value().string_value
            serialRate = self.get_parameter(
                'uwbRate').get_parameter_value().integer_value
            self.ser = serial.Serial(
                port=serialPort,
                timeout=10,
                baudrate=serialRate
            )
            self.get_logger().info('Connected to: "%s"' % serialPort)
        except SerialException:
            self.get_logger().error('Impossible to read UWB Tag')
            sys.exit()

    def timer_callback(self):
        # After the while loop, the new line (after the control line) will be the "prepare line"
        try:
            raw_data = self.ser.readline()
        except SerialException:
            self.get_logger().error('Impossible to read UWB Tag')
            sys.exit()

        if raw_data == serial.to_bytes([]):
            self.get_logger().warn("No data received from serial port")
        else:
            data = re.split(':\s|\s|:', raw_data.decode())[:-1]

            if data[0] == 'RNG' and data[3] == 'SUCCESS':
                msg = UwbSensor()

                msg.timestamp = self.get_clock().now().nanoseconds * 1e-9
                msg.anchor_id = int(data[2].split('->')[1], base=16)
                msg.range = float(data[6]) * 1e-2

                id = str(msg.anchor_id)
                msg.anchor_pos.x = self.anchors[id][0]
                msg.anchor_pos.y = self.anchors[id][1]
                msg.anchor_pos.z = self.anchors[id][2]

                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    uwb_device = UwbDevice()
    rclpy.spin(uwb_device)
    uwb_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

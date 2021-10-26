import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from ros2_px4_interfaces.msg import UwbSensor

import serial
import sys
import json


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


class UwbDevice(Node):
    def __init__(self):
        super().__init__('uwb_driver')

        self.msg_ = UwbSensor()

        # UART serial port
        self.declare_parameter('uwbPort', '/dev/ttyACM0')

        # UART baud rate
        self.declare_parameter('uwbRate', 57600)

        # Acquisition frequency
        self.declare_parameter('timerPeriod', 0.0)

        # Topic name where to send ranging data
        self.declare_parameter('topic_name', 'tag_0')

        # Getting UWB connection
        self.uwb_serial()

        # Anchors position file path
        self.declare_parameter('anchors_pos_file_path',
                               '/home/cosimocon/Dev/ros2_px4_ws/json/anchors.json')

        # Getting anchor info from file
        fp = open(self.get_parameter(
            'anchors_pos_file_path').get_parameter_value().string_value, 'r')
        self.anchors = json.load(fp)
        fp.close()

        # UWB messages publisher
        self.publisher_ = self.create_publisher(UwbSensor,  "/uwb_sensor_" + self.get_parameter(
            'topic_name').get_parameter_value().string_value, 1)
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
            self.rl = ReadLine(self.ser)
            self.get_logger().info('Connected to: "%s"' % serialPort)
        except serial.SerialException:
            self.get_logger().error('Impossible to read UWB Tag')
            sys.exit()

    def timer_callback(self):
        # After the while loop, the new line (after the control line) will be the "prepare line"
        raw_data = self.rl.readline()
        data = raw_data.decode().split()

        if len(data) > 7 and data[0] == 'RNG' and data[3] == 'SUCCESS':
            id = data[2][:-1]

            if id not in self.anchors.keys():
                return

            self.msg_.anchor_pose.header.stamp = self.get_clock().now().to_msg()
            self.msg_.anchor_pose.header.frame_id = id
            self.msg_.range = float(data[6]) * 1e-2

            self.msg_.anchor_pose.pose.position.x = self.anchors[id][0]
            self.msg_.anchor_pose.pose.position.y = self.anchors[id][1]
            self.msg_.anchor_pose.pose.position.z = self.anchors[id][2]

            self.publisher_.publish(self.msg_)


def main(args=None):
    rclpy.init(args=args)
    uwb_device = UwbDevice()
    rclpy.spin(uwb_device)
    uwb_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

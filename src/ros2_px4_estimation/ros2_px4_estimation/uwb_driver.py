import rclpy
from rclpy.node import Node

from ros2_px4_interfaces.msg import Uwb

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

import serial, time, os, sys, re
from serial import SerialException
#######################
#TO DO Publish CIR
#TO DO Add clibaration offset param
##################
class UwbDevice(Node):
    def __init__(self):
        super().__init__('uwb_driver')

        self.ok = []     #  SUCCESS/FAIL
        self.mes = []    # Ranging measurement (length = nr of anchors)
        self.fpp = []    # first peak power (length = nr of anchors)
        self.rxp = []    # received signal strength (length = nr of anchors)
        self.clay = []   # TO DO!!!!!!!!!!! cir vector layout
        self.cir = []    # TO DO !!!!!!!!!!!
        self.mn = []     # max noise
        self.stn = []    # std noise

        self.declare_parameter('uwbPort', '/dev/ttyACM0') # Set serial port
        self.declare_parameter('uwbRate', 57600) # Set baud rate
        #self.declare_parameter('calibrationOffset', '0') # ADD TO msg.range_mes Can be used to correct biased mesurements
        self.declare_parameter('timerPeriod', 0.0) # Acquisition frequency
        self.declare_parameter('topic_name', 'uwb_ranging')

        self.publisher_ = self.create_publisher(Uwb, self.get_parameter('topic_name').get_parameter_value().string_value, 10)
        timer_period = self.get_parameter('timerPeriod').get_parameter_value().double_value
        self.uwb_serial()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #print(timer_period)


    def uwb_serial(self):
        try:
            serialPort = self.get_parameter('uwbPort').get_parameter_value().string_value
            serialRate = self.get_parameter('uwbRate').get_parameter_value().integer_value
            self.ser = serial.Serial(
            port = serialPort,
            timeout = 10,
            baudrate = serialRate
            )
            #print(serialPort)
            #print(serialRate)
            self.get_logger().info('Connected to: "%s"' % serialPort)
        except SerialException:
            print("Could not open the serial port")
            sys.exit()


    def timer_callback(self):
        msg = Uwb()

        index = []

        raw_data = self.ser.readline()  # After the while loop, the new line (after the control line) will be the "prepare line"
        #print(raw_data)
        if raw_data == serial.to_bytes([]):
            print ("No data received from serial port")
        else:
            data = re.split(":\s|\s|:", raw_data.decode())[:-1]
            #print(data[0])
            #print(raw_data)
            if data[0] == 'Prepare':
                #print(data[0])
                k = 0 # Count ones in "index" list
                data = re.split(":\s|\s|:", self.ser.readline().decode())[:-1]
                #print(data)
                while data[0] == 'RNG' or data[0] == 'DIAG':
                    #print(index)
                    if data[0] == 'RNG':
                        range_ok = data[3]
                        #print(data)
                        if range_ok == 'SUCCESS':
                            self.ok.append(range_ok)
                            self.mes.append(data[6])
                            self.fpp.append(data[8])
                            self.rxp.append(data[10])
                            index.append('1')
                            k += 1
                            #print (self.mes)
                        elif range_ok == 'FAIL':
                            self.ok.append(range_ok)
                            self.mes.append('0')
                            self.fpp.append('0')
                            self.rxp.append('0')
                            index.append('0')
                            #print(range_ok)
                    elif data[0] == 'DIAG':
                        #print(index)
                        self.clay = []
                        self.cir = []
                        self.mn.append(data[10])
                        self.stn.append(data[12])

                    data = re.split(":\s|\s|:", self.ser.readline().decode())[:-1] # The last read line is the "control line"
                #print(self.ok)
                #print(self.mes)
                #print(self.fpp)
                #print(self.rxp)
                #print(index)
                #print(self.stn) # Use index to crate the proper list
                j = 0
                mn = []
                stn = []
                #print(index)
                #print(k)
                if len(self.mn) == k:
                    for i in range(0, len(index)):
                        if index[i] == '1':
                            self.clay = []
                            self.cir = []
                            #print(self.mn)
                            mn.append(self.mn[j])
                            stn.append(self.stn[j])
                            j += 1
                            #print(j)
                        elif index[i] == '0':
                            self.clay = '0'
                            self.cir = '0'
                            mn.append('0')
                            stn.append('0')
                clock = self.get_clock().now().to_msg()
                #print(type(clock.sec))
                msg.timestamp = [clock.sec, clock.nanosec]
                msg.range_ok = self.ok
                msg.range_mes = list(map(int, self.mes))
                msg.first_peak_pwr = list(map(int, self.fpp))
                msg.rx_pwr = list(map(int, self.rxp))
                msg.cir_layout = list(map(int, self.clay))
                msg.cir = list(map(int, self.cir))
                msg.max_noise = list(map(int, mn))
                msg.std_noise = list(map(int, stn))
                #self.get_logger().info('Publishing: "%s"' % msg.range_ok)

                self.publisher_.publish(msg)
                self.ok = []
                self.mes = []
                self.fpp = []
                self.rxp = []
                self.clay = []
                self.cir = []
                self.mn = []
                self.stn = []



def main(args = None):
    rclpy.init(args = args)

    uwb_device = UwbDevice()

    rclpy.spin(uwb_device)

    uwb_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

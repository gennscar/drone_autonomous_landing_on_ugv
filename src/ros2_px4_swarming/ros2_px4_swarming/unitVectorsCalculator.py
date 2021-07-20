#!/usr/bin/env python3
import math
from functools import partial
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, VehicleGlobalPosition
from ros2_px4_interfaces.msg import UnitVector, UnitVectorArray

# TODO: check the case of a drone disconnecting


class UnitVectorsCalculator(Node):
    def __init__(self):
        super().__init__("unitVectorsCalculator")

        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('N', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.N = self.get_parameter('N').value

        # Useful variables
        self.timestamp = 0
        self.anchorsPosition = {}
        self.anchorsPositionSubs = list()
        self.lastPositionReceivedTimer = {}

        # Subscribers initialization
        for i in range(self.N):
            self.anchorsPositionSubs.append(self.create_subscription(VehicleGlobalPosition, "X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.QUEUE_SIZE))
            self.lastPositionReceivedTimer[i] = 0

        # Publishers initialization
        self.unitVectorsPub = self.create_publisher(UnitVectorArray, "unitVectors", self.QUEUE_SIZE)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    def computeUnitVector(self, destinationDroneId, startDroneId, timestamp):
        deltaLat = math.log(math.tan(math.radians(self.anchorsPosition[destinationDroneId].lat) / 2 + math.pi / 4) / math.tan(math.radians(self.anchorsPosition[startDroneId].lat) / 2 + math.pi / 4))
        deltaLong = math.radians(self.anchorsPosition[destinationDroneId].lon - self.anchorsPosition[startDroneId].lon)
        angle = math.atan2(deltaLong, deltaLat)
        result = UnitVector()
        result.header.stamp = timestamp
        result.destination_drone_id = destinationDroneId
        result.start_drone_id = startDroneId
        result.east = math.sin(angle)
        result.north = math.cos(angle)
        return result

    # Callbacks
    def timerCallback(self):
        if len(self.anchorsPosition) < self.N:
            return

        unitVectors = []

        for i in range(self.N):
            self.lastPositionReceivedTimer[i] += 1
            for j in range(i + 1, self.N):
                if self.lastPositionReceivedTimer[i] < 5 * self.RATE and self.lastPositionReceivedTimer[j] < 5 * self.RATE:
                    unitVectors.append(self.computeUnitVector(i, j, self.get_clock().now().to_msg()))

        msg = UnitVectorArray()
        msg.data = unitVectors
        self.unitVectorsPub.publish(msg)

    def anchorsPositionCallback(self, msg, droneId):
        self.anchorsPosition[droneId] = msg
        self.lastPositionReceivedTimer[droneId] = 0


def main():
    rclpy.init(args=None)
    node = UnitVectorsCalculator()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

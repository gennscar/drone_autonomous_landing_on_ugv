#!/usr/bin/env python3
import copy
import math
from functools import partial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleLocalPosition
from ros2_px4_interfaces.msg import DistanceStamped, DistanceStampedArray, UwbSensor


class PerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__("performanceAnalyzer")

        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('TARGET_ID', None),
                ('N', None),
                ('NUM_TARGET', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.TARGET_ID = self.get_parameter('TARGET_ID').value
        self.N = self.get_parameter('N').value
        self.NUM_TARGET = self.get_parameter('NUM_TARGET').value

        # Useful variables
        self.anchorsPositionGroundTruth = {}
        self.anchorsPositionGroundTruthSubs = list()
        self.anchorsPosition = {}
        self.anchorsPositionSubs = list()
        self.targetPosition = None
        self.targetUwbDistances = {}

        # Subscribers initialization
        for i in range(self.N):
            self.anchorsPositionGroundTruthSubs.append(self.create_subscription(Odometry, "/X500_" + str(i) + "/GroundTruth/odom", partial(self.anchorsPositionGroundTruthCallback, droneId=i), self.QUEUE_SIZE))
            # self.anchorsPositionSubs.append(self.create_subscription(VehicleLocalPosition, "/X500_" + str(i) + "/VehicleLocalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.QUEUE_SIZE))
        if self.NUM_TARGET == 1:
            self.targetPositionSub = self.create_subscription(Odometry, "/targetRover/GroundTruth/odom", self.targetPositionCallback, self.QUEUE_SIZE)
            self.targetUwbSensorSub = self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.TARGET_ID), self.targetUwbSensorCallback, self.QUEUE_SIZE)

        # Publishers initialization
        if self.NUM_TARGET == 1:
            self.trackingErrorPub = self.create_publisher(DistanceStamped, "trackingError", self.QUEUE_SIZE)
        self.interAnchorsDistancesPub = self.create_publisher(DistanceStampedArray, "interAnchorsDistances", self.QUEUE_SIZE)
        self.targetAnchorsDistancesPub = self.create_publisher(DistanceStampedArray, "targetAnchorsDistances", self.QUEUE_SIZE)
        self.synchronizationErrorUwbPub = self.create_publisher(Float64, "synchronizationErrorUwb", self.QUEUE_SIZE)
        self.swarmCenterPub = self.create_publisher(Odometry, "swarmCenter", self.QUEUE_SIZE)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    def computeSwarmCenter(self):
        xCenter = 0.0
        yCenter = 0.0
        zCenter = 0.0
        for i in range(self.N):
            xCenter += self.anchorsPositionGroundTruth[i].pose.pose.position.x
            yCenter += self.anchorsPositionGroundTruth[i].pose.pose.position.y
            zCenter += self.anchorsPositionGroundTruth[i].pose.pose.position.z
        xCenter /= self.N
        yCenter /= self.N
        zCenter /= self.N
        return [xCenter, yCenter, zCenter]

    def computeTrackingError(self):
        result = DistanceStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        swarmCenter = self.computeSwarmCenter()
        result.destination_drone_id = 500
        result.start_drone_id = 500
        result.distance = math.sqrt((swarmCenter[0] - self.targetPosition.pose.pose.position.x)**2 + (swarmCenter[1] - self.targetPosition.pose.pose.position.y)**2)
        return result

    def computeInterAnchorsDistances(self):
        interAnchorsDistances = list()
        result = DistanceStampedArray()
        msg = DistanceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.N):
            for j in range(i + 1, self.N):
                msg.destination_drone_id = i
                msg.start_drone_id = j
                msg.distance = math.sqrt((self.anchorsPositionGroundTruth[i].pose.pose.position.x - self.anchorsPositionGroundTruth[j].pose.pose.position.x)**2 + (self.anchorsPositionGroundTruth[i].pose.pose.position.y - self.anchorsPositionGroundTruth[j].pose.pose.position.y)**2 + (self.anchorsPositionGroundTruth[i].pose.pose.position.z - self.anchorsPositionGroundTruth[j].pose.pose.position.z)**2)
                interAnchorsDistances.append(copy.deepcopy(msg))
        result.data = interAnchorsDistances
        return result

    def computeTargetAnchorsDistances(self):
        targetAnchorsDistances = list()
        result = DistanceStampedArray()
        msg = DistanceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.N):
            msg.destination_drone_id = i
            msg.start_drone_id = 200
            try:
                msg.distance = math.sqrt((self.anchorsPositionGroundTruth[i].pose.pose.position.x - self.targetPosition.pose.pose.position.x)**2 + (self.anchorsPositionGroundTruth[i].pose.pose.position.y - self.targetPosition.pose.pose.position.y)**2 + (self.anchorsPositionGroundTruth[i].pose.pose.position.z - self.targetPosition.pose.pose.position.z)**2)
            except:
                msg.distance = 0.0
            targetAnchorsDistances.append(copy.deepcopy(msg))
        result.data = targetAnchorsDistances
        return result

    def computeSynchronizationErrorUwb(self):
        msg = Float64()
        syncErr = 0.0
        for i in range(self.N):
            for j in range(i + 1, self.N):
                tmp = abs((self.targetUwbDistances[i].anchor_pose.header.stamp.sec + self.targetUwbDistances[i].anchor_pose.header.stamp.nanosec * 1e-9) - (self.targetUwbDistances[j].anchor_pose.header.stamp.sec + self.targetUwbDistances[j].anchor_pose.header.stamp.nanosec * 1e-9))
                if tmp > syncErr:
                    syncErr = tmp
        msg.data = syncErr
        return msg

    # Callbacks
    def timerCallback(self):
        if len(self.anchorsPositionGroundTruth) == self.N:
            self.interAnchorsDistancesPub.publish(self.computeInterAnchorsDistances())
            self.targetAnchorsDistancesPub.publish(self.computeTargetAnchorsDistances())
            swarmCenter = self.computeSwarmCenter()
            msg = Odometry()
            msg.pose.pose.position.x = swarmCenter[0]
            msg.pose.pose.position.y = swarmCenter[1]
            msg.pose.pose.position.z = swarmCenter[2]
            self.swarmCenterPub.publish(msg)
            if self.targetPosition is not None:
                self.trackingErrorPub.publish(self.computeTrackingError())

        if len(self.targetUwbDistances) == self.N:
            self.synchronizationErrorUwbPub.publish(self.computeSynchronizationErrorUwb())

    def anchorsPositionGroundTruthCallback(self, msg, droneId):
        self.anchorsPositionGroundTruth[droneId] = msg

    def anchorsPositionCallback(self, msg, droneId):
        self.anchorsPosition[droneId] = msg

    def targetPositionCallback(self, msg):
        self.targetPosition = msg

    def targetUwbSensorCallback(self, msg):
        self.targetUwbDistances[int(msg.anchor_pose.header.frame_id)] = msg


def main():
    rclpy.init(args=None)
    node = PerformanceAnalyzer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PerformanceAnalyzer(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml". It declares useful variables,
        publishers, and subscribers. It creates also a timer to regulate the frequency of the node.
        """

        super().__init__("performanceAnalyzer")

        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('TARGET_ID', None),
                ('BEST_EFFORT', None),
                ('N', None),
                ('NUM_TARGET', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.TARGET_ID = self.get_parameter('TARGET_ID').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        self.N = self.get_parameter('N').value
        self.NUM_TARGET = self.get_parameter('NUM_TARGET').value

        # Useful variables
        self.anchorsPositionGroundTruth = {}
        self.anchorsPositionGroundTruthSubs = list()
        self.anchorsPosition = {}
        self.anchorsPositionSubs = list()
        self.targetPosition = None
        self.targetUwbDistances = {}
        self.qosProfile = None

        # QOS initialization
        if self.BEST_EFFORT:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )
        else:
            self.qosProfile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )

        # Subscribers initialization
        for i in range(self.N):
            self.anchorsPositionGroundTruthSubs.append(self.create_subscription(Odometry, "/X500_" + str(i) + "/GroundTruth/odom", partial(self.anchorsPositionGroundTruthCallback, droneId=i), self.qosProfile))
            # self.anchorsPositionSubs.append(self.create_subscription(VehicleLocalPosition, "/X500_" + str(i) + "/VehicleLocalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.qosProfile))
        if self.NUM_TARGET == 1:
            self.targetPositionSub = self.create_subscription(Odometry, "/targetRover/GroundTruth/odom", self.targetPositionCallback, self.qosProfile)
            self.targetUwbSensorSub = self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.TARGET_ID), self.targetUwbSensorCallback, self.qosProfile)

        # Publishers initialization
        if self.NUM_TARGET == 1:
            self.trackingErrorPub = self.create_publisher(DistanceStamped, "trackingError", self.qosProfile)
        self.interAnchorsDistancesPub = self.create_publisher(DistanceStampedArray, "interAnchorsDistances", self.qosProfile)
        self.targetAnchorsDistancesPub = self.create_publisher(DistanceStampedArray, "targetAnchorsDistances", self.qosProfile)
        self.synchronizationErrorUwbPub = self.create_publisher(Float64, "synchronizationErrorUwb", self.qosProfile)
        self.swarmCenterPub = self.create_publisher(Odometry, "swarmCenter", self.qosProfile)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    def computeSwarmCenter(self):
        """
        @return: center of the swarm

        It computes the ground truth position of the swarm center, that will be used to evaluate the performance of the
        algorithm.
        """

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
        """
        @return: tracking error

        It computes the Euclidean horizontal distance between the center of the formation and the ground truth position
        of the rover to be followed. Then, it returns the message including this value.
        """

        result = DistanceStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        swarmCenter = self.computeSwarmCenter()
        result.destination_drone_id = 500
        result.start_drone_id = 500
        result.distance = math.sqrt((swarmCenter[0] - self.targetPosition.pose.pose.position.x)**2 + (swarmCenter[1] - self.targetPosition.pose.pose.position.y)**2)
        return result

    def computeInterAnchorsDistances(self):
        """
        @return: distances between the drones

        It computes the Euclidean distance between every couple of drones in the formation, and stores them in a list,
        that is finally returned as "DistanceStampedArray" message.
        """

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
        """
        @return: distances between each drone and the target

        It computes the Euclidean distance between each drone in the formation and the target, and stores them in a
        list, that is finally returned as "DistanceStampedArray" message.
        """

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
        """
        @return: synchronization error

        It computes the maximum synchronization error between the measurement performed by the UWB sensors, and it
        returns a message containing this value.
        """

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
        """
        It is called at a chosen rate, and it publishes the data computed to evaluate the performance of the system.
        """

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
        """
        @param msg: message
        @param droneId: drone id

        It saves the ground truth position of the drone identified by droneId.
        """
        self.anchorsPositionGroundTruth[droneId] = msg

    def anchorsPositionCallback(self, msg, droneId):
        """
        @param msg: message
        @param droneId: drone id

        It saves the position of the drone identified by droneId.
        """

        self.anchorsPosition[droneId] = msg

    def targetPositionCallback(self, msg):
        """
        @param msg: message

        It saves the position of the target rover.
        """

        self.targetPosition = msg

    def targetUwbSensorCallback(self, msg):
        """
        @param msg: message

        It saves the distance between the drone identified by msg.anchor_pose.header.frame_id and the target, as given
        by the UWB sensor.
        """

        self.targetUwbDistances[int(msg.anchor_pose.header.frame_id)] = msg


def main():
    rclpy.init(args=None)
    node = PerformanceAnalyzer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

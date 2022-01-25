#!/usr/bin/env python3
import math
from functools import partial
import statistics
import numpy.linalg
from itertools import combinations, permutations
import rclpy
from rclpy.node import Node
from px4_msgs.msg import Timesync, VehicleGlobalPosition
from std_msgs.msg import UInt64
from ros2_px4_interfaces.srv import SwarmCommand
from ros2_px4_interfaces.msg import UnitVector, UwbSensor, VelocityVector
from ros2_px4_functions import swarming_functions
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class TrackingVelocityCalculator(Node):
    def __init__(self):
        """
        This method declares and initializes the parameters imported by "params.yaml". It declares and initializes
        useful variables, publishers, subscribers, and services. It creates a timer to regulate the frequency of the
        controller.
        """

        super().__init__("trackingVelocityCalculator")

        # Parameters declaration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RATE', None),
                ('QUEUE_SIZE', None),
                ('TARGET_ID', None),
                ('TIMEOUT', None),
                ('TIMEOUT_UWB', None),
                ('MAX_TRACKING_SPEED', None),
                ('K_P', None),
                ('K_I', None),
                ('K_D', None),
                ('FILTER_WINDOW', None),
                ('MAX_ERROR_FOR_INTEGRAL_AND_DERIVATIVE', None),
                ('EARTH_RADIUS', None),
                ('BEST_EFFORT', None),
                ('N', None),
                ('NUM_TARGET', None)
            ]
        )

        # Parameters initialization
        self.RATE = self.get_parameter('RATE').value
        self.QUEUE_SIZE = self.get_parameter('QUEUE_SIZE').value
        self.TARGET_ID = self.get_parameter('TARGET_ID').value
        self.TIMEOUT = self.get_parameter('TIMEOUT').value
        self.TIMEOUT_UWB = self.get_parameter('TIMEOUT_UWB').value
        self.MAX_TRACKING_SPEED = self.get_parameter('MAX_TRACKING_SPEED').value
        self.K_P = self.get_parameter('K_P').value
        self.K_I = self.get_parameter('K_I').value
        self.K_D = self.get_parameter('K_D').value
        self.FILTER_WINDOW = self.get_parameter('FILTER_WINDOW').value
        self.MAX_ERROR_FOR_INTEGRAL_AND_DERIVATIVE = self.get_parameter('MAX_ERROR_FOR_INTEGRAL_AND_DERIVATIVE').value
        self.EARTH_RADIUS = self.get_parameter('EARTH_RADIUS').value
        self.BEST_EFFORT = self.get_parameter('BEST_EFFORT').value
        self.N = self.get_parameter('N').value
        self.NUM_TARGET = self.get_parameter('NUM_TARGET').value

        # Useful variables
        self.goalPosition = None
        self.anchorsPosition = {}
        self.anchorsPositionSubs = list()
        self.targetUwbDistances = {}
        self.lastUwbDistanceReceived = {}
        self.unitVectors = [[None] * self.N] * self.N
        self.anchorsPositionReceived = False
        self.trackingVelocityIntegral = [0.0, 0.0]
        self.lastTrackingVelocity = {0: self.FILTER_WINDOW * [0.0], 1: self.FILTER_WINDOW * [0.0]}
        self.firstIteration = True
        self.anchorsReadyForSwarming = {}
        self.anchorsReadyForSwarming = {}
        self.lastPositionReceivedTimer = {}
        self.qosProfile = None
        for i in range(self.N):
            self.lastPositionReceivedTimer[i] = 0
            self.lastUwbDistanceReceived[i] = 0

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
        if self.NUM_TARGET == 1:
            self.targetUwbSensorSub = self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.TARGET_ID), self.targetUwbSensorCallback, self.qosProfile)
        for i in range(self.N):
            self.anchorsPositionSubs.append(self.create_subscription(VehicleGlobalPosition, "/X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.anchorsPositionCallback, droneId=i), self.qosProfile))
            self.anchorsReadyForSwarmingSub = self.create_subscription(UInt64, "/readyForSwarming", self.anchorsReadyForSwarmingCallback, self.qosProfile)

        # Services initialization
        self.droneCommandSrv = self.create_service(SwarmCommand, "SwarmCommand", self.swarmCommandCallback)

        # Publishers initialization
        self.trackingVelocityPub = self.create_publisher(VelocityVector, "trackingVelocity", self.qosProfile)
        self.trackingVelocityProportionalPub = self.create_publisher(VelocityVector, "trackingVelocityProportional", self.qosProfile)
        self.trackingVelocityIntegralPub = self.create_publisher(VelocityVector, "trackingVelocityIntegral", self.qosProfile)
        self.trackingVelocityDerivativePub = self.create_publisher(VelocityVector, "trackingVelocityDerivative", self.qosProfile)

        # Control loop timer
        self.timer = self.create_timer(1 / self.RATE, self.timerCallback)

    def computeSwarmCenter(self):
        """
        @return: GNSS location of the swarm center

        Given the global position of every drone, this method computes the GNSS position of the center of the formation.
        """

        centerLatitude = 0.0
        centerLongitude = 0.0
        activeDrones = 0
        for key in self.lastPositionReceivedTimer.keys():
            if self.lastPositionReceivedTimer[key] < self.TIMEOUT * self.RATE:
                activeDrones += 1
                centerLatitude += self.anchorsPosition[key].lat
                centerLongitude += self.anchorsPosition[key].lon
        centerLatitude /= activeDrones
        centerLongitude /= activeDrones
        return [centerLatitude, centerLongitude]

    def computeTrackingVelocity(self):
        """
        @return: tracking velocity vector

        After checking that the drones communicate correctly, this method performs a PID control of the tracking
        velocity, that is then saturated to self.MAX_TRACKING_SPEED before being returned.
        """

        result = VelocityVector()
        timestamp = self.get_clock().now().to_msg()
        result.header.stamp = timestamp

        if len(self.anchorsReadyForSwarming) < self.N:
            result.east = 0.0
            result.north = 0.0
            return result

        if self.NUM_TARGET == 0:
            if self.goalPosition is None or len(self.anchorsPosition) < self.N:
                result.east = 0.0
                result.north = 0.0
                return result

            center = self.computeSwarmCenter()

            # Compute the directional unit vector
            deltaLatitude = math.log(math.tan(math.radians(self.goalPosition.lat) / 2 + math.pi / 4) / math.tan(math.radians(center[0]) / 2 + math.pi / 4))
            deltaLongitude = math.radians(self.goalPosition.lon - center[1])
            angle = math.atan2(deltaLongitude, deltaLatitude)
            direction = [math.sin(angle), math.cos(angle)]      # [east, north]

            # Compute the distance to the goal position
            a = (math.sin(math.radians(self.goalPosition.lat - center[0]) / 2))**2 + math.cos(math.radians(center[0])) * math.cos(math.radians(self.goalPosition.lat)) * (math.sin(math.radians(self.goalPosition.lon - center[1]) / 2))**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            distance = self.EARTH_RADIUS * c

            # Compute the tracking velocity (without target)
            result.east = distance * direction[0] * self.K_P
            result.north = distance * direction[1] * self.K_P

        elif self.NUM_TARGET == 1:
            if len(self.targetUwbDistances) < self.N or not self.anchorsPositionReceived:
                result.east = 0.0
                result.north = 0.0
                return result

            activeDrones = 0
            dronesToRemove = []
            for key in self.anchorsPosition.keys():
                if self.lastPositionReceivedTimer[key] >= self.TIMEOUT * self.RATE:
                    dronesToRemove.append(key)
                else:
                    activeDrones += 1

            for i in range(len(dronesToRemove)):
                self.anchorsPosition.pop(dronesToRemove[i])

            for couple in permutations(self.anchorsPosition, 2):
                self.unitVectors[couple[0]][couple[1]] = swarming_functions.computeUnitVector(couple[0], self.anchorsPosition[couple[0]], couple[1], self.anchorsPosition[couple[1]], self.get_clock().now().to_msg())

            # Compute the tracking velocity (with target)
            velEast = 0.0
            velNorth = 0.0

            # Check if some UWB has not been working for too long
            for key in self.lastUwbDistanceReceived.keys():
                if self.lastUwbDistanceReceived[key] >= self.TIMEOUT_UWB * self.RATE:
                    result.east = 0.0
                    result.north = 0.0
                    return result

            for couple in permutations(self.anchorsPosition, 2):
                componentEast = (self.targetUwbDistances[couple[1]] - self.targetUwbDistances[couple[0]]) * self.unitVectors[couple[0]][couple[1]].east
                componentNorth = (self.targetUwbDistances[couple[1]] - self.targetUwbDistances[couple[0]]) * self.unitVectors[couple[0]][couple[1]].north
                velEast += componentEast
                velNorth += componentNorth

            velEast *= (self.K_P / activeDrones)
            velNorth *= (self.K_P / activeDrones)

            # Da cancellare
            msgTmp = VelocityVector()
            msgTmp.header.stamp = timestamp
            msgTmp.east = velEast
            msgTmp.north = velNorth
            self.trackingVelocityProportionalPub.publish(msgTmp)
            #

            # Differentiation
            if self.firstIteration or numpy.linalg.norm([velEast, velNorth]) >= self.MAX_ERROR_FOR_INTEGRAL_AND_DERIVATIVE:
                velEastDiff = 0.0
                velNorthDiff = 0.0
                self.firstIteration = False
            else:
                velEastDiff = (velEast - statistics.median(self.lastTrackingVelocity[0])) * self.RATE * self.K_D
                velNorthDiff = (velNorth - statistics.median(self.lastTrackingVelocity[1])) * self.RATE * self.K_D

            self.lastTrackingVelocity[0].append(velEast)
            self.lastTrackingVelocity[1].append(velNorth)
            self.lastTrackingVelocity[0].pop(0)
            self.lastTrackingVelocity[1].pop(0)

            # Da cancellare
            msgTmp.east = velEastDiff
            msgTmp.north = velNorthDiff
            self.trackingVelocityDerivativePub.publish(msgTmp)
            #

            # Integration
            if numpy.linalg.norm([velEast, velNorth]) < self.MAX_ERROR_FOR_INTEGRAL_AND_DERIVATIVE:
                self.trackingVelocityIntegral[0] += velEast
                self.trackingVelocityIntegral[1] += velNorth
            else:
                self.trackingVelocityIntegral[0] = 0
                self.trackingVelocityIntegral[1] = 0

            velEastInt = self.trackingVelocityIntegral[0] * self.K_I
            velNorthInt = self.trackingVelocityIntegral[1] * self.K_I

            # Da cancellare
            msgTmp.east = velEastInt
            msgTmp.north = velNorthInt
            self.trackingVelocityIntegralPub.publish(msgTmp)
            #

            # Differential control
            velEast += velEastDiff
            velNorth += velNorthDiff

            # Integral control
            velEast += velEastInt
            velNorth += velNorthInt

            result.east = velEast
            result.north = velNorth

        # Tracking speed saturation
        speedNorm = math.sqrt(result.east**2 + result.north**2)
        if speedNorm > self.MAX_TRACKING_SPEED:
            result.east *= (self.MAX_TRACKING_SPEED / speedNorm)
            result.north *= (self.MAX_TRACKING_SPEED / speedNorm)

        return result

    # Callbacks
    def timerCallback(self):
        """
        It increments the counters keeping track of the last GNSS position and UWB distance sent by each drone, and it
        publishes the tracking velocity at a given rate.
        """

        for key in self.lastPositionReceivedTimer.keys():
            self.lastPositionReceivedTimer[key] += 1
        for key in self.lastUwbDistanceReceived.keys():
            self.lastUwbDistanceReceived[key] += 1
        msg = self.computeTrackingVelocity()
        self.trackingVelocityPub.publish(msg)

    def swarmCommandCallback(self, request, response):
        """
        @param request: service request
        @param response: service response
        @return: response

        The field "request.operation" describes the mode to be set, and the other fields (defined in "SwarmCommand.srv")
        are used to update the various position setpoint. Then, the response is returned. In case the operation field is
        not recognized, default values are used and hover mode is set to avoid problems.
        """

        response.result = "success"
        if request.operation == "hover":
            self.get_logger().info("Swarm hovering at current location")
            center = self.computeSwarmCenter()
            tmp = VehicleGlobalPosition()
            tmp.lat = center[0]
            tmp.lon = center[1]
            self.goalPosition = tmp
        elif request.operation == "goTo":
            if request.lat not in range(-90, 90) or request.lon not in range(-180, 180):
                self.get_logger().info("The location to be reached is not a valid one!")
                response.result = "failure"
                return response
            # TODO: cosa succede se la location è nell'altro emisfero?
            self.get_logger().info("Swarm moving to (%f, %f)" % (request.lat, request.lon))
            tmp = VehicleGlobalPosition()
            tmp.lat = request.lat
            tmp.lon = request.lon
            self.goalPosition = tmp

        return response

    def anchorsPositionCallback(self, msg, droneId):
        """
        @param msg: message
        @param droneId: drone id

        It saves the GNSS position of the drone indicated by droneId, it resets the related timer, and checks if all the
        drones sent their data already.
        """

        self.anchorsPosition[droneId] = msg
        self.lastPositionReceivedTimer[droneId] = 0
        if not self.anchorsPositionReceived and len(self.anchorsPosition) == self.N:
            self.anchorsPositionReceived = True

    def targetUwbSensorCallback(self, msg):
        """
        @param msg: message

        It saves the distance from the target to the drone denoted by msg.anchor_pose.header.frame_id, and resets the
        relative timer.
        """

        self.targetUwbDistances[int(msg.anchor_pose.header.frame_id)] = msg.range
        self.lastUwbDistanceReceived[int(msg.anchor_pose.header.frame_id)] = 0

    def anchorsReadyForSwarmingCallback(self, msg):
        """
        @param msg: message

        It records that the drone indicated by msg.data is ready for swarming.
        """

        self.anchorsReadyForSwarming[msg.data] = True


def main():
    rclpy.init(args=None)
    node = TrackingVelocityCalculator()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

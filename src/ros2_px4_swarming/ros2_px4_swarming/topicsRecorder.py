#!/usr/bin/env python3
import rclpy
import csv
from functools import partial
from datetime import datetime
import os
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from px4_msgs.msg import Timesync, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleGlobalPosition, VehicleStatus
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ros2_px4_interfaces.msg import DistanceStamped, DistanceStampedArray, UnitVector, UwbSensor, VelocityVector


class TopicsRecorder(Node):
    def __init__(self):
        super().__init__("topicsRecorder")

        # region Parameters
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
        # endregion

        self.startingTime = self.get_clock().now().to_msg()

        today = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        # Csv files path
        csvFilesPath = os.path.join(
            get_package_prefix('ros2_px4_swarming'),
            '..',
            '..',
            'src',
            'ros2_px4_swarming',
            'csvfiles/'
        )

        os.mkdir(os.path.join(csvFilesPath, today))

        swarmInfoFile = open(csvFilesPath + today + "/swarmInfo.csv", "w")
        swarmInfoWriter = csv.writer(swarmInfoFile)
        swarmInfoWriter.writerow([self.N, self.NUM_TARGET])
        swarmInfoFile.close()

        # region Files
        self.interAnchorsDistancesFile = open(csvFilesPath + today + "/interAnchorsDistances.csv", "w")
        self.synchronizationErrorUwbFile = open(csvFilesPath + today + "/synchronizationErrorUwb.csv", "w")
        self.swarmCenterFile = open(csvFilesPath + today + "/swarmCenter.csv", "w")
        self.trackingVelocityFile = open(csvFilesPath + today + "/trackingVelocity.csv", "w")
        self.trackingVelocityProportionalFile = open(csvFilesPath + today + "/trackingVelocityProportional.csv", "w")
        self.trackingVelocityIntegralFile = open(csvFilesPath + today + "/trackingVelocityIntegral.csv", "w")
        self.trackingVelocityDerivativeFile = open(csvFilesPath + today + "/trackingVelocityDerivative.csv", "w")
        self.vehicleLocalPositionFiles = list()
        self.vehicleGlobalPositionFiles = list()
        self.vehicleStatusFiles = list()
        self.uwb_sensorFiles = list()
        self.trajectorySetpointFiles = list()
        for i in range(self.N):
            self.vehicleLocalPositionFiles.append(open(csvFilesPath + today + "/X500_" + str(i) + "_VehicleLocalPosition.csv", "w"))
            self.vehicleGlobalPositionFiles.append(open(csvFilesPath + today + "/X500_" + str(i) + "_VehicleGlobalPosition.csv", "w"))
            self.vehicleStatusFiles.append(open(csvFilesPath + today + "/X500_" + str(i) + "_VehicleStatus.csv", "w"))
            self.uwb_sensorFiles.append(open(csvFilesPath + today + "/uwb_sensor_" + str(i) + ".csv", "w"))
            self.trajectorySetpointFiles.append(open(csvFilesPath + today + "/X500_" + str(i) + "_TrajectorySetpoint.csv", "w"))
        if self.NUM_TARGET != 0:
            self.uwb_sensor_targetFile = open(csvFilesPath + today + "/uwb_sensor_" + str(self.TARGET_ID) + ".csv", "w")
            self.trackingErrorFile = open(csvFilesPath + today + "/trackingError.csv", "w")
            self.targetAnchorsDistancesFile = open(csvFilesPath + today + "/targetAnchorsDistances.csv", "w")
        # endregion

        # region Writers
        self.interAnchorsDistancesWriter = csv.writer(self.interAnchorsDistancesFile)
        self.synchronizationErrorUwbWriter = csv.writer(self.synchronizationErrorUwbFile)
        self.swarmCenterWriter = csv.writer(self.swarmCenterFile)
        self.trackingVelocityWriter = csv.writer(self.trackingVelocityFile)
        self.trackingVelocityProportionalWriter = csv.writer(self.trackingVelocityProportionalFile)
        self.trackingVelocityIntegralWriter = csv.writer(self.trackingVelocityIntegralFile)
        self.trackingVelocityDerivativeWriter = csv.writer(self.trackingVelocityDerivativeFile)
        self.vehicleLocalPositionWriters = list()
        self.vehicleGlobalPositionWriters = list()
        self.vehicleStatusWriters = list()
        self.uwb_sensorWriters = list()
        self.trajectorySetpointWriters = list()
        for i in range(self.N):
            self.vehicleLocalPositionWriters.append(csv.writer(self.vehicleLocalPositionFiles[i]))
            self.vehicleGlobalPositionWriters.append(csv.writer(self.vehicleGlobalPositionFiles[i]))
            self.vehicleStatusWriters.append(csv.writer(self.vehicleStatusFiles[i]))
            self.uwb_sensorWriters.append(csv.writer(self.uwb_sensorFiles[i]))
            self.trajectorySetpointWriters.append(csv.writer(self.trajectorySetpointFiles[i]))
        if self.NUM_TARGET != 0:
            self.uwb_sensor_targetWriter = csv.writer(self.uwb_sensor_targetFile)
            self.trackingErrorWriter = csv.writer(self.trackingErrorFile)
            self.targetAnchorsDistancesWriter = csv.writer(self.targetAnchorsDistancesFile)
        # endregion

        # Useful variables
        self.subscribers = list()

        self.subscribers.append(self.create_subscription(DistanceStampedArray, "/performanceAnalyzer/interAnchorsDistances", self.interAnchorsDistancesCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(Float64, "/performanceAnalyzer/synchronizationErrorUwb", self.synchronizationErrorUwbCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(Odometry, "/performanceAnalyzer/swarmCenter", self.swarmCenterCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocity", self.trackingVelocityCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocityProportional", self.trackingVelocityProportionalCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocityIntegral", self.trackingVelocityIntegralCallback, self.QUEUE_SIZE))
        self.subscribers.append(self.create_subscription(VelocityVector, "/trackingVelocityCalculator/trackingVelocityDerivative", self.trackingVelocityDerivativeCallback, self.QUEUE_SIZE))
        for i in range(self.N):
            self.subscribers.append(self.create_subscription(VehicleLocalPosition, "/X500_" + str(i) + "/VehicleLocalPosition_PubSubTopic", partial(self.vehicleLocalPositionCallback, droneId=i), self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(VehicleGlobalPosition, "/X500_" + str(i) + "/VehicleGlobalPosition_PubSubTopic", partial(self.vehicleGlobalPositionCallback, droneId=i), self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(VehicleStatus, "/X500_" + str(i) + "/VehicleStatus_PubSubTopic", partial(self.vehicleStatusCallback, droneId=i), self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(UwbSensor, "/uwb_sensor_" + str(i), partial(self.uwb_sensorCallback, droneId=i), self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(TrajectorySetpoint, "/X500_" + str(i) + "/TrajectorySetpoint_PubSubTopic", partial(self.trajectorySetpointCallback, droneId=i), self.QUEUE_SIZE))
        if self.NUM_TARGET != 0:
            self.subscribers.append(self.create_subscription(UwbSensor, "/uwb_sensor_" + str(self.TARGET_ID), partial(self.uwb_sensorCallback, droneId=self.TARGET_ID), self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(DistanceStamped, "/performanceAnalyzer/trackingError", self.trackingErrorCallback, self.QUEUE_SIZE))
            self.subscribers.append(self.create_subscription(DistanceStampedArray, "/performanceAnalyzer/targetAnchorsDistances", self.targetAnchorsDistancesCallback, self.QUEUE_SIZE))

    def __del__(self):
        self.interAnchorsDistancesFile.close()
        self.synchronizationErrorUwbFile.close()
        self.swarmCenterFile.close()
        self.trackingVelocityFile.close()
        self.trackingVelocityProportionalFile.close()
        self.trackingVelocityIntegralFile.close()
        self.trackingVelocityDerivativeFile.close()
        for i in range(self.N):
            self.vehicleLocalPositionFiles[i].close()
            self.vehicleGlobalPositionFiles[i].close()
            self.vehicleStatusFiles[i].close()
            self.uwb_sensorFiles[i].close()
        if self.NUM_TARGET != 0:
            self.uwb_sensor_targetFile.close()
            self.trackingErrorFile.close()
            self.targetAnchorsDistancesFile.close()

    # region Auxiliary methods
    def getTime(self):
        currentTime = self.get_clock().now().to_msg()
        return (currentTime.sec - self.startingTime.sec) + (currentTime.nanosec - self.startingTime.nanosec) * 1e-9
    # endregion

    # region Callbacks
    def interAnchorsDistancesCallback(self, msg):
        data = [
            self.getTime(),
        ]
        for i in range(len(msg.data)):
            data.append(msg.data[i].destination_drone_id)
            data.append(msg.data[i].start_drone_id)
            data.append(msg.data[i].distance)
        self.interAnchorsDistancesWriter.writerow(data)

    def synchronizationErrorUwbCallback(self, msg):
        data = [
            self.getTime(),
            msg.data
        ]
        self.synchronizationErrorUwbWriter.writerow(data)

    def swarmCenterCallback(self, msg):
        data = [
            self.getTime(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        self.swarmCenterWriter.writerow(data)

    def trackingVelocityCallback(self, msg):
        data = [
            self.getTime(),
            msg.east,
            msg.north
        ]
        self.trackingVelocityWriter.writerow(data)

    def trackingVelocityProportionalCallback(self, msg):
        data = [
            self.getTime(),
            msg.east,
            msg.north
        ]
        self.trackingVelocityProportionalWriter.writerow(data)

    def trackingVelocityIntegralCallback(self, msg):
        data = [
            self.getTime(),
            msg.east,
            msg.north
        ]
        self.trackingVelocityIntegralWriter.writerow(data)

    def trackingVelocityDerivativeCallback(self, msg):
        data = [
            self.getTime(),
            msg.east,
            msg.north
        ]
        self.trackingVelocityDerivativeWriter.writerow(data)

    def vehicleLocalPositionCallback(self, msg, droneId):
        data = [
            self.getTime(),
            msg.x,
            msg.y,
            msg.z,
            msg.ref_lat,
            msg.ref_lon,
            msg.ref_alt
        ]
        self.vehicleLocalPositionWriters[droneId].writerow(data)

    def vehicleGlobalPositionCallback(self, msg, droneId):
        data = [
            self.getTime(),
            msg.lat,
            msg.lon,
            msg.alt
        ]
        self.vehicleGlobalPositionWriters[droneId].writerow(data)

    def vehicleStatusCallback(self, msg, droneId):
        data = [
            self.getTime(),
            msg.nav_state,
            msg.latest_disarming_reason
        ]
        self.vehicleStatusWriters[droneId].writerow(data)

    def uwb_sensorCallback(self, msg, droneId):
        data = [
            self.getTime(),
            msg.range,
            int(msg.anchor_pose.header.frame_id)
        ]
        if droneId == self.TARGET_ID:
            self.uwb_sensor_targetWriter.writerow(data)
        else:
            self.uwb_sensorWriters[droneId].writerow(data)

    def trajectorySetpointCallback(self, msg, droneId):
        data = [
            self.getTime(),
            msg.x,
            msg.y,
            msg.z,
            msg.yaw,
            msg.yawspeed,
            msg.vx,
            msg.vy,
            msg.vz
        ]
        self.trajectorySetpointWriters[droneId].writerow(data)

    def trackingErrorCallback(self, msg):
        data = [
            self.getTime(),
            msg.distance
        ]
        self.trackingErrorWriter.writerow(data)

    def targetAnchorsDistancesCallback(self, msg):
        data = [
            self.getTime(),
        ]
        for i in range(len(msg.data)):
            data.append(msg.data[i].destination_drone_id)
            data.append(msg.data[i].start_drone_id)
            data.append(msg.data[i].distance)
        self.targetAnchorsDistancesWriter.writerow(data)
    # endregion


def main():
    rclpy.init(args=None)
    node = TopicsRecorder()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

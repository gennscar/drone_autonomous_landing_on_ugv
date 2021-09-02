#!/usr/bin/env python3
import math
import sys
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt


class BagFileParser:
    def __init__(self, bagFile, bagFolder):
        self.folderName = bagFolder

        # Open the db3 connection
        self.connection = sqlite3.connect(bagFile)
        self.cursor = self.connection.cursor()

        # Create a message type map
        topicsInfo = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topicType = {topicName: topicType for topicId, topicName, topicType in topicsInfo}
        self.topicId = {topicName: topicId for topicId, topicName, topicType in topicsInfo}
        self.topicMessages = {topicName: get_message(topicType) for topicId, topicName, topicType in topicsInfo}

    def __del__(self):
        self.connection.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def getMessages(self, topicName):
        try:
            topicId = self.topicId[topicName]
        except KeyError:
            return list()
        # Get the data from the db3
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = " + str(topicId)).fetchall()
        # Deserialize all and timestamp them
        return [(timestamp, deserialize_message(data, self.topicMessages[topicName])) for timestamp, data in rows]

    def plotTopic(self, topicName):
        topicType = self.topicType[topicName]
        data = parser.getMessages(topicName)
        plt.figure()

        timeLabel = "$Time \ [s]$"
        legend = []

        if topicType == "drone_interfaces/msg/DistanceStamped":
            plt.plot(list((data[i][0] - data[0][0]) * 1e-9 for i in range(len(data))),
                     list(data[i][1].distance for i in range(len(data))))
            plt.xlabel(timeLabel)
            if topicName == "/performanceAnalyzer/trackingError":
                plt.title("Tracking error [m]", size="xx-large", weight="bold")

        elif topicType == "drone_interfaces/msg/DistanceStampedArray":
            for couple in range(len(data[0][1].data)):
                plt.plot(list((data[i][0] - data[0][0]) * 1e-9 for i in range(len(data))),
                         list(data[i][1].data[couple].distance for i in range(len(data))))
                legend.append("$d_{" + str(data[0][1].data[couple].destination_drone_id) + str(data[0][1].data[couple].start_drone_id) + "}$")
            plt.xlabel(timeLabel)
            if topicName == "/performanceAnalyzer/interAnchorsDistances":
                plt.title("Inter-anchors distances [m]", size="xx-large", weight="bold")

        plt.grid()
        if legend:
            plt.legend(legend)

    def plotTrackingError(self):
        trackingError = self.getMessages("/performanceAnalyzer/trackingError")
        trackingVelocityProportional = self.getMessages("/trackingVelocityCalculator/trackingVelocityProportional")
        plt.figure()
        legend = list()

        plt.plot(list((trackingError[i][0] - trackingError[0][0]) * 1e-9 for i in range(len(trackingError))),
                 list(trackingError[i][1].distance for i in range(len(trackingError))))
        legend.append("$Tracking \ error$")
        if trackingVelocityProportional:
            plt.plot(list((trackingVelocityProportional[i][0] - trackingError[0][0]) * 1e-9 for i in range(len(trackingVelocityProportional))),
                     list(math.sqrt(trackingVelocityProportional[i][1].east**2 + trackingVelocityProportional[i][1].north**2) for i in range(len(trackingVelocityProportional))))
            legend.append("$Tracking \ velocity \ (P)$")

        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Tracking error [m]", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "TrackingError.png")

    def plotInterAnchorsDistances(self):
        interAnchorsDistances = self.getMessages("/performanceAnalyzer/interAnchorsDistances")
        plt.figure()
        legend = list()

        for couple in range(len(interAnchorsDistances[0][1].data)):
            plt.plot(list((interAnchorsDistances[i][0] - interAnchorsDistances[0][0]) * 1e-9 for i in range(len(interAnchorsDistances))),
                     list(interAnchorsDistances[i][1].data[couple].distance for i in range(len(interAnchorsDistances))))
            legend.append("$d_{" + str(interAnchorsDistances[0][1].data[couple].destination_drone_id) + str(interAnchorsDistances[0][1].data[couple].start_drone_id) + "}$")

        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Inter-anchors distances [m]", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "InterAnchorsDistances.png")

    def plotTargetUwbDistances(self):
        targetUwbDistances = self.getMessages("/uwb_sensor_200")
        dronesTargetUwbDistances = {}
        for message in targetUwbDistances:
            if int(message[1].anchor_pose.header.frame_id) in dronesTargetUwbDistances.keys():
                dronesTargetUwbDistances[int(message[1].anchor_pose.header.frame_id)].append(message)
            else:
                dronesTargetUwbDistances[int(message[1].anchor_pose.header.frame_id)] = list()
                dronesTargetUwbDistances[int(message[1].anchor_pose.header.frame_id)].append(message)

        plt.figure()
        legend = list()

        for drone in range(len(dronesTargetUwbDistances)):
            plt.plot(list((dronesTargetUwbDistances[drone][i][0] - dronesTargetUwbDistances[drone][0][0]) * 1e-9 for i in range(0, len(dronesTargetUwbDistances[drone]))),
                     list(dronesTargetUwbDistances[drone][i][1].range for i in range(0, len(dronesTargetUwbDistances[drone]))))
            legend.append("$Drone_" + str(drone) + "$")

        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("UWB drone-target distances", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "TargetUwbDistances.png")

    def plotTrajectories(self):
        targetTrajectory = self.getMessages("/targetRover/GroundTruth/odom")
        dronesTrajectory = list()
        for i in range(10):
            tmp = self.getMessages("/X500_" + str(i) + "/GroundTruth/odom")
            if tmp:
                dronesTrajectory.append(tmp)
        fig = plt.figure().add_subplot(111, projection="3d")
        legend = list()

        if targetTrajectory:
            fig.plot(list(targetTrajectory[i][1].pose.pose.position.x for i in range(len(targetTrajectory))),
                     list(targetTrajectory[i][1].pose.pose.position.y for i in range(len(targetTrajectory))),
                     list(targetTrajectory[i][1].pose.pose.position.z for i in range(len(targetTrajectory))),
                     "--r")
            legend.append("$Target$")
        if dronesTrajectory:
            for drone in range(len(dronesTrajectory)):
                fig.plot(list(dronesTrajectory[drone][i][1].pose.pose.position.x for i in range(len(dronesTrajectory[drone]))),
                         list(dronesTrajectory[drone][i][1].pose.pose.position.y for i in range(len(dronesTrajectory[drone]))),
                         list(dronesTrajectory[drone][i][1].pose.pose.position.z for i in range(len(dronesTrajectory[drone]))))
                legend.append("$Drone_" + str(drone) + "$")

        plt.grid()
        plt.xlabel("$x \ [m]$")
        plt.ylabel("$y \ [m]$")
        plt.legend(legend)
        plt.title("Vehicles' trajectory", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "Trajectories.png")

    def plotAltitudes(self):
        dronesAltitude = list()
        for i in range(10):
            tmp = self.getMessages("/X500_" + str(i) + "/GroundTruth/odom")
            if tmp:
                dronesAltitude.append(tmp)
        plt.figure()
        legend = list()

        if dronesAltitude:
            for drone in range(len(dronesAltitude)):
                plt.plot(list((dronesAltitude[drone][i][0] - dronesAltitude[drone][0][0]) * 1e-9 for i in range(0, len(dronesAltitude[drone]))),
                         list(dronesAltitude[drone][i][1].pose.pose.position.z for i in range(0, len(dronesAltitude[drone]))))
                legend.append("$Drone_" + str(drone) + "$")

        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Drones' altitude", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "Altitudes.png")

    def plotTrackingVelocity(self):
        trackingVelocity = self.getMessages("/trackingVelocityCalculator/trackingVelocity")
        trackingVelocityProportional = self.getMessages("/trackingVelocityCalculator/trackingVelocityProportional")
        trackingVelocityIntegral = self.getMessages("/trackingVelocityCalculator/trackingVelocityIntegral")
        trackingVelocityDerivative = self.getMessages("/trackingVelocityCalculator/trackingVelocityDerivative")
        plt.figure()

        legend = list()
        plt.subplot(211)
        plt.plot(list((trackingVelocity[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocity))),
                 list(trackingVelocity[i][1].east for i in range(len(trackingVelocity))))
        legend.append("$PID$")
        if trackingVelocityProportional:
            plt.plot(list((trackingVelocityProportional[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityProportional))),
                     list(trackingVelocityProportional[i][1].east for i in range(len(trackingVelocityProportional))))
            legend.append("$P$")
        if trackingVelocityIntegral:
            plt.plot(list((trackingVelocityIntegral[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityIntegral))),
                     list(trackingVelocityIntegral[i][1].east for i in range(len(trackingVelocityIntegral))))
            legend.append("$I$")
        if trackingVelocityDerivative:
            plt.plot(list((trackingVelocityDerivative[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityDerivative))),
                     list(trackingVelocityDerivative[i][1].east for i in range(len(trackingVelocityDerivative))))
            legend.append("$D$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_{track, east} \ [m/s]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(212)
        plt.plot(list((trackingVelocity[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocity))),
                 list(trackingVelocity[i][1].north for i in range(len(trackingVelocity))))
        legend.append("$PID$")
        if trackingVelocityProportional:
            plt.plot(list((trackingVelocityProportional[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityProportional))),
                     list(trackingVelocityProportional[i][1].north for i in range(len(trackingVelocityProportional))))
            legend.append("$P$")
        if trackingVelocityIntegral:
            plt.plot(list((trackingVelocityIntegral[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityIntegral))),
                     list(trackingVelocityIntegral[i][1].north for i in range(len(trackingVelocityIntegral))))
            legend.append("$I$")
        if trackingVelocityDerivative:
            plt.plot(list((trackingVelocityDerivative[i][0] - trackingVelocity[0][0]) * 1e-9 for i in range(len(trackingVelocityDerivative))),
                     list(trackingVelocityDerivative[i][1].north for i in range(len(trackingVelocityDerivative))))
            legend.append("$D$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_{track, north} \ [m/s]$")
        plt.legend(legend)

        plt.suptitle("Tracking velocity [m/s]", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "TrackingVelocity.png")

    def plotVelocities(self):
        targetVelocity = self.getMessages("/targetRover/GroundTruth/odom")
        dronesVelocity = list()
        for i in range(10):
            tmp = self.getMessages("/X500_" + str(i) + "/GroundTruth/odom")
            if tmp:
                dronesVelocity.append(tmp)
        plt.figure()
        legend = list()

        plt.subplot(311)
        if targetVelocity:
            plt.plot(list((targetVelocity[i][0] - targetVelocity[0][0]) * 1e-9 for i in range(len(targetVelocity))),
                     list(targetVelocity[i][1].twist.twist.linear.x for i in range(len(targetVelocity))),
                     "--r")
            legend.append("$Target$")
        if dronesVelocity:
            for drone in range(len(dronesVelocity)):
                plt.plot(list((dronesVelocity[drone][i][0] - dronesVelocity[drone][0][0]) * 1e-9 for i in range(len(dronesVelocity[drone]))),
                         list(dronesVelocity[drone][i][1].twist.twist.linear.x for i in range(len(dronesVelocity[drone]))))
                legend.append("$Drone_" + str(drone) + "$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_x \ [m/s]$")
        plt.legend(legend)

        plt.subplot(312)
        if targetVelocity:
            plt.plot(list((targetVelocity[i][0] - targetVelocity[0][0]) * 1e-9 for i in range(len(targetVelocity))),
                     list(targetVelocity[i][1].twist.twist.linear.y for i in range(len(targetVelocity))),
                     "--r")
            legend.append("$Target$")
        if dronesVelocity:
            for drone in range(len(dronesVelocity)):
                plt.plot(list((dronesVelocity[drone][i][0] - dronesVelocity[drone][0][0]) * 1e-9 for i in range(len(dronesVelocity[drone]))),
                         list(dronesVelocity[drone][i][1].twist.twist.linear.y for i in range(len(dronesVelocity[drone]))))
                legend.append("$Drone_" + str(drone) + "$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_y \ [m/s]$")
        plt.legend(legend)

        plt.subplot(313)
        if targetVelocity:
            plt.plot(list((targetVelocity[i][0] - targetVelocity[0][0]) * 1e-9 for i in range(len(targetVelocity))),
                     list(targetVelocity[i][1].twist.twist.linear.z for i in range(len(targetVelocity))),
                     "--r")
            legend.append("$Target$")
        if dronesVelocity:
            for drone in range(len(dronesVelocity)):
                plt.plot(list((dronesVelocity[drone][i][0] - dronesVelocity[drone][0][0]) * 1e-9 for i in range(len(dronesVelocity[drone]))),
                         list(dronesVelocity[drone][i][1].twist.twist.linear.z for i in range(len(dronesVelocity[drone]))))
                legend.append("$Drone_" + str(drone) + "$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_z \ [m/s]$")
        plt.legend(legend)

        plt.suptitle("Vehicles' velocity [m/s]", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "Velocities.png")

    def plotPositions(self):
        targetPosition = self.getMessages("/targetRover/GroundTruth/odom")
        swarmCenter = self.getMessages("/performanceAnalyzer/swarmCenter")
        plt.figure()
        legend = list()

        plt.subplot(311)
        if targetPosition:
            plt.plot(list((targetPosition[i][0] - targetPosition[0][0]) * 1e-9 for i in range(len(targetPosition))),
                     list(targetPosition[i][1].pose.pose.position.x for i in range(len(targetPosition))),
                     "--r")
            legend.append("$Target$")
        if swarmCenter:
            plt.plot(list((swarmCenter[i][0] - swarmCenter[0][0]) * 1e-9 for i in range(len(swarmCenter))),
                     list(swarmCenter[i][1].pose.pose.position.x for i in range(len(swarmCenter))))
            legend.append("$Swarm \ center$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$x \ [m]$")
        plt.legend(legend)

        plt.subplot(312)
        if targetPosition:
            plt.plot(list((targetPosition[i][0] - targetPosition[0][0]) * 1e-9 for i in range(len(targetPosition))),
                     list(targetPosition[i][1].pose.pose.position.y for i in range(len(targetPosition))),
                     "--r")
            legend.append("$Target$")
        if swarmCenter:
            plt.plot(list((swarmCenter[i][0] - swarmCenter[0][0]) * 1e-9 for i in range(len(swarmCenter))),
                     list(swarmCenter[i][1].pose.pose.position.y for i in range(len(swarmCenter))))
            legend.append("$Swarm \ center$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$y \ [m]$")
        plt.legend(legend)

        plt.subplot(313)
        if targetPosition:
            plt.plot(list((targetPosition[i][0] - targetPosition[0][0]) * 1e-9 for i in range(len(targetPosition))),
                     list(targetPosition[i][1].pose.pose.position.z for i in range(len(targetPosition))),
                     "--r")
            legend.append("$Target$")
        if swarmCenter:
            plt.plot(list((swarmCenter[i][0] - swarmCenter[0][0]) * 1e-9 for i in range(len(swarmCenter))),
                     list(swarmCenter[i][1].pose.pose.position.z for i in range(len(swarmCenter))))
            legend.append("$Swarm \ center$")
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$z \ [m]$")
        plt.legend(legend)

        plt.suptitle("Positions [m]", size="xx-large", weight="bold")

        plt.savefig(self.folderName + "Positions.png")

    def plotSynchronizationErrorUwb(self):
        syncErrUwb = self.getMessages("/performanceAnalyzer/synchronizationErrorUwb")
        plt.figure()

        plt.plot(list((syncErrUwb[i][0] - syncErrUwb[0][0]) * 1e-9 for i in range(len(syncErrUwb))),
                 list(syncErrUwb[i][1].data for i in range(len(syncErrUwb))))

        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.title("Synchronization error UWB [s]", size="xx-large", weight="bold")

        # plt.gcf().set_size_inches((13.6, 7.65), forward=False)
        plt.savefig(self.folderName + "SynchronizationError.png")


if __name__ == "__main__":
    folder = "../bagfiles/"
    dbName = sys.argv[1]
    if dbName[len(dbName) - 1] == "/":
        dbName = dbName[0: len(dbName) - 1]
    folderName = folder + dbName + "/"
    bagFileName = folderName + dbName + "_0.db3"

    parser = BagFileParser(bagFileName, folderName)

    parser.plotTrackingError()
    parser.plotInterAnchorsDistances()
    parser.plotTargetUwbDistances()
    parser.plotTrajectories()
    parser.plotAltitudes()
    parser.plotTrackingVelocity()
    parser.plotVelocities()
    parser.plotPositions()
    parser.plotSynchronizationErrorUwb()

    plt.show()

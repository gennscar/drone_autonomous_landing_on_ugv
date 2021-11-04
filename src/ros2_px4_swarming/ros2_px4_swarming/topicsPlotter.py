#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import csv
import os
from ament_index_python.packages import get_package_prefix
from utm import from_latlon


TARGET_ID = 200


def main():
    # Csv files path
    csvFilesPath = os.path.join(
        get_package_prefix('ros2_px4_swarming'),
        '..',
        '..',
        'src',
        'ros2_px4_swarming',
        'csvfiles/'
    )

    folderName = sys.argv[1]

    swarmInfoFile = open(csvFilesPath + folderName + "/swarmInfo.csv", "r")
    swarmInfoReader = csv.reader(swarmInfoFile, delimiter=",")
    tmp = list()
    for row in swarmInfoReader:
        for field in row:
            tmp.append(int(field))
    N = tmp[0]
    NUM_TARGET = tmp[1]
    swarmInfoFile.close()

    # region Files
    interAnchorsDistancesFile = open(csvFilesPath + folderName + "/interAnchorsDistances.csv", "r")
    synchronizationErrorUwbFile = open(csvFilesPath + folderName + "/synchronizationErrorUwb.csv", "r")
    swarmCenterFile = open(csvFilesPath + folderName + "/swarmCenter.csv", "r")
    trackingVelocityFile = open(csvFilesPath + folderName + "/trackingVelocity.csv", "r")
    trackingVelocityProportionalFile = open(csvFilesPath + folderName + "/trackingVelocityProportional.csv", "r")
    trackingVelocityIntegralFile = open(csvFilesPath + folderName + "/trackingVelocityIntegral.csv", "r")
    trackingVelocityDerivativeFile = open(csvFilesPath + folderName + "/trackingVelocityDerivative.csv", "r")
    vehicleLocalPositionFiles = list()
    vehicleGlobalPositionFiles = list()
    vehicleStatusFiles = list()
    uwb_sensorFiles = list()
    trajectorySetpointFiles = list()
    for i in range(N):
        vehicleLocalPositionFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleLocalPosition.csv", "r"))
        vehicleGlobalPositionFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleGlobalPosition.csv", "r"))
        vehicleStatusFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleStatus.csv", "r"))
        uwb_sensorFiles.append(open(csvFilesPath + folderName + "/uwb_sensor_" + str(i) + ".csv", "r"))
        trajectorySetpointFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_TrajectorySetpoint.csv", "r"))
    if NUM_TARGET != 0:
        uwb_sensor_targetFile = open(csvFilesPath + folderName + "/uwb_sensor_" + str(TARGET_ID) + ".csv", "r")
        trackingErrorFile = open(csvFilesPath + folderName + "/trackingError.csv", "r")
        targetAnchorsDistancesFile = open(csvFilesPath + folderName + "/targetAnchorsDistances.csv", "r")
    # endregion

    # region Writers
    interAnchorsDistancesReader = csv.reader(interAnchorsDistancesFile, delimiter=",")
    synchronizationErrorUwbReader = csv.reader(synchronizationErrorUwbFile, delimiter=",")
    swarmCenterReader = csv.reader(swarmCenterFile, delimiter=",")
    trackingVelocityReader = csv.reader(trackingVelocityFile, delimiter=",")
    trackingVelocityProportionalReader = csv.reader(trackingVelocityProportionalFile, delimiter=",")
    trackingVelocityIntegralReader = csv.reader(trackingVelocityIntegralFile, delimiter=",")
    trackingVelocityDerivativeReader = csv.reader(trackingVelocityDerivativeFile, delimiter=",")
    vehicleLocalPositionReaders = list()
    vehicleGlobalPositionReaders = list()
    vehicleStatusReaders = list()
    uwb_sensorReaders = list()
    trajectorySetpointReaders = list()
    for i in range(N):
        vehicleLocalPositionReaders.append(csv.reader(vehicleLocalPositionFiles[i], delimiter=","))
        vehicleGlobalPositionReaders.append(csv.reader(vehicleGlobalPositionFiles[i], delimiter=","))
        vehicleStatusReaders.append(csv.reader(vehicleStatusFiles[i], delimiter=","))
        uwb_sensorReaders.append(csv.reader(uwb_sensorFiles[i], delimiter=","))
        trajectorySetpointReaders.append(csv.reader(trajectorySetpointFiles[i], delimiter=","))
    if NUM_TARGET != 0:
        uwb_sensor_targetReader = csv.reader(uwb_sensor_targetFile, delimiter=",")
        trackingErrorReader = csv.reader(trackingErrorFile, delimiter=",")
        targetAnchorsDistancesReader = csv.reader(targetAnchorsDistancesFile, delimiter=",")
    # endregion

    # region Data structures
    interAnchorsDistances = list()
    synchronizationErrorUwb = list()
    swarmCenter = list()
    trackingVelocity = list()
    trackingVelocityProportional = list()
    trackingVelocityIntegral = list()
    trackingVelocityDerivative = list()
    vehicleLocalPosition = list()
    vehicleGlobalPosition = list()
    vehicleStatus = list()
    uwb_sensor = list()
    trajectorySetpoint = list()
    uwb_sensor_target = list()
    trackingError = list()
    targetAnchorsDistances = list()
    interAnchorsGPSDistances = list()
    # endregion

    # region Data retrieval
    for row in interAnchorsDistancesReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        interAnchorsDistances.append(tmp)

    for row in synchronizationErrorUwbReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        synchronizationErrorUwb.append(tmp)

    for row in swarmCenterReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        swarmCenter.append(tmp)

    for row in trackingVelocityReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        trackingVelocity.append(tmp)

    for row in trackingVelocityProportionalReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        trackingVelocityProportional.append(tmp)

    for row in trackingVelocityIntegralReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        trackingVelocityIntegral.append(tmp)

    for row in trackingVelocityDerivativeReader:
        tmp = list()
        for field in row:
            tmp.append(float(field))
        trackingVelocityDerivative.append(tmp)

    for i in range(N):
        vehicleLocalPosition.append(list())
        for row in vehicleLocalPositionReaders[i]:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            vehicleLocalPosition[i].append(tmp)

        vehicleGlobalPosition.append(list())
        for row in vehicleGlobalPositionReaders[i]:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            utmCoord = from_latlon(tmp[1], tmp[2])
            vec = [tmp[0], utmCoord[0], utmCoord[1], tmp[3]]
            vehicleGlobalPosition[i].append(vec)

        vehicleStatus.append(list())
        for row in vehicleStatusReaders[i]:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            vehicleStatus[i].append(tmp)

        uwb_sensor.append(list())
        for row in uwb_sensorReaders[i]:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            uwb_sensor[i].append(tmp)

        trajectorySetpoint.append(list())
        for row in trajectorySetpointReaders[i]:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            trajectorySetpoint[i].append(tmp)

    if NUM_TARGET != 0:
        for row in uwb_sensor_targetReader:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            uwb_sensor_target.append(tmp)

        for row in trackingErrorReader:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            trackingError.append(tmp)

        for row in targetAnchorsDistancesReader:
            tmp = list()
            for field in row:
                tmp.append(float(field))
            targetAnchorsDistances.append(tmp)

    if len(vehicleGlobalPosition) > 1:
        counter = 0
        for i in range(len(vehicleGlobalPosition)):
            for j in range(i + 1, len(vehicleGlobalPosition)):
                interAnchorsGPSDistances.append(list())
                counter += 1
                for indexI in range(len(vehicleGlobalPosition[i])):
                    listJ = np.asarray(list(vehicleGlobalPosition[j][tmpIndex][0] for tmpIndex in range(len(vehicleGlobalPosition[j]))))
                    indexJ = (np.abs(listJ - vehicleGlobalPosition[i][indexI][0])).argmin()
                    # Check if the time difference is small enough
                    if math.isclose(vehicleGlobalPosition[i][indexI][0], vehicleGlobalPosition[j][indexJ][0], abs_tol=0.1):
                        interAnchorsGPSDistances[counter - 1].append([
                            np.average([vehicleGlobalPosition[i][indexI][0], vehicleGlobalPosition[j][indexJ][0]]),
                            np.linalg.norm([vehicleGlobalPosition[i][indexI][1] - vehicleGlobalPosition[j][indexJ][1], vehicleGlobalPosition[i][indexI][2] - vehicleGlobalPosition[j][indexJ][2]])])
    # endregion

    # region Time analysis
    minTimeList = list()
    maxTimeList = list()

    if interAnchorsDistances:
        minTimeList.append(interAnchorsDistances[0][0])
        maxTimeList.append(interAnchorsDistances[len(interAnchorsDistances) - 1][0])
    if synchronizationErrorUwb:
        minTimeList.append(synchronizationErrorUwb[0][0])
        maxTimeList.append(synchronizationErrorUwb[len(synchronizationErrorUwb) - 1][0])
    if swarmCenter:
        minTimeList.append(swarmCenter[0][0])
        maxTimeList.append(swarmCenter[len(swarmCenter) - 1][0])
    if trackingVelocity:
        minTimeList.append(trackingVelocity[0][0])
        maxTimeList.append(trackingVelocity[len(trackingVelocity) - 1][0])
    if trackingVelocityProportional:
        minTimeList.append(trackingVelocityProportional[0][0])
        maxTimeList.append(trackingVelocityProportional[len(trackingVelocityProportional) - 1][0])
    if trackingVelocityIntegral:
        minTimeList.append(trackingVelocityIntegral[0][0])
        maxTimeList.append(trackingVelocityIntegral[len(trackingVelocityIntegral) - 1][0])
    if trackingVelocityDerivative:
        minTimeList.append(trackingVelocityDerivative[0][0])
        maxTimeList.append(trackingVelocityDerivative[len(trackingVelocityDerivative) - 1][0])
    if uwb_sensor_target:
        minTimeList.append(uwb_sensor_target[0][0])
        maxTimeList.append(uwb_sensor_target[len(uwb_sensor_target) - 1][0])
    if trackingError:
        minTimeList.append(trackingError[0][0])
        maxTimeList.append(trackingError[len(trackingError) - 1][0])
    if targetAnchorsDistances:
        minTimeList.append(targetAnchorsDistances[0][0])
        maxTimeList.append(targetAnchorsDistances[len(targetAnchorsDistances) - 1][0])

    for i in range(N):
        if vehicleLocalPosition[i]:
            minTimeList.append(vehicleLocalPosition[i][0][0])
            maxTimeList.append(vehicleLocalPosition[i][len(vehicleLocalPosition) - 1][0])
        if vehicleGlobalPosition[i]:
            minTimeList.append(vehicleGlobalPosition[i][0][0])
            maxTimeList.append(vehicleGlobalPosition[i][len(vehicleGlobalPosition) - 1][0])
        if vehicleStatus[i]:
            minTimeList.append(vehicleStatus[i][0][0])
            maxTimeList.append(vehicleStatus[i][len(vehicleStatus) - 1][0])
        if trajectorySetpoint[i]:
            minTimeList.append(trajectorySetpoint[i][0][0])
            maxTimeList.append(trajectorySetpoint[i][len(trajectorySetpoint) - 1][0])

    timeLimits = [min(minTimeList), max(maxTimeList)]
    # endregion

    # region Plotters
    # region Inter-anchors distances
    if interAnchorsDistances:
        plt.figure()
        legend = list()

        for couple in range(int((len(interAnchorsDistances[0]) - 1) / 3)):
            plt.plot(list(interAnchorsDistances[i][0] for i in range(len(interAnchorsDistances))),
                     list(interAnchorsDistances[i][(couple + 1) * 3] for i in range(len(interAnchorsDistances))))
            legend.append("$d_{" + str(int(interAnchorsDistances[0][1 + couple * 3])) + str(int(interAnchorsDistances[0][2 + couple * 3])) + "}$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Inter-anchors distances [m]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/interAnchorsDistances.png")
    # endregion

    # region Inter-anchors GPS distances
    if interAnchorsGPSDistances:
        plt.figure()
        legend = list()

        counters = [0, 0]
        for couple in range(len(interAnchorsGPSDistances)):
            plt.plot(list(interAnchorsGPSDistances[couple][i][0] for i in range(len(interAnchorsGPSDistances[couple]))),
                     list(interAnchorsGPSDistances[couple][i][1] for i in range(len(interAnchorsGPSDistances[couple]))))
            if counters[1] < N - 1:
                counters[1] += 1
            else:
                counters[0] += 1
                counters[1] = counters[0] + 1
            legend.append("$d_{" + str(counters[0]) + str(counters[1]) + "}$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Inter-anchors GPS distances [m]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/interAnchorsGPSDistances.png")
    # endregion

    # region Synchronization error UWB
    if synchronizationErrorUwb:
        plt.figure()
        legend = list()

        plt.hist(list(synchronizationErrorUwb[i][1] for i in range(len(synchronizationErrorUwb))), bins=50, density=True)

        # plt.plot(list(synchronizationErrorUwb[i][0] for i in range(len(synchronizationErrorUwb))),
        #          list(synchronizationErrorUwb[i][1] for i in range(len(synchronizationErrorUwb))))

        # plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$\Delta t \ [s]$")
        plt.ylabel("$Frequency$")
        plt.legend(legend)
        plt.title("Maximum synchronization error UWB [s]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/synchronizationErrorUwb.png")
    # endregion

    # region Tracking velocity
    if trackingVelocity:
        plt.figure()
        legend = list()
        plt.subplot(211)

        plt.plot(list(trackingVelocity[i][0] for i in range(len(trackingVelocity))),
                 list(trackingVelocity[i][1] for i in range(len(trackingVelocity))))
        legend.append("$PID$")
        if trackingVelocityProportional:
            plt.plot(list(trackingVelocityProportional[i][0] for i in range(len(trackingVelocityProportional))),
                     list(trackingVelocityProportional[i][1] for i in range(len(trackingVelocityProportional))))
            legend.append("$P$")
        if trackingVelocityIntegral:
            plt.plot(list(trackingVelocityIntegral[i][0] for i in range(len(trackingVelocityIntegral))),
                     list(trackingVelocityIntegral[i][1] for i in range(len(trackingVelocityIntegral))))
            legend.append("$I$")
        if trackingVelocityDerivative:
            plt.plot(list(trackingVelocityDerivative[i][0] for i in range(len(trackingVelocityDerivative))),
                     list(trackingVelocityDerivative[i][1] for i in range(len(trackingVelocityDerivative))))
            legend.append("$D$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_{track, east} \ [m/s]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(212)

        plt.plot(list(trackingVelocity[i][0] for i in range(len(trackingVelocity))),
                 list(trackingVelocity[i][2] for i in range(len(trackingVelocity))))
        legend.append("$PID$")
        if trackingVelocityProportional:
            plt.plot(list(trackingVelocityProportional[i][0] for i in range(len(trackingVelocityProportional))),
                     list(trackingVelocityProportional[i][2] for i in range(len(trackingVelocityProportional))))
            legend.append("$P$")
        if trackingVelocityIntegral:
            plt.plot(list(trackingVelocityIntegral[i][0] for i in range(len(trackingVelocityIntegral))),
                     list(trackingVelocityIntegral[i][2] for i in range(len(trackingVelocityIntegral))))
            legend.append("$I$")
        if trackingVelocityDerivative:
            plt.plot(list(trackingVelocityDerivative[i][0] for i in range(len(trackingVelocityDerivative))),
                     list(trackingVelocityDerivative[i][2] for i in range(len(trackingVelocityDerivative))))
            legend.append("$D$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_{track, north} \ [m/s]$")
        plt.legend(legend)

        plt.suptitle("Tracking velocity [m/s]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/trackingVelocity.png")
    # endregion

    # region Vehicles 3D trajectories
    if vehicleGlobalPosition:
        # fig = plt.figure().add_subplot(111, projection="3d")
        fig = Axes3D(plt.figure())
        legend = list()
        maxRange = 0

        for droneId in range(N):
            fig.plot(list(vehicleGlobalPosition[droneId][i][1] for i in range(len(vehicleGlobalPosition[droneId]))),
                     list(vehicleGlobalPosition[droneId][i][2] for i in range(len(vehicleGlobalPosition[droneId]))),
                     list(vehicleGlobalPosition[droneId][i][3] for i in range(len(vehicleGlobalPosition[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        # region Set equal axes
        xLimits = fig.get_xlim3d()
        yLimits = fig.get_ylim3d()
        zLimits = fig.get_zlim3d()

        xRange = abs(xLimits[1] - xLimits[0])
        xMiddle = np.mean(xLimits)
        yRange = abs(yLimits[1] - yLimits[0])
        yMiddle = np.mean(yLimits)
        zRange = abs(zLimits[1] - zLimits[0])
        zMiddle = np.mean(zLimits)

        plotRadius = 0.5 * max([xRange, yRange, zRange])

        fig.set_xlim([xMiddle - plotRadius, xMiddle + plotRadius])
        fig.set_ylim([yMiddle - plotRadius, yMiddle + plotRadius])
        fig.set_zlim([zMiddle, zMiddle + 2 * plotRadius])
        # endregion

        plt.grid()
        plt.xlabel("$x \ [m]$")
        plt.ylabel("$y \ [m]$")
        plt.legend(legend)
        plt.title("Vehicles' 3D trajectory", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/3Dtrajectories.png")
    # endregion

    # region Vehicles 2D trajectories
    if vehicleGlobalPosition:
        plt.figure()
        plt.axis("equal")
        legend = list()

        for droneId in range(N):
            plt.plot(list(vehicleGlobalPosition[droneId][i][1] for i in range(len(vehicleGlobalPosition[droneId]))),
                     list(vehicleGlobalPosition[droneId][i][2] for i in range(len(vehicleGlobalPosition[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.grid()
        plt.xlabel("$x \ [m]$")
        plt.ylabel("$y \ [m]$")
        plt.legend(legend)
        plt.title("Vehicles' 2D trajectory", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/2Dtrajectories.png")
    # endregion

    # region Vehicle status
    if vehicleStatus:
        plt.figure()
        legend = list()
        plt.subplot(211)

        for droneId in range(N):
            plt.plot(list(vehicleStatus[droneId][i][0] for i in range(len(vehicleStatus[droneId]))),
                     list(vehicleStatus[droneId][i][1] for i in range(len(vehicleStatus[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$Navigation \ state$")
        plt.legend(legend)

        legend = list()
        plt.subplot(212)

        for droneId in range(N):
            plt.plot(list(vehicleStatus[droneId][i][0] for i in range(len(vehicleStatus[droneId]))),
                     list(vehicleStatus[droneId][i][2] for i in range(len(vehicleStatus[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$Last \ disarming \ reason$")
        plt.legend(legend)

        plt.suptitle("Drones status", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/dronesStatus.png")
    # endregion

    # region Trajectory setpoint
    if trajectorySetpoint:
        plt.figure()
        legend = list()
        plt.subplot(311)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][1] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$x \ [m]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(312)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][2] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$y \ [m]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(313)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][3] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$z \ [m]$")
        plt.legend(legend)

        plt.suptitle("Drones position setpoint", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/dronesPositionSetpoint.png")

        plt.figure()
        legend = list()
        plt.subplot(211)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][4] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$Yaw \ [rad]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(212)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][5] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$Yaw \ rate \ [rad/s]$")
        plt.legend(legend)

        plt.suptitle("Drones yaw setpoint", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/dronesYawSetpoint.png")

        plt.figure()
        legend = list()
        plt.subplot(311)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][6] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_x \ [m/s]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(312)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][7] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_y \ [m/s]$")
        plt.legend(legend)

        legend = list()
        plt.subplot(313)

        for droneId in range(N):
            plt.plot(list(trajectorySetpoint[droneId][i][0] for i in range(len(trajectorySetpoint[droneId]))),
                     list(trajectorySetpoint[droneId][i][8] for i in range(len(trajectorySetpoint[droneId]))))
            legend.append("$Drone_" + str(droneId) + "$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.ylabel("$v_z \ [m/s]$")
        plt.legend(legend)

        plt.suptitle("Drones velocity setpoint", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/dronesVelocitySetpoint.png")
    # endregion

    # region Target-anchors UWB distances
    if uwb_sensor_target:
        plt.figure()
        legend = list()

        for droneId in range(N):
            plt.plot(list(uwb_sensor_target[i][0] for i in range(len(uwb_sensor_target)) if uwb_sensor_target[i][2] == droneId),
                     list(uwb_sensor_target[i][1] for i in range(len(uwb_sensor_target)) if uwb_sensor_target[i][2] == droneId))
            legend.append("$d_{" + str(droneId) + "t}$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Target-anchors UWB distances [m]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/targetAnchorsUWBDistances.png")
    # endregion

    # region Tracking error
    if trackingError:
        plt.figure()
        legend = list()

        plt.plot(list(trackingError[i][0] for i in range(len(trackingError))),
                 list(trackingError[i][1] for i in range(len(trackingError))))

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Tracking error [m]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/trackingError.png")
    # endregion

    # region Target-anchors distances
    if targetAnchorsDistances:
        plt.figure()
        legend = list()

        for couple in range(int((len(targetAnchorsDistances[0]) - 1) / 3)):
            plt.plot(list(targetAnchorsDistances[i][0] for i in range(len(targetAnchorsDistances))),
                     list(targetAnchorsDistances[i][(couple + 1) * 3] for i in range(len(targetAnchorsDistances))))
            legend.append("$d_{" + str(int(targetAnchorsDistances[0][1 + couple * 3])) + "t}$")

        plt.xlim(timeLimits)
        plt.grid()
        plt.xlabel("$Time \ [s]$")
        plt.legend(legend)
        plt.title("Target-anchors distances [m]", size="xx-large", weight="bold")

        plt.savefig(csvFilesPath + folderName + "/targetAnchorsDistances.png")
    # endregion

    # region Animated 3d trajectories
    # fig = plt.figure()
    # ax = plt.axes(xlim=(min(list(vehicleGlobalPosition[0][i][1] for i in range(len(vehicleGlobalPosition[0])))),
    #                     max(list(vehicleGlobalPosition[0][i][1] for i in range(len(vehicleGlobalPosition[0]))))),
    #               ylim=(min(list(vehicleGlobalPosition[0][i][2] for i in range(len(vehicleGlobalPosition[0])))),
    #                     max(list(vehicleGlobalPosition[0][i][2] for i in range(len(vehicleGlobalPosition[0]))))))
    # ax.set_aspect('equal', adjustable='box')
    #
    # trajectory = ax.plot([], [], lw=2)
    #
    # xData = list()
    # yData = list()
    #
    # def init():
    #     trajectory[0].set_data(xData, yData)
    #     return trajectory
    #
    # def animate(counter):
    #     xData.append(vehicleGlobalPosition[0][counter][1])
    #     yData.append(vehicleGlobalPosition[0][counter][2])
    #     trajectory[0].set_data(xData, yData)
    #     return trajectory
    #
    # anim = animation.FuncAnimation(fig, func=animate, init_func=init,
    #                                frames=len(vehicleGlobalPosition[0]), interval=0,
    #                                repeat=False, blit=True)
    # plt.grid()
    # endregion
    # endregion

    plt.show()


if __name__ == '__main__':
    main()

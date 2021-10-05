#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
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
    for i in range(N):
        vehicleLocalPositionFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleLocalPosition.csv", "r"))
        vehicleGlobalPositionFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleGlobalPosition.csv", "r"))
        vehicleStatusFiles.append(open(csvFilesPath + folderName + "/X500_" + str(i) + "_VehicleStatus.csv", "r"))
        uwb_sensorFiles.append(open(csvFilesPath + folderName + "/uwb_sensor_" + str(i) + ".csv", "r"))
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
    for i in range(N):
        vehicleLocalPositionReaders.append(csv.reader(vehicleLocalPositionFiles[i], delimiter=","))
        vehicleGlobalPositionReaders.append(csv.reader(vehicleGlobalPositionFiles[i], delimiter=","))
        vehicleStatusReaders.append(csv.reader(vehicleStatusFiles[i], delimiter=","))
        uwb_sensorReaders.append(csv.reader(uwb_sensorFiles[i], delimiter=","))
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
    uwb_sensor_target = list()
    trackingError = list()
    targetAnchorsDistances = list()
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
    # endregion

    # region Plotters
    # region Inter-anchors distances
    plt.figure()
    legend = list()

    for couple in range(int((len(interAnchorsDistances[0]) - 1) / 3)):
        plt.plot(list(interAnchorsDistances[i][0] for i in range(len(interAnchorsDistances))),
                 list(interAnchorsDistances[i][(couple + 1) * 3] for i in range(len(interAnchorsDistances))))
        legend.append("$d_{" + str(int(interAnchorsDistances[0][1 + couple * 3])) + str(int(interAnchorsDistances[0][2 + couple * 3])) + "}$")

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.legend(legend)
    plt.title("Inter-anchors distances [m]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/interAnchorsDistances.png")
    # endregion

    # region Synchronization error UWB
    plt.figure()
    legend = list()

    plt.plot(list(synchronizationErrorUwb[i][0] for i in range(len(synchronizationErrorUwb))),
             list(synchronizationErrorUwb[i][1] for i in range(len(synchronizationErrorUwb))))

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.legend(legend)
    plt.title("Synchronization error UWB [s]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/synchronizationErrorUwb.png")
    # endregion

    # region Tracking velocity
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

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.ylabel("$v_{track, north} \ [m/s]$")
    plt.legend(legend)

    plt.suptitle("Tracking velocity [m/s]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/trackingVelocity.png")
    # endregion

    # region Vehicles trajectories
    fig = plt.figure().add_subplot(111, projection="3d")
    legend = list()

    for droneId in range(N):
        fig.plot(list(vehicleGlobalPosition[droneId][i][1] for i in range(len(vehicleGlobalPosition[droneId]))),
                 list(vehicleGlobalPosition[droneId][i][2] for i in range(len(vehicleGlobalPosition[droneId]))),
                 list(vehicleGlobalPosition[droneId][i][3] for i in range(len(vehicleGlobalPosition[droneId]))))
        legend.append("$Drone_" + str(droneId) + "$")

    plt.grid()
    plt.xlabel("$x \ [m]$")
    plt.ylabel("$y \ [m]$")
    plt.legend(legend)
    plt.title("Vehicles' trajectory", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/trajectories.png")
    # endregion

    # region Vehicles 2D trajectories
    plt.figure()
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
    plt.figure()
    legend = list()
    plt.subplot(211)

    for droneId in range(N):
        plt.plot(list(vehicleStatus[droneId][i][0] for i in range(len(vehicleStatus[droneId]))),
                 list(vehicleStatus[droneId][i][1] for i in range(len(vehicleStatus[droneId]))))
        legend.append("$Drone_" + str(droneId) + "$")

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

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.ylabel("$Last \ disarming \ reason$")
    plt.legend(legend)

    plt.suptitle("Drones status", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/dronesStatus.png")
    # endregion

    # region Target-anchors UWB distances
    plt.figure()
    legend = list()

    for droneId in range(N):
        plt.plot(list(uwb_sensor_target[i][0] for i in range(len(uwb_sensor_target)) if uwb_sensor_target[i][2] == droneId),
                 list(uwb_sensor_target[i][1] for i in range(len(uwb_sensor_target)) if uwb_sensor_target[i][2] == droneId))
        legend.append("$d_{" + str(droneId) + "t}$")

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.legend(legend)
    plt.title("Target-anchors UWB distances [m]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/targetAnchorsUWBDistances.png")
    # endregion

    # region Tracking error
    plt.figure()
    legend = list()

    plt.plot(list(trackingError[i][0] for i in range(len(trackingError))),
             list(trackingError[i][1] for i in range(len(trackingError))))

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.legend(legend)
    plt.title("Tracking error [m]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/trackingError.png")
    # endregion

    # region Target-anchors distances
    plt.figure()
    legend = list()

    for couple in range(int((len(targetAnchorsDistances[0]) - 1) / 3)):
        plt.plot(list(targetAnchorsDistances[i][0] for i in range(len(targetAnchorsDistances))),
                 list(targetAnchorsDistances[i][(couple + 1) * 3] for i in range(len(targetAnchorsDistances))))
        legend.append("$d_{" + str(int(targetAnchorsDistances[0][1 + couple * 3])) + "t}$")

    plt.grid()
    plt.xlabel("$Time \ [s]$")
    plt.legend(legend)
    plt.title("Target-anchors distances [m]", size="xx-large", weight="bold")

    plt.savefig(csvFilesPath + folderName + "/targetAnchorsDistances.png")
    # endregion
    # endregion

    plt.show()


if __name__ == '__main__':
    main()

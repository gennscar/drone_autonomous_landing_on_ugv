import math
from px4_msgs.msg import VehicleGlobalPosition
from ros2_px4_interfaces.msg import UnitVector


def computeUnitVector(destinationDroneId, destinationPosition, startDroneId, startPosition, timestamp):
    """
    Given the positions of two drones, it returns the unit vector
    denoting the direction of the line connecting them

    Args:
        destinationDroneId (int): identifier of the destination drone
        destinationPosition (VehicleGlobalPosition): position of the destination drone
        startDroneId (int): identifier of the start drone
        startPosition (VehicleGlobalPosition): position of the start drone
        timestamp (time): time of the computations

    Returns:
        UnitVector: the normalized vector connecting the drones
    """

    deltaLat = math.log(math.tan(math.radians(destinationPosition.lat) / 2 + math.pi / 4) / math.tan(math.radians(startPosition.lat) / 2 + math.pi / 4))
    deltaLong = math.radians(destinationPosition.lon - startPosition.lon)
    angle = math.atan2(deltaLong, deltaLat)
    result = UnitVector()
    result.header.stamp = timestamp
    result.destination_drone_id = destinationDroneId
    result.start_drone_id = startDroneId
    result.east = math.sin(angle)
    result.north = math.cos(angle)
    return result

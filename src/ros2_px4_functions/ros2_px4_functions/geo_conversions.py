from math import cos, sin, sqrt
import numpy as np

# WGS84 parameters
SEMI_MAJOR_AXIS = 6378137.0  # m
EARTH_ECCENTRICITY_SQUARED = 6.69437999014e-3


def WGS84_to_ECEF(coordinates):
    """Converts WGS84 coordinates into  ECEF coordinates

    Args:
        coordinates (np.array(3)): latitude, longitude in radians and altitude
        in meters

    Returns:
        np.array(3): ECEF coordinates
    """

    res = np.empty(3)
    lat_rad, lon_rad, alt_m = coordinates

    # Prime vertical radius of curvature
    N = SEMI_MAJOR_AXIS / \
        sqrt(1 - EARTH_ECCENTRICITY_SQUARED*(sin(lat_rad)**2))

    # Calculate ECEF coordinates
    res[0] = (N + alt_m)*cos(lat_rad)*cos(lon_rad)
    res[1] = (N + alt_m)*cos(lat_rad)*sin(lon_rad)
    res[2] = ((1 - EARTH_ECCENTRICITY_SQUARED)*N + alt_m)*sin(lat_rad)

    return res


def WGS84_to_ENU(coordinates, reference):
    """Converts WGS coordinates into  ENU coordinates

    Args:
        coordinates (np.array(3)): latitude, longitude in radians and altitude
        in meters
        reference (np.array(3)): oringin of the ENU frame

    Returns:
        np.array(3): ENU coordinates
    """

    # Converting all to ECEF first
    ECEF_position = WGS84_to_ECEF(coordinates)
    ECEF_reference = WGS84_to_ECEF(reference)

    # Building the conversion matrix
    lat_rad, lon_rad, alt_m = reference
    conversion_matrix = np.array([
        [-sin(lon_rad), cos(lon_rad), 0.],
        [-sin(lat_rad)*cos(lon_rad), -sin(lon_rad)*cos(lat_rad), cos(lat_rad)],
        [cos(lat_rad)*cos(lon_rad), cos(lat_rad)*sin(lon_rad), sin(lat_rad)]
    ])

    # Converting to relative ENU frame
    res = np.matmul(
        conversion_matrix,
        np.transpose(ECEF_position - ECEF_reference)
    )

    return res

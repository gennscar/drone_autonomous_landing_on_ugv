import math
import numpy as np

EARTH_RADIUS = 6378137
EARTH_ECCENTRICITY = 0.0818191908425


def NED_to_ECEF(lat_rad, lon_rad, alt_m):
    """Converts NED coordinates into  ECEF coordinates

    Args:
        lat_rad (double): latitude in radians
        lon_rad (double): longitude in radians
        alt_m (double): altitude in meters

    Returns:
        np.array(3): ECEF coordinates
    """

    res = np.empty(3)

    # Calculate ECEF coordinates
    res[0] = (EARTH_RADIUS + alt_m) * math.cos(lat_rad)*math.cos(lon_rad)
    res[1] = (EARTH_RADIUS + alt_m) * math.cos(lat_rad)*math.sin(lon_rad)
    res[2] = ((1-EARTH_ECCENTRICITY ** 2) *
              EARTH_RADIUS + alt_m)*math.sin(lat_rad)

    return res

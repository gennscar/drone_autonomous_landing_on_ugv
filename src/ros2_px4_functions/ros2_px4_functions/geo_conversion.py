import numpy as np

# Radius of the Earth (in meters)
RADIUS_OF_EARTH = np.float64(6378137.0)


def lla_to_ecef(ref_lat, ref_lon, ref_alt, lat, lon, alt):
    """[summary]

    Args:
        ref_lat ([type]): [description]
        ref_lon ([type]): [description]
        ref_alt ([type]): [description]
        lat ([type]): [description]
        lon ([type]): [description]
        alt ([type]): [description]

    Returns:
        [type]: [description]
    """

    ref_lat_rad = np.radians(ref_lat)
    ref_lon_rad = np.radians(ref_lon)
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    ref_cos_lat = np.cos(ref_lat_rad)
    ref_sin_lat = np.sin(ref_lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lat = np.sin(lon_rad)

    cos_d_long = np.cos(lon_rad - ref_lon_rad)

    arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat *
                  cos_lat * cos_d_long, -1.0, 0.0)
    c = np.arccos(arg)

    k = 1.0
    if (np.abs(c) > 0):
        k = c/np.sin(c)

    x = k * (ref_cos_lat * sin_lat - ref_sin_lat *
             cos_lat * cos_d_long) * RADIUS_OF_EARTH
    y = k * cos_lat * np.sin(lon_rad - ref_lon_rad) * RADIUS_OF_EARTH
    z = -(alt - ref_alt)

    return x, y, z

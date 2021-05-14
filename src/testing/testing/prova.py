import numpy as np


def extract_anchor_data(anchors):
    """Extract all the data from a dict of anchors

    Args:
        anchors (dict): Dict with keys the anchor Id and value the UwbSensor
          message sent by that anchor, in this way only the last message is
          saved

    Returns:
        anchor_pos (np.array): Matrix containing for each row the 3 coordinates
          of one anchor

        ranges (np.array): Array containing all the pseudoranges

        N (int): Number of anchors
    """

    N = len(anchors)

    # Extract anchor positions and ranges
    i = 0
    anchor_pos = np.empty((N, 3))
    ranges = np.empty(N)

    for _, data in anchors.items():
        anchor_pos[i, :] = np.array(
            [data.anchor_pos.x, data.anchor_pos.y, data.anchor_pos.z])
        ranges[i] = data.range
        i = i+1

    return anchor_pos, ranges, N


def ls_trilateration(anchors):
    """
    Solve the Least-Square problem to estimate the position of the sensor
      from the pseudoranges of the anchors and their position

    Args:
        anchors (dict): Dict with keys the anchor Id and value the UwbSensor

    Returns:
        np.array: The estimated position
    """
    # Extract anchor positions and ranges
    anchor_pos, ranges, N = extract_anchor_data(anchors)

    # Calculate b, A and its pseudoinverse
    b = ranges**2 - anchor_pos[:, 0]**2 - \
        anchor_pos[:, 1]**2 - anchor_pos[:, 2]**2
    A = np.concatenate((np.ones((N, 1)), -2*anchor_pos), axis=1)
    pinvA = np.linalg.pinv(A)

    # Resolve LS: y = pinv(A)*b
    y = pinvA.dot(b)
    return y[1:]


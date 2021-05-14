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


def gauss_newton_trilateration(old_pos, anchors):
    """
    Perform a Gauss-Newton iteration to estimate the position of the sensor
      from the pseudoranges of the anchors and their position

    Args:
        old_pos (np.array): The estimated position of the last iteration
        anchors (dict): Dict with keys the anchor Id and value the UwbSensor

    Returns:
        np.array: The estimated position
    """

    # Extract anchor positions and ranges
    anchor_pos, ranges, _ = extract_anchor_data(anchors)

    # Calculate Jacobian matrix
    den = ((old_pos[0] - anchor_pos[:, 0])**2 + (old_pos[1] -
                                                 anchor_pos[:, 1])**2 + (old_pos[2] - anchor_pos[:, 2])**2)**0.5
    num = np.transpose(old_pos) - anchor_pos
    J = np.divide(num, np.expand_dims(den, 1))

    # Calculate the pseudoinverse and the residues
    pinvJ = np.linalg.pinv(-J)
    residue = ranges - ((old_pos[0]-anchor_pos[:, 0])**2 +
                        (old_pos[1]-anchor_pos[:, 1])**2 +
                        (old_pos[2]-anchor_pos[:, 2])**2)**0.5

    # LS solution
    new_pos = old_pos - (pinvJ).dot(residue)

    return new_pos


def main():
    print('ciao')
    anchors = {}

    anchors['0'].anchor_pos = [3.5, 3, 0]
    anchors['0'].ranges = 3.20

    anchors['1'].anchor_pos = [4.5, 3, 0]
    anchors['1'].ranges = 4.03

    anchors['2'].anchor_pos = [4.5, 1, 0]
    anchors['2'].ranges = 3.5

    anchors['3'].anchor_pos = [3.5, 1, 0]
    anchors['3'].ranges = 2.5

    pos = ls_trilateration(anchors)
    print(pos)


if __name__ == "main":
    main()

import numpy as np


def ls_trilateration(anchor_pos, ranges, N):
    """
    Solve the Least-Square problem to estimate the position of the sensor
      from the pseudoranges of the anchors and their position

    Args:
        anchor_pos (np.array): Matrix containing for each row the 3 coordinates 
        of one anchor

        ranges (np.array): Array containing all the pseudoranges

        N (int): Number of anchors

    Returns:
        np.array: The estimated position
    """

    # Calculate b, A and its pseudoinverse
    b = ranges**2 - anchor_pos[:, 0]**2 - \
        anchor_pos[:, 1]**2 - anchor_pos[:, 2]**2
    A = np.concatenate((np.ones((N, 1)), -2*anchor_pos), axis=1)
    pinvA = np.linalg.pinv(A)

    # Resolve LS: y = pinv(A)*b
    y = pinvA.dot(b)
    return y[1:]


def gauss_newton_trilateration(old_pos, anchor_pos, ranges):
    """
    Perform a Gauss-Newton iteration to estimate the position of the sensor
      from the pseudoranges of the anchors and their position

    Args:
        old_pos (np.array): The estimated position of the last iteration

        anchor_pos (np.array): Matrix containing for each row the 3 coordinates 
        of one anchor

        ranges (np.array): Array containing all the pseudoranges

        N (int): Number of anchors

    Returns:
        np.array: The estimated position
    """

    # Calculate Jacobian matrix
    den = ((old_pos[0] - anchor_pos[:, 0])**2 +
           (old_pos[1] - anchor_pos[:, 1])**2 +
           (old_pos[2] - anchor_pos[:, 2])**2)**0.5
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

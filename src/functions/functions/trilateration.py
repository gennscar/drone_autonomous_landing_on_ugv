import numpy as np


def extract_anchor_data(anchors):
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
    # Extract anchor positions and ranges
    anchor_pos, ranges, N = extract_anchor_data(anchors)

    # Calculate b, A and its pseudoinverse
    b = ranges**2 - anchor_pos[:, 0]**2 - \
        anchor_pos[:, 1]**2 - anchor_pos[:, 2]**2
    A = np.concatenate((np.ones((N, 1)), -2*anchor_pos), axis=1)
    pinvA = np.linalg.pinv(A, rcond=1e-15, hermitian=False)

    # Resolve LS: y = pinv(A)*b
    y = pinvA.dot(b)
    return y[1:]


def gauss_newton_trilateration(anchors):
    # todo
    pass

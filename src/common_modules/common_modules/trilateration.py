import numpy as np


def ls_trilateration(anchors):
    N = len(anchors)

    # Extract anchor positions and ranges
    i = 0
    p = np.empty((N, 3))
    d = np.empty(N)

    for _, data in anchors.items():
        p[i, :] = np.array(
            [data.anchor_pos.x, data.anchor_pos.y, data.anchor_pos.z])
        d[i] = data.range
        i = i+1

    # Calculate b, A and its pseudoinverse
    b = d**2 - p[:, 0]**2 - p[:, 1]**2 - p[:, 2]**2
    A = np.concatenate((np.ones((N, 1)), -2*p), axis=1)
    pinvA = np.linalg.pinv(A, rcond=1e-15, hermitian=False)

    # Resolve LS
    y = pinvA.dot(b)
    return y

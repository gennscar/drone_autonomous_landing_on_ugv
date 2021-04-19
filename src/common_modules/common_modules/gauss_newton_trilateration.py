import numpy as np


p0 = np.array([0.0,0.0,0.0])
p1 = np.array([1.9,0.9,0.0])
p2 = np.array([0.0,0.9,0.0])
p3 = np.array([1.9,0.0,0.2])


def trilateration(old_pos, d0, d1, d2, d3):

    den_1 = ((old_pos[0]-p0[0])**2 + (old_pos[1]-p0[1])**2 + (old_pos[2]-p0[2])**2)**0.5
    den_2 = ((old_pos[0]-p1[0])**2 + (old_pos[1]-p1[1])**2 + (old_pos[2]-p1[2])**2)**0.5
    den_3 = ((old_pos[0]-p2[0])**2 + (old_pos[1]-p2[1])**2 + (old_pos[2]-p2[2])**2)**0.5
    den_4 = ((old_pos[0]-p3[0])**2 + (old_pos[1]-p3[1])**2 + (old_pos[2]-p3[2])**2)**0.5

    num_11 = old_pos[0] - p0[0]
    num_12 = old_pos[1] - p0[1]
    num_13 = old_pos[2] - p0[2]

    num_21 = old_pos[0] - p1[0]
    num_22 = old_pos[1] - p1[1]
    num_23 = old_pos[2] - p1[2]

    num_31 = old_pos[0] - p2[0]
    num_32 = old_pos[1] - p2[1]
    num_33 = old_pos[2] - p2[2]

    num_41 = old_pos[0] - p3[0]
    num_42 = old_pos[1] - p3[1]
    num_43 = old_pos[2] - p3[2]

    J_row_1 = np.array([num_11/den_1, num_12/den_1, num_13/den_1])
    J_row_2 = np.array([num_21/den_2, num_22/den_2, num_23/den_2])
    J_row_3 = np.array([num_31/den_3, num_32/den_3, num_33/den_3])
    J_row_4 = np.array([num_41/den_4, num_42/den_4, num_43/den_4])

    J = - np.array([J_row_1, J_row_2, J_row_3, J_row_4])
    pinvJ = (np.linalg.pinv(J, rcond=1e-15, hermitian=False)).transpose()

    residue_1 = d0 - ((old_pos[0]-p0[0])**2 + (old_pos[1]-p0[1])**2 + (old_pos[2]-p0[2])**2)**0.5
    residue_2 = d1 - ((old_pos[0]-p1[0])**2 + (old_pos[1]-p1[1])**2 + (old_pos[2]-p1[2])**2)**0.5
    residue_3 = d2 - ((old_pos[0]-p2[0])**2 + (old_pos[1]-p2[1])**2 + (old_pos[2]-p2[2])**2)**0.5
    residue_4 = d3 - ((old_pos[0]-p3[0])**2 + (old_pos[1]-p3[1])**2 + (old_pos[2]-p3[2])**2)**0.5
    residue = np.array([residue_1, residue_2, residue_3, residue_4])

    new_pos = old_pos - (pinvJ.transpose()).dot(residue)

    return new_pos
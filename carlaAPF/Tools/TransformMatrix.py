import numpy as np

def rotate2D(matrix, angle, degrees = True, north_zero = True, ccw_positive = True):

    if degrees == True: angle = np.radians(angle)


    if ccw_positive == False:
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],[np.sin(angle),np.cos(angle), 0], [0, 0, 1]])
    else:
        rotation = np.array([[np.cos(angle), np.sin(angle), 0],[-np.sin(angle),np.cos(angle), 0], [0, 0, 1]])

    matrix = np.array([matrix])  #needs to be 2D matrix for transpose
    rotated_matrix = rotation.dot(matrix.T)
    flattened_matrix = np.array([rotated_matrix[0][0], rotated_matrix[1][0], rotated_matrix[2][0]]) # make 1D again

    return flattened_matrix



def stretch2D(matrix, strech_factor):

    stretch_matrix = np.array([[strech_factor,0],[0, strech_factor]])
    matrix = np.array([matrix])  # needs to be 2D matrix for transpose

    streched_matrix = stretch_matrix * matrix.T
    flattened_matrix = np.array([streched_matrix[0][0], streched_matrix[1][0], streched_matrix[2][0]])  # make 1D again
    return flattened_matrix
